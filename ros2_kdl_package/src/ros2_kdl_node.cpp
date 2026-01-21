#include <stdio.h>
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/wait_for_message.hpp"

#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

// Header per le trasformate TF2
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "kdl_robot.h"
#include "kdl_control.h"
#include "kdl_planner.h"
#include "kdl_parser/kdl_parser.hpp"
#include "ros2_kdl_package/action/execute_trajectory.hpp"

using namespace KDL;
using FloatArray = std_msgs::msg::Float64MultiArray;
using namespace std::chrono_literals;

class Iiwa_pub_sub : public rclcpp::Node
{
public:
    using ExecuteTrajectory = ros2_kdl_package::action::ExecuteTrajectory;
    using GoalHandleExecuteTrajectory = rclcpp_action::ServerGoalHandle<ExecuteTrajectory>;

    Iiwa_pub_sub() : Node("ros2_kdl_node")
    {
        using namespace std::placeholders;

        // --- PARAMETRI ---
        this->declare_parameter<double>("Kp", 1.0);
        this->declare_parameter<std::string>("cmd_interface", "velocity");
        this->declare_parameter<std::string>("ctrl", "vision");
        
        this->get_parameter("Kp", Kp_);
        this->get_parameter("cmd_interface", cmd_interface_);
        this->get_parameter("ctrl", ctrl_);

        // --- INIZIALIZZAZIONE TF2 (Per ArUco Multi-Tag) ---
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // --- INIZIALIZZAZIONE KDL ---
        setup_kdl();

        // --- TOPIC E ACTION SERVER ---
        this->action_server_ = rclcpp_action::create_server<ExecuteTrajectory>(
            this, "ExecuteTrajectory",
            std::bind(&Iiwa_pub_sub::handle_goal, this, _1, _2),
            std::bind(&Iiwa_pub_sub::handle_cancel, this, _1),
            std::bind(&Iiwa_pub_sub::handle_accepted, this, _1));

        jointSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10, std::bind(&Iiwa_pub_sub::joint_state_subscriber, this, _1));

        std::string cmd_topic = (cmd_interface_ == "position") ? "iiwa_arm_controller/commands" : "velocity_controller/commands";
        cmdPublisher_ = this->create_publisher<FloatArray>(cmd_topic, 10);

        RCLCPP_INFO(this->get_logger(), "Node Ready con supporto Multi-Tag (TF2)");
    }

private:
    // --- KDL SETUP (Invariato) ---
    void setup_kdl() {
        auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this, "robot_state_publisher");
        while (!parameters_client->wait_for_service(1s)) {
            if (!rclcpp::ok()) return;
            RCLCPP_INFO(this->get_logger(), "Waiting for robot_description...");
        }
        auto parameter = parameters_client->get_parameters({"robot_description"});
        KDL::Tree robot_tree;
        kdl_parser::treeFromString(parameter[0].value_to_string(), robot_tree);
        robot_ = std::make_shared<KDLRobot>(robot_tree);
        joint_positions_.resize(robot_->getNrJnts());
        joint_velocities_.resize(robot_->getNrJnts());
        joint_velocities_cmd_.resize(robot_->getNrJnts());
        controller_ = KDLController(*robot_);
        robot_->addEE(KDL::Frame::Identity());
    }

    // --- ACTION CALLBACKS ---
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ExecuteTrajectory::Goal> goal) 
    { (void)uuid; return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleExecuteTrajectory> goal_handle) 
    { (void)goal_handle; return rclcpp_action::CancelResponse::ACCEPT; }

    void handle_accepted(const std::shared_ptr<GoalHandleExecuteTrajectory> goal_handle) 
    { std::thread{std::bind(&Iiwa_pub_sub::execute, this, std::placeholders::_1), goal_handle}.detach(); }

    // --- LOGICA DI ESECUZIONE (CORE) ---
    void execute(const std::shared_ptr<GoalHandleExecuteTrajectory> goal_handle)
    {
        auto feedback = std::make_shared<ExecuteTrajectory::Feedback>();
        auto result = std::make_shared<ExecuteTrajectory::Result>();
        rclcpp::Rate rate(50); 

        while (rclcpp::ok() && ctrl_ == "vision") {
            if (goal_handle->is_canceling()) {
                stop_robot();
                goal_handle->canceled(result);
                return;
            }

            // --- RICERCA MARKER TRAMITE TF2 ---
            geometry_msgs::msg::TransformStamped t;
            bool found = false;
            std::string target_frame = "";

            // Prova a cercare prima il Tag 1, poi il Tag 2
            for (std::string id : {"marker_id_1", "marker_id_2"}) {
                try {
                    // Cerca la trasformata più recente tra camera e marker
                    t = tf_buffer_->lookupTransform("iiwa2_camera_link", id, tf2::TimePointZero);
                    target_frame = id;
                    found = true;
                    break; // Se ne trova uno, interrompe la ricerca
                } catch (const tf2::TransformException & ex) {
                    continue; // Se non lo trova, passa al prossimo ID
                }
            }

            if (found) {
                // Aggiorna cPo_ con i dati della TF
                cPo_ << t.transform.translation.x, t.transform.translation.y, t.transform.translation.z;
                
                Eigen::Vector3d sd(0, 0, 1);
                joint_velocities_cmd_ = controller_.vision_ctrl(Kp_, cPo_, sd);
                
                // Feedback
                Eigen::Vector3d s = cPo_ / cPo_.norm();
                Eigen::Vector3d dir_error = s - sd;
                feedback->position_error = {dir_error(0), dir_error(1), dir_error(2)};
                
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Inseguendo: %s", target_frame.c_str());
            } else {
                // Se non vede nulla, ferma il robot
                joint_velocities_cmd_.data.setZero();
                feedback->position_error = {999.0, 999.0, 999.0};
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "NESSUN TAG RILEVATO");
            }

            goal_handle->publish_feedback(feedback);

            // Pubblicazione comandi
            robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));
            FloatArray cmd_msg;
            cmd_msg.data.assign(joint_velocities_cmd_.data.data(), joint_velocities_cmd_.data.data() + joint_velocities_cmd_.rows());
            cmdPublisher_->publish(cmd_msg);

            rate.sleep();
        }
        result->success = true;
        goal_handle->succeed(result);
        stop_robot();
    }

    void stop_robot() {
        FloatArray stop_msg;
        stop_msg.data.resize(7, 0.0);
        cmdPublisher_->publish(stop_msg);
    }

    void joint_state_subscriber(const sensor_msgs::msg::JointState& msg) {
        for (size_t i = 0; i < msg.position.size(); ++i) {
            joint_positions_.data[i] = msg.position[i];
            joint_velocities_.data[i] = msg.velocity[i];
        }
    }

    // --- MEMBRI ---
    std::shared_ptr<KDLRobot> robot_;
    KDLController controller_;
    double Kp_;
    std::string cmd_interface_, ctrl_;
    KDL::JntArray joint_positions_, joint_velocities_, joint_velocities_cmd_;
    Eigen::Vector3d cPo_;

    // TF2 Members
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp_action::Server<ExecuteTrajectory>::SharedPtr action_server_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscriber_;
    rclcpp::Publisher<FloatArray>::SharedPtr cmdPublisher_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Iiwa_pub_sub>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}