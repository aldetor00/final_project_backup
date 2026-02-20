#include <stdio.h>
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <vector>
#include <cmath>
#include <thread>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "aruco_msgs/msg/marker_array.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"

#include "kdl_robot.h"
#include "kdl_control.h"
#include "kdl_planner.h"
#include "kdl_parser/kdl_parser.hpp"
#include "ros2_kdl_package/action/execute_trajectory.hpp"

using namespace KDL;
using namespace std::chrono_literals;

class Iiwa_pub_sub : public rclcpp::Node {
public:
    using ExecuteTrajectory = ros2_kdl_package::action::ExecuteTrajectory;
    using GoalHandleExecuteTrajectory = rclcpp_action::ServerGoalHandle<ExecuteTrajectory>;
    using SwitchController = controller_manager_msgs::srv::SwitchController;

    Iiwa_pub_sub() : Node("ros2_kdl_node"), state_received_(false), marker_detected_(false) {
        this->declare_parameter<double>("traj_duration", 5.0);
        this->get_parameter("traj_duration", traj_duration_);

        if (!setup_kdl()) {
            RCLCPP_ERROR(this->get_logger(), "Setup KDL fallito!");
            return;
        }

        std::string ns = this->get_namespace();
        if (ns.size() > 0 && ns[0] == '/') ns = ns.substr(1);
        
        for(int i=1; i<=7; i++) {
            ordered_joint_names_.push_back(ns + "_joint_a" + std::to_string(i));
        }

        this->action_server_ = rclcpp_action::create_server<ExecuteTrajectory>(
            this, "ExecuteTrajectory",
            std::bind(&Iiwa_pub_sub::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&Iiwa_pub_sub::handle_cancel, this, std::placeholders::_1),
            std::bind(&Iiwa_pub_sub::handle_accepted, this, std::placeholders::_1));

        aruco_sub_ = this->create_subscription<aruco_msgs::msg::MarkerArray>(
            "/aruco_marker_publisher/markers", 10,
            std::bind(&Iiwa_pub_sub::aruco_callback, this, std::placeholders::_1));

        jointSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10, std::bind(&Iiwa_pub_sub::joint_state_subscriber, this, std::placeholders::_1));

        std::string traj_topic = "/" + ns + "/iiwa_arm_controller/joint_trajectory";
        std::string vel_topic = "/" + ns + "/velocity_controller/commands";
        
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
        qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        qos.durability(rclcpp::DurabilityPolicy::Volatile);
        
        cmdPublisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(traj_topic, qos);
        velocityPublisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(vel_topic, qos);
        
        if (ns == "iiwa2") {
            switch_controller_client_ = this->create_client<SwitchController>(
                "/" + ns + "/controller_manager/switch_controller");
        }
            
        RCLCPP_INFO(this->get_logger(), "🚀 ULTRA CLOSE MODE - Target 0.5cm (quasi contatto!)");
        RCLCPP_INFO(this->get_logger(), "   Gain adattivo: lento e preciso per contatto delicato");
    }

private:
    bool switch_to_velocity_controller() {
        if (!switch_controller_client_) return true;
        auto request = std::make_shared<SwitchController::Request>();
        request->activate_controllers = {"velocity_controller"};
        request->deactivate_controllers = {"iiwa_arm_controller"};
        request->strictness = SwitchController::Request::BEST_EFFORT;
        request->activate_asap = true;
        request->timeout = rclcpp::Duration::from_seconds(0.0);
        
        auto future = switch_controller_client_->async_send_request(request);
        if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
            if (future.get()->ok) {
                RCLCPP_INFO(this->get_logger(), "✅ Switched to velocity_controller");
                return true;
            }
        }
        return false;
    }
    
    bool switch_to_arm_controller() {
        if (!switch_controller_client_) return true;
        auto request = std::make_shared<SwitchController::Request>();
        request->activate_controllers = {"iiwa_arm_controller"};
        request->deactivate_controllers = {"velocity_controller"};
        request->strictness = SwitchController::Request::BEST_EFFORT;
        request->activate_asap = true;
        request->timeout = rclcpp::Duration::from_seconds(0.0);
        
        auto future = switch_controller_client_->async_send_request(request);
        if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
            if (future.get()->ok) {
                RCLCPP_INFO(this->get_logger(), "✅ Switched to arm_controller");
                return true;
            }
        }
        return false;
    }

    void aruco_callback(const aruco_msgs::msg::MarkerArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!msg->markers.empty()) {
            cPo_ << msg->markers[0].pose.pose.position.x, 
                    msg->markers[0].pose.pose.position.y, 
                    msg->markers[0].pose.pose.position.z;
            marker_detected_ = true;
        } else {
            marker_detected_ = false;
        }
    }

    void execute(const std::shared_ptr<GoalHandleExecuteTrajectory> goal_handle) {
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<ExecuteTrajectory::Result>();
        std::string ns = this->get_namespace();
        if (ns[0] == '/') ns = ns.substr(1);

        if (goal->ctrl == "vision" && ns == "iiwa2") {
            RCLCPP_INFO(this->get_logger(), "🎯 ULTRA CLOSE MODE - Target 0.5cm (QUASI CONTATTO)");
            
            if (!switch_to_velocity_controller()) {
                result->success = false;
                goal_handle->abort(result);
                return;
            }
            
            std::this_thread::sleep_for(500ms);
            
            rclcpp::Rate rate(50);
            int iterations = 0;
            int lost_frames = 0;
            const int MAX_LOST_FRAMES = 120; // 3 secondi (più tolleranza per avvicinamento lento)
            Eigen::Vector3d last_valid_cPo;
            bool marker_was_detected = false;
            
            // Aspetta prima rilevazione
            RCLCPP_INFO(this->get_logger(), "⏳ Attendo rilevazione marker...");
            while (!marker_detected_ && iterations < 200) {
                iterations++;
                rate.sleep();
            }
            
            if (!marker_detected_) {
                RCLCPP_ERROR(this->get_logger(), "❌ Marker non rilevato!");
                switch_to_arm_controller();
                result->success = false;
                goal_handle->abort(result);
                return;
            }
            
            last_valid_cPo = cPo_;
            marker_was_detected = true;
            iterations = 0;
            
            RCLCPP_INFO(this->get_logger(), "✅ Marker rilevato! Inizio avvicinamento ultra-preciso...");
            
            while (rclcpp::ok() && iterations < 10000) { // Timeout aumentato per movimento lento
                iterations++;
                
                if (goal_handle->is_canceling()) {
                    std_msgs::msg::Float64MultiArray zero_vel;
                    zero_vel.data.assign(7, 0.0);
                    velocityPublisher_->publish(zero_vel);
                    switch_to_arm_controller();
                    goal_handle->canceled(result);
                    return;
                }
                
                // Aggiorna ultima posizione valida
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    if (marker_detected_) {
                        last_valid_cPo = cPo_;
                        lost_frames = 0;
                    } else if (marker_was_detected) {
                        lost_frames++;
                        if (lost_frames < MAX_LOST_FRAMES) {
                            if (lost_frames % 25 == 0) {
                                RCLCPP_WARN(this->get_logger(),
                                    "⚠️ Marker perso, uso memoria (frame %d/%d)", 
                                    lost_frames, MAX_LOST_FRAMES);
                            }
                        } else {
                            RCLCPP_ERROR(this->get_logger(), "❌ Marker perso troppo a lungo!");
                            std_msgs::msg::Float64MultiArray zero_vel;
                            zero_vel.data.assign(7, 0.0);
                            velocityPublisher_->publish(zero_vel);
                            break;
                        }
                    }
                }
                
                // Usa posizione corrente o memorizzata
                Eigen::Vector3d target_pos = marker_detected_ ? cPo_ : last_valid_cPo;
                double distance = target_pos.norm();
                
                // GAIN MULTI-LIVELLO per avvicinamento progressivo ultra-preciso
                double gain;
                if (distance > 0.15) {
                    gain = -10.0;  // Veloce quando lontano
                } else if (distance > 0.08) {
                    gain = -3.0;   // Medio quando a media distanza
                } else if (distance > 0.03) {
                    gain = -1.0;   // Lento quando vicino
                } else {
                    gain = -0.5;   // MOLTO lento quando vicinissimo (sotto 3cm)
                }
                
                Eigen::Vector3d sd(0, 0, 0.01);
                KDL::JntArray qd = controller_.vision_ctrl(gain, target_pos, sd);
                
                std_msgs::msg::Float64MultiArray vel_msg;
                vel_msg.data.assign(qd.data.data(), qd.data.data() + qd.rows());
                velocityPublisher_->publish(vel_msg);

                // Logging dettagliato
                if (iterations % 50 == 0) {
                    std::string phase;
                    if (distance > 0.15) phase = "VELOCE";
                    else if (distance > 0.08) phase = "MEDIO";
                    else if (distance > 0.03) phase = "LENTO";
                    else phase = "ULTRA-LENTO";
                    
                    RCLCPP_INFO(this->get_logger(), 
                        "[%s] dist=%0.4fm (%.1fmm), pos=[%0.3f,%0.3f,%0.3f], marker=%s, gain=%.1f", 
                        phase.c_str(), distance, distance*1000.0,
                        target_pos(0), target_pos(1), target_pos(2),
                        marker_detected_ ? "OK" : "MEMORIA", gain);
                }

                // TARGET A 0.5CM (5mm) - QUASI CONTATTO
                if (distance < 0.05) {
                    RCLCPP_INFO(this->get_logger(), 
                        "✅✅✅ TARGET RAGGIUNTO! Distanza: %0.4fm (%.1fmm) - QUASI CONTATTO!", 
                        distance, distance*1000.0);
                    std_msgs::msg::Float64MultiArray zero_vel;
                    zero_vel.data.assign(7, 0.0);
                    velocityPublisher_->publish(zero_vel);
                    
                    // Log finale dettagliato
                    RCLCPP_INFO(this->get_logger(), 
                        "🎯 Posizione finale: [%0.4f, %0.4f, %0.4f]", 
                        target_pos(0), target_pos(1), target_pos(2));
                    RCLCPP_INFO(this->get_logger(), 
                        "📊 Marker rilevato: %s, Lost frames totali durante approccio: %d", 
                        marker_detected_ ? "SÌ" : "NO (usando memoria)", lost_frames);
                    break;
                }
                
                rate.sleep();
            }
            
            if (iterations >= 10000) {
                double final_distance = (marker_detected_ ? cPo_ : last_valid_cPo).norm();
                RCLCPP_WARN(this->get_logger(), 
                    "⏱️ Timeout raggiunto. Distanza finale: %0.4fm (%.1fmm)", 
                    final_distance, final_distance*1000.0);
            }
            
            switch_to_arm_controller();
            std::this_thread::sleep_for(500ms);
        } 
        else {
            if (ns == "iiwa2") {
                switch_to_arm_controller();
                std::this_thread::sleep_for(500ms);
            }
            publish_trajectory(goal->joints_target, traj_duration_);
            std::this_thread::sleep_for(std::chrono::milliseconds((int)(traj_duration_ * 1000 + 500)));
        }

        result->success = true;
        goal_handle->succeed(result);
    }

    bool setup_kdl() {
        auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this, "robot_state_publisher");
        while (!parameters_client->wait_for_service(1s)) {
            if (!rclcpp::ok()) return false;
        }
        auto parameter = parameters_client->get_parameters({"robot_description"});
        KDL::Tree robot_tree;
        kdl_parser::treeFromString(parameter[0].value_to_string(), robot_tree);
        robot_ = std::make_shared<KDLRobot>(robot_tree);
        controller_ = KDLController(*robot_);
        joint_positions_.resize(robot_->getNrJnts());
        return true;
    }

    void joint_state_subscriber(const sensor_msgs::msg::JointState& msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        for (size_t i = 0; i < ordered_joint_names_.size(); ++i) {
            for(size_t j=0; j<msg.name.size(); ++j) {
                if (msg.name[j] == ordered_joint_names_[i]) {
                    joint_positions_(i) = msg.position[j];
                    break;
                }
            }
        }
        robot_->update(toStdVector(joint_positions_.data), std::vector<double>(7,0.0));
    }

    void publish_trajectory(const std::vector<double>& joint_targets, double duration_sec) {
        std::string ns = this->get_namespace();
        if (ns.size() > 0 && ns[0] == '/') ns = ns.substr(1);
        
        trajectory_msgs::msg::JointTrajectory msg;
        msg.joint_names = ordered_joint_names_;
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = joint_targets;
        point.time_from_start = rclcpp::Duration::from_seconds(duration_sec);
        msg.points.push_back(point);
        cmdPublisher_->publish(msg);
    }

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID&, std::shared_ptr<const ExecuteTrajectory::Goal>) { 
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; 
    }
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleExecuteTrajectory>) { 
        return rclcpp_action::CancelResponse::ACCEPT; 
    }
    void handle_accepted(const std::shared_ptr<GoalHandleExecuteTrajectory> goal_handle) { 
        std::thread{std::bind(&Iiwa_pub_sub::execute, this, std::placeholders::_1), goal_handle}.detach(); 
    }

    KDLController controller_;
    std::shared_ptr<KDLRobot> robot_;
    KDL::JntArray joint_positions_;
    std::vector<std::string> ordered_joint_names_;
    double traj_duration_;
    bool marker_detected_, state_received_;
    Eigen::Vector3d cPo_;
    std::mutex mutex_;
    
    rclcpp::Subscription<aruco_msgs::msg::MarkerArray>::SharedPtr aruco_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscriber_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr cmdPublisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocityPublisher_;
    rclcpp_action::Server<ExecuteTrajectory>::SharedPtr action_server_;
    rclcpp::Client<SwitchController>::SharedPtr switch_controller_client_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Iiwa_pub_sub>());
    rclcpp::shutdown();
    return 0;
}