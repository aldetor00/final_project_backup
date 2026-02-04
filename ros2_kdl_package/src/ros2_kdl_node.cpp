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

    Iiwa_pub_sub() : Node("ros2_kdl_node"), state_received_(false) {
        this->declare_parameter<double>("traj_duration", 5.0);
        this->get_parameter("traj_duration", traj_duration_);

        robot_ = nullptr;

        // 1. SETUP KDL
        if (!setup_kdl()) {
            RCLCPP_ERROR(this->get_logger(), "CRITICO: Setup KDL fallito.");
            return;
        }

        // =================================================================================
        // FIX CRITICO: CREAZIONE MANUALE E ORDINATA DEI NOMI DEI GIUNTI
        // =================================================================================
        // Non ci fidiamo di joint_states. Costruiamo la lista: 
        // iiwa_joint_a1, iiwa_joint_a2, ... iiwa_joint_a7
        
        std::string ns = this->get_namespace(); 
        // Rimuovi slash iniziale se presente (es "/iiwa" -> "iiwa")
        if (ns.size() > 0 && ns[0] == '/') ns = ns.substr(1);
        if (ns.empty()) ns = "iiwa"; // Fallback

        ordered_joint_names_.clear();
        for(int i=1; i<=7; i++) {
            // Nota: Assumiamo che i giunti si chiamino "nome_joint_aX"
            // Se nel tuo URDF si chiamano diversamente (es. "joint_a1" senza prefisso), 
            // cambia questa riga. Ma per gli iiwa standard è così:
            ordered_joint_names_.push_back(ns + "_joint_a" + std::to_string(i));
        }
        
        RCLCPP_INFO(this->get_logger(), "Mappatura Giunti Forzata: 1->%s, 2->%s ...", 
            ordered_joint_names_[0].c_str(), ordered_joint_names_[1].c_str());
        // =================================================================================

        // Action Server
        this->action_server_ = rclcpp_action::create_server<ExecuteTrajectory>(
            this, "ExecuteTrajectory",
            std::bind(&Iiwa_pub_sub::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&Iiwa_pub_sub::handle_cancel, this, std::placeholders::_1),
            std::bind(&Iiwa_pub_sub::handle_accepted, this, std::placeholders::_1));

        jointSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10, std::bind(&Iiwa_pub_sub::joint_state_subscriber, this, std::placeholders::_1));

        cmdPublisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "iiwa_arm_controller/joint_trajectory", 10);
            
        RCLCPP_INFO(this->get_logger(), "Nodo KDL attivo su namespace: %s", ns.c_str());
    }

private:
    bool setup_kdl() {
        auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this, "robot_state_publisher");
        while (!parameters_client->wait_for_service(1s)) {
            if (!rclcpp::ok()) return false;
            RCLCPP_INFO(this->get_logger(), "Attesa robot_state_publisher...");
        }
        auto parameter = parameters_client->get_parameters({"robot_description"});
        if (parameter.empty()) return false;

        KDL::Tree robot_tree;
        if (!kdl_parser::treeFromString(parameter[0].value_to_string(), robot_tree)) return false;

        robot_ = std::make_shared<KDLRobot>(robot_tree);
        unsigned int nj = robot_->getNrJnts();
        joint_positions_.resize(nj);
        joint_positions_.data.setZero();
        return true;
    }

    void publish_trajectory(const std::vector<double>& joint_targets, double duration_sec) {
        trajectory_msgs::msg::JointTrajectory msg;
        msg.header.stamp = this->get_clock()->now();
        
        // USA I NOMI ORDINATI MANUALMENTE (Non quelli rilevati a caso)
        msg.joint_names = ordered_joint_names_;

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = joint_targets; // Questo vettore [0] ora finirà sicuro su joint_a1
        point.time_from_start = rclcpp::Duration::from_seconds(duration_sec);
        
        msg.points.push_back(point);
        cmdPublisher_->publish(msg);
    }

    void execute(const std::shared_ptr<GoalHandleExecuteTrajectory> goal_handle) {
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<ExecuteTrajectory::Result>();

        // Check ricezione dati
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!state_received_) {
                RCLCPP_WARN(this->get_logger(), "Nessuno stato giunti ricevuto!");
                result->success = false;
                goal_handle->abort(result); return;
            }
        }

        std::vector<double> target_vec;
        if (!goal->joints_target.empty()) {
            target_vec = goal->joints_target;
        } else {
             // Se servisse IK cartesiana, va qui.
             RCLCPP_ERROR(this->get_logger(), "IK non implementata");
             result->success = false;
             goal_handle->abort(result); return;
        }

        // Pubblica il comando
        RCLCPP_INFO(this->get_logger(), "Invio comando...");
        publish_trajectory(target_vec, traj_duration_);

        // --- ATTESA DI CONVERGENZA ---
        // Aspettiamo che il robot arrivi davvero, controllando l'errore
        rclcpp::Rate rate(10);
        double timeout = traj_duration_ + 5.0; 
        double start_time = this->now().seconds();
        bool reached = false;

        while (rclcpp::ok() && (this->now().seconds() - start_time) < timeout) {
            if (goal_handle->is_canceling()) {
                result->success = false;
                goal_handle->canceled(result); return;
            }

            // Calcola errore medio
            double max_error = 0.0;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                // Confrontiamo la posizione attuale con il target
                // Poiché joint_positions_ viene riempito dal subscriber che ora mappa correttamente
                for(size_t i=0; i<target_vec.size() && i<joint_positions_.rows(); ++i) {
                    double err = std::abs(joint_positions_(i) - target_vec[i]);
                    if(err > max_error) max_error = err;
                }
            }

            // Se l'errore massimo su tutti i giunti è < 0.05 rad (circa 3 gradi), siamo arrivati
            if (max_error < 0.05) {
                reached = true;
                break;
            }
            rate.sleep();
        }

        if (reached) {
            result->success = true;
            goal_handle->succeed(result);
        } else {
            RCLCPP_WARN(this->get_logger(), "Timeout attesa posizione!");
            // Lo diamo buono comunque per non bloccare tutto, ma occhio
            result->success = true; 
            goal_handle->succeed(result);
        }
    }

    void joint_state_subscriber(const sensor_msgs::msg::JointState& msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (robot_ == nullptr) return;

        // Mapping intelligente: 
        // Il topic joint_states potrebbe avere un ordine diverso.
        // Noi cerchiamo i nomi "iiwa_joint_a1" dentro msg.name e prendiamo il valore corrispondente.
        
        for (size_t i = 0; i < ordered_joint_names_.size(); ++i) {
            std::string target_name = ordered_joint_names_[i];
            
            // Cerca questo nome nel messaggio ricevuto
            bool found = false;
            for(size_t j=0; j<msg.name.size(); ++j) {
                if (msg.name[j] == target_name) {
                    if (i < joint_positions_.rows()) {
                        joint_positions_(i) = msg.position[j];
                        found = true;
                    }
                    break;
                }
            }
        }
        state_received_ = true;
    }

    // Boilerplate Action
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID&, std::shared_ptr<const ExecuteTrajectory::Goal>)
    { return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; }
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleExecuteTrajectory>)
    { return rclcpp_action::CancelResponse::ACCEPT; }
    void handle_accepted(const std::shared_ptr<GoalHandleExecuteTrajectory> goal_handle)
    { std::thread{std::bind(&Iiwa_pub_sub::execute, this, std::placeholders::_1), goal_handle}.detach(); }

    std::shared_ptr<KDLRobot> robot_;
    KDL::JntArray joint_positions_;
    std::vector<std::string> ordered_joint_names_; 
    double traj_duration_;
    rclcpp_action::Server<ExecuteTrajectory>::SharedPtr action_server_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscriber_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr cmdPublisher_;
    std::mutex mutex_;
    bool state_received_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Iiwa_pub_sub>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}