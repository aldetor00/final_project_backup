#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <iostream>
#include <thread> // Necessario per gestire i thread

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/empty.hpp"
#include "aruco_msgs/msg/marker_array.hpp"
#include "ros2_kdl_package/action/execute_trajectory.hpp"

using namespace std::chrono_literals;

class DualMarkerClient : public rclcpp::Node {
public:
    using ExecuteTrajectory = ros2_kdl_package::action::ExecuteTrajectory;
    using GoalHandle = rclcpp_action::ClientGoalHandle<ExecuteTrajectory>;

    enum class State {
        SEARCHING,
        MARKER_1_DETECTED,
        MARKER_2_DETECTED,
        EXECUTING_SEQUENCE_1,
        EXECUTING_SEQUENCE_2,
        FINISHED
    };

    DualMarkerClient() : Node("dual_marker_client"), current_state_(State::SEARCHING) {

        // Action Client
        action_client_ = rclcpp_action::create_client<ExecuteTrajectory>(this, "/iiwa2/ExecuteTrajectory");

        // ArUco Subscriber
        aruco_sub_ = this->create_subscription<aruco_msgs::msg::MarkerArray>(
            "/aruco_marker_publisher/markers", 10,
            std::bind(&DualMarkerClient::aruco_callback, this, std::placeholders::_1));

        // Gripper Publishers per MARKER 1
        attach_pub_ = this->create_publisher<std_msgs::msg::Empty>("/iiwa2/gripper/attach_package", 10);
        detach_pub_ = this->create_publisher<std_msgs::msg::Empty>("/iiwa2/gripper/detach_package", 10);

        // Gripper Publishers per MARKER 2
        attach2_pub_ = this->create_publisher<std_msgs::msg::Empty>("/iiwa2/gripper/attach_package2", 10);
        detach2_pub_ = this->create_publisher<std_msgs::msg::Empty>("/iiwa2/gripper/detach_package2", 10);

        RCLCPP_INFO(this->get_logger(), "🔍 Dual Marker Client Pronto - In attesa di marker...");
    }

private:
    // Sequenze Joint Marker 1
        std::vector<double> marker1_step0_ = {-3.0, 1.15, 0.0, -1.0, 0.0, 0.5, 0.0};

    std::vector<double> marker1_step1_ = {-3.0, 0.0, 0.0, -1.5, 0.0, 0.5, 0.0};
    std::vector<double> marker1_step2_ = {1.5, 0.0, 0.0, -1.5, 0.0, 0.5, 0.0};
    std::vector<double> marker1_step3_ = {1.5, 0.65, 0.0, -1.5, 0.0, 0.5, 0.0};
    std::vector<double> marker1_step4_ = {-3.0, 1.0, 0.0, -1.0, 0.0, 0.5, 0.0};

    // Sequenze Joint Marker 2
    std::vector<double> marker2_step1_ = {-3.0, 0.0, 0.0, -1.5, 0.0, 0.5, 0.0};
    std::vector<double> marker2_step2_ = {-1.5, 0.0, 0.0, -1.5, 0.0, 0.5, 0.0};
    std::vector<double> marker2_step3_ = {-1.5, 1.5, 0.0, -1.0, 0.0, 0.5, 0.0};

    void aruco_callback(const aruco_msgs::msg::MarkerArray::SharedPtr msg) {
        if (current_state_ != State::SEARCHING || msg->markers.empty()) return;

        int marker_id = msg->markers[0].id;

        if (marker_id == 1) {
            RCLCPP_WARN(this->get_logger(), "🎯 MARKER 1 RILEVATO! Avvio thread sequenza...");
            current_state_ = State::MARKER_1_DETECTED;
            // Avvio della sequenza in un thread separato per non bloccare l'executor
            std::thread(&DualMarkerClient::execute_marker_1_sequence, this).detach();
        }
        else if (marker_id == 2) {
            RCLCPP_WARN(this->get_logger(), "🎯 MARKER 2 RILEVATO! Avvio thread sequenza...");
            current_state_ = State::MARKER_2_DETECTED;
            std::thread(&DualMarkerClient::execute_marker_2_sequence, this).detach();
        }
    }

    void execute_marker_1_sequence() {
        current_state_ = State::EXECUTING_SEQUENCE_1;
        RCLCPP_INFO(this->get_logger(), "=== AVVIO SEQUENZA 1 ===");
        send_joint_goal_and_wait(marker1_step0_, 5.0);
      
        attach_pub_->publish(std_msgs::msg::Empty());
        std::this_thread::sleep_for(2s);

        send_joint_goal_and_wait(marker1_step1_, 5.0);
        send_joint_goal_and_wait(marker1_step2_, 5.0);
        send_joint_goal_and_wait(marker1_step3_, 5.0);

        detach_pub_->publish(std_msgs::msg::Empty());
        std::this_thread::sleep_for(2s);
        send_joint_goal_and_wait(marker1_step4_, 5.0);

        RCLCPP_INFO(this->get_logger(), "✅ SEQUENZA 1 COMPLETATA!");
        current_state_ = State::FINISHED;
    }

    void execute_marker_2_sequence() {
        current_state_ = State::EXECUTING_SEQUENCE_2;
        RCLCPP_INFO(this->get_logger(), "=== AVVIO SEQUENZA 2 ===");
        send_joint_goal_and_wait(marker1_step0_, 5.0);

        attach2_pub_->publish(std_msgs::msg::Empty());
        std::this_thread::sleep_for(2s);

        send_joint_goal_and_wait(marker2_step1_, 5.0);
        send_joint_goal_and_wait(marker2_step2_, 5.0);
        send_joint_goal_and_wait(marker2_step3_, 5.0);
        
        detach2_pub_->publish(std_msgs::msg::Empty());
        std::this_thread::sleep_for(2s);
        send_joint_goal_and_wait(marker1_step4_, 5.0);


        RCLCPP_INFO(this->get_logger(), "✅ SEQUENZA 2 COMPLETATA!");
        current_state_ = State::FINISHED;
    }

    void send_joint_goal_and_wait(const std::vector<double>& joints, double wait_time) {
    if (!action_client_->wait_for_action_server(5s)) {
        RCLCPP_ERROR(this->get_logger(), "Action server non disponibile!");
        return;
    }

    auto goal_msg = ExecuteTrajectory::Goal();
    goal_msg.joints_target = joints;

    // 1. Invio del goal (NON usare spin_until_future_complete qui)
    auto goal_handle_future = action_client_->async_send_goal(goal_msg);
    
    // Aspettiamo la risposta del server usando wait_for sul future stesso
    if (goal_handle_future.wait_for(5s) != std::future_status::ready) {
        RCLCPP_ERROR(this->get_logger(), "Invio goal fallito o timeout");
        return;
    }

    auto goal_handle = goal_handle_future.get();
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal rifiutato dal server");
        return;
    }

    // 2. Attesa del risultato finale
    auto result_future = action_client_->async_get_result(goal_handle);
    
    // Il thread si mette in pausa finché il risultato non è pronto
    if (result_future.wait_for(std::chrono::seconds(static_cast<int>(wait_time) + 2)) != 
        std::future_status::ready) 
    {
        RCLCPP_WARN(this->get_logger(), "Timeout nell'attesa del risultato");
        return;
    }

    auto result = result_future.get();
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(this->get_logger(), "✓ Movimento completato");
    } else {
        RCLCPP_ERROR(this->get_logger(), "✗ Movimento non riuscito");
    }
}

    State current_state_;
    rclcpp_action::Client<ExecuteTrajectory>::SharedPtr action_client_;
    rclcpp::Subscription<aruco_msgs::msg::MarkerArray>::SharedPtr aruco_sub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr attach_pub_, detach_pub_, attach2_pub_, detach2_pub_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DualMarkerClient>();
    
    // MultiThreadedExecutor permette di gestire contemporaneamente 
    // i callback e l'attesa degli action result
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}