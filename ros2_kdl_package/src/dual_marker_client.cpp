#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <iostream>
#include <thread>
#include <atomic>

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
        COOLDOWN
    };

    DualMarkerClient() : Node("dual_marker_client"), current_state_(State::COOLDOWN), initial_position_reached_(false) {
        action_client_ = rclcpp_action::create_client<ExecuteTrajectory>(this, "/iiwa2/ExecuteTrajectory");

        aruco_sub_ = this->create_subscription<aruco_msgs::msg::MarkerArray>(
            "/aruco_marker_publisher/markers", 10,
            std::bind(&DualMarkerClient::aruco_callback, this, std::placeholders::_1));

        attach_pub_ = this->create_publisher<std_msgs::msg::Empty>("/iiwa2/gripper/attach_package", 10);
        detach_pub_ = this->create_publisher<std_msgs::msg::Empty>("/iiwa2/gripper/detach_package", 10);
        attach2_pub_ = this->create_publisher<std_msgs::msg::Empty>("/iiwa2/gripper/attach_package2", 10);
        detach2_pub_ = this->create_publisher<std_msgs::msg::Empty>("/iiwa2/gripper/detach_package2", 10);

        RCLCPP_INFO(this->get_logger(), "🚀 Dual Marker Client Avviato con Switch Visione");
        std::thread(&DualMarkerClient::go_to_initial_position, this).detach();
    }

private:
    std::vector<double> marker1_step0_ = {-2.89, 0.45, 0.0, -1.0, 0.0, 0.8, 0.0};
    std::vector<double> marker1_step1_ = {-2.89, 0.0, 0.0, -1.25, 0.0, 0.5, 0.0};
    std::vector<double> marker1_step2_ = {1.2, 0.0, 0.0, -1.25, 0.0, 0.5, 0.0};
    std::vector<double> marker1_step3_ = {1.2, 0.65, 0.0, -1.25, 0.0, 0.5, 0.0};
    std::vector<double> marker1_step4_ = {1.2, 0.65, 0.0, -1.5, 0.0, 0.5, 0.0};
    std::vector<double> marker1_step5_ = {-2.89, 0.0, 0.0, -1.5, 0.0, 1.4, 0.0};
    std::vector<double> marker1_step6_ = {-2.89, 1.4, 0.0, -0.5, 0.0, 0.65, 0.0};
    std::vector<double> marker1_step7_ = {-2.89, 1.5, 0.0, -0.5, 0.0, 0.65, 0.0};


    std::vector<double> marker2_step0_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> marker2_step1_ = {-2.89, 0.0, 0.0, -1.5, 0.0, 0.5, 0.0};
    std::vector<double> marker2_step2_ = {-1.2, 0.0, 0.0, -1.5, 0.0, 0.5, 0.0};
    std::vector<double> marker2_step3_ = {-1.2, 0.65, 0.0, -1.5, 0.0, 0.5, 0.0};

    void go_to_initial_position() {
        if (!action_client_->wait_for_action_server(10s)) return;
        send_joint_goal_and_wait(marker1_step5_, 5.0, "joint"); //
        initial_position_reached_ = true;
        current_state_ = State::SEARCHING;
    }

    void aruco_callback(const aruco_msgs::msg::MarkerArray::SharedPtr msg) {
        if (!initial_position_reached_ || current_state_.load() != State::SEARCHING || msg->markers.empty()) return;
        int marker_id = msg->markers[0].id;
        if (marker_id == 1) {
            current_state_ = State::MARKER_1_DETECTED;
            std::thread(&DualMarkerClient::execute_marker_1_sequence, this).detach();
        } else if (marker_id == 2) {
            current_state_ = State::MARKER_2_DETECTED;
            std::thread(&DualMarkerClient::execute_marker_2_sequence, this).detach();
        }
    }

    void execute_marker_1_sequence() {
        current_state_ = State::EXECUTING_SEQUENCE_1;
        RCLCPP_INFO(this->get_logger(), "=== AVVIO SEQUENZA 1 (JOINT + VISION) ===");

       

        // 2. Switch a VISIONE per aggiustamento fine
        RCLCPP_INFO(this->get_logger(), "Attivazione Controllo Visione...");
        send_joint_goal_and_wait(marker1_step7_, 1.0, "vision"); // joints_target ignorato in vision
        std::this_thread::sleep_for(30s);

        // 3. Pick
        attach_pub_->publish(std_msgs::msg::Empty());

        // 4. Consegna con controllo JOINT
        send_joint_goal_and_wait(marker1_step0_, 5.0, "joint"); 
        send_joint_goal_and_wait(marker1_step1_, 5.0, "joint");
        send_joint_goal_and_wait(marker1_step2_, 5.0, "joint");
        send_joint_goal_and_wait(marker1_step3_, 5.0, "joint");
        send_joint_goal_and_wait(marker1_step4_, 5.0, "joint");

        detach_pub_->publish(std_msgs::msg::Empty());
        std::this_thread::sleep_for(2s);
        send_joint_goal_and_wait(marker2_step0_, 5.0, "joint");
        send_joint_goal_and_wait(marker1_step5_, 5.0, "joint");

        current_state_ = State::SEARCHING;
    }

    void execute_marker_2_sequence() {
        current_state_ = State::EXECUTING_SEQUENCE_2;
        RCLCPP_INFO(this->get_logger(), "=== AVVIO SEQUENZA 2 (JOINT + VISION) ===");

        
        // 2. Switch a VISIONE
        RCLCPP_INFO(this->get_logger(), "Attivazione Controllo Visione...");
        send_joint_goal_and_wait(marker1_step1_, 15.0, "vision");
        std::this_thread::sleep_for(10s);

        // 3. Pick
        attach2_pub_->publish(std_msgs::msg::Empty());

        // 4. Consegna
        send_joint_goal_and_wait(marker1_step0_, 5.0, "joint"); 
        send_joint_goal_and_wait(marker2_step1_, 5.0, "joint");
        send_joint_goal_and_wait(marker2_step2_, 5.0, "joint");
        send_joint_goal_and_wait(marker2_step3_, 5.0, "joint");
        
        detach2_pub_->publish(std_msgs::msg::Empty());
        std::this_thread::sleep_for(2s);
        send_joint_goal_and_wait(marker2_step0_, 5.0, "joint");
        send_joint_goal_and_wait(marker1_step5_, 5.0, "joint");

        current_state_ = State::SEARCHING;
    }

    // Aggiunto parametro std::string ctrl_type
    void send_joint_goal_and_wait(const std::vector<double>& joints, double wait_time, std::string ctrl_type) {
        if (!action_client_->wait_for_action_server(5s)) {
            RCLCPP_ERROR(this->get_logger(), "❌ Action server non disponibile!");
            return;
        }

        auto goal_msg = ExecuteTrajectory::Goal();
        goal_msg.joints_target = joints;
        goal_msg.ctrl = ctrl_type; // Passa il tipo di controllo al server

        RCLCPP_INFO(this->get_logger(), "📤 Invio goal: ctrl=%s, joints=[%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                    ctrl_type.c_str(), joints[0], joints[1], joints[2], joints[3], joints[4], joints[5], joints[6]);

        auto goal_handle_future = action_client_->async_send_goal(goal_msg);
        if (goal_handle_future.wait_for(5s) != std::future_status::ready) {
            RCLCPP_ERROR(this->get_logger(), "⏱️ Timeout invio goal");
            return;
        }

        auto goal_handle = goal_handle_future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "❌ Goal rifiutato dal server");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "✅ Goal accettato, attendo risultato...");
        auto result_future = action_client_->async_get_result(goal_handle);
        result_future.wait_for(std::chrono::seconds(static_cast<int>(wait_time) + 5));
        RCLCPP_INFO(this->get_logger(), "🏁 Goal completato");
    }

    std::atomic<State> current_state_;
    std::atomic<bool> initial_position_reached_;
    rclcpp_action::Client<ExecuteTrajectory>::SharedPtr action_client_;
    rclcpp::Subscription<aruco_msgs::msg::MarkerArray>::SharedPtr aruco_sub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr attach_pub_, detach_pub_, attach2_pub_, detach2_pub_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DualMarkerClient>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}