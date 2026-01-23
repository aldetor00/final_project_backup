#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <iostream>

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

        // ArUco Subscriber (riceve TUTTI i marker rilevati)
        aruco_sub_ = this->create_subscription<aruco_msgs::msg::MarkerArray>(
            "/aruco_marker_publisher/markers", 10,
            std::bind(&DualMarkerClient::aruco_callback, this, std::placeholders::_1));

        // Gripper Publishers per MARKER 1
        attach_pub_ = this->create_publisher<std_msgs::msg::Empty>("/iiwa2/gripper/attach_package", 10);
        detach_pub_ = this->create_publisher<std_msgs::msg::Empty>("/iiwa2/gripper/detach_package", 10);

        // Gripper Publishers per MARKER 2
        attach2_pub_ = this->create_publisher<std_msgs::msg::Empty>("/iiwa2/gripper/attach_package2", 10);
        detach2_pub_ = this->create_publisher<std_msgs::msg::Empty>("/iiwa2/gripper/detach_package2", 10);

        RCLCPP_INFO(this->get_logger(), "🔍 Dual Marker Client Ready - Waiting for markers...");
    }

private:
    // ========== SEQUENZE JOINT PER MARKER 1 ==========
    std::vector<double> marker1_step1_ = {3.0, 0.0, 0.0, -1.5, 0.0, 0.5, 0.0};
    std::vector<double> marker1_step2_ = {1.5, 0.0, 0.0, -1.5, 0.0, 0.5, 0.0};
    std::vector<double> marker1_step3_ = {1.5, 1.5, 0.0, -1.0, 0.0, 0.5, 0.0};

    // ========== SEQUENZE JOINT PER MARKER 2 ==========
    std::vector<double> marker2_step1_ = {3.0, 0.0, 0.0, -1.5, 0.0, 0.5, 0.0};
    std::vector<double> marker2_step2_ = {-1.5, 0.0, 0.0, -1.5, 0.0, 0.5, 0.0};
    std::vector<double> marker2_step3_ = {-1.5, 1.5, 0.0, -1.0, 0.0, 0.5, 0.0};

    void aruco_callback(const aruco_msgs::msg::MarkerArray::SharedPtr msg) {
        if (current_state_ != State::SEARCHING) return;

        if (msg->markers.empty()) {
            return; // Nessun marker visibile
        }

        // Prendi il primo marker rilevato (puoi anche cercare specificamente ID 1 o 2)
        int marker_id = msg->markers[0].id;

        if (marker_id == 1) {
            RCLCPP_WARN(this->get_logger(), "🎯 MARKER 1 RILEVATO! Avvio sequenza...");
            current_state_ = State::MARKER_1_DETECTED;
            detected_marker_id_ = 1;
            execute_marker_1_sequence();
        }
        else if (marker_id == 2) {
            RCLCPP_WARN(this->get_logger(), "🎯 MARKER 2 RILEVATO! Avvio sequenza...");
            current_state_ = State::MARKER_2_DETECTED;
            detected_marker_id_ = 2;
            execute_marker_2_sequence();
        }
    }

    // ========== SEQUENZA MARKER 1 ==========
    void execute_marker_1_sequence() {
        current_state_ = State::EXECUTING_SEQUENCE_1;
        sequence_step_ = 0;

        RCLCPP_INFO(this->get_logger(), "═══════════════════════════════════════");
        RCLCPP_INFO(this->get_logger(), "   SEQUENZA MARKER 1 - PACCO DESTRO");
        RCLCPP_INFO(this->get_logger(), "═══════════════════════════════════════");

        // Step 1: Attach gripper
        RCLCPP_INFO(this->get_logger(), "Step 1/5: Attach gripper...");
        attach_pub_->publish(std_msgs::msg::Empty());
        rclcpp::sleep_for(2s); // Aumentato a 2 secondi

        // Step 2: Primo movimento
        RCLCPP_INFO(this->get_logger(), "Step 2/5: Movimento a posizione intermedia...");
        send_joint_goal_and_wait(marker1_step1_, 5.0); // Aspetta 5 secondi
        
        RCLCPP_INFO(this->get_logger(), "Step 3/5: Movimento verso area di deposito...");
        send_joint_goal_and_wait(marker1_step2_, 5.0);
        
        RCLCPP_INFO(this->get_logger(), "Step 4/5: Posizionamento finale...");
        send_joint_goal_and_wait(marker1_step3_, 5.0);
        
        RCLCPP_INFO(this->get_logger(), "Step 5/5: Detach gripper...");
        detach_pub_->publish(std_msgs::msg::Empty());
        rclcpp::sleep_for(2s);

        RCLCPP_INFO(this->get_logger(), "═══════════════════════════════════════");
        RCLCPP_INFO(this->get_logger(), "✅ SEQUENZA MARKER 1 COMPLETATA!");
        RCLCPP_INFO(this->get_logger(), "═══════════════════════════════════════");
        current_state_ = State::FINISHED;
    }

    // ========== SEQUENZA MARKER 2 ==========
    void execute_marker_2_sequence() {
        current_state_ = State::EXECUTING_SEQUENCE_2;
        sequence_step_ = 0;

        RCLCPP_INFO(this->get_logger(), "═══════════════════════════════════════");
        RCLCPP_INFO(this->get_logger(), "   SEQUENZA MARKER 2 - PACCO SINISTRO");
        RCLCPP_INFO(this->get_logger(), "═══════════════════════════════════════");

        // Step 1: Attach gripper2
        RCLCPP_INFO(this->get_logger(), "Step 1/5: Attach gripper2...");
        attach2_pub_->publish(std_msgs::msg::Empty());
        rclcpp::sleep_for(2s);

        // Step 2: Primo movimento
        RCLCPP_INFO(this->get_logger(), "Step 2/5: Movimento a posizione intermedia...");
        send_joint_goal_and_wait(marker2_step1_, 5.0);
        
        RCLCPP_INFO(this->get_logger(), "Step 3/5: Movimento verso area di deposito...");
        send_joint_goal_and_wait(marker2_step2_, 5.0);
        
        RCLCPP_INFO(this->get_logger(), "Step 4/5: Posizionamento finale...");
        send_joint_goal_and_wait(marker2_step3_, 5.0);
        
        RCLCPP_INFO(this->get_logger(), "Step 5/5: Detach gripper2...");
        detach2_pub_->publish(std_msgs::msg::Empty());
        rclcpp::sleep_for(2s);

        RCLCPP_INFO(this->get_logger(), "═══════════════════════════════════════");
        RCLCPP_INFO(this->get_logger(), "✅ SEQUENZA MARKER 2 COMPLETATA!");
        RCLCPP_INFO(this->get_logger(), "═══════════════════════════════════════");
        current_state_ = State::FINISHED;
    }

    void result_callback(const GoalHandle::WrappedResult & result) {
        // Questa funzione non viene più usata nella nuova implementazione
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_ERROR(this->get_logger(), "❌ Movimento fallito!");
        }
    }

    void send_joint_goal_and_wait(const std::vector<double>& joints, double wait_time) {
        if (!action_client_->wait_for_action_server(5s)) {
            RCLCPP_ERROR(this->get_logger(), "Action server non disponibile!");
            return;
        }

        auto goal_msg = ExecuteTrajectory::Goal();
        goal_msg.joints_target = joints;

        auto send_goal_options = rclcpp_action::Client<ExecuteTrajectory>::SendGoalOptions();
        
        // Invia il goal e aspetta che sia accettato
        auto goal_handle_future = action_client_->async_send_goal(goal_msg, send_goal_options);
        
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_handle_future) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Invio goal fallito");
            return;
        }

        auto goal_handle = goal_handle_future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal rifiutato dal server");
            return;
        }

        // Aspetta il risultato
        auto result_future = action_client_->async_get_result(goal_handle);
        
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future, 
                                               std::chrono::seconds(static_cast<int>(wait_time) + 2)) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_WARN(this->get_logger(), "Timeout nell'attesa del risultato");
        }

        auto result = result_future.get();
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "✓ Movimento completato");
        } else {
            RCLCPP_ERROR(this->get_logger(), "✗ Movimento non riuscito");
        }
    }

    // Variabili Membro
    State current_state_;
    int sequence_step_;
    int detected_marker_id_;

    rclcpp_action::Client<ExecuteTrajectory>::SharedPtr action_client_;
    rclcpp::Subscription<aruco_msgs::msg::MarkerArray>::SharedPtr aruco_sub_;

    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr attach_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr detach_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr attach2_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr detach2_pub_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DualMarkerClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}