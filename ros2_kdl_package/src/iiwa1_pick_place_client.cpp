#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <iostream>
#include <thread>
#include <atomic>
#include <future>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/empty.hpp"
// Importiamo l'azione custom
#include "ros2_kdl_package/action/execute_trajectory.hpp"

using namespace std::chrono_literals;

class Iiwa1PickPlaceClient : public rclcpp::Node {
public:
    using ExecuteTrajectory = ros2_kdl_package::action::ExecuteTrajectory;
    using GoalHandle = rclcpp_action::ClientGoalHandle<ExecuteTrajectory>;

    enum class State {
        INIT,
        PICKING_PACKAGE_1,
        PLACING_PACKAGE_1,
        WAITING_TURTLEBOT,
        PICKING_PACKAGE_2,
        PLACING_PACKAGE_2,
        FINISHED
    };

    Iiwa1PickPlaceClient() : Node("iiwa1_pick_place_client"), current_state_(State::INIT) {
        
        // --- 1. ACTION CLIENT ---
        // Si collega al server nel namespace /iiwa
        action_client_ = rclcpp_action::create_client<ExecuteTrajectory>(this, "/iiwa/ExecuteTrajectory");

        // --- 2. GRIPPER PUBLISHERS ---
        attach_pkg1_pub_ = this->create_publisher<std_msgs::msg::Empty>("/iiwa/gripper/attach_package", 10);
        detach_pkg1_pub_ = this->create_publisher<std_msgs::msg::Empty>("/iiwa/gripper/detach_package", 10);
        attach_pkg2_pub_ = this->create_publisher<std_msgs::msg::Empty>("/iiwa/gripper/attach_package2", 10);
        detach_pkg2_pub_ = this->create_publisher<std_msgs::msg::Empty>("/iiwa/gripper/detach_package2", 10);

        RCLCPP_INFO(this->get_logger(), "🚀 IIWA1 Pick&Place Client (Action Version) Avviato");

        // Avvio sequenza in un thread separato
        execution_thread_ = std::thread(&Iiwa1PickPlaceClient::execute_full_sequence, this);
    }

    ~Iiwa1PickPlaceClient() {
        if (execution_thread_.joinable()) execution_thread_.join();
    }

private:
    // ========== TRAIETTORIE ==========
    std::vector<double> prepick_pkg1_ = {-2.8, 0.4, 0.0, -1.45, 0.0, 1.0, 0.0};
    std::vector<double> pick_pkg1_ = {-2.8, 0.63, 0.0, -1.45, 0.0, 1.0, 0.0};
    std::vector<double> prepick_pkg2_ = {2.8, 0.4, 0.0, -1.45, 0.0, 1.0, 0.0};
    std::vector<double> pick_pkg2_ = {2.8, 0.63, 0.0, -1.45, 0.0, 1.0, 0.0};
    
    std::vector<double> place_approach_ = {0.0, 0.4, 0.0, -1.40, 0.0, 1.0, 0.0};
    std::vector<double> place_final_ = {0.0, 0.7, 0.0, -1.60, 0.0, 1.0, 0.0};

    void execute_full_sequence() {
        RCLCPP_INFO(this->get_logger(), "⏳ Attesa Action Server...");
        if (!action_client_->wait_for_action_server(10s)) {
            RCLCPP_ERROR(this->get_logger(), "❌ Action Server non disponibile!");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "🎬 Inizio sequenza automatica...");

        // ================= SEQUENZA PACKAGE 1 =================
        current_state_ = State::PICKING_PACKAGE_1;
        RCLCPP_INFO(this->get_logger(), "📦 INIZIO SEQUENZA PACKAGE 1");
        
        std::this_thread::sleep_for(1s); 

        RCLCPP_INFO(this->get_logger(), "🤖 Spostamento verso Package 1...");
        send_joint_goal(prepick_pkg1_, 5.0); 
        send_joint_goal(pick_pkg1_, 2.0); 

        RCLCPP_INFO(this->get_logger(), "🔗 Attach Package 1");
        attach_pkg1_pub_->publish(std_msgs::msg::Empty());
        std::this_thread::sleep_for(2s); 

        current_state_ = State::PLACING_PACKAGE_1;
        RCLCPP_INFO(this->get_logger(), "📍 Spostamento verso posizione Place...");
        send_joint_goal(place_approach_, 5.0);
        
        RCLCPP_INFO(this->get_logger(), "⬇️  Abbassamento finale...");
        send_joint_goal(place_final_, 5.0);

        RCLCPP_INFO(this->get_logger(), "🔓 Detach Package 1");
        detach_pkg1_pub_->publish(std_msgs::msg::Empty());
        std::this_thread::sleep_for(2s);

        RCLCPP_INFO(this->get_logger(), "✅ Package 1 consegnato!");

        // ================= ATTESA TURTLEBOT =================
        current_state_ = State::WAITING_TURTLEBOT;
        RCLCPP_INFO(this->get_logger(), "⏳ ATTESA RITORNO TURTLEBOT (280s)");

        for (int i = 170; i > 0; i -= 10) {
            RCLCPP_INFO(this->get_logger(), "⏱️  %d secondi rimanenti...", i);
            std::this_thread::sleep_for(10s);
        }
        RCLCPP_INFO(this->get_logger(), "✅ Attesa completata!");

        // ================= SEQUENZA PACKAGE 2 =================
        current_state_ = State::PICKING_PACKAGE_2;
        RCLCPP_INFO(this->get_logger(), "📦 INIZIO SEQUENZA PACKAGE 2");

        std::this_thread::sleep_for(1s);

        RCLCPP_INFO(this->get_logger(), "🤖 Spostamento verso Package 2...");
        send_joint_goal(prepick_pkg2_, 5.0);
        send_joint_goal(pick_pkg2_, 2.0);

        RCLCPP_INFO(this->get_logger(), "🔗 Attach Package 2");
        attach_pkg2_pub_->publish(std_msgs::msg::Empty());
        std::this_thread::sleep_for(2s);

        current_state_ = State::PLACING_PACKAGE_2;
        RCLCPP_INFO(this->get_logger(), "📍 Spostamento verso posizione Place...");
        send_joint_goal(place_approach_, 5.0);
        
        RCLCPP_INFO(this->get_logger(), "⬇️  Abbassamento finale...");
        send_joint_goal(place_final_, 5.0);
        
        RCLCPP_INFO(this->get_logger(), "🔓 Detach Package 2");
        detach_pkg2_pub_->publish(std_msgs::msg::Empty());
        std::this_thread::sleep_for(2s);

        current_state_ = State::FINISHED;
        RCLCPP_INFO(this->get_logger(), "🎉 MISSIONE COMPLETATA!");
    }

    void send_joint_goal(const std::vector<double>& joints, double duration_sec) {
        auto goal_msg = ExecuteTrajectory::Goal();
        goal_msg.joints_target = joints;
        
        auto goal_handle_future = action_client_->async_send_goal(goal_msg);
        if (goal_handle_future.wait_for(5s) != std::future_status::ready) {
            RCLCPP_ERROR(this->get_logger(), "Timeout invio goal");
            return;
        }

        auto goal_handle = goal_handle_future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal rifiutato dal server");
            return;
        }

        auto result_future = action_client_->async_get_result(goal_handle);
        
        RCLCPP_INFO(this->get_logger(), "   → Movimento in corso...");
        
        // Aspetta che finisca. Aggiungiamo 5 secondi di margine al tempo previsto.
        if (result_future.wait_for(std::chrono::seconds((int)duration_sec + 5)) != std::future_status::ready) {
            RCLCPP_WARN(this->get_logger(), "⚠️ Timeout attesa risultato");
            return;
        }

        auto result = result_future.get();
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "   ✓ Movimento completato");
        } else {
            RCLCPP_ERROR(this->get_logger(), "   ✗ Errore nel movimento");
        }
    }

    std::atomic<State> current_state_;
    rclcpp_action::Client<ExecuteTrajectory>::SharedPtr action_client_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr attach_pkg1_pub_, detach_pkg1_pub_, attach_pkg2_pub_, detach_pkg2_pub_;
    std::thread execution_thread_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Iiwa1PickPlaceClient>();
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}