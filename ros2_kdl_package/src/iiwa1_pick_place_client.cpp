#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <iostream>
#include <thread>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using namespace std::chrono_literals;

class Iiwa1PickPlaceClient : public rclcpp::Node {
public:
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

        // Publisher diretto per trajectory (bypassiamo l'action server)
        cmd_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/iiwa/iiwa_arm_controller/joint_trajectory", 10);

        // Gripper Publishers per PACKAGE 1
        attach_pkg1_pub_ = this->create_publisher<std_msgs::msg::Empty>("/iiwa/gripper/attach_package", 10);
        detach_pkg1_pub_ = this->create_publisher<std_msgs::msg::Empty>("/iiwa/gripper/detach_package", 10);

        // Gripper Publishers per PACKAGE 2
        attach_pkg2_pub_ = this->create_publisher<std_msgs::msg::Empty>("/iiwa/gripper/attach_package2", 10);
        detach_pkg2_pub_ = this->create_publisher<std_msgs::msg::Empty>("/iiwa/gripper/detach_package2", 10);

        RCLCPP_INFO(this->get_logger(), "🚀 IIWA1 Pick&Place Client Avviato");
        
        // Breve attesa per assicurarsi che i publisher siano pronti
        std::this_thread::sleep_for(1s);
        
        // Avvia automaticamente la sequenza completa
        std::thread(&Iiwa1PickPlaceClient::execute_full_sequence, this).detach();
    }

private:
    // ========== TRAIETTORIE PACKAGE 1 ==========
    std::vector<double> pick_pkg1_ = {-2.8, 0.5, 0.0, -1.45, 0.0, 1.0, 0.0};
    
    // ========== TRAIETTORIE PACKAGE 2 ==========
    std::vector<double> pick_pkg2_ = {2.8, 0.5, 0.0, -1.45, 0.0, 1.0, 0.0};
    
    // ========== TRAIETTORIE PLACE (comuni ad entrambi) ==========
    std::vector<double> place_approach_ = {0.0, 0.5, 0.0, -1.45, 0.0, 1.0, 0.0};
    std::vector<double> place_final_ = {0.0, 0.5, 0.0, -1.55, 0.0, 1.0, 0.0};

    void execute_full_sequence() {
        RCLCPP_INFO(this->get_logger(), "🎬 Inizio sequenza automatica...");

        // ===== SEQUENZA PACKAGE 1 =====
        current_state_ = State::PICKING_PACKAGE_1;
        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "╔═══════════════════════════════════╗");
        RCLCPP_INFO(this->get_logger(), "║  📦 INIZIO SEQUENZA PACKAGE 1     ║");
        RCLCPP_INFO(this->get_logger(), "╚═══════════════════════════════════╝");
        
        // Pick Package 1
        RCLCPP_INFO(this->get_logger(), "🤖 Spostamento verso Package 1...");
        publish_joint_trajectory(pick_pkg1_, 4.0);
        std::this_thread::sleep_for(5s);  // Attesa movimento
        
        RCLCPP_INFO(this->get_logger(), "🔗 Attach Package 1");
        attach_pkg1_pub_->publish(std_msgs::msg::Empty());
        std::this_thread::sleep_for(2s);

        // Place Package 1
        current_state_ = State::PLACING_PACKAGE_1;
        RCLCPP_INFO(this->get_logger(), "📍 Spostamento verso posizione Place...");
        publish_joint_trajectory(place_approach_, 4.0);
        std::this_thread::sleep_for(5s);
        
        RCLCPP_INFO(this->get_logger(), "⬇️  Abbassamento finale...");
        publish_joint_trajectory(place_final_, 4.0);
        std::this_thread::sleep_for(5s);
        
        RCLCPP_INFO(this->get_logger(), "🔓 Detach Package 1");
        detach_pkg1_pub_->publish(std_msgs::msg::Empty());
        std::this_thread::sleep_for(2s);

        RCLCPP_INFO(this->get_logger(), "✅ Package 1 consegnato su fra2mo!");
        
        // ===== ATTESA TURTLEBOT =====
        current_state_ = State::WAITING_TURTLEBOT;
        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "╔═══════════════════════════════════╗");
        RCLCPP_INFO(this->get_logger(), "║  ⏳ ATTESA RITORNO TURTLEBOT      ║");
        RCLCPP_INFO(this->get_logger(), "║     (40 secondi)                  ║");
        RCLCPP_INFO(this->get_logger(), "╚═══════════════════════════════════╝");
        
        // Countdown visivo
        for (int i = 40; i > 0; i -= 10) {
            RCLCPP_INFO(this->get_logger(), "⏱️  %d secondi rimanenti...", i);
            std::this_thread::sleep_for(10s);
        }
        RCLCPP_INFO(this->get_logger(), "✅ Attesa completata! Turtlebot dovrebbe essere tornato.");

        // ===== SEQUENZA PACKAGE 2 =====
        current_state_ = State::PICKING_PACKAGE_2;
        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "╔═══════════════════════════════════╗");
        RCLCPP_INFO(this->get_logger(), "║  📦 INIZIO SEQUENZA PACKAGE 2     ║");
        RCLCPP_INFO(this->get_logger(), "╚═══════════════════════════════════╝");
        
        // Pick Package 2
        RCLCPP_INFO(this->get_logger(), "🤖 Spostamento verso Package 2...");
        publish_joint_trajectory(pick_pkg2_, 4.0);
        std::this_thread::sleep_for(5s);
        
        RCLCPP_INFO(this->get_logger(), "🔗 Attach Package 2");
        attach_pkg2_pub_->publish(std_msgs::msg::Empty());
        std::this_thread::sleep_for(2s);

        // Place Package 2
        current_state_ = State::PLACING_PACKAGE_2;
        RCLCPP_INFO(this->get_logger(), "📍 Spostamento verso posizione Place...");
        publish_joint_trajectory(place_approach_, 4.0);
        std::this_thread::sleep_for(5s);
        
        RCLCPP_INFO(this->get_logger(), "⬇️  Abbassamento finale...");
        publish_joint_trajectory(place_final_, 4.0);
        std::this_thread::sleep_for(5s);
        
        RCLCPP_INFO(this->get_logger(), "🔓 Detach Package 2");
        detach_pkg2_pub_->publish(std_msgs::msg::Empty());
        std::this_thread::sleep_for(2s);

        RCLCPP_INFO(this->get_logger(), "✅ Package 2 consegnato su fra2mo!");
        
        // ===== FINE =====
        current_state_ = State::FINISHED;
        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "╔═══════════════════════════════════╗");
        RCLCPP_INFO(this->get_logger(), "║  🎉 MISSIONE COMPLETATA!          ║");
        RCLCPP_INFO(this->get_logger(), "║  Tutti i package consegnati       ║");
        RCLCPP_INFO(this->get_logger(), "╚═══════════════════════════════════╝");
    }

    void publish_joint_trajectory(const std::vector<double>& joints, double duration_sec) {
        trajectory_msgs::msg::JointTrajectory msg;
        
        // IMPORTANTE: Usa timestamp = 0 per dire "esegui subito"
        // Oppure usa now() + un piccolo offset per dare tempo al controller
        msg.header.stamp = rclcpp::Time(0);  // Timestamp vuoto = esegui appena ricevuto
        msg.header.frame_id = "";
        
        msg.joint_names = {
            "iiwa_joint_a1", "iiwa_joint_a2", "iiwa_joint_a3",
            "iiwa_joint_a4", "iiwa_joint_a5", "iiwa_joint_a6", "iiwa_joint_a7"
        };

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = joints;
        
        // Aggiungi velocità e accelerazioni
        point.velocities = std::vector<double>(7, 0.0);
        point.accelerations = std::vector<double>(7, 0.0);
        
        point.time_from_start = rclcpp::Duration::from_seconds(duration_sec);
        
        msg.points.push_back(point);
        
        RCLCPP_INFO(this->get_logger(), "  → Pubblicato comando movimento (durata: %.1fs)", duration_sec);
        cmd_publisher_->publish(msg);
    }

    std::atomic<State> current_state_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr cmd_publisher_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr attach_pkg1_pub_, detach_pkg1_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr attach_pkg2_pub_, detach_pkg2_pub_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Iiwa1PickPlaceClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}