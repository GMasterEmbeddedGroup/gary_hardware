#pragma once

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"

#include "gary_can/socket_can_receiver.hpp"
#include "gary_can/socket_can_sender.hpp"
#include "utils/rm_motors.hpp"
#include "utils/offline_detector.hpp"

namespace gary_hardware{

    typedef struct {
        std::shared_ptr<utils::RMMotor> motor;
        std::shared_ptr<utils::OfflineDetector> offlineDetector;
        std::string motor_name;
        std::shared_ptr<double> cmd;
        std::shared_ptr<double> cmd_raw;
        std::shared_ptr<double> offline;
        std::shared_ptr<double> reset_position;
    }rm_motor_ctrl_t;

    class RMMotorSystem : public hardware_interface::SystemInterface {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(RMMotorSystem)

        CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

        CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

        hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

        hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    private:
        std::string system_name;
        int cmd_id{};
        std::shared_ptr<driver::can::SocketCANSender> can_sender;
        std::shared_ptr<driver::can::SocketCANReceiver> can_receiver;

        std::vector<rm_motor_ctrl_t> motors;
    };
}
