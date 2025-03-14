#pragma once


#include <vector>

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"

#include "gary_can/socket_can_receiver.hpp"
#include "gary_can/socket_can_sender.hpp"
#include "utils/lk_tech_motors.hpp"
#include "utils/offline_detector.hpp"

namespace gary_hardware{

    typedef struct {
        std::shared_ptr<utils::LKTechMotor> motor;
        std::shared_ptr<utils::OfflineDetector> offlineDetector;
        std::string motor_name;
        std::shared_ptr<double> cmd;
        std::shared_ptr<double> cmd_raw;
        std::shared_ptr<double> offline;
        std::shared_ptr<double> reset_position;
    }lk_tech_motor_ctrl_t;


    class LKTechMotorSystem :public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(LKTechMotorSystem)

        hardware_interface::return_type configure(const hardware_interface::HardwareInfo & info) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        hardware_interface::return_type start() override;

        hardware_interface::return_type stop() override;

        hardware_interface::return_type read() override;

        hardware_interface::return_type write() override;

    private:
        std::string system_name;
        int cmd_id{};
        std::shared_ptr<driver::can::SocketCANSender> can_sender;
        std::shared_ptr<driver::can::SocketCANReceiver> can_receiver;

        std::vector<lk_tech_motor_ctrl_t> motors;
    };
}
