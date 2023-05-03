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


    class RMSuperCAPSystem :public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(RMSuperCAPSystem)

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
        int fdb_id{};
        std::shared_ptr<driver::can::SocketCANSender> can_sender;
        std::shared_ptr<driver::can::SocketCANReceiver> can_receiver;
        std::shared_ptr<utils::OfflineDetector> offline_detector;
        double sensor_data[8]{};
        double cmd_data[1]{};
        double offline = 0;
    };
}
