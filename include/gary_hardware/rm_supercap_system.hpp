#pragma once

#include <vector>
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"

#include "gary_can/socket_can_receiver.hpp"
#include "gary_can/socket_can_sender.hpp"
#include "utils/offline_detector.hpp"

namespace gary_hardware {

    class RMSuperCAPSystem : public hardware_interface::SystemInterface {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(RMSuperCAPSystem)

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
        int fdb_id{};
        std::shared_ptr<driver::can::SocketCANSender> can_sender;
        std::shared_ptr<driver::can::SocketCANReceiver> can_receiver;
        std::shared_ptr<utils::OfflineDetector> offline_detector;
        double sensor_data[8]{};
        double cmd_data[1]{};
        double offline = 0;
    };
}

