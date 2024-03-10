#pragma once

#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"

#include "gary_can/socket_can_receiver.hpp"
#include "gary_can/socket_can_sender.hpp"
#include "utils/offline_detector.hpp"

namespace gary_hardware{
    class RMIMUSensor : public hardware_interface::SensorInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(RMIMUSensor)

        CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

        CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

        hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    private:
        std::shared_ptr<driver::can::SocketCANReceiver> can_receiver;
        std::shared_ptr<utils::OfflineDetector> offlineDetector;
        std::string sensor_name;
        int can_ids[4]{};
        double sensor_data[16]{};
        double offline{};
        bool use_corrected_angle;
    };
}

