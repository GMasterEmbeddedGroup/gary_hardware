#include "gary_hardware/rm_imu_sensor.hpp"
#include "utils/fp16_convert.hpp"

#include <chrono>
#include <string>
#include <memory>
#include <vector>


namespace gary_hardware {

    RMIMUSensor::RMIMUSensor() {
        RCLCPP_INFO(rclcpp::get_logger("rm_imu_system"), "rm imu system build time %s %s", __DATE__, __TIME__);
    }

    hardware_interface::return_type RMIMUSensor::configure(const hardware_interface::HardwareInfo &info) {

        RCLCPP_DEBUG(rclcpp::get_logger("rm_imu_system"), "configuring");

        //call the base class initializer
        if (configure_default(info) != hardware_interface::return_type::OK) {
            this->status_ = hardware_interface::status::UNKNOWN;
            return hardware_interface::return_type::ERROR;
        }

        //get sensor name
        this->sensor_name = info.name;

        //check parameter "can_bus"
        if (info.hardware_parameters.count("can_bus") != 1) {
            RCLCPP_ERROR(rclcpp::get_logger("rm_imu_system"), "invalid can bus definition in urdf");
            this->status_ = hardware_interface::status::UNKNOWN;
            return hardware_interface::return_type::ERROR;
        }
        std::string bus_name = info.hardware_parameters.at("can_bus");
        RCLCPP_DEBUG(rclcpp::get_logger("rm_imu_system"), "using can bus %s", bus_name.c_str());

        //create socket can receiver
        this->can_receiver = std::make_shared<driver::can::SocketCANReceiver>(bus_name);


        //check parameter "orientation_can_id"
        if (info.hardware_parameters.count("orientation_can_id") != 1) {
            RCLCPP_ERROR(rclcpp::get_logger("rm_imu_system"), "invalid orientation can id definition in urdf");
            this->status_ = hardware_interface::status::UNKNOWN;
            return hardware_interface::return_type::ERROR;
        }
        this->can_ids[0] = std::stoi(info.hardware_parameters.at("orientation_can_id"), nullptr, 16);
        RCLCPP_DEBUG(rclcpp::get_logger("rm_imu_system"), "using orientation can id 0x%x", this->can_ids[0]);


        //check parameter "gyro_can_id"
        if (info.hardware_parameters.count("gyro_can_id") != 1) {
            RCLCPP_ERROR(rclcpp::get_logger("rm_imu_system"), "invalid gyroscope can id definition in urdf");
            this->status_ = hardware_interface::status::UNKNOWN;
            return hardware_interface::return_type::ERROR;
        }
        this->can_ids[1] = std::stoi(info.hardware_parameters.at("gyro_can_id"), nullptr, 16);
        RCLCPP_DEBUG(rclcpp::get_logger("rm_imu_system"), "using gyroscope can id 0x%x", this->can_ids[1]);


        //check parameter "accel_can_id"
        if (info.hardware_parameters.count("accel_can_id") != 1) {
            RCLCPP_ERROR(rclcpp::get_logger("rm_imu_system"), "invalid acceleration can id definition in urdf");
            this->status_ = hardware_interface::status::UNKNOWN;
            return hardware_interface::return_type::ERROR;
        }
        this->can_ids[2] = std::stoi(info.hardware_parameters.at("accel_can_id"), nullptr, 16);
        RCLCPP_DEBUG(rclcpp::get_logger("rm_imu_system"), "using acceleration can id 0x%x", this->can_ids[2]);


        //check parameter "update_rate"
        if (info.hardware_parameters.count("update_rate") != 1) {
            RCLCPP_ERROR(rclcpp::get_logger("rm_imu_system"), "invalid update rate definition in urdf");
            this->status_ = hardware_interface::status::UNKNOWN;
            return hardware_interface::return_type::ERROR;
        }
        int update_rate = std::stoi(info.hardware_parameters.at("update_rate"));
        RCLCPP_DEBUG(rclcpp::get_logger("rm_imu_system"), "using update rate %d", update_rate);

        //calculate offline detection threshold
        double threshold = 1.0f / (double) update_rate;
        if (threshold < 0.1f) threshold = 0.1f;
        if (threshold > 1.0f) threshold = 1.0f;

        //create offline detector
        this->offlineDetector = std::make_shared<utils::OfflineDetector>(threshold);


        this->status_ = hardware_interface::status::CONFIGURED;
        RCLCPP_INFO(rclcpp::get_logger("rm_imu_system"), "configured");
        return hardware_interface::return_type::OK;
    }

    std::vector<hardware_interface::StateInterface> RMIMUSensor::export_state_interfaces() {

        //creat state interfaces
        std::vector<hardware_interface::StateInterface> state_interfaces;

        state_interfaces.emplace_back(this->sensor_name, "orientation.x", &this->sensor_data[0]);
        state_interfaces.emplace_back(this->sensor_name, "orientation.y", &this->sensor_data[1]);
        state_interfaces.emplace_back(this->sensor_name, "orientation.z", &this->sensor_data[2]);
        state_interfaces.emplace_back(this->sensor_name, "orientation.w", &this->sensor_data[3]);
        state_interfaces.emplace_back(this->sensor_name, "angular_velocity.x", &this->sensor_data[4]);
        state_interfaces.emplace_back(this->sensor_name, "angular_velocity.y", &this->sensor_data[5]);
        state_interfaces.emplace_back(this->sensor_name, "angular_velocity.z", &this->sensor_data[6]);
        state_interfaces.emplace_back(this->sensor_name, "linear_acceleration.x", &this->sensor_data[7]);
        state_interfaces.emplace_back(this->sensor_name, "linear_acceleration.y", &this->sensor_data[8]);
        state_interfaces.emplace_back(this->sensor_name, "linear_acceleration.z", &this->sensor_data[9]);

        state_interfaces.emplace_back(this->sensor_name, "offline", &this->offline);

        return state_interfaces;
    }


    hardware_interface::return_type RMIMUSensor::start() {

        RCLCPP_DEBUG(rclcpp::get_logger("rm_imu_system"), "starting");

        //bind can id
        for (int can_id: this->can_ids) {
            if (!this->can_receiver->bind(can_id)) {
                RCLCPP_ERROR(rclcpp::get_logger("rm_imu_system"), "failed to bind can id 0x%x to bus", can_id);
                this->status_ = hardware_interface::status::UNKNOWN;
                return hardware_interface::return_type::ERROR;
            }
        }

        //update offline detector
        this->offlineDetector->update(true);

        this->status_ = hardware_interface::status::STARTED;

        RCLCPP_INFO(rclcpp::get_logger("rm_imu_system"), "started");

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type RMIMUSensor::stop() {

        RCLCPP_DEBUG(rclcpp::get_logger("rm_imu_system"), "stopping");

        this->status_ = hardware_interface::status::STOPPED;

        RCLCPP_DEBUG(rclcpp::get_logger("rm_imu_system"), "stopped");

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type RMIMUSensor::read() {

        RCLCPP_DEBUG(rclcpp::get_logger("rm_imu_system"), "reading");

        struct can_frame frame{};
        int read_succ_cnt = 0;

        //read orientation
        if (this->can_receiver->read(this->can_ids[0], &frame)) {
            auto x = (uint16_t) (frame.data[0] | frame.data[1] << 8);
            this->sensor_data[0] = (double) utils::half_to_float(x);
            auto y = (uint16_t) (frame.data[2] | frame.data[3] << 8);
            this->sensor_data[1] = (double) utils::half_to_float(y);
            auto z = (uint16_t) (frame.data[4] | frame.data[5] << 8);
            this->sensor_data[2] = (double) utils::half_to_float(z);
            auto w = (uint16_t) (frame.data[6] | frame.data[7] << 8);
            this->sensor_data[3] = (double) utils::half_to_float(w);
            read_succ_cnt++;
        }
        //read gyroscope
        if (can_receiver->read(this->can_ids[1], &frame)) {
            auto x = (int16_t) (frame.data[0] | frame.data[1] << 8);
            this->sensor_data[4] = (double) utils::half_to_float(x);
            auto y = (int16_t) (frame.data[2] | frame.data[3] << 8);
            this->sensor_data[5] = (double) utils::half_to_float(y);
            auto z = (int16_t) (frame.data[4] | frame.data[5] << 8);
            this->sensor_data[6] = (double) utils::half_to_float(z);
            read_succ_cnt++;
        }
        //read acceleration
        if (can_receiver->read(this->can_ids[2], &frame)) {
            auto x = (int16_t) (frame.data[0] | frame.data[1] << 8);
            this->sensor_data[7] = (double) utils::half_to_float(x);
            auto y = (int16_t) (frame.data[2] | frame.data[3] << 8);
            this->sensor_data[8] = (double) utils::half_to_float(y);
            auto z = (int16_t) (frame.data[4] | frame.data[5] << 8);
            this->sensor_data[9] = (double) utils::half_to_float(z);
            read_succ_cnt++;
        }

        //update offline status
        this->offlineDetector->update(read_succ_cnt > 0);
        this->offline = static_cast<double>(this->offlineDetector->offline);
        if (this->offlineDetector->offline) {
            rclcpp::Clock clock;
            RCLCPP_WARN_THROTTLE(rclcpp::get_logger("rm_imu_system"), clock, 1000,
                                 "[name %s orientation 0x%x gyro 0x%x accel 0x%x] offline",
                                 this->sensor_name.c_str(), this->can_ids[0], this->can_ids[1], this->can_ids[2]);
        }

        return hardware_interface::return_type::OK;
    }

}   //namespace gary_hardware


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(gary_hardware::RMIMUSensor, hardware_interface::SensorInterface)
