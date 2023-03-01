#include "gary_hardware/rm_imu_sensor.hpp"
#include "utils/fp16_convert.hpp"

#include <chrono>
#include <string>
#include <memory>
#include <vector>


namespace gary_hardware {

    hardware_interface::return_type RMIMUSensor::configure(const hardware_interface::HardwareInfo &info) {

        //get sensor name
        this->sensor_name = info.name;

        RCLCPP_DEBUG(rclcpp::get_logger(this->sensor_name), "configuring");

        //call the base class initializer
        if (configure_default(info) != hardware_interface::return_type::OK) {
            this->status_ = hardware_interface::status::UNKNOWN;
            return hardware_interface::return_type::ERROR;
        }

        //check parameter "can_bus"
        if (info.hardware_parameters.count("can_bus") != 1) {
            RCLCPP_ERROR(rclcpp::get_logger(this->sensor_name), "invalid can bus definition in urdf");
            this->status_ = hardware_interface::status::UNKNOWN;
            return hardware_interface::return_type::ERROR;
        }
        std::string bus_name = info.hardware_parameters.at("can_bus");
        RCLCPP_DEBUG(rclcpp::get_logger(this->sensor_name), "using can bus %s", bus_name.c_str());

        //create socket can receiver
        this->can_receiver = std::make_shared<driver::can::SocketCANReceiver>(bus_name);


        //check parameter "orientation_can_id"
        if (info.hardware_parameters.count("orientation_can_id") != 1) {
            RCLCPP_ERROR(rclcpp::get_logger(this->sensor_name), "invalid orientation can id definition in urdf");
            this->status_ = hardware_interface::status::UNKNOWN;
            return hardware_interface::return_type::ERROR;
        }
        this->can_ids[0] = std::stoi(info.hardware_parameters.at("orientation_can_id"), nullptr, 16);
        RCLCPP_DEBUG(rclcpp::get_logger(this->sensor_name), "using orientation can id 0x%x", this->can_ids[0]);


        //check parameter "gyro_can_id"
        if (info.hardware_parameters.count("gyro_can_id") != 1) {
            RCLCPP_ERROR(rclcpp::get_logger(this->sensor_name), "invalid gyroscope can id definition in urdf");
            this->status_ = hardware_interface::status::UNKNOWN;
            return hardware_interface::return_type::ERROR;
        }
        this->can_ids[1] = std::stoi(info.hardware_parameters.at("gyro_can_id"), nullptr, 16);
        RCLCPP_DEBUG(rclcpp::get_logger(this->sensor_name), "using gyroscope can id 0x%x", this->can_ids[1]);


        //check parameter "accel_can_id"
        if (info.hardware_parameters.count("accel_can_id") != 1) {
            RCLCPP_ERROR(rclcpp::get_logger(this->sensor_name), "invalid acceleration can id definition in urdf");
            this->status_ = hardware_interface::status::UNKNOWN;
            return hardware_interface::return_type::ERROR;
        }
        this->can_ids[2] = std::stoi(info.hardware_parameters.at("accel_can_id"), nullptr, 16);
        RCLCPP_DEBUG(rclcpp::get_logger(this->sensor_name), "using acceleration can id 0x%x", this->can_ids[2]);


        //check parameter "update_rate"
        if (info.hardware_parameters.count("update_rate") != 1) {
            RCLCPP_ERROR(rclcpp::get_logger(this->sensor_name), "invalid update rate definition in urdf");
            this->status_ = hardware_interface::status::UNKNOWN;
            return hardware_interface::return_type::ERROR;
        }
        int update_rate = std::stoi(info.hardware_parameters.at("update_rate"));
        RCLCPP_DEBUG(rclcpp::get_logger(this->sensor_name), "using update rate %d", update_rate);

        //calculate offline detection threshold
        double threshold = 1.0f / (double) update_rate;
        if (threshold < 0.1f) threshold = 0.1f;
        if (threshold > 1.0f) threshold = 1.0f;

        //create offline detector
        this->offlineDetector = std::make_shared<utils::OfflineDetector>(threshold);


        RCLCPP_INFO(rclcpp::get_logger(this->sensor_name),
                    "add new imu, name: %s, can bus: %s, orientation id: 0x%x, gyro id: 0x%x, accel id: 0x%x,",
                    this->sensor_name.c_str(), bus_name.c_str(), this->can_ids[0], this->can_ids[1], this->can_ids[2]);


        //bind can id
        for (int can_id: this->can_ids) {
            if (!this->can_receiver->open_socket(can_id)) {
                RCLCPP_ERROR(rclcpp::get_logger(this->sensor_name), "[%s] failed to bind can id 0x%x to bus",
                             this->can_receiver->ifname.c_str(), can_id);
            }
        }

        this->status_ = hardware_interface::status::CONFIGURED;
        RCLCPP_INFO(rclcpp::get_logger(this->sensor_name), "configured");
        return hardware_interface::return_type::OK;
    }

    std::vector<hardware_interface::StateInterface> RMIMUSensor::export_state_interfaces() {

        //creat state interfaces
        std::vector<hardware_interface::StateInterface> state_interfaces;

        state_interfaces.emplace_back(this->sensor_name, "orientation.x", &this->sensor_data[0]);
        state_interfaces.emplace_back(this->sensor_name, "orientation.y", &this->sensor_data[1]);
        state_interfaces.emplace_back(this->sensor_name, "orientation.z", &this->sensor_data[2]);
        state_interfaces.emplace_back(this->sensor_name, "orientation.w", &this->sensor_data[3]);
        state_interfaces.emplace_back(this->sensor_name, "euler.x", &this->sensor_data[4]);
        state_interfaces.emplace_back(this->sensor_name, "euler.y", &this->sensor_data[5]);
        state_interfaces.emplace_back(this->sensor_name, "euler.z", &this->sensor_data[6]);
        state_interfaces.emplace_back(this->sensor_name, "angular_velocity.x", &this->sensor_data[7]);
        state_interfaces.emplace_back(this->sensor_name, "angular_velocity.y", &this->sensor_data[8]);
        state_interfaces.emplace_back(this->sensor_name, "angular_velocity.z", &this->sensor_data[9]);
        state_interfaces.emplace_back(this->sensor_name, "linear_acceleration.x", &this->sensor_data[10]);
        state_interfaces.emplace_back(this->sensor_name, "linear_acceleration.y", &this->sensor_data[11]);
        state_interfaces.emplace_back(this->sensor_name, "linear_acceleration.z", &this->sensor_data[12]);

        state_interfaces.emplace_back(this->sensor_name, "offline", &this->offline);

        return state_interfaces;
    }


    hardware_interface::return_type RMIMUSensor::start() {

        RCLCPP_DEBUG(rclcpp::get_logger(this->sensor_name), "starting");

        //update offline detector
        this->offlineDetector->update(true);

        this->status_ = hardware_interface::status::STARTED;

        RCLCPP_INFO(rclcpp::get_logger(this->sensor_name), "started");

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type RMIMUSensor::stop() {

        RCLCPP_DEBUG(rclcpp::get_logger(this->sensor_name), "stopping");

        this->status_ = hardware_interface::status::STOPPED;

        RCLCPP_DEBUG(rclcpp::get_logger(this->sensor_name), "stopped");

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type RMIMUSensor::read() {

        RCLCPP_DEBUG(rclcpp::get_logger(this->sensor_name), "reading");

        //update offline status
        this->offline = static_cast<double>(this->offlineDetector->offline);
        if (this->offlineDetector->offline) {
            rclcpp::Clock clock;
            RCLCPP_ERROR_THROTTLE(rclcpp::get_logger(this->sensor_name), clock, 1000, "[%s] offline",
                                 this->sensor_name.c_str());
        }

        //check if socket is down
        for (int can_id: this->can_ids) {
            if (!this->can_receiver->is_opened[can_id]) {
                //reopen socket
                if (!this->can_receiver->open_socket(can_id)) {
                    rclcpp::Clock clock;
                    RCLCPP_WARN_THROTTLE(rclcpp::get_logger(this->sensor_name), clock, 1000,
                                         "[%s] can receiver reopen failed, id 0x%x",
                                         this->can_receiver->ifname.c_str(), can_id);
                    this->offlineDetector->update(false);
                    return hardware_interface::return_type::ERROR;
                }
            }
        }

        struct can_frame frame{};
        int read_succ_cnt = 0;
        bool succ = false;
        //read orientation
        //get the latest data, read until socket buffer is empty
        while (true) {
            struct can_frame can_recv_frame_temp{};
            if (this->can_receiver->read(this->can_ids[0], &can_recv_frame_temp)) {
                frame = can_recv_frame_temp;
                succ |= true;
            } else {
                succ |= false;
                break;
            }
        }
        if (succ) {
            auto x = (uint16_t) (frame.data[0] | frame.data[1] << 8);
            this->sensor_data[0] = (double) utils::half_to_float(x);
            auto y = (uint16_t) (frame.data[2] | frame.data[3] << 8);
            this->sensor_data[1] = (double) utils::half_to_float(y);
            auto z = (uint16_t) (frame.data[4] | frame.data[5] << 8);
            this->sensor_data[2] = (double) utils::half_to_float(z);
            auto w = (uint16_t) (frame.data[6] | frame.data[7] << 8);
            this->sensor_data[3] = (double) utils::half_to_float(w);

            this->sensor_data[4] = atan2(2 * (y * z + w * x), w * w - x * x - y * y + z * z);
            this->sensor_data[5] = asin(-2 * (x * z - w * y));
            this->sensor_data[6] = atan2(2 * (x * y + w * z), w * w + x * x - y * y - z * z);

            read_succ_cnt++;
        }
        //read gyroscope
        //get the latest data, read until socket buffer is empty
        succ = false;
        while (true) {
            struct can_frame can_recv_frame_temp{};
            if (this->can_receiver->read(this->can_ids[1], &can_recv_frame_temp)) {
                frame = can_recv_frame_temp;
                succ |= true;
            } else {
                succ |= false;
                break;
            }
        }
        if (succ) {
            auto x = (int16_t) (frame.data[0] | frame.data[1] << 8);
            this->sensor_data[7] = (double) utils::half_to_float(x);
            auto y = (int16_t) (frame.data[2] | frame.data[3] << 8);
            this->sensor_data[8] = (double) utils::half_to_float(y);
            auto z = (int16_t) (frame.data[4] | frame.data[5] << 8);
            this->sensor_data[9] = (double) utils::half_to_float(z);
            read_succ_cnt++;
        }
        //read acceleration
        //get the latest data, read until socket buffer is empty
        succ = false;
        while (true) {
            struct can_frame can_recv_frame_temp{};
            if (this->can_receiver->read(this->can_ids[2], &can_recv_frame_temp)) {
                frame = can_recv_frame_temp;
                succ |= true;
            } else {
                succ |= false;
                break;
            }
        }
        if (succ) {
            auto x = (int16_t) (frame.data[0] | frame.data[1] << 8);
            this->sensor_data[10] = (double) utils::half_to_float(x);
            auto y = (int16_t) (frame.data[2] | frame.data[3] << 8);
            this->sensor_data[11] = (double) utils::half_to_float(y);
            auto z = (int16_t) (frame.data[4] | frame.data[5] << 8);
            this->sensor_data[12] = (double) utils::half_to_float(z);
            read_succ_cnt++;
        }

        //update offline status
        this->offlineDetector->update(read_succ_cnt > 0);

        return hardware_interface::return_type::OK;
    }

}   //namespace gary_hardware


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(gary_hardware::RMIMUSensor, hardware_interface::SensorInterface)
