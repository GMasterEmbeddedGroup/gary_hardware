#include "gary_hardware/rm_imu_sensor.hpp"
#include "utils/fp16_convert.hpp"

#include <chrono>
#include <string>
#include <memory>
#include <vector>


namespace gary_hardware {

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn RMIMUSensor::on_init(const hardware_interface::HardwareInfo &info) {
        use_corrected_angle = false;

        //get sensor name
        this->sensor_name = info.name;

        RCLCPP_DEBUG(rclcpp::get_logger(this->sensor_name), "configuring");

        //call the base class initializer
        if (hardware_interface::SensorInterface::on_init(info) != CallbackReturn::SUCCESS)
        {
            return CallbackReturn::ERROR;
        }
        //check parameter "can_bus"
        if (info.hardware_parameters.count("can_bus") != 1) {
            RCLCPP_ERROR(rclcpp::get_logger(this->sensor_name), "invalid can bus definition in urdf");
           // this->status_ = hardware_interface::status::UNKNOWN;
            return CallbackReturn::ERROR;
        }
        std::string bus_name = info.hardware_parameters.at("can_bus");
        RCLCPP_DEBUG(rclcpp::get_logger(this->sensor_name), "using can bus %s", bus_name.c_str());

        //create socket can receiver
        this->can_receiver = std::make_shared<driver::can::SocketCANReceiver>(bus_name);


        //check parameter "orientation_can_id"
        if (info.hardware_parameters.count("orientation_can_id") != 1) {
            RCLCPP_ERROR(rclcpp::get_logger(this->sensor_name), "invalid orientation can id definition in urdf");
           // this->status_ = hardware_interface::status::UNKNOWN;
            return CallbackReturn::ERROR;
        }
        this->can_ids[0] = std::stoi(info.hardware_parameters.at("orientation_can_id"), nullptr, 16);
        RCLCPP_DEBUG(rclcpp::get_logger(this->sensor_name), "using orientation can id 0x%x", this->can_ids[0]);


        //check parameter "gyro_can_id"
        if (info.hardware_parameters.count("gyro_can_id") != 1) {
            RCLCPP_ERROR(rclcpp::get_logger(this->sensor_name), "invalid gyroscope can id definition in urdf");
          //  this->status_ = hardware_interface::status::UNKNOWN;
            return CallbackReturn::ERROR;
        }
        this->can_ids[1] = std::stoi(info.hardware_parameters.at("gyro_can_id"), nullptr, 16);
        RCLCPP_DEBUG(rclcpp::get_logger(this->sensor_name), "using gyroscope can id 0x%x", this->can_ids[1]);

//        //check parameter "corrected_angle_can_id"
//        if (info.hardware_parameters.count("corrected_angle_can_id") == 1) {
//            this->can_ids[1] = std::stoi(info.hardware_parameters.at("corrected_angle_can_id"), nullptr, 16);
//            RCLCPP_DEBUG(rclcpp::get_logger(this->sensor_name), "using corrected_angle can id 0x%x", this->can_ids[3]);
//            use_corrected_angle = true;
//        }
//
//        //check parameter "accel_can_id"
//        if (info.hardware_parameters.count("accel_can_id") != 1) {
//            RCLCPP_ERROR(rclcpp::get_logger(this->sensor_name), "invalid acceleration can id definition in urdf");
//            this->status_ = hardware_interface::status::UNKNOWN;
//            return hardware_interface::return_type::ERROR;
//        }
//        this->can_ids[2] = std::stoi(info.hardware_parameters.at("accel_can_id"), nullptr, 16);
//        RCLCPP_DEBUG(rclcpp::get_logger(this->sensor_name), "using acceleration can id 0x%x", this->can_ids[2]);


        //check parameter "update_rate"
        if (info.hardware_parameters.count("update_rate") != 1) {
            RCLCPP_ERROR(rclcpp::get_logger(this->sensor_name), "invalid update rate definition in urdf");
           // this->status_ = hardware_interface::status::UNKNOWN;
            return CallbackReturn::ERROR;
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
                RCLCPP_DEBUG(rclcpp::get_logger(this->sensor_name), "[%s] failed to bind can id 0x%x to bus",
                             this->can_receiver->ifname.c_str(), can_id);
            }
        }

       // this->status_ = hardware_interface::status::CONFIGURED;
        RCLCPP_INFO(rclcpp::get_logger(this->sensor_name), "configured");
        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> RMIMUSensor::export_state_interfaces() {

        //creat state interfaces
        std::vector<hardware_interface::StateInterface> state_interfaces;

//        state_interfaces.emplace_back(this->sensor_name, "orientation.x", &this->sensor_data[0]);
//        state_interfaces.emplace_back(this->sensor_name, "orientation.y", &this->sensor_data[1]);
//        state_interfaces.emplace_back(this->sensor_name, "orientation.z", &this->sensor_data[2]);
//        state_interfaces.emplace_back(this->sensor_name, "orientation.w", &this->sensor_data[3]);
        state_interfaces.emplace_back(this->sensor_name, "euler.x", &this->sensor_data[4]);
        state_interfaces.emplace_back(this->sensor_name, "euler.y", &this->sensor_data[5]);
        state_interfaces.emplace_back(this->sensor_name, "euler.z", &this->sensor_data[6]);
        state_interfaces.emplace_back(this->sensor_name, "euler_sum.x", &this->sensor_data[7]);
        state_interfaces.emplace_back(this->sensor_name, "euler_sum.y", &this->sensor_data[8]);
        state_interfaces.emplace_back(this->sensor_name, "euler_sum.z", &this->sensor_data[9]);
        state_interfaces.emplace_back(this->sensor_name, "angular_velocity.x", &this->sensor_data[10]);
        state_interfaces.emplace_back(this->sensor_name, "angular_velocity.y", &this->sensor_data[11]);
        state_interfaces.emplace_back(this->sensor_name, "angular_velocity.z", &this->sensor_data[12]);
//        state_interfaces.emplace_back(this->sensor_name, "linear_acceleration.x", &this->sensor_data[13]);
//        state_interfaces.emplace_back(this->sensor_name, "linear_acceleration.y", &this->sensor_data[14]);
//        state_interfaces.emplace_back(this->sensor_name, "linear_acceleration.z", &this->sensor_data[15]);

        state_interfaces.emplace_back(this->sensor_name, "offline", &this->offline);

        return state_interfaces;
    }


    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn RMIMUSensor::on_activate(const rclcpp_lifecycle::State & previous_state) {

        RCLCPP_DEBUG(rclcpp::get_logger(this->sensor_name), "starting");

        //update offline detector
        this->offlineDetector->update(true);

       // this->status_ = hardware_interface::status::STARTED;

        RCLCPP_INFO(rclcpp::get_logger(this->sensor_name), "started");

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn RMIMUSensor::on_deactivate(const rclcpp_lifecycle::State & previous_state) {

        RCLCPP_DEBUG(rclcpp::get_logger(this->sensor_name), "stopping");

     //   this->status_ = hardware_interface::status::STOPPED;

        RCLCPP_DEBUG(rclcpp::get_logger(this->sensor_name), "stopped");

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type RMIMUSensor::read(const rclcpp::Time & time, const rclcpp::Duration & period) {

        RCLCPP_DEBUG(rclcpp::get_logger(this->sensor_name), "reading");

        //update offline status
        if (this->offlineDetector->offline) {
            if (this->offline == 0)
                RCLCPP_ERROR(rclcpp::get_logger(this->sensor_name), "[%s] imu offline",
                             this->sensor_name.c_str());
            this->offline = 1;
        } else {
            if (this->offline == 1)
                RCLCPP_ERROR(rclcpp::get_logger(this->sensor_name), "[%s] imu online",
                             this->sensor_name.c_str());
            this->offline = 0;
        }

        //check if socket is down
        for (int can_id: {0x101,0x102}) {
            if (!this->can_receiver->is_opened[can_id]) {
                //reopen socket
                if (!this->can_receiver->open_socket(can_id)) {
                    RCLCPP_DEBUG(rclcpp::get_logger(this->sensor_name),
                                         "[%s] can receiver reopen failed, id 0x%x",
                                         this->can_receiver->ifname.c_str(), can_id);
                    this->offlineDetector->update(false);
                    return hardware_interface::return_type::ERROR;
                }
            }
        }

        typedef union{
            float f_out;
            uint8_t u8_in[4];
        } uint2fp32;

        struct can_frame frame{};
        int read_succ_cnt = 0;
        bool succ = false;
        //read gyro
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
            this->sensor_data[0] = 0.0;
            this->sensor_data[1] = 0.0;
            this->sensor_data[2] = 0.0;
            this->sensor_data[3] = 0.0;

            uint2fp32 temp;
            temp.u8_in[0] = frame.data[0];
            temp.u8_in[1] = frame.data[1];
            temp.u8_in[2] = frame.data[2];
            temp.u8_in[3] = frame.data[3];
            this->sensor_data[12] = static_cast<double>(temp.f_out);

            temp.u8_in[0] = frame.data[4];
            temp.u8_in[1] = frame.data[5];
            temp.u8_in[2] = frame.data[6];
            temp.u8_in[3] = frame.data[7];
            this->sensor_data[10] = static_cast<double>(temp.f_out);

            RCLCPP_INFO(rclcpp::get_logger(this->sensor_name),
                         "0x101 received yaw:%lf, pitch:%lf",sensor_data[12],sensor_data[10]
                         );

//            auto raw_gyro_x = (int16_t) (frame.data[0] | frame.data[1] << 8);
//            this->sensor_data[10] = (double) utils::half_to_float(raw_gyro_x);
//            auto raw_gyro_y = (int16_t) (frame.data[2] | frame.data[3] << 8);
            this->sensor_data[11] = 0.0;
//            auto raw_gyro_z = (int16_t) (frame.data[4] | frame.data[5] << 8);
//            this->sensor_data[12] = (double) utils::half_to_float(raw_gyro_z);

            read_succ_cnt++;
        }
        //read euler
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
            double tmp_x = this->sensor_data[4];
//            double tmp_y = this->sensor_data[5];
            double tmp_z = this->sensor_data[6];

            uint2fp32 temp;
            temp.u8_in[0] = frame.data[0];
            temp.u8_in[1] = frame.data[1];
            temp.u8_in[2] = frame.data[2];
            temp.u8_in[3] = frame.data[3];
            this->sensor_data[4] = static_cast<double>(temp.f_out);

            temp.u8_in[0] = frame.data[4];
            temp.u8_in[1] = frame.data[5];
            temp.u8_in[2] = frame.data[6];
            temp.u8_in[3] = frame.data[7];
            this->sensor_data[6] = static_cast<double>(temp.f_out);

            RCLCPP_INFO(rclcpp::get_logger(this->sensor_name),
                        "0x102 received yaw:%lf, pitch:%lf",sensor_data[12],sensor_data[10]
            );

            double euler_x_sum = this->sensor_data[4] - tmp_x;
            if (euler_x_sum > M_PI) euler_x_sum -= M_PI * 2;
            if (euler_x_sum < -M_PI) euler_x_sum += M_PI * 2;
            this->sensor_data[7] += euler_x_sum;

//            double euler_y_sum = this->sensor_data[5] - tmp_y;
//            if (euler_y_sum > M_PI) euler_y_sum -= M_PI * 2;
//            if (euler_y_sum < -M_PI) euler_y_sum += M_PI * 2;
//            this->sensor_data[8] += euler_y_sum;

            double euler_z_sum = this->sensor_data[6] - tmp_z;
            if (euler_z_sum > M_PI) euler_z_sum -= M_PI * 2;
            if (euler_z_sum < -M_PI) euler_z_sum += M_PI * 2;
            this->sensor_data[9] += euler_z_sum;

            read_succ_cnt++;
        }
        //read acceleration
        //get the latest data, read until socket buffer is empty
//        succ = false;
//        while (true) {
//            struct can_frame can_recv_frame_temp{};
//            if (this->can_receiver->read(this->can_ids[2], &can_recv_frame_temp)) {
//                frame = can_recv_frame_temp;
//                succ |= true;
//            } else {
//                succ |= false;
//                break;
//            }
//        }
//        if (succ) {
//            auto raw_accel_x = (int16_t) (frame.data[0] | frame.data[1] << 8);
//            this->sensor_data[13] = (double) utils::half_to_float(raw_accel_x);
//            auto raw_accel_y = (int16_t) (frame.data[2] | frame.data[3] << 8);
//            this->sensor_data[14] = (double) utils::half_to_float(raw_accel_y);
//            auto raw_accel_z = (int16_t) (frame.data[4] | frame.data[5] << 8);
//            this->sensor_data[15] = (double) utils::half_to_float(raw_accel_z);
//            read_succ_cnt++;
//        }


//        if(use_corrected_angle){
//            succ = false;
//            while (true) {
//                struct can_frame can_recv_frame_temp{};
//                if (this->can_receiver->read(this->can_ids[3], &can_recv_frame_temp)) {
//                    frame = can_recv_frame_temp;
//                    succ |= true;
//                } else {
//                    succ |= false;
//                    break;
//                }
//            }
//            if (succ) {
//                auto raw_euler_x = (int16_t) (frame.data[0] | frame.data[1] << 8);
//                this->sensor_data[4] = (double) utils::half_to_float(raw_euler_x);
//                auto raw_euler_y = (int16_t) (frame.data[2] | frame.data[3] << 8);
//                this->sensor_data[5] = (double) utils::half_to_float(raw_euler_y);
//                auto raw_euler_z = (int16_t) (frame.data[4] | frame.data[5] << 8);
//                this->sensor_data[6] = (double) utils::half_to_float(raw_euler_z);
//                read_succ_cnt++;
//            }
//        }

        //update offline status
        this->offlineDetector->update(read_succ_cnt > 0);

        return hardware_interface::return_type::OK;
    }

}   //namespace gary_hardware


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(gary_hardware::RMIMUSensor, hardware_interface::SensorInterface)
