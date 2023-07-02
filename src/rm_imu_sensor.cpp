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


        //check parameter "imu_can_id"
        if (info.hardware_parameters.count("imu_can_id") != 1) {
            RCLCPP_ERROR(rclcpp::get_logger(this->sensor_name), "invalid imu can id definition in urdf");
            this->status_ = hardware_interface::status::UNKNOWN;
            return hardware_interface::return_type::ERROR;
        }
        this->can_ids[0] = std::stoi(info.hardware_parameters.at("imu_can_id"), nullptr, 16);
        RCLCPP_DEBUG(rclcpp::get_logger(this->sensor_name), "using imu can id 0x%x", this->can_ids[0]);



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


        //bind can id
        for (int can_id: this->can_ids) {
            if (!this->can_receiver->open_socket(can_id)) {
                RCLCPP_DEBUG(rclcpp::get_logger(this->sensor_name), "[%s] failed to bind can id 0x%x to bus",
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
        for (int can_id: {0x000}) {
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

        struct can_frame frame{};
        int read_succ_cnt = 0;
        static bool succ = false;
        static bool acc_flag = false;
        //read imu
        //get the latest data, read until socket buffer is empty
        struct can_frame can_recv_frame_temp{};
        if (this->can_receiver->read(this->can_ids[0], &can_recv_frame_temp)) {
            frame = can_recv_frame_temp;
            succ |= true;
        } else {
            succ |= false;
        }
        if (succ) {
            if(frame.data[0]==0xFC && last_num==0xFD) {
                Count_can=1;
                if((frame.data[1]==TYPE_AHRS)&&(frame.data[2]==AHRS_LEN)) acc_flag = true;
            }
            last_num=frame.data[7];

            if(Count_can)
            {
                for(int i=0;i<8;i++)
                {
                    ahrs_u8array[Count_can-1][i]=frame.data[i];
                }
                Count_can += 1;
            }


            if(acc_flag==1 && Count_can==AHRS_CAN)
            {
                Count_can=0;
                acc_flag = false;
                std::vector<uint8_t> rx_ahrs;
                rx_ahrs.clear();
                for(int k=0;k<(AHRS_CAN-1);k++)
                {
                    for(int i=0;i<8;i++)
                    {
                        rx_ahrs.emplace_back(ahrs_u8array[k][i]);
                    }
                }
                
                auto DATA_Trans = 
                        [this](uint8_t v1,uint8_t v2,uint8_t v3,uint8_t v4) -> float {
                            typedef union{
                                float f_out;
                                uint8_t u8_in[4];
                            } uint2fp32;
                            uint2fp32 transfer;
                            transfer.u8_in[0] = v1;
                            transfer.u8_in[1] = v2;
                            transfer.u8_in[2] = v3;
                            transfer.u8_in[3] = v4;
                            return transfer.f_out;
                };
                
                float RollSpeed=DATA_Trans(rx_ahrs[7],rx_ahrs[8],rx_ahrs[9],rx_ahrs[10]);     
                float PitchSpeed=DATA_Trans(rx_ahrs[11],rx_ahrs[12],rx_ahrs[13],rx_ahrs[14]); 
                float HeadingSpeed=DATA_Trans(rx_ahrs[15],rx_ahrs[16],rx_ahrs[17],rx_ahrs[18]);

                float Roll=DATA_Trans(rx_ahrs[19],rx_ahrs[20],rx_ahrs[21],rx_ahrs[22]);   
                float Pitch=DATA_Trans(rx_ahrs[23],rx_ahrs[24],rx_ahrs[25],rx_ahrs[26]);   
                float Heading=DATA_Trans(rx_ahrs[27],rx_ahrs[28],rx_ahrs[29],rx_ahrs[30]);	

                float Qw=DATA_Trans(rx_ahrs[31],rx_ahrs[32],rx_ahrs[33],rx_ahrs[34]);  
                float Qx=DATA_Trans(rx_ahrs[35],rx_ahrs[36],rx_ahrs[37],rx_ahrs[38]);
                float Qy=DATA_Trans(rx_ahrs[39],rx_ahrs[40],rx_ahrs[41],rx_ahrs[42]);
                float Qz=DATA_Trans(rx_ahrs[43],rx_ahrs[44],rx_ahrs[45],rx_ahrs[46]);

                double tmp_x = this->sensor_data[4];
                double tmp_y = this->sensor_data[5];
                double tmp_z = this->sensor_data[6];

                this->sensor_data[0] = static_cast<double>(Qx);
                this->sensor_data[1] = static_cast<double>(Qy);
                this->sensor_data[2] = static_cast<double>(Qz);
                this->sensor_data[3] = static_cast<double>(Qw);
                this->sensor_data[4] = static_cast<double>(Roll);
                this->sensor_data[5] = static_cast<double>(Pitch);
                this->sensor_data[6] = static_cast<double>(Heading);

                double euler_x_sum = this->sensor_data[4] - tmp_x;
                if (euler_x_sum > M_PI) euler_x_sum -= M_PI * 2;
                if (euler_x_sum < -M_PI) euler_x_sum += M_PI * 2;
                this->sensor_data[7] += euler_x_sum;

                double euler_y_sum = this->sensor_data[5] - tmp_y;
                if (euler_y_sum > M_PI) euler_y_sum -= M_PI * 2;
                if (euler_y_sum < -M_PI) euler_y_sum += M_PI * 2;
                this->sensor_data[8] += euler_y_sum;

                double euler_z_sum = this->sensor_data[6] - tmp_z;
                if (euler_z_sum > M_PI) euler_z_sum -= M_PI * 2;
                if (euler_z_sum < -M_PI) euler_z_sum += M_PI * 2;
                this->sensor_data[9] += euler_z_sum;


                this->sensor_data[10] = static_cast<double>(RollSpeed);
                this->sensor_data[11] = static_cast<double>(PitchSpeed);
                this->sensor_data[12] = static_cast<double>(HeadingSpeed);

            }

            read_succ_cnt++;
            if(last_num==0xFD){
                succ = false;
            }
        }

        //update offline status
        this->offlineDetector->update(read_succ_cnt > 0);

        return hardware_interface::return_type::OK;
    }

}   //namespace gary_hardware


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(gary_hardware::RMIMUSensor, hardware_interface::SensorInterface)
