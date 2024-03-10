#include "gary_hardware/rm_supercap_system.hpp"

#include <chrono>
#include <string>
#include <memory>
#include <vector>

namespace gary_hardware {

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn RMSuperCAPSystem::on_init(const hardware_interface::HardwareInfo &info) {
        //get system name
        this->system_name = info.name;

        RCLCPP_DEBUG(rclcpp::get_logger(this->system_name), "configuring");

        //call the base class initializer
        if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
        {
            return CallbackReturn::ERROR;
        }

        //check parameter "can_bus"
        if (info.hardware_parameters.count("can_bus") != 1) {
            RCLCPP_ERROR(rclcpp::get_logger(this->system_name), "invalid can bus definition in urdf");
           // this->status_ = hardware_interface::status::UNKNOWN;
            return CallbackReturn::ERROR;
        }
        std::string bus_name = info.hardware_parameters.at("can_bus");
        RCLCPP_DEBUG(rclcpp::get_logger(this->system_name), "using can bus %s", bus_name.c_str());

        //create socket can sender and receiver
        this->can_sender = std::make_shared<driver::can::SocketCANSender>(bus_name);
        this->can_receiver = std::make_shared<driver::can::SocketCANReceiver>(bus_name);

        //check parameter "cmd_id"
        if (info.hardware_parameters.count("cmd_id") != 1) {
            RCLCPP_ERROR(rclcpp::get_logger(this->system_name), "invalid cmd id definition in urdf");
           // this->status_ = hardware_interface::status::UNKNOWN;
            return CallbackReturn::ERROR;
        }
        this->cmd_id = std::stoi(info.hardware_parameters.at("cmd_id"), nullptr, 16);
        RCLCPP_DEBUG(rclcpp::get_logger(this->system_name), "using cmd id 0x%x", this->cmd_id);

        //check parameter "fdb_id"
        if (info.hardware_parameters.count("fdb_id") != 1) {
            RCLCPP_ERROR(rclcpp::get_logger(this->system_name), "invalid fdb id definition in urdf");
          //  this->status_ = hardware_interface::status::UNKNOWN;
            return CallbackReturn::ERROR;
        }
        this->fdb_id = std::stoi(info.hardware_parameters.at("fdb_id"), nullptr, 16);
        RCLCPP_DEBUG(rclcpp::get_logger(this->system_name), "using fdb id 0x%x", this->fdb_id);

        //open can sender
        if (!this->can_sender->open_socket()) {
            RCLCPP_DEBUG(rclcpp::get_logger(this->system_name), "[%s] failed to open can sender socket",
                         this->can_sender->ifname.c_str());
        }

        //open can receiver
        if (!this->can_receiver->open_socket(this->fdb_id)) {
            RCLCPP_DEBUG(rclcpp::get_logger(this->system_name), "[%s] failed to open can sender socket",
                         this->can_sender->ifname.c_str());
        }

        int update_rate = 1000;
        if (info.hardware_parameters.count("update_rate") == 1) {
            update_rate = std::stoi(info.hardware_parameters.at("update_rate"));
            RCLCPP_INFO(rclcpp::get_logger(this->system_name),
                        "using custom update rate %d", update_rate);
        }
        //calculate offline detection threshold
        double threshold = 1.0f / (double) update_rate;
        if (threshold < 0.1f) threshold = 0.1f;
        if (threshold > 1.0f) threshold = 1.0f;
        this->offline_detector = std::make_shared<utils::OfflineDetector>(threshold);

       // this->status_ = hardware_interface::status::CONFIGURED;
        RCLCPP_INFO(rclcpp::get_logger(this->system_name), "configured");
        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> RMSuperCAPSystem::export_state_interfaces() {

        //creat state interfaces
        std::vector<hardware_interface::StateInterface> state_interfaces;

        state_interfaces.emplace_back(this->system_name, "cap_mode", &this->sensor_data[0]);
        state_interfaces.emplace_back(this->system_name, "loop_mode", &this->sensor_data[1]);
        state_interfaces.emplace_back(this->system_name, "percent", &this->sensor_data[2]);
        state_interfaces.emplace_back(this->system_name, "voltage_input", &this->sensor_data[3]);
        state_interfaces.emplace_back(this->system_name, "voltage_cap", &this->sensor_data[4]);
        state_interfaces.emplace_back(this->system_name, "current_input", &this->sensor_data[5]);
        state_interfaces.emplace_back(this->system_name, "current_output", &this->sensor_data[6]);
        state_interfaces.emplace_back(this->system_name, "power_set", &this->sensor_data[7]);
        state_interfaces.emplace_back(this->system_name, "offline", &this->offline);

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> RMSuperCAPSystem::export_command_interfaces() {

        //creat state interfaces
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        command_interfaces.emplace_back(this->system_name, "power_set", &this->cmd_data[0]);

        return command_interfaces;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn RMSuperCAPSystem::on_activate(const rclcpp_lifecycle::State & previous_state) {

        RCLCPP_DEBUG(rclcpp::get_logger(this->system_name), "starting");

        //update offline detector
        this->offline_detector->update(true);

       // this->status_ = hardware_interface::status::STARTED;

        RCLCPP_INFO(rclcpp::get_logger(this->system_name), "started");

        return CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn RMSuperCAPSystem::on_deactivate(const rclcpp_lifecycle::State & previous_state) {

        RCLCPP_DEBUG(rclcpp::get_logger(this->system_name), "stopping");

        //this->status_ = hardware_interface::status::STOPPED;

        RCLCPP_DEBUG(rclcpp::get_logger(this->system_name), "stopped");

        return CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type RMSuperCAPSystem::read(const rclcpp::Time & time, const rclcpp::Duration & period) {

        RCLCPP_DEBUG(rclcpp::get_logger(this->system_name), "reading");

        //update offline status
        if (this->offline_detector->offline) {
            if (this->offline == 0.0)
                RCLCPP_ERROR(rclcpp::get_logger(this->system_name), "[%s] supercap offline",
                             this->system_name.c_str());
            this->offline = 1.0;
        } else {
            if (this->offline == 1.0)
                RCLCPP_ERROR(rclcpp::get_logger(this->system_name), "[%s] supercap online",
                             this->system_name.c_str());
            this->offline = 0.0;
        }

        //check if socket is down
        if (!this->can_receiver->is_opened[this->fdb_id]) {
            //reopen socket
            if (!this->can_receiver->open_socket(this->fdb_id)) {
                RCLCPP_DEBUG(rclcpp::get_logger(this->system_name),
                             "[%s] can receiver reopen failed, id 0x%x",
                             this->can_receiver->ifname.c_str(), this->fdb_id);
                this->offline_detector->update(false);
                return hardware_interface::return_type::ERROR;
            }
        }

        struct can_frame frame{};
        bool succ = false;
        //read orientation
        //get the latest data, read until socket buffer is empty
        while (true) {
            struct can_frame can_recv_frame_temp{};
            if (this->can_receiver->read(this->fdb_id, &can_recv_frame_temp)) {
                frame = can_recv_frame_temp;
                succ |= true;
            } else {
                succ |= false;
                break;
            }
        }
        if (succ) {
            //update offline status
            this->offline_detector->update(true);

            this->sensor_data[0] = frame.data[0] & 0x03; //cap_mode
            this->sensor_data[1] = (frame.data[0] >> 2) & 0x01; //loop_mode
            this->sensor_data[2] = frame.data[1]; //percent
            this->sensor_data[3] = static_cast<double>(frame.data[2]) / 10.0; //voltage_input
            this->sensor_data[4] = static_cast<double>(frame.data[3]) / 8.0; //voltage_cap
            this->sensor_data[5] = static_cast<double>((int8_t)frame.data[4]) / 16; //current_input
            this->sensor_data[6] = static_cast<double>((int8_t)frame.data[5]) / 16; //current_output
            this->sensor_data[7] = frame.data[6]; //power_set

        } else {
            //update offline status
            this->offline_detector->update(false);
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type RMSuperCAPSystem::write(const rclcpp::Time & time, const rclcpp::Duration & period) {

        RCLCPP_DEBUG(rclcpp::get_logger(this->system_name), "writing");

        //check if socket is down
        if (!this->can_sender->is_opened) {
            //reopen socket
            if (!this->can_sender->open_socket()) {
                RCLCPP_DEBUG(rclcpp::get_logger(this->system_name),
                             "[%s] can sender reopen failed",
                             this->can_sender->ifname.c_str());
                return hardware_interface::return_type::ERROR;
            }
        }

        struct can_frame frame{};
        frame.can_dlc = 8;
        frame.can_id = this->cmd_id;
        frame.data[0] = static_cast<uint8_t>(this->cmd_data[0]);


        //send the can frame
        if (this->can_sender->send(frame)) {
            RCLCPP_DEBUG(rclcpp::get_logger(this->system_name), "[%s] can frame send succ, cmd: 0x%x",
                         this->can_sender->ifname.c_str(), this->cmd_id);
            return hardware_interface::return_type::OK;
        } else {
            RCLCPP_DEBUG(rclcpp::get_logger(this->system_name),
                         "[%s] can send failed, cmd: 0x%x",
                         this->can_sender->ifname.c_str(), this->cmd_id);
        }
        return hardware_interface::return_type::ERROR;
    }

}   //namespace gary_hardware


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(gary_hardware::RMSuperCAPSystem, hardware_interface::SystemInterface)
