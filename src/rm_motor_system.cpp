#include "gary_hardware/rm_motor_system.hpp"

#include <chrono>
#include <string>
#include <memory>
#include <vector>

namespace gary_hardware {

    RMMotorSystem::RMMotorSystem() {
        RCLCPP_INFO(rclcpp::get_logger("rm_motor_system"), "rm motor system build time %s %s", __DATE__, __TIME__);
    }

    hardware_interface::return_type RMMotorSystem::configure(const hardware_interface::HardwareInfo &info) {

        RCLCPP_DEBUG(rclcpp::get_logger("rm_motor_system"), "configuring");

        //call the base class initializer
        if (configure_default(info) != hardware_interface::return_type::OK) {
            this->status_ = hardware_interface::status::UNKNOWN;
            return hardware_interface::return_type::ERROR;
        }

        //check parameter "can_bus"
        if (info.hardware_parameters.count("can_bus") != 1) {
            RCLCPP_ERROR(rclcpp::get_logger("rm_motor_system"), "invalid can bus definition in urdf");
            this->status_ = hardware_interface::status::UNKNOWN;
            return hardware_interface::return_type::ERROR;
        }
        std::string bus_name = info.hardware_parameters.at("can_bus");
        RCLCPP_DEBUG(rclcpp::get_logger("rm_motor_system"), "using can bus %s", bus_name.c_str());

        //create socket can sender and receiver
        this->can_sender = std::make_shared<driver::can::SocketCANSender>(bus_name);
        this->can_receiver = std::make_shared<driver::can::SocketCANReceiver>(bus_name);

        //check parameter "cmd_id"
        if (info.hardware_parameters.count("cmd_id") != 1) {
            RCLCPP_ERROR(rclcpp::get_logger("rm_motor_system"), "invalid cmd id definition in urdf");
            this->status_ = hardware_interface::status::UNKNOWN;
            return hardware_interface::return_type::ERROR;
        }
        this->cmd_id = std::stoi(info.hardware_parameters.at("cmd_id"), nullptr, 16);
        RCLCPP_DEBUG(rclcpp::get_logger("rm_motor_system"), "using cmd id %d", cmd_id);

        RCLCPP_DEBUG(rclcpp::get_logger("rm_motor_system"), "configure %d motors", info.joints.size());

        //foreach motor
        for (const auto &i: info.joints) {
            std::string motor_name = i.name;
            //check parameter "motor_id"
            if (i.parameters.count("motor_id") != 1) {
                RCLCPP_ERROR(rclcpp::get_logger("rm_motor_system"), "invalid motor id in urdf");
                this->status_ = hardware_interface::status::UNKNOWN;
                return hardware_interface::return_type::ERROR;
            }
            int motor_id = std::stoi(i.parameters.at("motor_id"));

            //check parameter "motor_type"
            if (i.parameters.count("motor_type") != 1) {
                RCLCPP_ERROR(rclcpp::get_logger("rm_motor_system"), "invalid motor type in urdf");
                this->status_ = hardware_interface::status::UNKNOWN;
                return hardware_interface::return_type::ERROR;
            }
            std::string motor_type = i.parameters.at("motor_type");

            //check parameter "update_rate"
            int update_rate = 1000;
            if (i.parameters.count("update_rate") == 1) {
                update_rate = std::stoi(i.parameters.at("update_rate"));
                RCLCPP_DEBUG(rclcpp::get_logger("rm_motor_system"),
                             "[motor name %s id %d type %s] using custom update rate %d", motor_name.c_str(), motor_id,
                             motor_type.c_str(), update_rate);
            }
            double threshold = 1.0f / (double)update_rate;
            if (threshold < 0.1f) threshold = 0.1f;
            if (threshold > 1.0f) threshold = 1.0f;

            //create new motor
            std::shared_ptr<utils::RMMotor> new_motor;

            //switch motor type
            if (motor_type == "m3508") {
                new_motor = std::make_shared<utils::RMMotor>(utils::M3508, motor_id);
            } else if (motor_type == "m2006") {
                new_motor = std::make_shared<utils::RMMotor>(utils::M2006, motor_id);
            } else if (motor_type == "M6020") {
                new_motor = std::make_shared<utils::RMMotor>(utils::M6020, motor_id);
            } else if (motor_type == "m3508_gearless") {
                new_motor = std::make_shared<utils::RMMotor>(utils::M3508_GEARLESS, motor_id);
            } else {
                RCLCPP_ERROR(rclcpp::get_logger("rm_motor_system"), "invalid motor type");
                this->status_ = hardware_interface::status::UNKNOWN;
                return hardware_interface::return_type::ERROR;
            }

            //check if motor id and cmd id is mismatched
            if (new_motor->cmd_id != this->cmd_id) {
                RCLCPP_ERROR(rclcpp::get_logger("rm_motor_system"), "motor cmd id mismatched");
                this->status_ = hardware_interface::status::UNKNOWN;
                return hardware_interface::return_type::ERROR;
            }

            //make motor, name, cmd and offline detector in pairs
            rm_motor_ctrl_t motor_ctrl;
            motor_ctrl.motor = new_motor;
            motor_ctrl.cmd = std::make_shared<double>(0);
            motor_ctrl.motor_name = motor_name;
            motor_ctrl.offlineDetector = std::make_shared<utils::OfflineDetector>(threshold);
            motor_ctrl.offline = std::make_shared<double>(0);

            //add to motors
            this->motors.emplace_back(motor_ctrl);

            RCLCPP_DEBUG(rclcpp::get_logger("rm_motor_system"), "new motor type %s cmd 0x%X fdb 0x%X",
                         motor_type.c_str(), new_motor->cmd_id, new_motor->feedback_id);
        }

        this->status_ = hardware_interface::status::CONFIGURED;
        RCLCPP_INFO(rclcpp::get_logger("rm_motor_system"), "configured");
        return hardware_interface::return_type::OK;
    }

    std::vector<hardware_interface::StateInterface> RMMotorSystem::export_state_interfaces() {

        //creat state interfaces
        std::vector<hardware_interface::StateInterface> state_interfaces;

        //foreach motor and add state interfaces
        for (const auto &i: this->motors) {
            auto motor_name = i.motor_name;
            auto motor_obj = i.motor;
            //foreach motor feedback data
            for (const auto &j: motor_obj->feedback_data) {
                auto feedback_name = j.first;
                auto feedback_data = j.second;
                state_interfaces.emplace_back(motor_name, feedback_name, feedback_data.get());
            }
            state_interfaces.emplace_back(motor_name, "offline", i.offline.get());
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> RMMotorSystem::export_command_interfaces() {

        //creat state interfaces
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        //foreach motor and add command interfaces
        for (const auto &i: this->motors) {
            auto motor_name = i.motor_name;
            auto cmd = i.cmd.get();
            command_interfaces.emplace_back(motor_name, "effort", cmd);
        }

        return command_interfaces;
    }

    hardware_interface::return_type RMMotorSystem::start() {

        RCLCPP_DEBUG(rclcpp::get_logger("rm_motor_system"), "starting");

        //foreach all motor
        for (const auto &i: this->motors) {
            //bind feedback can id
            this->can_receiver->bind(i.motor->feedback_id);
            //initialize cmd with zero
            *i.cmd = 0;
            //update offline detector
            i.offlineDetector->update(true);
        }

        this->status_ = hardware_interface::status::STARTED;

        RCLCPP_INFO(rclcpp::get_logger("rm_motor_system"), "started");

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type RMMotorSystem::stop() {

        RCLCPP_DEBUG(rclcpp::get_logger("rm_motor_system"), "stopping");

        this->status_ = hardware_interface::status::STOPPED;

        RCLCPP_DEBUG(rclcpp::get_logger("rm_motor_system"), "stopped");

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type RMMotorSystem::read() {

        RCLCPP_DEBUG(rclcpp::get_logger("rm_motor_system"), "reading");

        //foreach motor and read the corresponding can data
        for (const auto &i: this->motors) {

            struct can_frame can_recv_frame{};
            //read
            if (this->can_receiver->read(i.motor->feedback_id, &can_recv_frame)) {
                RCLCPP_DEBUG(rclcpp::get_logger("rm_motor_system"), "can frame read succ");

                //decode
                if (i.motor->feedback(can_recv_frame.data)) {
                    i.offlineDetector->update(true);
                } else {
                    RCLCPP_DEBUG(rclcpp::get_logger("rm_motor_system"), "[id 0x%X] data decode failed",
                                 i.motor->feedback_id);
                    i.offlineDetector->update(false);
                }
            } else {
                RCLCPP_DEBUG(rclcpp::get_logger("rm_motor_system"), "[id 0x%X] can read failed", i.motor->feedback_id);
                i.offlineDetector->update(false);
            }

            *i.offline = i.offlineDetector->offline;
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type RMMotorSystem::write() {

        RCLCPP_DEBUG(rclcpp::get_logger("rm_motor_system"), "writing");

        struct can_frame frame{};
        frame.can_dlc = 8;
        frame.can_id = this->cmd_id;

        //foreach motor and fill the can cmd
        for (const auto &i: this->motors) {
            //encode motor command
            i.motor->cmd(*i.cmd);

            //get motor id
            int id = i.motor->motor_id;
            if (id >= 5) id -= 4;

            //fill the cmd in a can frame
            frame.data[(id - 1) * 2 + 0] = i.motor->control_cmd[0];
            frame.data[(id - 1) * 2 + 1] = i.motor->control_cmd[1];

            //always clean the command to keep safe
            *i.cmd = 0;
        }

        //send the can frame
        if (this->can_sender->send(frame)) {
            RCLCPP_DEBUG(rclcpp::get_logger("rm_motor_system"), "can frame write succ");
            return hardware_interface::return_type::OK;
        } else {
            RCLCPP_DEBUG(rclcpp::get_logger("rm_motor_system"), "[id 0x%X dlc %d] can write failed", frame.can_id,
                         frame.can_dlc);
        }
        return hardware_interface::return_type::ERROR;
    }

}   //namespace gary_hardware


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(gary_hardware::RMMotorSystem, hardware_interface::SystemInterface)
