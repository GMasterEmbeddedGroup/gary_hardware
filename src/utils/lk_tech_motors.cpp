#include "utils/lk_tech_motors.hpp"
#include <string>
#include <stdexcept>
#include <cmath>


using namespace utils;


LKTechMotor::LKTechMotor(uint8_t _motor_id) {

    this->motor_id = _motor_id;
    double dps2rads = 2.0f * M_PI / 360.0f;

    double gear_ratio;
    double torque_constant;
    double max_current;

    //calc cmd id and feedback id
    this->feedback_id = _motor_id + 0x140;
    this->cmd_id = _motor_id + 0x140;

    //set motor parameters
    this->max_ecd_original = 32767;
    this->max_dps_original = 32767;
    this->max_current_original = 1000;
    this->max_temperature_original = 125;
    gear_ratio = 1.0f;
    torque_constant = 1.4f;
    max_current = 33.0f;

    //calc cmd and feedback ratio
    this->cmd_ratio = this->max_current_original / max_current / torque_constant;
    this->position_ratio = 1.0f / gear_ratio / this->max_ecd_original * 2 * M_PI;
    this->velocity_ratio = 1.0f / gear_ratio * dps2rads;
    this->effort_ratio = 1.0f / (double) this->max_current_original * max_current * torque_constant;

    this->feedback_data["position"] = std::make_shared<double>(0);        // rad
    this->feedback_data["encoder"] = std::make_shared<double>(0);         // rad
    this->feedback_data["encoder_raw"] = std::make_shared<double>(0);
    this->feedback_data["velocity"] = std::make_shared<double>(0);        // rad/s
    this->feedback_data["rpm"] = std::make_shared<double>(0);             // rpm
    this->feedback_data["effort"] = std::make_shared<double>(0);          // N/m
    this->feedback_data["effort_raw"] = std::make_shared<double>(0);
    this->feedback_data["temperature"] = std::make_shared<double>(0);// C°

}


bool LKTechMotor::cmd(double effort_set) {

    //convert effort to motor cmd
    auto cmd = static_cast<int16_t>(effort_set * this->cmd_ratio);

    bool out_of_range = false;

    //check weather cmd is in range
    if (cmd > this->max_current_original) {
        cmd = this->max_current_original;
        out_of_range = true;
    } else if (cmd < -this->max_current_original) {
        cmd = static_cast<int16_t>(-this->max_current_original);
        out_of_range = true;
    }

    this->control_cmd[0] = cmd >> 8;
    this->control_cmd[1] = cmd & 0xFF;

    return out_of_range;
}


bool LKTechMotor::feedback(const uint8_t fdb_data[8]) {
    double dps2rads = 2.0f * M_PI / 360.0f;

    //decode data
    uint16_t _ecd = ((uint16_t) (fdb_data[7] << 8 | fdb_data[6]));
    auto _dps = static_cast<int16_t>((uint16_t) (fdb_data[5] << 8 | fdb_data[4]));
    auto _current = static_cast<int16_t>((uint16_t) (fdb_data[3] << 8 | fdb_data[2]));
    auto _temperature = static_cast<int8_t>(fdb_data[1]);

    //verify data
    if (_ecd > this->max_ecd_original || (_dps > this->max_dps_original || _dps < -this->max_dps_original) ||
        (_current > this->max_current_original || _current < -this->max_current_original) ||
        _temperature > this->max_temperature_original) {
        return false;
    }

    //get position
    //skip first encoder data
    if (this->flag_first_data) {
        this->last_ecd = _ecd;
        this->flag_first_data = false;
        return true;
    }
    int delta = _ecd - this->last_ecd;
    this->last_ecd = _ecd;
    if (delta < -this->max_ecd_original / 2) delta += this->max_ecd_original;
    if (delta > this->max_ecd_original / 2) delta -= this->max_ecd_original;
    *this->feedback_data["position"] += delta * this->position_ratio;

    //get ecd
    *this->feedback_data["encoder"] = static_cast<double>(_ecd) * this->position_ratio - M_PI;

    //get raw ecd
    *this->feedback_data["encoder_raw"] = static_cast<double>(_ecd) ;

    //get velocity
    *this->feedback_data["velocity"] = static_cast<double>(_dps) * this->velocity_ratio;

    //get rpm
    *this->feedback_data["rpm"] = static_cast<double>(_dps) * dps2rads;

    //get effort
    *this->feedback_data["effort"] = static_cast<double>(_current) * this->effort_ratio;

    //get effort_raw
    *this->feedback_data["effort_raw"] = static_cast<double>(_current);

    //get temperature
    *this->feedback_data["temperature"] = static_cast<double>(_temperature);

    return true;
}
