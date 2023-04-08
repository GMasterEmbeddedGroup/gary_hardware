#pragma once

#include <cstdint>
#include <map>
#include <memory>

namespace utils {


class LKTechMotor {
public:
    LKTechMotor(uint8_t _motor_id);

    bool cmd(double effort_set);
    bool feedback(const uint8_t fdb_data[8]);
    void reset_position();

    uint8_t motor_id;
    uint16_t cmd_id;
    uint16_t feedback_id;

    std::map<std::string, std::shared_ptr<double>> feedback_data;
    uint8_t control_cmd[2] {};

private:

    uint16_t max_ecd_original;
    int16_t max_dps_original;
    int16_t max_current_original;
    uint8_t max_temperature_original;

    double cmd_ratio;
    double position_ratio;
    double velocity_ratio;
    double effort_ratio;

    uint16_t last_ecd = 0;
    bool flag_first_data = true;
};

} //namespace utils