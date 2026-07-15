#include <vector>
#include <algorithm>

#include <yaml-cpp/yaml.h>

#include "trajectory.hpp"

struct Motor
{
    int SERVO_CHANNEL;
    float MIN_ANGLE;
    float MAX_ANGLE;

    float angle;
};

struct LegPart
{
    float length;
    float curve;
    float flange_rotation;
};

struct Leg
{
    int leg_num;
    std::vector<float> base_offset;
    float base_rotation;
    LegPart parts[3];
    Motor motors[3];
    Vec3 position;
};

enum class LegProfile
{
    STOP,
    LINEAR,
    SWING,
    RETURN
};

struct AxesProfile
{
    AxisProfile x;
    AxisProfile y;
    AxisProfile z;
};

struct LegGaitParams
{
    bool is_cycle;
    std::vector<float> timings;
    std::vector<Vec3> positions;
    std::vector<LegProfile> leg_profiles;
};

class LegGait
{
private:
    int num_sections;

    bool is_cycle;
    std::vector<float> timings;
    std::vector<TrajectoryFn> trajectories;
public:
    AxesProfile resolveProfile(LegProfile profile)
    {
        switch (profile) {
            case LegProfile::STOP:
                return { AxisProfile::CONSTANT, AxisProfile::CONSTANT, AxisProfile::CONSTANT};
            case LegProfile::LINEAR:
                return { AxisProfile::LINEAR, AxisProfile::LINEAR, AxisProfile::LINEAR};
            case LegProfile::SWING:
                return { AxisProfile::CONSTANT, AxisProfile::LINEAR, AxisProfile::SINUSOIDAL};
            case LegProfile::RETURN:
                return { AxisProfile::CONSTANT, AxisProfile::LINEAR, AxisProfile::CONSTANT};
            default:
                return { AxisProfile::CONSTANT, AxisProfile::CONSTANT, AxisProfile::CONSTANT};
        }
    }

    LegGait(LegGaitParams params)
    {
        num_sections = (params.timings.size() - 1);
        timings.resize(num_sections + 1);
        trajectories.resize(num_sections);

        is_cycle = params.is_cycle;
        timings = params.timings;

        TrajectoryParams t_params;
        AxesProfile profiles;

        for (int section = 0; section < num_sections; section++) {
            profiles = resolveProfile(params.leg_profiles[section]);
            t_params = {
                .start_position = params.positions[section],
                .end_position = params.positions[section + 1],
                .total_time = params.timings[section + 1] - params.timings[section],
                .x = profiles.x,
                .y = profiles.y,
                .z = profiles.z
            };
            trajectories[section] = makeTrajectory(t_params);
        }
    }

    Vec3 LegInterpolation(float t)
    {
        int section_idx;
        float section_start_time;

        while (t < 0) {
            t += timings[num_sections];
        }

        for (int section = 0; section < num_sections; section++) {
            if (t < timings[section + 1]) {
                section_idx = section;
                section_start_time = timings[section];
                break;
            }
        }

        return trajectories[section_idx](t - section_start_time);
    }
};

struct GaitParams
{
    std::vector<float> timings;
    std::vector<LegGait> leg_gaits;
};

class Gait
{
private:
    int num_legs;

    std::vector<float> timings;
    std::vector<LegGait> leg_gaits;
public:
    Gait(GaitParams params)
    {
        num_legs = params.leg_gaits.size();
        timings.resize(num_legs);
        leg_gaits.reserve(num_legs);

        timings = params.timings;
        leg_gaits = params.leg_gaits;
    }

    std::vector<Vec3> computeLegPositions(float t)
    {
        std::vector<Vec3> leg_positions(num_legs);
        
        for (int leg = 0; leg < num_legs; leg++) {
            leg_positions[leg] = leg_gaits[leg].LegInterpolation(t - timings[leg]);
        }

        return leg_positions;
    }
};

class Hexapod
{
public:

    Leg legs[6];
    float height;

    Hexapod(YAML::Node config) {

        YAML::Node legs_config = config["hexapod"]["legs"];

        std::string part_names[3] = {"coxa", "femur", "tibia"};

        for (int leg_num = 0; leg_num < 6; leg_num++)
        {
            Leg& leg = legs[leg_num];
            leg.leg_num = leg_num + 1;
            leg.base_offset = legs_config["base-offset"]
                ["leg" + std::to_string(leg_num + 1)].as<std::vector<float>>();
            leg.base_rotation = legs_config["base-rotation"]
                ["leg" + std::to_string(leg_num + 1)].as<float>();

            for (int part_num = 0; part_num < (sizeof(leg.parts)/sizeof(leg.parts[0])); part_num++)
            {
                LegPart& part = leg.parts[part_num];
                part.length = legs_config["length"][part_names[part_num]].as<float>();
                part.curve = legs_config["curve"][part_names[part_num]].as<float>();
                part.flange_rotation = legs_config["flange-rotation"][part_names[part_num]].as<float>();

            }

            for (int motor_num = 0; motor_num < (sizeof(leg.motors)/sizeof(leg.motors[0])); motor_num++)
            {
                Motor& motor = leg.motors[motor_num];
                motor.SERVO_CHANNEL = leg_num*(sizeof(leg.motors)/sizeof(leg.motors[0])) + motor_num;
                
                if (part_names[motor_num] == "coxa") {
                    motor.MAX_ANGLE = 45;
                    motor.MIN_ANGLE = -45;
                } else if (part_names[motor_num] == "femur") {
                    motor.MAX_ANGLE = 75;
                    motor.MIN_ANGLE = -90;
                } else {
                    motor.MAX_ANGLE = 45;
                    motor.MIN_ANGLE = -90;
                }
            }
        }

        height = config["hexapod"]["body"]["height"].as<float>();

    }

    void inverseKinematics(int leg_num)
    {

        Leg& leg = legs[leg_num];

        float L_c = leg.parts[0].length;
        float L_f = leg.parts[1].length;
        float L_t = leg.parts[2].length;
        float x = leg.position.x;
        float y = leg.position.y;
        float z = leg.position.z;
        float L_xy = sqrt(x*x + y*y);
        float l = L_xy - L_c;
        float L_lz = sqrt(l*l + z*z);
        float theta_c = atan(y/x);
        float theta_f = acos((L_lz*L_lz + L_f*L_f - L_t*L_t)/(2*L_lz*L_f)) - atan(z/l);
        float theta_t = acos((L_f*L_f + L_t*L_t - L_lz*L_lz)/(2*L_f*L_t)) - PI;

        leg.motors[0].angle = theta_c*180/PI;
        leg.motors[1].angle = theta_f*180/PI;
        leg.motors[2].angle = theta_t*180/PI;

    }

    void convertAngles(int leg_num)
    {

        Leg& leg = legs[leg_num];

        for (int motor = 0; motor < 3; motor++) {
            leg.motors[motor].angle = std::max(leg.motors[motor].MIN_ANGLE, std::min(leg.motors[motor].angle, leg.motors[motor].MAX_ANGLE));
        }

        leg.motors[0].angle = leg.motors[0].angle + 90;
        leg.motors[1].angle = leg.motors[1].angle + 90;
        leg.motors[2].angle = leg.motors[2].angle + 90 - leg.parts[2].curve - leg.parts[2].flange_rotation;

    }

};