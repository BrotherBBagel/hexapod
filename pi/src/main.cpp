#include <cstdint>
#include <functional>

#include <yaml-cpp/yaml.h>

#include "stopwatch.hpp"
#include "pi_serial.hpp"
#include "hexapod.hpp"

#define UPDATE_RATE_HZ 10
#define UPDATE_DELAY_MS 1000/UPDATE_RATE_HZ

LegGaitParams params = {
    .is_cycle = true,
    .timings = {0, 1.5, 3.0},
    .positions = {
        {150,40,65},
        {150,-40,65},
        {150,40,65}
    },
    .leg_profiles = {
        LegProfile::SWING,
        LegProfile::LINEAR
    }
};

LegGait legGait(params);
std::vector<LegGait> leg_gaits(6, legGait);

GaitParams gaitParams = {
    .timings = {0, 1, 2, 3, 4, 5},
    .leg_gaits = leg_gaits
};

Gait hexGait(gaitParams);

void formatCommand(char* buffer, size_t buffer_size,int motor, float angle)
{
    snprintf(buffer, buffer_size, "<%d,%.1f>", motor, angle);
}

int main(int argc, char** argv)
{

    if (argc < 2) { 
        std::cerr << "Hexapod Config Path Not Provided\n"; 
        return 1; 
    }
    std::string hexapod_config_file = argv[1];
    Hexapod hexapod(YAML::LoadFile(hexapod_config_file));

    int fd = initiatePort();

    const uint8_t numChars = 64;
    char message[numChars];

    Stopwatch sw;
    Stopwatch gaitSW;

    float t;
    std::vector<Vec3> leg_positions;

    sw.start();
    gaitSW.start();
    while (gaitSW.milliseconds_elapsed() <= (params.timings[2]*1000))
    {
        if (sw.milliseconds_elapsed() > UPDATE_DELAY_MS) {
            sw.stop();
        
            t = gaitSW.milliseconds_elapsed()/1000;
            leg_positions = hexGait.computeLegPositions(t);
            for (int leg = 0; leg < 3; leg++) {
                hexapod.legs[leg].position = leg_positions[leg];
                hexapod.inverseKinematics(leg);
                hexapod.convertAngles(leg);
                for (int motor = 0; motor < 3; motor++) 
                {
                    formatCommand(message, sizeof(message), (3*leg) + motor, hexapod.legs[leg].motors[motor].angle);
                    if (leg == 0 && motor == 0) {
                        std::cout << message << std::endl;
                    }
                    writeToSerialPort(fd, message, strlen(message));
                }
            }

            sw.start();
        }

        if (((params.timings[2]*1000) - gaitSW.milliseconds_elapsed()) < UPDATE_DELAY_MS && params.is_cycle) {
            gaitSW.stop();
            gaitSW.start();
        }
    }
    gaitSW.stop();
    sw.stop();

    closeSerialPort(fd);
    return 0;
}