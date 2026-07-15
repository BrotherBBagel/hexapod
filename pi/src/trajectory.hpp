#include <functional>
#include <algorithm>

#define PI 3.1415926

struct Vec3
{
    float x, y, z;
};

enum class AxisProfile 
{
    CONSTANT,
    LINEAR,
    SINUSOIDAL,
    CUBIC
};

struct TrajectoryParams
{

    Vec3 start_position;
    Vec3 end_position;
    float total_time;
    AxisProfile x;
    AxisProfile y;
    AxisProfile z;

};

using TrajectoryFn = std::function<Vec3(float t)>;
using AxisFn = std::function<float(float start, float end, float u)>;

AxisFn interpolate(AxisProfile profile)
{

    switch (profile) {
        case AxisProfile::CONSTANT:
            return [](float start, float end, float u) -> float {
                return start;
            };
        case AxisProfile::LINEAR:
            return [](float start, float end, float u) -> float {
                return start + (end - start) * u;
            };
        case AxisProfile::SINUSOIDAL:
            return [](float start, float end, float u) -> float {
                return start - (start/2)*sinf(PI*u);
            };
        default:
            return [](float start, float end, float u) -> float {
                return start;
            };

    }
}

TrajectoryFn makeTrajectory(TrajectoryParams params)
{

    AxisFn xInterpolation = interpolate(params.x);
    AxisFn yInterpolation = interpolate(params.y);
    AxisFn zInterpolation = interpolate(params.z);

    return [params, xInterpolation, yInterpolation, zInterpolation](float t) -> Vec3 {
        float u = t / params.total_time;
        float x = xInterpolation(params.start_position.x, params.end_position.x, u);
        float y = yInterpolation(params.start_position.y, params.end_position.y, u);
        float z = zInterpolation(params.start_position.z, params.end_position.z, u);

        return Vec3 {.x = x, .y = y, .z = z};
    };
}