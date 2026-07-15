#include <chrono>

class Stopwatch
{

private:

    std::chrono::time_point<std::chrono::steady_clock> start_time;
    std::chrono::time_point<std::chrono::steady_clock> end_time;
    std::chrono::time_point<std::chrono::steady_clock> current_time;
    bool is_running;

public:

    void start() 
    {
        start_time = std::chrono::steady_clock::now();
        is_running = true;
    }

    void stop() 
    {
        if (is_running) {
            end_time = std::chrono::steady_clock::now();
            is_running = false;
        }
    }

    double milliseconds_elapsed()
    {
        if (is_running) {
            current_time = std::chrono::steady_clock::now();
        } else {
            current_time = end_time;
        }
        return std::chrono::duration<double, std::milli>(current_time - start_time).count();
    }

    int microseconds_elapsed()
    {
        if (is_running) {
            current_time = std::chrono::steady_clock::now();
        } else {
            current_time = end_time;
        }
        return std::chrono::duration<double, std::ratio<1,1000000>>(current_time - start_time).count();
    }

};