class Stopwatch {
private:
    unsigned long start_time;
    unsigned long end_time;
    unsigned long current_time;
    bool is_running;
public:

    void start()
    {
        if(!is_running) {
            start_time = micros();
            is_running = true;
        }
    }

    void stop()
    {
        if (is_running) {
            end_time = micros();
            is_running = false;
        }
    }

    unsigned long startToEndTime()
    {
        if (!is_running) {
            return end_time - start_time;
        } else {
            return 0;
        }
    }

    unsigned long elapsedTime()
    {
        if (is_running) {
            current_time = micros();
            return current_time - start_time;
        } else {
            return 0;
        }
    }

    void restart()
    {
        stop();
        start();
    }

};