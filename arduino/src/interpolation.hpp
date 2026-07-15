#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#include "stopwatch.hpp"

struct Trajectory
{
    float start_angle;
    float end_angle;
    unsigned long start_time_us;
    unsigned long duration_us;
    bool active;
};

struct Servo
{
    Stopwatch stopwatch;
    Trajectory trajectory;

    uint16_t US_MIN = 500;
    uint16_t US_MAX = 2500;
    float DEG_MIN = 0;
    float DEG_MAX = 180;
    uint8_t DEAD_BAND_US = 4;

    uint8_t channel;
    float angle;
    int32_t last_pulse_us;
};

float pulseToAngle(Servo& servo, int32_t pulse_us)
{
    if (servo.last_pulse_us < 0) return servo.trajectory.start_angle;
    float u = (float)(pulse_us - servo.US_MIN) / (float)(servo.US_MAX - servo.US_MIN);
    return servo.DEG_MIN + u * (servo.DEG_MAX - servo.DEG_MIN);
}

void startTrajectory(Servo& servo, float target_angle, unsigned long duration_us)
{
    servo.trajectory.start_angle = pulseToAngle(servo, servo.last_pulse_us);
    servo.trajectory.end_angle = target_angle;
    servo.trajectory.start_time_us = servo.stopwatch.elapsedTime();
    servo.trajectory.duration_us = duration_us;
    servo.trajectory.active = true;
}

float smoothStep(float u)
{
    u = constrain(u, 0.0, 1.0);
    return u * u * u * (u * (u * 6.0 - 15.0) + 10.0);
}

float angleToPulse(Servo& servo, float angle)
{
    float u = (angle - servo.DEG_MIN) / (servo.DEG_MAX - servo.DEG_MIN);
    return servo.US_MIN + u * (servo.US_MAX - servo.US_MIN);
}

void writeToServo(Adafruit_PWMServoDriver pwm, Servo& servo, float pulse_us)
{
    int32_t pulse_rounded_us = (int32_t)(pulse_us + 0.5);

    if (servo.channel == 0) {
        Serial.println(pulse_rounded_us);
    }

    if (servo.last_pulse_us >= 0 && abs(pulse_rounded_us - servo.last_pulse_us) < servo.DEAD_BAND_US) {
        return;
    }

    uint16_t ticks = (uint16_t)((pulse_rounded_us * 4096L) / 20000L);
    pwm.setPWM(servo.channel, 0, ticks);
    servo.last_pulse_us = pulse_rounded_us;
}

template <typename ServoArray, uint8_t num_servos>
void updateTrajectories(Adafruit_PWMServoDriver pwm, ServoArray (&servos)[num_servos])
{
    Stopwatch sw;
    
    for (int s = 0; s < num_servos; s++)
    {

        Servo& servo = servos[s];
        Trajectory& traj = servo.trajectory;

        if (servo.last_pulse_us != -1) {
            //Serial.println(sw.elapsedTime());
        }
        sw.restart();

        if (!traj.active) continue;

        unsigned long elapsed = servo.stopwatch.elapsedTime() - traj.start_time_us;
        float u = (float)elapsed / (float)traj.duration_us;

        float v = smoothStep(u);
        float angle = traj.start_angle + v * (traj.end_angle - traj.start_angle);

        writeToServo(pwm, servo, angleToPulse(servo, angle));

        if (u >= 1.0) {
            traj.active = false;
            float final_pulse = angleToPulse(servo, angle);
            uint16_t ticks = (uint16_t)((final_pulse * 4096L) / 20000L);
            pwm.setPWM(servo.channel, 0, ticks);
            servo.last_pulse_us = (int32_t)(final_pulse + 0.5);
        }
    }
}