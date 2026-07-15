#include "pi_parse.hpp"
#include "interpolation.hpp"

#define PI_UPDATE_FREQ 10
#define PI_UPDATE_DELAY_US 1000000/PI_UPDATE_FREQ
#define NUM_SERVOS 9
#define FREQ 50
#define UPDATE_DELAY_US 1000000/FREQ

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

Servo servos[NUM_SERVOS];

Stopwatch updateSW;
Stopwatch sw;

const byte numChars = 32;
char receivedChars[numChars];
boolean newData = false;

uint8_t servo_channel = 0;
float command_angle = 90;

uint16_t counter = 0;

void setup() {
    Serial.begin(115200);
    Wire.begin();
    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(FREQ);

    delay(10);

    for (int s = 0; s < NUM_SERVOS; s++) {
        servos[s].stopwatch.start();
        servos[s].trajectory.active = false;
        servos[s].trajectory.start_angle = 90;
        servos[s].angle = 90;
        servos[s].last_pulse_us = -1;
        servos[s].channel = s;
    }

    updateSW.start();
}

void loop() {
    if (updateSW.elapsedTime() > UPDATE_DELAY_US) {
        if (servos[0].last_pulse_us != -1) {
            //Serial.println(updateSW.elapsedTime());
        }
        updateSW.restart();
        if (servos[0].last_pulse_us != -1) {
            //Serial.println(sw.elapsedTime());
        }
        if (servos[0].last_pulse_us != -1) {
            counter++;
            //Serial.println(counter);
            //Serial.println(sw.elapsedTime());
        }
        updateTrajectories(pwm, servos);
    }

    readCommands(receivedChars, newData);

    if (newData) {
        parseData(receivedChars, servo_channel, command_angle);
        if (servo_channel == 0) {
            if (servos[0].last_pulse_us == -1) {
                sw.start();
            }
            //Serial.println(command_angle);
            //Serial.println(sw.elapsedTime());
        }
        startTrajectory(servos[servo_channel], command_angle, PI_UPDATE_DELAY_US*(1.2 - 1.0/(FREQ/PI_UPDATE_FREQ)));
        newData = false;
    }
}