#include <Arduino.h>

void readCommands(char* receivedChars, bool& newData) {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();
        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                //Serial.print(rc);
            }
            else {
                receivedChars[ndx] = '\0';
                recvInProgress = false;
                ndx = 0;
                newData = true;
                //Serial.println(rc);
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

void parseData(char* receivedChars, uint8_t& servo_channel, float& command_angle) {

    char * strtokIndx;

    strtokIndx = strtok(receivedChars, ",");
    servo_channel = atoi(strtokIndx);

    strtokIndx = strtok(NULL, ","); 
    command_angle = atof(strtokIndx);

}