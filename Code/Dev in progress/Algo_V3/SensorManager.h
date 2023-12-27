// SensorManager.h
#ifndef SensorManager_h
#define SensorManager_h

#include <Arduino.h>

class SensorManager {

    public:

        SensorManager();
        ~SensorManager();

        // Sensors indexes
        const int sensors_FL = 0;
        const int sensors_FR = 1;
        const int sensors_L = 2;
        const int sensors_R = 3;
        const int sensors_RL = 4;
        const int sensors_RR = 5;
        const int sensors_DL = 6;
        const int sensors_DR = 7;
        const int sensors_light = 8;

        // Sensors values stored in an array
        int sensorValues[9];

        // Sensors functions
        void getSensorValues();

    private:

        // Sensors pins
        const int LIGHT_SENSOR_PIN = 14;
        const int PROX_SENSOR_L_PIN = 9;
        const int PROX_SENSOR_R_PIN = 15;
        const int PROX_SENSOR_FL_PIN = 16;
        const int PROX_SENSOR_FR_PIN = 17;
        const int PROX_SENSOR_RL_PIN = 6;
        const int PROX_SENSOR_RR_PIN = 12;
        const int PROX_SENSOR_DL_PIN = 18;
        const int PROX_SENSOR_DR_PIN = 19;



};

#endif