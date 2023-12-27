// SensorManager.cpp
#include "SensorManager.h"



SensorManager::SensorManager() {

    // Initialize light sensor
    new VisionSensor(LIGHT_SENSOR_PIN, "ePuck_lightSensor", 0.1);

    // Initialize proximity sensors
    new ProximitySensor(PROX_SENSOR_FL_PIN, "ePuck_proxSensor3", 0.1, 1);
    new ProximitySensor(PROX_SENSOR_FR_PIN, "ePuck_proxSensor4", 0.1, 1);
    new ProximitySensor(PROX_SENSOR_L_PIN, "ePuck_proxSensor1", 0.1, 1);
    new ProximitySensor(PROX_SENSOR_R_PIN, "ePuck_proxSensor6", 0.1, 1);
    new ProximitySensor(PROX_SENSOR_RL_PIN, "ePuck_proxSensor8", 0.1, 1);
    new ProximitySensor(PROX_SENSOR_RR_PIN, "ePuck_proxSensor7", 0.1, 1);
    new ProximitySensor(PROX_SENSOR_DL_PIN, "ePuck_proxSensor2", 0.1, 1);
    new ProximitySensor(PROX_SENSOR_DR_PIN, "ePuck_proxSensor5", 0.1, 1);
    
    // Sensors pins setup
    pinMode(PROX_SENSOR_L_PIN, INPUT);
    pinMode(PROX_SENSOR_R_PIN, INPUT);
    pinMode(PROX_SENSOR_FL_PIN, INPUT);
    pinMode(PROX_SENSOR_FR_PIN, INPUT);
    pinMode(PROX_SENSOR_RL_PIN, INPUT);
    pinMode(PROX_SENSOR_RR_PIN, INPUT);
    pinMode(PROX_SENSOR_DL_PIN, INPUT);
    pinMode(PROX_SENSOR_DR_PIN, INPUT);
    pinMode(LIGHT_SENSOR_PIN, INPUT);
}

SensorManager::~SensorManager() {}

void SensorManager::getSensorValues() {
  sensorValues[sensors_FL] = analogRead(PROX_SENSOR_FL_PIN);
  sensorValues[sensors_FR] = analogRead(PROX_SENSOR_FR_PIN);
  sensorValues[sensors_L] = analogRead(PROX_SENSOR_L_PIN);
  sensorValues[sensors_R] = analogRead(PROX_SENSOR_R_PIN);
  sensorValues[sensors_RL] = analogRead(PROX_SENSOR_RL_PIN);
  sensorValues[sensors_RR] = analogRead(PROX_SENSOR_RR_PIN);
  sensorValues[sensors_DL] = analogRead(PROX_SENSOR_DL_PIN);
  sensorValues[sensors_DR] = analogRead(PROX_SENSOR_DR_PIN);
  sensorValues[sensors_light] = analogRead(LIGHT_SENSOR_PIN);
}