#include <Arduino.h>
#include <Wire.h> // Assuming you're using I2C for IMU, include SPI otherwise
#include <Orientation.h>

Gyro yourGyro; // IMU interface, will be library-dependent

uint64_t thisLoopMicros = 0; // Stores microsecond timestamp for current loop
uint64_t lastOriUpdate = 0; // Stores mircosecond timestamp for last time orientation updated

Orientation ori; // Main orientation measurement
EulerAngles oriMeasure; // Quaternion converted to Euler Angles for maths etc.

void setup()
{
    yourGyro.init();
    yourGyro.calibrate(); // These two functions will be specific to your IMU/library choice
    
    thisLoopMicros = lastOriUpdate = micros(); // Set starting time after init/calibration
}

void loop()
{
    thisLoopMicros = micros(); // Get new microsecond timestamp for this loop
    
    if(yourGyro.dataReady()) // This will also be IMU/library specific
    {
        float dtOri = (float)(thisLoopMicros - lastOriUpdate) / 1000000.; // Finds elapsed microseconds since last update, converts to float, and converts to seconds
        lastOriUpdate = thisLoopMicros; // We have updated, set the new timestamp
        
        /*
        This is where the magic actually happens
        
        The order of your axis measurements (x, y, z) will depend on your sensor, your reference frame, and your IMU library of choice
        Swap & invert your gyro measurements so that .update() is called with (yaw, pitch, roll, dt) in that order
        
        All gyro measurements must be measured right-handed (positive = yaw left, pitch down, roll right) and coverted to radians/sec
        */
        
        ori.update(yourGyro.x(), yourGyro.y(), yourGyro.z(), dtOri); // '* DEG_TO_RAD' after all gyro functions if they return degrees/sec
        oriMeasure = ori.toEuler();
    }
    
    /*
    Orientation measurement can then be used as follows:
    ori.orientation  : Main quaternion storing current orientation
    oriMeasure.yaw   
    oriMeasure.pitch 
    oriMeasure.roll  : Euler angles converted from quaternion (radians)
    */
    
    // Example use
    Serial.print("Yaw: ");
    Serial.print(oriMeasure.yaw);
    Serial.print(", Pitch: ");
    Serial.print(oriMeasure.pitch);
    Serial.print(", Roll: ");
    Serial.println(oriMeasure.roll);
}