// 4 Channel "Micro RC" Receiver with 4 standard RC Servo Outputs (used as active suspension)
// ATMEL Mega 328P TQFP 32 soldered directly to the board, 8MHz external resonator, (tested on a Pro Micro 8MHz)
// MPU-6050 gyro / accelerometer


/* This code is still very quick and dirty!
 *  MPU-6050 code based on the work of Joop Brokking, no library required!
 *  8MHz is a bit on the weak side, servo lag may occure
 *  This simulation shows an active truck cabin suspension, using 4 servos
 */  

const float codeVersion = 0.1; // Software revision (see)

//
// =======================================================================================================
// BUILD OPTIONS (comment out unneeded options)
// =======================================================================================================
//

#define DEBUG // if not commented out, Serial.print() is active! For debugging only!!

//
// =======================================================================================================
// INCLUDE LIRBARIES
// =======================================================================================================
//

// Libraries
#include <Wire.h> // I2C library (for the MPU-6050 gyro /accelerometer)
#include <Servo.h>
//#include <PID_v1.h> // https://github.com/br3ttb/Arduino-PID-Library/

// Tabs (header files in sketch directory)
#include "MPU6050.h"
#include "helper.h"

//
// =======================================================================================================
// PIN ASSIGNMENTS & GLOBAL VARIABLES
// =======================================================================================================
//

// Create Servo objects
Servo servoFL;
Servo servoFR;
Servo servoRL;
Servo servoRR;

// Pin assignment
#define SERVO_FL_PIN 4 // Front left servo pin
#define SERVO_FR_PIN 5 // Front right servo pin
#define SERVO_RL_PIN 6 // Rear left servo pin
#define SERVO_RR_PIN 7 // Rear right servo pin

//
// =======================================================================================================
// MAIN ARDUINO SETUP (1x during startup)
// =======================================================================================================
//

void setup() {

#ifdef DEBUG
  Serial.begin(115200);
  delay(3000);
#endif


  // Servo pins
  servoFL.attach(SERVO_FL_PIN);
  servoFR.attach(SERVO_FR_PIN);
  servoRL.attach(SERVO_RL_PIN);
  servoRR.attach(SERVO_RR_PIN);

  // MPU 6050 accelerometer / gyro setup
  setupMpu6050();

  // PID controller setup
  //setupPid();

}

//
// =======================================================================================================
// WRITE SERVO POSITIONS
// =======================================================================================================
//

void writeServos() {

    servoFL.write(constrain((90 + along / 20 - across / 20 - vertical / 20), 45, 135)); // 45 - 135° (front left)
    servoFR.write(constrain((90 - along / 20 - across / 20 + vertical / 20), 45, 135));; // 45 - 135° (front right)
    servoRL.write(constrain((90 - along / 20 - across / 20 - vertical / 20), 45, 135)); // 45 - 135° (rear left)
    servoRR.write(constrain((90 + along / 20 - across / 20 + vertical / 20), 45, 135)); // 45 - 135° (rear right)  
  
}

//
// =======================================================================================================
// BALANCING CALCULATIONS
// =======================================================================================================
//
/*
void balancing() {

  // Read sensor data
  readMpu6050Data();

  angleMeasured = angle_pitch - tiltCalibration;

  // Read speed pot with 0.2s fader
  static unsigned long lastPot;
  if (millis() - lastPot >= 40) { // 40ms
    lastPot = millis();
    speedPot = (speedPot * 4 + data.axis3) / 5; // 1:5
  }

  // PID Parameters (Test)
  double speedKp = 0.9, speedKi = 0.03, speedKd = 0.0;
  double angleKp = data.pot1 / 8.0, angleKi = 25.0, angleKd = 0.12; // /You need to connect a potentiometer to the transmitter analog input A6

  // PID Parameters (Working)
  //double speedKp = 0.9, speedKi = 0.03, speedKd = 0.0;
  //double angleKp = data.pot1 / 8.0, angleKi = 25.0, angleKd = 0.12; // You need to connect a potentiometer to the transmitter analog input A6

  // Speed PID controller (important to protect the robot from falling over at full motor rpm!)
  speedTarget = ((float)speedPot - 50.0) / 1.51; // (100 - 50) / 1.51 = Range of about +/- 33 (same as in setupPid() !)
  speedMeasured = speedAveraged * 1.3; //angleOutput; // 43 / 33 = 1.3
  speedPid.SetTunings(speedKp, speedKi, speedKd);
  speedPid.Compute();

  // Angle PID controller
  angleTarget = speedOutput / -8.25; // 33.0 (from above) / 8.25 = Range of about +/- 4.0° tilt angle
  //  angleTarget = (speedPot - 50) / -12.5; // 50 / 12.5 = Range of about +/- 4.0° tilt angle
  anglePid.SetTunings(angleKp, angleKi, angleKd);
  anglePid.Compute();

  // Send the calculated values to the motors
  driveMotorsBalancing();

  // Display PID variables on the transmitter OLED (for debugging only, comment out the corresponding variables in checkBattery() in this case)
  //loopDuration(); // compute the loop time
  //payload.vcc = loopTime;
  //payload.vcc = angleMeasured;
  //payload.vcc = speedTarget;
  //payload.batteryVoltage = speedOutput;
  //payload.batteryVoltage = speedMeasured;
}*/

//
// =======================================================================================================
// MRSC (MICRO RC STABILITY CONTROL) CALCULATIONS
// =======================================================================================================
// For cars with stability control (steering overlay depending on gyro yaw rate)
/*
void mrsc() {

  // Read sensor data
  readMpu6050Data();

  // If the MRSC gain is a fixed value, read it!
#ifdef MRSC_FIXED
  data.pot1 = mrscGain;
#endif

  // Compute steering compensation overlay
  int turnRateSetPoint = data.axis1 - 50;  // turnRateSetPoint = steering angle (0 to 100) - 50 = -50 to 50
  int turnRateMeasured = yaw_rate * abs(data.axis3 - 50); // degrees/s * speed
  int steeringAngle = turnRateSetPoint + (turnRateMeasured * data.pot1 / 100);  // Compensation depending on the pot value

  steeringAngle = constrain (steeringAngle, -50, 50); // range = -50 to 50

  // Control steering servo (MRSC mode only)
  servo1.write(map(steeringAngle, 50, -50, lim1L, lim1R) ); // 45 - 135°

  // Control motor 2 (steering, not on "High Power" board type)
  if (!HP) {
    Motor2.drive((steeringAngle + 50), 0, steeringTorque, 0, false); // The steering motor (if the original steering motor is reused instead of a servo)
  }

  // Control motors
  driveMotorsCar();

}*/

//
// =======================================================================================================
// SUSPENSION CALCULATIONS
// =======================================================================================================
// 

void calculateSuspension() {
  
  // Read sensor data
  readMpu6050Data();

}

//
// =======================================================================================================
// MAIN LOOP
// =======================================================================================================
//

void loop() {

  // Calculate suspension
  calculateSuspension();

  // Write the servo positions
  writeServos();
  
}
