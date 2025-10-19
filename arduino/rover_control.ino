#include <Wire.h>
#include <Adafruit_MotorShield.h>

// Undefine constants that might be redefined to prevent warnings
#ifdef PCA9685_MODE1
#undef PCA9685_MODE1
#endif
#ifdef PCA9685_SUBADR1
#undef PCA9685_SUBADR1
#endif
#ifdef PCA9685_SUBADR2
#undef PCA9685_SUBADR2
#endif
#ifdef PCA9685_SUBADR3
#undef PCA9685_SUBADR3
#endif

#include <Adafruit_PWMServoDriver.h>


// Individual motor speeds (set by Raspberry Pi)
int motorSpeeds[6] = {0, 0, 0, 0, 0, 0}; // FL, FR, ML, MR, RL, RR


// Create motor shield objects
Adafruit_MotorShield AFMS1(0x60);  // Motor Shield 1
Adafruit_MotorShield AFMS2(0x61);  // Motor Shield 2

// Servo controller
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);



// Servo PWM range (for 0–180° movement)
#define SERVOMIN  150
#define SERVOMAX  600

// Function to convert degrees to PWM pulse length
int angleToPulse(int angle) {
  return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

// Motor pointers
Adafruit_DCMotor *rearRight   = AFMS1.getMotor(1); // M1
Adafruit_DCMotor *frontRight  = AFMS1.getMotor(2); // M2
Adafruit_DCMotor *middleLeft  = AFMS1.getMotor(4); // M4

Adafruit_DCMotor *middleRight = AFMS2.getMotor(1); // M1
Adafruit_DCMotor *frontLeft   = AFMS2.getMotor(4); // M4
Adafruit_DCMotor *rearLeft    = AFMS2.getMotor(3); // M3


void setup() {
  Serial.begin(9600);
  Serial.println("Rover control system ready");

  AFMS1.begin();  // Initialize both motor shields
  AFMS2.begin();

  pwm.begin();         // Initialize servo shield
  pwm.setPWMFreq(60);  // Standard 60Hz for servos


  // Center all servos on startup
  centerAllServos();
  stopAllMotors();
}



void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command.startsWith("MOTORS:")) {
      // New command for individual motor speeds: MOTORS:FL,FR,ML,MR,RL,RR
      String speeds = command.substring(7);
      int comma1 = speeds.indexOf(',');
      int comma2 = speeds.indexOf(',', comma1 + 1);
      int comma3 = speeds.indexOf(',', comma2 + 1);
      int comma4 = speeds.indexOf(',', comma3 + 1);
      int comma5 = speeds.indexOf(',', comma4 + 1);
      
      if (comma5 > 0) {
        motorSpeeds[0] = speeds.substring(0, comma1).toInt();  // FL
        motorSpeeds[1] = speeds.substring(comma1 + 1, comma2).toInt();  // FR
        motorSpeeds[2] = speeds.substring(comma2 + 1, comma3).toInt();  // ML
        motorSpeeds[3] = speeds.substring(comma3 + 1, comma4).toInt();  // MR
        motorSpeeds[4] = speeds.substring(comma4 + 1, comma5).toInt();  // RL
        motorSpeeds[5] = speeds.substring(comma5 + 1).toInt();  // RR
        
        setIndividualMotorSpeeds();
      }
    }
    else if (command.startsWith("STEER:")) {
      // Calculated steering angles from Raspberry Pi: STEER:FL,FR,RL,RR
      String angles = command.substring(6);
      int comma1 = angles.indexOf(',');
      int comma2 = angles.indexOf(',', comma1 + 1);
      int comma3 = angles.indexOf(',', comma2 + 1);
      
      if (comma3 > 0) {
        int frontLeftAngle = angles.substring(0, comma1).toInt();  // FL
        int frontRightAngle = angles.substring(comma1 + 1, comma2).toInt();  // FR
        int rearLeftAngle = angles.substring(comma2 + 1, comma3).toInt();  // RL
        int rearRightAngle = angles.substring(comma3 + 1).toInt();  // RR
        
        // Set calculated steering angles
        pwm.setPWM(0, 0, angleToPulse(constrain(frontLeftAngle, 0, 180)));   // Front Left (ch0)
        pwm.setPWM(12, 0, angleToPulse(constrain(frontRightAngle, 0, 180)));  // Front Right (ch12)
        pwm.setPWM(4, 0, angleToPulse(constrain(rearLeftAngle, 0, 180)));    // Rear Left (ch4)
        pwm.setPWM(8, 0, angleToPulse(constrain(rearRightAngle, 0, 180)));  // Rear Right (ch8)
      }
    }

    else if (command.startsWith("PAN:")) {
      int angle = command.substring(4).toInt();
      setServo(14, angle);  // Pan servo on channel 14
    }
    else if (command.startsWith("TILT:")) {
      int angle = command.substring(5).toInt();
      setServo(15, angle);  // Tilt servo on channel 15
    }
    else if (command == "STOP") {
      stopAllMotors();
    }

    else if (command == "CENTER") {
      centerAllServos();
    }
  }
}



void setIndividualMotorSpeeds() {
  // Set individual motor directions based on speed sign
  if (motorSpeeds[0] > 0) {
    frontLeft->run(BACKWARD);
  } else if (motorSpeeds[0] < 0) {
    frontLeft->run(FORWARD);
  } else {
    frontLeft->run(RELEASE);
  }
  
  if (motorSpeeds[1] > 0) {
    frontRight->run(FORWARD);
  } else if (motorSpeeds[1] < 0) {
    frontRight->run(BACKWARD);
  } else {
    frontRight->run(RELEASE);
  }
  
  if (motorSpeeds[2] > 0) {
    middleLeft->run(BACKWARD);
  } else if (motorSpeeds[2] < 0) {
    middleLeft->run(FORWARD);
  } else {
    middleLeft->run(RELEASE);
  }
  
  if (motorSpeeds[3] > 0) {
    middleRight->run(BACKWARD);
  } else if (motorSpeeds[3] < 0) {
    middleRight->run(FORWARD);
  } else {
    middleRight->run(RELEASE);
  }
  
  if (motorSpeeds[4] > 0) {
    rearLeft->run(FORWARD);
  } else if (motorSpeeds[4] < 0) {
    rearLeft->run(BACKWARD);
  } else {
    rearLeft->run(RELEASE);
  }
  
  if (motorSpeeds[5] > 0) {
    rearRight->run(BACKWARD);
  } else if (motorSpeeds[5] < 0) {
    rearRight->run(FORWARD);
  } else {
    rearRight->run(RELEASE);
  }
  
  // Set individual speeds
  frontLeft->setSpeed(abs(motorSpeeds[0]));
  frontRight->setSpeed(abs(motorSpeeds[1]));
  middleLeft->setSpeed(abs(motorSpeeds[2]));
  middleRight->setSpeed(abs(motorSpeeds[3]));
  rearLeft->setSpeed(abs(motorSpeeds[4]));
  rearRight->setSpeed(abs(motorSpeeds[5]));
}



void stopAllMotors() {
  rearRight->run(RELEASE);
  frontRight->run(RELEASE);
  middleLeft->run(RELEASE);
  middleRight->run(RELEASE);
  frontLeft->run(RELEASE);
  rearLeft->run(RELEASE);
}



void centerAllServos() {
  setServo(14, 90); // Pan (14)
  setServo(15, 90); // Tilt (15)
  // Center steering servos
  pwm.setPWM(0, 0, angleToPulse(65));   // Front Left (90-25)
  pwm.setPWM(12, 0, angleToPulse(76));  // Front Right (90-14)
  pwm.setPWM(4, 0, angleToPulse(90));   // Rear Left
  pwm.setPWM(8, 0, angleToPulse(90));   // Rear Right
}

void setServo(uint8_t channel, int angle) {
  int pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(channel, 0, pulse);
}

