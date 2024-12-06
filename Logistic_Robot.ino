#include <Servo.h> // Include the Servo library

// Servo Definitions
Servo servo1; // Servo for "U" and "D" 180edg 
Servo servo2; // Servo for "V" and "B" 180deg
Servo servo3; // Servo for "C" and "O" 180deg (gripper)
Servo servo4; // Servo for "a" and "c" 360deg (base servo)

int angle1 = 90;          // Initial position for servo1
int angle2 = 90;          // Initial position for servo2
int rotationSignal4 = 90; // Neutral position for servo4 (no rotation)

unsigned long lastTime1 = 0; // Last update time for servo1
unsigned long lastTime2 = 0; // Last update time for servo2
unsigned long lastTime4 = 0; // Last update time for servo4
const unsigned long updateInterval = 20; // Delay for smooth rotation (milliseconds)
const unsigned long delayTime4 = 10;     // Delay for slower rotation for servo4 (milliseconds)

// Pin Definitions for Left Motor (A Part)
#define IN1 6  
#define IN2 7  

// Pin Definitions for Right Motor (B Part)
#define IN3 8  
#define IN4 9  

// Pin Definitions for IR Sensors
#define IR_LEFT 26    // Left IR sensor
#define IR_RIGHT 30   // Right IR sensor
#define IR_MIDDLEL 28  // Middle Left IR sensor
#define IR_MIDDLER 32  // Middle Right IR sensor

// Motor Speed Definitions (PWM values)
#define HIGH_SPEED 200  // Speed for manual control movement
#define MED_SPEED 155    // Speed for line following when turning
#define LOW_SPEED 60    // Speed for line following on straight line

// Line-following mode enabling
bool lineFollowingMode = false;

void setup() {
  Serial.begin(9600);     // Serial Monitor communication
  Serial1.begin(9600);    // Communication with bluetooth
  Serial.println("Bluetooth Ready. Waiting for commands...");   // Check the status of bluetooth in serial monitor

  // Define Servos
  servo1.attach(11);      
  servo2.attach(12);      
  servo3.attach(10);      
  servo4.attach(13);      

  servo1.write(angle1);
  servo2.write(angle2);
  servo3.write(0);
  servo4.write(rotationSignal4);

  // Define L298N pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Define IR sensor pins
  pinMode(IR_LEFT, INPUT);    //Left IR Sensor
  pinMode(IR_RIGHT, INPUT);   //Right IR Sensor
  pinMode(IR_MIDDLEL, INPUT);   //Middle Left IR Sensor
  pinMode(IR_MIDDLER, INPUT);   //Middle Right IR Sensor

  // Stop the motors in beginning
  stopMotors();
}

void loop() {
  static bool rotateClockwise1 = false;       // Clockwise rotation flag for servo1
  static bool rotateCounterClockwise1 = false; // Counterclockwise rotation flag for servo1
  static bool rotateClockwise2 = false;       // Clockwise rotation flag for servo2
  static bool rotateCounterClockwise2 = false; // Counterclockwise rotation flag for servo2

   // Check for incoming Bluetooth commands and operation
  if (Serial1.available()) {
    char command = Serial1.read(); // Read command from bluetooth
    Serial.print("Command received: ");
    Serial.println(command);

    // Handle commands for servo1
    if (command == 'U') {
      rotateClockwise1 = true;
      rotateCounterClockwise1 = false;
    } 
    else if (command == 'D') {
      rotateCounterClockwise1 = true;
      rotateClockwise1 = false;
    } 
    else {
      rotateClockwise1 = false;
      rotateCounterClockwise1 = false;
    }

    // Handle commands for servo2
    if (command == 'V') {
      rotateClockwise2 = true;
      rotateCounterClockwise2 = false;
    } 
    else if (command == 'B') {
      rotateCounterClockwise2 = true;
      rotateClockwise2 = false;
    } 
    else {
      rotateClockwise2 = false;
      rotateCounterClockwise2 = false;
    }

    // Handle commands for servo3
    if (command == 'C') {
      servo3.write(130); // Rotate servo3 to 130 degrees
      Serial.println("Servo3 rotated to 130 degrees");
    } else if (command == 'O') {
      servo3.write(0);   // Rotate servo3 to 0 degrees
      Serial.println("Servo3 rotated to 0 degrees");
    }

    // Handle commands for servo4
    if (command == 'a') {
      rotationSignal4 = 86; // Very slow counterclockwise rotation
      Serial.println("Servo4 rotating counterclockwise slowly...");
    } else if (command == 'c') {
      rotationSignal4 = 98; // Very slow clockwise rotation
      Serial.println("Servo4 rotating clockwise slowly...");
    } else {
      rotationSignal4 = 90; // Neutral signal stops servo4
      Serial.println("Stopping Servo4...");
    }

    //Movement control of the robot
    switch (command) {
      case 'u': // Move forward
        Serial.println("Moving Forward");
        lineFollowingMode = false; // Exit line-following mode
        moveForward(HIGH_SPEED);
        break;
      case 'd': // Move backward
        Serial.println("Moving Backward");
        lineFollowingMode = false; // Exit line-following mode
        moveBackward(HIGH_SPEED);
        break;
      case 'l': // Turn left
        Serial.println("Turning Left");
        lineFollowingMode = false; // Exit line-following mode
        turnLeft(HIGH_SPEED);
        break;
      case 'r': // Turn right
        Serial.println("Turning Right");
        lineFollowingMode = false; // Exit line-following mode
        turnRight(HIGH_SPEED);
        break;
      case 'A': // Activate line-following mode
        Serial.println("Line Following Mode Activated");
        lineFollowingMode = true;
        break;
      default: 
        Serial.println("Stopping Motors");
        lineFollowingMode = false; // Exit line-following mode
        stopMotors();
        break;
    }
  }

  // Rotate servo1
  if (millis() - lastTime1 > updateInterval) {
    if (rotateClockwise1 && angle1 < 180) {
      angle1++;
      servo1.write(angle1);
      Serial.println("Servo1 rotating clockwise...");
    } else if (rotateCounterClockwise1 && angle1 > 0) {
      angle1--;
      servo1.write(angle1);
      Serial.println("Servo1 rotating counterclockwise...");
    }
    lastTime1 = millis();
  }

  // Rotate servo2
  if (millis() - lastTime2 > updateInterval) {
    if (rotateClockwise2 && angle2 < 180) {
      angle2++;
      servo2.write(angle2);
      Serial.println("Servo2 rotating clockwise...");
    } else if (rotateCounterClockwise2 && angle2 > 0) {
      angle2--;
      servo2.write(angle2);
      Serial.println("Servo2 rotating counterclockwise...");
    }
    lastTime2 = millis();
  }

  // Control servo4 with a slower response
  if (millis() - lastTime4 > delayTime4) {
    servo4.write(rotationSignal4); // Send the current signal to servo4
    lastTime4 = millis();
  }

  // Line-following mode
  if (lineFollowingMode) {
    followLine();
  }
}

// Line-following function
void followLine() {
  bool leftSensor = digitalRead(IR_LEFT);
  bool rightSensor = digitalRead(IR_RIGHT);
  bool middleLSensor = digitalRead(IR_MIDDLEL);
  bool middleRSensor = digitalRead(IR_MIDDLER);

  if (middleLSensor == 0 && middleRSensor == 0) {
    // Middle Left and Middle Right sensor on the white area
    Serial.println("Moving Forward");
    moveForward(LOW_SPEED);
  } 
  else if (middleLSensor == 0 && middleRSensor == 0 && leftSensor == 0 && rightSensor == 0) {
    // All sensor on the white area
    Serial.println("Moving Forward");
    moveForward(LOW_SPEED);
  } 
  else if (middleLSensor == 1 && middleRSensor == 0 && leftSensor == 0 && rightSensor == 0) {
    // Middle left sensors on the line
    Serial.println("Adjusting Left");
    turnLeft(MED_SPEED);
  } 
  else if (middleLSensor == 0 && middleRSensor == 1 && leftSensor == 0 && rightSensor == 0) {
    // Middle right sensors on the line
    Serial.println("Adjusting Right");
    turnRight(MED_SPEED);
  } 
  else if (leftSensor == 1) {
    // Left turn ahead, keep moving until next condition is met
    Serial.println("Gradual Left Turn Detected");
    gradualTurnLeft();
  } 
   else if (middleLSensor == 1 && leftSensor == 1) {
    // Left turn ahead, keep moving until next condition is met
    Serial.println("Gradual Left Turn Detected");
    gradualTurnLeft();
  } 
  else if (middleRSensor == 1 && leftSensor == 1) {
    // Left turn ahead, keep moving until next condition is met
    Serial.println("Gradual Left Turn Detected");
    gradualTurnLeft();
  } 
  else if (middleLSensor == 1 && middleRSensor == 1 && leftSensor == 1) {
    // Left turn ahead, keep moving until next condition is met
    Serial.println("Gradual Left Turn Detected");
    gradualTurnLeft();
  } 
  else if (rightSensor == 1) {
    // Right turn ahead, keep moving until next condition is met
    Serial.println("Gradual Right Turn Detected");
    gradualTurnRight();
  }
  else if (middleRSensor == 1 && rightSensor == 1) {
    // Right turn ahead, keep moving until next condition is met
    Serial.println("Gradual Right Turn Detected");
    gradualTurnRight();
  }
  else if (middleLSensor == 1 && rightSensor == 1) {
    // Right turn ahead, keep moving until next condition is met
    Serial.println("Gradual Right Turn Detected");
    gradualTurnRight();
  }
  else if (middleRSensor == 1 && middleLSensor == 1 && rightSensor == 1) {
    // Right turn ahead, keep moving until next condition is met
    Serial.println("Gradual Right Turn Detected");
    gradualTurnRight();
  }
  else if (middleLSensor == 1 && middleRSensor == 1) {
    // All Middle sensors detect black
    Serial.println("All Middle Sensors Black - Stopping");
    stopMotors();
  } 
  else if (middleLSensor == 1 && middleRSensor == 1 && leftSensor == 1 && rightSensor == 1) {
    // All sensors detect black
    Serial.println("All Sensors Black - Stopping");
    stopMotors();
  }
  else {
    Serial.println("Line Lost - Stopping");
    stopMotors();
  }
}

// Gradual left turn
void gradualTurnLeft() {
  moveLeftMotorBackward(MED_SPEED / 2);  
  moveRightMotorForward(MED_SPEED);    
  delay(125);                           
  while (digitalRead(IR_MIDDLEL) == 0) { // Continue moving until middle left sensor detects the line
    turnLeft(MED_SPEED);
  }
  stopMotors();
}

// Gradual right turn
void gradualTurnRight() {
  moveLeftMotorForward(MED_SPEED);     
  moveRightMotorBackward(MED_SPEED / 2); 
  delay(125);                            
  while (digitalRead(IR_MIDDLER) == 0) {  // Continue moving until middle right sensor detects the line
    turnRight(MED_SPEED);
  }
  stopMotors();
}

// Left Motor Control (A Part)
void moveLeftMotorForward(int speed) {
  analogWrite(IN1, speed);
  analogWrite(IN2, 0);
}

void moveLeftMotorBackward(int speed) {
  analogWrite(IN1, 0);
  analogWrite(IN2, speed);
}

void stopLeftMotor() {
  analogWrite(IN1, 0);
  analogWrite(IN2, 0);
}

// Right Motor Control (B Part)
void moveRightMotorForward(int speed) {
  analogWrite(IN3, speed);
  analogWrite(IN4, 0);
}

void moveRightMotorBackward(int speed) {
  analogWrite(IN3, 0);
  analogWrite(IN4, speed);
}

void stopRightMotor() {
  analogWrite(IN3, 0);
  analogWrite(IN4, 0);
}

// Combined Movement Functions with Speed Control
void moveForward(int speed) {
  moveLeftMotorForward(speed);
  moveRightMotorForward(speed);
}

void moveBackward(int speed) {
  moveLeftMotorBackward(speed);
  moveRightMotorBackward(speed);
}

void turnLeft(int speed) {
  moveLeftMotorBackward(speed);
  moveRightMotorForward(speed);
}

void turnRight(int speed) {
  moveLeftMotorForward(speed);
  moveRightMotorBackward(speed);
}

void stopMotors() {
  stopLeftMotor();
  stopRightMotor();
}
