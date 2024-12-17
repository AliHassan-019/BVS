#include <ezButton.h>
#include <AccelStepper.h>

// Motor pin definitions
#define motor1DirPin 3
#define motor1StepPin 5
#define motor1EnPin 6

#define switchPin 7
#define switchPin2 8


bool motor1Direction = true;


// Create AccelStepper object for motor
AccelStepper motor1(AccelStepper::DRIVER, motor1StepPin, motor1DirPin);

ezButton limitSwitch1(switchPin);
ezButton limitSwitch2(switchPin2);

// UART Buffer
#define BUFFER_SIZE 9
uint8_t rx_buffer[BUFFER_SIZE];
uint8_t buffer_index = 0; // Tracks the index of received bytes
uint16_t VP_Address;

// Movement parameters for motor 1

const int maxSpeed1 = 30;
const int acceleration1 = 5;
int speedMotor1 = 9; // Constant high speed for motor 1

void setup() {
  Serial.begin(115200);

  // Configure motor 1
  motor1.setMaxSpeed(maxSpeed1);
  motor1.setAcceleration(acceleration1);
  motor1.setSpeed(speedMotor1);

  pinMode(motor1EnPin, OUTPUT);
  digitalWrite(motor1EnPin, 0); // Initially disable motor

  // Configure the switches
  limitSwitch1.setDebounceTime(50);  //debounce time to 50 milliseconds
  limitSwitch2.setDebounceTime(50);  // debounce time to 50 milliseconds
  Serial.println("Setup Complete");

}

void loop() {
  // Update switch state
  limitSwitch1.loop();
  limitSwitch2.loop();

  // Handle UART data
  while (Serial.available()) {
    uint8_t byteReceived = Serial.read();
   // Serial.print("Received Byte: 0x");
    //Serial.println(byteReceived, HEX);

    // Add byte to buffer
    rx_buffer[buffer_index++] = byteReceived;

    // Check if buffer is full
    if (buffer_index == BUFFER_SIZE) {
      buffer_index = 0; // Reset buffer index

      // Debug: Print full buffer
      //Serial.print("Full Buffer: ");
      //for (int i = 0; i < BUFFER_SIZE; i++) {
      //  Serial.print("0x");
      //  Serial.print(rx_buffer[i], HEX);
     //   Serial.print(" ");
    //  }
      //Serial.println();

      // Check for VP_Address in all valid positions
      for (int i = 0; i <= BUFFER_SIZE - 2; i++) 
      {
        VP_Address = (rx_buffer[i] << 8) | rx_buffer[i + 1];
        // Process VP_Address if it matches a known value
        if (VP_Address == 0x1000) {
          handleVPAddress(VP_Address, rx_buffer[8]); // Pass address and command byte
        }
        else if (VP_Address == 0x2000) {
          handleVPAddress(VP_Address, rx_buffer[8]); // Pass address and command byte
        }
      }

      // Clear buffer for next command
      memset(rx_buffer, 0, BUFFER_SIZE);
    }
  }

  // Check if limit switches are pressed
  if (limitSwitch1.isPressed() || limitSwitch2.isPressed()) {
    Serial.println("Limit Switch Pressed: Disabling Motor");
    digitalWrite(motor1EnPin, 0); // Disable motor
    motor1.stop(); // Stop motor
    
  }

  // Run motor at constant speed
  motor1.runSpeed();
}

void handleVPAddress(uint16_t address, uint8_t command) {
  if (address == 0x1000) {
    switch (command) {
      case 0:
        Serial.println("Motor: Enable and Clockwise");
        digitalWrite(motor1EnPin, 1); // Enable motor
        motor1.setSpeed(speedMotor1); // Set speed clockwise
        motor1Direction = true;
        break;

      case 1:
        Serial.println("Motor: Enable and Anticlockwise");
        digitalWrite(motor1EnPin, 1); // Enable motor
        motor1.setSpeed(-speedMotor1); // Set speed anticlockwise
        motor1Direction = false;
        break;
    }
  } else if (address == 0x2000) {
    switch (command) {
      case 0:
        speedMotor1 = 9;
        motor1.setSpeed(motor1Direction ? speedMotor1 : -speedMotor1);
        Serial.println("Flow Rate: 50µL/min");
        break;

      case 1:
        speedMotor1 = 19;
        motor1.setSpeed(motor1Direction ? speedMotor1 : -speedMotor1);
        Serial.println("Flow Rate: 100µL/min");
        break;

      case 2:
        speedMotor1 = 28;
        motor1.setSpeed(motor1Direction ? speedMotor1 : -speedMotor1);
        Serial.println("Flow Rate: 150µL/min");
        break;
    }
  }
}
