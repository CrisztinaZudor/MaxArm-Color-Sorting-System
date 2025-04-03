#include "PID.h"            // Include the PID controller library
#include "ESPMax.h"         // Include the library for ESPMax-specific functionalities
#include "Buzzer.h"         // Include the library for controlling a buzzer
#include "WonderCam.h"      // Include the library for the WonderCam
#include "SuctionNozzle.h"  // Include the library for controlling the suction nozzle
#include "ESP32PWMServo.h"  // Include the library for controlling PWM servos with ESP32

// Color tracking and sorting with the WonderCam

WonderCam cam;  // Create a camera object

arc::PID<double> x_pid(0.045, 0.0001, 0.0001);  // Set PID parameters for X-axis
arc::PID<double> y_pid(0.045, 0.0001, 0.0001);  // Set PID parameters for Y-axis

void setup() {
  cam.begin();                // Initialize the camera
  Buzzer_init();              // Initialize the buzzer
  ESPMax_init();              // Initialize ESPMax functionalities
  Nozzle_init();              // Initialize the suction nozzle
  PWMServo_init();            // Initialize PWM servo control
  Valve_on();                 // Turn on the valve (usually to enable suction or airflow)
  SetPWMServo(1, 1500, 1000); // Set the nozzle angle to 0
  Valve_off();                // Turn off the valve
  setBuzzer(100);             // Activate the buzzer for 100 milliseconds
  Serial.begin(115200);       // Start serial communication at 115200 baud rate
  Serial.println("start..."); // Print "start..." to the serial monitor
  cam.changeFunc(APPLICATION_COLORDETECT);  // Set the camera to color detection mode
}

void loop() {
  int i = 0;
  int angle_pwm = 0;
  float pos[3];  // Array to hold position coordinates
  float place[3];  // Array to hold placement coordinates
  pos[0] = 0;
  pos[1] = -120;
  pos[2] = 150;
  set_position(pos, 1500);  // Set initial position with a delay
  delay(1500);              // Delay to allow movements to complete

  while (true) {
    int color_x = 160;  // Default X coordinate for detected color
    int color_y = 120;  // Default Y coordinate for detected color
    cam.updateResult();  // Update camera results
    if (cam.anyColorDetected()) {  // Check if any color is detected
      WonderCamColorDetectResult p;
      // Check for specific color IDs and adjust positions accordingly
      if (cam.colorIdDetected(1)) {  // If color ID 1 is detected
        cam.colorId(1, &p);          // Get position data for color ID 1
        color_x = p.x; color_y = p.y;
        angle_pwm = 2100;  // Set PWM angle for this color
        place[0] = -120; place[1] = -140; place[2] = 85;  // Set placement coordinates

      } else if (cam.colorIdDetected(2)) {  // If color ID 2 is detected
        cam.colorId(2, &p);  // Get position data for color ID 2
        color_x = p.x; color_y = p.y;
        angle_pwm = 2300;  // Set PWM angle for this color
        place[0] = -120; place[1] = -80; place[2] = 85;  // Set placement coordinates

      } else if (cam.colorIdDetected(3)) {  // If color ID 3 is detected
        cam.colorId(3, &p);  // Get position data for color ID 3
        color_x = p.x; color_y = p.y;
        angle_pwm = 2500;  // Set PWM angle for this color
        place[0] = -120; place[1] = -20; place[2] = 85;  // Set placement coordinates
      }

      // Use PID control to adjust the arm's X and Y positions based on detected color coordinates
      if (abs(color_x - 160) < 15) {  // X-axis PID control
        color_x = 160;
      }
      x_pid.setTarget(160);
      x_pid.setInput(color_x);
      float dx = x_pid.getOutput();
      pos[0] -= dx;

      if (abs(color_y - 120) < 10) {  // Y-axis PID control
        color_y = 120;
      }
      y_pid.setTarget(120);
      y_pid.setInput(color_y);
      float dy = y_pid.getOutput();
      pos[1] -= dy;

      // Limit the mechanical arm's range of motion
      if (pos[0] > 100) pos[0] = 100;
      if (pos[0] < -100) pos[0] = -100;
      if (pos[1] > -60) pos[1] = -60;
      if (pos[1] < -240) pos[1] = -240;
      set_position(pos, 50);  // Drive the mechanical arm to the new position

      // Check if the position adjustments are minor enough to proceed with picking and placing the object
      if ((abs(dx) < 0.1) & (abs(dy) < 0.1)) {
        i++;
        if (i > 10) {
          i = 0;
          setBuzzer(100);  // Activate the buzzer for 100 milliseconds
          float d_x = pos[0] / 2.3;  // Calculate horizontal displacement
          float d_y = 68 - abs(d_x / 3);  // Calculate vertical displacement

          pos[0] += d_x; pos[1] -= d_y;
          set_position(pos, 1000);  // Move above the color block
          delay(1000);
          pos[2] = 85;
          set_position(pos, 600);  // Lower to pick up the color block
          Pump_on();  // Turn on the air pump
          delay(1000);
          pos[2] = 150;
          set_position(pos, 1000);  // Lift the mechanical arm
          delay(1000);
          pos[0] = place[0]; pos[1] = place[1];
          set_position(pos, 1200);  // Move above the placement area
          delay(1200);
          SetPWMServo(1, angle_pwm, 800);  // Set angle compensation
          delay(200);
          set_position(place, 1000);  // Move to the placement area
          delay(1000);
          Valve_on();  // Turn off the air pump and open the electromagnetic valve
          place[2] = 150;
          set_position(place, 1000);  // Lift the mechanical arm again
          delay(1000);
          Valve_off();  // Close the electromagnetic valve
          pos[0] = 0; pos[1] = -120; pos[2] = 150;
          set_position(pos, 1500);;  // Reset the mechanical arm
          SetPWMServo(1, 1500, 800); // Reset the nozzle angle
          delay(2000);
        }
      }
      delay(50);
    }
  }
}
