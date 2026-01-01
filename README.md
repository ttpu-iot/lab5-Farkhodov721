[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-22041afd0340ce965d47ae6ef1cefeee28c7c493a6346c4f15d667ab976d596c.svg)](https://classroom.github.com/a/P-L47F2W)
# IoT 2025 - Lab 5 Template

# Lab 5 - ESP32 + FreeRTOS + MQTT + Web

---

### Setup Configuration in this Wokwi Project template:

- RED LED - `D26`
- Green LED - `D27`
- Blue LED - `D14`
- Yellow LED - `D12`


- Button (Active high) - `D25`
- Light sensor (analog) - `D33`

- LCD I2C - SDA: `D21`
- LCD I2C - SCL: `D22`

- Servo Motor: `D5`

- Buzzer: `D32`

--------------------------------
**Wokwi Design screenshot**:

<img src="esp32_wokwi_setup.png" alt="ESP32 Pin Configuration" width="60%">


----------


--------------------------------
## Exercises (read carefully)

- You will use FreeRTOS tasks to run parallel activities.
- Use non-blocking code (no `delay()`), use `vTaskDelay()`).
- Use MQTT to publish/subscribe data.
- Use the I2C LCD to show status messages.
- 

--------------------------------
### Exercise 1 — Hardware I/O

- Create your own MQTT topics specific to your device (always must start with `ttpu/`)

- Create two FreeRTOS tasks:
  - Task 1: Control the servo motor to move between 0 and 180 degrees every 5 seconds at variable speed - speed must be set from the mqtt.
  - Task 2: Activate the buzzer to emit a sound for 1 second every 5 seconds - Frequency must be set from the mqtt.
  - Task 3: Read the button state and publish it to the MQTT broker when pressed.
  - Task 4: Read the light sensor every second and publish the value to the MQTT broker.
  - Task 5: Update the LCD display (line1: servo position, ligh sensor value, buzzer frequency; line2: message that came from MQTT) - update every 2 seconds.

* OPTIONAL: Implement simple web server interface to set and monitor the parameters through mqtt broker.  (BONUS points)

### Hints:

- Use following tutorial as reference to use FreeRTOS tasks on ESP32: [ESP32 with FreeRTOS (Arduino Framework)](https://randomnerdtutorials.com/esp32-freertos-arduino-tasks/)



- Use the `ESP32S2 AnalogWrite` library for controlling the servo motor and buzzer.

```cpp
#include <Arduino.h>
#include <Servo.h>

const int SERVO_PIN = 5;
const int BUTTON_PIN = 25;

Servo myServo = Servo();

// SETUP FUNCTION
void setup(void) 
{
  pinMode(SERVO_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  // Buzzer setup
  ledcSetup(2, buzzer_freq, 16);
  ledcAttachPin(BUZZER_PIN, 2);

  // Servo setup
  myServo.attach(SERVO_PIN);

  // initial position to 90 degrees
  myServo.write(SERVO_PIN, 90);
}

// LOOP FUNCTION
void loop(void) 
{
  // turn ON buzzer for 1 sec at 1000 Hz
  int buzzer_freq = 1000;
  ledcWriteTone(2, buzzer_freq);

  delay(1000); // wait for 1 second
  ledcWriteTone(2, 0); // turn OFF buzzer
}

```



