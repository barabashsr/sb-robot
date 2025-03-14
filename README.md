# sb-robot
Self balancing robot on ESP32 and micro ROS
| Component | Function | GPIVzO Pin | Notes |
|-----------|----------|----------|--------|
| **L298N Driver** | Motor A IN1 | GPIO 27 | Motor 1 direction |
| | Motor A IN2 | GPIO 26 | Motor 1 direction |
| | Motor A Enable | GPIO 25 | Motor 1 PWM |
| | Motor B IN1 | GPIO 33 | Motor 2 direction |
| | Motor B IN2 | GPIO 32 | Motor 2 direction |
| | Motor B Enable | GPIO 14 | Motor 2 PWM |
| **Encoders** | Encoder A Ch1 | GPIO 34 | Input only pin |
| | Encoder A Ch2 | GPIO 35 | Input only pin |
| | Encoder B Ch1 | GPIO 4 | Input only pin |
| | Encoder B Ch2 | GPIO 5 | Input only pin |
| **OLED Display** | SDA | GPIO 21 | I2C Bus 1 |
| | SCL | GPIO 22 | I2C Bus 1 |
| **BNO055** | SDA | GPIO 16 | I2C Bus 2 |
| | SCL | GPIO 17 | I2C Bus 2 |

| Component | Function | GPIO Pin | Notes |
|-----------|----------|----------|-------|
| **[L298N Driver](pplx://action/followup)** | Motor A IN1 | GPIO 27 | Motor 1 direction |
| | Motor A IN2 | GPIO 26 | Motor 1 direction |
| | Motor A Enable | GPIO 25 | Motor 1 PWM |
| | Motor B IN1 | GPIO 33 | Motor 2 direction |
| | Motor B IN2 | GPIO 32 | Motor 2 direction |
| | Motor B Enable | GPIO 14 | Motor 2 PWM |
| **[Encoders](pplx://action/followup)** | Encoder A Ch1 | GPIO 34 | Input only pin |
| | Encoder A Ch2 | GPIO 35 | Input only pin |
| | Encoder B Ch1 | GPIO 4 | Input only pin |
| | Encoder B Ch2 | GPIO 5 | Input only pin |
| **[Menu Encoder](pplx://action/followup)** | CLK | GPIO 18 | Input pin |
| | DT | GPIO 19 | Input pin |
| | SW | GPIO 23 | Button input |
| **[OLED Display](pplx://action/followup)** | SDA | GPIO 21 | I2C Bus 1 |
| | SCL | GPIO 22 | I2C Bus 1 |
| **[BNO055](pplx://action/followup)** | SDA | GPIO 19 | I2C Bus 2 |
| | SCL | GPIO 18 | I2C Bus 2 |
