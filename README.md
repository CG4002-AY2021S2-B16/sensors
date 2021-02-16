# Sensors

Sensors is a repo containing all the related code for the "Dance Dance" wearable unit for the Bluno Beetle V1.1.

# Getting Started

## Component List

- Bluno Beetle V1.1
- GY-521 MPU6050 IMU Motion Sensor
- SEN-13723 MyoWare Muscle Sensor

## Prerequisites

- Arduino IDE
- pySerial
- Any other requirements needed from the [Arduino-Makefile](Arduino-Makefile/README.md)

## Instructions

- Modify the Makefiles based on the Arduino-Makefile README, but don't commit the changed configurations.

# Usage

- Run `make` in the `src/` directory to compile the code in `main.ino`
- Connect the device to the computer's ports
- run `make upload` to upload the code to the board.

## Contributing

Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

Please make sure to update tests as appropriate.
