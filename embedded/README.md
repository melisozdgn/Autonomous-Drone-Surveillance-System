# ADSS Embedded System

STM32F407 + FreeRTOS firmware for the Autonomous Drone Surveillance System.

## MCU
**STM32F407VGT6** — Cortex-M4 @ 168 MHz + FPU

## Module Map

| Module | Location | Description |
|---|---|---|
| Flight Controller | `flight_controller/Core/Src/main.c` | Entry point, HAL init, task creation |
| PID Controller | `flight_controller/Core/Src/pid_controller.c` | Altitude / Roll / Pitch / Yaw PIDs |
| Flight Task | `flight_controller/Core/Src/flight_task.c` | 500 Hz state machine + motor mix |
| IMU Driver | `flight_controller/Core/Src/imu_driver.c` | MPU-6050 I2C + complementary filter |
| Barometer | `flight_controller/Core/Src/baro_driver.c` | BMP280 I2C |
| GPS Driver | `flight_controller/Core/Src/gps_driver.c` | NEO-M8N NMEA parser |
| Motor Control | `flight_controller/Core/Src/motor_control.c` | ESC PWM (TIM1) |
| RF Comms | `flight_controller/Core/Src/comms_rf.c` | AES-256 telemetry downlink |
| Sensor Manager | `flight_controller/Core/Src/sensor_manager.c` | 200 Hz sensor fusion |
| Telemetry Task | `flight_controller/Core/Src/telemetry_task.c` | 10 Hz packet assembly |
| Watchdog | `flight_controller/Core/Src/watchdog.c` | HW IWDG + SW task monitor |
| FLIR Lepton | `sensor_drivers/thermal_camera/` | 160×120 thermal camera VoSPI |
| VL53L1X Laser | `sensor_drivers/laser_rangefinder/` | ToF distance (docking) |
| MEMS Mic Array | `sensor_drivers/mems_microphone/` | 4× PDM mic, I2S DMA |
| TDOA Acoustic | `acoustic_processing/Core/Src/` | GCC-PHAT + Bayesian filter |
| Power Manager | `power_management/Core/Src/` | BMS, Coulomb counting, INA219 |
| Docking Ctrl | `docking_controller/Core/Src/` | ArUco + VL53L1X precision land |

## FreeRTOS Task Summary

| Task | Priority | Period | Function |
|---|---|---|---|
| FlightControlTask | 5 (highest) | 2 ms (500 Hz) | PID + motor mix |
| SensorManagerTask | 4 | 5 ms (200 Hz) | IMU + baro + GPS fusion |
| CommsTask | 3 | 100 ms (10 Hz) | RF telemetry TX/RX |
| AcousticTask | 3 | 32 ms (31 Hz) | TDOA frame processing |
| PowerManagerTask | 2 | 100 ms (10 Hz) | BMS + Coulomb counting |
| TelemetryTask | 2 | 100 ms (10 Hz) | Packet assembly |
| WatchdogTask | 1 (lowest) | 1000 ms (1 Hz) | IWDG kick + task monitor |

## Toolchain
- ARM GCC: `arm-none-eabi-gcc >= 12.x`
- STM32CubeMX for HAL generation
- OpenOCD for flashing via ST-Link V2

## Flash
```bash
arm-none-eabi-gcc -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard \
  -Os -g -Wall -T STM32F407VGTx_FLASH.ld \
  $(find . -name "*.c") -o adss_firmware.elf

openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
  -c "program adss_firmware.elf verify reset exit"
```
