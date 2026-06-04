# Components & UART Connections

## Quick Reference Table

| USART  | Label           | ArduPilot Serial | Component                   |
|--------|-----------------|------------------|-----------------------------|
| USART2 | TELEM1          | SERIAL1          | Jetson Orin Nano            |
| USART3 | TELEM2          | SERIAL2          | Siyi MK32                   |
| USART4 | GPS             | SERIAL3          | mRo Classic M9N + IST8308   |
| USART8 | Additional UART | SERIAL4          | RPLidar C1                  |
| USART1 | Additional UART | SERIAL5          | Optical Flow PMW3901        |
| USART7 | Additional UART | SERIAL6          | ToF Lightware LW20/C        |

---

## 1. Jetson Orin Nano — TELEM1 / SERIAL1

**Purpose:** Companion computer running mission logic, YOLO, and MAVLink communication.

### Wiring
```
FC TELEM1 TX  →  Jetson /dev/ttyTHS1 RX
FC TELEM1 RX  →  Jetson /dev/ttyTHS1 TX
FC TELEM1 GND →  Jetson GND
```
> Jetson THS1 GPIO is **1.8V logic**. FC TELEM1 is **3.3V**. Use a logic level shifter (3.3V ↔ 1.8V) between the two, or confirm your FC's TELEM1 is 3.3V tolerant and Jetson's UART accepts 3.3V input safely.

### ArduPilot Parameters
| Parameter         | Value | Notes                          |
|-------------------|-------|--------------------------------|
| SERIAL1_PROTOCOL  | 2     | MAVLink 2                      |
| SERIAL1_BAUD      | 115   | 115200 baud                    |
| SERIAL1_OPTIONS   | 0     | Default                        |
| SYSID_MYGCS       | 1     | Match Jetson MAVLink sysid     |

---

## 2. Siyi MK32 — TELEM2 / SERIAL2

**Purpose:** RC + video telemetry link (air unit side).

### Wiring
```
FC TELEM2 TX  →  MK32 Air Unit RX
FC TELEM2 RX  →  MK32 Air Unit TX
FC TELEM2 GND →  MK32 Air Unit GND
```
> MK32 air unit is powered separately. Only TX/RX/GND needed from FC side.

### ArduPilot Parameters
| Parameter        | Value | Notes                   |
|------------------|-------|-------------------------|
| SERIAL2_PROTOCOL | 2     | MAVLink 2               |
| SERIAL2_BAUD     | 57    | 57600 baud (MK32 default) |

---

## 3. GPS — mRo Classic M9N + IST8308 — GPS / SERIAL3

**Docs:** [mRo Classic M9N](https://docs.mrobotics.io/gps/classic-m9n-ist8308.html)

**Purpose:** Primary GPS + compass.

### Wiring
The mRo M9N uses the standard ArduPilot GPS connector (JST-GH 6-pin on most FCs):
```
FC GPS Port TX  →  GPS RX
FC GPS Port RX  →  GPS TX
FC GPS Port GND →  GPS GND
FC GPS Port VCC →  GPS VCC (5V)
FC GPS SDA      →  GPS SDA  (I2C — IST8308 compass)
FC GPS SCL      →  GPS SCL  (I2C — IST8308 compass)
```
> The IST8308 compass communicates over I2C, which is included in the same GPS connector.

### ArduPilot Parameters
| Parameter        | Value | Notes                        |
|------------------|-------|------------------------------|
| SERIAL3_PROTOCOL | 5     | GPS                          |
| SERIAL3_BAUD     | 38    | 38400 (u-blox auto-baud)     |
| GPS_TYPE         | 1     | Auto-detect (u-blox M9N)     |
| GPS_GNSS_MODE    | 0     | Default (all constellations) |
| COMPASS_DEV_ID   | —     | Auto-detected IST8308 on I2C |
| COMPASS_USE      | 1     | Enable external compass      |
| COMPASS_EXTERN   | 1     | Mark as external             |

---

## 4. RPLidar C1 — Additional UART / SERIAL4

**Docs:** [Slamtec C1](https://www.slamtec.com/en/c1) · [ArduPilot RPLidar](https://ardupilot.org/copter/docs/common-rplidar-a2.html)

**Purpose:** 360° proximity / obstacle avoidance.

### Wiring
RPLidar C1 has a USB-C interface; connect via USB-UART adapter **or** use the UART pads directly on the module:
```
FC SERIAL4 TX  →  LiDAR RX
FC SERIAL4 RX  →  LiDAR TX
FC SERIAL4 GND →  LiDAR GND
5V             →  LiDAR VCC (motor + logic)
```
> The C1 motor power and logic are both 5V. Ensure the FC's spare UART port can source enough current, or power the LiDAR from a dedicated 5V BEC.

### ArduPilot Parameters
| Parameter        | Value | Notes                         |
|------------------|-------|-------------------------------|
| SERIAL4_PROTOCOL | 11    | Lidar360                      |
| SERIAL4_BAUD     | 115   | 115200                        |
| PRX1_TYPE        | 5     | RPLidar                       |
| PRX1_MIN_CM      | 20    | Min range 0.2 m               |
| PRX1_MAX_CM      | 1200  | Max range 12 m (C1 spec)      |
| PRX1_ORIENT      | 0     | Adjust if LiDAR is rotated    |
| AVOID_ENABLE     | 3     | Proximity + fence avoidance   |

---

## 5. Optical Flow PMW3901 — Additional UART / SERIAL5

**Docs:** [ArduPilot Optical Flow](https://ardupilot.org/copter/docs/common-optical-flow-sensor-setup.html) · [Holybro PMW3901](https://holybro.com/products/pmw3901-optical-flow-sensor)

**Purpose:** Low-altitude position hold without GPS.

### Wiring
Holybro PMW3901 UART version:
```
FC SERIAL5 TX  →  Sensor RX
FC SERIAL5 RX  →  Sensor TX
FC SERIAL5 GND →  Sensor GND
3.3V           →  Sensor VCC
```
> Mount facing **straight down**. Keep clear of propeller shadow. Minimum ~10 cm above ground for valid readings.

### ArduPilot Parameters
| Parameter        | Value | Notes                                      |
|------------------|-------|--------------------------------------------|
| SERIAL5_PROTOCOL | 18    | OpticalFlow                                |
| SERIAL5_BAUD     | 115   | 115200                                     |
| FLOW_TYPE        | 5     | PMW3901 / Cheerson CX-OF (UART)            |
| FLOW_FXSCALER    | 0     | Calibrate per mount height                 |
| FLOW_FYSCALER    | 0     | Calibrate per mount height                 |
| FLOW_ORIENT_YAW  | 0     | Degrees if sensor is rotated               |
| EK3_SRC1_VELXY   | 5     | Use OpticalFlow for XY velocity (no GPS)   |
| EK3_SRC1_POSXY   | 0     | No absolute XY position source             |

> Optical flow works best below 5 m. Pair with a downward-facing rangefinder (LW20/C on SERIAL6) for accurate altitude scaling.

---

## 6. ToF Rangefinder Lightware LW20/C — Additional UART / SERIAL6

**Docs:** [ArduPilot LW20](https://ardupilot.org/copter/docs/common-lightware-lw20-lidar.html) · [LW20 Manual](https://www.documents.lightware.co.za/LW20%20-%20LiDAR%20Manual%20-%20Rev%2012.pdf)

**Purpose:** Downward-facing altitude / terrain following. Also scales optical flow readings.

### Wiring
LW20/C supports UART and I2C. Using UART here:
```
FC SERIAL6 TX  →  LW20 RX
FC SERIAL6 RX  →  LW20 TX
FC SERIAL6 GND →  LW20 GND
5V             →  LW20 VCC
```
> LW20/C default baud is **115200**. Confirm with Lightware's USB configurator if the unit has been reconfigured.

### ArduPilot Parameters
| Parameter           | Value | Notes                                  |
|---------------------|-------|----------------------------------------|
| SERIAL6_PROTOCOL    | 9     | Lidar (rangefinder)                    |
| SERIAL6_BAUD        | 115   | 115200                                 |
| RNGFND1_TYPE        | 7     | LightWare Serial                       |
| RNGFND1_MIN_CM      | 5     | 5 cm minimum range                     |
| RNGFND1_MAX_CM      | 10000 | 100 m maximum range                    |
| RNGFND1_ORIENT      | 25    | Downward facing (MAV_SENSOR_ROTATION_PITCH_270) |
| RNGFND1_GNDCLEAR    | 15    | cm from sensor to ground at rest       |
| EK3_SRC1_POSZ       | 2     | Use rangefinder for Z (altitude)       |

---

## Notes

- All UART wiring is **cross-connected**: FC TX → Peripheral RX, FC RX → Peripheral TX.
- Verify voltage levels before connecting — FC UART pads are typically 3.3V; some peripherals require 5V or 1.8V.
- After changing `SERIAL_x_PROTOCOL`, **reboot** the FC for the change to take effect.
- Use Mission Planner → Full Parameter List or MAVProxy `param set` to apply parameters.
