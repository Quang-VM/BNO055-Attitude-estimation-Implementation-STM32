# Madgwick Algorithm implementation with Bosch's Bno055 sensor
Quaternion Based Attitude Estimation Using Bno055 and STM32F407
![](./img/Proposal.gif)

# Hardware
- [STM32F407G-DISC1](https://www.st.com/en/evaluation-tools/stm32f4discovery.html)

- [ESP32](https://www.espressif.com/sites/default/files/documentation/esp32-wroom-32e_esp32-wroom-32ue_datasheet_en.pdf) streaming data over serial instead of using USB-to-Serial Converter

- [BNO055](https://www.bosch-sensortec.com/products/smart-sensor-systems/bno055/) (probably made in China)
![](./img/circuit.jfif)

# Connections

| BNO055            | STM32 Pin   |   | STM32 Pin         | ESP32       |
|-------------------|-------------|---|-------------------|-------------|
| SCL               | PB10        |   | PA2 (UART2 TX)    | GPIO3 (RX)  |
| SDA               | PB11        |   | PA3 (UART2 RX)    | GPIO1 (TX)  |
| GND               | GND         |   |                   |             |
| VIN               | 3V          |   |                   |             |

# Frame Representation
To change the frame to ENU Frame, in Sensor_Init() function set:
```
handle_bno055.Frame = ENU_FRAME;				
```in order to config the frame of BNO055 and Madgwick Algorithm also.

# Madgwick Algorithm
