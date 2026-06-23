## Feature

* Dedicated for personal research and studying.
* Made from scratch on STM32.
* Simple components that can easily be found.
* Maintains a stable hover position in outdoor environment.
* Position control (planned).

## Result

### Video

 Flight result video: https://youtu.be/znj9OqjzdcU

<a href="https://youtu.be/znj9OqjzdcU">
<img width=400 src="Images/FlightResult.png"></a>

 Hover video: https://youtube.com/shorts/ZwW649EOINM?feature=share

 <a href="https://youtube.com/shorts/ZwW649EOINM?feature=share">
 <img width=400 src="Images/PositionHold.png"></a>

## Controller Block Diagram

<p align="center">
 <img src="Images/BlockDiagram.svg" width=100% alt="Diagram">

## Schematic
### Connection Diagram

<p align="center">
 <img src="Images/Connection.svg" width=100% alt="Schematic"> 

### Wiring

#### ICM20498(IMU)

|STM32 pin|ICM20948 pin|
|-|-|
|GND|GND|
|3.3V|3.3V|
|PB5|INT|
|PB7|SDA|
|PB8|SCL|

#### MS5611(BAROMETER)

|STM32 pin|MS5611 pin|
|-|-|
|GND|GND|
|3.3V|3.3V|
|PA4|CS|
|PA5|SCL *(SCK)*|
|PA6|SDO *(MISO)*|
|PA7|SDA *(MOSI)*|

#### HC-05(BLUETOOTH)

|STM32 pin|HC-05 pin|
|-|-|
|GND|GND|
|PA11|TXD|
|PA12|RXD|

#### MTF-01P(BLUETOOTH)

|STM32 pin|MTF-01P pin|
|-|-|
|GND|GND|
|PA9|TX|
|PA10|RX|

#### F-10A(RC RECEIVER)
|STM32 pin|F-10A pin|
|-|-|
|GND|GND|
|PA8|CH1|
|PB4|CH2|
|PB6|CH3|