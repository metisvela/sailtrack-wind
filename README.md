<p align="center">
  <img src="https://raw.githubusercontent.com/metisvela/sailtrack/main/assets/sailtrack-logo.svg" width="180">
</p>

<p align="center">
  <img src="https://img.shields.io/github/license/metisvela/sailtrack-wind" />
  <img src="https://img.shields.io/github/v/release/metisvela/sailtrack-wind" />
</p>

# SailTrack Wind

SailTrack Wind is a component of the SailTrack system, it manages wind data, providing apparent wind speed and direction. To learn more about the SailTrack project, please visit the [project repository](https://github.com/metisvela/sailtrack).

The SailTrack Wind module is based on a battery powered LilyGo TTGO T7, consisting of an [ESP32](https://www.espressif.com/en/products/socs/esp32) microcontroller, connected to a custom-built ultrasonic anemometer. For a more detailed hardware description of the module, please refer to the [Bill Of Materials](hardware/BOM.csv).

The module performs the following tasks:

* It gets the wind data coming from the anemometer and sends them to the SailTrack Network.

<p align="center">
  <br/>
  <img src="hardware/Connection Diagram.svg">
</p>

## Installation

Follow the instructions below to get the SailTrack Wind firmware correctly installed. If you encounter any problem, please [open an issue](https://github.com/metisvela/sailtrack-wind/issues/new).

1. [Install PlatformIO Core](https://docs.platformio.org/en/latest/core/installation/index.html).
2. Clone the SailTrack Wind repository:
   ```
   git clone https://github.com/metisvela/sailtrack-wind.git 
   ``` 
3. Cd into the directory:
   ```
   cd sailtrack-wind
   ```
4. **(macOS ONLY)** Uncomment the commented lines after "Patch for macOS" in the `platformio.ini` file.
5. Connect the module with an USB cable.
6. Finally, flash the firmware:
   ```
   pio run
   ```

## Usage

Once the firmware is uploaded the module can work with the SailTrack system. When SailTrack Wind is turned on, a LED start to blink to notify the user about the connection status with SailTrack Core. Then, if the connection is successful, the LED stays on, otherwise the module will put itself to sleep, and it will try to connect later. Once the module is connected it will automatically start sending measurements.

## Contributing

Contributors are welcome. If you are a student of the University of Padova, please apply for the Metis Sailing Team in the [website](http://metisvela.dii.unipd.it), specifying in the appliaction form that you are interested in contributing to the SailTrack Project. If you are not a student of the University of Padova, feel free to open Pull Requests and Issues to contribute to the project.

## License

Copyright © 2023, [Metis Sailing Team](https://github.com/metisvela). SailTrack Wind is available under the [GPL-3.0 license](https://www.gnu.org/licenses/gpl-3.0.en.html). See the LICENSE file for more info.
