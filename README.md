# Arduino Core for neoPLC boards

Program neoPLC boards with the [Arduino](https://www.arduino.cc) IDE.

## Supported boards

 * [neoPLC v1.0](https://www.neoplc.org/shop/microcontroller-ble)

## Installing

### From Arduino

 1. Open the Arduino IDE
 2. Open the Arduino Preferences and add the following URL to the Additional Board Manager URLS:
	* https://deftdynamics.github.io/neoPLC-arduino/package_neoPLC_index.json
 3. Open the Board Manager (Tools -> Board -> Board Manager...)
 4. Search for "neoPLC" and install the board files
    * note that the tools can take some time to install, the GCC compiler is rather large

## Credits

This core is based on the [Arduino-nRF5 Core](https://github.com/sandeepmistry/arduino-nRF5) and licensed under the same [GPL License](LICENSE)

The following tools are used:

 * [GCC ARM Embedded](https://launchpad.net/gcc-arm-embedded) as the compiler
