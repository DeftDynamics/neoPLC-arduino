# Arduino Core for Nordic Semiconductor nRF5 based boards

Program [Nordic Semiconductor](https://www.nordicsemi.com) nR5 based boards with the [Arduino](https://www.arduino.cc) IDE.

## TO DO

 1. create new board files for the bootloader
 2. cut some fat from the bootloader
 3. ensure BLE update works with the bootloader (may currently be broken due to being forced to reset the softdevice)
 4. create variants for specific boards, not just the modules
 5. ~~add SOFT_RESET_PIN definition to all variants~~
 6. ~~configure SOFT_RESET_PIN as a pulldown input and attach a rising interrupt to it for resetting the board (done via initVariant();)~~

## Supported boards

 * [nRF52 DK](https://www.nordicsemi.com/eng/Products/Bluetooth-Smart-Bluetooth-low-energy/nRF52-DK)
   * For boards prior to ```2016.9``` (see sticker), the lastest JLink bootloader is required to upload sketches. To upgrade, press the boot/reset button while powering on the board and copy over the latest [bootloader](https://www.nordicsemi.com/eng/nordic/Products/nRF52-DK/nRF5x-OB-JLink-IF/52275).

## Installing

### From git (for core development)

 1. Install Git: https://git-scm.com/download
 2. Open the Git-Gui (either type git-gui in the command line or find it in your installed programs)
 3. Select "Help" -> "Show SSH Key" and generate a public key, send the key to kak0032@auburn.edu to be added to the repository
 4. Once your SSH key has been addede, Select "Clone Exisiting Repository"
  * Source Location is: ```git@gitlab.com:kylekubik/arduino-nRF5.git```
  * Target directory is: ```<SKETCHBOOK>/hardware/deft/nRF5/``` , where ```<SKETCHBOOK>``` is your Arduino Sketch folder:
   * OS X: ```~/Documents/Arduino```
   * Linux: ```~/Arduino```
   * Windows: ```~/Documents/Arduino```
 5. click Clone


### Flashing a SoftDevice

 1. ```cd <SKETCHBOOK>```, where ```<SKETCHBOOK>``` is your Arduino Sketch folder:
  * OS X: ```~/Documents/Arduino```
  * Linux: ```~/Arduino```
  * Windows: ```~/Documents/Arduino```
 2. Create the following directories: ```tools/nRF5FlashSoftDevice/tool/```
 3. Download [nRF5FlashSoftDevice.jar](https://github.com/sandeepmistry/arduino-nRF5/releases/download/tools/nRF5FlashSoftDevice.jar) to ```<SKETCHBOOK>/tools/nRF5FlashSoftDevice/tool/```
 4. Restart the Arduino IDE
 5. Select your nRF board from the Tools -> Board menu
 6. Select a SoftDevice from the Tools -> "SoftDevice: " menu
 7. Select a Programmer (J-Link, ST-Link V2, or CMSIS-DAP) from the Tools -> "Programmer: " menu
 8. Select Tools -> nRF5 Flash SoftDevice
 9. Read license agreement
 10. Click "Accept" to accept license and continue, or "Decline" to decline and abort
 11. If accepted, SoftDevice binary will be flashed to the board

## Credits

This core is based on the [Arduino-nRF5 Core](https://github.com/sandeepmistry/arduino-nRF5) and licensed under the same [GPL License](LICENSE)

The following tools are used:

 * [GCC ARM Embedded](https://launchpad.net/gcc-arm-embedded) as the compiler
 * A [forked](https://github.com/sandeepmistry/openocd-code-nrf5) version of [OpenOCD](http://openocd.org) to flash sketches
