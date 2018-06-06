# Prusa MK3 + Multi-Material-v1

## How to make it work

1. A 5 pin connector is needed to replace the 4 pin connector on the cable end that connects to the EINSY RAMBO board.
2. The wires should be connected like [white][black][empty][blue][red] with the red wire being closest to the edge. See hardware folder for photos.
3. Upload Firmware.ino.rambo.hex to your printer.

## 1. Development environment preparation

   1. Install latest version of `"Arduino Software IDE"`
`https://www.arduino.cc -> Software->Downloads`  

   2. Add (`UltiMachine`) `RAMBo` board into the list of Arduino target boards  
`File->Preferences->Settings`  
into text field `"Additional Boards Manager URLs"`  
type location  
`"https://raw.githubusercontent.com/ultimachine/ArduinoAddons/master/package_ultimachine_index.json"`  
or you can 'manually' modify the item  
`"boardsmanager.additional.urls=....."`  
at the file `"preferences.txt"` (this parameter allows you to write a comma-separated list of addresses)  
_note: you can find location of this file on your disk by following way:  
`File->Preferences->Settings`  (`"More preferences can be edited in file ..."`)_  
than do it  
`Tools->Board->BoardsManager`  
from viewed list select an item `"RAMBo"` (will probably be labeled as `"RepRap Arduino-compatible Mother Board (RAMBo) by UltiMachine"`  
'clicking' the item will display the installation button; select choice `"1.0.1"` from the list(last known version as of the date of issue of this document)  
_(after installation, the item is labeled as `"INSTALLED"` and can then be used for target board selection)_  


## 2. Source code compilation

run `"Arduino IDE"`; select the file `"Firmware.ino"` from the subdirectory `"Firmware/"` at the location, where you placed the source codes  
`File->Open`  
make the desired code customizations; **all changes are on your own risk!**  

select the target board `"RAMBo"`  
`Tools->Board->RAMBo`  
_note: it is not possible to use any of the variants `"Arduino Mega …"`, even though it is the same MCU_  

run the compilation  
`Sketch->Verify/Compile`  

upload the result code into the connected printer  
`Sketch->Upload`  

or you can also save the output code to the file (in so called `HEX`-format) `"Firmware.ino.rambo.hex"`:  
`Sketch->ExportCompiledBinary`  
and then upload it to the printer using the program `"FirmwareUpdater"`  
_note: this file is created in the directory `"Firmware/"`_  
