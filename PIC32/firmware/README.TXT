Most users will want to upload the dazzler-usb.hex file to their PIC32. 
This is the firmware for the current hardware setup corresponding to the 
PCB in the ../schematic directory.

If you intend to connect to the Dazzler via a serial connection
(750000 baud, 8N1, PIC pin 21 is TX, PIC pin 22 is RX) instead of USB
then upload the dazzler-serial.hex file.

If you built a board using the old hardware setup without audio
(there was no PCB published for that setup) then you must use the 
dazzler-noaudio-usb.hex file in the "noaudio" subdirectory.
