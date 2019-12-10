To support audio output, a few changes in pin assignments on the PIC32
were necessary. 

Boards built using the old hardware setup without audio
(../schematic/noaudio) must use the "dazzler-noaudio*.hex" 
files in this directory. 

Boards built using the new hardware setup  (../schematic) 
must use the "dazzler-*.hex" files.

Files named "*-serial.hex" are firmware for connecting the Dazzler via a 3.3V 
serial connection (at 750000 baud, 8N1).

Files named "*-usb.hex" are firmware for connecting the Dazzler via a USB
connection (Dazzler is USB host, device must identify as a USB CDC/ACM device).
