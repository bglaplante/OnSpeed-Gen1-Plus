# OnSpeed-Gen1-Plus
Takes the original OnSpeed Gen1, ports to Arduino Mega

The original work at https://github.com/flyonspeed-org/OnSpeed-Gen1 used a $50 Due board.
I have ported the project to the Mega, which is sufficient as the Gaussian average was turned off.
This required changing timer libraries and a substantial rework of the code.

Currently as of August 2026, the project has been tested successfully with the Dynon D10A,
which means it works with that and the whole D100 series.

In addition it supports Skyview, and has been bench tested with a data generator emulating
Skyview data.

Garmin G3X is planned.

The project includes an LED display which shows the current reported AOA percent.
This simplifies configuration - you do a test flight at various slow flight speeds and record the reported AOA % values.
Then you can (post-flight) determine the starting thresholds for each OnSpeed point.
Those thresholds can then be placed onto an SD card.  When the system starts, it reads the SD card for configuration values.

A circuit board has been designed to hold the Mega Pro Mini, RS232 converter, LED display, SD card reader, and Step Down power supply.
