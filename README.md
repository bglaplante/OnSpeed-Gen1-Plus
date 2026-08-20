# OnSpeed-Gen1-Plus
Takes the original OnSpeed Gen1, ports to Arduino Mega

The original work at https://github.com/flyonspeed-org/OnSpeed-Gen1 used a $50 Due board
I have ported the project to the Mega, which is sufficient as the Gaussian average was turned off.
This required changing timer libraries and a substantial rework of the code.

Currently as of August 2026, the project has been tested successfully with the Dynon D10A,
which means it works with that and the whole D100 series.

In addition it supports Skyview, and has been bench tested with a data generator emulating
Skyview data.

Garmin G3X is planned.
