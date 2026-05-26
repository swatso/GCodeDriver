# GCodeDriver
Generates and Streams GCode for Marlin based X-Y diarama controller

Set `-DGCODE_USB_DEBUG=1` in your PlatformIO `build_flags` to mirror streamed GCode to USB
and to echo CNC serial input lines to USB with a `Marlin:` prefix.

The firmware records streamed GCode lines for stepwise replay. STOP still halts motion, and once
stopped you can use active-low GPIO buttons on 33 (FORWARD), 18 (BACKWARD), and 19 (PRINT) to
step through or replay the recorded buffer.
