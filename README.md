# GCodeDriver
Generates and Streams GCode for Marlin based X-Y diarama controller

Set `-DGCODE_USB_DEBUG=1` in your PlatformIO `build_flags` to mirror streamed GCode to USB
and to echo CNC serial input lines to USB with a `Marlin:` prefix.
