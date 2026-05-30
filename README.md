# GCodeDriver
Generates and Streams GCode for Marlin based X-Y diarama controller

Set `-DGCODE_USB_DEBUG=1` in your PlatformIO `build_flags` to mirror streamed GCode to USB
and to echo CNC serial input lines to USB with a `Marlin:` prefix.

The firmware records streamed GCode lines for stepwise replay. STOP still halts motion, and once
stopped you can use active-low GPIO buttons on 33 (FORWARD), 18 (BACKWARD), and 19 (PRINT) to
step through or replay the recorded buffer.

## Reusable Marlin Handshake Helper

This repository now includes a reusable helper in include/marlin_handshake.h that implements
the same handshake pattern used by the firmware:

1. sendLine increments an in-flight command counter.
2. processInput parses line responses and treats lines starting with ok as acknowledgements.
3. canSendNow gates sending when any command is still in flight.
4. Optional ok token consumption supports strict one-line-send-one-ok replay loops.

Minimal integration pattern:

1. Instantiate the helper with your CNC serial stream.
2. Call processInput at the top of your main loop.
3. Before sending a motion line, check canSendNow.
4. For replay/print loops, send one line, then wait until consumeOk returns true.

Suggested loop order:

1. handshake.processInput()
2. input/state updates
3. send logic (gated by handshake.canSendNow())
