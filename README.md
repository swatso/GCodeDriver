# GCodeDriver
ESP32 firmware for a Marlin-based X-Y diorama controller.

The firmware listens for pose updates over MQTT, converts local inputs into streamed GCode, and
keeps the controller state in sync with the most recent received pose. It supports two encoder
driving modes: nudge mode for small X/Y adjustments, and speed/bearing mode for continuous motion
control. Encoder changes are debounced so a single twist publishes one settled pose update instead
of a burst of intermediate moves.

Local controls include STOP, SET, BACK, and FORWARD buttons, along with a vehicle encoder input
that is published after it settles. The firmware also records streamed poses for stepwise replay,
stores its Wi-Fi and broker configuration in SPIFFS, and serves the web UI assets found under
`data/`.

Set `-DGCODE_USB_DEBUG=1` in your PlatformIO `build_flags` to mirror streamed GCode to USB and to
echo CNC serial input lines to USB with a `Marlin:` prefix.

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
