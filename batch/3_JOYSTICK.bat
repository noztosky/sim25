@echo off
REM ===== STEP 3: SBUS virtual joystick (after SITL is linked) =====
REM Connects SBUS over TCP to 127.0.0.1:5765 (SERIAL5_PROTOCOL=23 = RCIN).
REM In the GUI: Connect -> Loiter -> ARM(rudder) -> R = throttle up.
start "SBUS Joystick" pythonw D:\xlab\sim25\x_memory2\ap_tools\sbus_joystick.py 127.0.0.1:5765
echo Joystick launched (async).
