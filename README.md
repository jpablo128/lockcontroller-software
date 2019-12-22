# Lock Controller - Software - WIP

This is a **Work In Progress**.

This is the AVR AT90S2313 assembler code that goes with the [corresponding hardware](https://github.com/jpablo128/lockcontroller-hardware).

Note that the AT90S2313 is a very old part, no longer sold or supported by Atmel. Modern versions of Atmel Studio don't support this part. We are using an older version, AVRStudio v4.19, for this.

## Hardware
The system will allow to control a simple sliding lock using bluetooth. We will reuse parts from an old CD-drive, namely the stepper motor and the linear drive (mechanics that convert the rotary motion into linear motion). Other hardware parts present in the system:

- a toggle switch, to manually cause the lock to open or close.
- two microswitches, to detect the end positions of the lock.
- two leds, one red and one green, to indicate if the lock is open or closed (stationary), or opening or closing (blinking). Not used as of now.
- the bluetooth module for serial communication.	

So the software on the microcontroller must:

- send pulses to move the bipolar stepper motor (an L293D Quadruple Half-H drivers IC is used to drive the coils in the stepper motor)
- detect when the sliding part has reached the end position (either totally open, or totally closed)
- turn on or off the leds, or make them blink, according to the current state of the system.
- detect a push on the switch, decide what to do, and do it (open if closed, close if open, open in any other case).
- detect serial communication (UART), which is really coming from the Bluetooth module. The data received will be a command (toggle).
- possibly, send data through the UART (BT module) to a remote system, to inform of the state of the system.


The at90s2313 pins are used as follows:

- Pin  2: RXD (connected to TXD of BT module or arduino)
- Pin  3: TXD (connected to RXD of BT module or arduino)
- Pin  6: INT0/PD2 push button (toggle open/close)
- Pin  8: PD4: open limit microswitch (positive logic: 1 when active, 0 when inactive, because most of the time it will be active)
- Pin  9: PD5: closed limit microswitch (negative logic: 0 when active, 1 when inactive, because most of the time it will be inactive)
- Pin 12: PB0, to pin2 of L293D, actuates on pin3 of L293D,  Coil 2B
- Pin 13: PB1, to pin7 of L293D, actuates on pin6 of L293D,  Coil 2A
- Pin 14: PB2, to pin10 of L293D, actuates on pin11 of L293D,  Coil 1B
- Pin 15: PB3, to pin15 of L293D, actuates on pin14 of L293D,  Coil 1A
- Pin 16: PB4: output enable/disable L293
- Pin 17: PB5: green led output
- Pin 18: PB6: red led output


## Open/close logic notes

The logic for the limit switches is as follows:

    - Pin  8: PD4: OPEN signal, 1 when active, 0 when inactive⋅⋅
    - Pin  9: PD5: CLOSED signal, 0 when active, 1 when inactive

     pd5   pd4
      1     1     -> lock opened
      0     0     -> lock closed
      1     0     -> somewhere in between
      0     1     -> impossible

    
There's an impossible state (the lock would be both open and closed at the same time):
    pd5 = 0   and pd4 = 1  


## Toggle button

The push button implements some simple logic with bounce control. At startup, the pull-up resistor on int0/pd2 is activated, so the button is normally at 1, and we set int0 to detect a falling edge.
When the button is pushed, the program waits for the bounce to finish. For this, it samples the input of the button every few milliseconds, and puts the sampled bit into a regiser.The samples are rolled from left to right, so the sampling routine will always look at the last 8 samples. If all the samples have the same value, the bounce is considered to have finished (thus, a value or 0 is stable down, and a value of 0xFF is stable up).
When stable down is detected, int0 is set to trigger on the rising edge, and the exact same bounce routine will be called. This bounce method seems to work very well. It will work for short or long pulses, or even in the unlikely case that the push/release is so fast that the down bounce doesn't finish and the up-bounce starts. The routine only cares about reaching a stable state.
Stable down just stops things. Stable up triggers a togggle on the lock.


## UART
The system accepts a toggle command through the serial port (uart), at 9600 baud, N81. The command is 0b01010101  (85 dec, 55 hex, 'U' in ascii). When that character is received, the toggle routine is called automatically.
The UART interrupt is disconnected when the toggle button is being used.

## Status / To do
The whole program has been refactored totally from previous versions. It uses low-power 'sleep' mode, which not only saves energy, but it enables a (hopefully) simpler flow.

No misbehavior has been observed when running the program in the simulator. In the test circuit there are some anomalies and inconsistent behaviors. The stepper motor sometimes stalls on the  backward direction (dir1, clockwise). Sometimes, but not always, which is really strange. The effect is as though there's a wrong sequence being sent (or, equivalently, a wrong wiring), but it doesn't make sense, because the exact same values sent to the motor in the other direction make it move flawlessly. Toggling through the UART seems to cause less stuttering. It works most of the time, but sometimes it doesn't.
Maybe it's a hardware problem in the motor. Other motors behave even worse... but we don't have the specs for any of the motors, they're all old salvaged parts from old CD or diskette drives.

The software part of this project should be frozen until we resolvethe mechanical aspects (lock, motor, drive) and other things (power). As of now, the motor behavior is not good enough. We might end up using a simpler DC motor, or a servo, or whatever works well the the mechanics... and of course the code will have to change.
