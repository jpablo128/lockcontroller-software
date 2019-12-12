# Lock Controller - Software - WIP

This is a **Work In Progress**.

This is the AVR AT90S2313 assembler code that goes with the [corresponding hardware](https://github.com/jpablo128/lockcontroller-hardware).

Note that the AT90S2313 is a very old part, no longer sold or supported by Atmel. Modern versions of Atmel Studio don't support this part. We are using an older version, AVRStudio v4.19, for this.

## Hardware
The system will allow to control a simple sliding lock using bluetooth. We will reuse parts from an old CD-drive, namely the stepper motor and the linear drive (mechanics that convert the rotary motion into linear motion). Other hardware parts present in the system:

- a toggle switch, to manually cause the lock to open or close.
- two microswitches, to detect the end positions of the lock.
- two leds, one red and one green, to indicate if the lock is open or closed (stationary), or opening or closing (blinking).
- the bluetooth module (to be determined, likely to be HC-06).

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
- Pin 12: PB0, to pin2 of L293D, actuates on pin3 of L293D,  Winding 1 A
- Pin 13: PB1, to pin7 of L293D, actuates on pin6 of L293D,  Winding 2 A
- Pin 14: PB2, to pin10 of L293D, actuates on pin11 of L293D,  Winding 1 B
- Pin 15: PB3, to pin15 of L293D, actuates on pin14 of L293D,  Winding 2 B
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

    open:
        ldi limitsw, 0b00010000    ; define which limit switch to test for,  pd4 is for the 'open' limit switch
        rjmp sb1                    ; just jumpo to the 'open' sequence. Let's say that the 'open' movement is 'backward'


    close:    
        ldi limitsw, 0b00100000    ; define which limit switch to test for, pd5 is for 'close' limit switch
        rjmp sf1                    ; just jumpo to the 'close' sequence. Let's say that the 'close' movement is 'forward'

There's an impossible state (the lock would be both open and closed at the same time):
    pd5 = 0   and pd4 = 1  


## Toggle button

The push button implements some simple logic with bounce control. At startup, the pull-up resistor on int0/pd2 is activated, so the button is normally at 1, and we set int0 to detect a falling edge.
When the button is pushed,timer1 is started. When timer1 overflows, we set int0 to detect a rising edg. When a rising edge is detected, we wait again (start timer1), and after that time, the button is set to 'up' again (detecting falling edge), and the toggle routine is called.

## UART
The system accepts a toggle command through the serial port (uart), at 9600 baud, N81. The command is 0b01010101  (85 dec, 55 hex, 'U' in ascii). When that character is received, the toggle routine is called automatically.
The UART interrupt is disconnected when the toggle button is being used.

## Status / To do
The motor stalls sometimes, particularly in one direction. Step delays between 10 and 20 ms have been tried, maybe we need longer delays.

The essential parts of the program have been tried. The next step is a fairly big refactoring that should have been done along the way, from the beginning.
