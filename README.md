# Lock Controller - Software - WIP

This is a **Work In Progress**.

This is the AVR AT90S2313 assembler code that goes with the [corresponding hardware](https://github.com/jpablo128/lockcontroller-hardware).

Note that the AT90S2313 is a very old part, no longer sold or supported by Atmel. Modern versions of Atmel Studio don't support this part. We are using an older version, AVRStudio v4.19, for this.

## Hardware
The system will allow to control a simple sliding lock using bluetooth. We will reuse parts from an old CD-drive, namely the stepper motor and the linear drive (mechanics that convert the rotary motion into linear motion). Other hardware parts present in the system:

- a toggle switch, to manually cause the lock to open or close.
- two infrared slot switches, to detect the end positions of the lock.
- two leds, one red and one green, to indicate if the lock is open or closed (stationary), or opening or closing (blinking).
- the bluetooth module (to be determined).

So the software on the microcontroller must:

- send pulses to move the bipolar stepper motor (an L293D Quadruple Half-H drivers IC is used to drive the coils in the stepper motor)
- detect when the sliding part has reached the end position (either totally open, or totally closed)
- turn on or off the leds, or make them blink, according to the current state of the system.
- detect a push on the switch, decide what to do, and do it (open if closed, close if open, open in any other case).
- detect serial communication (UART), which is really coming from the Bluetooth module. The data received will be a command (to open, close, or toggle).
- possibly, send data through the UART (BT module) to a remote system, to inform of the state of the system.


The at90s2313 pins are used as follows:

- Pin  2: RXD (connected to TXD of BT module or arduino)
- Pin  3: TXD (connected to RXD of BT module or arduino)
- Pin  6: INT0/PD2 push button (toggle open/close)
- Pin  8: PD4: input IR barrier 1
- Pin  9: PD5: input IR barrier 2
- Pin 12: PB0, motor control W2-0 (W=winding) these really go to the L293
- Pin 13: PB1, motor control W2-1
- Pin 14: PB2, motor control W1-0
- Pin 15: PB3, motor control W1-1
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

The open limit switch uses positive logic: 1 when active, 0 when inactive.
The close limit switch uses negative logic: 0 when active, 1 when inactive.
Thus, pd5=0 and pd4=1 would mean it would be opened and closed at the same time.


## Status / To do

Pull request #1 (by 0xb0lu) has been merged. To test the functionality, the program makes the motor run 256 steps of each type in each direction.
Videos of the program in action:

- [https://youtu.be/ZtfYEbUmUg4](https://youtu.be/ZtfYEbUmUg4)
- [https://youtu.be/187Hbh8Rkw4](https://youtu.be/187Hbh8Rkw4)  


Next steps:

- Modify the circuit on the breadboard to add the infrared slot detectors (limit switches)
- Add the code to check the limit switches to stop the motor
- Add push button to manually toggle the status of the lock and implement corresponding code.


