.include "2313def.inc"

; THIS 1ST VERSION of the program just runs the motor in one direction continuously

.def temp =r16

; Maybe we don't really need a 'state' control... just looking at the IR inputs may be enough?
.def state=r17		; current state of the system: bits 2, 1 and 0:
					; 001 -> lock closed
					; 010 -> lock opened
					; 101 -> closing lock
					; 110 -> opening lock
					; 111 -> undetermined or changing state
					; bits 5, 4 and 3 are the 'previous' state

.def outctrl=r18	; outcontrol will have the value to output on port b. It controls the motor, both L293D enable pins, and the leds.
					; bits 3, 2, 1, and 0: control the motor (values 1, 2, 4, 8)
					; bit 4 enables/disables the L293D
					; bit 5 turns on/off the Red led  (used when closing/closed)
					; bit 6 turns on/off the Green led (used when opening/opened)

; let's say that the sequence 1, 2, 4, 8 causes the 'closing' movement, and the reverse causes the 'opening' movement.
; this will depend on how things are arranged physically.

.def retaddr=r19

;.equ nibbles =0b10100101

.org 0x0000		; 
rjmp reset		; reset vector address (0x0000)
rjmp toggle		; external interrupt 0 vector address (0x0001
reti			; external interrupt 1 vector address (0x0002)
reti			; timer 1 capture event vector address (0x0003)
reti			; timer 1 compare match vector address (0x0004)
reti			;rjmp timer1_ovf ; timer 1 overflow vector address (0x0005)
rjmp delay_end	;rjmp delay_end	; timer 0 overflow vector address (0x0006)
reti			; UART Rx Complete vector address (0x0007)
reti			; UART Data Register Empty vector address (0x0008)
reti			; UART Tx Complete vector address (0x0009)
reti			; Analog Comparator vector address (0x000A)


; calculations for a clock of 6.1440 MHz
; 6144000 / 1024 = 6000 this is the number of ticks I get in a second. 6 ticks per millisecond

; change the name of this to something more generic... it's not 6 ticks any more...
.equ timer6ticks=0xC4		; -6 = 0xFA



reset:
	ldi temp,RAMEND
	out SPL,temp	;init Stack Pointer

	; Pin  2: RXD (connected to TXD of BT module or arduino)
    ; Pin  3: TXD (connected to RXD of BT module or arduino)
    ; Pin  6: INT0/PD2 push button (toggle open/close)
    ; Pin  8: PD4: input, "lock open" IR barrier (active on logical 1)
    ; Pin  9: PD5: input, "lock closed" IR barrier (active on logical 0) 
    ; Pin 12: PB0, motor control W2-0 (W=winding) these really go to the L293
    ; Pin 13: PB1, motor control W2-1
    ; Pin 14: PB2, motor control W1-0
    ; Pin 15: PB3, motor control W1-1
    ; Pin 16: PB4: output enable/disable L293
    ; Pin 17: PB5: green led output
    ; Pin 18: PB6: red led output

	; set port B as output

	ldi state, 0b00000111	; set state to unknown
	ser temp
	out DDRB,temp		; Set port B direction out
	clr outctrl			; set r18 to 0b00000000
	out PortB, outctrl 	; set all port B outputs to 0

	; set pd2, pd4 and pd5 as inputs
	; port D is input by default
	; set state according to input

 
	sei						; enable global interrupts
	; enable int0
	
	; TEMPORARY HACK!! in the final program we will only enable the l293D when the motor needs to move!
	sbi PortB, 4	; enable L293D

	ldi ZL, 0
	ldi ZH, 0
	rjmp phase1
	

idle:
	; if timer 0 is off, it means we've just returned from delay_end, so we need to check ZL
	; if timer 0 is on, we're still in a delay, so let's continue idling
	;sbrc temp, 7		; if bit 7 of TIMSK is cleared,it means that timer 1 is off
	in temp, TIMSK
	sbrc temp, 1		; if bit 1 of TIMSK is cleared,it means that timer 0 is off
	rjmp idle
	cpi	ZL, 0			; compare return address with 0
	breq idle			; is not 0, loop again
	ijmp				; it is 0, do an indirect jump (address in Z register).


step:
	; direction is set by a register or something... depending on direction,
	; the sequence will be phase1 to 4, or phase4 to 1
	;sbi PortB, 4	; enable L293D

phase1:
	cbi PortB, 3		; turn off bit 3
	cbi PortB, 2		; turn off bit 2
	cbi PortB, 1		; turn off bit 1
	sbi PortB, 0			; turn on bit 0
	ldi ZL, low(phase2)		; save the return address
	ldi ZH, high(phase2)
	rjmp delay			; JUMP!! call delay 1ms

phase2:
	cbi PortB, 0		; turn off bit 0
	sbi PortB, 1		; turn on bit 1
	ldi ZL, low(phase3)	; save the return address
	ldi ZH, high(phase3)
	rjmp delay		;	 JUMP!! call delay 1ms
	
phase3:
	cbi PortB, 1		; turn off bit 1
	sbi PortB, 2		; turn on bit 2
	ldi ZL, low(phase4)	; save the return address
	ldi ZH, high(phase4)
	rjmp delay		;	 JUMP!! call delay 1ms
	
phase4:
	cbi PortB, 2			; turn off bit 2
	sbi PortB, 3			; turn on bit 3
	ldi ZL, low(endstep)	; save the return address
	ldi ZH, high(endstep)
	rjmp delay				; JUMP!! call delay 1ms

endstep:
	; if we've reached the end position (open, if it's opening, closed if it's closing), set new state, put retaddr to 0, and jump to idle
	; otherwise, continue with phase 1

	; check if we've reached one of the end positions (closed, or open)
	rjmp phase1



; this is for timer 0
delay:		; we need a delay after setting each phase of a step.
	ldi r16, 0b00000101			; set prescaler to CK/1024
	out TCCR0, r16				; Timer/Counter 0 Control Register  

	ldi r16, 0b00000010	;Timer/Counter 0 Overflow Interrupt Enable bit set
	out TIMSK, r16

	ldi r16, timer6ticks
	out TCNT0, r16				; Put counter time in TCNT0 (Timer/Counter 0), start counting
	rjmp idle



delay_end:		; Timer 0 ISR, the delay finished.
	in r16, SREG	; save the status register
	push r16		; on the stack

	; if there's a reason to stop the delay timer, stop it, otherwise it will keep on running.
	ldi r16, 0
	out TCCR0, r16		; disable Timer0 

	ldi r16, 0b00000000	; Timer/Counter 0 Overflow Interrupt Enable bit cleared (interrupt disabled)
	out TIMSK, r16

	pop r16			; from the stack
	out SREG, r16	; restore the status register
	reti

find_state:
	nop
	; if open barrier active, set to open
	; if closed barrier active, set to closed


open:
	nop
	; here goes the open lock routine
	; set motor direction. While open barrier inactive (0), do a step (4 phases)
	; maybe use bit 6 or 7 of 'state' (r17) to control the direction of the motor, used to determine the jumps
	; between phases (1 -> 4 or 4 -> 1)

close:
	nop
	; here goes the close lock routine
	; set motor direction. While closed barrier inactive (1), do a step (4 phases)


toggle:
	nop
	; this is hardware interrupt 0. Set it up on the rising edge of INT0.
	; WE MUST eliminate bounces in the software, so, when this ISR is called:
	;	- disable hardware interrupt 0
	;	- start counting 50 ms (for example), reti
	;	- at the end of the 50 ms, check if INT0 is still 1
	;	- if it is, determine what to do, as noted below (re-enable HW interrupt 0, rising edge)
	;	- if it's not,  re-enable HW interrupt 0, rising edge) and reti 
	
	; read open barrier, if active (logical 1), close.
	; read closed barrier, if active (logical 0), open
	; if neither is active, just open (default action)


; NEXT STEPS:
; - light up leds according to state/action.
; - UART interrupt: receive an Open, Close or Toggle command through the serial port (in the future, through Bluetooth)
