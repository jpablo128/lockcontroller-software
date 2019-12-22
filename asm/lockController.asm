.include "2313def.inc"

.org 0x0000			; 
rjmp reset			; reset vector address (0x0000)
rjmp btn_action		; external interrupt 0 vector address (0x0001
reti				; external interrupt 1 vector address (0x0002)
reti				; timer 1 capture event vector address (0x0003)
reti				; timer 1 compare match vector address (0x0004)
reti 				;rjmp timer1_ovf		;rjmp timer1_ovf ; timer 1 overflow vector address (0x0005)
rjmp timer0_ovf	;rjmp delay_end		;rjmp delay_end	; timer 0 overflow vector address (0x0006)
rjmp uart_rxd   	; UART Rx Complete vector address (0x0007)
reti				; UART Data Register Empty vector address (0x0008)
reti				; UART Tx Complete vector address (0x0009)
reti				; Analog Comparator vector address (0x000A)

; hardware connections:
	; Pin  2: RXD (connected to TXD of BT module or arduino)
	; Pin  3: TXD (connected to RXD of BT module or arduino)
    ; Pin  6: INT0/PD2 push button (toggle open/close)
    ; Pin  8: PD4: input, "lock open" IR barrier (active on logical 1)
    ; Pin  9: PD5: input, "lock closed" IR barrier (active on logical 0) 
    ; Pin 12: PB0, motor control Coil2B 
    ; Pin 13: PB1, motor control Coil2A
    ; Pin 14: PB2, motor control Coil1B
    ; Pin 15: PB3, motor control Coil1A
    ; Pin 16: PB4: output enable/disable L293
    ; Pin 17: PB5: green led output not used as of now.
    ; Pin 18: PB6: red led output not used as of now.


; calculations for a clock of 6.1440 MHz
; 6144000 / 1024 = 6000 this is the number of ticks I get in a second. 6 ticks per millisecond.
; 60 ticks = 10 milliseconds. Enough for it to move reliably (?)
; for 150ms: 900 ticks

;.equ time_ms_10	=0xC4		; -60 = 0xC4
;.equ time_ms_20=0x88		; -120 = 0x88
;.equ time_ms_150=0xFC7C		; -900 = 0xFC7C
;.equ time_ms_250=0xF8F	8		; -900 = 0xF8F8

; calculations for a clock of 8 MHz  (new test board)
; 8000000 / 1024 = 7812.5 this is the number of ticks I get in a second. 7.8125 ticks per millisecond (8 ticks)
; for 10 millisecond -> 80 ticks.
; for 15 millisecond -> 120 ticks.
; for 20 millisecond -> 160 ticks.
; for 60 millisecond -> 480 ticks.
; for 150 milliseconds: 1200 ticks. We need timer 1 (2 bytes)
; for 250 milliseconds: 2000 ticks. We need timer 1 (2 bytes)

.equ time_ms_5=0xD8			; -40 = 0xD8
.equ time_ms_10=0xB0		; -80 = 0xB0
.equ time_ms_15=0x88		; -120 = 0x88
.equ time_ms_20=0x60		; -160 = 0x60
.equ time_ms_30=0x10		; -240 = 0x10

.equ time_ms_60=0xFE20		; -480= 0xFE20
.equ time_ms_150=0xFB50		; -1200 = 0xFB50
.equ time_ms_250=0xFF06		; -2000 = 0xFF06
.equ time_ms_2500=0xB1E0	; -20000 = 0xB1E0

; define my registers
.def sysstate=r16
.def coilbits=r17
.def limitsw=r18

.def temp=r25
.def temp2=r24
.def arg1=r23
.def btn_samples=r22
.def result1=r21

; sysstate keeps track of the state / settings of the system. The meaning of the bits is:
; 	bit 0: motor direction:
;			0: Forward step sequence
;			1: Backward step sequence
;	bits 2 and 1: push button state:
;		11: stable up
;    	00: stable down
;    	10 and 01: bouncing, not yet stable
;	bit 3: UNUSED
;	bits 5 and 4: limit switches status status, converted to POSITIVE logic!
;		bit 5 corresponds to the 'closed' switch, bit 4 to the 'open' switch


reset:
	ldi temp,RAMEND
	out SPL,temp	;init Stack Pointer

	; set port B as output
	ser temp
	out DDRB,temp		; Set port B direction out
	clr temp			; 
	out PortB, temp		; set all port B outputs to 0

	; set pd2, pd4 and pd5 as inputs, activate their pull-up resistors
	; port D is input by default
	clr temp
	out DDRD, temp			; set port D as input
	ldi temp, 0b00110100	; set bits 2, 4 and 5 ...
	out PortD, temp			; of portD, thus activating pull-up resistors on pins 2, 4 and 5

	ldi temp, 0b10000000 	; 
	out ACSR, temp			; set ACD bit in ACSR: disable the analog comparator

	; set up sleep mode
	in temp, MCUCR			; load whatever is in MCUCR
	ori temp, 0b00100000	; set bit 5
	andi temp, 0b11101111	; clear bit 4
	out MCUCR, temp			; enable sleep, set sleep mode 0 (idle mode)

	; set prescalers for timers
	ldi temp, 0b00000101			; set prescaler to CK/1024
	out TCCR0, temp				; set timer 0 prescaler to CK/1024  Timer/Counter 0 Control Register  
	out TCCR1B, temp			; set timer 1 prescaler to CK/1024 

	; TEMPORARY HACK!! in the final program we will only enable the l293D when the motor needs to move!
	sbi PortB, 4	; enable L293D
	
	; load the values of the sequence into registers 0 to 7. These registers will act as a table with values to output to the motor.
	; This table can be used in either direction (dir0 is from r0 to r7, dir 1 is from r7 to r0). Register X will be used for indirect
	; addressing, using post increment (in dir0) and pre-decrement (in dir1)
	ldi temp, 0b0101
	mov r0, temp
	ldi temp, 0b0001
	mov r1, temp
	ldi temp, 0b1001
	mov r2, temp
	ldi temp, 0b1000
	mov r3, temp
	ldi temp, 0b1010
	mov r4, temp
	ldi temp, 0b0010
	mov r5, temp
	ldi temp, 0b0110
	mov r6, temp
	ldi temp, 0b0100
	mov r7, temp

	; set up initial sysstate
	clr sysstate 		; initialize sysstate to 0
	rcall set_btn_up	; this changes bits 1 and 2 of sysstate, and enables the uart
	
	sei						; enable global interrupts
	rcall update_sw_states
	rjmp slumber			; go to a low-power state, waiting for something to happen.

; *** Main routines

dir0:							; use the step sequence in a forward fashion (r0 to r7)
	cbr sysstate, 0b00000001	; clear bit 0, set direction to clockwise, close
	clr XH
	clr XL
	rjmp do_step

dir1:							; use the step sequence in a backward fashion (r7 to r0)
	sbr sysstate, 0b00000001	; set bit 0,  set direction to counter clockwise, open
	ldi XL, 8
	rjmp do_step

wait4_bounce:				; this routine waits for the switch bounce to stabilize
	ldi	arg1, time_ms_30 	; set the argument to the next function call
	rcall start_timer0		; start delay timer (timer0)
	sleep
	; check if bounce finished, if not, continue waiting
	; put value of PD2 bit into bit 0 of btn_samples, rotating left all other bits
	clc					; clear carry
	sbic PinD, 2		; sample pushbutton:  skip if bit 2 in Port D (PD2, int0) is 0 
	sec					; set carry
	rol btn_samples		; update the btn samples
	cpi btn_samples, 0x00
	breq stable_down
	cpi btn_samples, 0xFF
	breq stable_up
	; it's not stable yet.
	rjmp wait4_bounce

stable_down:
	rcall set_btn_down			; this routine already sets bits 1 and 2  of sysstate (to 00=stable down)
	rjmp slumber

stable_up:
	sbr sysstate, 0b00000110	; set bits 1 and 2 of sysstate (to 11=stable up)
	rcall set_btn_up
	rjmp toggle					; stable up after bounce implies toggle... 

toggle:
	bst	sysstate, 0			; store bit 0 of sysstate (motor direction) in T
	brts dir0				; if it's 1 (backward), change direction to 0 (forward)
	brtc dir1				; if it's 0 (forward), change direction to 1 (backward)
	rjmp slumber			; this will never be reached

slumber:
	sleep
	rjmp slumber


; *** END main routines

uart_rxd:					; UART Receive interrupt service routine
	in temp2, UDR 
	cpi temp2, 0b01010101
	brne doreti
	; since this is an interrupt service routine but we're not using reti, we have to 
	; perform the effects of a reti by hand: increment stack pointer by 2, and set global interrupts
	pop temp		; inc SPL is not allowed, so we pop twice to remove the return address from the stack
	pop temp
	sei
	rjmp toggle
	doreti:
		reti

; *** Motor code ***

do_step:
	rcall calc_next_step ; update coilbits, and prepare the X pointer for the next round
	rcall set_coils		 ; output the value to act on the motor coils
	sleep				 ; go to low-power mode until next interruption	
	rcall check_limit	 ; after this call, result1 contains 0 (limit not reached), or 0xFF (limit reached)
	cpi result1, 0
	breq do_step		; if limit not reached, do one more step
	rjmp slumber

calc_next_step:			; put in coilbits the next step in the sequence, and update X
	bst	sysstate, 0		; store bit 0 of sysstate in T
	brts calc_dir1		; if T is 1, return the pr
	; calc_dir0
	ld coilbits, X+			; if 0, load and increment
	cpi XL, 8			; is XL 8?
	breq reset_xl		; if so, set it to 0
	ret
		reset_xl:
			clr XL				; if so, reset XL
			ret
	calc_dir1:
		ld coilbits, -X			; predecrement!
		cpi XL, 0x00			; is XL 0?
		breq set8_xl			; if so, set it to 8
		ret
		set8_xl:
			ldi XL, 8			; set XL to point to register r8
			ret

set_coils:				; set the coils to a given polarization.
	in	temp, PortB
	andi temp, 0b11110000	; preserve the high nibble, clear the low nibble

	mov  temp2, coilbits	; copy coilbints into temp2
	andi temp2, 0b00001111	; clear the high nibble, preserve the low nibble of coilbits. We only want the 4 lower bits.	
	or   temp, temp2		; make temp keep the high nibble of portB, and the low nibble of coilbits
	out PortB, temp			; send new value to PortB
	
	ldi	arg1, time_ms_15	; set the argument to the next function call
	rcall start_timer0		; start delay timer (timer0)
	ret

; *** Limit switches code

check_limit:
	; this function puts the result in result1. 0x00 means that limit has not been reached. 0xFF means that limit has been reached.
	rcall update_sw_states
	bst	sysstate, 0			; store bit 0 of sysstate (motor direction) in T
	brts check_open			; if T is set (direction is 1=backward), check the open switch
	; check_closed			; check bit 5 of sysstate
		; if bit 5 of sysstate is 1, closedlimit has been reached
		ldi result1, 0xFF		; assume limit reached
		sbrs sysstate, 5		; skip next instruction our assumption was right(if bit 5 is set = if limit has been reached)
		ldi result1, 0x00		; oops, our assupmtion was wrong, set result1 to 0 (limit not reached)
		ret
	check_open:
		; check bit 4 of sysstate
		; if bit 4 of sysstate is 1, open limit has been reached
		ldi result1, 0xFF		; assume limit reached
		sbrs sysstate, 4		; skip next instruction our assumption was right(if bit 4 is set = if limit has been reached)
		ldi result1, 0x00		; oops, our assupmtion was wrong, set result1 to 0 (limit not reached)
		ret
		
update_sw_states:	; updates bits 5 and 4 in sysstate. Bit 5 corresponds to the 'closed' switch, bit 4 to the 'open' switch
	in temp, PinD
	sbr sysstate, 0b00100000	; set bit 5 of sysstate (assume bit 5 of pinD is 0) this converts to positive logic!
	sbrc temp, 5 				; skip if our assumption was correct, if bit 5 is set
	cbr sysstate, 0b00100000	; clear bit 5 otherwise
	sbr sysstate, 0b00010000	; set bit 4 of sysstate (assume bit 4 of pinD is 1) 
	sbrs temp, 4 				; skip if our assumption was correct, if bit 4 of pinD is set
	cbr sysstate, 0b00010000	; clear bit 4 otherwise
	; here, bit 5 of sysstate reflects the state of the 'closed' switch, and bit 4 reflects the state of the open switch, both with positive logic
	ret

; *** Push button code ***
btn_action:				; a button action happened. Disconnect int0 and wait for bounce to stabilize.
	rcall disable_btn
	rcall stop_timer0	; let's stop timer0 in case it was running
	rcall disable_uart
	ldi btn_samples, 0b01010101
	cbr sysstate, 0b00000100	; clear bit 2
	sbr sysstate, 0b00000010	; set bit 1. Now state is 01, which is unstable
	; since this is an interrupt service routine but we're not using reti, we have to 
	; perform the effects of a reti by hand: increment stack pointer by 2, and set global interrupts
	pop temp		; inc SPL is not allowed, so we pop twice to remove the return address from the stack
	pop temp
	sei						
	;ldi	arg1, time_ms_30 	; set the argument to the next function call
	;rcall start_timer0		; start delay timer (timer0)
	rjmp wait4_bounce
	
disable_btn:
	ldi temp, 0b00000000
	out GIMSK, temp			; disable int0
	ret

set_btn_up:					; set btnstate to 11, int0 to falling edge. 
	in temp, MCUCR			; get whatever is in MCUCR
	sbr temp, 0b00000010	; set bit 1
	cbr temp, 0b00000001	; clear bit 0
	out MCUCR, temp			
	ldi temp, 0b01000000
	out GIMSK, temp			; enable int0
	sbr sysstate, 0b00000110	; set bits 1 and 2 of sysstate: state stable up
	rcall enable_uart
	ret

set_btn_down:				; set btnstate to 00, int0 to raising edge. 
	in temp, MCUCR			; get whatever is in MCUCR
	sbr temp, 0b00000011	; set bits 0 and 1, activate int0 on rising edge
	out MCUCR, temp			
	ldi temp, 0b01000000
	out GIMSK, temp			; enable int0
	cbr sysstate, 0b00000110	; clear bits 1 and 2 of sysstate: state stable down
	rcall disable_uart		; let's ignore the remote control by uart when the button is pressed down
	ret

; *** Timer 0 code ***
start_timer0:
	; The argument is arg1
	ldi temp, 2
	out TIFR, temp		; VITAL: reset timer overflow flag. It's cleared by writing a 1 to the flag.
	out TCNT0, arg1		; Put counter time (passed in register arg1) into TCNT0 (Timer/Counter 0), start counting.
	in	temp, TIMSK
	sbr	temp, 2			; set bit 1 of whataver was in TIMSK
	out TIMSK, temp		; set bit 1 of TIMSK, Timer/Counter 0 Overflow Interrupt Enable
	ret	

stop_timer0:
	in	temp, TIMSK
	cbr	temp, 2			; clear bit 1 of whataver was in TIMSK
	out TIMSK, temp		; clear bit 1 of TIMSK, Timer/Counter 0 Overflow Interrupt Enable
	ret	

timer0_ovf:		; Timer 0 ISR
	rcall stop_timer0	; let's always stop it !
	reti


; uart routines
enable_uart:
	ldi temp, 51			;9600 baud on an 8 MHz clock
	;ldi temp, 39			;9600 baud on a 6.1440 MHz clock
	out UBRR, temp
	 
	ldi temp, 0b10010000		; enable reception (RXEN) and reception interrupt (RXCIE)
	out UCR, temp
	ret
 
disable_uart:
	ldi temp, 0b00000000		; disable reception (RXEN) and reception interrupt (RXCIE)
	out UCR, temp
	ret


		
