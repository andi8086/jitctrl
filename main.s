;			Jitter debugger test controller
;			    for Arduino Uno Rev 3
;
;				  rev 0.1
;
; Licenced under MIT
; SPDX short identifier: MIT
;
; Copyright 2019 Andreas J. Reichel
;
; Permission is hereby granted, free of charge, to any person obtaining a copy
; of this software and associated documentation files (the "Software"), to deal
; in the Software without restriction, including without limitation the rights
; to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
; copies ; of the Software, and to permit persons to whom the Software is
; furnished to do so, subject to the following conditions:

; The above copyright notice and this permission notice shall be included in
; all copies or substantial portions of the Software.

; THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
; IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
; FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
; AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
; LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
; OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
; SOFTWARE.


;************************ PINOUT *************************
;          Atmega	Arduino			Function
;	#12 OC0A,PD6	digital pin 6 (PWM)	signal out
;	# 5 INT1,PD3	digital pin 3 (PWM)	signal in
;	# 3 TXD,PD1	digital pin 1 (TX)	serial tx
;	# 2 RXD,PD0	digital pin 0 (RX)	serial rx
;
;*********************** COMMANDS ************************
;
;	'f'	set counter compare for frequency
;
;	send-scheme:
;	Arduino		     'A'		'A'
;	TestBoard	'f'       F-Lo   F-Hi
;
;	F = 16M/(2*f_wanted)-1
;	i.e. 7 for 1 MHz, 79 for 100 kHz
;	you want to be << 19.2 kHz, i.e. 150 Hz with F=53332
;
;	'r'	read current timer value
;
;	send-scheme:
;	Arduino			  T-Lo   T-Hi
;	TestBoard	'r'
;
;	'e'	enable test pulses
;	'd'	disable test pulses
;*********************************************************
.include "m328Pdef.inc"

.org 0x00

; Interrupt vector table

reset:          jmp main
int0:           jmp defaultInt
int1:           jmp isrExtInterrupt1
pcint0:         jmp defaultInt
pcint1:         jmp defaultInt
pcint2:         jmp defaultInt
intwdt:         jmp defaultInt
intTIMER2_COMPA:jmp defaultInt
intTIMER2_COMPB:jmp defaultInt
intTIMER2_OVF:  jmp defaultInt
intTIMER1_CAPT: jmp defaultInt
intTIMER1_COMPA:jmp defaultInt
intTIMER1_COMPB:jmp defaultInt
intTIMER1_OVF:  jmp defaultInt
intTIMER0_COMPA:jmp defaultInt
intTIMER0_COMPB:jmp defaultInt
intTIMER0_OVF:  jmp defaultInt
intSPI:         jmp defaultInt
intUSART0_RX:   jmp isrUSART0RX
intUSART0_UDRE: jmp defaultInt
intUSART0_TX:   jmp defaultInt
intADC:         jmp defaultInt
intEE_READY:    jmp defaultInt
intACIaddr:	jmp defaultInt
intTWI:         jmp defaultInt
intSPM_READY:   jmp defaultInt

.equ F_CPU,	16000000	; 16 MHz
.equ BAUD,	9600

.equ RX_BYTE,	0x400		; memory address for RX Buffer (1 Byte)

.equ RING_BUFFER_MIN,	0x500
.equ RING_BUFFER_MAX,	RAMEND	; 0x8FF for atmega328 => 1KB buffer size

.equ UBRR_VAL, ((F_CPU+BAUD*8)/(BAUD*16)-1)
.equ BAUD_REAL, (F_CPU/(16*(UBRR_VAL+1)))
.equ BAUD_ERROR, ((BAUD_REAL*1000)/BAUD-1000)

.if ((BAUD_ERROR > 20) || (BAUD_ERROR < -20))
	.error "Systematic error of baud rate larger than 2 percent"
.endif

main:
	cli
	; ****************** Init Stack **********************
	ldi r16, lo8(RAMEND)
	out SPL, r16
	ldi r16, hi8(RAMEND)
	out SPH, r16

	ldi r16, 0xF0	; PD4-PD7 are outputs
			; PD2 is trigger input
			; PD1,PD0 is controlled by USART0

	out DDRD, r16

	; ****************** Setup USART0 ********************
	ldi r16, UBRR_VAL & 0xFF
	sts UBRR0L, r16
	ldi r16, UBRR_VAL >> 8
	sts UBRR0H, r16

	;ldi r16, (1 << U2X0)		; enable double speed mode
	;sts UCSR0A, r16

	ldi r16, (1 << TXEN0) | (1 << RXEN0) | (1 << RXCIE0)
	sts UCSR0B, r16
	ldi r16, (1 << UCSZ01) | (1 << UCSZ00)
					; 8 data bits, no parity, 1 stop bit
					; async mode
	sts UCSR0C, r16

	; *********** Setup Int2 to detect falling edge *********
	ldi r16, (1 << ISC11)		; falling edge
	sts EICRA, r16			; ext int control register A

	ldi r16, (1 << INT1)
	out EIMSK, r16			; ext int mask

	; ****************** Initialize ring buffer *************

	ldi YH, hi8(RING_BUFFER_MIN)
	ldi YL, lo8(RING_BUFFER_MIN)

	; *********************** PWM Setup *********************
	clr r16
	sts OCR1AH, r16			; OCR1A = 0 in CTC => highest
					; possible toggle frequence
					; on OC1A pin
	sts OCR1AL, r16

	sts TCNT1H, r16
	sts TCNT1L, r16			; initial timer counter = 0

;	ldi r16, (1 << COM1A0)		; Toggle OC1A on Compare match
	sts TCCR1A, r16			; enabled only after receiving 'e' later

	ldi r16, (1 << CS10) | (1 << WGM12)	; no prescaling, CTC Mode
	sts TCCR1B, r16

	sei
	; ********************** main loop **********************
mloop:
	lds r16, RX_BYTE		; is there a byte in the buffer?
	tst r16
	breq mloop			; no, loop
	clr r17
	sts RX_BYTE, r17		; yes, delete buffer, content is in r16

	cpi r16, 'f'
	brne skip_set_frequency
	call set_frequency
	jmp mloop
skip_set_frequency:
	cpi r16, 'd'
	brne skip_disable
	;	        *** disable pwm pulses ***
	lds r17, TCCR1A
	andi r17, ~(1 << COM1A0)
	sts TCCR1A, r17
	jmp mloop
skip_disable:
	cpi r16, 'e'
	brne skip_enable
	;	        *** enable pwm pulses ***
	lds r17, TCCR1A
	ori r17, 1 << COM1A0
	sts TCCR1A, r17
	jmp mloop
skip_enable:
	;		   *** read buffer ***
	cpi r16, 'r'
	brne skip_read_buffer
	cli
	mov XH, YH
	mov XL, YL
	cpi XL, 2
	brsh no_underflow
	cpi XH, hi8(RING_BUFFER_MIN)
	brne no_underflow
	; here we have X <= RING_BUFFER_MIN+1
	; which is RING_BUFFER, so set to RING_BUFFER_MAX+1
	ldi XH, hi8(RING_BUFFER_MAX)
	ldi XL, lo8(RING_BUFFER_MAX)
	inc XL
	brcc no_underflow
	inc XH		; adjust for carry
no_underflow:
	ld r17, -X	; last high value
	ld r18, -X	; last low value
	mov YH, XH
	mov YL, XL
	sei
	mov r20, r18
	call putc
	mov r20, r17
	call putc
skip_read_buffer:
	jmp mloop

;******************* set frequency ***********************
; expects two bytes over serial:
;	1st byte: TCNT1L
;	2nd byte: TCNT1H
;
set_frequency:
	clr r17
	ldi r20, 'A'
	call putc
sf_waitlo:
	lds r16, RX_BYTE
	tst r16
	breq sf_waitlo
	mov r18, r16			; low byte
	sts RX_BYTE, r17
sf_waithi:
	lds r16, RX_BYTE
	tst r16
	breq sf_waithi
					; write new compare value
	sts OCR1AH, r16			; received high byte
	sts OCR1AL, r18			; received low byte
	sts TCNT1H, r17
	sts TCNT1L, r17			; initial timer counter = 0
	sts RX_BYTE, r17
	ldi r20, 'A'
	call putc
	ret

;************ put a byte in r20 onto UART0 ***************

putc:
	push r17
wait_tx_ready:
	; flush transmit buffer
	lds r17, UCSR0A			; load status
	sbrs r17, UDRE0			; skip next opcode if data reg is empty
	rjmp wait_tx_ready		; else loop
	sts UDR0, r20			; write to data register
	pop r17
	ret

;************* Interrupt handler for UART0 ***************

isrUSART0RX:
	push r16
	lds r16, UDR0
	sts RX_BYTE, r16
	pop r16
	reti

isrExtInterrupt1:
	push r16
	push r17
	lds r16, TCNT1L			; read current timer L
	lds r17, TCNT1H			; read current timer H
	st Y+, r16
	st Y+, r17
	cpi YH, hi8(RAMEND)
	brne ringbuff_nooverflow	; loop if same or higher
	ldi YH, hi8(RING_BUFFER_MIN)
	ldi YL, lo8(RING_BUFFER_MIN)
ringbuff_nooverflow:
	pop r17
	pop r16
	reti

defaultInt:
	reti

