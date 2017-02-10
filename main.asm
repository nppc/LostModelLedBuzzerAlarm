#define INVERTED_INPUT	; for FCs like CC3D when buzzer controlled by inverted signal (LOW means active)
#define PROGRESSIVE_DELAY	; Enables longer delay with time (Delay: 8 sec, after 5min - 16 sec, after 10 min - 24 sec, after 15 min - 32 sec)

/*
 * Author: nppc
 * Hardware design: nppc
 * Contributor to the hardware design: universam
 * 
 * This file is the main routine for the Lost Alarm Buzzer module
 *
 * Lost Alarm Buzzer is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Lost Alarm Buzzer software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
 
#ifdef INVERTED_INPUT
 .MACRO SKIP_IF_INPUT_OFF
	sbis PINB, BUZZ_Inp
 .ENDMACRO
 .MACRO SKIP_IF_INPUT_ON
	sbic PINB, BUZZ_Inp
.ENDMACRO
#else
 .MACRO SKIP_IF_INPUT_OFF
	sbic PINB, BUZZ_Inp
 .ENDMACRO
 .MACRO SKIP_IF_INPUT_ON
	sbis PINB, BUZZ_Inp
.ENDMACRO
#endif
 
.EQU	BUZZ_Out	= PB2	; PWM buzzer output 
.EQU	BUZZ_Inp	= PB1	; BUZZER Input from FC 
.EQU	V_Inp		= PB0	; Input for supply voltage sensing

.EQU	TMR_COMP_VAL 	= 595	; about 3.1 khz (50% duty cycle) at 4mhz clock source
.EQU	PWM_FAST_DUTY	= 20		; Fast PWM (Volume) duty cycle len
.EQU	DEFAULT_VOUME	= 20		; Buzzer volume (1-20)

.undef XL
.undef XH
.undef YL
.undef YH
.undef ZL
.undef ZH
; r0-r15 is not available in this tiny mcu series.
.def	tmp			= r16 	; general temp register
.def	tmp1		= r17 	; general temp register
.def	buz_on_cntr	= r18	; 0 - buzzer is beeps until pinchange interrupt occurs. 255 - 84ms beep
.def	pwm_volume	= r19	; range: 1-20. Variable that sets the volume of buzzer (interval when BUZZ_Out in fast PWM is HIGH)
.def	pwm_counter	= r20	; just a counter for fast PWM duty cycle
.def	itmp_sreg	= r21	; storage for SREG in interrupts
.def	W1_DATA_L	= r22	; L data register for data, received by 1Wire protocol
.def	W1_DATA_H	= r23	; H data register for data, received by 1Wire protocol
.def	icp_d		= r24	; delay from ICP routine (len of the signal)
.def	mute_buzz	= r25	; flag indicates that we need to mute buzzer (after reset manually pressed).
#ifdef PROGRESSIVE_DELAY
.def	beeps_cntr	= r30	; Used in PROGRESSIVE_DELAY mode to count beeps
#endif
.def	z0			= r31	; zero reg
; r30 has the flag (no sound)

.DSEG
.ORG 0x0040	; start of SRAM data memory
RST_OPTION: 	.BYTE 1	; store here count of reset presses after power-on to determine special modes of operation
COMP_VAL_RAM_L:	.BYTE 1	; storage for freq value of buzzer. Can be changed in configuration mode.
COMP_VAL_RAM_H:	.BYTE 1 ; storage for freq value of buzzer. Can be changed in configuration mode.

.CSEG
		rjmp RESET 	; Reset Handler
		reti		;rjmp INT0 	; IRQ0 Handler
		reti            ; just wake up... rjmp PC_int	; PCINT0 Handler
		reti		;rjmp TCAP_int; Timer0 Capture Handler
		set			; Timer0 Overflow Handler try T flag first
		set			; Timer0 Compare A Handler (save time for rcall). exits by next reti command
		reti		; Timer0 Compare B Handler
		reti		; Analog Comparator Handler
		reti		;rjmp WDT_int; Watchdog Interrupt Handler
		reti		; Voltage Level Monitor Handler
		reti		;rjmp ADC_int; ADC Conversion Handler

RST_PRESSED: ; we come here when reset button is pressed
		; logic will be:
		; very first action - is just wait for 100ms for example, to eliminate noise on reset button
		; beep shortly n times
		; Then, increment RST_OPTION variable and wait about 2 seconds in loop.
		; if during that time RESET was pressed again, then variable will just increase.
		; After 2 seconds of waiting check RST_OPTION variable and decide what to do.
		; Currently I think about this options:
		; 1. disable buzzer. After battery is disconnected, if user press reset, then it disables beakon functionality until batery is connected back.
		; 2. configure LostModelBuzzer. with some 1wire simple protocol change parameters. (Actually just test them, then reflash, because of no EEPROM for config).
		;    configurable: Buzzer freq.
		rcall WAIT100MS
		; if we are not powered from battery, do only buzzer mute 
		sbis PINB, V_Inp	; if pin is low, then only buzzer mute can be enabled
		sts RST_OPTION, z0			; clear RESET counter to stay in first option 
		; increment counter
		lds tmp, RST_OPTION
		inc tmp
		; loop if pressed too much times
		cpi tmp, 3			; 3 is non existing mode
		brne SKP_OPT_LOOP
		ldi tmp, 1			; gp back to option 1
SKP_OPT_LOOP:
		sts RST_OPTION, tmp
		; beep n times according to RST_OPTION
L1_BUZ_RST:
		push tmp
		ldi buz_on_cntr, 100 ; load 100 to the buzzer counter (about 30ms)
		rcall BEEP_ON		; ignore mute flag
		rcall WAIT100MS
		pop tmp
		dec tmp
		brne L1_BUZ_RST
		;wait 2 seconds more...
		ldi tmp, 20
L1_RST_WAIT:
		push tmp
		rcall WAIT100MS
		pop tmp
		dec tmp
		brne L1_RST_WAIT
		
		; now decide to what mode to go
		; do we just turn off buzzer?
		lds tmp, RST_OPTION
		cpi tmp, 1
		breq RST_BUZZ_OFF
		; if no buzz off then go to 1 wire transfer mode
		; also go to 1 wire mode, if we are powered from battery
		
		; TODO 1wire protocol
		; configure timer0 for capturing 1w data
		; TCCR0A, TCCR0B and TCCR0C is already configured
W1_L0:	rcall MAIN_CLOCK_4MHZ	; for simplicity lets run this routines allways on 4mhz
		rcall TIMER_ENABLE	; enable timer0 and reset timer counter
		ldi tmp, (1 << ICNC0) | (0 << ICES0) | (0 << WGM02) | (1 << CS00) ; configure ICP mode
		out TCCR0B, tmp
		ldi tmp, (1 << TOIE0); enable Overflow interrupt 
		out TIMSK0, tmp
		ldi tmp, (1 <<ICF0); clear ICF flag
		out TIFR0, tmp
		clt		; clear T flag 
		; Now wait for data (first need to catch timer overflow, indicating that all transfers are finished)
W1_L1:	; Oveflow will be indicated by T flag is set
		brtc W1_L1		; loop here until overflow will come
		clt				; reset overflow flag
		clr W1_DATA_L	; prepare register for receiving 16 bit of data
		clr W1_DATA_H	; prepare register for receiving 16 bit of data
		ldi tmp1, 16	; counter for receiving bits
		; now wait for the first ICP, it will be the beginning of the first bit.		

		;ldi buz_on_cntr, 30 ; debug beep
		;rcall BEEP_ON		; ignore mute flag
		;rjmp W1_L0

W1_L2:	brts W1_L0		; start over, desync or no data came
		in tmp, TIFR0
		sbrs tmp, ICF0
		rjmp W1_L2		; loop
		out TCNT0H, z0	; clear counter
		out TCNT0L, z0
		ldi tmp, (1 <<ICF0)	; clear ICF flag
		out TIFR0, tmp		; clear ICF flag
W1_L3:	brts W1_L0		; Now wait for end bit strobe
		in tmp, TIFR0
		sbrs tmp, ICF0
		rjmp W1_L3		; loop
		; read ICP register
		in icp_d, ICR0L
		in icp_d, ICR0H	; we are interested in High byte only
		ldi tmp, (1 <<ICF0)	; clear ICF flag
		out TIFR0, tmp		; clear ICF flag
		; clear timer0 to get new timing... need to check, maybe it is does automatically
		out TCNT0H, z0
		out TCNT0L, z0
		; timing to check
		;low bit - 4ms = 64, 6ms = 96
		;high bit - 9ms = 144, 11ms = 176
		cpi icp_d, 64
		brlo W1_L0		; desync or corrupted data - start over
		cpi icp_d, 176
		brsh W1_L0		; desync or corrupted data - start over
		cpi icp_d, 96
		brlo W1_LOW		; low bit received
		cpi icp_d, 144
		brlo W1_L0		; desync or corrupted data - start over
		; high bit received
		sec				; set C flag
		rjmp W1_CONT
W1_LOW:	clc				; clear C flag
W1_CONT:ror W1_DATA_H	; shift right one bit with C
		ror W1_DATA_L	; shift right one bit with C
		dec tmp1		; go tro next bit
		brne W1_L3		; loop until all data received
		; all data received
		; it is frequency value
		; adjust frequency variable
		STS COMP_VAL_RAM_L, W1_DATA_L
		STS COMP_VAL_RAM_H, W1_DATA_H
		rcall TIMER_DISABLE
		; make sample beep
		ldi buz_on_cntr, 255 ; load 255 to the buzzer counter (about 84ms)
		rcall BEEP_ON	; skip mute check
		rjmp W1_L0		; back to listen for 1Wire protocol
RST_BUZZ_OFF:
		ldi mute_buzz, 1
		rjmp PRG_CONT	; back to main program

; start of the program
RESET: 	
		cli
		; determine why we are here (by power-on or by reset pin goes to low)
		in tmp1, RSTFLR	; tmp1 should not be changed til sbrc tmp1, EXTRF
		
		ldi tmp, high (RAMEND) ; Main program start
		out SPH,tmp ; Set Stack Pointer
		ldi tmp, low (RAMEND) ; to top of RAM
		out SPL,tmp
		
		rcall MAIN_CLOCK_250KHZ	; set main clock to 250KHZ...
		
		; initialize variables
		clr z0				; general 0 value register
		ldi pwm_volume, DEFAULT_VOUME	; Volume of buzzer 1-20
		clr mute_buzz		; by default buzzer is ON
		; default Buzzer frequency
		ldi tmp, low(TMR_COMP_VAL)
		STS COMP_VAL_RAM_L, tmp
		ldi tmp, high(TMR_COMP_VAL)
		STS COMP_VAL_RAM_H, tmp

		; configure pins
		ldi tmp, 	(1 << BUZZ_Out)	; set pin as output
		out DDRB,	tmp				; all other pins will be inputs		
		; If input is not inverted we need external pull-down resistor about 50K
		#ifdef INVERTED_INPUT
		ldi tmp, 	(1 << BUZZ_Inp)	; enable pull-up to protect floating input when no power on FC
		out PUEB,	tmp				; 
		out PORTB, tmp				; all pins to LOW except pull-up
		#endif
		
		#ifndef _TN9DEF_INC_
		; Disable ADC
		in tmp, PRR
		sbr tmp, (1 << PRADC) ; set bit to disable ADC
		out PRR, tmp
		#endif

		out TCCR0A, z0
		rcall TIMER_DISABLE ; disable timer0 for now

		; disable analog comparator
		ldi	tmp, (1 << ACD)	; analog comp. disable
		out ACSR, tmp			; disable power to analog comp.

		; Configure Pin Change interrupt for BUZZER input
		ldi tmp, 	(1 << BUZZ_Inp)
		out PCMSK, 	tmp	; configure pin for ext interrupt
		sbi PCICR, PCIE0	; pin change interrupt enable
						
		sei ; Enable interrupts

		out RSTFLR, z0	; reset all reset flags 

		sbrc tmp1, EXTRF ; skip next command if reset occurs not by external reset
		rjmp RST_PRESSED

		; TEST ***************************
		;rjmp W1_L0
PRG_CONT:
		
;******* MAIN LOOP *******	
MAIN_loop:
		; here we should clear SRAM variable, that counts reset presses...
		sts RST_OPTION, z0

		sbis PINB, V_Inp	; if pin is low, then power is disconnected
		rjmp GO_BEACON
		
		; check input pin for state
		clr buz_on_cntr ; if pin on, we are ready
		SKIP_IF_INPUT_OFF	; macro for sbis or sbic command
		rcall BEEP  ; beep until pin change come
		; go sleep, it will speed up supercap charging a bit...
		rcall WDT_On_8s
		rcall GO_sleep
		; we will wake up on pin change or wdt interrupt
		rjmp MAIN_loop

GO_BEACON:      ; right after power loss we wait a minute, and then beep
		ldi buz_on_cntr, 40 ; load 255 to the buzzer counter (about 84ms)
		rcall BEEP_ON

		#ifdef PROGRESSIVE_DELAY
		clr beeps_cntr		; prepare counter for beeps in PROGRESSIVE_DELAY mode
		#endif

		ldi tmp, 8 ; about 1 minute
BEAC_WT1:	
		push tmp
		rcall WDT_On_8s
		rcall GO_sleep
		pop tmp
		sbic PINB, V_Inp	; if pin is high, then power is connected, go out from Beacon mode
		rjmp BEAC_EXIT
		dec tmp
		brne BEAC_WT1
		
BEAC_L1:ldi buz_on_cntr, 200 ; load 255 to the buzzer counter (about 84ms)
		rcall BEEP
		rcall WDT_On_250ms	; make small pause...
		rcall GO_sleep ; stops here until wake-up event occurs
		ldi buz_on_cntr, 200 ; load 255 to the buzzer counter (about 84ms)
		rcall BEEP
#ifdef PROGRESSIVE_DELAY
		inc beeps_cntr	; inclrement counter. It should not overflow, because we will have very long delays at the end
		ldi tmp, 1			; 8sec x 1 pause
		cpi beeps_cntr, 37	; 5 minutes
		brlo BEAC_GO
		ldi tmp, 2			; 8 sec x 2 pause
		cpi beeps_cntr, 55	; 10 minutes
		brlo BEAC_GO
		ldi tmp, 3			; 8 sec x 3 pause
		cpi beeps_cntr, 65	; 15 minutes
		brlo BEAC_GO
		ldi tmp, 4			; 8 sec x 4 pause
BEAC_GO:push tmp
		rcall WDT_On_8s
		rcall GO_sleep ; stops here until wake-up event occur
		pop tmp
		sbic PINB, V_Inp	; if pin is low, then power is not connected, stay in Beacon mode
		rjmp BEAC_EXIT		; Jump out, because power is connected
		dec tmp
		brne BEAC_GO		; loop for pause
		; if counter 0 then just continue...		
#else
		rcall WDT_On_8s
		rcall GO_sleep ; stops here until wake-up event occur
		sbis PINB, V_Inp	; if pin is low, then power is connected, stay in Beacon mode
#endif
		rjmp BEAC_L1
BEAC_EXIT:
		; go back to main loop - battery connected
		; turn mute off (in case buzzer was muted)
		clr mute_buzz	; buzzer should not be muted after going back to normal mode
		;sts RST_OPTION, z0
		rjmp MAIN_loop 
;******* END OF MAIN LOOP *******	

	

WDT_On_250ms:
		ldi tmp1, (0<<WDE) | (1<<WDIE) | (1<<WDIF) | (0 << WDP3) | (1 << WDP2) | (0 << WDP1) | (0 << WDP0) ; 0.25 sec, interrupt enable
		rjmp WDT_On
;WDT_On_4s:
;		ldi tmp1, (0<<WDE) | (1<<WDIE) | (1<<WDIF) | (1 << WDP3) | (0 << WDP2) | (0 << WDP1) | (0 << WDP0) ; 4 sec, interrupt enable
;		rjmp WDT_On
WDT_On_8s:
		ldi tmp1, (0<<WDE) | (1<<WDIE) | (1<<WDIF) | (1 << WDP3) | (0 << WDP2) | (0 << WDP1) | (1 << WDP0) ; 8 sec, interrupt enable
WDT_On:	wdr					; reset the WDT
		; Clear WDRF in RSTFLR
		in tmp, RSTFLR
		andi tmp, ~(1<<WDRF)
		out RSTFLR, tmp		; Write signature for change enable of protected I/O register
		ldi tmp, 0xD8
		out CCP, tmp
		out  WDTCSR, tmp1	; preset register
		wdr
		ret	; End WDT_On
		

GO_sleep:
		; Configure Sleep Mode
		ldi tmp, (1<<SE) | (0<<SM2) | (1<<SM1) | (0<<SM0)	; enable power down sleep mode
		out SMCR, tmp
		SLEEP
		; stops here until wake-up event occurs
		ret

;WAIT100MS:  ; routine that creates delay 100ms at 4mhz
;		rcall WAIT50MS
;WAIT50MS:	; routine that creates delay 50ms at 4mhz
;		ldi  tmp, 255
;		ldi  tmp1, 139
;WT50_1: dec  tmp1
;		brne WT50_1
;		dec  tmp
;		brne WT50_1
;		ret
WAIT100MS:  ; routine that creates delay 100ms at 250KHZ
		ldi  tmp, 33
		ldi  tmp1, 119
L1: 	dec  tmp1
		brne L1
		dec  tmp
		brne L1
		ret
		
; Beep the buzzer.
;variable buz_on_cntr determines, will routine beep until PCINT cbange interrupt (0 value), or short beep - max 84ms (255 value)
BEEP:
		cp mute_buzz, z0
		brne PWM_exit		; no sound if flag mute_buzz is set
		; set volume
BEEP_ON:; call from here if we want to skip beep mute check... 
		rcall MAIN_CLOCK_4MHZ
		; enable timer0
		rcall TIMER_ENABLE	; reset timer counter
		ldi tmp, (1 << OCIE0A) ; enable compare interrupt 
		out TIMSK0, tmp
		; load Compare register to get 3khz
		; Set OCR0A to 645 - about 3khz at 4mhz clock (50% duty cycle)
		lds tmp1, COMP_VAL_RAM_H
		lds tmp, COMP_VAL_RAM_L
;		cli	; no needed to disable interrupts, they only set T flag, but anyway we clear it 
		out OCR0AH,tmp1
		out OCR0AL,tmp
		ldi tmp, (1 <<OCF0A); clear OCF0A flag
		out TIFR0, tmp
;		sei
		clt		; clear T flag

PWM_loop:
		; PWM the buzzer at 50% duty cycle
		sbi PORTB, BUZZ_Out 			;turn buzzer ON
		mov pwm_counter, pwm_volume		;Initialize counter
PWM_F1:	brts PWM_low					;Jump out if T flag is set (Compare Match)
		dec pwm_counter					;count value for pin ON for Buzzer volume regulation
		brne PWM_F1
		ldi pwm_counter, PWM_FAST_DUTY	;initialize couner for remaining cycle
 		sub pwm_counter, pwm_volume		;adjust counter to correct value
		breq PWM_loop					;if volume at max (pwm_dutyfst=pwm_volume) then do not turn buzzer pin low.
		cbi PORTB, BUZZ_Out 			;turn buzzer OFF
PWM_F2:	brts PWM_low					; Jump out if T flag is set (Compare Match)
		dec pwm_counter					;count value for pin OFF for Buzzer volume regulation
		brne PWM_F2
		rjmp PWM_loop					;loop untill exit by Timer Compare match (T flag)
PWM_low:; now second part of 3khz duty cycle
		cbi PORTB, BUZZ_Out	; PWM in low state
		clt		; reset timer capture match flag
PWM_L2:
		brts PWM_loop_cycle_end				;Jump out if T flag is set (Compare Match)
		rjmp PWM_L2 ; otherwise just wait here. 
; 3khz 50% duty cycle transition		
PWM_loop_cycle_end:
		clt				; clear T flag ("Compare Match" flag clear)
		cpi buz_on_cntr, 0
		breq chck_pcint		; go to routine to check, does PC_int (pin change interrupt) occurs?
		dec buz_on_cntr
		breq PWM_loop_exit	; Stop Buzzer beep
		rjmp PWM_loop
chck_pcint:		
		; we also need to check voltage readings for voltage drop, if, for example, power will be disconnected while beep...
		SKIP_IF_INPUT_ON	; macro for sbis or sbic command
		rjmp PWM_loop_exit
		sbic PINB, V_Inp	; if pin is high, stay in this beep loop
		rjmp PWM_loop
; here we finish our handmade PWM routine for buzzer.
PWM_loop_exit:
		rcall MAIN_CLOCK_250KHZ
		; disable the timer
		rcall TIMER_DISABLE
; ***** END OF MANUAL PWM ROUTINE ******
PWM_exit:
ret

TIMER_ENABLE:
		; enable the timer
		in tmp, PRR
		cbr tmp, (1 << PRTIM0) ; clear bit
		out PRR, tmp
		; configure timer 0 to work in CTC mode (4), no prescaler
		ldi tmp, (0 << ICNC0) | (0 << ICES0) | (1 << WGM02) | (1 << CS00) ; also preconfigure ICP mode
		out TCCR0B, tmp
		; reset timer
		out TCNT0H, z0
		out TCNT0L, z0
		ret

TIMER_DISABLE:
		; disable the timer
		out TCCR0B, z0	; stop timer before turning it off
		in tmp, PRR
		sbr tmp, (1 << PRTIM0) ; set bit
		out PRR, tmp
		ret

MAIN_CLOCK_4MHZ:
		; 4Mhz (Leave 8 mhz osc with prescaler 2)
		; Write signature for change enable of protected I/O register
		ldi tmp, 0xD8
		out CCP, tmp
		ldi tmp, (0 << CLKPS3) | (0 << CLKPS2) | (0 << CLKPS1) | (1 << CLKPS0) ;  prescaler is 2 (4mhz)
		out  CLKPSR, tmp
		ret

MAIN_CLOCK_250KHZ:
		; 250khz (Leave 8 mhz osc with prescaler 32)
		; Write signature for change enable of protected I/O register
		ldi tmp, 0xD8
		out CCP, tmp
		ldi tmp, (0 << CLKPS3) | (1 << CLKPS2) | (0 << CLKPS1) | (1 << CLKPS0) ;  prescaler is 32 (250khz)
		out  CLKPSR, tmp
		ret
