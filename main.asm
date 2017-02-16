/*
* We run MCU at default 1Mhz (8mhz and DIV8)
*/

//#define INVERTED_INPUT	; for FCs like CC3D when buzzer controlled by inverted signal (LOW means active)
#define PROGRESSIVE_DELAY	; Enables longer delay with time (Delay: 8 sec, after 5min - 16 sec, after 10 min - 24 sec, after 15 min - 32 sec)
//#define FREQ_GEN	; procedure to beep on different freq via 1 wire uart protocol 
//#define DEBUG ; skip one minute delay after power loss


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
 
.EQU	BUZZ_Out	= PB4	; PWM buzzer output 
.EQU	BUZZ_Inp	= PB0	; BUZZER Input from FC 
.EQU	BLED_Out	= PB3	; BEACON LED output 
.EQU	V_Inp		= PB2	; Input for supply voltage sensing
.EQU	LEDS_Out	= PB1	; LEDS output 


; 595 - 3.1khz
.EQU	TMR_COMP_VAL 	= 250	; about 2 khz (50% duty cycle) at 1mhz clock source

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
;.ORG 0x0040	; start of SRAM data memory
RST_OPTION: 	.BYTE 1	; store here count of reset presses after power-on to determine special modes of operation
COMP_VAL_RAM:	.BYTE 1	; storage for freq value of buzzer.

.CSEG
		rjmp RESET 	; Reset Handler
		reti		; External Interrupt Request 0
		reti            ; just wake up... ; PCINT0 Handler
		reti		; Timer/Counter1 Compare Match A
		reti 		; Timer/Counter1 Overflow
		set		; Timer0 Overflow Handler try T flag first
		reti		; EEPROM Ready
		reti		; Analog Comparator Handler
		reti		; ADC Conversion Handler
		reti		; Timer/Counter1 Compare Match B
		set		; Timer0 Compare A Handler (save time for rcall). exits by next reti command
		reti		; Timer0 Compare B Handler
		rjmp WDT_INT	; Watchdog Interrupt Handler
		reti		; USI START
		reti		; USI Overflow

WDT_INT:
	in itmp_sreg, SREG
	in tmp, WDTCR
	sbr tmp, (1 << WDIE)
	out WDTCR, tmp
	out SREG, itmp_sreg
	reti
	
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
		ldi tmp, 1			; go back to option 1
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


RST_BUZZ_OFF:
		ldi mute_buzz, 1
		rjmp PRG_CONT	; back to main program

; start of the program
RESET: 	
		cli
		; determine why we are here (by power-on or by reset pin goes to low)
		in tmp1, MCUSR	; tmp1 should not be changed til sbrc tmp1, EXTRF
		clr z0				; general 0 value register
		; reset RESET flag
		out MCUSR, z0
		
		; Initialize Stack pointer
		ldi tmp, high (RAMEND) ; Main program start
		out SPH,tmp ; Set Stack Pointer
		ldi tmp, low (RAMEND) ; to top of RAM
		out SPL,tmp
		
		rcall MAIN_CLOCK_250KHZ	; set main clock to 250KHZ...
		
		; initialize variables
		clr mute_buzz		; by default buzzer is ON
		; default Buzzer frequency
		ldi tmp,TMR_COMP_VAL
		STS COMP_VAL_RAM, tmp

		; configure pins
		ldi tmp, 	(1<<BUZZ_Out) | (1<<LEDS_Out) | (1<<BLED_Out)	; set pins as output
		out DDRB,	tmp				; all other pins will be inputs		
		; If input is not inverted we need external pull-down resistor about 50K
		;#ifdef INVERTED_INPUT
		;ldi tmp, 	(1 << BUZZ_Inp)	; enable pull-up to protect floating input when no power on FC
		;out PUEB,	tmp				; 
		;out PORTB, tmp				; all pins to LOW except pull-up
		;#endif
		
		out PORTB, z0				; all pins to LOW
		
		; Disable ADC
		out ADCSRA, z0
		
		; Disable some other pereperials clock (Timer0, USI, ADC)
		ldi tmp, (1<<PRTIM0) | (1<<PRUSI) | (1<<PRADC)
		out PRR, tmp

		rcall TIMER_DISABLE ; disable timer1 for now

		; disable analog comparator
		ldi	tmp, (1 << ACD)	; analog comp. disable
		out ACSR, tmp			; disable power to analog comp.

		; Configure Pin Change interrupt for BUZZER controll input
		ldi tmp, (1 << BUZZ_Inp)
		out PCMSK, tmp	; configure pin for ext interrupt
		ldi tmp, (1 << PCIE)
		out GIMSK, tmp	; pin change interrupt enable
						
		sei ; Enable interrupts

		;out MCUSR, z0	; reset all reset flags 

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
#ifdef DEBUG
		ldi tmp, 1 ; about 8 seconds for debugging
#endif			
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
WDT_On:	wdr	; reset the WDT
		; first, enable writing to the WD control register
		in tmp, WDTCR
		ori tmp, (1<<WDCE) | (1<<WDE)
		out WDTCR, tmp
		; Now we can write new configuration and reset watchdog
		out WDTCR, tmp1	; tmp1 previously set
		wdr
		ret	; End WDT_On
		

GO_sleep:
		; Configure Sleep Mode
		ldi tmp, (1<<SE) | (1<<SM1) | (0<<SM0)	; enable power down sleep mode
		out MCUCR, tmp
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
BEEP_ON:; call from here if we want to skip beep mute check... 
		rcall MAIN_CLOCK_1MHZ
		; enable timer1
		rcall TIMER_ENABLE	; reset timer counter
		; load Compare register to get desired tone freq
		lds tmp, COMP_VAL_RAM
		out OCR1C,tmp

PWM_loop:
		; just some delay... Later we can use some sort of sleep here
		ldi tmp, 255
		dec tmp
		brne PWM_loop
		
		cpi buz_on_cntr, 0
		breq chck_pcint		; go to routine to check, does PC_int (pin change interrupt) occurs?
		dec buz_on_cntr
		breq PWM_loop_exit	; Stop Buzzer beep
		rjmp PWM_loop
		
		; we also need to check voltage readings for voltage drop, if, for example, power will be disconnected while beep...
chck_pcint:	SKIP_IF_INPUT_ON	; macro for sbis or sbic command
		rjmp PWM_loop_exit
		sbic PINB, V_Inp	; if pin is high, stay in this beep loop
		rjmp PWM_loop
; here we finish our routine for buzzer.
PWM_loop_exit:
		rcall MAIN_CLOCK_250KHZ
		; disable the timer
		rcall TIMER_DISABLE
; ***** END OF MANUAL PWM ROUTINE ******
PWM_exit:
		ret

TIMER_ENABLE:
		; enable the timer1
		in tmp, PRR
		cbr tmp, (1 << PRTIM1) ; clear bit
		out PRR, tmp
		; configure timer 1 to work in normal mode, no prescaler
		ldi tmp, (1<<CTC1) | (1<<CS10) ; no prescaller
		out TCCR1, tmp
		ldi tmp, (1 << COM1B0) ; toggle PB4 pin
		out GTCCR, tmp
		ldi tmp, 1	; set OCR1B to some value (that is allways smaller than OCR1C)
		out OCR1B, tmp
		ldi tmp, 255
		out OCR1C, tmp ; set max top value just for initialization. Beep routine will set it to correct one.
		; initialize interrupts - no interrupts at the moment
		;ldi tmp, (1<<OCIE0A) | (1<<TOIE0)
		;out TIMSK, tmp
		; reset timer
		out TCNT1, z0
		ret

TIMER_DISABLE:
		; disable the timer
		out TCCR1, z0
		out GTCCR, z0	; stop timer before turning it off
		; stop clock to timer0 module
		in tmp, PRR
		sbr tmp, (1 << PRTIM1) ; set bit
		out PRR, tmp
		ret

MAIN_CLOCK_1MHZ:
		; 1Mhz (Leave 1 mhz osc with prescaler 1)
		; Write signature for change enable of protected I/O register
		ldi tmp, (1 << CLKPCE)
		out CLKPR, tmp
		ldi tmp, (0 << CLKPS3) | (0 << CLKPS2) | (0 << CLKPS1) | (0 << CLKPS0) ;  prescaler is 1 (1Mhz)
		out  CLKPR, tmp
		ret

MAIN_CLOCK_250KHZ:
		; 250Khz (Leave 1 mhz osc with prescaler 4)
		; Write signature for change enable of protected I/O register
		ldi tmp, (1 << CLKPCE)
		out CLKPR, tmp
		ldi tmp, (0 << CLKPS3) | (0 << CLKPS2) | (1 << CLKPS1) | (0 << CLKPS0) ;  prescaler is 4 (250Khz)
		out  CLKPR, tmp
		ret
