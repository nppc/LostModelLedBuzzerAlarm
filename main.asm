/*
* We run MCU at default 1Mhz (8mhz and DIV8)

Head lights: 4 LEDs, powered directly from battery.
BEACON LED: который "дублирует" сигнал для/с буззера и/или используется при поиске модели при отстреле аккума.

1. FLIGHT:      Всё включенно: ходовые огни, световая сигнализация, буззер
2. FLIGHT:      Ходовые огни выключены. Включено: световая сигнализация, буззер
3. MAINTANANCE: Ходовые огни и буззер выключены, световая сигнализация включена.
4. MAINTANANCE: Всё выключенно
5. ADJUST BUZZER FREQ

*/

//#define INVERTED_INPUT	; for FCs like CC3D when buzzer controlled by inverted signal (LOW means active)
#define PROGRESSIVE_DELAY	; Enables longer delay with time (Delay: 8 sec, after 5min - 16 sec, after 10 min - 24 sec, after 15 min - 32 sec)
#define DEBUG ; skip one minute delay after power loss

; Mostly for Debugging. 
.equ	DEFAULT_RST_MODE = 1	; 1 - 4

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


; Calculate TMR_COMP_VAL for desired frequency:
; TMR_COMP_VAL = 500000 / Freq(Hz)
; Value should be 10-255
; For Example: To get 2100Hz, TMR_COMP_VAL would be: INT(500000 / 2100) = 238
.EQU	TMR_COMP_VAL 	= 250	; 2 khz (50% duty cycle) at 1mhz clock source

.undef XL
.undef XH
.undef YL
.undef YH
.undef ZL
.undef ZH
; r0-r15 is not available in this tiny mcu series.
.def	z0			= r0	; zero reg
.def	itmp_sreg	= r15	; storage for SREG in interrupts
.def	itmp		= r16	; for using in interrupts
.def	tmp			= r17 	; general temp register
.def	tmp1		= r18 	; general temp register
.def	tmp2		= r19 	; general temp register
.def	buz_on_cntr	= r20	; 0 - buzzer is beeps until pinchange interrupt occurs. 255 - 84ms beep
.def	mute_flags	= r21	; flags for muting buzzer, headlight LEDs and Beacon LED
.equ	MUTE_FLAG_BUZ	= 0	; bit in register mute_flags
.equ	MUTE_FLAG_BLED	= 1	; bit in register mute_flags
.equ	MUTE_FLAG_LEDs	= 2	; bit in register mute_flags

#ifdef PROGRESSIVE_DELAY
.def	beeps_cntr	= r22	; Used in PROGRESSIVE_DELAY mode to count beeps
#endif

.DSEG
;.ORG 0x0040	; start of SRAM data memory
RST_OPTION: 	.BYTE 1	; store here count of reset presses after power-on to determine special modes of operation
CHANGING_MODE:	.BYTE 1 ; flag inidicates that we are CHANGING MODES with RESET button 
COMP_VAL_RAM:	.BYTE 1	; storage for freq value of buzzer.

.CSEG
		rjmp RESET 	; Reset Handler
		reti		; External Interrupt Request 0
		reti            ; just wake up... ; PCINT0 Handler
		reti		; Timer/Counter1 Compare Match A
		reti 		; Timer/Counter1 Overflow
		reti		; Timer0 Overflow Handler try T flag first
		reti		; EEPROM Ready
		reti		; Analog Comparator Handler
		reti		; ADC Conversion Handler
		reti		; Timer/Counter1 Compare Match B
		reti		; Timer0 Compare A Handler (save time for rcall). exits by next reti command
		reti		; Timer0 Compare B Handler
		rjmp WDT_INT	; Watchdog Interrupt Handler
		reti		; USI START
		reti		; USI Overflow

WDT_INT:
	in itmp_sreg, SREG
	in itmp, WDTCR
	sbr itmp, (1 << WDIE)
	out WDTCR, itmp
	out SREG, itmp_sreg
	reti
	
RST_PRESSED: ; we come here when reset button is pressed
		; logic will be:
		; 1. disable buzzer. After battery is disconnected, if user press reset, then it disables beakon functionality until batery is connected back.
		; 2. configure LostModelBuzzer.
		rcall MAIN_CLOCK_1MHZ	; Mode change routine run at 1mhz for simplicity
		rcall WAIT100MS_1MHZ
		; if we are not powered from battery, mute everything, but do not write to EEPROM 
		sbic PINB, V_Inp
		rjmp skp_all_off
		ldi tmp, 4			; go to mode 4 - everything is OFF
		sts RST_OPTION, tmp
		rcall UPDATE_MUTE_FLAGS
skp_all_off:
		; if we are in changing mode, then increnet MODE
		lds tmp, CHANGING_MODE
		sbrs tmp, 0	; skip if bit0 is 1
		rjmp SHOW_CUR_MODE
		; increment counter
		lds tmp, RST_OPTION
		inc tmp
		; loop if pressed too much times
		cpi tmp, 5			; 5 is non existing mode
		brlo SKP_OPT_LOOP
		ldi tmp, 1			; go back to option 1
SKP_OPT_LOOP:
		sts RST_OPTION, tmp
		rcall UPDATE_MUTE_FLAGS

		; show current mode (only with BEACON LED)
SHOW_CUR_MODE:
		lds	tmp, RST_OPTION
SHOWML1:push tmp
		ldi buz_on_cntr, 100 ; load 100 to the buzzer counter (about 30ms)
		push mute_flags		; preserve current mute flags
		ldi mute_flags, (1<<MUTE_FLAG_BUZ)	; Disable buzzer, only BEAKON LED is enabled 
		rcall BEACON_PULSE
		rcall MAIN_CLOCK_1MHZ	; restore 1Mhz mode
		pop mute_flags		; restore mute flas
		rcall WAIT100MS_1MHZ
		rcall WAIT100MS_1MHZ
		pop tmp
		dec tmp
		brne SHOWML1
		; turn on MODE change flag
		ldi tmp, 1
		sts CHANGING_MODE, tmp
		;CHANGING_MODE

		;wait 2 seconds more...
		cbi	PORTB, BLED_Out	; start from OFF
		rcall WAIT100MS_1MHZ	; just small pause to separate it from mode indication
		ldi tmp1, 1	; counter for fade (start from the darkness)
LSM3:	ldi	tmp2, 12 ; delay for fade (about 2 seconds - 12 * 770us * 255)
		; ---- pwm cycle (770us) ----
LSM4:	ldi  tmp, 255 ; software pwm
		sub tmp, tmp1
LSM1: 	dec  tmp
		brne LSM1	; delay for BLED OFF
		sbi	PORTB, BLED_Out	; Turn BLED ON
		add tmp, tmp1
LSM2:	dec	tmp
		brne LSM2	; delay for BLED ON
		cbi	PORTB, BLED_Out	; start from OFF
		; ---- end of pwm cycle (770us) ----
		dec tmp2
		brne LSM4	; stay 
		inc tmp1
		cpi tmp1, 255
		brne LSM3
		
		rcall MAIN_CLOCK_250KHZ	; switch back to econ slow mode
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
		; default Buzzer frequency
		ldi tmp,TMR_COMP_VAL
		STS COMP_VAL_RAM, tmp

		; configure pins
		ldi tmp, 	(1<<BUZZ_Out) | (1<<LEDS_Out) | (1<<BLED_Out)	; set pins as output
		out DDRB,	tmp				; all other pins will be inputs		
		
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

		; determine MODE and configure board accordingly
		; **** TODO read mode from EEPROM 
		
		sei ; Enable interrupts

		sbrc tmp1, EXTRF ; skip next command if reset occurs not by external reset
		rjmp RST_PRESSED
		
		; here we come only after Power ON (not after RESET)
		ldi tmp, DEFAULT_RST_MODE
		sts	RST_OPTION, tmp
		rcall UPDATE_MUTE_FLAGS

		rcall TURN_HEADLIGHTS_ON	; turn on if not muted				
PRG_CONT:
		; if we are here, then change mode operation is finished
		sts CHANGING_MODE, z0
		
;******* MAIN LOOP *******	
MAIN_loop:

		sbis PINB, V_Inp	; if pin is low, then power is disconnected
		rjmp GO_BEACON
		
		; check input pin for state
		clr buz_on_cntr ; if pin on, we are ready
		SKIP_IF_INPUT_OFF	; macro for sbis or sbic command
		;rcall SHORT_BLINK_BEACON
		rcall BEACON_PULSE  ; beep until pin change come
		; go sleep, it will speed up supercap charging a bit...
		rcall WDT_On_8s
		rcall GO_sleep
		; we will wake up on pin change or wdt interrupt
		rjmp MAIN_loop

GO_BEACON:      ; right after power loss we wait a minute, and then beep
		ldi buz_on_cntr, 20 ; very short beep
		rcall BEACON_PULSE

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
		rcall BEACON_PULSE
		rcall WDT_On_250ms	; make small pause...
		rcall GO_sleep ; stops here until wake-up event occurs
		ldi buz_on_cntr, 200 ; load 255 to the buzzer counter (about 84ms)
		rcall BEACON_PULSE
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
		; **** TODO ******
		; enable BUZ or BLED accordingly to mode selected
		cbr mute_flags, (1 << MUTE_FLAG_BUZ) | (1 << MUTE_FLAG_BLED)
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

WAIT25MS_1MHZ:  ; routine that creates delay 25ms at 1Mhz (100ms at 250Khz)
		ldi  tmp, 33
		ldi  tmp1, 119
L1: 	dec  tmp1
		brne L1
		dec  tmp
		brne L1
		ret

WAIT100MS_1MHZ:	; routine that creates delay 100ms at 1Mhz
	rcall WAIT25MS_1MHZ
	rcall WAIT25MS_1MHZ
	rcall WAIT25MS_1MHZ
	rcall WAIT25MS_1MHZ
	ret
		
; Beep the buzzer.
;variable buz_on_cntr determines, will routine beep until PCINT cbange interrupt (0 value), or short beep - max 84ms (255 value)
BEACON_PULSE:
		rcall TURN_BLED_ON	; turn accordingly to mute flag
; seems we never want to skip murte flags anymore
;BEACON_PULSE_ON:; call from here if we want to skip beep mute check... 
		rcall MAIN_CLOCK_1MHZ
		sbrc	mute_flags, MUTE_FLAG_BUZ
		rjmp PWM_loop		; no sound if flag mute_buzz is set
		; enable timer1
		rcall TIMER_ENABLE	; reset timer counter
		; load Compare register to get desired tone freq
		lds tmp, COMP_VAL_RAM
		;ldi tmp, 250
		out OCR1C,tmp

PWM_loop:
		; just some delay... Later we can use some sort of sleep here
		ldi tmp, 128	; about 385us
small_pause:
		dec tmp
		brne small_pause
		
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
		; disable the timer
		rcall TIMER_DISABLE
		rcall MAIN_CLOCK_250KHZ
		rcall TURN_BLED_OFF
		ret

TIMER_ENABLE:
		; enable the timer1
		in tmp, PRR
		cbr tmp, (1 << PRTIM1) ; clear bit
		out PRR, tmp
		; configure timer 1 to work in normal mode, no prescaler
		ldi tmp, (1<<CTC1) | (0<<CS11) | (1<<CS10) ; no prescaller
		out TCCR1, tmp
		out TIMSK, z0
		; reset timer
		out TCNT1, z0
		ldi tmp, (1 << COM1B0) ; toggle PB4 pin
		out GTCCR, tmp
		ldi tmp, 5	; set OCR1B to some value (that is allways smaller than OCR1C)
		out OCR1B, tmp
		;ldi tmp, 255
		out OCR1C, z0 ; set max top value just for initialization. Beep routine will set it to correct one.
		ret

TIMER_DISABLE:
		; disable the timer
		out TIMSK, z0
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
		ldi tmp, (0 << CLKPS3) | (0 << CLKPS2) | (1 << CLKPS1) | (1 << CLKPS0) ;  prescaler is 8 (1Mhz)
		out  CLKPR, tmp
		ret

MAIN_CLOCK_250KHZ:
		; 250Khz (Leave 1 mhz osc with prescaler 4)
		; Write signature for change enable of protected I/O register
		ldi tmp, (1 << CLKPCE)
		out CLKPR, tmp
		ldi tmp, (0 << CLKPS3) | (1 << CLKPS2) | (0 << CLKPS1) | (1 << CLKPS0) ;  prescaler is 32 (250Khz)
		out  CLKPR, tmp
		ret

TURN_HEADLIGHTS_ON:
	sbrs	mute_flags, MUTE_FLAG_LEDs
	sbi	PORTB, LEDS_Out
	ret

TURN_HEADLIGHTS_OFF:
	cbi	PORTB, LEDS_Out
	ret

TURN_BLED_ON:
	sbrs	mute_flags, MUTE_FLAG_BLED
	sbi	PORTB, BLED_Out
	ret

TURN_BLED_OFF:
	cbi	PORTB, BLED_Out
	ret

UPDATE_MUTE_FLAGS:
	lds tmp, RST_OPTION
	cpi tmp, 1
	brne nxt_mode2
	cbr mute_flags, (1 << MUTE_FLAG_BUZ) | (1 << MUTE_FLAG_BLED) | (1 << MUTE_FLAG_LEDs)	
	ret
nxt_mode2:
	cpi tmp, 2
	brne nxt_mode3
	cbr mute_flags, (1 << MUTE_FLAG_BUZ) | (1 << MUTE_FLAG_BLED)
	sbr mute_flags, (1 << MUTE_FLAG_LEDs)
	ret
nxt_mode3:
	cpi tmp, 3
	brne nxt_mode4
	cbr mute_flags, (1 << MUTE_FLAG_BLED)
	sbr mute_flags, (1 << MUTE_FLAG_BUZ) | (1 << MUTE_FLAG_LEDs)
	ret
nxt_mode4:
	sbr mute_flags, (1 << MUTE_FLAG_BUZ) | (1 << MUTE_FLAG_BLED) | (1 << MUTE_FLAG_LEDs)
	ret
