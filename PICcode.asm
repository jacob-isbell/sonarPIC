	INCLUDE	"P18F2620.inc"
;This is the code used to control the sonar system from a PIC18F2620.
;
; - will transmit up to 32 obstacles on all 3 receivers to
; 
; PORT A = AN0, AN1, AN2 (pins AN0=2, AN1=3, AN2=4)	
; PORT B = sonar trigger port (pins B0=21 : B7=28)
; PORT C = 6, 7 serial port
; 
; This version pings all sonars, and determines which channel has the highest signal.
; When the echo's positive derivative is above
; a threshold level the starting range counter value is stored
; up to thirty targets will be kept.
; upon completion of the ping sampling, the run-length data is transmitted from the serial port.
; there is a starting 0xFF byte, # of objects, N x (RNG,ANG,MAG), checksum with 0xFF mapped to 0x00

; portb, 0 - init, evaluation, "markit" flag
; portb, 1 -  sampling flag
; portb, 2 - flag to indicate derivative above the threshold

; portb, 3 being used to trigger the transducers.

; portc, 2 - proximity flag - LED

	
; PIC18F2620 Configuration Bit Settings
; ASM source line config statements
; CONFIG1H
  CONFIG  OSC = INTIO67         ; Oscillator Selection bits (Internal oscillator block, port function on RA6 and RA7)
  CONFIG  FCMEN = OFF           ; Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
  CONFIG  IESO = OFF            ; Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

; CONFIG2L
  CONFIG  PWRT = ON             ; Power-up Timer Enable bit (PWRT enabled)
  CONFIG  BOREN = SBORDIS       ; Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
  CONFIG  BORV = 3              ; Brown Out Reset Voltage bits (Minimum setting)

; CONFIG2H
  CONFIG  WDT = OFF             ; Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
  CONFIG  WDTPS = 32768         ; Watchdog Timer Postscale Select bits (1:32768)

; CONFIG3H
  CONFIG  CCP2MX = PORTC        ; CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
  CONFIG  PBADEN = OFF          ; PORTB A/D Enable bit (PORTB<4:0> pins are configured as digital I/O on Reset)
  CONFIG  LPT1OSC = OFF         ; Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
  CONFIG  MCLRE = OFF           ; MCLR Pin Enable bit (RE3 input pin enabled; MCLR disabled)

; CONFIG4L
  CONFIG  STVREN = ON           ; Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
  CONFIG  LVP = OFF             ; Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
  CONFIG  XINST = OFF           ; Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

; CONFIG5L
  CONFIG  CP0 = OFF             ; Code Protection bit (Block 0 (000800-003FFFh) not code-protected)
  CONFIG  CP1 = OFF             ; Code Protection bit (Block 1 (004000-007FFFh) not code-protected)
  CONFIG  CP2 = OFF             ; Code Protection bit (Block 2 (008000-00BFFFh) not code-protected)
  CONFIG  CP3 = OFF             ; Code Protection bit (Block 3 (00C000-00FFFFh) not code-protected)

; CONFIG5H
  CONFIG  CPB = OFF             ; Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
  CONFIG  CPD = OFF             ; Data EEPROM Code Protection bit (Data EEPROM not code-protected)

; CONFIG6L
  CONFIG  WRT0 = OFF            ; Write Protection bit (Block 0 (000800-003FFFh) not write-protected)
  CONFIG  WRT1 = OFF            ; Write Protection bit (Block 1 (004000-007FFFh) not write-protected)
  CONFIG  WRT2 = OFF            ; Write Protection bit (Block 2 (008000-00BFFFh) not write-protected)
  CONFIG  WRT3 = OFF            ; Write Protection bit (Block 3 (00C000-00FFFFh) not write-protected)

; CONFIG6H
  CONFIG  WRTC = OFF            ; Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
  CONFIG  WRTB = OFF            ; Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
  CONFIG  WRTD = OFF            ; Data EEPROM Write Protection bit (Data EEPROM not write-protected)

; CONFIG7L
  CONFIG  EBTR0 = OFF           ; Table Read Protection bit (Block 0 (000800-003FFFh) not protected from table reads executed in other blocks)
  CONFIG  EBTR1 = OFF           ; Table Read Protection bit (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
  CONFIG  EBTR2 = OFF           ; Table Read Protection bit (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)
  CONFIG  EBTR3 = OFF           ; Table Read Protection bit (Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks)

; CONFIG7H
  CONFIG  EBTRB = OFF           ; Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)

; Set Data Storage
CH1B	equ		0x20  ; previous transducer value storage
CH2B	equ		0x21
CH3B	equ		0x22

CH1		equ		0x25  ; current transducer value storage
CH2		equ		0x26
CH3		equ		0x27

CNTR	equ		0x2A		; low byte of the 2-byte counter
CNTRH	equ		0x2B		; high byte of the 2-byte counter
CNT1	equ		0x2C
CNT2	equ		0x2D
CNT3	equ		0x2E
CYCNT	equ		0x2F
TMP		equ		0x30
TMP2	equ		0x31		; reference address for the RANGE DATA

RNG1	equ		0x32		; target range memory
RNG32	equ		0x51		; reference address for the highbyte of the RANGE DATA

RNG1H	equ		0x52
RNG32H	equ		0x71		; reference address for the MAG data sensor 1

MG1_1	equ		0x72		; target Magnitude Left memory
MG1_32	equ		0x91		; reference address for the MAGnitude DATA sensor 2

MG2_1	equ		0x92		; target Magnitude Center memory
MG2_32	equ		0xB1		; reference address for MAGnitude DATA sensor 3

MG3_1	equ		0xB2		; target magnitude Right memory
MG3_32	equ		0xD1		; reference address for sensor 4 

FLAG	equ		0xD2		; echo continuation flag
TCNT	equ		0xD3		; target counter
MAX		equ		0xD4
MAXI	equ		0xD5
THRESH	equ		0xD6
RNGTH	equ		0xD7		; indicator LED range threshold
SPATT	equ		0xD8		; sonar firing pattern
IPIN	equ		0xD9
MAX2	equ		0xDA
MAX2I	equ		0xDB

		org		0

		bsf		OSCCON, 4	; set the on-board oscillator frequency
		bsf		OSCCON, 5
		bsf		OSCCON, 6

		call	ipi			; wait some time for the power supply to settle
		call	ipi			; before starting any pinging
		call	ipi
		call	ipi
		call	ipi
		call	ipi

		clrf	PORTA		; clear outputs
		movlw	0xFF		; make PORT A all inputs
		movwf	TRISA
		clrf	PORTB		; port B
		movlw	0x00
		movwf	TRISB		; set port B as digital output
		clrf	PORTB		; clear outputs
		bsf		PORTB, 0	; turn on portB 0 to indicate beginning of init

		bcf		TRISC, 2	; PORTC, pins 2 and 3 will be used for range indicators
		bcf		TRISC, 3	; outputs!

; now set up the A/D converter
		bsf		ADCON0, 0	; bit 0 turns on the A/D
		movlw	0x0A		;
		movwf	ADCON1		; 0000 1010 = use AN0, AN1, AN2, AN3, AN4 0V and Vdd voltage refs
		movlw	0x10		; 0 0 010 000 = 4TAD, Fosc/2, left justified
		movwf	ADCON2

; setup and initialize the serial port
; The serial port uses PORT C - RC6 and RC7
; serial port enable bit
		bsf RCSTA, 7	; SPEN bit set (= 1)
		bsf	TRISC, 7	; set bit RC7 to be input
		bsf	TRISC, 6	; set bit RC6 to be input
; The EUSART control will automatically reconfigure the pin from input to output as necessary.
; the operation of the EUSART module is controlled through TXSTA, RCSTA, and BAUDCON
; Configure the TXSTA - the transmit status and control register
		bcf	TXSTA, 6	; selects 8-bit transmission (TX9)
		bsf TXSTA, 5	; transmit enabled (TXEN)
		bcf	TXSTA, 4	; selects asynchronous mode (SYNC)
		bsf	TXSTA, 2	; selects high-speed baud rate (BRGH)

; Configure the RCSTA - receive status and control register
		bcf	RCSTA, 6	; selects 8-bit reception (RX9)
		bsf	RCSTA, 4	; enables continuous receive mode (CREN)
		bcf	RCSTA, 3	; disables address detection (needs to be in 9-bit mode anyway)

; We are expecting the chip to be run on the internal oscillator which is at 8 MHz.
		bcf		BAUDCON, 3	; (BRG16 = 0) - part of setting the baud rate
		movlw	0x19	; movlw	0x33	; load the decimal value 25 (19200 baud)
		movwf	SPBRG	; send it to the baud rate generator low-byte
		bcf		BAUDCON, 0	; disable auto-baud detection (ABDEN)
		bcf		BAUDCON, 1	; disable wakeup (WUE)

		movlw	0x0A		; 
		movwf	THRESH

		movlw	0x18		; range threshold for the LED
		movwf	RNGTH

		movlw	0x19		; default duration 2, 25 pulses is the maximum
		movwf	CYCNT		; this duration is effectively the volume of the pulse
		
		movlw	0x10		; 0000 1000   x08 x10 x20 x40 x80
		movwf	SPATT		; default sonar bit pattern for transmission 

		movlw	0x50
		movwf	IPIN		; ipi count

		bcf		PORTB, 0	; done initializing
		
; - 
; - Loop:  A/D baseline, ping, repetitive A/D, analyze, communicate
; -

loop	bcf		PORTC, 2	; clear the proximity flag

		clrf	TCNT		; clear the target counter
		clrf	FLAG		; clear the continuation flag

; --------- begin firing pulse -------

ldpatt	movf	SPATT, 0	; load up bit pattern
ping	movwf	PORTB		; turn on trigger
base	movlw	0x03
		movwf	TMP
b1		decf	TMP, 1
		btfss	STATUS, Z
		goto	b1
		movff	CYCNT, TMP	; tmp is the working counter for cycle counter
pcnt	movlw	0x0A
		movwf	TMP2
p1		decf	TMP2, 1
		btfss	STATUS, Z
		goto	p1
		nop
		nop
		nop
		nop
		decf	TMP, 1		; decrement and leave in TMP
		btfss	STATUS, Z
		goto	pcnt

		clrf	PORTB		; shut off trigger pulse

		movlw	0x08		; deadtime counter
		movwf	CNTR		; low byte of the counter - temporary use
dtloop	call	qmsdel
		decf	CNTR, 1
		btfss	STATUS, Z
		goto	dtloop

		call	sampAD		; storing up an initial value
		movff	CH1, CH1B
		movff	CH2, CH2B
		movff	CH3, CH3B
	
		call	emsdel

; --------- Sampling the Transducers -------
		clrf	CNTR		; clear the low-byte of the 2-byte counter
		clrf	CNTRH		; clear the high-byte of the 2-byte counter
sloop	incf	CNTR, 1
		movlw	0xFE		; compare counter with max value = 0xFF
		cpfseq	CNTR
		bra		samp
		incf	CNTRH, 1
		movlw	0x01
		cpfseq	CNTRH
		bra		samp
		goto	eval

samp	call	sampAD

		movf	THRESH, 0	; load thresh
		subwf	CH1, 0		; load CH1, leave CH1 unaltered
		cpfsgt	CH1B		; compare with old value
		goto	found
		movf	THRESH, 0
		subwf	CH2, 0		; load CH2 - subtracting the threshold
		cpfsgt	CH2B		; compare with old value
		goto	found	
		movf	THRESH, 0
		subwf	CH3, 0		; load CH3
		cpfsgt	CH3B		; compare with threshold
		goto	found

				
		; otherwise
		bcf		PORTB, 2	; turn off flag - no target found
		movlw	0x00		; ---> OK, no rising signals - was the FLAG previously on?
		cpfseq	FLAG		; compare f with W, skip if =0, if it was zero, just finish up
		goto	markit		; OTHERWISE! - Mark it up!

		clrf	FLAG		; shutoff continuation flag
		movff	CH1, CH1B	; put current values into old value storage
		movff	CH2, CH2B
		movff	CH3, CH3B

		call	emsdel
		goto	sloop		; go back to sampling

found	bsf		FLAG, 1		; one of the signals is rising!
		bsf		PORTB, 2	; turn on detection flag
		movff	CH1, CH1B	; put current values into old value storage
		movff	CH2, CH2B
		movff	CH3, CH3B

		call	emsdel
		goto	sloop		; go back to sampling

; -------------- The rising portion of the signal is over, so now we evaluate the target.
markit	clrf	FLAG		; OK, go ahead and turn off the FLAG and mark this one up.
		movf	CNTRH, 0	; load up the high-byte of the counter
		btfss	STATUS, Z	; if zero, keep going
		bra		noprox		; if not zero, jump to noprox "no proximity flag"
		movf	RNGTH, 0	; if a target is within the first N (= RNGTH) samples, turn on proximity flag
		cpfsgt	CNTR
		bsf		PORTC, 2	; if close, turn on proximity flag - turn on!  LED
noprox	bsf		PORTB, 0	; init, evaluation, markit flag
							; after detection, determine direction - if different direction
							; from previous sample, clear the FLAG and call it a different target
							; TMP is counter for the different channels during comparison


		movlw	0x1D		; if TCNT is >  29, i.e., 30, stop
		cpfsgt	TCNT
		goto	addtar
		goto	skip
addtar	incf	TCNT, 1		; increment the target counter
		lfsr	FSR1, TMP2	; put the address of TMP2 in FSR1 - ref for RNG
		movf	TCNT, 0		; load TCNT to W
		addwf	FSR1L, 1	; add W to FSR1
		movff	CNTR, INDF1	; range counter --> RNG variable
		lfsr	FSR1, RNG32	; reference address for high-byte of RNG variable
		movf	TCNT, 0		; load TCNT into W
		addwf	FSR1L, 1	; add W to FSR1
		movff	CNTRH, INDF1	; range counter high byte --> RNG variable	
		lfsr	FSR1, RNG32H	; reference address for MG1_x
		movf	TCNT, 0
		addwf	FSR1L, 1
		movff	CH1, INDF1	; CH1 () --> MG1_x
		lfsr	FSR1, MG1_32	; reference address for MG2_x
		movf	TCNT, 0
		addwf	FSR1L, 1
		movff	CH2, INDF1	; CH2 () --> MG2_x
		lfsr	FSR1, MG2_32	; reference address for MG3_x
		movf	TCNT, 0
		addwf	FSR1L, 1
		movff	CH3, INDF1	; CH3 () --> MG3_x

		
skip	movff	CH1, CH1B
		movff	CH2, CH2B
		movff	CH3, CH3B

		call	emsdel
		bcf		PORTB, 0	; turn off markit (init/evaluation) flag
		goto	sloop		; keep sampling; sloop

eval	bcf		PORTB, 2	; turn off the detection flag after the final sample.
		bsf		PORTB, 0	; put high at beginning of evaluation

		movlw	0x00
		cpfsgt	TCNT		; if target count > 0, skip
		goto	pao			; target count = 0, don't transmit
		call	datasnd

pao		bcf		PORTB, 0	; turn off to indicate that the evaluation is over.

		btfsc	PIR1, 5		   ;check for serial input
		call	oneping

		call	msdel
		call	ipi
		goto	loop		; this is where we decide what to do with the data

		
; ------ serial command: change pulse duration ----------
serpd	movff	RCREG, TMP	; transfer data to grab data into W
		movlw	0xAA		; 170
		cpfseq	TMP			; default power is: 0x0A  = max is 0x19(25)
		goto	serth		;  not a command, discard

pwrset	btfss	PIR1, 5		; set when data is received
		goto	pwrset
		movff	RCREG, CYCNT
		movlw	0x19
		cpfsgt	CYCNT
		return				; if it's equal or less than, finish up
		movwf	CYCNT		; load with max value if larger
		return 
; -------- serial command: change threshold
serth	movlw	0xBB		; 187
		cpfseq	TMP			; default threshold is: 0x60
		goto	serrth
thset	btfss	PIR1, 5		; set when data is received
		goto	thset
		movff	RCREG, THRESH
		return
; ---------- serial command: change indicator range threshold
serrth	movlw	0xCC		; 204
		cpfseq	TMP			; default value is: 0x20(32) max 0x90
		goto	serspat
kk		btfss	PIR1, 5
		goto	kk
		movff	RCREG, RNGTH
		return
; ----------- serial command: Sonar Bit pattern for transmission
serspat	movlw	0xDD		; 221
		cpfseq	TMP
		goto	serstat
spat	btfss	PIR1, 5
		goto	spat
		movff	RCREG, SPATT
		return
; --------------- serial command: status (report programmable parameters)
serstat movlw	0xEE
		cpfseq	TMP
		goto	serdat
				
		movlw	0x02		; transmit 0x02 to indicate status packet
stat0	btfss	TXSTA, TRMT
		goto	stat0
		movwf	TXREG

stat1	btfss	TXSTA, TRMT
		goto	stat1
		movff	CYCNT, TXREG		; send cycle count (pulse duration) 1st
stat2	btfss	TXSTA, TRMT
		goto	stat2
		movff	THRESH, TXREG		; send derivative threshold 2nd
stat3	btfss	TXSTA, TRMT
		goto	stat3
		movff	RNGTH, TXREG		; send range threshold
stat4	btfss	TXSTA, TRMT
		goto	stat4
		movff	SPATT, TXREG		; send sonar pattern code
		return
; --------------- serial command: data repeat -----
serdat	movlw	0xF0
		cpfseq	TMP
		return
		call	datasnd
		return

; --------- Transmitting Data -------
datasnd clrf	TMP2		; clear the checksum

		movlw	0xFF		; transmit 0xFF for framing
z0		btfss	TXSTA, TRMT
		goto	z0
		movwf	TXREG
		addwf	TMP2, 1		; add to checksum

		movlw	0x01		; transmit 0x01 to indicate data packet
z1		btfss	TXSTA, TRMT
		goto	z1
		movwf	TXREG
		addwf	TMP2, 1		; add to checksum

a0		btfss	TXSTA, TRMT		; TXIF = set if data received
		goto	a0			; empty if TXIF = 1
		movf	TCNT, 0		; load TCNT into W
		movwf	TXREG		; send the target count
		addwf	TMP2, 1		; add TCNT to the checksum

		clrf	CNT1
serlp	incf	CNT1, 1	

a1		btfss	TXSTA, TRMT
		goto	a1
		
		lfsr	FSR1, TMP2	; reference address for low-byte of the RNG counter
		movf	CNT1, 0			; target counter		
		addwf	FSR1L, 1
		movf	INDF1, 0		;
		movwf	TXREG
		addwf	TMP2, 1		; add RNG to checksum

a1b		btfss	TXSTA, TRMT
		goto	a1b
		lfsr	FSR1, RNG32	; reference for the high-byte of the RNG counter
		movf	CNT1, 0		; put it in a temporary location
		addwf	FSR1L, 1
		movf	INDF1, 0
		movwf	TXREG
		addwf	TMP2, 1	 	; ang to checksum

a2		btfss	TXSTA, TRMT
		goto	a2
		lfsr	FSR1, RNG32H	; reference address for MG1_x
		movf	CNT1, 0
		addwf	FSR1L, 1
		movf	INDF1, 0
		movwf	TXREG
		addwf	TMP2, 1		; add MG1_x to checksum

a3		btfss	TXSTA, TRMT
		goto	a3
		lfsr	FSR1, MG1_32	; reference address for MG2_x
		movf	CNT1, 0
		addwf	FSR1L, 1
		movf	INDF1, 0
		movwf	TXREG
		addwf	TMP2, 1		;add MG2_x to checksum

a4		btfss	TXSTA, TRMT
		goto	a4
		lfsr	FSR1, MG2_32	; reference address for MG3_x
		movf	CNT1, 0
		addwf	FSR1L, 1
		movf	INDF1, 0
		movwf	TXREG
		addwf	TMP2, 1		;add MG3_x to checksum

		movf	CNT1, 0		; load current target number
		cpfseq	TCNT
		goto	serlp

chktx	movlw	0xFF
		cpfseq	TMP2		; compare checksum with 0xFF
		bra		chksnd
		clrf	TMP2
chksnd	movf	TMP2, 0		; load up checksum
cs0		btfss	TXSTA, TRMT
		goto	cs0
		movwf	TXREG
		return

; ---- sample the A/D - subroutine ------

sampAD	bsf		PORTB, 1
		movlw	0x11		; 00 0000 01 = select AN0, and enable A/D  <-- left is AN0
;		movlw	0x05		; 00 0001 01 = channel 2 being used for CH1 (left)
		movwf	ADCON0		; select AN0 - auto acquisition delay
		bsf		ADCON0, 1	; GO!  - start conversion
ad1		btfsc	ADCON0, 1	; skip if conversion flag = 0
		goto	ad1			; waiting for conversion to finish
		movff	ADRESH, CH1
		movlw	0x0D		; 00 0001 01 = select AN1, and enable A/D  <-
;		movlw	0x0D		; 00 0011 01 = channel 4 being used for CH2 (middle)
		movwf	ADCON0		; select AN1 - auto acquisition delay
		bsf		ADCON0, 1	; GO!  - start conversion
ad2		btfsc	ADCON0, 1	; skip if conversion flag = 0
		goto	ad2			; waiting for conversion to finish
		movff	ADRESH, CH2	; store in CH2
		movlw	0x09		; 00 0010 01 = select AN2, and enable A/D  <-- 
;		movlw	0x11		; 00 0100 01  channel 5 being used for CH3 (right)
		movwf	ADCON0		; select AN2 - auto acquisition delay
		bsf		ADCON0, 1	; GO!  - start conversion
ad3		btfsc	ADCON0, 1	; skip if conversion flag = 0
		goto	ad3			; waiting for conversion to finish
		movff	ADRESH, CH3	; store in CH3
		bcf		PORTB, 1 	; sampling flag off

		return

; ----------------------------------------------------------
ipi		movf	IPIN, 0  	;	
		movwf	CNT3
		
; now we check the serial input to see if there are any commands
;ipi1	btfsc	PIR1, 5		; set if data is received
;		call	serpd		; skip if clear - no serial data
ipi1	call	msdel
		decfsz	CNT3, 1
		goto	ipi1

		return

; unitary time delay
msdel	movlw	04h
		movwf	CNT1

ms1		movlw	0xA4
		movwf	CNT2
 
ms2		decfsz	CNT2, 1
		goto	ms2

		decfsz	CNT1, 1
		goto	ms1
		return

; 1/4 ms
qmsdel	movlw	01h
		movwf	CNT1

hms1	movlw	0xA4
		movwf	CNT2
 
hms2	decfsz	CNT2, 1
		goto	hms2

		decfsz	CNT1, 1
		goto	hms1
		return
; 1/8 ms
emsdel	movlw	0x01
		movwf	CNT1
ems1	movlw	0x50
		movwf	CNT2
ems2	decfsz	CNT2, 1
		goto	ems2
		decfsz	CNT1, 1
		goto	ems1
		return

		end
