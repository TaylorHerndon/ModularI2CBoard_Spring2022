;===============================================================================
;
;                    Modular I2C Sensor Board Assembly Code
;			   Created By: Taylor Herndon
;				  Spring 2022
;
;===============================================================================
 ; Summary:
 ;     This program has two primary functions. The first of which is to
 ; search for and interface with sensors through I2C. The program, once in
 ; programming mode, will allow the user to search for new sensors attached to
 ; the SSP2 I2C bus line. Once a new sensor is found it will reassign the address
 ; of the sensor and log its address in EEPROM. The program is capable of
 ; interfacing with up to 8 different sensors at a time. There are two different
 ; sensors available to interface with. These are either the TF-LUNA Lidar 
 ; or the Infrared sensor I2C board developed by Kendall Callister. 
 ;     The second function of this board is to be an I2C slave device to an
 ; external master board. The slave address of this board is physically selected
 ; by the DIP switch array on PORTB. The slave interface for this board is 
 ; consistent with typical I2C slave device interfaces. A write command to the
 ; slave device will select the I2C register that will be read out first when
 ; a read command is sent to the device. When a read command is detected, this
 ; board will load the selected register to the buffer for it to be clocked out
 ; by the master device. The 0 register will indicate the number of data bytes
 ; that are loaded into I2C registers. Registers 1-24 are distance data registers.
 ; Each of the distance data registers contain the data of a unique distance
 ; sensor. The Registers 25-28 are the board identification registers. These
 ; values are static and are used to help identify the board. The values loaded
 ; into these registers are ASCII 'M' 'O' 'D' 'X'.
;===============================================================================
    
    LIST p=18F26k40_g
    #include "p18f26k40.inc"

; CONFIG1L
  CONFIG  FEXTOSC = OFF         ; External Oscillator mode Selection bits (Oscillator not enabled)
  CONFIG  RSTOSC = HFINTOSC_64MHZ; Power-up default value for COSC bits (HFINTOSC with HFFRQ = 64 MHz and CDIV = 1:1)

; CONFIG1H
  CONFIG  CLKOUTEN = OFF        ; Clock Out Enable bit (CLKOUT function is disabled)
  CONFIG  CSWEN = ON            ; Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
  CONFIG  FCMEN = ON            ; Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor enabled)

; CONFIG2L
  CONFIG  MCLRE = EXTMCLR       ; Master Clear Enable bit (If LVP = 0, MCLR pin is MCLR; If LVP = 1, RE3 pin function is MCLR )
  CONFIG  PWRTE = OFF           ; Power-up Timer Enable bit (Power up timer disabled)
  CONFIG  LPBOREN = OFF         ; Low-power BOR enable bit (ULPBOR DISABLED)
  CONFIG  BOREN = ON            ; Brown-out Reset Enable bits (Brown-out Reset enabled , SBOREN bit is ignored)

; CONFIG2H
  CONFIG  BORV = VBOR_285       ; Brown Out Reset Voltage selection bits (Brown-out Reset Voltage (VBOR) set to 2.85V)
  CONFIG  ZCD = OFF             ; ZCD Disable bit (ZCD disabled. ZCD can be enabled by setting the ZCDSEN bit of ZCDCON)
  CONFIG  PPS1WAY = ON          ; PPSLOCK bit One-Way Set Enable bit (PPSLOCK bit can be cleared and set only once; PPS registers remain locked after one clear/set cycle)
  CONFIG  STVREN = ON           ; Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
  CONFIG  DEBUG = OFF           ; Debugger Enable bit (Background debugger disabled)
  CONFIG  XINST = OFF           ; Extended Instruction Set Enable bit (Extended Instruction Set and Indexed Addressing Mode disabled)

; CONFIG3L
  CONFIG  WDTCPS = WDTCPS_5     ; WDT Period Select bits (Divider ratio 1:1024)
  CONFIG  WDTE = ON             ; WDT operating mode (WDT enabled regardless of sleep)

; CONFIG3H
  CONFIG  WDTCWS = WDTCWS_7     ; WDT Window Select bits (window always open (100%); software control; keyed access not required)
  CONFIG  WDTCCS = LFINTOSC     ; WDT input clock selector (WDT reference clock is the 31.0 kHz LFINTOSC)

; CONFIG4L
  CONFIG  WRT0 = OFF            ; Write Protection Block 0 (Block 0 (000800-003FFFh) not write-protected)
  CONFIG  WRT1 = OFF            ; Write Protection Block 1 (Block 1 (004000-007FFFh) not write-protected)
  CONFIG  WRT2 = OFF            ; Write Protection Block 2 (Block 2 (008000-00BFFFh) not write-protected)
  CONFIG  WRT3 = OFF            ; Write Protection Block 3 (Block 3 (00C000-00FFFFh) not write-protected)

; CONFIG4H
  CONFIG  WRTC = OFF            ; Configuration Register Write Protection bit (Configuration registers (300000-30000Bh) not write-protected)
  CONFIG  WRTB = OFF            ; Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
  CONFIG  WRTD = OFF            ; Data EEPROM Write Protection bit (Data EEPROM not write-protected)
  CONFIG  SCANE = ON            ; Scanner Enable bit (Scanner module is available for use, SCANMD bit can control the module)
  CONFIG  LVP = ON              ; Low Voltage Programming Enable bit (Low voltage programming enabled. MCLR/VPP pin function is MCLR. MCLRE configuration bit is ignored)

; CONFIG5L
  CONFIG  CP = OFF              ; UserNVM Program Memory Code Protection bit (UserNVM code protection disabled)
  CONFIG  CPD = OFF             ; DataNVM Memory Code Protection bit (DataNVM code protection disabled)

; CONFIG5H

; CONFIG6L
  CONFIG  EBTR0 = OFF           ; Table Read Protection Block 0 (Block 0 (000800-003FFFh) not protected from table reads executed in other blocks)
  CONFIG  EBTR1 = OFF           ; Table Read Protection Block 1 (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
  CONFIG  EBTR2 = OFF           ; Table Read Protection Block 2 (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)
  CONFIG  EBTR3 = OFF           ; Table Read Protection Block 3 (Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks)

; CONFIG6H
  CONFIG  EBTRB = OFF           ; Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)
    
;===CPU MEMORY MAP==============================================================
    
WSAVE	    EQU	    0x00		;ISR VARIABLES
STATUSSAVE  EQU	    0x01		;//
PFLAG	    EQU	    0x02		;/

;pFlag [7] [6] [5] [4] [3] [2] [1] [0]
; [7]: TMR4 Handle Flag (Debugging)
; [6]: TMR2 Handle Flag
; [5]: IOC Handle Flag
; [4]: Erase EEPROM Flag
; [3]: Search for New Sensor Flag
; [2]: Request Data From Sensors Flag
; [1]: 
; [0]: 
	    
VARRAY	    EQU	    0x03		;MACRO VARIABLES
VINDEX	    EQU	    0x04		;/

EEDAT	    EQU	    0X05		;EEPROM VARIABLES
EEADR	    EQU	    0X06		;/
	    
I2C2ADR	    EQU	    0X07		;I2C2 VARIABLES
I2C2DAT	    EQU	    0X08		;//
I2C2RXDAT   EQU	    0X09		;/
   
I2C1STATUS  EQU	    0X0A		;I2C1VARIABLES
I2C1RXDAT   EQU	    0X0B		;//
I2C1REGIND  EQU	    0X0C		;/
 
SCANADR	    EQU	    0X0D		;SCAN AND SAVE SENSOR VARIABLES
SENSOREEADR EQU	    0X0E		;/
 
LASTSENSOR  EQU	    0X0F		;REQUEST DATA VARIABLES
BYTECOUNT   EQU	    0X10		;//////
SENSORID    EQU	    0X11		;/////
DISTBYTES   EQU	    0X12		;////
COUNT	    EQU	    0X13		;///
DISTDATHOLD EQU	    0X14		;//
DISTDATIND  EQU	    0X15		;/
 
T2DELAY	    EQU	    0X16		;TIMER VARIABLES
T2COUNT	    EQU	    0X17		;//
T4DELAY	    EQU	    0X18		;/
	    
TEMPADR	    EQU	    0X19		;HOLDING VARIABLE FOR I2C ADDRESS
	    
I2C1REG	    EQU	    0X20		;I2C1 SLAVE REGISTERS [0X20-0X4F]
	    
;I2C1Registers[] [0] [1-24] [25-28]
;[0]: Number of distance data bytes ready to transmit
;[01-24] : Distance data registers (Register 12 = Sensor 12 Distance Data)
;[25-28] : Board Identification Registers [M]-[O]-[D]-[X]
;[28+] : Not used
	    
ID	    EQU	    0X50		;ID ARRAY [0X50-0X5F]
DISTDATA    EQU	    0X60		;DISTANCE DATA ARRAY [0X60-0X6F]
    
;EEPROM Data Storage [0-0x0F] : [0x10 - 0x1F] : [0x20 - 0xFF+]
;[0x00 - 0x0F] : Not Used
;[0x20 - 0xFF+]: Not Used
;[0x10 - 0x1F] : Sensor Storage Registers
;   [x0h] : Identifier (Type of Sensor) : 'L' = Luna, 'I' = Infrared
;   [x1h] : Number of distance bytes to request
  
;===END CPU MEMORY MAP==========================================================
;===ORG BEGIN===================================================================
	    
	    ORG	    0x00		;DEFAULT SETUP LOCATION VECTOR IS 0X00
	    GOTO    SETUP		;/
	    ORG	    0x08		;DEFAULT INTERRUPT VECTOR IS 0X08
	    GOTO    INTERRUPT		;/
	    
;===ORG END=====================================================================
;===MACROS BEGIN================================================================
;------ARRAY MACRO--------------------------------------------------------------
ARRAY	    MACRO   VARRAY,VINDEX
	    CLRF    FSR0H		; CLEAR HIGH BYTE
	    LFSR    0,VARRAY		; FSR0 = &vArray[0]
	    MOVF    VINDEX,W		; INDEX, 0..255
	    ADDWF   FSR0L,F		; FSR0 = &vArray[vIndex]
	    ENDM			; END MACRO
;------END ARRAY MACRO----------------------------------------------------------
;===MACROS END==================================================================
;===SETUP MICROCONTROLLER FUNCTIONS=============================================
SETUP
;------VARIABLE SETUP-----------------------------------------------------------
	    BANKSEL WSAVE		;CLEAR ALL VARIABLES ON STARTUP
	    CLRF    WSAVE		;/////////////////////////
	    CLRF    STATUSSAVE		;////////////////////////
	    CLRF    PFLAG		;///////////////////////
	    CLRF    VARRAY		;//////////////////////
	    CLRF    VINDEX		;/////////////////////
	    CLRF    EEDAT		;////////////////////
	    CLRF    EEADR		;///////////////////
	    CLRF    I2C2ADR		;//////////////////
	    CLRF    I2C2DAT		;/////////////////
	    CLRF    I2C2RXDAT		;////////////////
	    CLRF    I2C1STATUS		;///////////////
	    CLRF    I2C1RXDAT		;//////////////
	    CLRF    I2C1REGIND		;/////////////
	    CLRF    SCANADR		;////////////
	    CLRF    SENSOREEADR		;///////////
	    CLRF    LASTSENSOR		;//////////
	    CLRF    BYTECOUNT		;/////////
	    CLRF    SENSORID		;////////
	    CLRF    DISTBYTES		;///////
	    CLRF    COUNT		;//////
	    CLRF    DISTDATHOLD		;/////
	    CLRF    DISTDATIND		;////
	    CLRF    T2DELAY		;///
	    CLRF    T2COUNT		;//
	    CLRF    T4DELAY		;/
	    
	    ARRAY   0X20,D'0'		;CLEAR SLAVE REGISTERS 0 TO 24
CLEARREGLOOP				;///////
	    MOVLW   0X00		;//////
	    MOVWF   INDF0		;/////
	    INCF    FSR0L,1		;////
	    MOVLW   D'25'		;///
	    CPFSEQ  FSR0L		;//
	    GOTO    CLEARREGLOOP	;/
	    
	    ARRAY   0X20,D'25'		;LOAD 'MODX' INTO THE SLAVE REGISTERS 25-28
	    MOVLW   'M'			;///////////
	    MOVWF   INDF0		;//////////
	    ARRAY   0X20,D'26'		;/////////
	    MOVLW   'O'			;////////
	    MOVWF   INDF0		;///////
	    ARRAY   0X20,D'27'		;//////
	    MOVLW   'D'			;/////
	    MOVWF   INDF0		;////
	    ARRAY   0X20,D'28'		;///
	    MOVLW   'X'			;//
	    MOVWF   INDF0		;/
	    
	    MOVLW   D'150'		;SET T2DELAY TO INDICATE PROGRAMMING MODE IMMEDIATLY
	    MOVWF   T2DELAY		;/
;------END VARIABLE SETUP-------------------------------------------------------
;------PORT SETUP---------------------------------------------------------------
	    BANKSEL PPSLOCK		;PPSLOCK CHARGE PUMP TO UNLOCK PPS REASSIGNMENT
	    MOVLW   0x55		;////
	    MOVWF   PPSLOCK		;///
	    MOVLW   0xAA		;//
	    MOVWF   PPSLOCK		;/
	    BCF	    PPSLOCK,PPSLOCKED	;UNLOCK PPS
	    
	    BANKSEL RC0PPS		;SET RC0 INPUT DESITINATION AS MSSP1 DATA
	    MOVLW   0X10		;//
	    MOVWF   RC0PPS		;/
	    BANKSEL RC1PPS		;
	    MOVLW   0X0F		;SET RC1 INPUT DESTINATION AS MSSP1 CLOCK
	    MOVWF   RC1PPS		;/
	    
	    BANKSEL SSP1DATPPS		;SET MSSP1 DATA OUTPUT AS RC0
	    MOVLW   B'00010000'		;//
	    MOVWF   SSP1DATPPS		;/
	    BANKSEL SSP1CLKPPS
	    MOVLW   B'00010001'		;SET MSSP1 CLK OUTPUT AS RC1
	    MOVWF   SSP1CLKPPS		;/
	    
	    BANKSEL RC2PPS		;SET RC2 INPUT DESTINATION AS MSSP2 DATA
	    MOVLW   0X12		;//
	    MOVWF   RC2PPS		;/
	    BANKSEL RC3PPS
	    MOVLW   0X11		;SET RC3 INPUT DESTINATION AS MSSP2 CLOCK
	    MOVWF   RC3PPS		;/
	    
	    BANKSEL SSP2DATPPS		;SET MSSP2 DATA OUTPUT AS RC2
	    MOVLW   B'00010010'		;//
	    MOVWF   SSP2DATPPS		;/
	    BANKSEL SSP2CLKPPS
	    MOVLW   B'00010011'		;SET MSSP2 CLOCK OUTPUT AS RC3
	    MOVWF   SSP2CLKPPS		;/
	    
	    BANKSEL PPSLOCK		;PPSLOCK CHARGE PUMP TO LOCK PPS REASSIGNMENTS
	    MOVLW   0X55		;////
	    MOVWF   PPSLOCK		;///
	    MOVLW   0XAA		;//
	    MOVWF   PPSLOCK		;/
	    BSF	    PPSLOCK,PPSLOCKED	;LOCK PPS
	    
	    BANKSEL LATA		;CLEAR ALL UNDESIRED OUTPUTS
	    CLRF    LATA		;////
	    CLRF    LATB		;///
	    CLRF    LATC		;//
	    
	    BANKSEL ANSELA		;ALL PINS ARE DIGITAL IOs
	    CLRF    ANSELA		;////
	    CLRF    ANSELB		;///
	    CLRF    ANSELC		;//
	    
	    BANKSEL TRISA		;INPUT/OUTPUT PIN SELECTION
	    CLRF    TRISA		;PORTA IS DEBUGGING LEDS
	    SETF    TRISB		;PORTB IS SLAVE ADDRESS DIP SWITCHES
	    SETF    TRISC		;PORTC IS PROGRAMMING FUNCTIONS AND I2C
	    
;------END PORT SETUP-----------------------------------------------------------
;------STARTUP PROCESS----------------------------------------------------------
;HERE THE PROGRAM HEADER IS EVAULUATED AND WHAT MODULES TO START IS DETERMINED
;PORT SETUP IS ALLWAYS NEEDED SO IT IS BEFORE THIS PROCESS
	    
	    BANKSEL PORTC		;IF PROGRAM JUMPER IS CONNECTED, ENTER PROGRAM MODE
	    BTFSC   PORTC,6		;//
	    GOTO    PROGRAMMODE		;/
	    GOTO    STANDARDMODE	;OTHERWISE ENTER STANDARD MODE
	    
;------END STARTUP PROCESS------------------------------------------------------
;------PROGRAM MODE-------------------------------------------------------------
PROGRAMMODE ;IN PROGRAM MODE: I2C1: DISABLED, I2C2: ENABLED, TMR2: ENABLED, IOC: ENABLED
	    
	    CALL    SSP2SETUP		;ENABLE SSP2 TO COMMUNICATE WITH SENSORS
	    CALL    TIMER2SETUP		;ENABLE TMR2 FOR PROGRAM MODE INDICATOR LIGHTS
	    CALL    IOCSETUP		;ENABLE 
	    CALL    INTERRUPTSETUP	;
	    
	    BANKSEL LATA
	    SETF    LATA
	    
	    GOTO    MAINBEGIN
;------END PROGRAM MODE---------------------------------------------------------
;------STANDARD MODE------------------------------------------------------------
STANDARDMODE ;IN STANDARD MODE: I2C1: ENABLED, I2C2: ENABLED, TMR2: DISABLED, IOC: DISABLED
	    
	    ;CALL    SSP1SETUP		;SSP1 WILL BE ENABLED AND CONFIGURED WITH TMR4
	    CALL    SSP2SETUP
	    CALL    TIMER4SETUP
	    CALL    INTERRUPTSETUP
	    
	    GOTO    MAINBEGIN
;------END STANDARD MODE--------------------------------------------------------
;------TIMER SETUP--------------------------------------------------------------
TIMER2SETUP
	    ;TIMER2: PRE = 1:128, POST = 1:16 PR = 250, t/MATCH = 32ms
	    
	    BANKSEL T2PR		;TIMER2 PERIOD REGISTER, SET TO 250
	    MOVLW   D'250'		;//
	    MOVWF   T2PR		;/
	    
	    BANKSEL T2CON		;TIMER2 CONTROL REGISTER
	    BSF	    T2CON,T2ON		;TIMER2 ENABLE BIT
	    BSF	    T2CON,T2CKPS2	;TIMER2 PRESCALER = 1:128
	    BSF	    T2CON,T2CKPS1	;//
	    BSF	    T2CON,T2CKPS0	;/
	    BSF	    T2CON,T2OUTPS3	;TIMER2 POSTSCALER = 1:16
	    BSF	    T2CON,T2OUTPS2	;///
	    BSF	    T2CON,T2OUTPS1	;//
	    BSF	    T2CON,T2OUTPS0	;/
	    
	    BANKSEL T2HLT		;TIMER2 HARDWARE LIMIT CONTROL REGISTER
	    BCF	    T2HLT,PSYNC		;PRESCALER SYNCRONIZATION ENABLE BIT
	    BCF	    T2HLT,CKPOL		;TIMER CLOCK POLARITY SELECT BIT
	    BCF	    T2HLT,CKSYNC	;TIMER CLOCK SYNCRONIZATION ENABLE BIT
	    BCF	    T2HLT,MODE0		;TIMER2 MODE SELECT BITS, 00000 = NORMAL OPERATION
	    BCF	    T2HLT,MODE1		;////
	    BCF	    T2HLT,MODE2		;///
	    BCF	    T2HLT,MODE3		;//
	    BCF	    T2HLT,MODE4		;/
	    
	    BANKSEL T2CLKCON		;TIMER2 CLOCK SOURCE SELECTION REGISTER
	    BCF	    T2CLKCON,CS3	;TIMER2 CLOCK SOURCE SELECT BITS, 0001 = FOSC/4
	    BCF	    T2CLKCON,CS2	;///
	    BCF	    T2CLKCON,CS1	;//
	    BSF	    T2CLKCON,CS0	;/
	    
	    BANKSEL PIE4		;TIMER INTERRUPT ENABLE BITS
	    BSF	    PIE4,TMR2IE		;TIMER2 INTERRUPTS = ENABLED
	    
	    RETURN
	    
TIMER4SETUP
	    
	    ;TIMER4 IS USED ONLY FOR STARTUP DELAY
	    ;TIMER4: PRE = 1:128, POST = 1:16, PR = 250, t/MATCH = 32mS
	    
	    BANKSEL T4PR		;TIMER4 PERIOD REGISTER, SET TO 125
	    MOVLW   D'250'		;//
	    MOVWF   T4PR		;/
	    
	    BANKSEL T4CON		;TIMER4 CONTROL REGISTER
	    BSF	    T4CON,T4ON		;TIMER4 ENABLE BIT
	    BSF	    T4CON,T4CKPS2	;TIMER4 PRESCALER = 1:128
	    BSF	    T4CON,T4CKPS1	;//
	    BSF	    T4CON,T4CKPS0	;/
	    BSF	    T4CON,T4OUTPS3	;TIMER4 POSTSCALER = 1:16
	    BSF	    T4CON,T4OUTPS2	;///
	    BSF	    T4CON,T4OUTPS1	;//
	    BSF	    T4CON,T4OUTPS0	;/
	    
	    BANKSEL T4HLT		;TIMER4 HARDWARE LIMIT CONTROL REGISTER
	    BCF	    T4HLT,PSYNC		;PRESCALER SYNCRONIZATION ENABLE BIT
	    BCF	    T4HLT,CKPOL		;TIMER CLOCK POLARITY SELECT BIT
	    BCF	    T4HLT,CKSYNC	;TIMER CLOCK SYNCRONIZATION ENABLE BIT
	    BCF	    T4HLT,MODE0		;TIMER2 MODE SELECT BITS, 00000 = NORMAL OPERATION
	    BCF	    T4HLT,MODE1		;////
	    BCF	    T4HLT,MODE2		;///
	    BCF	    T4HLT,MODE3		;//
	    BCF	    T4HLT,MODE4		;/
	   
	    BANKSEL T4CLKCON		;TIMER4 CLOCK SOURCE SELECTION REGISTER
	    BCF	    T4CLKCON,CS3	;TIMER4 CLOCK SOURCE SELECT BITS, 0001 = FOSC/4
	    BCF	    T4CLKCON,CS2	;///
	    BCF	    T4CLKCON,CS1	;//
	    BSF	    T4CLKCON,CS0	;/
	    
	    BANKSEL PIE4		;TIMER INTERRUPT ENABLE BITS
	    BSF	    PIE4,TMR4IE		;TIMER4 INTERRUPTS = ENABLED
	    
	    RETURN
;------END TIMER SETUP----------------------------------------------------------
;------SSP1SETUP----------------------------------------------------------------
SSP1SETUP
	    
	    BANKSEL PORTB		;READ PORTB TO GET THE DIP SWITCH ARRAY ADDRESS
	    COMF    PORTB,0		;COMPLEMENT PORTB BECAUSE PORTB READS OPPSOSITE OF LEDs
	    BANKSEL TEMPADR		;LOAD PORTB TO TEMPADR FOR FURTHER MANIPULATION
	    MOVWF   TEMPADR		;/
	    BCF	    STATUS,C		;ROTATE SLAVE ADDRESS LEFT ONCE
	    RLCF    TEMPADR,1		;/
	    MOVLW   B'01111110'		;IGNORE LSB AND MSB OF ADDRESS
	    ANDWF   TEMPADR,1		;/
	    
	    MOVLW   0X10		;SET THE LOWEST SLAVE ADDRESS VALUE TO 0X10
	    CPFSGT  TEMPADR		;//
	    MOVWF   TEMPADR		;/
	    
	    MOVF    TEMPADR,0		;LOAD THE SLAVE ADDRESS TO SSP1
	    BANKSEL SSP1ADD		;//
	    MOVWF   SSP1ADD		;/
	    
	    BANKSEL SSP1CON1
	    BSF	    SSP1CON1,CKP	;RELEASE CLOCK ON STARTUP
	    BCF	    SSP1CON1,SSPM3	;I2C 7 BIT ADDRESS SLAVE MODE
	    BSF	    SSP1CON1,SSPM2	;///
	    BSF	    SSP1CON1,SSPM1	;//
	    BCF	    SSP1CON1,SSPM0	;/
	    
	    BANKSEL SSP1CON2		
	    BCF	    SSP1CON2,GCEN	;DISABLE GENERAL CALLS
	    
	    BANKSEL SSP1STAT		
	    BCF	    SSP1STAT,SMP	;SET SLEW RATE CONTROL FOR 400KHz
	    BCF	    SSP1STAT,CKE	;DISABLE SMBus SPECIFICATIONS
	    
	    BANKSEL SSP1MSK		;USE ALL BITS TO ADDRESS SLAVE
	    SETF    SSP1MSK		;/
	    
	    BANKSEL PIE3		;SERIAL DATA INTERRUPT ENABLE BITS
	    BCF	    PIE3,BCL1IE		;SSP1 BUS COLLISION INTERRUPT ENABLE
	    BSF	    PIE3,SSP1IE		;SSP1 INTERRUPT ENABLE
	    
	    BANKSEL SSP1CON1		;ENABLE SERIAL PORT
	    BSF	    SSP1CON1,SSPEN	;/
	    
	    RETURN
;------END SSP1SETUP------------------------------------------------------------
;------SSP2SETUP----------------------------------------------------------------
SSP2SETUP
	    BANKSEL SSP2ADD		;SET BAUD TO 400KHZ
	    MOVLW   D'159'		;//
	    MOVWF   SSP2ADD		;/
	    
	    BANKSEL SSP2CON1		
	    BSF	    SSP2CON1,SSPM3	;I2C MASTER CONFIGURATION
	    BCF	    SSP2CON1,SSPM2	;///
	    BCF	    SSP2CON1,SSPM1	;//
	    BCF	    SSP2CON1,SSPM0	;/
	    
	    BANKSEL SSP2STAT		
	    BCF	    SSP2STAT,SMP	;SET SLEW RATE CONTROL TO 400KHZ
	    BCF	    SSP2STAT,CKE	;DISABLE SMBus SPECIFICATIONS
	    
	    BANKSEL SSP2CON1		;ENABLE THE SERIAL PORT
	    BSF	    SSP2CON1,SSPEN	;/
	    
	    BANKSEL PIE3		;SERIAL DATA INTERRUPT ENABLE BITS
	    BCF	    PIE3,BCL2IE		;SSP2 BUS COLLISION INTERRUPT ENABLE
	    BCF	    PIE3,SSP2IE		;SSP2 INTERRUPT ENABLE
	    
	    RETURN
;------END SSP2SETUP------------------------------------------------------------
;------IOC SETUP----------------------------------------------------------------    
IOCSETUP
	    BANKSEL IOCCP		;ENABLE RISING EDGE INTERRUPTS ON RC4 & RC5
	    MOVLW   B'00110000'		;//
	    MOVWF   IOCCP		;/
	    
	    BANKSEL IOCCN		;DISABLED PORTC FALLING EDGE INTERRUPTS
	    MOVLW   B'00000000'		;//
	    MOVWF   IOCCN		;/
	    
	    BANKSEL PIE0
	    BCF	    PIE0,TMR0IE		;DISABLE TIMER 0 INTERRUPTS
	    BSF	    PIE0,IOCIE		;ENABLE ON CHANGE INTERRUPTS
	    BCF	    PIE0,INT0IE		;DIABLE EXTERNAL INTERRUPTS
	    BCF	    PIE0,INT1IE		;//
	    BCF	    PIE0,INT2IE		;/
	    RETURN	    
;------END IOC SETUP------------------------------------------------------------
;------INTERRUPT SETUP----------------------------------------------------------
INTERRUPTSETUP
	    BANKSEL PIR0		;CLEAR ALL POSSIBLE UNWANTED FLAGS
	    CLRF    PIR0		;/////////
	    BANKSEL PIR1		;////////
	    CLRF    PIR1		;///////
	    BANKSEL PIR2		;//////
	    CLRF    PIR2		;/////
	    BANKSEL PIR3		;////
	    CLRF    PIR3		;///
	    BANKSEL PIR4		;//
	    CLRF    PIR4		;/
	    
	    BSF	    INTCON,GIE		;ENABLE GLOBAL INTERRUPTS
	    BSF	    INTCON,PEIE		;ENABLE PERIPHERALL INTERRUPTS
	    RETURN
;------END INTERRUPT SETUP------------------------------------------------------
	    GOTO    MAINBEGIN		;SETUP END, GOTO MAIN PROGRAM LOOP
;===END SETUP MICROCONTROLLER FUNCTIONS=========================================
;===INTERRUPT SERVICE ROUTINE===================================================
INTERRUPT
	    BANKSEL WSAVE		;SAVE WREG AND STATUS REG
	    MOVWF   WSAVE		;///
	    MOVF    STATUS,0		;//
	    MOVWF   STATUSSAVE		;/
	    
	    BANKSEL PIR4		;CHECK IF TMR2 NEEDS SERVICED
	    BTFSC   PIR4,TMR2IF		;//
	    CALL    TMR2PROCESS		;/
	    
	    BANKSEL PIR0		;CHECK IF IOC NEEDS SERVICED
	    BTFSC   PIR0,IOCIF		;//
	    CALL    IOCPROCESS		;/
	    
	    BANKSEL PIR3		;CHECK IF I2C1 NEEDS SERVICED
	    BTFSC   PIR3,SSP1IF		;//
	    CALL    I2C1ISR		;/
	    
	    BANKSEL PIR4		;CHECK IF TIMER4 NEEDS SERVICE
	    BTFSC   PIR4,TMR4IF		;//
	    CALL    TMR4PROCESS		;/
	    
	    BANKSEL STATUSSAVE		;RESTORE WREG AND STATUS REG
	    MOVF    STATUSSAVE,0	;///
	    MOVWF   STATUS		;//
	    MOVF    WSAVE,0		;/
	    
	    RETFIE
;===END INTERRUPT SERVICE ROUTINE===============================================
;===INTERRUPT FUNCTIONS=========================================================
;------TMR2PROCESS--------------------------------------------------------------
TMR2PROCESS
	    
	    BCF	    PIR4,TMR2IF		;CLEAR FLAG TO NOT REPEAT ISR
	    BANKSEL PFLAG		;SET PFLAG TO DEAL WITH IN MAIN
	    BSF	    PFLAG,6		;/
	    
	    RETURN 
;------END TMR2PROCESS----------------------------------------------------------
;------TMR4PROCESS--------------------------------------------------------------
TMR4PROCESS
	    BCF	    PIR4,TMR4IF

	    BANKSEL T4DELAY		;ONLY RUN T4TICK AFTER 3 COUNTS
	    MOVLW   D'3'		;THIS INCREASES DELAY AFTER STARTUP
	    CPFSLT  T4DELAY,0		;///
	    GOTO    T4TICK		;//
	    INCF    T4DELAY,1		;/
	    
	    RETURN
;------END TMR4PROCESS----------------------------------------------------------
;------T4TICK-------------------------------------------------------------------
T4TICK
	    CALL    SSP1SETUP		;CONFIGURE SSP1
	    
	    BANKSEL T4CON		;DISABLE TMR4 NOW THAT SSP2 HAS BEEN CONFIGURED
	    BCF	    T4CON,T4ON		;/
	    
	    RETURN
;------END T4TICK---------------------------------------------------------------
;------IOCPROCESS---------------------------------------------------------------
IOCPROCESS
	    
	    BANKSEL IOCCF		;CLEAR INDIVIDUAL PORTC IOC FLAGS
	    CLRF    IOCCF		;/
	    BANKSEL PIR0		;CLEAR IOC GENREAL FLAG
	    BCF	    PIR0,IOCIF		;/
	    
	    BANKSEL PFLAG		;SET PFLAG TO DEAL WITH IN MAIN
	    BSF	    PFLAG,5		;/
	    
	    RETURN
;------END IOCPROCESS-----------------------------------------------------------
;------I2C1ISR------------------------------------------------------------------
I2C1ISR
	    BCF	    PIR3,SSP1IF		;CLEAR I2C FLAG
	    BANKSEL SSP1CON1		;CLEAR ANY OVERFLOW EERORS
	    BCF	    SSP1CON1,SSPOV	;/
	    
	    BANKSEL I2C1STATUS		;IF STATUS IS NOT > 0 THEN STATUS IS 0
	    MOVLW   0X00		;GOTO CASE 0 TO EVALUATE NEW CONVERSATION
	    CPFSGT  I2C1STATUS,0	;//
	    GOTO    CASE0		;/
	    
	    MOVLW   0X01		;IF STATUS IS NOT > 1 THEN STATUS IS 1
	    CPFSGT  I2C1STATUS,0	;GOTO CASE 1 TO EVALUATE WRITE CONVERSATION
	    GOTO    CASE1		;/
	    
	    MOVLW   0X02		;IF STATUS IS NOT > 2 THEN STATUS IS 2
	    CPFSGT  I2C1STATUS,0	;GOTO CASE 2 TO EVAULATE READ CONVERSATION
	    GOTO    CASE2		;/ 
	    
	    GOTO    CASEDEFAULT		;NO OTHER CASE OCCOURED, GO TO DEFAULT
	    
;~~~~~~~~~I2C1CASES~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
CASE0	    
	    BANKSEL SSP1BUF		;READ BUFFER TO CLEAR FLAG
	    MOVF    SSP1BUF,0		;/
	    
	    BANKSEL I2C1STATUS		;LOAD WRITE CONVERSATION BY DEFAULT
	    MOVLW   0X01		;//
	    MOVWF   I2C1STATUS		;/
	    
	    BANKSEL SSP1STAT		;CHECK FOR READ CONVERSATION
	    BTFSC   SSP1STAT,R_W	;//
	    CALL    SLAVESENDDAT	;/
	    
	    BSF	    SSP1CON1,CKP	;RELEASE CLOCK
	    
	    GOTO    CASEEND
	    
CASE1	    
	    BANKSEL SSP1BUF		;READ BUFFER TO GET REGISTER INDEX
	    MOVF    SSP1BUF,0		;///
	    BANKSEL I2C1REGIND		;//
	    MOVWF   I2C1REGIND		;/
	    
	    BANKSEL SSP1CON1		;RELEASE CLOCK
	    BSF	    SSP1CON1,CKP	;/
	    
	    BANKSEL I2C1STATUS		;CLEAR STATUS TO WAIT FOR NEXT CONVERSATION
	    CLRF    I2C1STATUS		;/
	    
	    GOTO    CASEEND
	    
CASE2
	    BANKSEL SSP1CON1		;TEST IF NACK CONDITION WAS GENERATED
	    BTFSC   SSP1CON1,CKP	;/
	    GOTO    I2C1ENDREAD		;IF NACK WAS DETECTED, CLEAR CONVERSATION
	    
	    CALL    SLAVESENDDAT	;IF NACK WAS NOT DETECTED, SEND NEXT BYTE
	    
	    BANKSEL SSP1CON1		;RELEASE CLOCK
	    BSF	    SSP1CON1,CKP	;/
	    
    	    GOTO    CASEEND
	    
CASEDEFAULT				;DO NOTHING ON DEFAULT CASE
	    GOTO    CASEEND    
	    
CASEEND
;~~~~~~~~~END I2C1CASES~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	    RETURN
;------END I2C1ISR--------------------------------------------------------------
;---------I2C1ENDREAD-----------------------------------------------------------
I2C1ENDREAD
	    BANKSEL I2C1STATUS		;CLEAR I2C1STATUS TO WAIT FOR NEXT CONVERSATION
	    CLRF    I2C1STATUS		;/
	    BSF	    PFLAG,2		;SET PFLAG[2] TO REQUEST NEW DISTANCE DATA
	    GOTO    CASEEND		;GOTO END OF CASE STRUCTURE
;---------END I2C1ENDREAD-------------------------------------------------------
;---------SLAVESENDDAT----------------------------------------------------------
SLAVESENDDAT
	    
	    BANKSEL I2C1STATUS		;SET STORED CONVERSATION AS READ
	    MOVLW   0X02		;//
	    MOVWF   I2C1STATUS		;/
	    
	    CLRF    FSR0H		;GET THE VALUE OF THE SLAVE REGISTER AT I2C1REGIND
	    MOVLW   0X20		;//////
	    MOVWF   FSR0L		;/////
	    BANKSEL I2C1REGIND		;////
	    MOVF    I2C1REGIND,0	;///
	    ADDWF   FSR0L,1		;//
	    MOVF    INDF0,0		;/
	    BANKSEL SSP1BUF		;LOAD SELECTED REGISTER TO BUFFER TO BE CLOCKED OUT BY MASTER
	    MOVWF   SSP1BUF		;/
	    
	    INCF    I2C1REGIND,1	;INCREMENT TO NEXT REGISTER
	    
	    RETURN
;---------END SLAVESENDDAT------------------------------------------------------
;===END INTERRUPT FUNCTIONS=====================================================
;===FUNCTIONS===================================================================
;------WRITE TO EEPROM----------------------------------------------------------
;REQUIRED: LOAD EEDAT AND EEADR WITH CORESPONDING VALUES
WRITETOEEPROM
	    
	    BCF	    INTCON,GIE		;DISABLE INTERRUPTS FOR THIS PROCESS
	    
	    BANKSEL NVMADRH		;CLEAR HIGH BYTE OF ADDRESS REGISTER
	    CLRF    NVMADRH		;/
	    BANKSEL EEADR		;LOAD EEADR TO NVMADR TO SELECT NVM ADDRESS
	    MOVF    EEADR,0		;///
	    BANKSEL NVMADRL		;//
	    MOVWF   NVMADRL		;/
	    
	    BANKSEL EEDAT		;LOAD EEDAT TO NVMDAT TO STORE THAT DATA
	    MOVF    EEDAT,0		;///
	    BANKSEL NVMDAT		;//
	    MOVWF   NVMDAT		;/
	    
	    BANKSEL NVMCON1		;ALLOW FOR EEPROM TO BE WRITTEN TO
	    BSF	    NVMCON1,WREN	;/
	    
	    MOVLW   0X55		;EEPROM CHARGE PUMP
	    MOVWF   NVMCON2		;///
	    MOVLW   0XAA		;//
	    MOVWF   NVMCON2		;/
	    
	    BSF	    NVMCON1,WR		;START WRITE SEQUENCE TO EEPROM
	    
	    BSF	    INTCON,GIE		;ENABLE INTERRUPTS AGAIN
	    
	    BTFSC   NVMCON1,WR		;WAIT FOR WRITE SEQUENCE TO FINSIH
	    GOTO    $-2			;/
	    
	    BCF	    NVMCON1,WREN	;DISABLE WRITE ACCESS
	    
	    RETURN
;------END WRITE TO EEPROM------------------------------------------------------
;------READ EEPROM--------------------------------------------------------------
;REQUIRED: LOAD EEADR WITH THE CORRECT ADDRESS
READEEPROM
	    
	    BANKSEL NVMADR		;LOAD THE TARGET ADDRESS
	    CLRF    NVMADRH		;/////
	    BANKSEL EEADR		;////
	    MOVF    EEADR,0		;///
	    BANKSEL NVMADR		;//
	    MOVWF   NVMADRL		;/
	    
	    BANKSEL NVMCON1		;INITIATE READ PROCESS
	    BSF	    NVMCON1,RD		;/
	    
	    BANKSEL NVMDAT		;STORE READ EEPROM VALUE IN EEDAT VARIABLE
	    MOVF    NVMDAT,0		;///
	    BANKSEL EEDAT		;//
	    MOVWF   EEDAT		;/
	    
	    RETURN
;------END READ EEPROM----------------------------------------------------------
	    
;------I2C2FAIL-----------------------------------------------------------------
I2C2FAIL
	    BANKSEL SSP2CON2		;GENERATE STOP CONDITION
	    BSF	    SSP2CON2,PEN	;/
	    BTFSC   SSP2CON2,PEN	;WAIT FOR STOP CONDITION TO FINSIH
	    GOTO    $-2			;/
	    
	    RETLW   0X00		;RETURN WITH FAIL VALUE IN W REGISTER
;------END I2C2FAIL-------------------------------------------------------------
;------I2C2STOP-----------------------------------------------------------------
I2C2STOP
	    BANKSEL SSP2CON2		;GENERATE STOP CONDITION
	    BSF	    SSP2CON2,PEN	;/
	    BTFSC   SSP2CON2,PEN	;WAIT FOR STOP CONDITION TO FINSIH
	    GOTO    $-2			;/
	    
	    RETURN
;------END I2C2STOP-------------------------------------------------------------
;------I2C2TXADDRESS------------------------------------------------------------
;REQUIRED: LOAD I2C2ADR WITH ADDRESS TO TRANSMIT
I2C2TXADDRESS
	    
	    BANKSEL SSP2CON2		;GENERATE START CONDITION
	    BSF	    SSP2CON2,0		;/
	    BTFSC   SSP2CON2,0		;WAIT FOR START CONDITION TO FINSIH
	    GOTO    $-2			;/
	    
	    BANKSEL I2C2ADR		;LOAD THE ADDRESS INTO THE TX BUFFER
	    MOVF    I2C2ADR,0		;///
	    BANKSEL SSP2BUF		;//
	    MOVWF   SSP2BUF		;/
	    
	    BANKSEL SSP2STAT		;WAIT FOR BYTE TO FINISH TRANSMITING
	    BTFSC   SSP2STAT,R_W	;//
	    GOTO    $-2			;/
	    
	    BANKSEL SSP2CON2		;CHECK FOR ACKNOWLEDGE BIT
	    BTFSC   SSP2CON2,ACKSTAT	;/
	    GOTO    I2C2FAIL		;EXIT AND RETURN 0X00 IF NO ACK OCCOURED
	    
	    RETLW   0XFF		;ADDRESS TRANSMIT WAS SUCCESSFULL
;------END I2C2TXADDRESS--------------------------------------------------------
;------I2C2TXDATA---------------------------------------------------------------
;REQUIRED: LOAD I2C2DAT WITH DATA TO TRANSMIT
I2C2TXDATA
	    BANKSEL I2C2DAT		;LOAD DATA INTO THE TX BUFFER
	    MOVF    I2C2DAT,0		;///
	    BANKSEL SSP2BUF		;//
	    MOVWF   SSP2BUF		;/
	    
	    BANKSEL SSP2STAT		;WAIT FOR DATA TO TRANSMIT
	    BTFSC   SSP2STAT,R_W	;//
	    GOTO    $-2			;/
	    
	    BANKSEL SSP2CON2		;CHECK FOR ACKNOWLEDGE BIT
	    BTFSC   SSP2CON2,ACKSTAT	;/
	    GOTO    I2C2FAIL		;EXIT AND RETURN 0X00 IF NO ACK OCCOURED
	    
	    RETLW   0XFF		;DATA TRANSMIT WAS SUCCESSFULL
;------END I2C2TXDATA-----------------------------------------------------------
;------I2C2REQUESTFIRST---------------------------------------------------------
;REQUIRED: LOAD I2C2ADR WITH THE CORRECT REQUEST ADDRESS
I2C2REQUESTFIRST
	    
	    CALL    I2C2TXADDRESS	;SEND THE READ ADDRESS
	    
	    SUBLW   0X00		;TEST IF ADDRESS TRANSMIT WAS SUCCESSFUL
	    BTFSC   STATUS,Z		;/
	    RETLW   0X00		;EXIT AND RETURN 0X00 IF ADDRESS TX FAILED
	    
	    BANKSEL SSP2CON2		;ENABLE MASTER RECEIVE MODE
	    BSF	    SSP2CON2,RCEN	;/
	    
	    BTFSC   SSP2CON2,RCEN	;WAIT FOR DATA TO BE RECEIVED
	    GOTO    $-2			;/
	    
	    BANKSEL SSP2BUF		;SAVE DATA TO I2C2RXDAT
	    MOVF    SSP2BUF,0		;///
	    BANKSEL I2C2RXDAT		;//
	    MOVWF   I2C2RXDAT		;/
	    
	    BANKSEL SSP2CON2		;SET ACKNOWLEDGE TYPE TO !ACK
	    BCF	    SSP2CON2,ACKDT	;/
	    BSF	    SSP2CON2,ACKEN	;GENERATE !ACK
	    
	    BTFSC   SSP2CON2,ACKEN	;WAIT UNTIL ACKNOWLEDGE SEQUENCE IS DONE
	    GOTO    $-2			;/
	    
	    RETLW   0XFF		;FIRST DATA BYTE REQUEST SUCCESS
;------END I2C2REQUESTFIRST-----------------------------------------------------
;------I2C2REQUESTMID-----------------------------------------------------------
I2C2REQUESTMID
	    
	    BANKSEL SSP2CON2		;ENABLE MASTER RECEIVE MODE
	    BSF	    SSP2CON2,RCEN	;/
	    
	    BTFSC   SSP2CON2,RCEN	;WAIT FOR DATA TO BE RECEIVED
	    GOTO    $-2			;/
	    
	    BANKSEL SSP2BUF		;SAVE DATA TO I2C2RXDAT
	    MOVF    SSP2BUF,0		;///
	    BANKSEL I2C2RXDAT		;//
	    MOVWF   I2C2RXDAT		;/
	    
	    BANKSEL SSP2CON2		;SET ACKNOWLEDGE TYPE TO !ACK
	    BCF	    SSP2CON2,ACKDT	;/
	    BSF	    SSP2CON2,ACKEN	;GENERATE !ACK
	    
	    BTFSC   SSP2CON2,ACKEN	;WAIT UNTIL ACKNOWLEDGE SEQUENCE IS DONE
	    GOTO    $-2			;/
	    
	    RETLW   0XFF		;N'TH DATA BYTE REQUEST SUCCESS
;------END I2C2REQUESTMID-------------------------------------------------------
;------I2C2REQUESTLAST----------------------------------------------------------
I2C2REQUESTLAST
	    
	    BANKSEL SSP2CON2		;ENABLE MASTER RECEIVE MODE
	    BSF	    SSP2CON2,RCEN	;/
	    
	    BTFSC   SSP2CON2,RCEN	;WAIT FOR DATA TO BE RECEIVED
	    GOTO    $-2			;/
	    
	    BANKSEL SSP2BUF		;SAVE DATA TO I2C2RXDAT
	    MOVF    SSP2BUF,0		;///
	    BANKSEL I2C2RXDAT		;//
	    MOVWF   I2C2RXDAT		;/
	    
	    BANKSEL SSP2CON2		;SET ACKNOWLEDGE TYPE TO NACK
	    BSF	    SSP2CON2,ACKDT	;/
	    BSF	    SSP2CON2,ACKEN	;GENERATE NACK
	    
	    BTFSC   SSP2CON2,ACKEN	;WAIT UNTIL ACKNOWLEDGE SEQUENCE IS DONE
	    GOTO    $-2			;/
	    
	    RETLW   0XFF		;LAST DATA BYTE REQUST SUCCESS
;------END I2C2REQUESTLAST------------------------------------------------------

;------I2C2SCAN-----------------------------------------------------------------
I2C2SCAN
	    BANKSEL I2C2ADR		;START ADDRESS IS 0X10 (WILL INCREMENT TO 0X10)
	    MOVLW   0X0E		;//
	    MOVWF   I2C2ADR		;/
	    
SCANLOOP
	    
	    BANKSEL I2C2ADR		;GO TO NEXT POSSIBLE ADDRESS
	    INCF    I2C2ADR,1		;//
	    INCF    I2C2ADR,1		;/
	    
	    MOVLW   0X7F		;MAX ADDRESS IS 0X7F (ABOVE IS RESERVED)
	    CPFSLT  I2C2ADR,0		;/
	    RETLW   0X00		;RETURN FAIL CONDITION
	    
	    CALL    I2C2TXADDRESS	;SEND ADDRESS ON I2C BUS
	    SUBLW   0X00		;IF ADDRESS FAILED, REPEAT THE LOOP
	    BTFSC   STATUS,Z		;//
	    GOTO    SCANLOOP		;/
		
	    CALL    I2C2STOP		;END CONVERSATION MANUALLY ON SUCCESS
	    
	    BANKSEL I2C2ADR		;SAVE FOUND ADDRESS
	    MOVFF   I2C2ADR,SCANADR	;/
	    
	    RETLW   0XFF		;RETURN SUCCESS CONDITION
;------END I2C2SCAN-------------------------------------------------------------
;------SAVE SENSOR--------------------------------------------------------------
SAVESENSOR
	    
	    BANKSEL SENSOREEADR		;START SEARCHING ADDRESSES AT 0X0E (WILL INCREMENT TO 0X10)
	    MOVLW   0X0E		;//
	    MOVWF   SENSOREEADR		;/
	    
EESCANLOOP
	    
	    BANKSEL SENSOREEADR		;GO TO NEXT ADDRESS
	    INCF    SENSOREEADR,1	;//
	    INCF    SENSOREEADR,1	;/
	    
	    MOVLW   0X1F		;COULD NOT FIND ADDRESS
	    CPFSLT  SENSOREEADR,0	;/
	    RETLW   0X00		;RETURN FAIL CONDITION
	    
	    MOVFF   SENSOREEADR,EEADR	;READ EEPROM @ SENSOREEADR
	    CALL    READEEPROM		;
	    
	    MOVLW   0X00		;CHECK IF LOCATION IS EMPTY
	    CPFSEQ  EEDAT,0		;/
	    GOTO    EESCANLOOP		;IF LOCATION IS NOT EMPTY, CHECK NEXT
	    
	    ;---SELECT SENSOR REGISTER 0X3C (SENSOR IDENTIFICATION)-------------
	    
	    MOVFF   SCANADR,I2C2ADR	;TRANSMIT KNOWN ADDRESS TO START CONVERSATION
	    CALL    I2C2TXADDRESS	;/
	    SUBLW   0X00		;CHECK IF ADDRESS TRANSMIT WAS SUCCESSFULL
	    BTFSC   STATUS,Z		;//
	    GOTO    I2C2FAIL		;/
	    
	    BANKSEL I2C2DAT		;SELECT REGISTER 0X3C
	    MOVLW   0X3C		;///
	    MOVWF   I2C2DAT		;//
	    CALL    I2C2TXDATA		;/
	    SUBLW   0X00		;CHECK IF DATA TRANSMIT WAS SUCCESSFULL
	    BTFSC   STATUS,Z		;//
	    GOTO    I2C2FAIL		;/
	    
	    CALL    I2C2STOP		;STOP I2C CONVERSATION
	    
	    ;---READ AND STORE SENSOR IDENTIFICATION----------------------------
	    
	    BSF	    I2C2ADR,0		;SET LSB FOR READ REQUEST
	    CALL    I2C2REQUESTFIRST	;REQUEST FIRST BYTE (I2C2ADR DID NOT CHANGE)
	    SUBLW   0X00		;CHECK IF DATA REQUEST WAS SUCCESSFULL
	    BTFSC   STATUS,Z		;//
	    GOTO    I2C2FAIL		;/
	    
	    ARRAY   0X50,D'0'		;SELECT ARRAY ID[0]
	    BANKSEL I2C2RXDAT		;STORE RECEIVED DATA IN ID[0]
	    MOVFF   I2C2RXDAT,INDF0	;/
	    
	    CALL    I2C2REQUESTMID	;REQUEST SECOND BYTE
	    SUBLW   0X00		;CHECK IF DATA REQUEST WAS SUCCESSFULL
	    BTFSC   STATUS,Z		;//
	    GOTO    I2C2FAIL		;/
	    
	    ARRAY   0X50,D'1'		;SELECT ARRAY ID[1]
	    BANKSEL I2C2RXDAT		;STORE RECEIVED DATA IN ID[1]
	    MOVFF   I2C2RXDAT,INDF0	;/
	    
	    CALL    I2C2REQUESTMID	;REQUEST THIRD BYTE
	    SUBLW   0X00		;CHECK IF DATA REQUEST WAS SUCCESSFULL
	    BTFSC   STATUS,Z		;//
	    GOTO    I2C2FAIL		;/
	    
	    ARRAY   0X50,D'2'		;SELECT ARRAY ID[2]
	    BANKSEL I2C2RXDAT		;STORE RECEIVED DATA IN ID[2]
	    MOVFF   I2C2RXDAT,INDF0	;/
	    
	    CALL    I2C2REQUESTLAST	;REQUEST FOURTH BYTE
	    SUBLW   0X00		;CHECK IF DATA REQUEST WAS SUCCESSFULL
	    BTFSC   STATUS,Z		;//
	    GOTO    I2C2FAIL		;/
	    
	    ARRAY   0X50,D'3'		;SELECT ARRAY ID[3]
	    BANKSEL I2C2RXDAT		;STORE RECEIVED DATA IN ID[3]
	    MOVFF   I2C2RXDAT,INDF0	;/
	    
	    CALL    I2C2STOP		;STOP I2C CONVERSATION
	    
	    ;---ADDRESS REASSIGNMENT--------------------------------------------

	    BCF	    I2C2ADR,0		;CLEAR LSB FOR WRITE REQUEST
	    CALL    I2C2TXADDRESS	;TRANSMIT KNOWN ADDRESS TO START CONVERSATION
	    SUBLW   0X00		;CHECK IF ADDRESS TRANSMIT WAS SUCCESSFULL
	    BTFSC   STATUS,Z		;//
	    GOTO    I2C2FAIL		;/
	    
	    BANKSEL I2C2DAT		;SELECT REGISTER 0X22
	    MOVLW   0X22		;///
	    MOVWF   I2C2DAT		;//
	    CALL    I2C2TXDATA		;/
	    SUBLW   0X00		;CHECK IF DATA TRANSMIT WAS SUCCESSFULL
	    BTFSC   STATUS,Z		;//
	    GOTO    I2C2FAIL		;/

	    BANKSEL SENSOREEADR		;TRANSMIT SENSOREEADR TO ASSIGN NEW ADDRESS
	    BCF	    STATUS,C		;////
	    RRCF    SENSOREEADR,0	;///
	    MOVWF   I2C2DAT		;//
	    CALL    I2C2TXDATA		;/
	    SUBLW   0X00		;CHECK IF DATA TRANSMIT WAS SUCCESSFULL
	    BTFSC   STATUS,Z		;//
	    GOTO    I2C2FAIL		;/
	    
	    CALL    I2C2STOP		;STOP I2C CONVERSATION
	    
	    ;---SAVE PROCESSES--------------------------------------------------
	    
	    ARRAY   0X50,D'0'		;PREPARE ID[0] TO BE ACCESSED
	    
	    MOVLW   'L'			;CHECK FOR LUNA
	    CPFSEQ  INDF0,0		;///
	    GOTO    CHECK1		;//
	    GOTO    LUNADETECT		;/
	    
CHECK1
	    
	    MOVLW   'I'			;CHECK FOR INFRARED BOARD
	    CPFSEQ  INDF0,0		;///
	    GOTO    CHECK2		;//
	    GOTO    INFDETECT		;/
	    
CHECK2
	    
	    RETLW   0X00		;SENSOR UNKNOWN RETURN FAIL CONDITION
	    
LUNADETECT
	    BANKSEL EEDAT		;SAVE SENSOR TYPE LUNA TO EEPROM AT ADDRESS
	    MOVLW   'L'			;////
	    MOVWF   EEDAT		;///
	    MOVFF   SENSOREEADR,EEADR	;//
	    CALL    WRITETOEEPROM	;/
	    
	    BANKSEL EEDAT		;SAVE 1 DATA BYTE IN EEPROM AT ADDRESS +1
	    MOVLW   0X01		;////
	    MOVWF   EEDAT		;///
	    IORWF   EEADR,1		;//
	    CALL    WRITETOEEPROM	;/
	    
	    MOVFF   SENSOREEADR,I2C2ADR	;USE NEW ADDRESS TO FIND LUNA
	    CALL    I2C2TXADDRESS	;TRANSMIT NEW ADDRESS TO START CONVERSATION
	    SUBLW   0X00		;CHECK IF ADDRESS TRANSMIT WAS SUCCESSFULL
	    BTFSC   STATUS,Z		;//
	    GOTO    I2C2FAIL		;/
	    
	    BANKSEL I2C2DAT		;SELECT REGISTER 0X20 (SAVE REGISTER)
	    MOVLW   0X20		;///
	    MOVWF   I2C2DAT		;//
	    CALL    I2C2TXDATA		;/
	    SUBLW   0X00		;CHECK IF DATA TRANSMIT WAS SUCCESSFULL
	    BTFSC   STATUS,Z		;//
	    GOTO    I2C2FAIL		;/
	    
	    BANKSEL I2C2DAT		;TRANSMIT 0X01 TO START SAVE PROCESS
	    MOVLW   0X01		;///
	    MOVWF   I2C2DAT		;//
	    CALL    I2C2TXDATA		;/
	    SUBLW   0X00		;CHECK IF DATA TRANSMIT WAS SUCCESSFULL
	    BTFSC   STATUS,Z		;//
	    GOTO    I2C2FAIL		;/
	    
	    CALL    I2C2STOP		;STOP I2C CONVERSATION
	    
	    GOTO    DETECTCONTINUE	;GOTO END OF PROCESS
	    
INFDETECT
	    
	    BANKSEL EEDAT		;SAVE SENSOR TYPE LUNA TO EEPROM AT ADDRESS
	    MOVLW   'I'			;////
	    MOVWF   EEDAT		;///
	    MOVFF   SENSOREEADR,EEADR	;//
	    CALL    WRITETOEEPROM	;/
	    
	    BANKSEL EEDAT		;SAVE NUMBER OF DATA BYTES TO RECEIVE FROM SENSOR
	    ARRAY   0X50,D'3'		;ID[3] HOLDS NUMBER OF SENSORS, SAVE TO EEADR + 1
	    MOVF    INDF0,0		;/////
	    MOVWF   EEDAT		;////
	    MOVLW   0X01		;///
	    IORWF   EEADR,1		;//
	    CALL    WRITETOEEPROM	;/
	    
	    GOTO    DETECTCONTINUE	;GOTO END OF PROCESS
	    
DETECTCONTINUE
	    
	    RETLW   0XFF		;RETURN SUCCESS CONDITION
;------END SAVE SENSOR----------------------------------------------------------
;------ERASE SENSORS------------------------------------------------------------
ERASESENSORS

	    BCF	    PFLAG,3		;CLEAR PFLAG TO ONLY ENTER ONCE
	   
	    MOVLW   0X10		;SET COUNT TO FIRST SENSOR LOCATION
	    MOVWF   COUNT		;/
	    CLRF    EEDAT		;LOAD 0X00 TO BE PUT INTO SENSOR LOCATIONS
	    
ERASELOOP
	    
	    CLRWDT			;CLEAR WATCHDOG TIMER, WRITING TO EEPROM TAKES A WHILE
	    
	    BANKSEL COUNT		;LOAD NEXT SENSOR TO EEPROM LOCATION
	    MOVF    COUNT,0		;//
	    MOVWF   EEADR		;/
	    CALL    WRITETOEEPROM	;ERASE CURRENT EEPROM LOCATION
	    
	    BANKSEL COUNT		;GOTO NEXT EEPROM LOCATION
	    INCF    COUNT,1		;/
	    
	    BANKSEL COUNT		;STOP ERASING EEPROM AFTER LOCATION 0X22
	    MOVLW   0X23		;///
	    CPFSEQ  COUNT,0		;//
	    GOTO    ERASELOOP		;/
	    
	    BANKSEL LATA		;SHOW 0X0F ON LEDS TO SHOW EEPROM ERASED
	    MOVLW   0X0F		;//
	    MOVWF   LATA		;/
	    
	    RETURN
;------ERASE SENSORS------------------------------------------------------------
	    
;------TMR2HANDLE---------------------------------------------------------------
TMR2HANDLE
	    BCF	    PFLAG,6		;CLEAR PERSONAL FLAG
	    
	    MOVLW   D'150'		;IF T2DELAY IS NOT GREATER THAN 150 INCREMENT T2DELAY
	    CPFSGT  T2DELAY,0		;//
	    INCF    T2DELAY,1		;/
	    
	    MOVLW   D'150'		;ONCE DELAY IS >= 150 CALL T2TICK TO SHIFT LEDS
	    CPFSLT  T2DELAY,0		;//
	    CALL    T2TICK		;/
	    
	    RETURN
;------END TMR2HANDLE-----------------------------------------------------------
;---------T2TICK----------------------------------------------------------------
T2TICK
	    INCF    T2COUNT,1		;INCREMENT T2COUNT
	    MOVLW   D'6'		;IF T2COUNT IS >= 6 CALL T2SHIFT
	    CPFSLT  T2COUNT,0		;//
	    CALL    T2SHIFT		;/
	    
	    RETURN
;---------END T2TICK------------------------------------------------------------
;-------------T2SHIFT-----------------------------------------------------------
T2SHIFT
	    CLRF    T2COUNT		;RESET T2COUNT
	    BCF	    STATUS,C		;CLEAR CARRY TO ONLY HAVE ONE BIT SHIFTING
	    RLCF    PORTA,0		;SHIFT PORTA LEFT BY 1
	    MOVWF   LATA		;/
	    
	    MOVF    PORTA,0		;CARRY ONLY 1 BIT FROM MSB TO LSB
	    BTFSC   STATUS,Z		;//
	    BSF	    LATA,0		;/
	    
	    RETURN
;-------------END T2SHIFT-------------------------------------------------------
	    
;------REQUESTSENSORDATA--------------------------------------------------------
REQUESTSENSORDATA
	    
	    BCF	    PFLAG,2
	    
	    ;---DETERMINE THE LAST SENSOR CONNECTED-----------------------------
	    
	    BANKSEL LASTSENSOR		;FIRST SENSOR IS ALLWAYS 0X10 (WILL INCREMENT TO 0X10)
	    MOVLW   0X0E		;///
	    MOVWF   LASTSENSOR		;//
	    MOVWF   COUNT		;/
	    
	    CLRF    BYTECOUNT		;CLEAR BYTE COUNT FOR A FRESH COUNT
	    
LASTSENSORLOOP
	    
	    BANKSEL COUNT		;INCREMENT TO THE NEXT ADDRESS
	    INCF    COUNT,1		;//
	    INCF    COUNT,1		;/
	    
	    MOVF    COUNT,0		;LOAD NEW ADDRESS TO READ FROM EEPROM
	    MOVWF   EEADR		;//
	    CALL    READEEPROM		;/
	    MOVLW   0X00		;IF READ VALUE IS EMPTY, DO NOT REASSGIN LASTSENSOR
	    CPFSEQ  EEDAT,0		;//
	    MOVFF   COUNT,LASTSENSOR	;/
	    
	    MOVLW   0X1E		;LAST VALUE TO EVALUATE IS 0X1E
	    CPFSEQ  COUNT		;//
	    GOTO    LASTSENSORLOOP	;/
	    
	    MOVLW   0X0E		;RESET COUNT TO FIRST SENSOR
	    MOVWF   COUNT		;/
	    
	    ;---READ DATA FROM ALL SENSORS--------------------------------------
	    
READDATALOOP
	    
	    BANKSEL COUNT		;GOTO NEXT SENSOR
	    INCF    COUNT,1		;//
	    INCF    COUNT,1		;/
	    
	    MOVF    COUNT,0		;GET SENSOR ID FROM EEPROM
	    MOVWF   EEADR		;///
	    CALL    READEEPROM		;//
	    MOVFF   EEDAT,SENSORID	;/
	    
	    MOVLW   0X01		;GET NUMBER OF DISTANCE BYTES FROM EEPROM
	    IORWF   COUNT,0		;////
	    MOVWF   EEADR		;///
	    CALL    READEEPROM		;//
	    MOVFF   EEDAT,DISTBYTES	;/
	    
	    MOVFF   COUNT,I2C2ADR	;TRANSMIT KNOWN ADDRESS TO START CONVERSATION
	    CALL    I2C2TXADDRESS	;/
	    SUBLW   0X00		;CHECK IF ADDRESS TRANSMIT WAS SUCCESSFULL
	    BTFSC   STATUS,Z		;//
	    GOTO    I2C2FAIL		;/
	    
	    BANKSEL I2C2DAT		;SEND 0X00 TO SELECT FIRST BYTE REGISTER
	    MOVLW   0X00		;///
	    MOVWF   I2C2DAT		;//
	    CALL    I2C2TXDATA		;/
	    SUBLW   0X00		;CHECK IF DATA TRANSMIT WAS SUCCESSFULL
	    BTFSC   STATUS,Z		;//
	    GOTO    I2C2FAIL		;/
	    
	    CALL    I2C2STOP		;END I2C CONVERSATION
	    
	    ;---REQUEST THREE BYTES OF DISTANCE DATA----------------------------
	    
	    BANKSEL I2C2ADR
	    BSF	    I2C2ADR,0		;SET LSB FOR READ REQUEST
	    CALL    I2C2REQUESTFIRST	;REQUEST FIRST BYTE (ADDRESS DID NOT CHANGE)
	    SUBLW   0X00		;CHECK IF DATA REQUEST WAS SUCCESSFULL
	    BTFSC   STATUS,Z		;//
	    GOTO    I2C2FAIL		;/
	    
	    ARRAY   0X60,0		;SELECT ARRAY DISTDATA[0]
	    BANKSEL I2C2RXDAT		;STORE RECEIVED DATA IN DISTDATA[0]
	    MOVFF   I2C2RXDAT,INDF0	;/
	    
	    CALL    I2C2REQUESTMID	;REQUEST SECOND BYTE
	    SUBLW   0X00		;CHECK IF DATA REQUEST WAS SUCCESSFULL
	    BTFSC   STATUS,Z		;//
	    GOTO    I2C2FAIL		;/
	    
	    ARRAY   0X60,1		;SELECT ARRAY DISTDATA[1]
	    BANKSEL I2C2RXDAT		;STORE RECEIVED DATA IN DISTDATA[1]
	    MOVFF   I2C2RXDAT,INDF0	;/
	    
	    CALL    I2C2REQUESTLAST	;REQUEST THIRD BYTE
	    SUBLW   0X00		;CHECK IF DATA REQUEST WAS SUCCESSFULL
	    BTFSC   STATUS,Z		;//
	    GOTO    I2C2FAIL		;/
	    
	    ARRAY   0X60,2		;SELECT ARRAY DISTDATA[2]
	    BANKSEL I2C2RXDAT		;STORE RECEIVED DATA IN DISTDATA[2]
	    MOVFF   I2C2RXDAT,INDF0	;/
	    
	    CALL    I2C2STOP		;STOP I2C CONVERSATION
	    
	    ;---IF SENSOR IS LUNA, MAKE MAX VALUE 0XFF--------------------------
	    
	    MOVLW   'I'			;IF SENSOR IS NOT INFRARED ASSUME IT IS LUNA
	    CPFSEQ  SENSORID		;//
	    GOTO    LUNAVALPROCESS	;//
	    GOTO    INFRVALPROCESS	;/
	    
	    ;---CHECK IF ALL SENSORS HAVE BEEN READ FROM------------------------
	    
STOREEND
	    
	    BANKSEL COUNT
	    MOVF    COUNT,0		;DO NOT REPEAT IF THIS WAS THE LAST SENSOR
	    CPFSEQ  LASTSENSOR,0	;//
	    GOTO    READDATALOOP	;/

	    CLRF    FSR0H		;LOAD THE NUMBER OF SENSORS FOUND INTO THE I2C1SLAVE REGISTER ARRAY
	    MOVLW   0X20		;/////
	    MOVWF   FSR0L		;////
	    BANKSEL BYTECOUNT		;///
	    MOVF    BYTECOUNT,0		;//
	    MOVWF   INDF0		;/
	    
	    BANKSEL LATA		;SHOW THE NUMBER OF SENSORS THE BOARD WAS ABLE TO INTERFACE WITH
	    MOVWF   LATA		;/
	    
	    RETLW   0XFF		;RETURN SUCCESS CONDITION
	    
;------END REQUESTSENSORDATA----------------------------------------------------
;---------LUNA VAL PROCESS------------------------------------------------------
LUNAVALPROCESS
	    
	    ARRAY   0X60,D'1'		;DO NOT CAP DISTANCE IF THERE WAS NO ROLLOVER
	    MOVLW   0X00    		;///
	    CPFSEQ  INDF0,0		;//
	    CALL    LUNAVALCAP		;/
	    
	    BANKSEL BYTECOUNT		;
	    INCF    BYTECOUNT,1		;COUNT THIS BYTE
	    
	    ARRAY   0X60,0		;GET DISTANCE DATA FROM DISTDATA ARRAY
	    MOVFF   INDF0,DISTDATHOLD	;/
	    ARRAY   0X20,BYTECOUNT	;SELECT I2C1REG ARRAY @ INDEX BYTECOUNT
	    MOVFF   DISTDATHOLD,INDF0	;STORE DIST DATA IN I2C1REGISTERS
	    
	    GOTO    STOREEND
;---------END LUNA VAL PROCESS--------------------------------------------------
;------------LUNA VAL CAP-------------------------------------------------------
LUNAVALCAP
	    ARRAY   0X60,D'0'		;SET DISTANCE VALUE TO 0XFF IF THERE WAS ROLLOVER
	    MOVLW   0XFF		;//
	    MOVWF   INDF0		;/
	    RETURN
;------------END LUNA VAL CAP---------------------------------------------------
;---------INFR VAL PROCESS------------------------------------------------------
INFRVALPROCESS
	    
	    CALL    INFRVALSTORE1	;INFR BOARD WILL ALLWAYS HAVE AT LEAST 1 DIST BYTE
	    
	    BANKSEL DISTBYTES		;IF THERE ARE LESS THAN 1 DIST BYTES DON'T REQUEST A SECOND 
	    MOVLW   0X01		;///
	    CPFSLT  DISTBYTES,0		;//
	    CALL    INFRVALSTORE2	;/
	    
	    BANKSEL DISTBYTES		;IF THERE ARE LESS THAN 2 DIST BYTES DON'T REQUEST A THIRD
	    MOVLW   0X02		;///
	    CPFSLT  DISTBYTES,0		;//
	    CALL    INFRVALSTORE3	;/
	    
	    GOTO    STOREEND
;---------END INFR VAL PROCESS--------------------------------------------------
;------------INFR VAL STORE1----------------------------------------------------
INFRVALSTORE1
	    INCF    BYTECOUNT,1		;COUNT THIS BYTE
	    ARRAY   0X60,0		;GET DISTANCE DATA FROM DISTDATA ARRAY
	    MOVFF   INDF0,DISTDATHOLD	;/
	    ARRAY   0X20,BYTECOUNT	;SELECT I2C1REG ARRAY @ INDEX BYTECOUNT
	    MOVFF   DISTDATHOLD,INDF0	;STORE DIST DATA IN I2C1REGISTERS
	    RETURN
;------------END INFR VAL STORE1------------------------------------------------
;------------INFR VAL STORE2----------------------------------------------------
INFRVALSTORE2
	    INCF    BYTECOUNT,1		;COUNT THIS BYTE
	    ARRAY   0X60,1		;GET DISTANCE DATA FROM DISTDATA ARRAY
	    MOVFF   INDF0,DISTDATHOLD	;/
	    ARRAY   0X20,BYTECOUNT	;SELECT I2C1REG ARRAY @ INDEX BYTECOUNT
	    MOVFF   DISTDATHOLD,INDF0	;STORE DIST DATA IN I2C1REGISTERS
	    RETURN
;------------END INFR VAL STORE2------------------------------------------------
;------------INFR VAL STORE3----------------------------------------------------
INFRVALSTORE3
	    INCF    BYTECOUNT,1		;COUNT THIS BYTE   
	    ARRAY   0X60,2		;GET DISTANCE DATA FROM DISTDATA ARRAY
	    MOVFF   INDF0,DISTDATHOLD	;/
	    ARRAY   0X20,BYTECOUNT	;SELECT I2C1REG ARRAY @ INDEX BYTECOUNT
	    MOVFF   DISTDATHOLD,INDF0	;STORE DIST DATA IN I2C1REGISTERS
	    RETURN
;------------END INFR VAL STORE3------------------------------------------------
;------SEARCH PROCESS-----------------------------------------------------------
SEARCHPROCESS
	    BCF	    PFLAG,4		;CLEAR PFLAG TO ONLY ENTER ONCE
	    
	    CALL    I2C2SCAN		;INITIATE SCAN PROCESS
	    SUBLW   0X00		;CHECK IF SCAN PROCESS WAS SUCCESSFULL
	    BTFSC   STATUS,Z		;//
	    GOTO    SEARCHFAIL		;/
	    
	    CALL    SAVESENSOR		;INITIATE SAVE SENSOR PROCESS
	    SUBLW   0X00		;CHECK IF SAVE SENSOR PROCESS WAS SUCCESSFULL
	    BTFSC   STATUS,Z		;//
	    GOTO    SAVEFAIL		;/
	    
	    BANKSEL SENSOREEADR		;SHOW ADDRESS OF NEW SENSOR ON SUCCESS
	    MOVF    SENSOREEADR,0	;//
	    MOVWF   LATA		;/
	    
	    RETURN
;------END SEARCH PROCESS-------------------------------------------------------
;------SEARCH FAIL--------------------------------------------------------------
SEARCHFAIL
	    BANKSEL LATA		;SHOW 0XEE IF SEARCH FAILED TO FIND SENSOR
	    MOVLW   0XEE		;//
	    MOVWF   LATA		;/
	    
	    RETURN
;------END SEARCH FAIL----------------------------------------------------------
;------SAVE FAIL----------------------------------------------------------------
SAVEFAIL
	    BANKSEL LATA		;SHOW 0XCC IF SENSOR COULD NOT BE SAVED
	    MOVLW   0XCC		;//
	    MOVWF   LATA		;/
	    
	    RETURN
;------END SAVE FAIL------------------------------------------------------------
;------IOCHANDLE----------------------------------------------------------------
IOCHANDLE
	    BCF	    PFLAG,5		;CLEAR PFLAG TO ONLY ENTER ONCE
	    
	    BANKSEL PORTC		;IF RC4 IS PRESSED SET PFLAG TO ERASE SENSORS
	    BTFSC   PORTC,4		;//
	    BSF	    PFLAG,3		;/
	    BTFSC   PORTC,4		;ALSO CLEAR T2DELAY TO STOP LED SHIFTING
	    CLRF    T2DELAY		;/
	    
	    BANKSEL PORTC		;IF RC5 IS PRESSED SET PFLAG TO START SEARCH PROCESS
	    BTFSC   PORTC,5		;//
	    BSF	    PFLAG,4		;/
	    BTFSC   PORTC,5		;ALSO CLEAR T2DELAY TO STOP LED SHIFTING
	    CLRF    T2DELAY		;/
	    
	    RETURN
;------END IOCHANDLE------------------------------------------------------------
	    
;===END FUNCTIONS===============================================================
;===MAINBEGIN===================================================================
MAINBEGIN
	    
	    BANKSEL PFLAG		;PFLAG[6] IS LED SHIFTING PROCESS
	    BTFSC   PFLAG,6		;//
	    CALL    TMR2HANDLE		;/
	    
	    BANKSEL PFLAG		;PFLAG[5] IS IOC HANDLE PROCESS
	    BTFSC   PFLAG,5		;//
	    CALL    IOCHANDLE		;/
	    
	    BANKSEL PFLAG		;PFLAG[4] IS SEARCH FOR NEW SENSOR FLAG
	    BTFSC   PFLAG,4		;//  
	    CALL    SEARCHPROCESS	;/
	    
	    BANKSEL PFLAG		;PFLAG[3] IS ERASE EEPROM FLAG
	    BTFSC   PFLAG,3		;//
	    CALL    ERASESENSORS	;/
	    
	    BANKSEL PFLAG		;PFLAG[2] REQUESTS DATA FROM KNOWN SENSORS
	    BTFSC   PFLAG,2		;//
	    CALL    REQUESTSENSORDATA	;/
	    
	    CLRWDT			;CLEAR WATCHDOG TIMER
	    
	    GOTO    MAINBEGIN		;MAIN LOOP END
;===END MAINBEGIN===============================================================
	    
    END


