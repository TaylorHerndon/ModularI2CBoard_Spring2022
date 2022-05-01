/*******************************************************************************
 * 
 *                        I2C Sensor Board Project
 *                       Created By: Taylor Herndon
 *                              Spring 2022
 * 
 ******************************************************************************
 * Summary:
 *     This program has two primary functions. The first of which is to
 * search for and interface with sensors through I2C. The program, once in
 * programming mode, will allow the user to search for new sensors attached to
 * the SSP2 I2C bus line. Once a new sensor is found it will reassign the address
 * of the sensor and log its address in EEPROM. The program is capable of
 * interfacing with up to 8 different sensors at a time. There are two different
 * sensors available to interface with. These are either the TF-LUNA Lidar 
 * or the Infrared sensor I2C board developed by Kendall Callister. 
 *     The second function of this board is to be an I2C slave device to an
 * external master board. The slave address of this board is physically selected
 * by the DIP switch array on PORTB. The slave interface for this board is 
 * consistent with typical I2C slave device interfaces. A write command to the
 * slave device will select the I2C register that will be read out first when
 * a read command is sent to the device. When a read command is detected, this
 * board will load the selected register to the buffer for it to be clocked out
 * by the master device. The 0 register will indicate the number of data bytes
 * that are loaded into I2C registers. Registers 1-24 are distance data registers.
 * Each of the distance data registers contain the data of a unique distance
 * sensor. The Registers 25-28 are the board identification registers. These
 * values are static and are used to help identify the board. The values loaded
 * into these registers are ASCII 'M' 'O' 'D' 'X'.
 ******************************************************************************/

//=====Config Bits============================================================//

// PIC18F46K40 Configuration Bit Settings
// 'C' source line config statements
// CONFIG1L
#pragma config FEXTOSC = OFF     // External Oscillator not enabled.
#pragma config RSTOSC = HFINTOSC_64MHZ// Power-up default value for COSC bits (HFINTOSC with HFFRQ = 64 MHz and CDIV = 1:1)

// CONFIG1H
#pragma config CLKOUTEN = OFF   // Clock Out Enable bit (CLKOUT function is disabled)
#pragma config CSWEN = ON       // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor enabled)

// CONFIG2L
#pragma config MCLRE = EXTMCLR  // Master Clear Enable bit (If LVP = 0, MCLR pin is MCLR; If LVP = 1, RE3 pin function is MCLR )
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (Power up timer disabled)
#pragma config LPBOREN = OFF    // Low-power BOR enable bit (ULPBOR disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable bits (Brown-out Reset disabled)

// CONFIG2H
#pragma config BORV = VBOR_2P45 // Brown Out Reset Voltage selection bits (Brown-out Reset Voltage (VBOR) set to 2.45V)
#pragma config ZCD = OFF        // ZCD Disable bit (ZCD disabled. ZCD can be enabled by setting the ZCDSEN bit of ZCDCON)
#pragma config PPS1WAY = ON     // PPSLOCK bit One-Way Set Enable bit (PPSLOCK bit can be cleared and set only once; PPS registers remain locked after one clear/set cycle)
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config DEBUG = OFF      // Debugger Enable bit (Background debugger disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Extended Instruction Set and Indexed Addressing Mode disabled)

// CONFIG3L
#pragma config WDTCPS = WDTCPS_31// WDT Period Select bits (Divider ratio 1:65536; software control of WDTPS)
#pragma config WDTE = OFF       // WDT operating mode (WDT Disabled)

// CONFIG3H
#pragma config WDTCWS = WDTCWS_7// WDT Window Select bits (window always open (100%); software control; keyed access not required)
#pragma config WDTCCS = SC      // WDT input clock selector (Software Control)

// CONFIG4L
#pragma config WRT0 = OFF       // Write Protection Block 0 (Block 0 (000800-003FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection Block 1 (Block 1 (004000-007FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection Block 2 (Block 2 (008000-00BFFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection Block 3 (Block 3 (00C000-00FFFFh) not write-protected)

// CONFIG4H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-30000Bh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)
#pragma config SCANE = ON       // Scanner Enable bit (Scanner module is available for use, SCANMD bit can control the module)
#pragma config LVP = ON         // Low Voltage Programming Enable bit (Low voltage programming enabled. MCLR/VPP pin function is MCLR. MCLRE configuration bit is ignored)

// CONFIG5L
#pragma config CP = OFF         // UserNVM Program Memory Code Protection bit (UserNVM code protection disabled)
#pragma config CPD = OFF        // DataNVM Memory Code Protection bit (DataNVM code protection disabled)

// CONFIG5H

// CONFIG6L
#pragma config EBTR0 = OFF      // Table Read Protection Block 0 (Block 0 (000800-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection Block 1 (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection Block 2 (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection Block 3 (Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks)

// CONFIG6H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include "xc.h" //Include embedded c header file
#include <proc/pic18f46k40.h> //Possibly not needed, depends on XC8 compiler install

//=====Configbits End=========================================================//
//=====Config Start===========================================================//
//--------Variables-----------------------------------------------------------//

//pFlag [7] [6] [5] [4] [3] [2] [1] [0]
// [7]: TMR4 Handle Flag (Debugging)
// [6]: TMR2 Handle Flag
// [5]: IOC Handle Flag
// [4]: Erase EEPROM Flag
// [3]: Search for New Sensor Flag
// [2]: Request Data From Sensors Flag
// [1]: 
// [0]: 

char pFlag = 0x00; //Personal Flag Register Refer to pFlag register bit functions

char scanAddress = 0; //Sensor scanning and saving variables
char sensorEEADR = 0; ///

char T2Count = 0; //Timer Variables
char T2Delay = 0; ////
char T4Count = 0; ///

//I2C1Registers[] [0] [1-24] [25-28]
//[0]: Number of distance data bytes ready to transmit
//[01-24] : Distance data registers (Register 12 = Sensor 12 Distance Data)
//[25-28] : Board Identification Registers [M]-[O]-[D]-[X]

char I2C1Registers[29]; //Slave data holding registers
char I2C1RegIndex = 0;  //Variable to index the slave data registers
char I2C1Status = 0;    //I2C Conversation status register
char I2C1RxData;        //Received data holding register

char I2C2Data = 0; //Data holding register for SSP2/I2C2 Module
char ID[4]; //Sensor Identification Code
char distData[3]; //Distance data bytes from sensors

//EEPROM Data Storage [0-0x0F] : [0x10 - 0x1F] : [0x20 - 0xFF+]
//[0x00 - 0x0F] : Not Used
//[0x20 - 0xFF+]: Not Used
//[0x10 - 0x1F] : Sensor Storage Registers
//   [x0h] : Identifier (Type of Sensor) : 'L' = Luna, 'I' = Infrared
//   [x1h] : Number of distance bytes to request

//--------Variables End-------------------------------------------------------//
//--------Modules Config Routines---------------------------------------------//

void PortsConfig(void)
{
    PPSLOCK = 0x55;             //Unlock PPS to set I2C pins
    PPSLOCK = 0xAA;             ////
    PPSLOCKbits.PPSLOCKED = 0;  ///
    
    RC0PPS = 0x10; //Set RC0 as MSSP1 SDA
    RC1PPS = 0x0F; //Set RC1 as MSSP1 SCL    
    SSP1DATPPS = 0b00010000; //SSP1DAT input set as RC0
    SSP1CLKPPS = 0b00010001; //SSP1CLK input set as RC1
    
    RD0PPS = 0x12; //Set RC2 as MSSP2 SDA
    RD1PPS = 0x11; //Set RC3 as MSSP2 SCL
    SSP2DATPPS = 0b00011000; //SSP2DAT input set as RD0
    SSP2CLKPPS = 0b00011001; //SSP2CLK input set as RD1
    
    PPSLOCK = 0x55;            //Lock PPS
    PPSLOCK = 0xAA;            ////
    PPSLOCKbits.PPSLOCKED = 1; ///
    
    LATA = 0x00; //Stop Unwanted outputs
    LATB = 0x00; //
    LATC = 0x00; //
    LATD = 0x00; //
    
    ANSELA = 0x00; //All Ports are digital IOs
    ANSELB = 0x00; //
    ANSELC = 0x00; //
    ANSELD = 0x00; //
    
    TRISA = 0x00; //PortA is Debugging LEDs
    TRISB = 0xFF; //PortB is Slave Address DIP Switch
    TRISC = 0xFF; //PortC is Programming Functions and I2C
    TRISD = 0xFF; //
}

void IOCConfig(void)
{
    IOCCP = 0b00110000; //Enable Rising Edge Interrupts on RC4 and RC5
    IOCCN = 0b00000000; //Disable Falling Edge Interrupts
    
    PIR0bits.IOCIF = 0; //Clear any unwanted flags
    PIE0bits.IOCIE = 1; //Enable On-Change Interrupts
}

void TMR2Config(void)
{
    //TMR2 : Fosc/4 : Pre = 1:128 : Post = 1:16 : PR = 250 : Tick = 32ms
    T2CLKbits.T2CS = 0b0001; //Select Fosc/4 for timer clk
    T2CONbits.T2CKPS = 0b111; //1:128 Prescaler
    T2CONbits.T2OUTPS = 0b1111; //1:16 Postscaler
    T2PR = 250; //250 Period register
    
    T2HLT = 0x00; //Standard Timer Operation
    T2RST = 0xFF; //No external resets
    
    T2CONbits.T2ON = 1; //Enable TMR2
    
    PIR4bits.TMR2IF = 0; //Clear unwanted TMR2 flags
    PIE4bits.TMR2IE = 1; //Enable TMR2 Interrupts
}

void TMR4Config(void)
{
    //TMR4 : Fosc/4 : Pre = 1:128 : Post = 1:16 : PR = 250 : Tick = 32ms
    T4CLKbits.T4CS = 0b0001; //Select Fosc/4 for timer clk
    T4CONbits.T4CKPS = 0b111; //1:128 Prescaler
    T4CONbits.T4OUTPS = 0b1111; //1:16 Postscaler
    T4PR = 250; //250 Period Register
    
    T4HLT = 0x00; //Standard Timer Operation
    T4RST = 0xFF; //No external resets
    
    PIR4bits.TMR4IF = 0; //Clear unwanted TMR4 flags
    PIE4bits.TMR4IE = 1; //Enable TMR4 interrupts
    T4CONbits.T4ON = 1; //Enable TMR4
}

void IntConfig(void)
{   
    PIR1 = 0; //Clear all unwanted flags on startup
    PIR2 = 0; //////
    PIR3 = 0; /////
    PIR4 = 0; ////
    PIR5 = 0; ///
    
    INTCONbits.IPEN = 0; //Disable Interrupt Priority Levels
    INTCONbits.PEIE = 1; //Enable Peripheral Interrupts
    INTCONbits.GIE = 1;  //Enable Global Interrupts
}

void SSP1Config (void)
{
    char slaveAdd = 0;
    
    //MSSP1 : RC0 = SDA1 : RC1 = SCL1 : Slave to External Master Board
    //Set SSP1 slave address from PORTB, minimum value for address is 0x10
    slaveAdd = (~(PORTB << 1)) & 0b01111110;
    if(slaveAdd < 0x10) {SSP1ADD = 0x10;} else {SSP1ADD = slaveAdd;}
    SSP1MSK = 0xFF; //Use all bits to match address
    
    SSP1CON1bits.CKP = 1;       //Release Clock
    SSP1CON1bits.SSPM = 0b0110; //7 Bit Address Slave Mode
    SSP1CON2bits.GCEN = 0;      //Disable General Call
    
    SSP1STATbits.SMP = 0; //Slew rate control set for 400kHz baud.
    SSP1STATbits.CKE = 0; //Disable SMBus Specific Inputs
    
    SSP1MSK = 0xFF; //Use all bits to match address
    
    SSP1CON3bits.PCIE = 1; //Enable interrupts on stop condition detect
    SSP1CON3bits.SCIE = 0; //Disable interrupts on start condition detect
    
    PIR3bits.SSP1IF = 0; //Clear unwanted SSP flags
    PIE3bits.SSP1IE = 1; //Enable SSP1 interrupts
    
    SSP1CON1bits.SSPEN = 1; //Enable I2C Port
}

void SSP2Config (void)
{
    //MSSP2 : RC2 = SDA2 : RC3 = SCL2 : Master to distance sensors
    SSP2ADD = 39; //Set BRG divider for 400kHz baud.    
    SSP2CON1bits.SSPM = 0b1000; //I2C Master Mode
    SSP2STATbits.SMP = 0; //Slew Rate set for 400kHz
    SSP2STATbits.CKE = 0; //Disable SMBus specific inputs
    
    SSP2CON1bits.SSPEN = 1; //Enable Serial Port
}

void NVMConfig(void)
{
    NVMCON1bits.NVMREG = 0b00; //Specify access to Data EEPROM
    NVMCON1bits.WREN = 0; //Disable write access on startup  
}

//--------Modules Config Routines End-----------------------------------------//
//=====Config End=============================================================//
//=====Functions Start========================================================//

void WriteToEEPROM(char address, char val)
{
    INTCONbits.GIE = 0; //Disable interrupts for this process
    
    NVMADR = address; //Load EEPROM Address
    NVMDAT = val; //Load EEPROM Data
    
    NVMCON1bits.WREN = 1; //Enable Write Access to EEPROM
    
    NVMCON2 = 0x55; //Data EEPROM Charge Pump
    NVMCON2 = 0xAA; ///
    NVMCON1bits.WR = 1; //Write to EEPROM
    
    while(NVMCON1bits.WR == 1) {/*Wait for EEPROM write to complete*/}
    
    NVMCON1bits.WREN = 0; //Disable Write Access to EEPROM
    INTCONbits.GIE = 1; //Enable interrupts after completion
}

char ReadEEPROM(char address)
{
    NVMADR = address; //Load Address
    NVMCON1bits.RD = 1; //Initiate Read
    return NVMDAT;
}

char I2C2TransmitAddress(char addr)
{
    SSP2CON2bits.SEN = 1; //Send start bit
    while(SSP2CON2bits.SEN == 1) {/*Wait for start condition to finish*/}
    
    SSP2BUF = addr; //Load address to TX register
    while(SSP2STATbits.RW == 1) {/*Wait for address to transmit*/}
    
    if(SSP2CON2bits.ACKSTAT == 1) //Check for ACK bit
    {
        SSP2CON2bits.PEN = 1; //No acknowledge received, generate stop condition
        while(SSP2CON2bits.PEN == 1) {/*Wait for stop condition to finish*/}
        return 0x00; //Return fail condition
    }
   
    return 0xFF; //Return success condition.
}

char I2C2TransmitData (char data)
{
    SSP2BUF = data; //Load data into buffer.
    while(SSP2STATbits.RW == 1) {/*Wait for data to transmit*/}
    
    if(SSP2CON2bits.ACKSTAT == 1) //Check for ACK bit
    {
        SSP2CON2bits.PEN = 1; //No acknowledge received, generate stop condition
        while(SSP2CON2bits.PEN == 1) {/*Wait for stop condition to finish*/}
        return 0x00; //Return fail condition
    }
    
    return 0xFF; //Return success condition
}

char I2C2RequestData (char function, char readAddr)
{
    switch (function)
    {
        //function = 1 : Request first data byte
        //function = 2 : Request nth data byte
        //function = 3 : Request last data byte
        
        case 1:
            SSP2CON2bits.SEN = 1;
            while (SSP2CON2bits.SEN == 1) {/*Wait for start to finish*/}
            
            SSP2BUF = readAddr; //Write read address to I2C bus
            while(SSP2STATbits.RW == 1) {/*Wait for address to transmit*/}
            
            if(SSP2CON2bits.ACKSTAT == 1) //Check for ACK bit
            {
                SSP2CON2bits.PEN = 1; //No acknowledge received, generate stop condition
                while(SSP2CON2bits.PEN == 1) {/*Wait for stop condition to finish*/}
                return 0x00; //Return fail condition
            }
            
            SSP2CON2bits.RCEN = 1; //Enable Master Receive mode
            while (SSP2CON2bits.RCEN == 1) {/*Wait for data to be received*/}
            
            I2C2Data = SSP2BUF;
            SSP2CON2bits.ACKDT = 0; //Set Acknowledge sequence to !ACK
            SSP2CON2bits.ACKEN = 1; //Generate !ACK
            while(SSP2CON2bits.ACKEN == 1) {/*Wait for ACK sequence to finish*/}
            
            return 0xFF; //Return success condition
            break;
        case 2:
            SSP2CON2bits.RCEN = 1; //Enable Master Receive mode
            while (SSP2CON2bits.RCEN == 1) {/*Wait for data to be received*/}
            
            I2C2Data = SSP2BUF;
            SSP2CON2bits.ACKDT = 0; //Set Acknowledge sequence to !ACK
            SSP2CON2bits.ACKEN = 1; //Generate !ACK
            while(SSP2CON2bits.ACKEN == 1) {/*Wait for ACK sequence to finish*/}
            
            return 0xFF; //Return success condition
            break;
        case 3:
            SSP2CON2bits.RCEN = 1; //Enable Master Receive mode
            while (SSP2CON2bits.RCEN == 1) {/*Wait for data to be received*/}
            
            I2C2Data = SSP2BUF;
            SSP2CON2bits.ACKDT = 1; //Set Acknowledge sequence to NACK
            SSP2CON2bits.ACKEN = 1; //Generate NACK
            while(SSP2CON2bits.ACKEN == 1) {/*Wait for NACK sequence to finish*/}
            SSP2CON2bits.PEN = 1;
            while(SSP2CON2bits.PEN == 1) {/*Wait for stop condition to finish*/}
            
            return 0xFF; //Return success condition
            break;
        default:
            return 0x00; //Process not recognized, return fail condition
            break;
    }
}

void I2C2Stop(void)
{
    SSP2CON2bits.PEN = 1; //Generate Stop Condition
    while(SSP2CON2bits.PEN == 1) {/*Wait for stop condition to finish*/}
}

char I2C2Scan (void)
{
    //Reserved Addresses : 0000 0000 to 0000 1111 and 1111 000 to 1111 1111
    //Scan : Start = 0001 000x : End 0111 111x : x = R/W Bit
    scanAddress = 0x10;

    while(1) //Increment through addresses until a match is found, then return the match
    {
        if(I2C2TransmitAddress(scanAddress) == 0xFF) {I2C2Stop(); return scanAddress;} else {scanAddress += 2;}
        if(scanAddress >= 0b10000000) {return 0x00;} //No match found, return fail condition
    }
}

char SaveSensor(void)
{
    //---Find the next available EEPROM location for the new sensor-------------
    sensorEEADR = 0x10; //Start at address 0x10, final address is 0x1E
    while (ReadEEPROM(sensorEEADR) != 0x00) //ReadEEPROM to find the next available EEPROM location
    {
        sensorEEADR += 2; //Goto next possible address
        if(sensorEEADR >= 0x20) {return 0x00;} //NO LOCATION AVAILABLE, Return Fail Condition
    }
    
    //---Set register to read the board identification registers----------------
    if(I2C2TransmitAddress(scanAddress) == 0x00) {return 0x00;} //Write to sensor
    if(I2C2TransmitData(0x3C) == 0x00) {return 0x00;} //Select Identifier Register
    I2C2Stop();
    
    //---Read and store the board identification registers----------------------
    if(I2C2RequestData(1, scanAddress | 0x01) == 0x00) {return 0x00;} //Read first byte of identifier code
    ID[0] = I2C2Data;
    if(I2C2RequestData(2, 0x00) == 0x00) {return 0x00;} //Read second byte of identifier code
    ID[1] = I2C2Data;
    if(I2C2RequestData(2, 0x00) == 0x00) {return 0x00;} //Read third byte of identifier code
    ID[2] = I2C2Data;
    if(I2C2RequestData(3, 0x00) == 0x00) {return 0x00;} //Read last byte of identifier code
    ID[3] = I2C2Data;
    
    //---Instruct the sensor to save the EEPROM location as their new address---
    if(I2C2TransmitAddress(scanAddress) == 0x00) {return 0x00;}   //Select Slave Address Register
    if(I2C2TransmitData(0x22) == 0x00) {return 0x00;}             ///
    if(I2C2TransmitData(sensorEEADR >> 1) == 0x00) {return 0x00;} //Write New Slave Address
    I2C2Stop(); //Address Reassignment Success!
    
    //---Save processes unique to sensors---------------------------------------
    if(ID[0] == 'L' && ID[1] == 'U' && ID[2] == 'N' && ID[3] == 'A') //TF LUNA DETECTED
    {
        //---Save Identifier as 'L' and number of data bytes as 2---------------
        WriteToEEPROM(sensorEEADR, 'L'); //Save sensor type as TF-LUNA.
        WriteToEEPROM(sensorEEADR | 0x01, 1); //Save number of data bytes as 1 
        
        //---Luna requires a save settings instruction--------------------------
        if(I2C2TransmitAddress(sensorEEADR) == 0x00) {return 0x00;}   //Select Save Settings Register
        if(I2C2TransmitData(0x20) == 0x00) {return 0x00;}             ///
        if(I2C2TransmitData(0x01) == 0x00) {return 0x00;}             //Instruct TF-Luna to save settings
        I2C2Stop(); //Save process success!
    }
    else if (ID[0] == 'I' && ID[1] == 'N' && ID[2] == 'F') //INFARED BOARD DETECTED
    {
        //---Save number of sensors as identifier-------------------------------
        WriteToEEPROM(sensorEEADR, 'I'); //Save sensor type as infrared
        WriteToEEPROM(sensorEEADR | 0x01, ID[3]); //Save number of sensors
    }
    else {return 0x00;} //Unknown sensor, return fail condition
    
    return 0xFF; //Return success
}

void HandleTMR2(void) 
{
    //Do not start bit shift sequence for 150 ticks
    if(T2Delay != 150) {T2Delay++;} else 
    {
        T2Count++;
        if(T2Count == 6) //Shift bit left once every 6 ticks
        {
            T2Count = 0;
            LATA = LATA << 1;
            if(LATA == 0) {LATA = 1;} //Carry bit from MSB to LSB
        }
    }
}

void HandleTMR4(void) //TMR4 is used only for debugging
{
    pFlag &= 0b01111111;
}

void HandleIOC(void)
{
    if(PORTC & 0b00100000) {pFlag |= 0b00010000; T2Delay = 0;} //Search for new sensor
    if(PORTC & 0b00010000) {pFlag |= 0b00001000; T2Delay = 0;} //Erase stored data
}

void SearchProcess(void)
{    
    for(;;)
    {
        if(I2C2Scan() == 0x00) {LATA = 0xEE; break;}
        if(SaveSensor() == 0x00) {LATA = 0xFF; break;} else {LATA = sensorEEADR;}
        break;
    }
}

void EraseSensors(void)
{
    char i;
    LATA = 0x0F;
    for (i = 0x10; i != 0x22 ; i++)
    {
        WriteToEEPROM(i, 0x00); //Erase EEPROM data locations 0x10 to 0x20
    }
}

void RequestData (void)
{
    char lastSensor = 0;
    
    char i, i2;
    char byteCount = 0;
    
    char sensorID;
    char distanceBytes;

    //Determine the last sensor connected to the I2C bus
    for(i = 0x10; i <= 0x1E; i+=2)
    {
        if(ReadEEPROM(i) != 0) {lastSensor = i;}
    }
    
    for(i = 0x10; i <= lastSensor; i+=2)
    {
        sensorID = ReadEEPROM(i); 
        distanceBytes = ReadEEPROM(i | 0x01); 
        
        if(I2C2TransmitAddress(i) == 0x00) {break;} //Point to first distance data register
        if(I2C2TransmitData(0x00) == 0x00) {break;} // /
        I2C2Stop();
        
        if(I2C2RequestData(1, i | 0x01) == 0x00) {break;} //Request distance data
        distData[0] = I2C2Data;                           // /////
        if(I2C2RequestData(2, 0x00) == 0x00) {break;}     // ////
        distData[1] = I2C2Data;                           // ///
        if(I2C2RequestData(3, 0x00) == 0x00) {break;}     // //
        distData[2] = I2C2Data;                           // /       

        if(sensorID == 'L') {if(distData[1] != 0) {distData[0] = 0xFF;}}

        for(i2 = 0; i2 <= (distanceBytes - 1); i2++)
        {
            byteCount++;
            I2C1Registers[byteCount] = distData[i2];
        }
    }
    I2C1Registers[0] = byteCount;
}

void StartStatus()
{   
    //The status of the board is determined on startup...
    if(PORTC & 0b01000000) //RC6 is program header
    {
        //The board is in program mode
        //I2C1:Disabled : I2C2=Enabled : TMR2=Enabled : IOC=Enabled
        SSP2Config();
        TMR2Config();
        IOCConfig();
    }
    else
    {
        //The board is in normal operation
        //I2C1=Enabled : I2C2=Enabled : TMR4=Disabled : IOC=Disabled
        SSP1Config();
        SSP2Config();
    }
    
    char ptr = 0;
    for(ptr = 0; ptr != 29; ptr++) //Load all I2C1 registers with their default values
    {
        I2C1Registers[ptr] = ptr;
    }
    I2C1Registers[25] = 'M'; //Load Identifier Code to I2C1 Registers
    I2C1Registers[26] = 'O'; /////
    I2C1Registers[27] = 'D'; ////
    I2C1Registers[28] = 'X'; ///
}
//-----Functions End----------------------------------------------------------//
//-----Interrupts Start-------------------------------------------------------//

void TMR2_ISR(void)
{
    pFlag |= 0b01000000; //pFlag bit 6 is TMR2
    PIR4bits.TMR2IF = 0;
}

void TMR4_ISR(void)
{
    pFlag |= 0b10000000; //pFlag bit 7 is TMR4
    PIR4bits.TMR4IF = 0;
}

void IOC_ISR(void)
{
    pFlag |= 0b00100000; //pFlag bit 5 is IOC
    IOCCF &= 0x00; //Clear IOC PortC Individual Flags
    PIR0bits.IOCIF = 0;
}

void I2C1_ISR(void)
{
    //I2C is processed in interrupt routine to ensure good communication with master
    
    //Possible functions slave can complete...
    //1: Write address, set I2C1 register index
    //2: Read address, increment through I2C registers until NACK is received.
    
    //Possible reasons for being in interrupt
    //Master write to slave
    //1: Address match detected
    //2: Data byte received from master
    //3: Stop bit is received
    //Master read from slave
    //1: Address match detected
    //2: /ACK is received from master after clocking out data
    //3: NACK is received from master, clock not held in this case
    //4: Stop bit is received
    
    //Other required steps
    //Must read SSPBUF to clear BF flag
    //Must set SSPCON1 CKP to release control of clock
    
    PIR3bits.SSP1IF = 0;
    SSP1CON1bits.SSPOV = 0;
    
    //Step 1: Check if SSP1 is currently in a conversation
    switch (I2C1Status)
    {
        case 0: //Not currently in a conversation
            //Check RW to know the expected conversation
            
            I2C1RxData = SSP1BUF; //Read buffer to clear flag
            
            if(SSP1STATbits.R_nW == 0) {I2C1Status = 1;} else {I2C1Status = 2;}
            
            //If the determined conversation is read, load the first byte immediately
            if(I2C1Status == 2) 
            {
                SSP1BUF = I2C1Registers[I2C1RegIndex]; 
                I2C1RegIndex++;
            } 
            
            SSP1CON1bits.CKP = 1; //Release clock
            break;
            
        case 1: //Currently in a master write to slave conversation
            //If stop was generated end process
            if(SSP1STATbits.P) {I2C1Status = 0;} else
            {
                I2C1RegIndex = SSP1BUF; //Read buffer to assign register
                SSP1CON1bits.CKP = 1; //Release clock
            }
            break;
            
        case 2: //Currently in a master read from slave conversation
            if(SSP1STATbits.P == 1) {I2C1Status = 0; pFlag |= 0b00000100;} else
            {
                SSP1BUF = I2C1Registers[I2C1RegIndex]; //Load register to SSP buffer
                I2C1RegIndex++; //Increment to next register
                SSP1CON1bits.CKP = 1; //Release clock
            }
            break;
            
        default: //Unknown condition
            break;
    }
}

void __interrupt() ISR(void)
{
    if (PIR4bits.TMR4IF == 1) {TMR4_ISR();}
    if (PIR4bits.TMR2IF == 1) {TMR2_ISR();} 
    
    if (PIR0bits.IOCIF == 1)  {IOC_ISR();}
    if (PIR3bits.SSP1IF == 1) {I2C1_ISR();}
}

//-----Interrupts End---------------------------------------------------------//
//=====Main Code Start========================================================//
void main(void)
{
    PortsConfig(); //Port Configuration is Always Needed
    NVMConfig(); //Non Volatile Memory Config is Always Needed
    
    StartStatus(); //Determine status on startup
    
    IntConfig(); //Configure and enable interrupts
    
    while(1) //Main Program Loop
    {
        //Personal flag routines (Refer to pFlag register functions)
        //if(pFlag bit x high) then {DoProcess(); Clear pFlag bit;}   
        if(pFlag & 0b10000000) {HandleTMR4();    pFlag &= 0b01111111;} //Handles from ISRs
        if(pFlag & 0b01000000) {HandleTMR2();    pFlag &= 0b10111111;} ////
        if(pFlag & 0b00100000) {HandleIOC();     pFlag &= 0b11011111;} ///
        
        if(pFlag & 0b00010000) {SearchProcess(); pFlag &= 0b11101111;} //Specific function pFlag handles
        if(pFlag & 0b00001000) {EraseSensors();  pFlag &= 0b11110111;} ////
        if(pFlag & 0b00000100) {RequestData();   pFlag &= 0b11111011;} ///
    }
}
//=====Main Code End==========================================================//