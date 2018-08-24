/*
    File Name:        :  mcp9808.h

    Device            :  PIC32MM0256GPM048
    Compiler          :  XC32 2.05
    MPLAB             :  MPLAB X 4.15
    Created by        :  http://strefapic.blogspot.com
*/

#ifndef _MCP9808_H
#define _MCP9808_H

#define MCP9808_ADDRESS                 0x30

// MCP9808 Register pointers
#define MCP9808_REG_CONFIG 				0x01
#define MCP9808_REG_ALERT_UPPER_TEMP	0x02
#define MCP9808_REG_ALERT_LOWER_TEMP	0x03
#define MCP9808_REG_CRITICAL_TEMP		0x04
#define MCP9808_REG_AMBIENT_TEMP		0x05
#define MCP9808_REG_MANUF_ID			0x06
#define MCP9808_REG_DEVICE_ID_REV		0x07
#define MCP9808_REG_RESOLUTION			0x08

// Config masks
#define MCP9808_CONFIG_HYST_6			0x0600
#define MCP9808_CONFIG_HYST_3			0x0400
#define MCP9808_CONFIG_HYST_1_5			0x0200
#define MCP9808_CONFIG_SHUTDOWN			0x0100
#define MCP9808_CONFIG_CRITLOCKED		0x0080
#define MCP9808_CONFIG_WINLOCKED		0x0040
#define MCP9808_CONFIG_INTCLR			0x0020
#define MCP9808_CONFIG_ALERTSTAT		0x0010
#define MCP9808_CONFIG_ALERTCTRL		0x0008
#define MCP9808_CONFIG_ALERTSEL			0x0002
#define MCP9808_CONFIG_ALERTPOL			0x0002
#define MCP9808_CONFIG_ALERTMODE		0x0001

// Power modes
#define MCP9808_CONTINUOUS				0x0100
#define MCP9808_LOW_POWER				0x0000

// Resolution/Speed modes
#define MCP9808_FASTEST					0x00 // +0.5°C resolution 		- 30ms conversion time
#define MCP9808_FAST 					0x01 // +0.25°C resolution		- 65ms conversion time
#define MCP9808_SLOW					0x02 // +0.125°C resolution 	- 130ms conversion time
#define MCP9808_SLOWEST					0x03 // +0.0625°C resolution	- 250ms conversion time

	
    void read_Temp(void);
	
	/* Flag bits */
	bool _criticalTemp;
	bool _upperTemp;
	bool _lowerTemp;
	bool _negativeTemp;

	extern uint8_t UpperByte ;
    extern uint8_t LowerByte ;
    extern float Temperature ;
    extern char Temperature_bufor[5] ;
    
#endif
