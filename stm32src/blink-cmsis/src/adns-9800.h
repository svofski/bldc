#pragma once

#include <inttypes.h>

#define ADNS_Product_ID                           0x00
#define ADNS_Revision_ID                          0x01
#define ADNS_Motion                               0x02
#define ADNS_Delta_X_L                            0x03
#define ADNS_Delta_X_H                            0x04
#define ADNS_Delta_Y_L                            0x05
#define ADNS_Delta_Y_H                            0x06
#define ADNS_SQUAL                                0x07
#define ADNS_Pixel_Sum                            0x08
#define ADNS_Maximum_Pixel                        0x09
#define ADNS_Minimum_Pixel                        0x0a
#define ADNS_Shutter_Lower                        0x0b
#define ADNS_Shutter_Upper                        0x0c
#define ADNS_Frame_Period_Lower                   0x0d
#define ADNS_Frame_Period_Upper                   0x0e
#define ADNS_Configuration_I                      0x0f
#define ADNS_Configuration_II                     0x10
#define ADNS_Frame_Capture                        0x12
#define ADNS_SROM_Enable                          0x13
#define ADNS_Run_Downshift                        0x14
#define ADNS_Rest1_Rate                           0x15
#define ADNS_Rest1_Downshift                      0x16
#define ADNS_Rest2_Rate                           0x17
#define ADNS_Rest2_Downshift                      0x18
#define ADNS_Rest3_Rate                           0x19
#define ADNS_Frame_Period_Max_Bound_Lower         0x1a
#define ADNS_Frame_Period_Max_Bound_Upper         0x1b
#define ADNS_Frame_Period_Min_Bound_Lower         0x1c
#define ADNS_Frame_Period_Min_Bound_Upper         0x1d
#define ADNS_Shutter_Max_Bound_Lower              0x1e
#define ADNS_Shutter_Max_Bound_Upper              0x1f
#define ADNS_LASER_CTRL0                          0x20
#define ADNS_Observation                          0x24
#define ADNS_Data_Out_Lower                       0x25
#define ADNS_Data_Out_Upper                       0x26
#define ADNS_SROM_ID                              0x2a
#define ADNS_Lift_Detection_Thr                   0x2e
#define ADNS_Configuration_V                      0x2f
#define ADNS_Configuration_IV                     0x39
#define ADNS_Power_Up_Reset                       0x3a
#define ADNS_Shutdown                             0x3b
#define ADNS_Inverse_Product_ID                   0x3f
#define ADNS_Snap_Angle							  0x42
#define ADNS_Motion_Burst                         0x50
#define ADNS_SROM_Load_Burst                      0x62
#define ADNS_Pixel_Burst                          0x64

#define ADNS_FRAME_PIXEL_COUNT					  900

typedef union {
	struct {
		uint8_t 	Motion;
		uint8_t		Observation;
		int16_t		Dx;
		int16_t		Dy;
		uint8_t		SQUAL;
		uint8_t		Pixel_Sum;
		uint8_t 	Maximum_Pixel;
		uint8_t		Minimum_Pixel;
		uint8_t		Shutter_Upper;
		uint8_t 	Shutter_Lower;
		uint8_t 	FrameRate_Upper;
		uint8_t		FrameRate_Lower;
	} Fields;
	uint8_t Bytes[14];
}  __attribute__((packed)) ADNSMotionStruct;


void ADNS9800_Config(void);
uint8_t ADNSReadReg(uint8_t addr);
void ADNSWriteReg(uint8_t addr, uint8_t value);
uint8_t ADNSInitialize(void);
int ADNSCheckFirmware(void);
int ADNSConfigureResolution();
int ADNSPoll(void);
int ADNSDMAPoll(void);
int ADNSCheckMotion(void);
int ADNSGetX(void);
int ADNSGetY(void);
int ADNSGetFramerate(void);
void ADNSLaserOn(void);
int ADNSFrameCapture(void);
void ADNSFrameDump(void);
