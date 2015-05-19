#pragma once

typedef enum {
	NORMAL,
	SERVO
} MotorMode;

void Motori_Init(void);
void Motori_Halt(void);
void Motori_Reset(void);
void Motori_SetSpeed(int speed);
int Motori_GetSpeed(void);
void Motori_Enable(int enable);
int Motori_GetEnabled(void);
void Motori_SetPower(int power);
int Motori_GetPower(void);
int Motori_GetElectricalAngle(void);
int Motori_GetEAError(int quadrature_count);
int Motori_GetEAError2(int quadrature_count);
void Motori_SetElectricalAngleDegrees(int angle);
void Motori_SetElectricalAngle(int angle);
void Motori_SetPosition(int position);
void Motori_SetMode(const MotorMode mode);