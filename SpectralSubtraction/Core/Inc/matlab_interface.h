/*
 * matlab_interface.h
 *
 *  Created on: Aug 12, 2025
 *      Author: 2023
 */

#ifndef INC_MATLAB_INTERFACE_H_
#define INC_MATLAB_INTERFACE_H_


#include "main.h"
#include "stdbool.h"
#include "arm_math.h"
#include "usbd_cdc_if.h"



#define COM_START_SEND_SIGNAL	0x10
#define COM_END_SEND_SIGNAL		0x11
#define ANS_MATLAB_READY_RX		0x81


extern uint8_t BufferReceive[64];
extern bool FalgReadyPacketRx;

typedef enum
{
    MAT_OK 	  		 = 0x00U,
    MAT_ERROR 		 = 0x01U,
	MAT_ERROR_CONNECT		 = 0x02U

} MAT_StatusTypeDef;



bool CreateBuffeSample(uint8_t *BufferSample, uint32_t indexSample, uint32_t sample);
bool CreateCommand(uint8_t * command_buffer , uint8_t command_type, uint16_t scalePow, uint16_t SmaplRate, uint16_t numSamFram);
bool AnalizeCommand(uint8_t analizePorpuse);
//bool SendSignal(float32_t *Samples, int scale, uint8_t numSignal, uint32_t numSample, uint16_t sampleRate);

MAT_StatusTypeDef MAT_Connect(uint16_t sampleRate, uint16_t scale, uint16_t numSamFram);
MAT_StatusTypeDef MAT_SendSamples(float32_t *Samples, uint32_t numSample,  float32_t scale);
MAT_StatusTypeDef MAT_EndSignal(void);












#endif /* INC_MATLAB_INTERFACE_H_ */
