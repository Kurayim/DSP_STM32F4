/*
 * matlab_interface.c
 *
 *  Created on: Aug 12, 2025
 *      Author: 2023
 */
#include "matlab_interface.h"

#define LENGTH_COMMAND	15
#define LENGTH_PACKET	16




bool CreateBuffeSample(uint8_t *BufferSample, uint32_t indexSample, uint32_t sample)
{

/*  | 0  |  1 |  2 |  3 |  4 |  5 ||    6    |    7    |    8    |    9    ||    10    |    11    |    12    |    13    ||      14     |      15     |
 *	|____|____|____|____|____|____||_________|_________|_________|_________||__________|__________|__________|__________||_____________|_____________|
 *	|	 |    |    |    |    |    ||         | 		   |         |		   ||		   |	      |		     |		    ||			   |			 |
 *	| AA | 55 | 5A | A5 | BB | TP || Data[3] | Data[2] | Data[1] | Data[0] || Count[3] | Count[2] | Count[1] | Count[0] || CheckSum[1] | CheckSum[0] |
 *	|____|____|____|____|____|____||_________|_________|_________|_________||__________|__________|__________|__________||_____________|_____________|
 *   ////////////Header\\\\\\\\\\\  ////////////////Sample\\\\\\\\\\\\\\\\\  ////////////////Index Sample\\\\\\\\\\\\\\\  ////////Check Sum\\\\\\\\\\
 */

	uint16_t checkSum = 0;


	//[0] : header
	*BufferSample = 0xAA;
	checkSum += *BufferSample;
	BufferSample++;

	//[1] : header
	*BufferSample = 0x55;
	checkSum += *BufferSample;
	BufferSample++;

	//[2] : header
	*BufferSample = 0x5A;
	checkSum += *BufferSample;
	BufferSample++;

	//[3] : header
	*BufferSample = 0xA5;
	checkSum += *BufferSample;
	BufferSample++;

	//[4] : header
	*BufferSample = 0xBB;
	checkSum += *BufferSample;
	BufferSample++;

/************************************************/

	//[5] : TYPE PACKET
	*BufferSample = 0x91;
	checkSum += *BufferSample;
	BufferSample++;

/************************************************/

	//[6] : Data[3]
	*BufferSample = (sample & 0xff000000) >> 24;
	checkSum += *BufferSample;
	BufferSample++;

	//[7] : Data[2]
	*BufferSample = (sample & 0x00ff0000) >> 16;
	checkSum += *BufferSample;
	BufferSample++;

	//[8] : Data[1]
	*BufferSample = (sample & 0x0000ff00) >> 8;
	checkSum += *BufferSample;
	BufferSample++;

	//[9] : Data[0]
	*BufferSample = (sample & 0x000000ff);
	checkSum += *BufferSample;
	BufferSample++;

/************************************************/

	//[10] : Count[3]
	*BufferSample = (indexSample & 0xff000000) >> 24;
	checkSum += *BufferSample;
	BufferSample++;

	//[11] : Count[2]
	*BufferSample = (indexSample & 0x00ff0000) >> 16;
	checkSum += *BufferSample;
	BufferSample++;

	//[12] : Count[1]
	*BufferSample = (indexSample & 0x0000ff00) >> 8;
	checkSum += *BufferSample;
	BufferSample++;

	//[13] : Count[0]
	*BufferSample = (indexSample & 0x000000ff);
	checkSum += *BufferSample;
	BufferSample++;

/************************************************/

	//[15] : CheckSum[0]
	*BufferSample = (checkSum & 0xff00) >> 8;
	BufferSample++;

	//[14] : CheckSum[1]
	*BufferSample = (checkSum & 0x00ff);
	//BufferSample++;



	return 1;
}



bool CreateCommand(uint8_t * command_buffer , uint8_t command_type, uint16_t scalePow, uint16_t SmaplRate, uint16_t numSamFram)
{

/*  | 0  |  1 |  2 |  3 |  4  |  5 ||      6       |     7     |   8   |   9   ||      10      ||      11      ||      12      ||      13      ||      14      ||      15     |
 *	|____|____|____|____|_____|____||______________|___________|_______|_______||______________||______________||______________||______________||______________||_____________|
 *	|	 |    |    |    |scale|    ||         	   | 		   | SAMPL | SAMPL ||	   	       ||	           ||		       ||		       ||			   ||			  |
 *	| AA | 55 | 5A | A5 | Pow | TP || command_type | numSignal |RATE[1]|RATE[0]|| numSample[3] || numSample[2] || numSample[1] || numSample[0] || CheckSum[1]  || CheckSum[0] |
 *	|____|____|____|____|_____|____||______________|___________|_______|_______||______________||______________||______________||______________||______________||_____________|
 *   ////////////Header\\\\\\\\\\\\  ///////\\\\\\\ //////\\\\\ /// \\\ /// \\\  ////////////////////////Number Samples\\\\\\\\\\\\\\\\\\\\\\\  //////////Check Sum\\\\\\\\\\
 */


/*  | 0  |  1 |  2 |  3 |  4  |  5 ||      6       |   7   |   8   ||      9       ||      10     ||      11       ||       12      ||      13      ||      14     |
 *	|____|____|____|____|_____|____||______________|_______|_______||______________||_____________||_______________||_______________||______________||_____________|
 *	|	 |    |    |    |     |    ||         	   | SAMPL | SAMPL ||			   ||			  ||			   ||			    ||		 	    ||		       |
 *	| AA | 55 | 5A | A5 | nul | TP || command_type |RATE[1]|RATE[0]|| scalePow[1]  || scalePow[0] || numSamFram[1] || numSamFram[0] || CheckSum[1]  || CheckSum[0] |
 *	|____|____|____|____|_____|____||______________|_______|_______||______________||_____________||_______________||_______________||______________||_____________|
 *    //////Header\\\\\\             ///////\\\\\\\ /// \\\ /// \\\   //////////scalePow\\\\\\\\\\  /////////NumSampleFram\\\\\\\\\   //////////CheckSum\\\\\\\\\\
 */

	uint16_t checkSum = 0;



	//[0] : header
	*command_buffer = 0xAA;
	checkSum += *command_buffer;
	command_buffer++;

	//[1] : header
	*command_buffer = 0x55;
	checkSum += *command_buffer;
	command_buffer++;

	//[2] : header
	*command_buffer = 0x5A;
	checkSum += *command_buffer;
	command_buffer++;

	//[3] : header
	*command_buffer = 0xA5;
	checkSum += *command_buffer;
	command_buffer++;

/************************************************/

	//[4] : scale Power
	*command_buffer = scalePow;
	checkSum += *command_buffer;
	command_buffer++;

/************************************************/

	//[5] : TYPE PACKET
	*command_buffer = 0x15;
	checkSum += *command_buffer;
	command_buffer++;

/************************************************/

	//[6] : command_type
	*command_buffer = command_type;
	checkSum += *command_buffer;
	command_buffer++;

/************************************************/

	//[7] : SampleRate[1]
	*command_buffer = (SmaplRate & 0xff00) >> 8;
	checkSum += *command_buffer;
	command_buffer++;


	//[8] : SampleRate[0]
	*command_buffer = (SmaplRate & 0x00ff);
	checkSum += *command_buffer;
	command_buffer++;

/************************************************/


	//[9] : scalePow[1]
	*command_buffer = (scalePow & 0xff00) >> 8;
	checkSum += *command_buffer;
	command_buffer++;


	//[10] : scalePow[0]
	*command_buffer = (scalePow & 0x00ff);
	checkSum += *command_buffer;
	command_buffer++;

/************************************************/


	//[11] : NumSamFram[1]
	*command_buffer = (numSamFram & 0xff00) >> 8;
	checkSum += *command_buffer;
	command_buffer++;


	//[12] : NumSamFram[0]
	*command_buffer = (numSamFram & 0x00ff);
	checkSum += *command_buffer;
	command_buffer++;

/************************************************/

	//[13] : CheckSum[0]
	*command_buffer = (checkSum & 0xff00) >> 8;
	command_buffer++;

	//[14] : CheckSum[1]
	*command_buffer = (checkSum & 0x00ff);
	//BufferSample++;

	return 1;
}


bool AnalizeCommand(uint8_t analizePorpuse)
{

	uint16_t cal_checkSum = 0;
	uint16_t com_chechSum = 0;


	if(BufferReceive[0] != 0xAA  ||  BufferReceive[1] != 0x55  ||  BufferReceive[2] != 0x5A  ||
	   BufferReceive[3] != 0xA5  ||  BufferReceive[4] != 0xBB  ||  BufferReceive[5] != 0xCC)
		return 0;

	for(uint8_t index = 0 ; index < LENGTH_PACKET-2 ; index++)
	{
		cal_checkSum += BufferReceive[index];
	}

	com_chechSum = ((((uint16_t)BufferReceive[14]) << 8) | ((uint16_t)BufferReceive[15]));

	if(cal_checkSum != com_chechSum)
		return 0;


	if(BufferReceive[6] == ANS_MATLAB_READY_RX)
		return 1;
	else
		return 0;

}


//bool SendSignal(float32_t *Samples, int scale, uint8_t numSignal, uint32_t numSample, uint16_t sampleRate)
//{
//	#define Limit_Wait		1000
//	#define LENGTH_COMMAND	16
//	#define ScalseSample	5
//
//	int32_t	 sample_u32			= 0;
//	uint32_t sampleIndex 	= 0;
//	uint8_t  bufferTx[17] 	= {'\0'};
//	uint16_t  timeWait 		= 0;
//
//
////HAL_Delay(5000);
//
//	memset(bufferTx, '\0', 17);
////	CreateCommand(bufferTx , COM_START_SEND_SIGNAL, ScalseSample, numSignal, numSample, sampleRate);
//	FalgReadyPacketRx = false;
//	CDC_Transmit_FS((uint8_t*)bufferTx, LENGTH_COMMAND);
//	timeWait = 0;
//	//while(1);
//	while(timeWait <= Limit_Wait)
//	{
//		HAL_Delay(1);
//		timeWait++;
//		if(FalgReadyPacketRx)
//		{
//			FalgReadyPacketRx = false;
//			if(AnalizeCommand(ANS_MATLAB_READY_RX))
//			{
//				break;
//			}
//			else
//				return 0;
//		}
//		if(timeWait == Limit_Wait)
//			return 0;
//	}
//
//
//	for(sampleIndex = 0 ; sampleIndex < numSample ; sampleIndex++ , Samples++)
//	{
//
//		memset(bufferTx, '\0', LENGTH_COMMAND+1);
//		sample_u32 = (int32_t)((*Samples) * scale);
//		CreateBuffeSample(bufferTx, sampleIndex, sample_u32);
//		CDC_Transmit_FS((uint8_t*)bufferTx, LENGTH_COMMAND);
//		HAL_Delay(2);
//	}
//
//
//	memset(bufferTx, '\0', LENGTH_COMMAND+1);
////	CreateCommand(bufferTx , COM_END_SEND_SIGNAL, ScalseSample, numSignal, numSample, sampleRate);
//	CDC_Transmit_FS((uint8_t*)bufferTx, LENGTH_COMMAND);
//	return 1;
//}


MAT_StatusTypeDef MAT_Connect(uint16_t sampleRate, uint16_t scale, uint16_t numSamFram){

	#define Limit_Wait		1000
//	#define ScalseSample	5


	uint8_t  bufferTx[LENGTH_COMMAND] 	= {'\0'};
	uint16_t timeWait 		= 0;


	//HAL_Delay(5000);

	memset(bufferTx, '\0', LENGTH_COMMAND);
	CreateCommand(bufferTx, COM_START_SEND_SIGNAL, scale, sampleRate, 2*numSamFram);
	FalgReadyPacketRx = false;
	CDC_Transmit_FS((uint8_t*)bufferTx, LENGTH_COMMAND);
	timeWait = 0;
//	while(1);
	while(timeWait <= Limit_Wait)
	{
		HAL_Delay(1);
		timeWait++;
		if(FalgReadyPacketRx)
		{
			FalgReadyPacketRx = false;
			if(AnalizeCommand(ANS_MATLAB_READY_RX))
			{
				break;
			}
			else
				return MAT_ERROR_CONNECT;
		}
		if(timeWait == Limit_Wait)
			return MAT_ERROR_CONNECT;
	}

	return MAT_OK;
}

MAT_StatusTypeDef MAT_SendSamples(float32_t *Samples, uint32_t numSample,  float32_t scale){

	#define LENGTH_PACKET	16

	int16_t  valSample   = 0;
	uint32_t CheckSum    = 0;
	uint32_t sampleIndex = 0;
	uint16_t lengthPack  = (numSample * 2);
	uint8_t  bufferTx[(numSample * 2) + 5];

	for(sampleIndex = 0 ; sampleIndex < lengthPack ; sampleIndex += 2){
		valSample = (int16_t)((*Samples) * scale);
		bufferTx[sampleIndex]   = (((int16_t)valSample) & 0xff00) >> 8;
		bufferTx[sampleIndex+1] =  ((int16_t)valSample) & 0x00ff;
		CheckSum = CheckSum + bufferTx[sampleIndex] + bufferTx[sampleIndex+1];
		Samples++;
	}

	bufferTx[lengthPack  ] = CheckSum >> 24;
	bufferTx[lengthPack+1] = CheckSum >> 16;
	bufferTx[lengthPack+2] = CheckSum >> 8;
	bufferTx[lengthPack+3] = CheckSum;
	bufferTx[lengthPack+4] = 0;


	lengthPack += 5;
	HAL_Delay(50);
	CDC_Transmit_FS((uint8_t*)bufferTx, lengthPack);
	HAL_Delay(500);

	return MAT_OK;
}

MAT_StatusTypeDef MAT_EndSignal(void){

	uint8_t  bufferTx[17] 	= {'\0'};

	memset(bufferTx, '\0', LENGTH_COMMAND+1);
	CreateCommand(bufferTx, COM_END_SEND_SIGNAL, 0xff, 0xff, 0xff);
	CDC_Transmit_FS((uint8_t*)bufferTx, LENGTH_COMMAND);

	return MAT_OK;
}





