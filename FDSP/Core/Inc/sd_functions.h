/******************************************************************************
 *  File        : sd_functions.h
 *  Author      : ControllersTech
 *  Website     : https://controllerstech.com
 *  Date        : June 26, 2025
 *
 *  Description :
 *    This file is part of a custom STM32/Embedded tutorial series.
 *    For documentation, updates, and more examples, visit the website above.
 *
 *  Note :
 *    This code is written and maintained by ControllersTech.
 *    You are free to use and modify it for learning and development.
 ******************************************************************************/

#ifndef __SD_FUNCTIONS_H__
#define __SD_FUNCTIONS_H__

#include "fatfs.h"
#include "sd_diskio_spi.h"
#include "sd_spi.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "ff.h"
#include "ffconf.h"
#include "arm_math.h"


#define WAVE_HED_SIZ		60
#define WAVE_DAT_RED_SIZ	2048

#define NORMALISE			32767.5f


extern char sd_path[];


typedef struct CsvRecord {
    char field1[32];
    char field2[32];
    int value;
} CsvRecord;

typedef enum
{
    WAV_OK 	  		 = 0x00U,
    WAV_ERROR 		 = 0x01U,
	WAV_ERROR_MOUNT  = 0x02U,
	WAV_ERROR_F_OPEN = 0x03U,
	WAV_ERROR_F_STAT = 0x04U,
	WAV_ERROR_F_READ = 0x05U,
	WAV_ERROR_HEADER = 0x06U,
	WAV_ERROR_F_WRIT = 0x07U

} WAV_StatusTypeDef;

//typedef union {
//    uint8_t raw[WAVE_HED_SIZ+1]; // Access header as raw 44 bytes
//
//    struct {
//        char     ChunkID[4];       // Bytes 0-3   : Contains "RIFF"
//        uint32_t ChunkSize;        // Bytes 4-7   : File size - 8 bytes
//        char     Format[4];        // Bytes 8-11  : Contains "WAVE"
//
//        char     Subchunk1ID[4];   // Bytes 12-15 : Contains "fmt "
//        uint32_t Subchunk1Size;    // Bytes 16-19 : PCM = 16
//        uint16_t AudioFormat;      // Bytes 20-21 : PCM = 1
//        uint16_t NumChannels;      // Bytes 22-23 : Mono = 1, Stereo = 2
//        uint32_t SampleRate;       // Bytes 24-27 : Samples per second
//        uint32_t ByteRate;         // Bytes 28-31 : SampleRate * NumChannels * BitsPerSample/8
//        uint16_t BlockAlign;       // Bytes 32-33 : NumChannels * BitsPerSample/8
//        uint16_t BitsPerSample;    // Bytes 34-35 : Bits per sample
//
//        char     Subchunk2ID[4];   // Bytes 36-39 : Contains "data"
//        uint32_t Subchunk2Size;    // Bytes 40-43 : NumSamples * NumChannels * BitsPerSample/8
//    };
//} WAVHeader;



typedef union {
    uint8_t raw[WAVE_HED_SIZ+1]; // Access header as raw 44 bytes

    struct {
        /* 0..11 */
        uint8_t  ChunkID[4];       // 0-3   : "RIFF"
        uint32_t ChunkSize;        // 4-7   : RIFF size = file_size - 8 (will compute)
        uint8_t     Format[4];        // 8-11  : "WAVE"
        /* fmt chunk  (12..35) */
        uint8_t  Subchunk1ID[4];   // 12-15 : "fmt "
        uint32_t Subchunk1Size;    // 16-19 : usually 16 for PCM
        uint16_t AudioFormat;      // 20-21 : PCM = 1
        uint16_t NumChannels;      // 22-23 : mono=1, stereo=2
        uint32_t SampleRate;       // 24-27 : e.g., 22050
        uint32_t ByteRate;         // 28-31 : SampleRate * NumChannels * BitsPerSample/8
        uint16_t BlockAlign;       // 32-33 : NumChannels * BitsPerSample/8
        uint16_t BitsPerSample;    // 34-35 : bits per sample (e.g., 16)
        /* extra chunk (IDIT) (36..51) */
        uint8_t  ExtraChunkID[4];  // 36-39 : e.g., "IDIT" (you may set other ID)
        uint32_t ExtraChunkSize;   // 40-43 : size of extra payload (in your file = 8)
        uint8_t  ExtraChunkData[8];// 44-51 : payload (exact size = ExtraChunkSize; here fixed 8)
        /* data chunk header (52..59) */
        uint8_t  Subchunk2ID[4];   // 52-55 : "data"
        uint32_t Subchunk2Size;    // 56-59 : num_bytes_of_PCM = NumSamples * NumChannels * BitsPerSample/8
    };
} WAVHeader;



//typedef union PACKED {
//    uint8_t raw[WAVE_HDR_SIZE]; // raw view (60 bytes)
//
//    struct PACKED {
//        /* 0..11 */
//        uint8_t  ChunkID[4];       // 0-3   : "RIFF"
//        uint32_t ChunkSize;        // 4-7   : RIFF size = file_size - 8 (will compute)
//        uint8_t     Format[4];        // 8-11  : "WAVE"
//
//        /* fmt chunk  (12..35) */
//        uint8_t  Subchunk1ID[4];   // 12-15 : "fmt "
//        uint32_t Subchunk1Size;    // 16-19 : usually 16 for PCM
//        uint16_t AudioFormat;      // 20-21 : PCM = 1
//        uint16_t NumChannels;      // 22-23 : mono=1, stereo=2
//        uint32_t SampleRate;       // 24-27 : e.g., 22050
//        uint32_t ByteRate;         // 28-31 : SampleRate * NumChannels * BitsPerSample/8
//        uint16_t BlockAlign;       // 32-33 : NumChannels * BitsPerSample/8
//        uint16_t BitsPerSample;    // 34-35 : bits per sample (e.g., 16)
//
//        /* extra chunk (IDIT) (36..51) */
//        uint8_t  ExtraChunkID[4];  // 36-39 : e.g., "IDIT" (you may set other ID)
//        uint32_t ExtraChunkSize;   // 40-43 : size of extra payload (in your file = 8)
//        uint8_t  ExtraChunkData[8];// 44-51 : payload (exact size = ExtraChunkSize; here fixed 8)
//
//        /* data chunk header (52..59) */
//        uint8_t  Subchunk2ID[4];   // 52-55 : "data"
//        uint32_t Subchunk2Size;    // 56-59 : num_bytes_of_PCM = NumSamples * NumChannels * BitsPerSample/8
//    } hdr;
//} WAVHeader;






// WAVE FILE
WAV_StatusTypeDef WAVFIL_Start_Write(const char *filename, uint32_t sizeByteFile, uint32_t samRat);
WAV_StatusTypeDef WAVFIL_Give_Write(float32_t *puerData);
WAV_StatusTypeDef WAVFIL_End_Write(void);
WAV_StatusTypeDef WAVFIL_Start_Read(const char *filename);
WAV_StatusTypeDef WAVFIL_Catch_Data(float32_t *dataBuf, uint32_t *puerData);
WAV_StatusTypeDef WAVFIL_End_Read(void);




// Mount and unmount
int sd_mount(void);
int sd_unmount(void);

// Basic file operations
int sd_write_file(const char *filename, const char *text);
int sd_append_file(const char *filename, const char *text);
int sd_read_file(const char *filename, char *buffer, UINT bufsize, UINT *bytes_read);
int sd_delete_file(const char *filename);
int sd_rename_file(const char *oldname, const char *newname);

// Directory handling
FRESULT sd_create_directory(const char *path);
void sd_list_directory_recursive(const char *path, int depth);
void sd_list_files(void);

// Space information
int sd_get_space_kb(void);

//csv File operations
// CSV Record structure
// CSV reader (caller defines record array)
int sd_read_csv(const char *filename, CsvRecord *records, int max_records, int *record_count);

#endif // __SD_FUNCTIONS_H__
