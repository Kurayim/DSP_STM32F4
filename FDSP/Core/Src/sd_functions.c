/******************************************************************************
 *  File        : sd_functions.c
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

#include "sd_functions.h"





char sd_path[4];
FATFS fs;
FATFS _wav;
FILINFO fileInfo;
FIL r_filwav;
FIL w_filwav;
FILINFO infowav;
UINT bytes_read;
FILINFO fno;
FRESULT resl;



uint8_t  BufHeader[WAVE_HED_SIZ+1] 	= {'\0'};
uint32_t R_WavSize 		= 0;
uint32_t R_WavDataNum 	= 0;
uint32_t R_WavremainData 	= 0;
uint16_t R_WavSamRat		= 0;

uint32_t W_WavremainData = 0;



WAV_StatusTypeDef WAVFIL_Start_Write(const char *filename, uint32_t sizeByteFile, uint32_t samRat){


	UINT bytesWritten;
	WAVHeader w_header;

	memcpy(w_header.ChunkID, "RIFF", 4);

	w_header.ChunkSize = sizeByteFile + WAVE_HED_SIZ - 8;

	memcpy(w_header.Format, "WAVE", 4);

	memcpy(w_header.Subchunk1ID, "fmt ", 4);

	w_header.Subchunk1Size = 16;

	w_header.AudioFormat = 1;

	w_header.NumChannels = 1;

	w_header.SampleRate = samRat;

	w_header.BitsPerSample = 16;

	w_header.ByteRate = w_header.SampleRate * w_header.NumChannels * w_header.BitsPerSample / 8;

	w_header.BlockAlign = w_header.NumChannels * w_header.BitsPerSample / 8;

	memcpy(w_header.ExtraChunkID, "IDIT", 4);

	w_header.ExtraChunkSize = 8;

	memcpy(w_header.ExtraChunkData, "abcdefg", 8);   // Payload

	memcpy(w_header.Subchunk2ID, "data", 4);

	w_header.Subchunk2Size = sizeByteFile;


	resl = f_open(&w_filwav, filename, FA_WRITE | FA_CREATE_ALWAYS);
	if (resl != FR_OK)
	{
		sd_unmount();
		return WAV_ERROR_F_OPEN;
	}

	resl = f_write(&w_filwav, &w_header.raw, WAVE_HED_SIZ, &bytesWritten);
	if (resl != FR_OK  ||  bytesWritten != WAVE_HED_SIZ)
	{
		sd_unmount();
		return WAV_ERROR_F_WRIT;
	}

	W_WavremainData = sizeByteFile;



	return WAV_OK;
}



WAV_StatusTypeDef WAVFIL_Give_Write(float32_t *puerData){

	UINT 	  bytesWritten;
	uint32_t  writeNum = 0;
	uint8_t   w_rawdata[WAVE_DAT_RED_SIZ+1] = {'\0'};
//	float32_t f_sample = 0;
//	int16_t   i_sample = 0;
//	uint16_t  u_sample = 0;
	int8_t   sizeFram = 0;
	uint32_t  indexWrite = 0;
	float32_t valSample = 0;


	for(int i = 0; i < WAVE_DAT_RED_SIZ; i=i+2){

		valSample = *puerData;

		if(valSample >  1.0f)
			valSample =  1.0f;
		if(valSample < -1.0f)
			valSample = -1.0f;

		valSample = valSample * NORMALISE;

		w_rawdata[i]   =  ((int16_t)valSample) & 0x00ff;
		w_rawdata[i+1] = (((int16_t)valSample) & 0xff00) >> 8;
		puerData++;
	}


//	if(W_WavremainData > WAVE_DAT_RED_SIZ)
//		writeNum = WAVE_DAT_RED_SIZ;
//	else
//		writeNum = W_WavremainData;
//
//
//	resl = f_write(&w_filwav, w_rawdata, writeNum, &bytesWritten);
//	if (resl != FR_OK  ||  bytesWritten != writeNum)
//	{
//		sd_unmount();
//		return WAV_ERROR_F_WRIT;
//	}
//	W_WavremainData = W_WavremainData - bytesWritten;
//	*NumByteWrite = bytesWritten;


	if(W_WavremainData >= WAVE_DAT_RED_SIZ)
		sizeFram = WAVE_DAT_RED_SIZ / 1024;
	else{
		sizeFram = ceilf((float32_t)W_WavremainData / 1024.0f);
	}

	indexWrite = 0;
	while(sizeFram > 0)
	{
		if(W_WavremainData > 1024)
		{
			writeNum = 1024;
		}
		else
		{
			writeNum = W_WavremainData;
			sizeFram = 0;
		}

		resl = f_write(&w_filwav, &w_rawdata[indexWrite], writeNum, &bytesWritten);
		if (resl != FR_OK  ||  bytesWritten != writeNum) {
			sd_unmount();
			return WAV_ERROR_F_WRIT;
		}
		sizeFram--;
		indexWrite = indexWrite + bytesWritten;
		W_WavremainData = W_WavremainData - bytesWritten;

	}


	return WAV_OK;
}

WAV_StatusTypeDef WAVFIL_End_Write(void){

	f_close(&w_filwav);

	return WAV_OK;
}









WAV_StatusTypeDef WAVFIL_Start_Read(const char *filename){

	R_WavSize 		= 0;
	R_WavDataNum 	= 0;
	R_WavremainData = 0;
	R_WavSamRat		= 0;
	memset(BufHeader, '\0', WAVE_HED_SIZ+1);

	WAVHeader r_header;


	// Read size file
	resl = f_stat(filename, &fno);
	if (resl != FR_OK)
	{
		sd_unmount();
		return WAV_ERROR_F_STAT;
	}
	R_WavSize = fno.fsize;


	if (f_open(&r_filwav, filename, FA_READ) != FR_OK)
	{
		sd_unmount();
		return WAV_ERROR_F_OPEN;
	}

	// Read header
	resl = f_read(&r_filwav, r_header.raw, WAVE_HED_SIZ, &bytes_read);
    if (resl != FR_OK || bytes_read != WAVE_HED_SIZ) {
        f_close(&r_filwav);
        sd_unmount();
        return WAV_ERROR_F_READ;
    }

    //  Header check
    if(memcmp(r_header.ChunkID, "RIFF", 4) || r_header.ChunkSize != (R_WavSize-8) ||
       memcmp(r_header.Format , "WAVE", 4) || memcmp(r_header.Subchunk1ID, "fmt ", 4) ||
	   memcmp(r_header.ExtraChunkID , "IDIT", 4) || r_header.BitsPerSample != 16,
	   memcmp(r_header.Subchunk2ID , "data", 4)){

    	sd_unmount();
    	return WAV_ERROR_HEADER;
    }


	R_WavSamRat  	  = r_header.SampleRate;
	R_WavDataNum 	  = R_WavSize - (WAVE_HED_SIZ);
//	R_WavremainData   = R_WavDataNum;

	R_WavremainData = r_header.Subchunk2Size;

//	if(R_WavremainData != r_header.Subchunk2Size){
//		sd_unmount();
//		return WAV_ERROR_HEADER;
//	}


	return WAV_OK;
}


WAV_StatusTypeDef WAVFIL_Catch_Data(float32_t *dataBuf, uint32_t *puerData){

	UINT     bytes_read = 0;
	uint32_t readNum 	= 0;
	uint8_t  r_rawData[WAVE_DAT_RED_SIZ+1] = {'\0'};
	int8_t   sizeFram = 0;
	uint32_t indexRead = 0;


//	if(R_WavremainData > WAVE_DAT_RED_SIZ)
//		readNum = WAVE_DAT_RED_SIZ;
//	else
//		readNum = R_WavremainData;
//
//
//	resl = f_read(&r_filwav, r_rawData, readNum, &bytes_read);
//	if (resl != FR_OK  ||  bytes_read != readNum) {
//		sd_unmount();
//		return WAV_ERROR_F_READ;
//	}



	if(R_WavremainData >= WAVE_DAT_RED_SIZ)
		sizeFram = WAVE_DAT_RED_SIZ / 1024;
	else{
		sizeFram = ceilf((float32_t)R_WavremainData / 1024.0f);
	}

	indexRead = 0;
	while(sizeFram > 0)
	{
		if(R_WavremainData > 1024)
		{
			readNum = 1024;
		}
		else
		{
			readNum = R_WavremainData;
			sizeFram = 0;
		}

		resl = f_read(&r_filwav, &r_rawData[indexRead], readNum, &bytes_read);
		if (resl != FR_OK  ||  bytes_read != readNum) {
			sd_unmount();
			return WAV_ERROR_F_READ;
		}
		sizeFram--;
		indexRead = indexRead + bytes_read;
		R_WavremainData = R_WavremainData - bytes_read;

	}




	*puerData = 0;
	for(int i = 0; i < WAVE_DAT_RED_SIZ; i = i+2){
		*dataBuf = ((float32_t)((int16_t)(((int16_t)r_rawData[i+1]) << 8  |  r_rawData[i]))) / NORMALISE;
		dataBuf++;
		*puerData = *puerData + 1;
	}


	return WAV_OK;
}


WAV_StatusTypeDef WAVFIL_End_Read(void){

	f_close(&r_filwav);
	return WAV_OK;
}

/*
===============================================================================
    WAV File Header Structure (Standard PCM Format)
    Author: Your Name
    Description:
        This table describes the standard WAV file header fields, their sizes,
        byte offsets, and meaning. It follows the RIFF/WAVE PCM format spec.

===============================================================================
 Field Name       | Size (bytes) | Byte Offset | Description
-------------------------------------------------------------------------------
 ChunkID          | 4            | 0  - 3      | Contains the letters "RIFF" in ASCII.
                                              | Marks the file as a RIFF file.

 ChunkSize        | 4            | 4  - 7      | Size of the entire file minus 8 bytes
                                              | (from "RIFF" to the end of file).

 Format           | 4            | 8  - 11     | Contains the letters "WAVE" in ASCII.
                                              | Specifies that this is a WAV file.

 Subchunk1ID      | 4            | 12 - 15     | Contains the letters "fmt " in ASCII.
                                              | Marks the beginning of the format chunk.

 Subchunk1Size    | 4            | 16 - 19     | Size of the "fmt" chunk.
                                              | PCM = 16 bytes. Non-PCM can be larger.

 AudioFormat      | 2            | 20 - 21     | Format type:
                                              |   1 = PCM (uncompressed)
                                              |   3 = IEEE Float
                                              |   6 = 8-bit A-law
                                              |   7 = 8-bit mu-law
                                              |   Others = compressed

 NumChannels      | 2            | 22 - 23     | Number of channels:
                                              |   1 = Mono, 2 = Stereo, etc.

 SampleRate       | 4            | 24 - 27     | Sampling rate (samples per second):
                                              | Common: 44100, 48000, 96000, etc.

 ByteRate         | 4            | 28 - 31     | Bytes per second:
                                              |   ByteRate = SampleRate × NumChannels × BitsPerSample/8

 BlockAlign       | 2            | 32 - 33     | Number of bytes for one sample frame:
                                              |   BlockAlign = NumChannels × BitsPerSample/8

 BitsPerSample    | 2            | 34 - 35     | Number of bits per sample:
                                              |   8, 16, 24, 32 are common values.

 Subchunk2ID      | 4            | 36 - 39     | Contains the letters "data" in ASCII.
                                              | Marks the beginning of the data section.

 Subchunk2Size    | 4            | 40 - 43     | Size of the audio data in bytes:
                                              |   Subchunk2Size = NumSamples × NumChannels × BitsPerSample/8
===============================================================================
*/




int sd_format(void) {
    FRESULT res;
    BYTE work[512];

    res = f_mkfs(sd_path, FM_FAT32, 0, work, sizeof(work));
    if (res != FR_OK) {
        printf("Format failed: f_mkfs returned %d\r\n", res);
    }
    return res;
}

int sd_get_space_kb(void) {
    FATFS *pfs;
    DWORD fre_clust, tot_sect, fre_sect, total_kb, free_kb;
    FRESULT res = f_getfree(sd_path, &fre_clust, &pfs);
    if (res != FR_OK) return res;

    tot_sect = (pfs->n_fatent - 2) * pfs->csize;
    fre_sect = fre_clust * pfs->csize;
    total_kb = tot_sect / 2;
    free_kb = fre_sect / 2;
    printf("💾 Total: %lu KB, Free: %lu KB\r\n", total_kb, free_kb);
    return FR_OK;
}

int sd_mount(void) {
    FRESULT res;
    extern uint8_t sd_is_sdhc(void);

    printf("Linking SD driver...\r\n");
    if (FATFS_LinkDriver(&SD_Driver, sd_path) != 0) {
        printf("FATFS_LinkDriver failed\n");
        return FR_DISK_ERR;
    }

    printf("Initializing disk...\r\n");
    DSTATUS stat = disk_initialize(0);
    if (stat != 0) {
        printf("disk_initialize failed: 0x%02X\n", stat);
        printf("FR_NOT_READY\tTry Hard Reset or Check Connection/Power\r\n");
        return FR_NOT_READY;
    }

    printf("Attempting mount at %s...\r\n", sd_path);
    res = f_mount(&fs, sd_path, 1);
    if (res == FR_OK)
    {
        printf("SD card mounted successfully at %s\r\n", sd_path);
        printf("Card Type: %s\r\n", sd_is_sdhc() ? "SDHC/SDXC" : "SDSC");

        sd_get_space_kb();
        return FR_OK;
    }

    if (res == FR_NO_FILESYSTEM)
    {
        printf("No filesystem found on SD card. Attempting format...\r\nThis will create 32MB Partition (Most Probably)\r\n");
        printf("If you need the full sized SD card, use the computer to format into FAT32\r\n");
        sd_format();

        printf("Retrying mount after format...\r\n");
        res = f_mount(&fs, sd_path, 1);
        if (res == FR_OK) {
            printf("SD card formatted and mounted successfully.\r\n");
            printf("Card Type: %s\r\n", sd_is_sdhc() ? "SDHC/SDXC" : "SDSC");

            sd_get_space_kb();
        }
        else {
            printf("Mount failed even after format: %d\r\n", res);
        }
        return res;
    }

    printf("Mount failed with code: %d\r\n", res);
    return res;
}

int sd_unmount(void) {
    FRESULT res = f_mount(NULL, sd_path, 1);
    printf("SD card unmounted: %s\r\n", (res == FR_OK) ? "OK" : "Failed");
    return res;
}

int sd_write_file(const char *filename, const char *text) {
    FIL file;
    UINT bw;
    FRESULT res = f_open(&file, filename, FA_CREATE_ALWAYS | FA_WRITE);
    if (res != FR_OK) return res;

    res = f_write(&file, text, strlen(text), &bw);
    f_close(&file);
    printf("Write %u bytes to %s\r\n", bw, filename);
    return (res == FR_OK && bw == strlen(text)) ? FR_OK : FR_DISK_ERR;
}

int sd_append_file(const char *filename, const char *text) {
    FIL file;
    UINT bw;
    FRESULT res = f_open(&file, filename, FA_OPEN_ALWAYS | FA_WRITE);
    if (res != FR_OK) return res;

    res = f_lseek(&file, f_size(&file));
    if (res != FR_OK) {
        f_close(&file);
        return res;
    }

    res = f_write(&file, text, strlen(text), &bw);
    f_close(&file);
    printf("Appended %u bytes to %s\r\n", bw, filename);
    return (res == FR_OK && bw == strlen(text)) ? FR_OK : FR_DISK_ERR;
}

int sd_read_file(const char *filename, char *buffer, UINT bufsize, UINT *bytes_read) {
    FIL file;
    *bytes_read = 0;

    FRESULT res = f_open(&file, filename, FA_READ);
    if (res != FR_OK) {
        printf("f_open failed with code: %d\r\n", res);
        return res;
    }

    res = f_read(&file, buffer, bufsize - 1, bytes_read);
    if (res != FR_OK) {
        printf("f_read failed with code: %d\r\n", res);
        f_close(&file);
        return res;
    }

    buffer[*bytes_read] = '\0';

    res = f_close(&file);
    if (res != FR_OK) {
        printf("f_close failed with code: %d\r\n", res);
        return res;
    }

    printf("Read %u bytes from %s\r\n", *bytes_read, filename);
    return FR_OK;
}

int sd_read_csv(const char *filename, CsvRecord *records, int max_records, int *record_count) {
    FIL file;
    char line[128];
    *record_count = 0;

    FRESULT res = f_open(&file, filename, FA_READ);
    if (res != FR_OK) {
        printf("Failed to open CSV: %s (%d)", filename, res);
        return res;
    }

    printf("📄 Reading CSV: %s\r\n", filename);
    while (f_gets(line, sizeof(line), &file) && *record_count < max_records) {
        char *token = strtok(line, ",");
        if (!token) continue;
        strncpy(records[*record_count].field1, token, sizeof(records[*record_count].field1));

        token = strtok(NULL, ",");
        if (!token) continue;
        strncpy(records[*record_count].field2, token, sizeof(records[*record_count].field2));

        token = strtok(NULL, ",");
        if (token)
            records[*record_count].value = atoi(token);
        else
            records[*record_count].value = 0;

        (*record_count)++;
    }

    f_close(&file);

    for (int i = 0; i < *record_count; i++) {
        printf("[%d] %s | %s | %d\r\n", i,
               records[i].field1,
               records[i].field2,
               records[i].value);
    }

    return FR_OK;
}

int sd_delete_file(const char *filename) {
    FRESULT res = f_unlink(filename);
    printf("Delete %s: %s\r\n", filename, (res == FR_OK ? "OK" : "Failed"));
    return res;
}

int sd_rename_file(const char *oldname, const char *newname) {
    FRESULT res = f_rename(oldname, newname);
    printf("Rename %s to %s: %s\r\n", oldname, newname, (res == FR_OK ? "OK" : "Failed"));
    return res;
}

FRESULT sd_create_directory(const char *path) {
    FRESULT res = f_mkdir(path);
    printf("Create directory %s: %s\r\n", path, (res == FR_OK ? "OK" : "Failed"));
    return res;
}

void sd_list_directory_recursive(const char *path, int depth) {
    DIR dir;
    FILINFO fno;

    FRESULT res = f_opendir(&dir, path);
    if (res != FR_OK) {
        printf("%*s[ERR] Cannot open: %s\r\n", depth * 2, "", path);
        return;
    }

    while (1) {
        res = f_readdir(&dir, &fno);
        if (res != FR_OK || fno.fname[0] == 0) break;

        const char *name = fno.fname;  // استفاده فقط از fname

        if (fno.fattrib & AM_DIR) {
            if (strcmp(name, ".") && strcmp(name, "..")) {
                printf("%*s📁 %s\r\n", depth * 2, "", name);
                char newpath[128];
                snprintf(newpath, sizeof(newpath), "%s/%s", path, name);
                sd_list_directory_recursive(newpath, depth + 1);
            }
        } else {
            printf("%*s📄 %s (%lu bytes)\r\n", depth * 2, "", name, (unsigned long)fno.fsize);
        }
    }
    f_closedir(&dir);
}

void sd_list_files(void) {
    printf("📂 Files on SD Card:\r\n");
    sd_list_directory_recursive(sd_path, 0);
    printf("\r\n\r\n");
}
