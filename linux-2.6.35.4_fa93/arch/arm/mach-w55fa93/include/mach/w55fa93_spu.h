
#include <mach/w55fa93_reg.h>

#ifndef _W55FA93_SPU_H_
#define _W55FA93_SPU_H_

#include <linux/io.h>

#define SPU_ACTIVE			0x1
#define SPU_PLAY_ACTIVE		0x2
#define SPU_REC_ACTIVE		0x4

#define E_DRVSPU_WRONG_CHANNEL     	0xFFFFFFFF
#define E_DRVSPU_WRONG_INTERRUPT   	0xFFFFFFFE
#define E_SUCCESS					0x00


#define DRVSPU_USER_INT			EV_USR_FG		// User event interrupt enable bit
#define DRVSPU_SILENT_INT		EV_SLN_FG		// Silent event interrupt enable bit
#define DRVSPU_LOOPSTART_INT	EV_LP_FG		// Loop start event interrupt enable bit
#define DRVSPU_END_INT			EV_END_FG		// End event interrupt enable bit
#define DRVSPU_ENDADDRESS_INT	END_FG			// End Address event interrupt enable bit
#define DRVSPU_THADDRESS_INT	TH_FG			// Threshold Address event interrupt enable bit
#define DRVSPU_ALL_INT			EV_USR_FG+EV_SLN_FG+EV_LP_FG+EV_END_FG+END_FG+TH_FG	


#define DRVSPU_LOAD_SELECTED_CHANNEL	0x01	// load selected channel
#define DRVSPU_UPDATE_ALL_SETTINGS		0x02	// update all registers settings in selected channel
#define DRVSPU_UPDATE_PARTIAL_SETTINGS	0x03	// update partial registers settings in selected channel
#define DRVSPU_UPDATE_IRQ_PARTIAL		0x80	// update Interrupt partial
#define DRVSPU_UPDATE_DFA_PARTIAL		0x40	// update DFA partial
#define DRVSPU_UPDATE_PAN_PARTIAL		0x20	// update PAN partial
#define DRVSPU_UPDATE_VOL_PARTIAL		0x10	// update Volume partial
#define DRVSPU_UPDATE_ALL_PARTIALS		0xF0	// update ALL partials
#define DRVSPU_UPDATE_PAUSE_PARTIAL		0x0B	// update PAUSE partial


#define DRVSPU_MDPCM_FORMAT				0x00	// source format is MDPCM
#define DRVSPU_LP8_FORMAT				0x01	// source format is LP8
#define DRVSPU_PCM16_FORMAT				0x03	// source format is PCM16
#define DRVSPU_TONE_FORMAT				0x04	// source format is Tone
#define DRVSPU_MONO_PCM16				0x05	// mono PCM16 
#define DRVSPU_STEREO_PCM16_LEFT		0x06	// stereo PCM16 left channel [15:0]
#define DRVSPU_STEREO_PCM16_RIGHT		0x07	// stereo PCM16 left channel [31:16]


typedef enum {
				eDRVSPU_EQBAND_DC = 0, 
				eDRVSPU_EQBAND_1, 
				eDRVSPU_EQBAND_2, 
				eDRVSPU_EQBAND_3, 
				eDRVSPU_EQBAND_4, 
				eDRVSPU_EQBAND_5, 
				eDRVSPU_EQBAND_6, 
				eDRVSPU_EQBAND_7, 
				eDRVSPU_EQBAND_8, 
				eDRVSPU_EQBAND_9,
				eDRVSPU_EQBAND_10	} E_DRVSPU_EQ_BAND;
				
typedef enum {
				eDRVSPU_CHANNEL_0 = 0, 
				eDRVSPU_CHANNEL_1, 
				eDRVSPU_CHANNEL_2, 
				eDRVSPU_CHANNEL_3, 
				eDRVSPU_CHANNEL_4, 
				eDRVSPU_CHANNEL_5, 
				eDRVSPU_CHANNEL_6, 
				eDRVSPU_CHANNEL_7, 
				eDRVSPU_CHANNEL_8, 
				eDRVSPU_CHANNEL_9,
				eDRVSPU_CHANNEL_10, 
				eDRVSPU_CHANNEL_11, 
				eDRVSPU_CHANNEL_12, 
				eDRVSPU_CHANNEL_13, 
				eDRVSPU_CHANNEL_14, 
				eDRVSPU_CHANNEL_15, 
				eDRVSPU_CHANNEL_16, 
				eDRVSPU_CHANNEL_17, 
				eDRVSPU_CHANNEL_18, 
				eDRVSPU_CHANNEL_19, 
				eDRVSPU_CHANNEL_20, 
				eDRVSPU_CHANNEL_21, 
				eDRVSPU_CHANNEL_22, 
				eDRVSPU_CHANNEL_23, 
				eDRVSPU_CHANNEL_24, 
				eDRVSPU_CHANNEL_25, 
				eDRVSPU_CHANNEL_26, 
				eDRVSPU_CHANNEL_27, 
				eDRVSPU_CHANNEL_28, 
				eDRVSPU_CHANNEL_29, 
				eDRVSPU_CHANNEL_30, 
				eDRVSPU_CHANNEL_31	} E_DRVSPU_CHANNEL;
				

typedef enum {
				eDRVSPU_EQGAIN_M7DB = 0, 
				eDRVSPU_EQGAIN_M6DB,
				eDRVSPU_EQGAIN_M5DB,
				eDRVSPU_EQGAIN_M4DB,
				eDRVSPU_EQGAIN_M3DB,
				eDRVSPU_EQGAIN_M2DB,
				eDRVSPU_EQGAIN_M1DB,
				eDRVSPU_EQGAIN_M0DB,
				eDRVSPU_EQGAIN_P1DB,
				eDRVSPU_EQGAIN_P2DB,
				eDRVSPU_EQGAIN_P3DB,
				eDRVSPU_EQGAIN_P4DB,
				eDRVSPU_EQGAIN_P5DB,
				eDRVSPU_EQGAIN_P6DB,
				eDRVSPU_EQGAIN_P7DB,
				eDRVSPU_EQGAIN_P8DB	} E_DRVSPU_EQ_GAIN;

typedef enum {
				eDRVSPU_FREQ_96000 = 96000, 
				eDRVSPU_FREQ_88200 = 88200, 				
				eDRVSPU_FREQ_64000 = 64000, 
				eDRVSPU_FREQ_48000 = 48000, 
				eDRVSPU_FREQ_44100 = 44100, 
				eDRVSPU_FREQ_32000 = 32000, 
				eDRVSPU_FREQ_24000 = 24000, 
				eDRVSPU_FREQ_22050 = 22050, 
				eDRVSPU_FREQ_16000 = 16000, 
				eDRVSPU_FREQ_12000 = 12000, 
				eDRVSPU_FREQ_11025 = 11025, 
				eDRVSPU_FREQ_8000  = 8000	} E_DRVSPU_SAMPLING;

#if 0
typedef struct {
				UINT32 u32ChannelIndex;
				UINT8  u8ChannelVolume;
				UINT16  u16PAN;
				UINT8  u8DataFormat;				
				UINT16 u16DFA;				
//				UINT8  u8SubIndex;				
//				UINT8  u8EventIndex;								
				UINT32 u32SrcBaseAddr;
				UINT32 u32SrcThresholdAddr;				
				UINT32 u32SrcEndAddr;								
//				UINT32 u32CurrentAddr;				
//				UINT32 u32LoopStartAddr;								
//				UINT32 u32LoopPlayByteCnt;												
				UINT16 u16SrcSampleRate;												
				UINT16 u16OutputSampleRate;																
											} S_CHANNEL_CTRL;
#endif
//typedef int (PFN_DRVSPU_CB_FUNC)(UINT8 *);


#endif
