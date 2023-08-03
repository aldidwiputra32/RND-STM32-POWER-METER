#ifndef _HT7036
#define _HT7036

#include "main.h"
#include "spi.h"

/* NOTE FORMULA
 * 1) POWER PARAMETER = powerGroup*K
 *                    = Power
 *      K = 2.592*10^10/(HFconst*EC*2^23)  | HFconst = 1280(def)  &  EC = 6400
 *
 * 2) IrmsOffset = (Irms^2)/ (2^15)
 * 3) VrmsOffset = (Vrms^2)/ (2^15)
 * 4) V Gain:
 *		Vrms = RAWrms / 2 ^ 13
 *		Vgain = (Vactual / Vrms) - 1
 *		if Vgain >= 0:
 *			Vgain = int(Vgain * 2 ^ 15)
 *		if Vgain < 0:
 *			Vgain = int(2 ^ 16 + Vgain * 2 ^ 15)
 * 5) I Gain:
 */

// ------------------------------GROUP SENSOR & FORMULA----------------------------------------
#define POWER 			0
#define RMS				1
#define ENERGY			2

#define VRMS_OFFSET 	3
#define VRMS_GAIN 		4
#define IRMS_OFFSET 	5
#define IRMS_GAIN		6

#define COEF_POWER(n) 	((2.592*10000000000)/(n*43.700f*8388608.000f)) // K = 2.592*10^10/(HFconst*EC*2^23)

// ----------------------------DATA WRITE / READ-------------------------------------
#define BYTE_ENABLE 	0x00005A
#define BYTE_DISABLE 	0x0000FF
#define BYTE_NULL		0x000000

// ----------------------------SPECIAL COMMAND-------------------------------------
#define w_start_buffer	0xC0
#define w_read_buffer	0xC1
#define w_calib_restore	0xC3
#define w_reset 		0xD3
#define w_calib_state	0xC9
#define w_read_calib	0xC6

// ------------------REGISTER ADDRESS VALUE SENSOR (READ ONLY)---------------------
#define deviceId	0x00
#define r_Pa		0x01
#define r_Pb		0x02
#define r_Pc		0x03
#define r_Pt		0x04
#define r_Qa		0x05
#define r_Qb		0x06
#define r_Qc		0x07
#define r_Qt		0x08
#define r_Sa		0x09
#define r_Sb		0x0A
#define r_Sc		0x0B
#define r_St		0x0C
#define r_UaRms		0x0D
#define r_UbRms		0x0E
#define r_UcRms		0x0F
#define r_IaRms		0x10
#define r_IbRms		0x11
#define r_IcRms		0x12
#define r_ItRms		0x13
#define r_Pfa		0x14
#define r_Pfb		0x15
#define r_Pfc		0x16
#define r_Pft		0x17
#define r_Pga		0x18
#define r_Pgb		0x19
#define r_Pgc		0x1A
#define r_INTFlag	0x1B
#define r_Freq		0x1C
#define r_EFlag		0x1D
#define r_Epa		0x1E
#define r_Epb		0x1F
#define r_Epc		0x20
#define r_Ept		0x21
#define r_Eqa		0x22
#define r_Eqb		0x23
#define r_Eqc		0x24
#define r_Eqt		0x25
#define r_YUaUb		0x26
#define r_YUaUc		0x27
#define r_YUbUc		0x28
#define r_TPSD		0x2A
#define r_UtRms		0x2B
#define r_Sflag		0x2C
#define r_BckReg	0x2D
#define r_ComChksum	0x2E
#define r_Sample_IA	0x2F
#define r_Sample_IB	0x30
#define r_Sample_IC	0x31
#define r_Sample_UA	0x32
#define r_Sample_UB	0x33
#define r_Sample_UC	0x34
#define r_Esa		0x35
#define r_Esb		0x36
#define r_Esc		0x37
#define r_Est		0x38
#define r_FstCntA	0x39
#define r_FstCntB	0x3A
#define r_FstCntC	0x3B
#define r_FstCntT	0x3C
#define r_PFlag		0x3D
#define r_ChkSum	0x3E
#define r_Vrefgain	0x5C
#define r_ChipID	0x5D
#define r_ChkSum1	0x5E

// ------------------REGISTER ADDRESS POWER QUALITY(READ/WRITE)---------------------
#define r_LinePa		0x40
#define r_LinePb		0x41
#define r_LinePc		0x42
#define r_LinePt		0x43
#define r_LineEpa		0x44
#define r_LineEpb		0x45
#define r_LineEpc		0x46
#define r_LineEpt		0x47
#define r_LineUaRrms	0x48
#define r_LineUbRrms	0x49
#define r_LineUcRrms	0x4A
#define r_LineIaRrms	0x4B
#define r_LineIbRrms	0x4C
#define r_LineIcRrms	0x4D
#define r_LEFlag		0x4E
#define r_SAGFlag		0x4F
#define r_PeakUa		0x50
#define r_PeakUb		0x51
#define r_PeakUc		0x52
#define Reserved1		0x53
#define Reserved2		0x54
#define Reserved3		0x54
#define Reserved4		0x56
#define r_LineQa		0x57
#define r_LineQb		0x58
#define r_LineQc		0x59
#define r_LineQt		0x5A
#define r_PtrWavebuff	0x7E
#define r_WaveBuff		0x7F

// ------------------REGISTER ADDRESS CALIBRATION SENSOR(READ/WRITE)---------------------
#define	w_ModeCfg		0x1
#define	w_PGACtrl		0x2
#define	w_EMUCfg		0x3
#define	w_PgainA		0x4
#define	w_PgainB		0x5
#define	w_PgainC		0x6
#define	w_QgainA		0x7
#define	w_QgainB		0x8
#define	w_QgainC		0x9
#define	w_SgainA		0x0A
#define	w_SgainB		0x0B
#define	w_SgainC		0x0C
#define	w_PhSregApq0	0x0D
#define	w_PhSregBpq0	0x0E
#define	w_PhSregCpq0	0x0F
#define	w_PhSregApq1	0x10
#define	w_PhSregBpq1	0x11
#define	w_PhSregCpq1	0x12
#define	w_PoffsetA		0x13
#define	w_PoffsetB		0x14
#define	w_PoffsetC		0x15
#define	w_QPhscal		0x16
#define	w_UgainA		0x17
#define	w_UgainB		0x18
#define	w_UgainC		0x19
#define	w_IgainA		0x1A
#define	w_IgainB		0x1B
#define	w_IgainC		0x1C
#define	w_Istarup		0x1D
#define	w_Hfconst		0x1E
#define	w_FailVoltage	0x1F
#define	w_QoffsetA		0x21
#define	w_QoffsetB		0x22
#define	w_QoffsetC		0x23
#define	w_UaRmsoffse	0x24
#define	w_UbRmsoffse	0x25
#define	w_UcRmsoffse	0x26
#define	w_IaRmsoffse	0x27
#define	w_IbRmsoffse	0x28
#define	w_IcRmsoffse	0x29
#define	w_UoffsetA		0x2A
#define	w_UoffsetB		0x2B
#define	w_UoffsetC		0x2C
#define	w_IoffsetA		0x2D
#define	w_IoffsetB		0x2E
#define	w_IoffsetC		0x2F
#define	w_EMUIE		  	0x30
#define	w_ModuleCFG		0x31
#define	w_AllGain		0x32
#define	w_HFDouble		0x33
#define	w_LineGain		0x34
#define	w_PinCtrl		0x35
#define	w_Pstartup		0x36
#define	w_Iregion0		0x37
#define	w_Cyclength		0x38
#define	w_SAGLvl		0x39
#define	w_Iregion1		0x60
#define	w_PhSregApq2	0x61
#define	w_PhSregBpq2	0x62
#define	w_PhSregCpq2	0x63
#define	w_PoffsetAL		0x64
#define	w_PoffsetBL		0x65
#define	w_PoffsetCL		0x66
#define	w_QoffsetAL		0x67
#define	w_QoffsetBL		0x68
#define	w_QoffsetCL		0x69
#define	w_ItRmsoffset	0x6A
#define	w_TPSoffset		0x6B
#define	w_TPSgain		0x6C
#define	w_TCcoffA		0x6D
#define	w_TCcoffB		0x6E
#define	w_TCcoffC		0x6F
#define	w_EMCfg			0x70
#define	w_OILVL			0x71

// -------------------------------------------------------------------------------------
#define BIT_SIZE_21		2
#define BIT_SIZE_24 	0
#define BIT_SIZE_32		1
// -------------------------------PRIVATE VARIABLE--------------------------------------

void spiDisable();
void spiEnable();
HAL_StatusTypeDef spiWrite16(uint8_t address, uint16_t dataSet);
HAL_StatusTypeDef spiWrite24(uint8_t address, uint32_t dataSet);
HAL_StatusTypeDef spiCommandSpecial(uint8_t address, uint32_t dataSet);
uint32_t spiReadCalib(uint8_t address);
HAL_StatusTypeDef spiWriteCalib(uint8_t address, uint32_t dataSet);
uint16_t spiRead16(uint8_t address);
uint32_t spiRead24(uint8_t address);
void powerSetup(uint8_t * address, uint32_t * dataSet, HAL_StatusTypeDef * dataStatus, uint8_t numberCalib);
int32_t unsignToSign(uint32_t * data, uint8_t bitsize);
uint32_t powerScanValue(uint8_t address, uint32_t * addressBuffer ,uint32_t * valueBuffer, uint8_t size);
void powerReadSensor(uint8_t * address, uint32_t * valueBuffer, float * valueFloat, uint8_t size);
void powerCalib(uint8_t * addressBuffer, uint32_t * dataSet, HAL_StatusTypeDef * status, uint8_t size);
uint32_t powerCalculateCalib(uint8_t type, uint32_t dataRaw, float dataActual);
void powerRestoreCalib();
void handleAbsolute(float * value);

#endif

