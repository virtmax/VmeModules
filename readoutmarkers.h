/*

    @brief  Markers for the identification of the VME module data blocks in a data stream.
            Have to be used in the GSI MBS f_user.c file.

    @example

    #ifdef V560
            *pl_dat++ = 0xCAE00560; *l_se_read_len += 4;	// module id, see readout_markers.h
            *pl_dat++ = 0x00000000; *l_se_read_len += 4;	// module number
            *l_se_read_len += CaenV560_SingleCycleReadout(&v560, &pl_dat);
            *pl_dat++ = 0x12233445; *l_se_read_len += 4;	// END-marker
    #endif

*/

#pragma once

const static int32_t STRUCK_SIS3316_ID =    0x83733316; // Struck SIS3316 ADC "SI"+3316
const static int32_t STRUCK_SIS3302_ID =    0x83733302; // Struck SIS3302 ADC "SI"+3302
const static int32_t CAEN_V560_ID =         0xCAE00560; // CAEN V560 Scaler
const static int32_t CAEN_V1190TDC_ID =     0xCAE01190; // CAEN V1190 TDC
const static int32_t ELB_VFB6TDC_ID =       0x56464236; // ELB VFB6 TDC "VFB6"
const static int32_t READOUTNR_ID =         0x5245434f; // optional readout marker
const static int32_t END_ID =               0x3c454e44; // end marker, should be placed after every module readout: e.g. SIS3316_ID readout code END_ID ... SIS3302_ID readout code END_ID
const static int32_t END_ID_OLD =           0x12233445;
