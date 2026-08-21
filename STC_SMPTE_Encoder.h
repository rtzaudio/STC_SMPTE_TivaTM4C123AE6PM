/* ============================================================================
 *
 * DTC-1200 Digital Transport Controller for Ampex MM-1200 Tape Machines
 *
 * Copyright (C) 2016, RTZ Professional Audio, LLC
 * All Rights Reserved
 *
 * RTZ is registered trademark of RTZ Professional Audio, LLC
 *
 * ============================================================================ */

#ifndef STC_SMPTE_ENCODER_H
#define STC_SMPTE_ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include "libltc/ltc.h"

/*** Function Prototypes ***************************************************/

void SMPTE_initEncoder(void);
int SMPTE_Encoder_Start();
int SMPTE_Encoder_Stop(void);
void SMPTE_Encoder_Reset(void);

#ifdef __cplusplus
}
#endif

#endif /* SMPTE_LTC_H */

/* End-Of-File */
