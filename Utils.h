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

#ifndef __UTILS_H__
#define __UTILS_H__

void InitSysDefaults(SYSPARMS* p);
int32_t SysParamsWrite(SYSPARMS* sp);
int32_t SysParamsRead(SYSPARMS* sp);

#endif /* __UTILS_H__ */

/* end-of-file */
