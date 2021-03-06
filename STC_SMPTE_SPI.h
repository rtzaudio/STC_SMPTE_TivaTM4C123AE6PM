/* ============================================================================
 *
 * STC_SMPTE Time Code Controller for Ampex MM-1200 Tape Machines
 *
 * Copyright (C) 2021, RTZ Professional Audio, LLC
 * All Rights Reserved
 *
 * RTZ is registered trademark of RTZ Professional Audio, LLC
 *
 * ============================================================================ */

#ifndef _STC_SMPTE_SPI_H_
#define _STC_SMPTE_SPI_H_

/* SMPTE CONTROLLER SPI SLAVE REGISTERS
 *
 * All registers are 16-bits with the upper word containing the command
 * and flag bits. The lower 8-bits contains any associated data byte.
 *
 *   +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
 *   | R | A | A | A | C | C | C | C | B | B | B | B | B | B | B | B |
 *   | W | 6 | 5 | 4 | 3 | 2 | 1 | 0 | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *     |   |       |   |           |   |                           |
 *     |   +---+---+   +-----+-----+   +-------------+-------------+
 *     |       |             |                       |
 *    R/W     RSVD          REG                  DATA/FLAGS
 */

/* Upper bit of indicates R/W */
#define SMPTE_F_READ            (1 << 15)       /* Bit-16 1=read 0=write reg   */

/* SMPTE Daughter Card ID */
#define SMPTE_REVID             0xBA10          /* upper nibble always BA      */

/* Register number bits */
#define SMPTE_REG_MASK          0x0F00          /* C0-C3 register op-code      */

/* SMPTE Controller Registers (C0-C3) */
#define SMPTE_REG_REVID         1               /* Rev=lower nibble(RO, 16-bit)*/
#define SMPTE_REG_ENCCTL        2               /* Encoder Cntrl   (RW, 8-bit) */
#define SMPTE_REG_DECCTL        3               /* Decoder Cntrl   (RW, 8-bit) */
#define SMPTE_REG_STAT          4               /* Decode Status   (RO, 8-bit) */
#define SMPTE_REG_DATA          5               /* Data Register   (RO, 8-bit) */

#define SMPTE_REG_SET(x)        (((x) << 8) & SMPTE_REG_MASK)
#define SMPTE_REG_GET(x)        (((x) & SMPTE_REG_MASK) >> 8)

/* SMPTE_REG_GENCTL Register (B0-B7) */
#define SMPTE_ENCCTL_FPS24      0               /* Generator set for 24 FPS    */
#define SMPTE_ENCCTL_FPS25      1               /* Generator set for 25 FPS    */
#define SMPTE_ENCCTL_FPS30      2               /* Generator set for 30 FPS    */
#define SMPTE_ENCCTL_FPS30D     3               /* Set for 30 FPS drop frame   */

#define SMPTE_ENCCTL_FPS(x)     (((x) & 0x03))
#define SMPTE_ENCCTL_RESUME     (1 << 2)        /* do not reset time on start  */
#define SMPTE_ENCCTL_ENABLE     (1 << 7)        /* SMPTE generator enable bit  */
#define SMPTE_ENCCTL_DISABLE    (0)

#endif /*_STC_SMPTE_SPI_H_*/

/* End-Of-File */
