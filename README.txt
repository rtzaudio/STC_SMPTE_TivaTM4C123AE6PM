============================================================================

 STC-SMPTE Daughter Card for the STC-1200 communications interface.

 Copyright (C) 2016-2019, RTZ Professional Audio, LLC

 All Rights Reserved

 RTZ is registered trademark of RTZ Professional Audio, LLC

 Visit us on the web at http://www.rtzaudio.com
 
============================================================================

This archive contains firmware for the RTZ Audio STC-SMPTE digital transport
controller based on the Tiva TM4C123AE6PM M4 ARM processor. 

The file "STC_SMPTE_TivaTM4C123AE6PM.out" contains the object format version
of the firmware image for use with the XDS JTAG programming pods. These
are generally used for development systems or recovery only.

=== VERSION 1.00 (06/04/2019) ==============================================

First version for development and testing.

=== VERSION 1.01 (11/17/2026) ==============================================

Added another LED to allow RXD and TXD status indication. Added I2C0 port
to support all configuration and control requests from the master.
The SPI port was changed to operate in MASTER mode and will drive
all received packet data to the STC through a slave reader thread. The
SPI port will only be used to send timecode data to the STC slave reader
task. No interrupts are needed to the STC and data will simply stream
out the SPI port as packets are decoded. All control and configuration
requests will be handled via the I2C interface.  
