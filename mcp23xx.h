#ifndef _LINUX_MCP32XX_H
#define _LINUX_MCP23XX_H

/* Platform data for the mcp23xx family of I2C/SPI to gpio expanders */
struct mcp23xx_platform_data {
	/*
	 * The address depends on the A[0-2] pins value
	 * the layout is:
	 *	0 1 0 0 0 A2 A1 A0
	 * A2 is 0 for SPI configurations.
	 */
	u8 address;
};

#endif /* _LINUX_MCP23XX_H */

