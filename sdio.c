/** @addtogroup sdio_file

@author @htmlonly &copy; @endhtmlonly 2016 Kristian Nielsen <knielsen@knielsen-hq.org>

Library for SDIO interface in STM32 ARM microcontrollers by ST Microelectronics.

*/

/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2016 Kristian Nielsen <knielsen@knielsen-hq.org>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

/**@{*/

#include <libopencm3/stm32/sdio.h>

/*---------------------------------------------------------------------------*/
/** @brief Set SDIO Data length.

Sets the number of data bytes to be transfered.

@param[in] length unsigned 32 bit. Data length to set, in bytes.
*/

void sdio_set_data_length(uint32_t length)
{
	SDIO_DLEN = length;
}

/*---------------------------------------------------------------------------*/
/** @brief Set SDIO Data Timeout.

The timeout sets a limit on how long the SDIO host can remain in the Wait_R or
Busy states.

@param[in] timeout unsigned 32 bit. Timeout, in card bus clock periods.
*/

void sdio_set_data_timeout(uint32_t timeout)
{
	SDIO_DTIMER = timeout;
}

/*---------------------------------------------------------------------------*/
/** @brief Setup SDIO data transfer.

The SDIO data path state machine is started with specified parameters.
The data length and timeout should be setup prior to calling this function
(eg with sdio_set_data_length() and sdio_set_data_timeout()).

This function sets the DTEN bit of SDIO_DCTRL to enable the data path state
machine.

@param[in] read boolean. If true, the data transfer is a read from the card
into the SDIO peripheral. If false, it is a write from the SDIO to the card.
@param[in] stream boolean. If true, do a stream/SDIO multibyte transfer. If
false, do a block data transfer.
@param[in] blocksize unsigned 32 bit. The block size (for block transfers); one
of SDIO_DCTRL_DBLOCKSIZE_0 ... SDIO_DCTRL_DBLOCKSIZE_14.
*/

void sdio_data_transfer(bool read, bool stream, uint32_t blocksize)
{
	uint32_t mask = SDIO_DCTRL_DTEN | SDIO_DCTRL_DTDIR | SDIO_DCTRL_DTMODE |
		SDIO_DCTRL_DBLOCKSIZE_MASK;
	uint32_t reg32 = SDIO_DCTRL;

	reg32 &= ~mask;
	reg32 |= SDIO_DCTRL_DTEN | blocksize |
		(read ? SDIO_DCTRL_DTDIR : 0) |
		(stream ? SDIO_DCTRL_DTMODE : 0);
	SDIO_DCTRL = reg32;
}

/*---------------------------------------------------------------------------*/
/** @brief SDIO Read Status Flag.

@param[in] flags unsigned 32 bit. Any logical-or combination of the
24 SDIO_STA_* flags.
@returns boolean: true if any of the specified flags are set.
*/

bool sdio_get_flag(uint32_t flags)
{
	return ((SDIO_STA & flags) != 0);
}

/*---------------------------------------------------------------------------*/
/** @brief SDIO Clear Status Flag.

@param[in] flags unsigned 32 bit. Any logical-or combination of the
24 SDIO_ICR_* flags.
*/

void sdio_clear_flag(uint32_t flags)
{
	SDIO_ICR = flags;
}

/*---------------------------------------------------------------------------*/
/** @brief SDIO Read Power State.

@returns unsigned 32 bit: SDIO_POWER_PWRCTRL_PWROFF,
SDIO_POWER_PWRCTRL_RSVPWRUP, or SDIO_POWER_PWRCTRL_PWRON, if the card is in
the powered-off, power-up, or powered-on state, respectively.
*/

uint32_t sdio_get_power_state(void)
{
	return SDIO_POWER & SDIO_POWER_PWRCTRL_MASK;
}

/*---------------------------------------------------------------------------*/
/** @brief SDIO Set Power State.

@param[in] state unsigned 32 bit: SDIO_POWER_PWRCTRL_PWROFF,
SDIO_POWER_PWRCTRL_RSVPWRUP, or SDIO_POWER_PWRCTRL_PWRON.
*/

void sdio_set_power_state(uint32_t state)
{
	uint32_t reg32 = SDIO_POWER & ~SDIO_POWER_PWRCTRL_MASK;
	SDIO_POWER = reg32 | state;
}

/*---------------------------------------------------------------------------*/
/** @brief Read an SDIO status response register.

@param[in] reg unsigned 32 bit: Register to read (1, 2, 3, or 4). For a short
response, only register 1 is used
@returns unsigned 32 bit: The requested response status value.
*/

uint32_t sdio_get_status_response(uint32_t reg)
{
	switch (reg) {
	case 1:
	default:
		return SDIO_RESP1;
	case 2:
		return SDIO_RESP2;
	case 3:
		return SDIO_RESP3;
	case 4:
		return SDIO_RESP4;
	}
}

/*---------------------------------------------------------------------------*/
/** @brief SDIO Read Status Flag.

@returns unsigned 32 bit: Command index of the last command response received.
*/

uint32_t sdio_get_command_response(void)
{
	return SDIO_RESPCMD;
}

/*---------------------------------------------------------------------------*/
/** @brief Enable SDIO card clock.

*/
void sdio_enable(void)
{
	SDIO_CLKCR |= SDIO_CLKCR_CLKEN;
}

/*---------------------------------------------------------------------------*/
/** @brief Disable SDIO card clock.

*/
void sdio_disable(void)
{
	SDIO_CLKCR &= ~(SDIO_CLKCR_CLKEN);
}

/*---------------------------------------------------------------------------*/
/** @brief Set the width of the SDIO databus.

@param[in] mode unsigned 32 bit. Should be one of SDIO_CLKCR_WIDBUS_1,
SDIO_CLKCR_WIDBUS_4, or SDIO_CLKCR_WIDBUS_8.
*/
void sdio_set_widemode(uint32_t mode)
{
	uint32_t reg32;

	reg32 = SDIO_CLKCR;
	reg32 = (reg32 & ~SDIO_CLKCR_WIDBUS_MASK) | mode;
	SDIO_CLKCR = reg32;
}

/*---------------------------------------------------------------------------*/
/** @brief Read one word from the FIFO.

Reads the next word from the SDIO receive FIFO.
Before calling this function, it should be checked that the FIFO is not empty.
@returns unsigned 32 bit: Next word in FIFO.
*/

uint32_t sdio_read_fifo(void)
{
	return SDIO_FIFO;
}

/*---------------------------------------------------------------------------*/
/** @brief Write one word to the FIFO.

Write a word into the next free slot in the SDIO transmit FIFO.
Before calling this function, it should be checked that the FIFO is not full.
@param[in] value unsigned 32 bit. The word to write to the FIFO.
*/

void sdio_write_fifo(uint32_t value)
{
	SDIO_FIFO = value;
}

/*---------------------------------------------------------------------------*/
/** @brief Enable Specific SDIO interrupts.

@param[in] ints unsigned 32 bit. Logical-or combination of SDIO interrupts
to enable, selected from SDIO_MASK_* values.
*/

void sdio_enable_interrupts(uint32_t ints)
{
	SDIO_MASK |= ints;
}

/*---------------------------------------------------------------------------*/
/** @brief Disable Specific SDIO interrupts.

@param[in] ints unsigned 32 bit. Logical-or combination of SDIO interrupts
to disable, selected from SDIO_MASK_* values.
*/

void sdio_disable_interrupts(uint32_t ints)
{
	SDIO_MASK &= ~ints;
}

/*---------------------------------------------------------------------------*/
/** @brief Enable SDIO DMA.

*/

void sdio_enable_dma(void)
{
	SDIO_DCTRL |= SDIO_DCTRL_DMAEN;
}

/*---------------------------------------------------------------------------*/
/** @brief Disable SDIO DMA.

*/

void sdio_disable_dma(void)
{
	SDIO_DCTRL &= ~SDIO_DCTRL_DMAEN;
}

/*---------------------------------------------------------------------------*/
/** @brief Initiate an SDIO command.

Write the command argument and index to the SDIO peripheral. Automatically
sets the CPSMEN bit to start sending the command.

@param[in] cmdindex unsigned 32 bit. The 6-bit index of the command to send.
@param[in] waitresp unsigned 32 bit. One of SDIO_CMD_WAITRESP_NO_0,
SDIO_CMD_WAITRESP_SHORT, SDIO_CMD_WAITRESP_NO_2, or SDIO_CMD_WAITRESP_LONG to
specify which kind of response is expected.
@param[in] flags unsigned 32 bit. Optional additonal flags to write to the
SDIO peripheral. Any logical-or combination of SDIO_CMD_WAITINT,
SDIO_CMD_WAITPEND, SDIO_CMD_SDIOSUSPEND, SDIO_CMD_ENCMDCOMPL, SDIO_CMD_NIEN,
and SDIO_CMD_ATACMD (SDIO_CMD_CPSMEN is always set by this function).
@param[in] arg unsigned 32 bit. The argument for the command.
*/
void sdio_send_cmd(uint32_t cmdindex, uint32_t waitresp, uint32_t flags,
		   uint32_t arg)
{
	SDIO_ARG = arg;
	SDIO_CMD = (cmdindex << SDIO_CMD_CMDINDEX_SHIFT) | waitresp | flags |
		SDIO_CMD_CPSMEN;
}

/**@}*/

