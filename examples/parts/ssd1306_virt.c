/*
 ssd1306_virt.c

 Copyright 2011 Michel Pollet <buserror@gmail.com>
 Copyright 2014 Doug Szumski <d.s.szumski@gmail.com>

 This file is part of simavr.

 simavr is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 simavr is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with simavr.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "sim_time.h"

#include "ssd1306_virt.h"
#include "avr_spi.h"
#include "avr_ioport.h"

//#define DEBUG
#ifdef DEBUG
#define DEBUG_PRINT(...) printf(__VA_ARGS__)
#else
#define DEBUG_PRINT(...)
#endif

#define OUR_I2C_ADDRESS 0x3C

/*
 * Write a byte at the current cursor location and then scroll the cursor.
 */
static void
ssd1306_write_data (ssd1306_t *part, uint8_t data)
{
	part->vram[part->cursor.page][part->cursor.column] = data;

	// Scroll the cursor
	if (++(part->cursor.column) > part->column_end)
	{
		part->cursor.column = part->column_start;
		if (++(part->cursor.page) >= SSD1306_VIRT_PAGES)
		{
			part->cursor.page = 0;
		}
	}

	ssd1306_set_flag (part, SSD1306_FLAG_DIRTY, 1);
}

/*
 * Called on the first command byte sent. For setting single
 * byte commands and initiating multi-byte commands.
 */
void
ssd1306_update_command_register (ssd1306_t *part, uint8_t data)
{
	switch (data)
	{
		case SSD1306_VIRT_SET_CONTRAST:
			part->command_register = data;
			DEBUG_PRINT("SSD1306: CONTRAST SET COMMAND: 0x%02x\n",
			            data);
			return;
		case SSD1306_VIRT_DISP_NORMAL:
			ssd1306_set_flag (part, SSD1306_FLAG_DISPLAY_INVERTED,
			                  0);
			ssd1306_set_flag (part, SSD1306_FLAG_DIRTY, 1);
			DEBUG_PRINT("SSD1306: DISPLAY NORMAL\n");
			SSD1306_CLEAR_COMMAND_REG(part);
			return;
		case SSD1306_VIRT_DISP_INVERTED:
			ssd1306_set_flag (part, SSD1306_FLAG_DISPLAY_INVERTED,
			                  1);
			ssd1306_set_flag (part, SSD1306_FLAG_DIRTY, 1);
			DEBUG_PRINT("SSD1306: DISPLAY INVERTED\n");
			SSD1306_CLEAR_COMMAND_REG(part);
			return;
		case SSD1306_VIRT_DISP_SUSPEND:
			ssd1306_set_flag (part, SSD1306_FLAG_DISPLAY_ON, 0);
			ssd1306_set_flag (part, SSD1306_FLAG_DIRTY, 1);
			DEBUG_PRINT("SSD1306: DISPLAY SUSPENDED\n");
			SSD1306_CLEAR_COMMAND_REG(part);
			return;
		case SSD1306_VIRT_DISP_ON:
			ssd1306_set_flag (part, SSD1306_FLAG_DISPLAY_ON, 1);
			ssd1306_set_flag (part, SSD1306_FLAG_DIRTY, 1);
			DEBUG_PRINT("SSD1306: DISPLAY ON\n");
			SSD1306_CLEAR_COMMAND_REG(part);
			return;
		case SSD1306_VIRT_SET_PAGE_START_ADDR
		                ... SSD1306_VIRT_SET_PAGE_START_ADDR
		                                + SSD1306_VIRT_PAGES - 1:
			part->cursor.page = data
			                - SSD1306_VIRT_SET_PAGE_START_ADDR;
			DEBUG_PRINT("SSD1306: SET PAGE ADDRESS: 0x%02x\n",
			            data);
			SSD1306_CLEAR_COMMAND_REG(part);
			return;
		case SSD1306_VIRT_SET_COLUMN_LOW_NIBBLE
		                ... SSD1306_VIRT_SET_COLUMN_LOW_NIBBLE + 0xF:
			data -= SSD1306_VIRT_SET_COLUMN_LOW_NIBBLE;
			part->cursor.column = (part->cursor.column & 0xF0)
			                | (data & 0xF);
			DEBUG_PRINT("SSD1306: SET COLUMN LOW NIBBLE: 0x%02x\n",
			            data);
			SSD1306_CLEAR_COMMAND_REG(part);
			return;
		case SSD1306_VIRT_SET_COLUMN_HIGH_NIBBLE
		                ... SSD1306_VIRT_SET_COLUMN_HIGH_NIBBLE + 0xF:
			data -= SSD1306_VIRT_SET_COLUMN_HIGH_NIBBLE;
			part->cursor.column = (part->cursor.column & 0xF)
			                | ((data & 0xF) << 4);
			DEBUG_PRINT("SSD1306: SET COLUMN HIGH NIBBLE: 0x%02x\n",
			            data);
			SSD1306_CLEAR_COMMAND_REG(part);
			return;
		case SSD1306_VIRT_SET_SEG_REMAP_0:
			ssd1306_set_flag (part, SSD1306_FLAG_SEGMENT_REMAP_0,
			                  1);
			DEBUG_PRINT("SSD1306: SET COLUMN ADDRESS 0 TO OLED SEG0 to \n");
			SSD1306_CLEAR_COMMAND_REG(part);
			return;
		case SSD1306_VIRT_SET_SEG_REMAP_127:
			ssd1306_set_flag (part, SSD1306_FLAG_SEGMENT_REMAP_0,
			                  0);
			DEBUG_PRINT("SSD1306: SET COLUMN ADDRESS 127 TO OLED SEG0 to \n");
			SSD1306_CLEAR_COMMAND_REG(part);
			return;
		case SSD1306_VIRT_SET_COM_SCAN_NORMAL:
			ssd1306_set_flag (part, SSD1306_FLAG_COM_SCAN_NORMAL,
			                  1);
			DEBUG_PRINT("SSD1306: SET COM OUTPUT SCAN DIRECTION NORMAL \n");
			SSD1306_CLEAR_COMMAND_REG(part);
			return;
		case SSD1306_VIRT_SET_COM_SCAN_INVERTED:
			ssd1306_set_flag (part, SSD1306_FLAG_COM_SCAN_NORMAL,
			                  0);
			DEBUG_PRINT("SSD1306: SET COM OUTPUT SCAN DIRECTION REMAPPED \n");
			SSD1306_CLEAR_COMMAND_REG(part);
			return;
		case SSD1306_VIRT_SET_COL_ADDR:
			part->command_register = data;
			part->column_start = 0xFF;
			DEBUG_PRINT("SSD1306: SET COLUMN ADDRESS: 0x%02x\n",
			            data);
			return;
		default:
			// Unknown command
			return;
	}
}

/*
 * Multi-byte command setting
 */
void
ssd1306_update_setting (ssd1306_t *part, uint8_t data)
{
	switch (part->command_register)
	{
		case SSD1306_VIRT_SET_CONTRAST:
			part->contrast_register = data;
			ssd1306_set_flag (part, SSD1306_FLAG_DIRTY, 1);
			SSD1306_CLEAR_COMMAND_REG(part);
			DEBUG_PRINT("SSD1306: CONTRAST SET: 0x%02x\n",
			            part->contrast_register);
			return;
		case SSD1306_VIRT_SET_COL_ADDR:
			if (part->column_start == 0xFF) {
				part->column_start = data;
				return;
			}
			part->column_end = data;
			part->cursor.column = part->column_start;
			DEBUG_PRINT("SSD1306: SET COLUMN ADDRESS: %d-%d\n",
					part->column_start, part->column_end);
			SSD1306_CLEAR_COMMAND_REG(part);
			return;
		default:
			// Unknown command
			return;
	}
}

/*
 * Determines whether a new command has been sent, or
 * whether we are in the process of setting a multi-
 * byte command.
 */
static void
ssd1306_write_command (ssd1306_t *part, uint8_t data)
{
	if (!part->command_register)
	{
		// Single byte or start of multi-byte command
		ssd1306_update_command_register (part, data);
	} else
	{
		// Multi-byte command setting
		ssd1306_update_setting (part, data);
	}
}

/*
 * Called when a SPI byte is sent
 */
static void
ssd1306_spi_in_hook (struct avr_irq_t * irq, uint32_t value, void * param)
{
	ssd1306_t * part = (ssd1306_t*) param;

	// Chip select should be pulled low to enable
	if (part->cs_pin)
		return;

	uint8_t data = value & 0xFF;

	switch (part->di_pin)
	{
		case SSD1306_VIRT_DATA:
			ssd1306_write_data (part, data);
			break;
		case SSD1306_VIRT_INSTRUCTION:
			ssd1306_write_command (part, data);
			break;
		default:
			// Invalid value
			break;
	}
}

/*
 * Called when chip select changes
 */
static void
ssd1306_cs_hook (struct avr_irq_t * irq, uint32_t value, void * param)
{
	ssd1306_t * p = (ssd1306_t*) param;
	p->cs_pin = value & 0xFF;
	DEBUG_PRINT("SSD1306: CHIP SELECT:  0x%02x\n", value);

}

/*
 * Called when data/instruction changes
 */
static void
ssd1306_di_hook (struct avr_irq_t * irq, uint32_t value, void * param)
{
	ssd1306_t * part = (ssd1306_t*) param;
	part->di_pin = value & 0xFF;
	DEBUG_PRINT("SSD1306: DATA / INSTRUCTION:  0x%08x\n", value);
}

/*
 * Called when a RESET signal is sent
 */
static void
ssd1306_reset_hook (struct avr_irq_t * irq, uint32_t value, void * param)
{
	DEBUG_PRINT("SSD1306: RESET\n");
	ssd1306_t * part = (ssd1306_t*) param;
	if (irq->value && !value)
	{
		// Falling edge
		memset (part->vram, 0, part->rows * part->pages);
		part->cursor.column = 0;
		part->cursor.page = 0;
		part->flags = 0;
		part->command_register = 0x00;
		part->contrast_register = 0x7F;
		ssd1306_set_flag (part, SSD1306_FLAG_COM_SCAN_NORMAL, 1);
		ssd1306_set_flag (part, SSD1306_FLAG_SEGMENT_REMAP_0, 1);
	}

}

/* I2C IRQ functions */

enum {
	I2C_IDLE,
	I2C_START,
	I2C_ADDRESS,
	I2C_COMMAND,
	I2C_PENDING_ACK,
	I2C_ACK,
	I2C_NOTFORUS
};

static void
ssd1306_i2c_handle_byte(ssd1306_t *part)
{
	DEBUG_PRINT("i2c_byte: 0x%x state:%d\n", part->i2c_byte,
			part->i2c_state);
	if (part->i2c_state == I2C_ADDRESS) {
		if (part->i2c_byte>>1 != OUR_I2C_ADDRESS) {
			part->i2c_state = I2C_NOTFORUS;
			return;
		}
		uint8_t rw = part->i2c_byte & 1;
		if (rw) {
			/* Read mode - not supported */
			part->i2c_state = I2C_NOTFORUS;
		}
		return;
	}
	switch(part->di_pin) {
		case 0:
			part->di_pin = part->i2c_byte | 1;
			break;
		case 0x81:
			part->di_pin = 0;
			/* Fall through */
		case 0x1:
			ssd1306_write_command(part, part->i2c_byte);
			break;
		case 0xc1:
			part->di_pin = 0;
			/* Fall through */
		case 0x41:
			ssd1306_write_data(part, part->i2c_byte);
			break;
	}
}

static void
ssd1306_i2c_scl(struct avr_irq_t *irq, uint32_t value, void *param)
{
	ssd1306_t * part = (ssd1306_t*) param;

	switch (part->i2c_state) {
		case I2C_START:
			if (value == 0) {
				part->i2c_state = I2C_ADDRESS;
				part->i2c_bits = 0;
				part->i2c_byte = 0;
				part->di_pin = 0;
			}
			break;
		case I2C_COMMAND:
		case I2C_ADDRESS:
			if (value == 1) {
				part->i2c_byte <<= 1;
				part->i2c_byte |= part->i2c_sda?1:0;
				part->i2c_bits++;
				if (part->i2c_bits == 8) {
					ssd1306_i2c_handle_byte(part);
					if (part->i2c_state != I2C_NOTFORUS)
						part->i2c_state =
							I2C_PENDING_ACK;
				}
			}
			break;
		case I2C_PENDING_ACK:
			if (value) {
				/* Send ack bit */
				avr_raise_irq(part->irq + IRQ_SSD1306_I2C_SDA, 0);
				part->i2c_state = I2C_ACK;
			}
			break;
		case I2C_ACK:
			if (!value) {
				/* Stop sending ack bit */
				avr_raise_irq_float(part->irq + IRQ_SSD1306_I2C_SDA, 1, 1);
				part->i2c_state = I2C_COMMAND;
				part->i2c_bits = 0;
				part->i2c_byte = 0;
			}
			break;
	}

	part->i2c_scl = value;
}

static void
ssd1306_i2c_sda(struct avr_irq_t *irq, uint32_t value, void *param)
{
	ssd1306_t * part = (ssd1306_t*) param;

	switch (part->i2c_state) {
		case I2C_IDLE: /* Start */
		case I2C_COMMAND: /* Repeated start */
			if (value == 0 && part->i2c_scl) {
				part->i2c_state = I2C_START;
			}
			break;
	}

	if (part->i2c_scl && value) {
		/* STOP */
		part->i2c_state = I2C_IDLE;
	}

	part->i2c_sda = value;
}

static const char * irq_names[IRQ_SSD1306_COUNT] =
{ [IRQ_SSD1306_SPI_BYTE_IN] = "=ssd1306.SDIN", [IRQ_SSD1306_RESET
                ] = "<ssd1306.RS", [IRQ_SSD1306_DATA_INSTRUCTION
                ] = "<ssd1306.RW", [IRQ_SSD1306_ENABLE] = "<ssd1306.E",
                [IRQ_SSD1306_ADDR] = "7>hd44780.ADDR" };

static const char * irq_names_i2c[IRQ_SSD1306_I2C_COUNT] = {
	[IRQ_SSD1306_I2C_SCL] = "ssd1306.SCL",
	[IRQ_SSD1306_I2C_SDA] = "ssd1306.SDA"
};

void
ssd1306_connect (ssd1306_t * part, ssd1306_wiring_t * wiring)
{
	avr_connect_irq (
	                avr_io_getirq (part->avr, AVR_IOCTL_SPI_GETIRQ(0),
	                               SPI_IRQ_OUTPUT),
	                part->irq + IRQ_SSD1306_SPI_BYTE_IN);

	avr_connect_irq (
	                avr_io_getirq (part->avr,
	                               AVR_IOCTL_IOPORT_GETIRQ(
	                                               wiring->chip_select.port),
	                               wiring->chip_select.pin),
	                part->irq + IRQ_SSD1306_ENABLE);

	avr_connect_irq (
	                avr_io_getirq (part->avr,
	                               AVR_IOCTL_IOPORT_GETIRQ(
	                                               wiring->data_instruction.port),
	                               wiring->data_instruction.pin),
	                part->irq + IRQ_SSD1306_DATA_INSTRUCTION);

	avr_connect_irq (
	                avr_io_getirq (part->avr,
	                               AVR_IOCTL_IOPORT_GETIRQ(
	                                               wiring->reset.port),
	                               wiring->reset.pin),
	                part->irq + IRQ_SSD1306_RESET);
}

void
ssd1306_connect_i2c (ssd1306_t *part, avr_irq_t *scl, avr_irq_t *sda)
{
	avr_connect_irq ( scl, part->irq + IRQ_SSD1306_I2C_SCL );
	avr_connect_irq ( sda, part->irq + IRQ_SSD1306_I2C_SDA );
}

static void
ssd1306_init_internal (struct avr_t *avr, struct ssd1306_t * part, int width, int height, int i2c)
{
	if (!avr || !part)
		return;

	memset (part, 0, sizeof(*part));
	part->avr = avr;
	part->columns = width;
	part->rows = height;
	part->pages = height / 8; 	// 8 pixels per page
	part->column_start = 0;
	part->column_end = width-1;

	if (!i2c) {
		/*
		 * Register callbacks on all our IRQs
		 */
		part->irq = avr_alloc_irq (&avr->irq_pool, 0, IRQ_SSD1306_COUNT,
		                           irq_names);

		avr_irq_register_notify (part->irq + IRQ_SSD1306_SPI_BYTE_IN,
		                         ssd1306_spi_in_hook, part);
		avr_irq_register_notify (part->irq + IRQ_SSD1306_RESET,
		                         ssd1306_reset_hook, part);
		avr_irq_register_notify (part->irq + IRQ_SSD1306_ENABLE,
		                         ssd1306_cs_hook, part);
		avr_irq_register_notify (part->irq + IRQ_SSD1306_DATA_INSTRUCTION,
		                         ssd1306_di_hook, part);
	} else {
		part->irq = avr_alloc_irq (&avr->irq_pool, 0, IRQ_SSD1306_I2C_COUNT,
				irq_names_i2c);

		avr_irq_register_notify(part->irq + IRQ_SSD1306_I2C_SCL,
				ssd1306_i2c_scl, part);
		avr_irq_register_notify(part->irq + IRQ_SSD1306_I2C_SDA,
				ssd1306_i2c_sda, part);
	}

	printf ("SSD1306: %duS is %d cycles for your AVR\n", 37,
	        (int) avr_usec_to_cycles (avr, 37));
	printf ("SSD1306: %duS is %d cycles for your AVR\n", 1,
	        (int) avr_usec_to_cycles (avr, 1));
}

void
ssd1306_init (struct avr_t *avr, struct ssd1306_t * part, int width, int height)
{
	ssd1306_init_internal(avr, part, width, height, 0);
}

void
ssd1306_init_i2c (struct avr_t *avr, struct ssd1306_t * part, int width, int height)
{
	ssd1306_init_internal(avr, part, width, height, 1);
	/* Both inputs should start high */
	part->i2c_sda = 1;
	part->i2c_scl = 1;
}
