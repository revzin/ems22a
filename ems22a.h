/* --------------------------------------------------------------------
MIT License

Copyright (c) 2016 Grigory Revzin

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
------------------------------------------------------------------------ */

/** ems22a
 * A driver for Bourns EMS22A SPI rotary encoders.
 * Supports daisy-chained EMS22As.
 * Must be provided with your own SPI routines.
 */

#ifndef _EMS22A_H_
#define _EMS22A_H_

#define EMS22A_MAX_INSTANCES 3
#define EMS22A_MAX_DAISY_CHAIN 5

typedef int ems22a_angle;

typedef enum
{
	EMS22A_OK,
	EMS22A_BAD_SETTINGS,
	EMS22A_WARNING,
	EMS22A_FAILURE,
	EMS22A_WRONG_PARITY
} ems22a_rc_t;


/** SPI callback interface typedefs ---------------------------------------- */
/* must return 0 if successfull, anything != 0 if not 						 */

/** ems22a_spi_init: function that should initialize the SPI interface used to
 * read from
 * that particular ems22 instance, hardware or software */
typedef int (*ems22a_spi_init)(int id);
/** remember to clean up your SPI stuff in case this function fails! */

/** ems22a_spi_read: function that should read 16 bits from the SPI interface
 * associated with that particular ems22a sensor (sensors) and
 * put them to *reading_out.
 * NOTE: ems22a (https://www.bourns.com/pdfs/EMS22A.pdf) is CPOL=1, CPHA=0 SPI
 * and, due Bourns' laziness, it manages to send a fake extra 17th bit
 * in front of the packet (see fake_bit.png); this 17th bit
 * does not mean anything; however, it will be received by most hardware SPI
 * implementations INSTEAD of the 16th parity bit. So you'll have to shift
 * your reading one bit right like this:
 * uint16_t reading = SPI->DR;
 * reading = reading >> 1;
 * *reading_out = reading; */
typedef int (*ems22a_spi_read)(int id, uint16_t *reading_out);

/** ems22a_spi_deinit: function that should deinit the SPI interface used
 * by that particular instance */
typedef int (*ems22a_spi_deinit)(int id);


/** external interface ----------------------------------------------------- */
/* ------------------------------------------------------------------------- */

/* starts the instance with SPI interface functions.
 * n_daisy_chain - number of daisy-chained devices on this interface,
 * comment_string is a comment string (can be NULL),
 * no_parity_control switches off parity control*/
int ems22a_instance_start(ems22a_spi_init initf,
											ems22a_spi_read readf,
											ems22a_spi_deinit deinitf,
											int n_daisy_chain,
											char *comment_string,
											int no_parity_control);
/* NOTE: this will call the deinitf callback in case init fails */

/* ends the instance */
ems22a_rc_t ems22a_instance_end(int ems22a_instance_id);

/* reads the current angle to *angle */
ems22a_rc_t ems22a_read_angle(int id,
										int daisy_chain_id,
										ems22a_angle *angle);

/* updates values in all instances */
int ems22a_update(void);

#endif
