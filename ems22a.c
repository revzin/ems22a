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

#include <stdlib.h>

/* zed's 'awesome debug macros' */
#include "zed_dbg.h"

#include "ems22a.h"

/* ems22 frame layout */
typedef struct
{
	uint16_t
			parity 			: 1,
			mag_decrease 	: 1,
			mag_increase 	: 1,
			linearity_alarm : 1,
			cordic_oflow 	: 1,
			end_offst_comp 	: 1,
			reading 		: 10;
} frame;

/* ems22 instance control */
typedef struct
{
	ems22a_spi_init initf;
	ems22a_spi_read readf;
	ems22a_spi_deinit deinitf;

	short *readings;
	short *zeroes;
	int n_daisy_chain;
	int lock;

	int disable_parity_checks;

	char *comment;
} instance;

/* global module variables */
static instance *instances[EMS22A_MAX_INSTANCES] = {NULL};
static size_t n_instances = 0;


/* useful macros */
#define CHECK_ID(id) 														\
		check(id < EMS22A_MAX_INSTANCES, "instance id too large");			\
		check(id > -1, "invalid instance id");								\
		check(instances[id] != NULL, "invalid instance id"); 				\

#define CHECK_DAISY_CHAIN(inst, dcn) 										\
		check(dcn < EMS22A_MAX_DAISY_CHAIN, "daisy chain number too large"); \
		check(dcn > -1, "daisy chain number can't be negative"); 			\
		check(dcn < inst->n_daisy_chain, 									\
							"daisy chain number %d too large on instance %s", \
							dcn, inst->comment); 							\

/* update the ith instance  */
static ems22a_rc_t ems22a_update_instance(int i);

/* extract angle from frame struct */
static ems22a_rc_t reading2angle(instance *inst, frame *f, ems22a_angle *angle_out);

/* cast frame as a uint16_t (which it is) */
static uint16_t frame2data(frame *f);

/* updates the given instance */
static ems22a_rc_t ems22a_update_instance(int i);

/* ------------------------------------------------------ */
/* external interface 									  */

int ems22a_instance_start(ems22a_spi_init initf,
											ems22a_spi_read readf,
											ems22a_spi_deinit deinitf,
											int n_daisy_chain,
											char *comment_string,
											int no_parity_control)
{
	instance *inst = NULL;

	check(initf && readf && deinitf, "ems22a_instance_start: args can't be NULL");

	check(n_daisy_chain > 0 && n_daisy_chain < EMS22A_MAX_INSTANCES,
													"ems22a_instance_start: number "
													"of daisy chained devices wrong");

	int free_inst = -1;
	for (int j = 0; j < EMS22A_MAX_INSTANCES; ++j) {
		if (instances[j] == NULL) {
			free_inst = j;
			break;
		}
	}
	check(free_inst != -1, "too many ems22 instances");

	n_instances++;

	check(n_instances <= EMS22A_MAX_INSTANCES, "maximum number of instances reached");

	inst = malloc(sizeof(instance));
	check_mem(inst);
	memset(inst, 0, sizeof(instance));

	inst->lock = 1;
	inst->initf = initf;
	inst->readf = readf;
	inst->deinitf = deinitf;
	inst->n_daisy_chain = n_daisy_chain;
	inst->disable_parity_checks = no_parity_control;

	if (comment_string) {
		inst->comment = comment_string;
	}
	else
		comment_string = "ems22a instance";

	inst->readings = malloc(sizeof(short) * n_daisy_chain);
	check_mem(inst->readings);
	memset(inst->readings, 0, sizeof(short) * inst->n_daisy_chain);

	inst->zeroes = malloc(sizeof(short) * n_daisy_chain);
	check_mem(inst->zeroes);
	memset(inst->zeroes, 0, sizeof(short) * inst->n_daisy_chain);



	int rr;
	check((rr = inst->initf(free_inst)) == 0, "ems22a_instance_start: "
										"initf returned error %d", rr);

	instances[free_inst] = inst;

	inst->lock = 0;

	return free_inst;

	error:

	if (inst) {

		inst->deinitf(free_inst);

		if (inst->readings)
			free(inst->readings);

		if (inst->zeroes)
			free(inst->zeroes);

		free(inst);
	}

	return -1;
}

int ems22a_update(void)
{
	int rc = EMS22A_OK;
	for (int i = 0; i < EMS22A_MAX_INSTANCES; ++i) {
		if (instances[i]) {
			 rc = ems22a_update_instance(i);
		}
	}
	return rc;
}

ems22a_rc_t ems22a_read_angle(int id, int daisy_chain_id, ems22a_angle *angle)
{
	CHECK_ID(id);

	instance *inst = instances[id];

	CHECK_DAISY_CHAIN(inst, daisy_chain_id);

	*angle = inst->readings[daisy_chain_id];

	return EMS22A_OK;

	error:
		return EMS22A_FAILURE;
}

ems22a_rc_t ems22a_instance_end(int ems22a_instance_id)
{
	CHECK_ID(ems22a_instance_id);

	instance *inst = instances[ems22a_instance_id];

	if (!inst->deinitf)
		log_err("inst->deinitf NULL, this is wrong");
	else
		inst->deinitf(ems22a_instance_id);

	if (!inst->readings)
		log_err("inst->readings NULL, shouldn't be that");
	else
		free(inst->readings);

	if (!inst->zeroes)
			log_err("inst->readings NULL, shouldn't be that");
	else
		free(inst->zeroes);

	free(inst);

	return EMS22A_OK;

	error:
		return EMS22A_FAILURE;
}


/* ------------------------------------------------------ */
/* internal stuff										  */

ems22a_rc_t check_parity(frame *f)
{
	/* frame must be exactly 16 bits - or we're doing something wrong */
	STATIC_ASSERT(sizeof(frame) == sizeof(uint16_t));
	return EMS22A_OK;

	int parity_counter = 0;
	uint16_t *frame_r = (uint16_t *) f;

	for (int i = 0; i < 16; ++i) {
		uint16_t mask = 1 << i;
		if (*frame_r & mask)
			parity_counter++;
	}

	return EMS22A_OK;

	if (parity_counter % 2 == f->parity)
		return EMS22A_OK;
	else {
		return EMS22A_WRONG_PARITY;
	}
}

uint16_t frame2data(frame *f)
{
	check(f, "frame must not be NULL");
	uint16_t *f_data = (uint16_t *) f;
	return *f_data;

	error:
		return 0;
}

ems22a_rc_t reading2angle(instance *inst, frame *f, ems22a_angle *angle_out)
{
	check(f, "frame can't be NULL");
	check(angle_out, "angle_out can't be NULL");

	if (!inst->disable_parity_checks) {
		int rc = check_parity(f);
		check(rc == EMS22A_OK, "Parity failure on frame %d", frame2data(f));
	}

	if (f->cordic_oflow || f->linearity_alarm)
		goto error;

	*angle_out = (360 * f->reading) / 1024;

	return EMS22A_OK;

	error:
		*angle_out = 0;
		return EMS22A_FAILURE;
}


ems22a_rc_t ems22a_update_instance(int i)
{
	CHECK_ID(i);
	instance *inst = instances[i];

	while (inst->lock) {;};
	/* wait on unlock */

	inst->lock = 1;

	check(inst->n_daisy_chain > 0 && inst->n_daisy_chain < EMS22A_MAX_DAISY_CHAIN,
							"wrong daisy chain number %d", inst->n_daisy_chain);


	int failed = 0;
	frame f;
	for (int j = 0; j < inst->n_daisy_chain; ++j) {
		int rc = inst->readf(i, (uint16_t *) &f);
		if (EMS22A_OK != rc) {
			log_err("failed reading instance %d, number %d in chain", i, j);
			failed++;
			continue;
		}

		ems22a_angle new_angle;
		rc = reading2angle(inst, &f, &new_angle);

		if (rc) {
			log_err("instance %s: read frame contains error flags: %d",
												inst->comment, frame2data(&f));
			failed++;
			continue;
		}
		else {
			inst->readings[j] = new_angle;
		}
	}

	inst->lock = 0;

	if (failed > 0) {
		log_err("instance %s: polling %d out of %d sensors had failed",
											inst->comment, failed, inst->n_daisy_chain);
		return EMS22A_WARNING;
	}

	return EMS22A_OK;

	error:
		inst->lock = 0;
		return EMS22A_FAILURE;
}
