/*
 * logging.h
 *
 *  Created on: Jul 31, 2022
 *      Author: guanlunlu
 */

#ifndef INC_LOGGING_H_
#define INC_LOGGING_H_

#include <motor.h>

typedef struct logger{
	float time_stamp;
	int sample_size;
	int sample_iter_idx;
	int finished_logging;
}Logger;

void logger_setup(Logger *logger);

float get_logger_input(Logger *logger);

void log_motor_state(motor *motor, float time_stamp);


#endif /* INC_LOGGING_H_ */
