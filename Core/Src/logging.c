/*
 * logging.c
 *
 *  Created on: Jul 31, 2022
 *      Author: guanlunlu
 */
#include "logging.h"

float random_input[] = {10.8284,10.5706,11.5183,11.5189,11.902,11.0753,11.2791,11.7464,10.5519,10.5802,11.2946,11.5067,10.5115,11.0751,10.6003,11.1262,11.5302,11.3835,11.8957,11.7693,11.2904,10.6379,11.4809,11.124,11.5518,11.8655,11.6433,10.8937,10.5712,11.6041,10.9924,11.449,11.6346,11.9866,11.048,10.8706,11.9738,11.584,11.63,11.4773,10.609,11.4475,11.8271,10.9091,11.1546,11.6497,11.2166,10.8567,10.9124,11.0389,10.7498,11.2298,11.8465,11.8638,10.5908,11.857,11.2568,11.2744,10.9785,11.98,11.241,10.8992,10.6361,11.9216,10.6106,11.2511,11.0762,10.9156,11.8707,11.2946,11.1967,11.9115,10.5751,11.6423,11.6553,11.7417,10.688,10.5238,11.5327,11.8024,11.4443,11.6043,11.5881,11.9992,11.8329,10.8498,10.9595,11.0265,11.2699,11.3867,11.769,11.1181,11.7623,10.904,11.1231,11.306,11.2019,10.9308,10.7675,10.7306,11.3575,11.7036,10.5496,11.3017,11.2477,11.933,11.6224,11.3319,11.8361,11.4373,11.7631,10.7397,10.8191,11.5721,10.6956,10.6365,10.9119,10.5045,11.1214,10.5403,11.5647,11.9068,10.8599,10.7713,10.9763,11.8305,11.4781,10.7255,11.522,11.0787,11.0816,11.2496,10.7213,11.3808,11.7684,11.3852,11.9331,11.3342,10.7222,11.975,11.1132,10.7127,11.3473,10.8782,11.2328,11.196,11.9416,10.689,10.7996,10.9789,11.4439,10.6901,11.4769,11.4325,11.7046,10.8718,11.2146,11.084,10.8049,10.5426,11.8525,11.1397,10.713,11.9212,11.1155,10.6968,11.8285,10.6383,10.7433,10.6066,11.048,10.8796,10.7027,11.6747,11.183,11.0243,11.1785,11.7134,11.8975,11.4775,10.8229,11.5194,11.8634,10.8752,11.7913,11.2069,11.2589,11.4006,11.7263,11.6338,11.1934,11.9271,11.4491,11.159,11.737,11.5335,11.5533,11.9807,11.9316,11.7769,10.934,11.3061,11.2717,10.6552,11.121,11.3651,11.8148,11.1601,11.5946,11.8039,11.5735,11.7011,11.5598,11.6126,10.5286,11.829,11.2875,11.195,10.5978,11.5701,11.2334,11.5015,11.5231,10.7993,11.875,11.7988,11.835,11.3159,10.7088,11.1755,11.984,10.8233,11.169,10.9736,11.272,11.8223,11.1596,11.2013,11.71,11.0477,10.8173,11.9987,10.7304,11.4457,11.4245,10.5009,10.5013,11.66,11.591,10.9788,11.1266,11.5237,11.5208,10.8079,11.7546,11.5634,11.7431,10.6418,10.6226,11.646,11.4444,10.8208,10.8203,10.6216,11.0832,11.9282,11.9213,11.0848,10.9038,11.5383,10.9261,11.6653,11.6758,11.1337,10.9232,10.791,10.517,10.7877,11.9749,10.8661,11.7296,10.7047,11.0972,11.4015,10.7653,11.7425,10.7366,11.9819,10.8858,10.8504,10.6525,10.8291,11.4521,11.544,11.6922,11.5444,11.6294,11.5043,11.4501,10.5847,11.3973,10.8405,10.9782,11.5498,10.6762,11.6439,11.2892,11.3309,11.382,10.9945,11.5545,10.7146,10.7425,11.228,11.7903,11.7199,11.3353,11.6085,10.974,10.703,11.2928,10.9661,11.3822,11.2771,11.1463,10.8883,11.0553,11.0895,11.1703,11.2138,11.0817,10.9189,10.6174,11.0546,10.8809,11.5077,11.5144,11.2709,11.5929,11.5812,11.9171,11.191,11.9102,10.9823,11.1907,11.2757,11.492,11.1027,11.4085,11.9798,10.7256,11.5051,11.0157,11.3126,11.2842,11.7439,10.5224,10.9113,11.4645,11.3192,11.8768,10.8999,11.9551,10.8701,11.766,11.541,11.1836,11.7257,10.5331,10.7411,11.5604,11.5617,11.155,11.3736,11.6276,11.9873,11.5435,10.9193,11.6308,10.7568,10.5064,11.2435,10.6199,10.7446,11.855,11.6745,11.62,11.1546,10.5095,11.3772,11.2777,10.6003,11.316,11.0846,10.8237,11.0787,11.4403,11.4886,11.564,11.308,10.9204};
int random_input_size = 400;

int _write(int file, char *ptr, int len){
	for(int i = 0; i < len; i ++){
		ITM_SendChar((*ptr++));
	}
	return len;
}
void logger_setup(Logger *logger){
	logger->time_stamp = 0;
	logger->sample_size = sizeof(random_input)/sizeof(float);
	logger->sample_iter_idx = 0;
	logger->finished_logging = 0;
}

void log_motor_state(motor *motor, float time_stamp){
	printf("%f, %f, %f\n", time_stamp, motor->driver.pwm_duty, motor->angular_vel);
}

float get_logger_input(Logger *logger){
	float input_val = random_input[logger->sample_iter_idx];

	if(logger->sample_iter_idx == logger->sample_size -1){
		logger->finished_logging = 1;
	}
	logger->sample_iter_idx ++;
	return input_val;
}
