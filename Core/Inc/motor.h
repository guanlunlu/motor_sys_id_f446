#ifndef __robot_H
#define __robot_H

# include <main.h>
# include <math.h>

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim8;
extern float random_input[];


typedef struct driver_hardware{
    GPIO_TypeDef * ENA_port;
    uint16_t ENA_pin;

    GPIO_TypeDef * IN2_port;
    uint16_t IN2_pin;

    GPIO_TypeDef * EN_port;
    uint16_t EN_pin;

    TIM_HandleTypeDef * IN1_pwm_timer;
    unsigned int IN1_pwm_channel;
    float IN1_pwm_counter;

    TIM_HandleTypeDef * IN2_pwm_timer;
    unsigned int IN2_pwm_channel;
    float IN2_pwm_counter;
    float pwm_duty;

    float max_voltage;
}Driver;

typedef struct encoder_param{
    int count_per_rev;
    int reduction_ratio;

    int previous_counter;
    int current_counter;
    int delta_counter;

    TIM_HandleTypeDef * enc_timer;
}Encoder;

typedef struct controller_param{
    int control_frequency;

    double command_vel;
    double tracking_error;
    double tracking_error_sum;
    double tracking_tolerance;

    double p_gain;
    double i_gain;
    double d_gain;
}Controller;

typedef struct motor{
	Driver driver;
	Encoder encoder;
    Controller controller;
    
	double angular_vel;
}motor;

void motor_hardware_setup(motor *motor);

void motor_encoder_update(motor *motor);

void motor_driver_initialize(motor *motor);

void motor_duty_output(motor *motor, float duty_cycle);

void motor_command_tracking(motor *motor);

float voltage_to_dutycycle(motor *motor, float voltage);



#endif
