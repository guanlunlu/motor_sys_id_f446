# include <motor.h>

void motor_hardware_setup(motor *motor){
	// L298N motor driver setup
    // EN IN1 IN2 = PA8(PWM) PA9 PA10
	motor->driver.IN1_port = GPIOA;
    motor->driver.IN1_pin = GPIO_PIN_9;
	motor->driver.IN2_port = GPIOC;
    motor->driver.IN2_pin = GPIO_PIN_7;

	motor->driver.pwm_timer = &htim4;
	motor->driver.pwm_channel = TIM_CHANNEL_1;
	motor->driver.pwm_counter = 0;
	motor->driver.pwm_duty = 0;
	motor->driver.max_voltage = 12.0;

    // encoder parameter setup
    motor->encoder.enc_timer = &htim3;
    motor->encoder.current_counter = 0;
    motor->encoder.previous_counter = 0;
    motor->encoder.count_per_rev = 500;
    motor->encoder.reduction_ratio = 16;

	// controller parameter setup
    motor->controller.control_frequency = 1000;
	motor->controller.command_vel = 0;
	motor->controller.tracking_error = 0;
	motor->controller.tracking_tolerance = 0.08;
//	motor->controller.p_gain = 1.52;
//	motor->controller.i_gain = 75.46;
	motor->controller.p_gain = 0.68;
	motor->controller.i_gain = 21.65;
	motor->controller.d_gain = 0;

    HAL_TIM_Encoder_Start_IT(motor->encoder.enc_timer, TIM_CHANNEL_ALL);
}

void motor_driver_initialize(motor *motor){
	HAL_TIM_PWM_Start(motor->driver.pwm_timer, motor->driver.pwm_channel);
	// initialize as slow decay mode
	HAL_GPIO_WritePin(motor->driver.IN1_port, motor->driver.IN1_pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(motor->driver.IN2_port, motor->driver.IN2_pin, GPIO_PIN_RESET);
	__HAL_TIM_SET_COMPARE(motor->driver.pwm_timer, motor->driver.pwm_channel,0);
}

void motor_duty_output(motor *motor, float duty_cycle, int dir){
	if (dir >= 0){
		HAL_GPIO_WritePin(motor->driver.IN1_port, motor->driver.IN1_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(motor->driver.IN2_port, motor->driver.IN2_pin, GPIO_PIN_SET);
	}
	else {
		HAL_GPIO_WritePin(motor->driver.IN1_port, motor->driver.IN1_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(motor->driver.IN2_port, motor->driver.IN2_pin, GPIO_PIN_RESET);
	}
	int timer_counter_period = motor->driver.pwm_timer->Instance->ARR + 1;
	motor->driver.pwm_counter = (float)timer_counter_period * duty_cycle;
    motor->driver.pwm_duty = duty_cycle;
	__HAL_TIM_SET_COMPARE(motor->driver.pwm_timer, motor->driver.pwm_channel, motor->driver.pwm_counter);
}

void motor_encoder_update(motor *motor){
    double rad_per_cnt = ((2*M_PI) / (motor->encoder.count_per_rev * motor->encoder.reduction_ratio * 4));

    motor->encoder.current_counter = motor->encoder.enc_timer->Instance->CNT;
    motor->encoder.delta_counter = motor->encoder.current_counter - motor->encoder.previous_counter;

    motor->angular_vel = motor->encoder.delta_counter * rad_per_cnt * motor->controller.control_frequency;
    motor->encoder.previous_counter = motor->encoder.current_counter;
}

void motor_command_tracking(motor *motor){
	float output_duty;
	motor->controller.tracking_error =motor->controller.command_vel - motor->angular_vel;
	if (fabs(motor->controller.tracking_error) > motor->controller.tracking_tolerance){
		motor->controller.tracking_error_sum += motor->controller.tracking_error;
		if (motor->controller.tracking_error_sum > 1)
			motor->controller.tracking_error_sum = 1;

		output_duty = motor->controller.p_gain * motor->controller.tracking_error + motor->controller.i_gain * motor->controller.tracking_error_sum;
		if (output_duty >= 1)
			output_duty = 1;
	//	float output_duty = voltage_to_dutycycle(motor, output_voltage);
	}
	else{
		output_duty = motor->driver.pwm_duty;
	}
	motor_duty_output(motor, output_duty, output_duty);
}

float voltage_to_dutycycle(motor *motor, float voltage){
	return voltage / motor->driver.max_voltage;
}
