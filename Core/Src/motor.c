# include <motor.h>

void motor_hardware_setup(motor *motor){
	// L298N motor driver setup
    // ENA IN2 (GPIO) = PA9 PC7; IN1 IN2(PWM) = PB6(tim4_ch1), PC7(tim8_ch2)
	motor->driver.ENA_port = GPIOA;
    motor->driver.ENA_pin = GPIO_PIN_9;

	motor->driver.IN1_pwm_timer = &htim4;
	motor->driver.IN1_pwm_channel = TIM_CHANNEL_1;
	motor->driver.IN1_pwm_counter = 0;
	
	motor->driver.IN2_pwm_timer = &htim8;
	motor->driver.IN2_pwm_channel = TIM_CHANNEL_2;
	motor->driver.IN2_pwm_counter = 0;

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
	motor->controller.p_gain = 0.17070489113648;
	motor->controller.i_gain = 8.64438286598949;
//	motor->controller.p_gain = 0.68;
//	motor->controller.i_gain = 21.65;
//	motor->controller.p_gain = 0.307;
//	motor->controller.i_gain = 12.59;
	motor->controller.d_gain = 0;

    HAL_TIM_Encoder_Start_IT(motor->encoder.enc_timer, TIM_CHANNEL_ALL);
}

void motor_driver_initialize(motor *motor){
	HAL_TIM_PWM_Start(motor->driver.IN1_pwm_timer, motor->driver.IN1_pwm_channel);
	HAL_TIM_PWM_Start(motor->driver.IN2_pwm_timer, motor->driver.IN2_pwm_channel);
	// initialize as slow decay mode
	HAL_GPIO_WritePin(motor->driver.ENA_port, motor->driver.ENA_pin, GPIO_PIN_SET);
	__HAL_TIM_SET_COMPARE(motor->driver.IN1_pwm_timer, motor->driver.IN1_pwm_channel,0);
	__HAL_TIM_SET_COMPARE(motor->driver.IN2_pwm_timer, motor->driver.IN2_pwm_channel,0);
}

void motor_duty_output(motor *motor, float duty_cycle){

	if (duty_cycle >= 0){
		int timer_counter_period = motor->driver.IN1_pwm_timer->Instance->ARR + 1;
		motor->driver.IN1_pwm_counter = (float)timer_counter_period * duty_cycle;
		motor->driver.pwm_duty = duty_cycle;
		__HAL_TIM_SET_COMPARE(motor->driver.IN1_pwm_timer, motor->driver.IN1_pwm_channel,motor->driver.IN1_pwm_counter);
		__HAL_TIM_SET_COMPARE(motor->driver.IN2_pwm_timer, motor->driver.IN2_pwm_channel,0);
	}
	else {
		duty_cycle = fabs(duty_cycle);
		int timer_counter_period = motor->driver.IN2_pwm_timer->Instance->ARR + 1;
		motor->driver.IN2_pwm_counter = (float)timer_counter_period * duty_cycle;
		motor->driver.pwm_duty = duty_cycle;
		__HAL_TIM_SET_COMPARE(motor->driver.IN1_pwm_timer, motor->driver.IN1_pwm_channel,0);
		__HAL_TIM_SET_COMPARE(motor->driver.IN2_pwm_timer, motor->driver.IN2_pwm_channel,motor->driver.IN2_pwm_counter);
	}
}

void motor_encoder_update(motor *motor){
    double rad_per_cnt = ((2*M_PI) / (motor->encoder.count_per_rev * motor->encoder.reduction_ratio * 4));

    motor->encoder.current_counter = motor->encoder.enc_timer->Instance->CNT;
    motor->encoder.delta_counter = (double)motor->encoder.current_counter - motor->encoder.previous_counter;


	if (motor->encoder.delta_counter < -32768)   //32768 = 65535 / 2
		motor->angular_vel = (65536 + motor->encoder.delta_counter) * rad_per_cnt * motor->controller.control_frequency;
	else if (motor->encoder.delta_counter > 32768)
		motor->angular_vel = (motor->encoder.delta_counter - 65536) * rad_per_cnt * motor->controller.control_frequency;
	else
		motor->angular_vel = motor->encoder.delta_counter * rad_per_cnt * motor->controller.control_frequency;


    motor->encoder.previous_counter = motor->encoder.current_counter;
}

void motor_command_tracking(motor *motor){
	float output_duty;
	motor->controller.tracking_error = motor->controller.command_vel - motor->angular_vel;
	motor->controller.tracking_error_sum += motor->controller.tracking_error / (float)motor->controller.control_frequency;

	output_duty = motor->controller.p_gain * motor->controller.tracking_error + motor->controller.i_gain * motor->controller.tracking_error_sum;

	if (output_duty >= 1)
		output_duty = 1;
	else if (output_duty <= -1)
		output_duty = -1;

//	if (fabs(motor->controller.tracking_error) > motor->controller.tracking_tolerance){
//		motor->controller.tracking_error_sum += motor->controller.tracking_error / motor->controller.control_frequency;
//
////		if (motor->controller.tracking_error_sum > 10)
////			motor->controller.tracking_error_sum = 10;
////		if (motor->controller.tracking_error_sum < -10)
////			motor->controller.tracking_error_sum = -10;
//
//		output_duty = motor->controller.p_gain * motor->controller.tracking_error + motor->controller.i_gain * motor->controller.tracking_error_sum;
//		if (output_duty >= 1)
//			output_duty = 1;
//		else if (output_duty <= -1)
//			output_duty = -1;
//	}
//	else{
//		output_duty = motor->driver.pwm_duty;
//	}

	motor_duty_output(motor, output_duty);
}

float voltage_to_dutycycle(motor *motor, float voltage){
	return voltage / motor->driver.max_voltage;
}
