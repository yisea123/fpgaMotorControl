void zero_motor_axis(void);



void zero_motor_axis(void){
	int j, dir_bitmask, positive_pid_output=1;
	for(j = 0; j<8; j++){
		if(j < 4 ){
			alt_write_word(h2p_lw_pwm_values_addr[j], (25));
			
			dir_bitmask = alt_read_word(h2p_lw_gpio_addr);
			if(positive_pid_output)
				dir_bitmask |= (1<<(j+4));
			else
				dir_bitmask &= ~(1<<(j+4));
			//dir_bitmask = dir_bitmask
			//alt_write_word(h2p_lw_gpio_addr, (positive_pid_output * (0b00010000<<j)));
			//alt_write_word(h2p_lw_gpio_addr, (positive_pid_output * (1<<(j+4))));
			alt_write_word(h2p_lw_gpio_addr, dir_bitmask);
		}
		else{
			if (j < 8){

			alt_write_word(h2p_lw_pwm_values_addr[j], (25));
			dir_bitmask = alt_read_word(h2p_lw_gpio_addr);
			if(positive_pid_output == 0)
				dir_bitmask |= (1<<(j-4));
			else
				dir_bitmask &= ~(1<<(j-4));
			alt_write_word(h2p_lw_gpio_addr, dir_bitmask);
			}
		}
	}
}