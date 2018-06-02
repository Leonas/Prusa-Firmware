s;
  float tmp_motor[3] = DEFAULT_PWM_MOTOR_CURRENT;
  float tmp_motor_loud[3] = DEFAULT_PWM_MOTOR_CURRENT_LOUD;}
  #ifdef TMC2130
  
  
  #ifdef TMC2130
              uint8_t tmc2130_current_r_bckp = tmc2130_current_r[E_AXIS];
              tmc2130_set_current_r(E_AXIS, TMC2130_UNLOAD_CURRENT_R);
  #else 

  			st_current_set(2, 200); //set lower E motor current for unload to protect filament sensor and ptfe tube
  			float tmp_motor[3] = DEFAULT_PWM_MOTOR_CURRENT;
  			float tmp_motor_loud[3] = DEFAULT_PWM_MOTOR_CURRENT_LOUD;

  #endif //TMC2130
  
  
  plan_buffer_line
  
  disable_e0();
  disable_e1();
  disable_e2();
  
