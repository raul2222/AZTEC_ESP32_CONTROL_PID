
void no_error_motor_break(){
      if(abs(error_0) <= 1.5){
            output = 0;
            
     } 
}

void no_error_motor_break2(){
      if(abs(error_02) <= 1.5){
            output2 = 0;
            
     } 
}

void clean(){
  excita_motor(0);
  output = 0;
}

void clean2(){
  excita_motor2(0);
  output2 = 0;
}



