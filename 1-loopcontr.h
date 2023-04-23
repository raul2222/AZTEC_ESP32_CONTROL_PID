
void task_loopcontr2(void* arg) {

  while(1) {    
    
      if(start_stop2 == 1){

          Akpi2=Kp2+(Ki2*dt2);
          Akp2 = -Kp2;
          A0d2 = Kd2/dt2;
          A1d2 = (-2.0)*(Kd2/dt2);
          A2d2 = Kd2/dt2;
          Kd2 == 0 ? tau2 = 0 : tau2 = Kd2/(Kp2*N2); // IIR filter time constant  
          isinf(dt2 / (2*tau2)) ? alpha2 = 0 : alpha2 = dt / (2*tau2);

          if (ACTIVA_P1C_MED_ANG2 == 0){
            // Calculo rps
            v_medida2 = (ang_cnt2 * 2.0 * PI) / flancos;
            da2 = v_medida2 - anterior2;
            anterior2 = v_medida2;
            v_medida2 = da2 / (BLOQUEO_TAREA_LOOPCONTR_MS / 1000.0); // rad/s
            //v_medida2 = v_medida2 / (2.0 * PI); // rps
          } else {
             v_medida2 = (ang_cnt2 * 360.0) / flancos;  // Calculo de angulo
             no_error_motor_break2(); 
          }

          error_22 = error_12;
          error_12 = error_02;
          error_02 = setpoint2 - v_medida2;
          // PI
          output2 = output2+(Akpi2*error_02)+(Akp2*error_12);
          // Filtered D
          if(alpha2 !=0) {
            d12 = d02;
            d02 = (A0d2 * error_02) + (A1d2 * error_12) + (A2d2 * error_22);
            fd12 = fd02;
            fd02 = ((alpha2) / (alpha2 + 1)) * (d02 + d12) - ((alpha2 - 1) / (alpha2 + 1)) * fd12;
            output2 = output2 + fd02;  
          }

          if (abs(output2) > volt_max and output2 > 0) output2 = volt_max ; 
          if (abs(output2) > volt_max and output2 < 0) output2 = -volt_max ;

          if (ACTIVA_P1C_MED_ANG2 == 1) {
            no_error_motor_break2();
          } else {
            if(setpoint2 == 0){
              output2 = 0;
            } else {
              if (abs(output2) < volt_min and setpoint2 > 0) output2 = volt_min; 
              if (abs(output2) < volt_min and setpoint2 < 0) output2 = -volt_min;
            }
          }

          excita_motor2(output2);
          
      } else {
          //clean();
      }
      // Activacion de la tarea cada 0.01s
      vTaskDelay(BLOQUEO_TAREA_LOOPCONTR_MS / portTICK_PERIOD_MS);
  }
}

void IRAM_ATTR ISR_enc2() {
  // Lee las salidas del Encoder    
  uint8_t a2 = digitalRead(A_enc_pin2);
  uint8_t b2 = digitalRead(B_enc_pin2);
  uint8_t r2;
  
  if(a2 == 0 && b2 == 0){
      r2 = 1;
  }
  else if(a2 == 0 && b2 == 1){
      r2 = 2;
  }
  else if(a2 == 1 && b2 == 0){
      r2 = 3;
  }
  else if(a2 == 1 && b2 == 1){
      r2 = 4;
  }
  // Enviar los bytes a la cola 
  if (xQueueSendFromISR( cola_enc2 , &r2 ,NULL) != pdTRUE)
  {
      printf("Error de escritura en la cola cola_enc2 \n");
  }
}

void excita_motor2(float v_motor){
    // Obtención de la dirección
    if(v_motor > 0){
        direccion2 = 1;
    }else{
        direccion2 = 0;
    }
    if(direccion_ant2 != direccion2){  //("Cambio de sentido");
      v_motor = 0;
    }
    if(v_motor > 0){    //Serial.println("Hacia adelante");
        digitalWrite(PWM_f2, 0); // el pin de direccion
    }
    if(v_motor < 0){    //("Hacia atras");
        v_motor = abs(v_motor); // valor en positivo del voltaje el cambio de direccion lo hacen las variables
        digitalWrite(PWM_f2,1);
    }
    direccion_ant2 = direccion2;
  	// Calcula y limita el valor de configuración del PWM
    dutyCycle2 = (int) ((v_motor * PWM_Max)/volt_max);
    // El valor de excitación debe estar entro 0 y PWM_Max
    if(dutyCycle2 >= PWM_Max){
        dutyCycle2 = PWM_Max;
    }
    if(dutyCycle2 <= 0){
        dutyCycle2 = 0;
    }
  	ledcWrite(1, dutyCycle2);
}  


/*
 Tarea task_enc #####################################################################
*/

void task_enc2(void* arg) {
  uint8_t r2 ;
  uint8_t anterior_enc ;
  while(1){
    // Espera a leer los datos de la cola
    if (xQueueReceive( cola_enc2 , &r2 ,(TickType_t) portMAX_DELAY) == pdTRUE){
  if(r2 == 4){
        r2 = 3;
      }
      else if(r2 == 3){
        r2 = 4;
      }
      // Calcular incremento/decremento y actualizar valor 
      if(r2 > anterior_enc){
        if(r2 == 4 && anterior_enc == 1){
          ang_cnt2--;
        }
        else{
          ang_cnt2++;
        }
      }
      else{
        if(r2 == 1 && anterior_enc ==4){
          ang_cnt2++;
        }
        else{
          ang_cnt2--;
        }
      }
      anterior_enc = r2;
    } else {
        printf("Error de lectura de la cola cola_enc \n");
    }
  }
}
