
#define NOMBRE_PRAC "AZTEC_PID"
#define VERSION_SW "0.1"
#define ACTIVA_DEBUG
#include <stdio.h>
#include <stdlib.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "global.h"
#include "extras.h"
#include "keyinput.h"
#include "1-loopcontr.h"
#include "log.h"
#include "setup.h"

void task_loopcontr(void* arg) {

  while(1) {    
    if(start_stop == 1){
      Akpi=Kp+(Ki*dt);
      Akp = -Kp;
      A0d = Kd/dt;
      A1d = (-2.0)*(Kd/dt);
      A2d = Kd/dt;
      Kd == 0 ? tau = 0 : tau = Kd/(Kp*N); // IIR filter time constant  
      isinf(dt / (2*tau)) ? alpha = 0 : alpha = dt / (2*tau);

      if (ACTIVA_P1C_MED_ANG == 0){
        v_medida = (ang_cnt * 2.0 * PI) / flancos;
        da = v_medida - anterior;
        anterior = v_medida;
        v_medida = da / (BLOQUEO_TAREA_LOOPCONTR_MS / 1000.0); // rad/s
        //v_medida = v_medida / (2.0 * PI); // rps
      } else {
        v_medida = (ang_cnt * 360.0) / flancos;  // Calculo de angulo
        no_error_motor_break(); 
      }

      error_2 = error_1;
      error_1 = error_0;
      error_0 = setpoint - v_medida;
      // PI
      output = output+(Akpi*error_0)+(Akp*error_1);
      // Filtered D
      if(alpha !=0) {
        d1 = d0;
        d0 = (A0d * error_0) + (A1d * error_1) + (A2d * error_2);
        fd1 = fd0;
        fd0 = ((alpha) / (alpha + 1)) * (d0 + d1) - ((alpha - 1) / (alpha + 1)) * fd1;
        output = output + fd0;  
      }

      if (abs(output) > volt_max and output > 0) output = volt_max ; // min voltage value for dc-motor
      if (abs(output) > volt_max and output < 0) output = -volt_max ;

      if (ACTIVA_P1C_MED_ANG == 1) {
        no_error_motor_break();
      } else {
        if(setpoint == 0){
          output = 0; // output = 0 is brake
        } else {
          if (abs(output) < volt_min and setpoint > 0) output = volt_min ; 
          if (abs(output) < volt_min and setpoint < 0) output = -volt_min ;
        }
      }

      excita_motor(output);
    
    } else {
      //clean();
    }
      // delay 0.01s
      vTaskDelay(BLOQUEO_TAREA_LOOPCONTR_MS / portTICK_PERIOD_MS);
  }
}


void excita_motor(float v_motor){
    // Obtención de la dirección
    if(v_motor > 0){
        direccion = 1;
    }else{
        direccion = 0;
    }
    if(direccion_ant != direccion){  //("Cambio de sentido");
      v_motor = 0;
    }
    if(v_motor > 0){    
        digitalWrite(PWM_f, 0); // el pin de direccion
    }
    if(v_motor < 0){   
        v_motor = abs(v_motor); // valor en positivo 
        digitalWrite(PWM_f,1);
    }
    direccion_ant = direccion;
  	// Calcula y limita el valor de configuración del PWM
    dutyCycle = (int) ((v_motor * PWM_Max) / volt_max);
    // El valor de excitación debe estar entro 0 y PWM_Max
    if(dutyCycle >= PWM_Max){
        dutyCycle = PWM_Max;
    }
    if(dutyCycle <= 0){
        dutyCycle = 0;
    }
  	ledcWrite(0, dutyCycle);
}  


void task_enc(void* arg) {
  uint8_t r;
  uint8_t anterior;

  while(1){
    // Espera a leer los datos de la cola
    if (xQueueReceive( cola_enc , &r ,(TickType_t) portMAX_DELAY) == pdTRUE){
      // Codificar la fase del encoder
      if(r == 4){
        r = 3;
      }
      else if(r == 3){
        r = 4;
      }
      // Calcular incremento/decremento y actualizar valor 
      if(r > anterior){
        if(r == 4 && anterior == 1){
          ang_cnt--;
        }
        else{
          ang_cnt++;
        }
      }
      else{
        if(r == 1 && anterior ==4){
          ang_cnt++;
        }
        else{
          ang_cnt--;
        }
      }
      anterior = r;
    } else {
      printf("Error de lectura de la cola cola_enc \n");
    }
  
  }
}


void loop() {
/*
  delay(5000);
  K_p = 1.8;

  T_i = 0.4;
  //T_d = 0;
  //K_p = 1.9;
  //T_i = 0.7;
  T_d = 0.038;
  Tcb = 15;
  
  fd_bool = 1;
  alpha = 2;
  ACTIVA_P1C_MED_ANG = 0;
  ref_val = 4;
  start_stop = 1;
  delay(3600);
  //ref_val = 360;
  //delay(3600);
  start_stop = 0;
  delay(900000);
  */
/*

    if(Serial.available() > 0){

        lastMotorCommand = millis();
        String str = Serial.readStringUntil('\r');
        //Serial2.println(str);
        if (str.indexOf("e") == 0 ) {
            Serial.print(ang_cnt); 
            Serial.print(" "); 
            Serial.println(ang_cnt2);
            
        }
        if (str.indexOf("u") == 0 ) {
            Serial.println("OK"); 
           
        }
        if (str.indexOf("c") == 0 ) {
            ang_cnt=0;
            ang_cnt2=0;
            //reset contador encoder
            Serial.println("OK"); 
  
        } 
        if (str.indexOf("m") == 0 ) {
            str.replace("m", "");
            // ACTIVA_P1C_MED_ANG2 = 0;
            // ACTIVA_P1C_MED_ANG = 0;
            int i1 = str.indexOf(" ");
            String firstValue = str.substring(0, i1);
            if (firstValue != 0) ACTIVA_P1C_MED_ANG =0;
            String second = str.substring(i1 + 1);
            if (second != 0) ACTIVA_P1C_MED_ANG2 =0;
            setpoint = firstValue.toFloat();
            setpoint2 = second.toFloat();
            Serial.println("OK"); 

        }
        if (str.indexOf("r") == 0 ) {
            str.replace("r", "");
            if (ACTIVA_P1C_MED_ANG2 == 0){
              ACTIVA_P1C_MED_ANG2 = 1;
              ang_cnt2=0;
            }
            
            volt_max = 6.0;
            
            clean();  
            setpoint2 = str.toFloat();
        }
        if (str.indexOf("l") == 0 ) {
            str.replace("l", "");
            if (ACTIVA_P1C_MED_ANG == 0){
              ACTIVA_P1C_MED_ANG = 1;
              ang_cnt=0;
            }
            volt_max = 6.0;
            
            clean();   
            setpoint = str.toFloat();
        }

        if(str.indexOf("P") == 0 or str.indexOf("p") == 0  ){
            str.replace("P",""); str.replace("p","");str.replace(",",".");
            Kp =Kp2 = str.toFloat();
        }
       if(str.indexOf("I") == 0 or str.indexOf("i") == 0){
            str.replace("I","");str.replace("i","");str.replace(",",".");
            Ki = Ki2 =  str.toFloat();  
        }
        if(str.indexOf("D") == 0 or str.indexOf("d") == 0){
            str.replace("D","");str.replace("d","");str.replace(",",".");
            Kd =Kd2 = str.toFloat();  
        }            
        if(str.indexOf("N") == 0 or str.indexOf("n") == 0){
            str.replace("N","");str.replace("n","");str.replace(",",".");
            N =N2 = str.toFloat();  
        }           

    }
    delay(1);
    if (millis() > (AUTO_STOP_INTERVAL + lastMotorCommand) ){
          //setpoint = 0;
          //setpoint2 = 0;
    }


    */
}





