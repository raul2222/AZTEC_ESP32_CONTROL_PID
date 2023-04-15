


void task_serial(void* arg)
{
  int32_t cnt_left= 0;
  int32_t cnt_right = 0;
  while (1) {
    cnt_left = ang_cnt;
    cnt_right = ang_cnt2;
    if(Serial.available() > 0){

       lastMotorCommand = millis();
       String str = Serial.readStringUntil('\r');
       Serial.println(str);
       if (str.indexOf("e") == 0 ) {
            Serial.print(cnt_left); 
            Serial.print(" "); 
            Serial.println(cnt_right);
            Serial.flush();
        }

        if (str.indexOf("u") == 0 ) {
            Serial.println("OK"); 
            Serial.flush();
        }
          
        if (str.indexOf("r") == 0 ) {
            ang_cnt=0;
            ang_cnt2=0;
            //reset contador encoder
            Serial.println("OK"); 
            Serial.flush();
        }
                
        if (str.indexOf("m") == 0 ) {
            //Serial.println(str);
            str.replace("m", "");
            int i1 = str.indexOf(" ");

            String firstValue = str.substring(0, i1);
            String second = str.substring(i1 + 1);
            setpoint = firstValue.toFloat();
            setpoint2 = second.toFloat();
            Serial.println("OK"); 
            Serial.flush();


        }
          

    }
    if (millis() > (AUTO_STOP_INTERVAL + lastMotorCommand) ){
          //setpoint = 0;
          //setpoint2 = 0;
    }
    vTaskDelay(5 / portTICK_PERIOD_MS);
    
  }
}


/* 
Tarea de configuración de parámetros  #####################################################################
*/
void task_config(void *pvParameter) {
  char ini_char = '0';

  while(1) { 
    // Detectar caracter enviado
    if(Serial.available() > 0){
        String str = Serial.readStringUntil('\n');
        ini_char = str[0];
        if(str.indexOf("V") == 0 or str.indexOf("v") == 0){
            str.replace("V","");str.replace("v","");
            /*
            pwm_volt = str.toFloat();
            Serial.print("Voltaje motor = ");
            Serial.println(pwm_volt, 2);
            //ref_val = pwm_volt; estabamos usando esta forma
            */
            //puesta_a_cero();
            volt_max = 11.99;
            setpoint=0;
            setpoint2=0;
            ACTIVA_P1C_MED_ANG = 0;ACTIVA_P1C_MED_ANG2 = 0;
            //init_eeprom();
            #ifdef ACTIVA_DEBUG
            Serial.print("----Modo Velocidad----");
            #endif
            
      }
      if (str.indexOf("R") == 0  or str.indexOf("r") == 0) {
            str.replace("R", "");str.replace("r", "");str.replace(",",".");

            //setpoint = str.toFloat();
            if( ACTIVA_P1C_MED_ANG == 1 ){
                setpoint =setpoint+str.toFloat();// + ang_cnt ;
                setpoint2 = setpoint2+str.toFloat();// + ang_cnt2;  
                #ifdef ACTIVA_DEBUG
                  Serial.print("Angulo= ");
                  Serial.print(setpoint, 2);
                  Serial.println("º");
                #endif
            }else{
                setpoint =str.toFloat() ;
                setpoint2 = str.toFloat();  
                
                #ifdef ACTIVA_DEBUG
                Serial.print("Velocidad= ");
                Serial.print(setpoint);
                Serial.println("rps");
                #endif
            }

         }
      if(str.indexOf("A") == 0 or str.indexOf("a") == 0){
            str.replace("A", "");str.replace("a", "");
            setpoint=setpoint2=0;
            ACTIVA_P1C_MED_ANG = 1;
            ACTIVA_P1C_MED_ANG2 = 1;
            volt_max = 2.7;
            Kp = Kp2=  0.08;
            Ki = Ki2= 0.15;
            Kd = Kd2 = 0.00111;
            setpoint =(ang_cnt * 360.0) / flancos; ;
                
            setpoint2 = (ang_cnt2 * 360.0) / flancos; ;  // GOL
            //ang_cnt=ang_cnt2=0;
            //clean();   //init_eeprom();
            #ifdef ACTIVA_DEBUG
            Serial.println("----Modo Angulo----");
            #endif
      }

      if(str.indexOf("Z") == 0 or str.indexOf("z") == 0){
            str.replace("Z", "");str.replace("z", "");
             start_stop = 1;
             #ifdef ACTIVA_DEBUG
             Serial.println("--START--");
             #endif
             
      }
      
      
      if(str.indexOf("S") == 0 or str.indexOf("s") == 0){
          str.replace("S", "");str.replace("s", "");
          if(str == "1"){
              start_stop = 1;
              
              #ifdef ACTIVA_DEBUG
              Serial.println("--START--");
              #endif
          }
          else{
              start_stop = 0;
              #ifdef ACTIVA_DEBUG
              Serial.println("--STOP--");
              #endif
          }
          clean();
      }
      if(str.indexOf("P") == 0 or str.indexOf("p") == 0  ){
          str.replace("P",""); str.replace("p","");str.replace(",",".");
          //K_p = str.toFloat();
          Kp = str.toFloat(); Kp2 = Kp;
          Serial.println(Kp);
         
          
      }
      if(str.indexOf("I") == 0 or str.indexOf("i") == 0){
          str.replace("I","");str.replace("i","");str.replace(",",".");
          //T_i = str.toFloat();
          Ki = Ki2 =  str.toFloat();  
          Serial.println(Ki);
         
        
      }
      if(str.indexOf("D") == 0 or str.indexOf("d") == 0){
          str.replace("D","");str.replace("d","");str.replace(",",".");
          //T_d = str.toFloat();
          Kd =Kd2 = str.toFloat();
          Serial.println(Kd);
          
      }


    }


    // Activacion de la tarea cada 0.1s
    vTaskDelay(150 / portTICK_PERIOD_MS);
  }
}
