/*
  Tarea imprimir por terminal del controlador  #####################################################################
*/

void task_medidas(void* arg)
{

  while (1) {
    if (start_stop == 1) {
      // Mostrar medidas de angulo y velocidad del motor
      Serial.print("C: ");
      Serial.print(ang_cnt);
      Serial.print(", M:");
      Serial.print(v_medida);
      Serial.print(", R:");
      Serial.print(setpoint, 2);
      Serial.print(", V: ");
      Serial.print(output);
      Serial.print(", D: ");
      Serial.print(dutyCycle);

      Serial.print(" ");
      Serial.print(", C: ");
      Serial.print(ang_cnt2);
      Serial.print(", M:");
      Serial.print(v_medida2);
      Serial.print(", R:");
      Serial.print(setpoint2, 2);
      Serial.print(", V: ");
      Serial.print(output2);
      Serial.print(", D: ");
      Serial.print(dutyCycle2);

      Serial.print("   ");
      Serial.print(" Kp:");
      Serial.print(Kp, 5);
      Serial.print(" Ki:");
      Serial.print(Ki, 5);
      Serial.print(" Kd:");
      Serial.print(Kd, 5);


  /*    if ( ACTIVA_P1C_MED_ANG == 1 ) { // Medida de angulo
        a_medida = (ang_cnt * 360) / flancos;
        Serial.print(", M:");
        Serial.print(a_medida);
        Serial.print(" A:");
        Serial.print(setpoint, 2);

      } else { // Medida de velocidad
          Serial.print(", Med:");
          Serial.print(v_medida);
          Serial.print(", Ref:");
          Serial.print(setpoint, 2);

      }

      Serial.print(", V:");
      Serial.print(output, 2);
      Serial.print(", D: ");
      Serial.print(dutyCycle);


      Serial.print(",// C: ");
      Serial.print(ang_cnt2);

      if ( ACTIVA_P1C_MED_ANG2 == 1 ) { // Medida de angulo
        a_medida2 = (ang_cnt2 * 360) / flancos;
        Serial.print(", M:");
        Serial.print(a_medida2);
        Serial.print(" A:");
        Serial.print(setpoint2, 2);

      } else { // Medida de velocidad
          Serial.print(", Med:");
          Serial.print(v_medida2);
          Serial.print(", Ref:");
          Serial.print(setpoint2, 2);

      }

      Serial.print(", V:");
      Serial.print(output2, 2);
      Serial.print(", D: ");
      Serial.print(dutyCycle2);

      


      /*Serial.print(", A:");
      Serial.print(current,4);        // Print Raw Reading
      Serial.print(", W:");
      Serial.print(output*current); */
      
  //Serial.print(',');                // Comma Separator
  //Serial.print(AN_Pot1_Filtered); // Print Filtered Output

      

      
/*
#ifdef ACTIVA_DEBUG
      Serial.print(" Kp:");
      Serial.print(Kp, 6);
            Serial.print(" Ki:");
      Serial.print(Ki, 6);
            Serial.print(" Kd:");
      Serial.print(Kd, 6);
            Serial.print(" error_0:");
      Serial.print(error_0, 3);
            Serial.print(" z_d:");
      Serial.print(z_d, 3);

      Serial.print(" tau:");
      Serial.print(tau, 2);
      Serial.print(" alfa:");
      Serial.print(alfa, 2);
      Serial.print(" m:");
      Serial.print(measuredvalue, 2);
      Serial.print(" AKpx:");
      Serial.print(AKpx, 2);
      Serial.print(" A1:");

      
      Serial.print(A1, 2);
      Serial.print(" e2:");
      Serial.print(errordos, 2);
      Serial.print(" e1:");
      Serial.print(erroruno, 2);
      Serial.print(" e0:");
      Serial.print(errorcero, 2);
      
            Serial.print(" A0d:");
      Serial.print(A0d, 2);
      Serial.print(" A1d:");
      Serial.print(A1d, 2);
      Serial.print(" A2d:");
      Serial.print(A2d, 2);
      


#endif*/

      Serial.println(" ");

    } else {

    }

    // Activacion de la tarea cada 1s
    vTaskDelay(BLOQUEO_TAREA_MEDIDA_MS / portTICK_PERIOD_MS);
  }
}


void task_adc(void* arg)
{
  
  while (1) {
    /*
    if (start_stop == 1) {
        

        adc0 = ads.readADC_SingleEnded(0);
        //current = (adc0 * 0.1875)/1000;
        current = ads.computeVolts(adc0)/0.5;

    } else {

    }
*/
    // Activacion de la tarea cada 1s
    vTaskDelay(BLOQUEO_TAREA_LOOPCONTR_MS / portTICK_PERIOD_MS);
  }
}
