
/*
SET UP -----------------------------------------------------------------------------------
*/
void setup() {
  // Configuracion puerto serie
  config_sp();
  digitalRead(A_enc_pin);

  // Configuracion PWM
  config_PWM();

  // Configuracion PWM
  //config_ADC();

  // Crear cola_enc
  cola_enc2 = xQueueCreate(TAM_COLA_I, TAM_MSG_I);
  if(cola_enc2 == NULL){
      Serial.println("Error en creacion de cola_enc2");
      exit(-1);
  }

  cola_enc = xQueueCreate(TAM_COLA_I, TAM_MSG_I);
  if(cola_enc == NULL){
      Serial.println("Error en creacion de cola_enc");
      exit(-1);
  }

   //Crear la tarea task_enc
  if(xTaskCreatePinnedToCore( task_enc , "task_enc", 4096, NULL, 3, NULL,1) != pdPASS){
      Serial.println("Error en creacion tarea task_enc");
      exit(-1);
  }

  if(xTaskCreatePinnedToCore( task_enc2 , "task_enc2", 4096, NULL, 3, NULL,0) != pdPASS){
      Serial.println("Error en creacion tarea task_enc");
      exit(-1);
  }

  // Crear la tarea task_config
  if(xTaskCreatePinnedToCore( task_config , "task_config", 2048, NULL, 1, NULL,0) != pdPASS){
      Serial.println("Error en creacion tarea task_config");
      exit(-1);
  }

  // Crear la tarea task_loopcontr
  if(xTaskCreatePinnedToCore( task_loopcontr , "task_loopcontr", 4096, NULL, 2, NULL,1) != pdPASS){
      Serial.println("Error en creacion tarea task_loopcontr");
      exit(-1);
  }

  if(xTaskCreatePinnedToCore( task_loopcontr2 , "task_loopcontr2", 4096, NULL, 2, NULL,0) != pdPASS){
      Serial.println("Error en creacion tarea task_loopcontr");
      exit(-1);
  }

/*
  if(xTaskCreatePinnedToCore( task_adc , "task_adc", 2048, NULL, 3, NULL,0) != pdPASS){
      Serial.println("Error en creacion tarea task_medidas");
      exit(-1);
  }
*/

   // Crear la tarea task_medidas
  if(xTaskCreatePinnedToCore( task_medidas , "task_medidas", 2048, NULL,1, NULL,0) != pdPASS){
      Serial.println("Error en creacion tarea task_medidas");
      exit(-1);
  }

 /*if(xTaskCreatePinnedToCore( task_serial , "task_serial", 1024, NULL, 5, NULL, 1) != pdPASS){
      Serial.println("Error en creacion tarea task_medidas");
      exit(-1);
 }*/


  // Configuracion del encoder
  config_enc();
  flancos = 1200.0;

  dt = (BLOQUEO_TAREA_LOOPCONTR_MS / 1000.0);
  dt2 = (BLOQUEO_TAREA_LOOPCONTR_MS / 1000.0);
  Kp = Kp2=  0.05;
  Ki = Ki2= 0.3;
  Kd = Kd2 = 0.00033;
  N = N2 = 4;
  
  
  ACTIVA_P1C_MED_ANG2 =ACTIVA_P1C_MED_ANG= 0;

  setpoint = 0.0; // LEFT
  setpoint2 = 0;  // 
  start_stop=1;
  start_stop2=1;

  pinMode(PWM_en, OUTPUT); 
  digitalWrite(PWM_en,1);
  pinMode(PWM_en2, OUTPUT); 
  digitalWrite(PWM_en2,1);
}

/*
 RUTINAS ATENCION INTERRUPCIONES ########################################################################
*/
/* 
 Rutina de atención a interrupción ISC_enc --------------------------------------------
*/
void IRAM_ATTR ISR_enc() {
  // Lee las salidas del Encoder    
  uint8_t a = digitalRead(A_enc_pin);
  uint8_t b = digitalRead(B_enc_pin);
  uint8_t r;
  // Procesa los datos    // r=2*a+b;
  if(a == 0 && b == 0){
      r = 1;
  }
  else if(a == 0 && b == 1){
      r = 2;
  }
  else if(a == 1 && b == 0){
      r = 3;
  }
  else if(a == 1 && b == 1){
      r = 4;
  }
  // Enviar los bytes a la cola 
  if (xQueueSendFromISR( cola_enc , &r ,NULL) != pdTRUE)
  {
      printf("Error de escritura en la cola cola_enc \n");
  }
}

void IRAM_ATTR ISR_enc2() {
  // Lee las salidas del Encoder    
  uint8_t a = digitalRead(A_enc_pin2);
  uint8_t b = digitalRead(B_enc_pin2);
  uint8_t r2;
  
  if(a == 0 && b == 0){
      r2 = 1;
  }
  else if(a == 0 && b == 1){
      r2 = 2;
  }
  else if(a == 1 && b == 0){
      r2 = 3;
  }
  else if(a == 1 && b == 1){
      r2 = 4;
  }
  // Enviar los bytes a la cola 
  if (xQueueSendFromISR( cola_enc2 , &r2 ,NULL) != pdTRUE)
  {
      printf("Error de escritura en la cola cola_enc \n");
  }
}

////////////////////////////////////////////////////////////////////////////////////
// Funcion configuracion del adc
////////////////////////////////////////////////////////////////////////////////////

void config_ADC(){
  Serial.println("Getting single-ended readings from AIN0..3");
  Serial.println("ADC Range: +/- 6.144V (1 bit = 3mV/ADS1015, 0.1875mV/ADS1115)");
  /*if (!ads.begin()) {
    Serial.println("Failed to initialize ADS.");
    while (1);
  }*/
}  

////////////////////////////////////////////////////////////////////////////////////
// Funcion configuracion del encoder
////////////////////////////////////////////////////////////////////////////////////

void config_enc(){
    // Configuracion de pines del encoder
    pinMode(A_enc_pin, INPUT);
    pinMode(B_enc_pin, INPUT);
    pinMode(A_enc_pin2, INPUT);
    pinMode(B_enc_pin2, INPUT);
    // Configuracion interrupcion
    attachInterrupt(A_enc_pin, ISR_enc, CHANGE);
    attachInterrupt(B_enc_pin, ISR_enc, CHANGE);
    attachInterrupt(A_enc_pin2, ISR_enc2, CHANGE);
    attachInterrupt(B_enc_pin2, ISR_enc2, CHANGE);
} 

////////////////////////////////////////////////////////////////////////////////////
// Funcion configuracion del PWM
////////////////////////////////////////////////////////////////////////////////////

void config_PWM(){
    // Configuracion de pines de control PWM
    pinMode(PWM_f, OUTPUT); digitalWrite(PWM_f,0);
    pinMode(PWM_f2, OUTPUT); digitalWrite(PWM_f2,0);
    // Configuracion LED PWM 
    ledcSetup(pwmChannel, pwmfreq, pwmresolution);
    ledcSetup(pwmChannel2, pwmfreq, pwmresolution);
    // Asignar el controlador PWM al GPIO
    ledcAttachPin(PWM_Pin, 0);
    ledcAttachPin(PWM_Pin2, 1);
}  

////////////////////////////////////////////////////////////////////////////////////
// Funcion configuracion del puerto serie
////////////////////////////////////////////////////////////////////////////////////
void config_sp(){
  Serial.begin(115200);
  //Serial2.begin(115200);
  while (!Serial) {}
}  


////////////////////////////////////////////////////////////////////////////////////
// Funcion de interpolacion LUT de Velocidad-Voltaje
////////////////////////////////////////////////////////////////////////////////////

float interpola_vel_vol_lut(float x) {
    // Buscar el valor superior más pequeño del array
    int8_t i = 0;
    if ( x >= Vel_LUT[LONG_LUT - 2] ) {i = LONG_LUT - 2;}
    else {while ( x > Vel_LUT[i+1] ) i++;}
  
    // Guardar valor superior e inferior
    float xL = Vel_LUT[i];
    float yL = Vol_LUT[i];
    float xR = Vel_LUT[i+1];
    float yR = Vol_LUT[i+1];
  
    // Interpolar
    float dydx = ( yR - yL ) / ( xR - xL );
  
    return yL + dydx * ( x - xL );
}
