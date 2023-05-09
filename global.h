

// Parametros cola de la interrupcion del encoder ///////////////////////////////////////
#define TAM_COLA_I 1024 /*num mensajes*/
#define TAM_MSG_I 1 /*num caracteres por mensaje*/

// TIEMPOS
#define BLOQUEO_TAREA_LOOPCONTR_MS 10
#define BLOQUEO_TAREA_MEDIDA_MS 200

#define AUTO_STOP_INTERVAL 2100
long lastMotorCommand = 0;

int LED_PWM = 18;

int dutyCycle = 0;
uint8_t r=0;
float flancos = 1200.0;
float volt_max = 11.99;
float volt_min = 0.1;
float Akpi=0.0;
float Akp=0.0;
float error_2 = 0.0; // e(t-2)
float error_1 = 0.0; // e(t-1)
float error_0 = 0.0; // e(t)
float output = 0.0;  
float A0d=0.0;
float A1d=0.0;
float A2d=0.0;
float N=5;
float tau = 0.0; // IIR filter time constant
float alpha = 0.0;
float d0 = 0.0;
float d1 = 0.0;
float fd0 = 0.0;
float fd1 = 0.0;
float Kp = 0.0;
float Ki = 0.0;
float Kd = 0.0;
float setpoint = 0.0;
float dt = (BLOQUEO_TAREA_LOOPCONTR_MS / 1000.0);
float z_i = 0;
float z_d = 0;
float Tiuser = 0;
float Tduser = 0;
float Ti = 0;
float Td = 0;
int ACTIVA_P1C_MED_ANG = 0;
int32_t ang_cnt = 0;
float v_medida = 0.0;     
float a_medida = 0;
int8_t start_stop = 0;  
int direccion = 0;
int direccion_ant = 0;  
float anterior = 0;
float da = 0;
float error_anterior = 0;


int dutyCycle2 = 0;
uint8_t r2=0;

float Akpi2=0.0;
float Akp2=0.0;
float error_22=0.0; // e(t-2)
float error_12 = 0.0; // e(t-1)
float error_02 = 0.0; // e(t)
float output2 = 0.0;  
float A0d2=0;
float A1d2=0.0;
float A2d2=0.0;
float N2=5;
float tau2 = 0.0; // IIR filter time constant
float alpha2 = 0.0;
float d02 = 0;
float d12 = 0;
float fd02 = 0;
float fd12 = 0;
float Kp2 = 0;
float Ki2 = 0;
float Kd2 = 0;
float setpoint2 = 0;
float dt2 = (BLOQUEO_TAREA_LOOPCONTR_MS / 1000.0);
float z_i2= 0;
float z_d2 = 0;
float Tiuser2 = 0;
float Tduser2 = 0;
float Ti2 = 0;
float Td2 = 0;
int ACTIVA_P1C_MED_ANG2 = 0;
int32_t ang_cnt2 = 0;
float v_medida2 = 0.0;     
float a_medida2 = 0;
int8_t start_stop2 = 0;  
int direccion2 = 0;
int direccion_ant2 = 0;  
float anterior2 = 0;
float da2 = 0;
float error_anterior2 = 0;


// Configuración PWM  ////////////////////////////////////////////////////////////////////
uint32_t pwmfreq = 8000; // 1KHz
const uint8_t pwmChannel = 0;
const uint8_t pwmresolution = 10;
const int PWM_Max = pow(2,pwmresolution)-1; //
const uint8_t pwmChannel2 = 1;
//adc
//Adafruit_ADS1115 ads;
float current = 0;
int16_t adc0 = 0;


// Pines driver motor ///////////////A/////////////////////////////////////////////////////


const uint8_t PWM_Pin2 = 25; //
const uint8_t PWM_f2 = 27; //  // direccion
const uint8_t A_enc_pin2 = 35;
const uint8_t B_enc_pin2 = 34;
const uint8_t PWM_en2 = 32;

const uint8_t PWM_Pin = 26; // 
const uint8_t PWM_f = 14; 
const uint8_t A_enc_pin = 22;
const uint8_t B_enc_pin = 21;
const uint8_t PWM_en = 33;

/*
const uint8_t PWM_Pin = 25; // 
const uint8_t PWM_f = 27; 
const uint8_t A_enc_pin = 35;
const uint8_t B_enc_pin = 34;
const uint8_t PWM_en = 32; 
*/
/*
const uint8_t PWM_Pin2 = 26; //
const uint8_t PWM_f2 = 14; //  // direccion
const uint8_t A_enc_pin2 = 36;
const uint8_t B_enc_pin2 = 39;
const uint8_t PWM_en2 = 33;*/ 
// Voltaje maximo motor ////////////////////////////////////////////////////////////////////
float SupplyVolt = 12.0;


// Conversión a angulo y velocidad del Pololu 3072
//const float conv_rad = ; 
//const float conv_rev = ;
//const float conv_rad_grados = ; 
const float rpm_to_radians = 0.10471975512;
const float rad_to_deg = 57.29578;

// Declarar funciones ////////////////////////////////////////////////////////////////////
void config_sp(); // Configuracion puerto serie
void config_oled(); // Configuracion OLED
void config_enc(); // Configuracion del encoder
void config_PWM(); // Configuracion PWM
void excita_motor(float v_motor); // Excitacion motor con PWM
float interpola_vel_vol_lut(float x); // Interpolacion velocidad/voltios LUT
void proceso_angulo();
void proceso_rpm();
void puesta_a_cero();
void init_params();
void config_ADC();
void clean();
void clean2();
void excita_motor2(float v_motor);
void task_serial(void* arg);
void task_loopcontr(void* arg);
void task_enc(void* arg);
void IRAM_ATTR ISR_enc();
void IRAM_ATTR ISR_enc2();

// TABLA VELOCIDAD-VOLTAJE P1D
#define LONG_LUT 12
//Vector de tensiones
const float Vol_LUT[LONG_LUT] = {0.6,  1 ,  1.5 , 2 ,  3,   4,  5,  6, 7, 8, 9, 100};
// Vector de velocidades
const float Vel_LUT[LONG_LUT] = {0.25,0.7, 1.12, 1.58, 2.5, 3.25, 4.1, 5, 5.91, 6.75,7.58,7.58 };



// Declaracion objetos  ////////////////////////////////////////////////////////////////////
xQueueHandle cola_enc; // Cola encoder

xQueueHandle cola_enc2; 
