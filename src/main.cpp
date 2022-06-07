/*
Autora: Thatyanne Prado
Placa: ESP32 DOIT devkit
porta: /dev/ttyACM0
Em caso de erro de conexão com placa ESP32, utilizar 
código no cmd : sudo chmod a+rw /dev/ttyUSB0
*/

#include "main.h"
#include <PID_v1.h>
#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling

static esp_adc_cal_characteristics_t *adc_chars1;
static const adc_channel_t channel1 = ADC_CHANNEL_6;     //GPIO34 if ADC1, GPIO14 if ADC2
static esp_adc_cal_characteristics_t *adc_chars2;
static const adc_channel_t channel2= ADC_CHANNEL_7;     //GPIO35 if ADC1,


static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;



//DEFINIÇÃO DE PARÂMETROS DE INPUT
//Parâmetro com entradas dos sensores de posição
uint32_t adc1Value;
uint32_t adc2Value;

//DEFINIÇÃO DO PID
/*Define as variáveis do processo */
double_t Setpoint1, Input1, Output1;
double_t Setpoint2, Input2, Output2;

/* Define os parâmetros do PID */
float_t Kp = 7.5e3, Ki = 90.0, Kd = 18.61;
//float_t Kp = 10e10, Ki = 90, Kd = 90;

/* Define links e parâmetros iniciais do PID tunning*/
PID myPID1(&Input1,&Output1,&Setpoint1,Kp,Ki,Kd,AUTOMATIC);
PID myPID2(&Input2,&Output2,&Setpoint2,Kp,Ki,Kd,AUTOMATIC);

//PARÂMETROS DO PWM
/*Define as propriedades do PWM*/
int freq = 10000;
int canal1 = 0;
int canal2 = 1;
int canal3 = 2;
int canal4 = 3;
int resolucao = 10;
/*Define dutycycle*/
int dutycycle = 102;

/*Define correntes para bobinas*/
float_t ixa1,ixa2,iya1,iya2;

/*Define corrente de base*/
float_t ib = 1.0; /*Corrente base = 1A*/



void setup() {
  initSaida();//inicia pinos de saída do PWM e do LED indicador

  //Calibração do ADC
  adc1_config_width(width);
  adc1_config_channel_atten((adc1_channel_t)channel1, atten);
  adc1_config_channel_atten((adc1_channel_t)channel2, atten);  

  //Characterize ADC
  adc_chars1 = (esp_adc_cal_characteristics_t*)calloc(1, sizeof(esp_adc_cal_characteristics_t));
  esp_adc_cal_value_t val_type1 = esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, adc_chars1);
  adc_chars2 = (esp_adc_cal_characteristics_t*)calloc(1, sizeof(esp_adc_cal_characteristics_t));
  esp_adc_cal_value_t val_type2 = esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, adc_chars2);

  // Valores de input e setpoint para PID
  Input1 = adc1Value;
  Setpoint1 = 0.0;
  
  Input2 = adc2Value;
  Setpoint2 = 0.0;

  /*Inicia o PID */
  myPID1.SetMode(AUTOMATIC);
  myPID2.SetMode(AUTOMATIC);
  
  //Setup do PWM
  /*Conecta o canal ao GPIO a ser controlado*/
  ledcAttachPin(PIN_PWM1, canal1);
  ledcAttachPin(PIN_PWM2, canal2);
  ledcAttachPin(PIN_PWM3, canal3);
  ledcAttachPin(PIN_PWM4, canal4);

  ledcSetup(canal1, freq, resolucao);
  ledcSetup(canal2, freq, resolucao);
  ledcSetup(canal3, freq, resolucao);
  ledcSetup(canal4, freq, resolucao);

  delay(10);  

}

void loop() {

  //Liga LED indicador
  digitalWrite(LED_HEART_BEAT, HIGH);

  //Multisampling do Sensor 1 (X)
  for (int i = 0; i < NO_OF_SAMPLES; i++) {    
    adc1Value += adc1_get_raw((adc1_channel_t)channel1);
  }
  adc1Value /= NO_OF_SAMPLES;
  //Convert adc1Value to voltage in mV
  uint32_t voltage1 = esp_adc_cal_raw_to_voltage(adc1Value, adc_chars1);
  printf("----------------------------------------------\n");
  printf("Raw1: %d\tVoltage1: %dmV\n", adc1Value, voltage1);

  //Multisampling do Sensor 2 (Y)
  for (int i = 0; i < NO_OF_SAMPLES; i++) {    
    adc2Value += adc1_get_raw((adc1_channel_t)channel2);
  }
  adc2Value /= NO_OF_SAMPLES;
  //Convert adc2Value to voltage in mV
  uint32_t voltage2 = esp_adc_cal_raw_to_voltage(adc2Value, adc_chars2);
  printf("Raw2: %d\tVoltage2: %dmV\n", adc2Value, voltage2);

  //Calibração com curva dos sensores de posição
  float_t sensor1 = (0.17848*(voltage1*3*pow(10,-3))+0.072349);
  float_t sensor2 = (0.17816*(voltage2*3*pow(10,-3))+0.12782);

  //IMPLEMENTAÇÃO PID

  /*Loop para PID1, eixo X*/
  
  Input1 = sensor1;
  Setpoint1 = 1.0;
  myPID1.SetTunings(Kp,Ki,Kd);
  myPID1.Compute();        
      

  /*Loop para PID2, eixo Y*/
  Input2 = sensor2;
  Setpoint2 = 1.0;
  myPID2.SetTunings(Kp,Ki,Kd);
  myPID2.Compute();        
  

  /*Calcula correntes para bobinas 
  a partir das correntes de controle*/
  ixa1 = ib+Output1;
  ixa2 = ib-Output1;
  iya1 = ib+Output2;
  iya2 = ib-Output1;

  //IMPLEMENTAÇÃO PWM
  //Faz write do PWM  
  ledcWrite(canal1,dutycycle*ixa1);
  ledcWrite(canal2,dutycycle*ixa2);
  ledcWrite(canal3,dutycycle*iya1);
  ledcWrite(canal4,dutycycle*iya2);

  
  //Printa os valores das posições X Y calculadas do mancal
  printf("Valor posicao x: %f \n", sensor1);
  printf("Valor posicao y: %f \n", sensor2);
  //Printa os valores das correntes para as bobinas em A
  printf("Valores das correntes ixa1:%f \t ixa2: %f\n", ixa1, ixa2);
  printf("Valores das correntes iya1:%f \t iya2: %f\n", iya1, iya2);
  //Printa os valores das correntes de controle
  printf("Valores das correntes de controle ix:%f \t iy: %f\n", Output1, Output2);
 

  //digitalWrite(LED_HEART_BEAT, LOW);
  delay(1000);

}