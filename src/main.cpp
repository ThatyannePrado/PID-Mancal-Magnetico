#include "main.h"
#include "PID_v1.h"
#include <Arduino.h>

//DEFINIÇÃO DO PID
/*Define as variáveis do processo */
double Setpoint1, Input1, Output1;
double Setpoint2, Input2, Output2;

/* Define os parâmetros do PID */
double Kp = 7.5e3, Ki = 90.0, Kd = 18.61;

/* Define links e parâmetros iniciais do PID tunning*/
PID myPID1(&Input1,&Output1,&Setpoint1,Kp,Ki,Kd,AUTOMATIC);
PID myPID2(&Input2,&Output2,&Setpoint2,Kp,Ki,Kd,AUTOMATIC);


void setup() {
  initSaida();//inicia pinos de saída e LED indicador

  //DEFINIÇÃO DE PARÂMETROS DE INPUT
  //Parâmetro com entradas dos sensores em posição
  int adc1Value;
  int adc2Value;

  adc1Value = sensor1Read();
  adc2Value = sensor2Read();

  // Valores de input e setpoint para PID
  Input1 = adc1Value;
  Setpoint1 = 0.0;
  
  Input2 = adc2Value;
  Setpoint2 = 0.0;

  /*Inicia o PID */
  myPID1.SetMode(AUTOMATIC);
  myPID2.SetMode(AUTOMATIC);

}

void loop() {
  //DEFINIÇÃO DE PARÂMETROS DE INPUT
  //Parâmetro com entradas dos sensores em posição
  int adc1Value;
  int adc2Value;

  adc1Value = sensor1Read();
  adc2Value = sensor2Read();

  //IMPLEMENTAÇÃO PID
  /*Define correntes para bobinas*/
  int ixa1,ixa2,iya1,iya2;

  /*Define corrente de base*/
  int ib = 1; /*Corrente base = 1A*/

  /*Loop para PID1, eixo X*/
  
  Input1 = adc1Value;
  Setpoint1 = 0.0;
  myPID1.SetTunings(Kp,Ki,Kd);
  myPID1.Compute();        
      

  /*Loop para PID2, eixo Y*/
  Input2 = adc2Value;
  Setpoint2 = 0.0;
  myPID2.SetTunings(Kp,Ki,Kd);
  myPID2.Compute();        
  

  /*Calcula correntes para bobinas 
  a partir das correntes de controle*/
  ixa1 = ib+Output1;
  ixa2 = ib-Output1;
  iya1 = ib+Output2;
  iya2 = ib-Output1;

  //IMPLEMENTAÇÃO PWM
  /*Define as propriedades do PWM*/
  int freq = 10000;
  int canal1 = 0;
  int canal2 = 1;
  int canal3 = 2;
  int canal4 = 3;
  int resolucao = 12;

  /*Conecta o canal ao GPIO a ser controlado*/
  ledcAttachPin(PIN_PWM1, canal1);
  ledcAttachPin(PIN_PWM2, canal2);
  ledcAttachPin(PIN_PWM3, canal3);
  ledcAttachPin(PIN_PWM4, canal4);

  /*Define dutycycle*/
  int dutycycle = 102;

  /*Configura as funcionalidades do PWM*/
  ledcSetup(canal1, freq, resolucao);
  ledcWrite(canal1,dutycycle*ixa1);    
   
  /*Configura as funcionalidades do PWM*/
  ledcSetup(canal2, freq, resolucao);
  ledcWrite(canal2,dutycycle*ixa2);
     
  /*Configura as funcionalidades do PWM*/
  ledcSetup(canal3, freq, resolucao);
  ledcWrite(canal3,dutycycle*iya1);
      
  /*Configura as funcionalidades do PWM*/
  ledcSetup(canal4, freq, resolucao);
  ledcWrite(canal4,dutycycle*iya2);

  delay(10);

}