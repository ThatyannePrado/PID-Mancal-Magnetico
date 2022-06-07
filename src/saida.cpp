#include "saida.h"

void initSaida(){
    Serial.begin(9600); 
    pinMode(LED_HEART_BEAT,OUTPUT);
    pinMode(PIN_PWM1, OUTPUT);//Definimos o pino 27 como saída.
    pinMode(PIN_PWM2, OUTPUT);//Definimos o pino 14 como saída.
    pinMode(PIN_PWM3, OUTPUT);//Definimos o pino 12 como saída.
    pinMode(PIN_PWM4, OUTPUT);//Definimos o pino 13 como saída.
}