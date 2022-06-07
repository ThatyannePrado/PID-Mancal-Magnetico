#include "sensor.h"

int buffer[100];
int i = 0;

void sensorInit(){
    
}

int sensor1Read(){
    int valor  = 0;

    i++;
    if(i==10) i = 0;
    buffer[i] = analogRead(SENSOR_PIN1);

    //Serial.print("Valor lido no sensor x:");
    //Serial.println(analogRead(SENSOR_PIN1));

    for(char x = 0;x<10;x++){
         valor +=buffer[i];
    }


    return map((0.17848*valor+0.072349)*0.001,0,4095,0,100);
}

int sensor2Read(){
    int valor  = 0;

    i++;
    if(i==10) i = 0;
    buffer[i] = analogRead(SENSOR_PIN2);

    //Serial.print("Valor lido no sensor x:");
    //Serial.println(analogRead(SENSOR_PIN1));

    for(char x = 0;x<10;x++){
         valor +=buffer[i];
    }


    return map((0.17816*valor+0.12782)*0.001,0,4095,0,100);
}

