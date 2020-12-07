#include "Arduino.h"
void InitMotor(void)
{
    int brightness = 55;    // how bright the LED is
    int fadeAmount = 5;
    int num_loop=0;
    while (num_loop<=10)
    {

        analogWrite(10,brightness);
    
        analogWrite(9,brightness);
        
        brightness = brightness + fadeAmount;
        
        num_loop++;
        
        if (brightness <= 0 || brightness >= 255) 
        {
        fadeAmount = -fadeAmount;      
        }
        if ( num_loop<=1 )
        {
        delay(3000);
        }
        else
        delay(10);
    }
    analogWrite(10,20);
    analogWrite(9,20);
}