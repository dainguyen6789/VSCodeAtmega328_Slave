 #include "Arduino.h"
 
 void HM10MasterSerialInit(void)

 {
  //==============================================================
    Serial.begin(9600);
    delay(1000);
    // set Master mode
    Serial.print("AT+RENEW");// because Module 1 is not stable 
    delay(500);
    Serial.print("AT+RENEW");
    delay(500);
    Serial.print("AT+RENEW");
    delay(500);
    Serial.print("AT+RENEW");
    delay(500);
    Serial.print("AT+RENEW");
    delay(500);
    Serial.print("AT+ROLE1"); 
    delay(1000);
    Serial.print("AT+MODE2");
    delay(1000);
    //Serial.print("AT+DISC?");
    delay(1000);
    // this is slave HM10 MAC Address
    Serial.println("AT+COND43639D711FA");
    
    delay(4000);
}