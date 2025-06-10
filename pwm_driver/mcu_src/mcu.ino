//this is ATTiny85 code for safety PWM shutdown after 5 seconds.
//ATTiny85 should be programed using 1MHz internal clock for better time keeping
#include <Wire.h>

int pwm_status;

int heartbeatTimeout = 10000;
unsigned long lastHeartbeatTime;

void setup() {
  Wire.begin(8);                // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // register event

  pinMode(PB1, OUTPUT);
  digitalWrite(PB1, LOW);   //Enable the PWM
  pwm_status = 1;

  lastHeartbeatTime = millis();

}

void loop() {

  if (millis() - lastHeartbeatTime > heartbeatTimeout) 
  {
    if(pwm_status == 1)
    {
      digitalWrite(PB1, HIGH);   //Disable the PWM
      pwm_status = 0;
    }
  }
  delay(10);
}

void receiveEvent(int howMany) {
     lastHeartbeatTime = millis(); //Get Time
     char data = Wire.read(); // receive byte as a character
     if (data == 'H')
     {
        digitalWrite(PB1, LOW);   //Enable the PWM
        pwm_status = 1;     
        
     }
}
