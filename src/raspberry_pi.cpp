#include <arl_hw/raspberry_pi.h>

RaspberryPi::RaspberryPi(){
}

RaspberryPi::~RaspberryPi(){
}

bool RaspberryPi::read(){
  delayMicroseconds(100);
  digitalWrite(0, HIGH);
}

bool RaspberryPi::write(){
	delayMicroseconds(100);
    digitalWrite(0, LOW);
	return true;
}

bool RaspberryPi::initialize(){
	
	wiringPiSetup();
    pinMode(0, OUTPUT);
    digitalWrite (0, LOW);
    
	return true;
}

bool RaspberryPi::close(){
	return true;
}




