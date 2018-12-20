#include "Timer.h"

Timer::Timer(){
	reset();
}

Timer::~Timer(){}

void Timer::reset(){
	lastTime = millis();
}

unsigned long Timer::getMillis(){
	return millis() - lastTime;
}