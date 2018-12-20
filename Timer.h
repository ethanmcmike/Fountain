#ifndef TIMER_H
#define TIMER_H

#include <Arduino.h>
 
class Timer {
	
	public:
        Timer();
        ~Timer();
        void reset();
		unsigned long getMillis();
		
	private:
		unsigned long lastTime, currentTime;
};
 
#endif