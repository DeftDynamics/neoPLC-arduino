#ifndef NEOTIMER_H
#define NEOTIMER_H

#include <nrf.h>
#include <nrf_soc.h>
#include <nrf_nvic.h>

typedef void(*_event_handler)();

extern "C" void TIMER1_IRQHandler(void);

class neoTimer{
	public:
		neoTimer();
		void setPrescaler(int prescaler);
		void addTimer(int timerID, int delay_micro, _event_handler event_handler);
		void addTimerMillis(int timerID, int delay_millis, _event_handler event_handler);
		void addTimerTicks(int timerID, int delay_ticks, _event_handler event_handler);
		void removeTimer(int timerID);
		void start();
		void stop();
		float maxTime();
		float resolution();
	private:
		friend void TIMER1_IRQHandler(void);
		void ISR();
		int _delay[6] = {0,0,0,0,0,0};
		_event_handler _event_handlers[6];
		int _prescaler = 4;
};

#ifdef __cplusplus

extern neoTimer Timer;

#endif

#endif