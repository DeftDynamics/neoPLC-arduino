#include <nrf.h>
#include <nrf_soc.h>
#include <nrf_nvic.h>
#include "neoTimer.h"

//nrf_nvic_state_t nrf_nvic_state;

neoTimer::neoTimer(){
}

void neoTimer::setPrescaler(int prescaler){
	_prescaler = prescaler;
	if(_prescaler<0){_prescaler=0;}
	else if(_prescaler > 9){_prescaler=9;}
}

void neoTimer::ISR(){
	if(NRF_TIMER1->EVENTS_COMPARE[0]){
		NRF_TIMER1->EVENTS_COMPARE[0] = 0;
		NRF_TIMER1->CC[0] += _delay[0]; 
		_event_handlers[0]();
	}
	if(NRF_TIMER1->EVENTS_COMPARE[1]){
		NRF_TIMER1->EVENTS_COMPARE[1] = 0;
		NRF_TIMER1->CC[1] += _delay[1]; 
		_event_handlers[1]();
	}
	if(NRF_TIMER1->EVENTS_COMPARE[2]){
		NRF_TIMER1->EVENTS_COMPARE[2] = 0;
		NRF_TIMER1->CC[2] += _delay[2]; 
		_event_handlers[2]();
	}
	if(NRF_TIMER1->EVENTS_COMPARE[3]){
		NRF_TIMER1->EVENTS_COMPARE[3] = 0;
		NRF_TIMER1->CC[3] += _delay[3]; 
		_event_handlers[3]();
	}
	if(NRF_TIMER1->EVENTS_COMPARE[4]){
		NRF_TIMER1->EVENTS_COMPARE[4] = 0;
		NRF_TIMER1->CC[4] += _delay[4]; 
		_event_handlers[4]();
	}
	if(NRF_TIMER1->EVENTS_COMPARE[5]){
		NRF_TIMER1->EVENTS_COMPARE[5] = 0;
		NRF_TIMER1->CC[5] += _delay[5]; 
		_event_handlers[5]();
	}
}

void neoTimer::addTimer(int timerID, int delay_micro, _event_handler event_handler){
	switch(timerID){
		case 0:
			NRF_TIMER1->INTENSET = TIMER_INTENSET_COMPARE0_Msk;
			break;
		case 1:
			NRF_TIMER1->INTENSET = TIMER_INTENSET_COMPARE1_Msk;
			break;
		case 2:
			NRF_TIMER1->INTENSET = TIMER_INTENSET_COMPARE2_Msk;
			break;
		case 3:
			NRF_TIMER1->INTENSET = TIMER_INTENSET_COMPARE3_Msk;
			break;
		case 4:
			NRF_TIMER1->INTENSET = TIMER_INTENSET_COMPARE4_Msk;
			break;
		case 5:
			NRF_TIMER1->INTENSET = TIMER_INTENSET_COMPARE5_Msk;
			break;
	}
	_delay[timerID] = delay_micro * (16.0/(1<<_prescaler));
	_event_handlers[timerID] = event_handler;
}

void neoTimer::addTimerMillis(int timerID, int delay_millis, _event_handler event_handler){
	switch(timerID){
		case 0:
			NRF_TIMER1->INTENSET = TIMER_INTENSET_COMPARE0_Msk;
			break;
		case 1:
			NRF_TIMER1->INTENSET = TIMER_INTENSET_COMPARE1_Msk;
			break;
		case 2:
			NRF_TIMER1->INTENSET = TIMER_INTENSET_COMPARE2_Msk;
			break;
		case 3:
			NRF_TIMER1->INTENSET = TIMER_INTENSET_COMPARE3_Msk;
			break;
		case 4:
			NRF_TIMER1->INTENSET = TIMER_INTENSET_COMPARE4_Msk;
			break;
		case 5:
			NRF_TIMER1->INTENSET = TIMER_INTENSET_COMPARE5_Msk;
			break;
	}
	_delay[timerID] = delay_millis * (16.0/(1<<_prescaler))*1000;
	_event_handlers[timerID] = event_handler;
}

void neoTimer::addTimerTicks(int timerID, int delay_ticks, _event_handler event_handler){
	switch(timerID){
		case 0:
			NRF_TIMER1->INTENSET = TIMER_INTENSET_COMPARE0_Msk;
			break;
		case 1:
			NRF_TIMER1->INTENSET = TIMER_INTENSET_COMPARE1_Msk;
			break;
		case 2:
			NRF_TIMER1->INTENSET = TIMER_INTENSET_COMPARE2_Msk;
			break;
		case 3:
			NRF_TIMER1->INTENSET = TIMER_INTENSET_COMPARE3_Msk;
			break;
		case 4:
			NRF_TIMER1->INTENSET = TIMER_INTENSET_COMPARE4_Msk;
			break;
		case 5:
			NRF_TIMER1->INTENSET = TIMER_INTENSET_COMPARE5_Msk;
			break;
	}
	_delay[timerID] = delay_ticks;
	_event_handlers[timerID] = event_handler;
}

void neoTimer::removeTimer(int timerID){
	switch(timerID){
		case 0:
			NRF_TIMER1->INTENCLR = TIMER_INTENCLR_COMPARE0_Msk;
			break;
		case 1:
			NRF_TIMER1->INTENCLR = TIMER_INTENCLR_COMPARE1_Msk;
			break;
		case 2:
			NRF_TIMER1->INTENCLR = TIMER_INTENCLR_COMPARE2_Msk;
			break;
		case 3:
			NRF_TIMER1->INTENCLR = TIMER_INTENCLR_COMPARE3_Msk;
			break;
		case 4:
			NRF_TIMER1->INTENCLR = TIMER_INTENCLR_COMPARE4_Msk;
			break;
		case 5:
			NRF_TIMER1->INTENCLR = TIMER_INTENCLR_COMPARE5_Msk;
			break;
	}
	_delay[timerID] = 0;
}

void neoTimer::start(){
	//sd_clock_hfclk_request();
	NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
	while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) {
        // Do nothing.
    }
	NRF_TIMER1->MODE = TIMER_MODE_MODE_Timer;
	NRF_TIMER1->PRESCALER = _prescaler;
	sd_nvic_ClearPendingIRQ(TIMER1_IRQn);
	sd_nvic_SetPriority(TIMER1_IRQn,3);
	sd_nvic_EnableIRQ(TIMER1_IRQn);
	NRF_TIMER1->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
	NRF_TIMER1->CC[0] = _delay[0];
	NRF_TIMER1->CC[1] = _delay[1];
	NRF_TIMER1->CC[2] = _delay[2];
	NRF_TIMER1->CC[3] = _delay[3];
	NRF_TIMER1->CC[4] = _delay[4];
	NRF_TIMER1->CC[5] = _delay[5];
    NRF_TIMER1->TASKS_START = 1;  
}

void neoTimer::stop(){
    NRF_TIMER1->TASKS_CLEAR = 1;  
	NRF_TIMER1->TASKS_STOP = 1;  
}

float neoTimer::maxTime(){
	return 4294967295.0 / (16.0/(1<<_prescaler)) / 1000000;
}

float neoTimer::resolution(){
	return 1.0 / (16.0/(1<<_prescaler));
}

neoTimer Timer;

void TIMER1_IRQHandler(void){
	Timer.ISR();
}

