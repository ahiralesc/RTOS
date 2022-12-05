#ifndef INC_POLYPHONIC_TUNES_H_
#define INC_POLYPHONIC_TUNES_H_

#include "stm32f4xx_hal.h"
#include "stdint.h"

#define SINE     0
#define TRIANGLE 1

#define ENVELOPE0 0
#define ENVELOPE1 1
#define ENVELOPE2 2
#define ENVELOPE3 3

#define FS 20000.0                                              //-Sample rate (NOTE: must match tables.h)


//Must be called in your timer interrupt function
void audio_synthesis();
//Setup the synth engine
void setup_synth_engine(double timer_frequency, TIM_HandleTypeDef* ctrl_tim);
//If you want to use a different output method other than PWM, provide your own function
void setup_synth_custom_output_handler(void (*output_handler)(uint32_t));
//Use default PWM output, must provide a PWM configured timer with its channel
void setup_synth_pwm_output_handler(TIM_HandleTypeDef* output_tim, uint8_t out_channel);
//Setup individual voice
void setupVoice(uint8_t voice, uint8_t wave, uint8_t pitch, uint8_t env, uint8_t length, uint16_t mod);
//Set wave type for individual voice
void setWave(uint8_t voice, uint8_t wave);
//Set pitch for individual voice
void setPitch(uint8_t voice, uint8_t MIDInote);
//Set envelope for individual voice
void setEnvelope(uint8_t voice, uint8_t env);
//Set length for individual voice
void setLength(uint8_t voice, uint8_t length);
//Set modulation for individual voice. Higher values mean higher pitch modulation to the voice. to zero out, use 64
void setMod(uint8_t voice, uint16_t mod);
//Trigger a midi note for individual voice
void mTrigger(uint8_t voice, uint8_t MIDInote);
//Set frequency for individual voice
void setFrequency(uint8_t voice, float f);
//Set time period for individual voice
void setTime(uint8_t voice, float t);
//Trigger individual voice
void trigger(uint8_t voice);
//Pause individual voice
void pause(uint8_t voice);
//Suspend timer counting
void synth_suspend();
//Resume timer counting
void synth_resume();


#endif /* INC_POLYPHONIC_TUNES_H_ */
