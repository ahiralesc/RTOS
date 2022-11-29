/*
 * song_engine.h
 *	Universidade Federal de Minas Gerais
 *  Created on: Oct 7, 2020
 *      Author: Renan Moreira, Rodolfo Lessa
 *     Version: 1.0
 *     License: GPLv3.0
 */

#ifndef POLYPHONIC_TUNES_LIB_SONG_ENGINE_H_
#define POLYPHONIC_TUNES_LIB_SONG_ENGINE_H_

#include "stdint.h"
#include "stm32f4xx_hal.h"

typedef enum {
	WHOLE_NOTE   		= 1,
	HALF_NOTE	 		= 2,
	QUARTER_NOTE 		= 4,
	EIGHTH_NOTE	 		= 8,
	SIXTEENTH_NOTE 		= 16,
	THIRTY_SECOND_NOTE 	= 32,
	SIXTY_FOURTH_NOTE 	= 64
} note_value;


typedef enum {
	STOPPED,
	PLAYING,
	PAUSED
} song_status;

typedef struct {
	volatile uint8_t note[4];
	volatile uint32_t remaining_ticks[4];
	volatile uint32_t pause_ticks[4];
	volatile double duration[4];
	volatile uint16_t posicao[4];
} song_ctrl;

typedef struct {
	note_value nota_ref;
	uint16_t bpm;
	uint8_t oitava;

	const char* voices[4];

	volatile song_ctrl control;
} music;

//Percentage between song and stop for note differentiation
#define NR_PERC 95.0

//Initialize the song engine. Must provide a configured timer set to interrupt in 20khz
void initialize_song_engine(double timer_freq, TIM_HandleTypeDef* ctrl_tim);
//Load the song into memory
void load_song(music musica);
//Clear the song
void clear_song();
//Use default PWM output, must provide a PWM configured timer with its channel
void set_pwm_output(TIM_HandleTypeDef* output_tim, uint8_t out_channel);
//If you want to use a different output method other than PWM, provide your own function
void set_custom_output_handler(void (*output_handler)(uint32_t));
//Starts playing the song
void play_song();
//Pause the song
void pause_song();
//Stop playing the song
void stop_song();
//Must be called in your implementation of the timer ellapsed time callback
void song_scheduler(TIM_HandleTypeDef* htim);
//Return whether the song is playing or not
song_status get_song_status();



#endif /* POLYPHONIC_TUNES_LIB_SONG_ENGINE_H_ */
