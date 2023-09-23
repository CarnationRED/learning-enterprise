#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdbool.h>

#define LED1 3
#define LED2 46

void led_init2();
void led_init();
void led1Flash(bool on, uint8_t hz);
void led2Flash(bool on, uint8_t hz);
void led1Flash2( uint8_t hz, uint32_t times, uint8_t brightness);
void led2Flash2( uint8_t hz, uint32_t times, uint8_t brightness);
void led1Switch(bool on);
void led2Switch(bool on);