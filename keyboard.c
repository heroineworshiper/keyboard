/*
 * LAPTOP KEYBOARD TO USB CONVERTER
 * Copyright (C) 2024 Adam Williams <broadcast at earthling dot net>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 * 
 */

// This runs on an STM32F407 because it had the required pin count, USB, &
// was free.  The smart solution today would be an RP2040.

// make;./uart_programmer keyboard.bin

#include "linux.h"
#include "uart.h"
#include "misc.h"
#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_syscfg.h"
#include "stm32f4xx_tim.h"
#include "arm_usb.h"
#include "usb_core.h"

#define HZ 1000
#define REPORT_SIZE 8
// us delay after raising GND pin
#define DELAY1 1
// us delay after lowering GND pin
#define DELAY2 1
//#define DELAY2 50 // 400us per scan
//#define DELAY2 100 // 800us per scan
//#define DELAY2 3000 // 24ms per scan
// number of passes to integrate
//#define PASSES 500 // 50ms per report
//#define PASSES 100 // 10ms per report
#define PASSES 150


#define MAX_HITS 16
// all keycodes detected in the previous integration
int prev_hits[MAX_HITS];
// all keycodes detected in the current integration
int next_hits[MAX_HITS];
// hits to ignore until they're released
int blacklist[MAX_HITS];

//#define TEST_KEYS
//#define FAKE_KEYPRESS

uint8_t current_report[REPORT_SIZE];
uint8_t prev_report[REPORT_SIZE];
volatile int connected = 0;
extern USB_OTG_CORE_HANDLE  USB_OTG_dev;

// got chars from the host
uint8_t class_cb_DataOut (void  *pdev, uint8_t epnum)
{
    int USB_Rx_Cnt = ((USB_OTG_CORE_HANDLE*)pdev)->dev.out_ep[epnum].xfer_count;


// process the codes
    int i;
    for(i = 0; i < USB_Rx_Cnt; i++)
    {
    }

    usb_start_receive();
    return 0;
}


void USART6_IRQHandler(void)
{
	unsigned char c = USART6->DR;
	uart.input = c;
	uart.got_input = 1;
}

// reg, pin number
typedef struct 
{
    GPIO_TypeDef *reg;
    int number;
} pin_t;

const pin_t pins[] =
{
    { GPIOA, 1 },
    { GPIOA, 2 },
    { GPIOA, 3 },
    { GPIOA, 4 },
    { GPIOA, 5 },
    { GPIOA, 6 },
    { GPIOA, 7 },
    { GPIOC, 4 },
    { GPIOC, 5 },
    { GPIOB, 0 },
    { GPIOB, 1 },
    { GPIOB, 2 },
    { GPIOE, 7 },
    { GPIOE, 8 },
    { GPIOE, 9 },
    { GPIOE, 10 },
    { GPIOE, 11 },
    { GPIOE, 12 },
    { GPIOE, 13 },
    { GPIOE, 14 },
    { GPIOE, 15 },
    { GPIOB, 10 },
    { GPIOB, 11 },
    { GPIOB, 12 }, // last 3 are not in geographic order because of swapped traces
    { GPIOD, 11 }, // was B13
    { GPIOD, 13 } // was D8
};
#define TOTAL_PINS (sizeof(pins) / sizeof(pin_t))


// USB bitmask for modifier keys
// offset 0
#define LEFT_CTRL 0x01
#define RIGHT_CTRL 0x10
#define LEFT_ALT 0x04
#define RIGHT_ALT 0x40
#define WINDOWS 0x08
#define LEFT_SHIFT 0x02
#define RIGHT_SHIFT 0x20
// momentary keys
// offset 2
#define NUMLOCK 0x53
#define INSERT 0x49
#define HOME 0x4a
#define PGUP 0x4b
#define DELETE 0x4c
#define END 0x4d
#define PGDN 0x4e
#define KEY_A 0x04
#define KEY_B 0x05
#define KEY_C 0x06
#define KEY_D 0x07
#define KEY_E 0x08
#define KEY_F 0x09
#define KEY_G 0x0a
#define KEY_H 0x0b
#define KEY_I 0x0c
#define KEY_J 0x0d
#define KEY_K 0x0e
#define KEY_L 0x0f
#define KEY_M 0x10
#define KEY_N 0x11
#define KEY_O 0x12
#define KEY_P 0x13
#define KEY_Q 0x14
#define KEY_R 0x15
#define KEY_S 0x16
#define KEY_T 0x17
#define KEY_U 0x18
#define KEY_V 0x19
#define KEY_W 0x1a
#define KEY_X 0x1b
#define KEY_Y 0x1c
#define KEY_Z 0x1d
#define KEY_1 0x1e
#define KEY_9 0x26
#define KEY_0 0x27
#define TILDA 0x35
#define CAPSLOCK 0x39
#define TAB 0x2b
#define SPACE 0x2c
#define ESC 0x29
#define F1 0x3a
#define F12 0x45
#define MINUS 0x2d
#define PLUS 0x2e
#define BACKSPACE 0x2a
#define LBRACKET 0x2f
#define RBRACKET 0x30
#define PIPE 0x31
#define COLON 0x33
#define QUOTE 0x34
#define ENTER 0x28
#define LESSTHAN 0x36
#define GREATERTHAN 0x37
#define QUESTION 0x38
#define UP 0x52
#define DOWN 0x51
#define LEFT 0x50
#define RIGHT 0x4f
#define NUMPAD_SLASH 0x54
#define NUMPAD_TIMES 0x55
#define NUMPAD_MINUS 0x56
#define NUMPAD_PLUS 0x57
#define NUMPAD_ENTER 0x58
#define NUMPAD_PERIOD 0x63
#define NUMPAD_1 0x59
#define NUMPAD_9 0x61
#define NUMPAD_0 0x62
#define PRINTSCREEN 0x46
#define SCROLLOCK 0x47
#define PAUSE 0x48


// translate pin combos to key codes
typedef struct
{
    int sense;
    int gnd;
    uint8_t mod;
    uint8_t momentary;
} key_t;

const key_t keys[] = 
{
    { 1, 18, 0, ESC },
    { 1, 19, 0, F1 },
    { 1, 20, 0, TILDA },
    { 1, 24, 0, KEY_1 },
    { 1, 21, 0, TAB },
    { 1, 22, 0, CAPSLOCK },
    { 1, 25, 0, KEY_A },
    { 2, 19, LEFT_SHIFT, 0 },
    { 2, 22, RIGHT_SHIFT, 0 },
    { 3, 20, LEFT_CTRL, 0 },
    { 3, 24, RIGHT_CTRL, 0 },
    { 4, 21, WINDOWS, 0 },
    { 5, 19, 0, F1 + 1 },
    { 5, 18, 0, F1 + 2 },
    { 5, 20, 0, KEY_1 + 1 },
    { 5, 24, 0, KEY_Q },
    { 5, 21, 0, KEY_W },
    { 5, 22, 0, KEY_S },
    { 5, 23, 0, KEY_Z },
    { 5, 25, 0, KEY_X },
    { 6, 25, LEFT_ALT, 0 },
    { 6, 23, RIGHT_ALT, 0 },
    { 7, 18, 0, F1 + 4 },
    { 7, 19, 0, F1 + 3 },
    { 7, 24, 0, KEY_1 + 2 },
    { 7, 20, 0, KEY_1 + 3 },
    { 7, 21, 0, KEY_E },
    { 7, 22, 0, KEY_D },
    { 7, 25, 0, KEY_C },
    { 8, 18, 0, F1 + 5 },
    { 8, 20, 0, KEY_1 + 4 },
    { 8, 19, 0, KEY_1 + 5 },
    { 8, 21, 0, KEY_R },
    { 8, 24, 0, KEY_T },
    { 8, 25, 0, KEY_F },
    { 8, 22, 0, KEY_G },
    { 8, 23, 0, KEY_V },
    { 9, 18, 0, F1 + 6 },
    { 9, 20, 0, KEY_1 + 6 },
    { 9, 19, 0, KEY_1 + 7 },
    { 9, 21, 0, KEY_Y },
    { 9, 24, 0, KEY_U },
    { 9, 22, 0, KEY_H },
    { 9, 23, 0, KEY_B },
    { 9, 25, 0, KEY_N },
    { 10, 19, 0, F1 + 7 },
    { 10, 18, 0, F1 + 8 },
    { 10, 20, 0, KEY_I },
    { 10, 24, 0, KEY_J },
    { 10, 21, 0, KEY_K },
    { 10, 22, 0, KEY_M },
    { 10, 25, 0, LESSTHAN },
    { 10, 23, 0, SPACE },
    { 11, 24, 0, INSERT },
    { 11, 21, 0, NUMPAD_TIMES },
    { 11, 19, 0, NUMPAD_MINUS },
    { 11, 20, 0, NUMPAD_1 + 6 },
    { 11, 18, 0, NUMPAD_1 + 7 },
    { 11, 23, WINDOWS, 0 },
    { 12, 25, 0, GREATERTHAN },
    { 12, 23, 0, QUOTE },
    { 12, 24, 0, KEY_P },
    { 12, 21, 0, KEY_L },
    { 12, 20, 0, KEY_O },
    { 12, 19, 0, KEY_1 + 8 },
    { 12, 18, 0, F1 + 9 },
    { 13, 18, 0, F1 + 10 },
    { 13, 19, 0, F1 + 11 },
    { 13, 20, 0, KEY_0 },
    { 13, 24, 0, MINUS },
    { 13, 21, 0, LBRACKET },
    { 13, 25, 0, QUESTION },
    { 14, 19, 0, PRINTSCREEN },
    { 14, 20, 0, PLUS },
    { 14, 24, 0, RBRACKET },
    { 14, 25, 0, PGDN },
    { 14, 22, 0, UP },
    { 14, 23, 0, LEFT },
    { 14, 18, 0, NUMLOCK },
    { 15, 18, 0, END },
    { 15, 19, 0, PAUSE },
    { 15, 23, 0, DOWN },
    { 15, 24, 0, RIGHT },
    { 15, 25, 0, COLON },
    { 15, 21, 0, ENTER },
    { 15, 22, 0, PIPE },
    { 15, 20, 0, BACKSPACE },
    { 16, 23, 0, DELETE },
    { 16, 22, 0, HOME },
    { 16, 25, 0, NUMPAD_1 + 8 },
    { 16, 18, 0, NUMPAD_SLASH },
    { 16, 19, 0, NUMPAD_PLUS },
    { 16, 20, 0, NUMPAD_1 + 3 },
    { 16, 21, 0, NUMPAD_0 },
    { 16, 24, 0, NUMPAD_1 + 2 },
    { 17, 25, 0, PGUP },
    { 17, 24, 0, NUMPAD_1 + 4 },
    { 17, 23, 0, NUMPAD_1 + 5 },
    { 17, 19, 0, NUMPAD_1 },
    { 17, 20, 0, NUMPAD_1 + 1 },
    { 17, 22, 0, NUMPAD_PERIOD },
    { 17, 18, 0, NUMPAD_ENTER }
};


#define TOTAL_KEYS (sizeof(keys) / sizeof(key_t))

// discovered pin range
#define GND1 18
#define GND2 26

void setup_pins(int test_pin)
{
    int i;
// reset all pins
    for(i = 0; i < TOTAL_PINS; i++)
    {
        GPIO_TypeDef *reg = pins[i].reg;
        if(i == test_pin)
            CLEAR_PIN(reg, (1 << pins[i].number))
        else
            SET_PIN(reg, (1 << pins[i].number))
    }
}

void dump_pins(int test_pin)
{
    int i;
    for(i = 0; i < TOTAL_PINS; i++)
    {
        GPIO_TypeDef *reg = pins[i].reg;
        if(i == test_pin)
            print_number(i);
        else
        if(PIN_IS_CLEAR(reg, 1 << pins[i].number))
            print_number(i);
        else
            print_text("* ");
    }
    send_uart('\n');
}

void blacklist_key(int code)
{
    int i;
    if(code == 0) return;

    for(i = 0; i < MAX_HITS; i++)
    {
        if(blacklist[i] == code) return;
    }
    
    for(i = 0; i < MAX_HITS; i++)
    {
        if(blacklist[i] == 0)
        {
            blacklist[i] = code;
            return;
        }
    }
}

// blacklist all the next hits except the given code
void blacklist_except(int code)
{
    int k;
    for(k = 0; k < MAX_HITS; k++)
    {
        if(next_hits[k] && next_hits[k] != code)
            blacklist_key(next_hits[k]);
    }
}

int is_blacklisted(int code)
{
    int i;
    if(code == 0) return 0;
    for(i = 0; i < MAX_HITS; i++)
    {
        if(blacklist[i] == code) return 1;
    }
    return 0;
}

// add momentary keypress to the next_hits
void add_keypress(int code)
{
    int k;
    for(k = 0; k < MAX_HITS; k++)
    {
        if(next_hits[k] == code)
        {
            break;
        }
        else
        if(next_hits[k] == 0)
        {
            next_hits[k] = code;
            break;
        }
    }
}

// the code exists in prev_hits
int prev_exists(int code)
{
    int j;
    if(code == 0) return 0;
    for(j = 0; j < MAX_HITS; j++)
    {
        if(code == prev_hits[j])
        {
            return 1;
        }
    }
    return 0;
}

// the code exists in next_hits
int next_exists(int code)
{
    int j;
    if(code == 0) return 0;
    for(j = 0; j < MAX_HITS; j++)
    {
        if(code == next_hits[j])
        {
            return 1;
        }
    }
    return 0;
}

// next hits have a value
int has_next()
{
    return (next_hits[0] != 0);
}

void assert_wakeup()
{
//    GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, 0);
//    GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, 0);
	GPIO_InitTypeDef  GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    SET_PIN(GPIOB, (1 << 14));
    CLEAR_PIN(GPIOB, (1 << 15));


    close_usb();

//    while(1)
//    {
//    }
}

int main(void)
{
	int i, j, k;

	init_linux();
/* Enable the GPIOs */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA |
			RCC_AHB1Periph_GPIOB |
            RCC_AHB1Periph_GPIOC |
            RCC_AHB1Periph_GPIOD |
            RCC_AHB1Periph_GPIOE, 
		ENABLE);
	GPIO_InitTypeDef  GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    for(i = 0; i < TOTAL_PINS; i++)
    {
#ifdef TEST_KEYS
        GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
#else
// sense pins are open drain.  GND pins are push pull.
        if(i < GND1)
            GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
        else
            GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
#endif
        GPIO_InitStructure.GPIO_Pin = 1 << pins[i].number;
        GPIO_Init(pins[i].reg, &GPIO_InitStructure);
    }
    setup_pins(-1);



	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, ENABLE);
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_DeInit(TIM10);
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period = 6719;
	TIM_TimeBaseStructure.TIM_Prescaler = 24;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM10, &TIM_TimeBaseStructure);
	TIM_Cmd(TIM10, ENABLE);

// enable the interrupt handler
 	NVIC_SetVectorTable(NVIC_VectTab_FLASH, PROGRAM_START - 0x08000000);
	init_uart();
	print_text("Welcome to USB keyboard converter\n");
    print_text("TOTAL_KEYS=");
    print_number(TOTAL_KEYS);
    print_lf();

 	NVIC_InitTypeDef NVIC_InitStructure;
 	NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
 	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
 	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
 	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
 	NVIC_Init(&NVIC_InitStructure);
    USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);



	init_usb();


    int test_pin = 0;
    int tick = 0;
// total scans in the integration
    int total_counter = 0;
// benchmarking
    int scans = 0;
    int do_wakeup = 0;
    int want_usb = 0;
    bzero(next_hits, sizeof(int) * MAX_HITS);
    bzero(prev_hits, sizeof(int) * MAX_HITS);
    bzero(blacklist, sizeof(int) * MAX_HITS);

// mods detected in the integration
    int total_mod = 0;
    setup_pins(test_pin);
    while(1)
    {
        HANDLE_UART_OUT
// poll USB to use the UART
        if(!do_wakeup) handle_usb();

// read matrix
#ifdef TEST_KEYS
        if(uart.got_input)
        {
            uart.got_input = 0;
//            print_text("Got ");
//            print_number(uart.input);
//            print_lf();
            
            switch(uart.input)
            {
                case 'a':
                    test_pin--;
                    if(test_pin < 0) test_pin = TOTAL_PINS - 1;
                    setup_pins(test_pin);
                    break;
                case 'd':
                    test_pin++;
                    if(test_pin >= TOTAL_PINS) test_pin = 0;
                    setup_pins(test_pin);
                    break;
            }
        }
#endif // TEST_KEYS



#ifndef TEST_KEYS
// integrate all the currently pressed keys
        int prev_gnd = -1;

// test each GND pin
        for(i = GND1; i < GND2; i++)
        {
// reset the last GND pin
            if(i > GND1)
            {
                SET_PIN(pins[i - 1].reg, (1 << pins[i - 1].number));
            }
            else
            {
                SET_PIN(pins[GND2 - 1].reg, (1 << pins[GND2 - 1].number));
            }
// wait for pins to rise
            udelay(DELAY1);
// lower the current GND pin
            CLEAR_PIN(pins[i].reg, (1 << pins[i].number));
// wait for pins to lower
            udelay(DELAY2);

// scan all the sense pins
            for(j = 0; j < TOTAL_KEYS; j++)
            {
                const key_t *current_key = &keys[j];
                if(current_key->gnd == i)
                {
                    int sense = current_key->sense;
                    if(PIN_IS_CLEAR(pins[sense].reg, (1 << pins[sense].number)))
                    {
                        total_mod |= current_key->mod;
                        if(current_key->momentary)
                        {
                            add_keypress(current_key->momentary);
                        }
                    }
                }
            }
        }

// update the integration
        total_counter++;
        if(total_counter >= PASSES)
        {
//if(ignore_key) print_text("IGNORE KEY\n");
            current_report[0] = total_mod;

// erase the blacklist if no hits
            if(next_hits[0] == 0)
                bzero(blacklist, sizeof(int) * MAX_HITS);

// report the 1st hit which is not in the previous hits
            current_report[2] = 0;
            for(i = 0; i < MAX_HITS; i++)
            {
                if(next_hits[i] != 0 && !prev_exists(next_hits[i]))
                {
                    current_report[2] = next_hits[i];
// blacklist the hits which we're not using
                    blacklist_except(next_hits[i]);
                    break;
                }
            }

// Reuse the previous report's key if it's still down
            if(current_report[2] == 0)
            {
                if(prev_report[2] && next_exists(prev_report[2]))
                    current_report[2] = prev_report[2];
            }

// Get the 1st hit which isn't blacklisted
            if(current_report[2] == 0)
            {
                for(i = 0; i < MAX_HITS; i++)
                {
                    if(next_hits[i] != 0 && 
                        !is_blacklisted(next_hits[i]))
                    {
                        current_report[2] = next_hits[i];
                        break;
                    }
                }
            }

            if(memcmp(current_report, prev_report, REPORT_SIZE))
            {
                print_buffer(current_report, 4);
                memcpy(prev_report, current_report, REPORT_SIZE);
                if(connected)
                    usb_start_transmit(current_report, REPORT_SIZE, 0);
                else
                if(!do_wakeup && (current_report[0] || current_report[2]))
                {
// key down without USB
                    do_wakeup = 1;
                    assert_wakeup();
                }
                else
                if(do_wakeup && !current_report[0] && !current_report[2])
                {
                    do_wakeup = 0;
// restart USB
                    init_usb();
                }
            }

            scans++;
// reset the integration
            total_counter = 0;
            total_mod = 0;
            memcpy(prev_hits, next_hits, sizeof(int) * MAX_HITS);
            bzero(next_hits, sizeof(int) * MAX_HITS);
        }
#endif // !TEST_KEYS



        if(TIM10->SR & TIM_FLAG_Update)
    	{
    		TIM10->SR = ~TIM_FLAG_Update;
		    tick++;

#ifndef TEST_KEYS
            if(tick >= HZ)
            {
                tick = 0;
                print_text("scans=");
                print_number(scans);
                print_lf();
                scans = 0;
            }
#endif // !TEST_KEYS

// test keys
#ifdef TEST_KEYS
            if(tick >= HZ / 4)
            {
                dump_pins(test_pin);
                flush_uart();
                tick = 0;
            }
#endif // TEST_KEYS

// send a fake keypress
#ifdef FAKE_KEYPRESS
            if(tick >= HZ / 2)
            {
                tick = 0;

                if(connected)
                {
//                    send_uart('.');

// test report
                    bzero(current_report, REPORT_SIZE);
                    static int counter = 0;
                    if(counter & 0x01) current_report[2] = KEY_A;
                    counter++;
                    print_buffer(current_report, 4);

                    usb_start_transmit(current_report, REPORT_SIZE, 1);
                }  
            }
#endif // FAKE_KEYPRESS

        }
    }
}






