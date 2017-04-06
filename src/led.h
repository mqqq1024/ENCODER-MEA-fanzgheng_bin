
#ifndef __LED_H__
#define __LED_H__


#define RUN_LED_PORT GPIOB
#define RUN_LED_PIN GPIO_Pin_12

#define set_run_led_on()  GPIO_ResetBits(RUN_LED_PORT, RUN_LED_PIN)
#define set_run_led_off() GPIO_SetBits(RUN_LED_PORT, RUN_LED_PIN)

void led_init(void);
void toggle_run_led(void);

#endif
