#ifndef WIRING_PINS_H
#define WIRING_PINS_H
/**
 * Source:nhttps://pinout.xyz/pinout/wiringpi
 *
 */
#define PI 3.14159265358979323846 /* pi */

#define GPIO_I2C_0 30
#define GPIO_I2C_1 31
#define GPIO_I2C_2 8
#define GPIO_I2C_3 9
#define GPIO_4 7
#define GPIO_5 21
#define GPIO_6 22
#define GPIO_SPI_7 11
#define GPIO_SPI_8 10
#define GPIO_SPI_9 13
#define GPIO_SPI_10 12
#define GPIO_SPI_11 14
#define GPIO_12 26
#define GPIO_13 23
#define GPIO_UART_14 15
#define GPIO_UART_15 16
#define GPIO_16 27
#define GPIO_17 0
#define GPIO_18 1
#define GPIO_SPI_19 24
#define GPIO_SPI_20 28
#define GPIO_SPI_21 29
#define GPIO_22 3
#define GPIO_23 4
#define GPIO_24 5
#define GPIO_25 6
#define GPIO_26 25
#define GPIO_27 2

#define MOTOR_L GPIO_13
#define MOTOR_R GPIO_12

#define UNKNOWN_ERROR -255
#define INTERRUPT -1
#define RETRY -15
#define CONTROL_OK 0
#define CONTROL_CONTINUE 1

#endif
