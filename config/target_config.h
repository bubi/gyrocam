

#define LED_PORT 0		// Port for led
#define LED_BIT 7		// Bit on port for led
#define LED_ON  LED_PORT,LED_BIT,1		// Level to set port to turn on led
#define LED_OFF LED_PORT,LED_BIT,0		// Level to set port to turn off led

#define MPU6050_ADRESS	0xD0
#define UART_BAUD 		115200

#define DEBUG_OUTPUT
#define DEBUG_TIME_MS	100 // max 1s

#define dMIN_ANGLE		1
