

#define LED_PORT 0		// Port for led
#define LED_BIT 7		// Bit on port for led
#define LED_ON  LED_PORT,LED_BIT,1		// Level to set port to turn on led
#define LED_OFF LED_PORT,LED_BIT,0		// Level to set port to turn off led

#define MPU6050_ADRESS	0xD0
#define UART_BAUD 		115200

#define SERVO_PERIODE 		(TIMER_1US * 2500)  // 400hz
#define SERVO_ZERO 	  		(TIMER_1US * 1500)
#define SERVO_MAX_R			(TIMER_1US * 800)
#define SERVO_MAX_L			(TIMER_1US * 2200)
#define SERVO_MAX_ANGLE		120


#define MIN_ANGLE			1
#define SLEW_RATE			5
#define MECH_OFFSET			4.7

//#define DEBUG_OUTPUT
//#define DUMP
#define DEBUG_TIME_MS	100 // max 1s

