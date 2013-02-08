

#define LED_PORT 0		// Port for led
#define LED_BIT 7		// Bit on port for led
#define LED_ON  LED_PORT,LED_BIT,1		// Level to set port to turn on led
#define LED_OFF LED_PORT,LED_BIT,0		// Level to set port to turn off led

#define MPU6050_ADRESS	0xD0
#define UART_BAUD 		115200

//#define SERVO_PERIODE 		(TIMER_1US * 2500)  // 400hz
#define SERVO_PERIODE		(TIMER_1MS * 20)	// 50hz
#define SERVO_ZERO 	  		(SERVO_PERIODE - (TIMER_1US * 1500))
#define SERVO_MAX_L			(SERVO_PERIODE - (TIMER_1US * 1000))
#define SERVO_MAX_R			(SERVO_PERIODE - (TIMER_1US * 2000))
#define SERVO_MAX_ANGLE		60
#define dMIN_ANGLE		0


//#define DEBUG_OUTPUT
//#define DUMP
#define DEBUG_TIME_MS	100 // max 1s

