#define CLK_FREQ 8000000UL
#define UART_TX 0
#define F_CPU CLK_FREQ
#define EN_POW 2

#define MOSI 0x06
#define MISO 0x05

#define FRAME_LENGTH 38
#define DATA_LENGTH FRAME_LENGTH - 6

#define MAX_PACKET_LEN 2

#define BAT_ADC_CH 7
#define PD_ADC_CH 3


#define DDR_SCL DDRB
#define DDR_SDA DDRA
#define PORT_SCL PORTB
#define PORT_SDA PORTA
#define PIN_SCL PINB
#define PIN_SDA PINA
#define SCL 0x02
#define SDA 0x01

