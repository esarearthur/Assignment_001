/*
 * Assignment_001.cpp
 *
 * Created: 5/26/2020 3:45:18 AM
 * Author : Ronald Arthur
 */

#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define RED_LED_OFF() PORTC |= (1 << PC0)
#define GREEN_LED_OFF() PORTC |= (1 << PC1)
#define RED_LED_ON() PORTC &= ~(1 << PC0)
#define GREEN_LED_ON() PORTC &= ~(1 << PC1)

#define ALARM() RED_LED_ON(); GREEN_LED_OFF()
#define NO_ALARM() RED_LED_OFF(); GREEN_LED_ON()

#define RS 2
#define EN 3

#define en PD3      // Define Enable pin
#define rs PD2      // Define Register Select pin
#define lcdDirection  DDRD      // Define LCD data direction port
#define lcdPort PORTD //Define LCD data port

//#define USART_BAUDRATE 9600
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

#define N 32					//#samples

uint16_t f[N], angle;
float vt;
uint8_t x, k;

char buffer[16];
bool freq_error = true;
int measured_freq = 0;

const int16_t freq_lookup[] PROGMEM = {
	1023, 1014, 986, 940, 879, 805, 720, 628, 532, 436, 344, 259, 185, 124, 78, 50, 41, 50, 78, 124, 185, 259, 344, 436, 532, 628, 720, 805, 879, 940, 986, 1014,
	1023, 986, 879, 720, 532, 344, 185, 78, 41, 78, 185, 344, 532, 720, 879, 986, 1023, 986, 879, 720, 532, 344, 185, 78, 41, 78, 185, 344, 532, 720, 879, 986,
	1023, 940, 720, 436, 185, 50, 78, 259, 532, 805, 986, 1014, 879, 628, 344, 124, 41, 124, 344, 628, 879, 1014, 986, 805, 532, 259, 78, 50, 185, 436, 720, 940,
	1023, 879, 532, 185, 41, 185, 532, 879, 1023, 879, 532, 185, 41, 185, 532, 879, 1023, 879, 532, 185, 41, 185, 532, 879, 1023, 879, 532, 185, 41, 185, 532, 879,
	1023, 805, 344, 50, 185, 628, 986, 940, 532, 124, 78, 436, 879, 1014, 720, 259, 41, 259, 720, 1014, 879, 436, 78, 124, 532, 940, 986, 628, 185, 50, 344, 805,
	1023, 720, 185, 78, 532, 986, 879, 344, 41, 344, 879, 986, 532, 78, 185, 720, 1023, 720, 185, 78, 532, 986, 879, 344, 41, 344, 879, 986, 532, 78, 185, 720,
	1023, 628, 78, 259, 879, 940, 344, 50, 532, 1014, 720, 124, 185, 805, 986, 436, 41, 436, 986, 805, 185, 124, 720, 1014, 532, 50, 344, 940, 879, 259, 78, 628,
	1023, 532, 41, 532, 1023, 532, 41, 532, 1023, 532, 41, 532, 1023, 532, 41, 532, 1023, 532, 41, 532, 1023, 532, 41, 532, 1023, 532, 41, 532, 1023, 532, 41, 532,
	1023, 436, 78, 805, 879, 124, 344, 1014, 532, 50, 720, 940, 185, 259, 986, 628, 41, 628, 986, 259, 185, 940, 720, 50, 532, 1014, 344, 124, 879, 805, 78, 436,
	1023, 344, 185, 986, 532, 78, 879, 720, 41, 720, 879, 78, 532, 986, 185, 344, 1023, 344, 185, 986, 532, 78, 879, 720, 41, 720, 879, 78, 532, 986, 185, 344,
	1023, 259, 344, 1014, 185, 436, 986, 124, 532, 940, 78, 628, 879, 50, 720, 805, 41, 805, 720, 50, 879, 628, 78, 940, 532, 124, 986, 436, 185, 1014, 344, 259,
	1023, 185, 532, 879, 41, 879, 532, 185, 1023, 185, 532, 879, 41, 879, 532, 185, 1023, 185, 532, 879, 41, 879, 532, 185, 1023, 185, 532, 879, 41, 879, 532, 185,
	1023, 124, 720, 628, 185, 1014, 78, 805, 532, 259, 986, 50, 879, 436, 344, 940, 41, 940, 344, 436, 879, 50, 986, 259, 532, 805, 78, 1014, 185, 628, 720, 124,
	1023, 78, 879, 344, 532, 720, 185, 986, 41, 986, 185, 720, 532, 344, 879, 78, 1023, 78, 879, 344, 532, 720, 185, 986, 41, 986, 185, 720, 532, 344, 879, 78,
	1023, 50, 986, 124, 879, 259, 720, 436, 532, 628, 344, 805, 185, 940, 78, 1014, 41, 1014, 78, 940, 185, 805, 344, 628, 532, 436, 720, 259, 879, 124, 986, 50,
	1023, 41, 1023, 41, 1023, 41, 1023, 41, 1023, 41, 1023, 41, 1023, 41, 1023, 41, 1023, 41, 1023, 41, 1023, 41, 1023, 41, 1023, 41, 1023, 41, 1023, 41, 1023, 41
};

// reverses a string 'str' of length 'len'
void reverse(char *str, int len) {
	int i=0, j=len-1, temp;
	while (i<j) {
		temp = str[i];
		str[i] = str[j];
		str[j] = temp;
		i++;
		j--;
	}
}

// Converts a given integer x to string str[].  d is the number
// of digits required in output. If d is more than the number
// of digits in x, then 0s are added at the beginning.
int intToStr(int x, char str[], int d) {
	int i = 0;
	while (x) {
		str[i++] = (x%10) + '0';
		x = x/10;
	}

	// If number of digits required is more, then
	// add 0s at the beginning
	while (i < d) {
		str[i++] = '0';
	}

	reverse(str, i);
	str[i] = 0x00;
	return i;
}

// Converts a floating point number to string.
void ftoa(float n, char *res, int afterpoint) {
	// Extract integer part
	int ipart = (int)n;

	// Extract floating part
	float fpart = n - (float)ipart;

	// convert integer part to string
	int i = intToStr(ipart, res, 0);

	// check for display option after point
	if (afterpoint != 0) {
		res[i] = '.';  // add dot

		// Get the value of fraction part upto given no.
		// of points after dot. The third parameter is needed
		// to handle cases like 233.007
		fpart = fpart * pow(10, afterpoint);

		intToStr((int)fpart, res + i + 1, afterpoint);
	}
}

void ADC_Init() {
	ADMUX=(1<<REFS0);      // Selecting internal reference voltage
	ADCSRA=(1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);     // Enable ADC also set Prescaler as 128
}

unsigned int ADC_read(unsigned char ch) {
	// select the corresponding channel 0~7
	// ANDing with ’7? will always keep the value
	// of ‘ch’ between 0 and 7
	ch &= 0b00000111;  // AND operation with 7
	ADMUX = (ADMUX & 0xF8)|ch; // clears the bottom 3 bits before ORing

	// start single conversion
	// write ’1? to ADSC
	ADCSRA |= (1<<ADSC);

	// wait for conversion to complete
	// ADSC becomes ’0? again
	// till then, run loop continuously
	while(ADCSRA & (1<<ADSC));

	return (ADC);
}

void lcdCommand( unsigned char commands ) { // commands will be sent from this function
	lcdPort = (lcdPort & 0x0F) | (commands & 0xF0); // send upper nibble of 8 bit
	lcdPort &= ~ (1<<rs);       // rs=0 i.e select command reg
	lcdPort |= (1<<en);         // give high pulse to enable pin
	_delay_us(1);
	lcdPort &= ~ (1<<en);            // give low pulse to enable pin

	_delay_us(200);

	lcdPort = (lcdPort & 0x0F) | (commands << 4);  // sending lower nibble of 8 bit i.e 1byte
	lcdPort |= (1<<en); // give high pulse to enable pin
	_delay_us(1);
	lcdPort &= ~ (1<<en); // give low pulse to enable pin
	_delay_ms(2);
}

void lcdChar( unsigned char string ) {
	lcdPort = (lcdPort & 0x0F) | (string & 0xF0); // send upper nibble
	lcdPort |= (1<<rs);                       // rs=1 i.e select data reg
	lcdPort|= (1<<en); // give high pulse to enable pin
	_delay_us(1);
	lcdPort &= ~ (1<<en); // give low pulse to enable pin

	_delay_us(200);

	lcdPort = (lcdPort & 0x0F) | (string << 4);   //send lower nibble
	lcdPort |= (1<<en); // give high pulse to enable pin
	_delay_us(1);
	lcdPort &= ~ (1<<en); // give low pulse to enable pin
	_delay_ms(2);
}


void lcdString (char *str) {                // convert char to string fucntion
	int j;
	for(j=0; str[j]!=0; j++) {
		lcdChar (str[j]);
	}
}

void lcdClear() {
	lcdCommand (0x01);                   // send hex 01 to Clear display
	_delay_ms(2);
	lcdCommand (0x80);                   // send hex 80 to Cursor at home position
}

void lcdInit() {
	lcdDirection = 0xFC;			// set LCD port direction in output
	_delay_ms(20);					// keep LCD Power ON delay >15ms always

	lcdCommand(0x02);				// send for 4 bit initialization of LCD
	lcdCommand(0x28);				// 2 line, 5*7 matrix in 4-bit mode
	lcdCommand(0x0c);				// Display on cursor off
	lcdCommand(0x06);				// take curson to next position (shift cursor to right)
	lcdCommand(0x01);				// Clear display screen
	_delay_ms(2);					//little delay
}

void lcdSetCursor(char x, char y) {
	if(y) {
		lcdCommand(0x80 + 0x40 + x);
	} else if(!y) {
		lcdCommand(0x80 + x);
	}
}

void lcdCustomChar(char i, char *arr) {
	lcdCommand(0x40 | (8*i));
	for(int j = 0; j <8; j++) {
		lcdChar(arr[j]);
	}
	lcdCommand(0x80);
}

void UART_init(long USART_BAUDRATE) {
	UCSRB |= (1 << RXEN) | (1 << TXEN);	/* Turn on transmission and reception */
	UCSRC |= (1 << URSEL) | (1 << UCSZ0) | (1 << UCSZ1);/* Use 8-bit char size */
	UBRRL = BAUD_PRESCALE;			/* Load lower 8-bits of the baud rate */
	UBRRH = (BAUD_PRESCALE >> 8);		/* Load upper 8-bits*/
}

unsigned char UART_RxChar() {
	while ((UCSRA & (1 << RXC)) == 0);/* Wait till data is received */
	return(UDR);		/* Return the byte */
}

void UART_TxChar(char ch) {
	while (! (UCSRA & (1<<UDRE)));  /* Wait for empty transmit buffer */
	UDR = ch ;
}

void UART_SendString(char *str) {
	unsigned char j=0;

	while (str[j]!=0) {	/* Send string till null */
		UART_TxChar(str[j]);
		j++;
	}
}

double temperature() {
	unsigned int ADC_Count = 0;
	// We assume temperature sensor is connected to ADC channel 0
	ADC_Count = ADC_read(0);

	// Since ATMEGA16 has 10bit ADC 2^10bit=1024
	// Therefore 5V = 1024 ADC Count or 5000mV = 1024
	// Converting from ADC count to voltage
	double voltage = 4.8828125 * ADC_Count;

	// From the function 8mV = 1DegC
	double temprature_reading = 0.125 * voltage;

	char b[6];
	ftoa(temprature_reading, b, 1);
	snprintf(buffer, sizeof(buffer), "Temp: %s C", b);
	UART_SendString(buffer);
	UART_SendString("\r\n");

	lcdSetCursor(0, 1);
	lcdString(buffer);

	return temprature_reading;
}

void temperature_control() {
	if(temperature() < 80.0) {
		ALARM();
	} else {
		NO_ALARM();
	}
}

int main(void) {
	DDRC = 0xFF;
	DDRD = 0xFC;		//set o/p port for LCD

	PORTD = 0x00;
	PORTC = 0x00;

	ADC_Init();

	_delay_ms(1);
	lcdInit();
	lcdClear();

	lcdSetCursor(1, 0);
	lcdString("Hello");
	_delay_ms(1000);

	UART_init(9600);
	UART_SendString("\n\t Echo Test \r\n");

	lcdClear();

	/* Replace with your application code */
	while (1) {
		temperature_control();

		while(ADC_read(1) < 1023);
		for(k = 0; k < N; k++) {
			f[k] = ADC_read(1);    // fetch DFT samples
			_delay_us(31210);
		}

		freq_error = true;

		for(k = 0; k < N/2; k++) {
			x = 0;
			int _vt = 0;
			for(x = 0; x < N; x++) {
				angle = x + (32*k);
				int deadband =  ((f[x] * 1.) / (pgm_read_word(&freq_lookup[angle]) * 1.)) * 100;
				deadband = abs(100 - deadband);

				if(deadband <= 5) {
					_vt += 1;
				}
			}

			if(_vt > ((0.75)*N)) {
				measured_freq = k+1;

				snprintf(buffer, sizeof(buffer), "Freq: %04i Hz", measured_freq);
				UART_SendString(buffer);
				UART_SendString("\r\n");
				freq_error = false;

				lcdSetCursor(0, 0);
				lcdString(buffer);

				if(measured_freq >= 5) {
					ALARM();
				} else {
					NO_ALARM();
				}

				break;
			}
		}
	}

	return 0;
}