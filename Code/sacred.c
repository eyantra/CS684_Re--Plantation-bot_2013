


#include<avr/io.h>
#include<avr/interrupt.h>
#include<util/delay.h>
#include"lcd.c"

int botId;

volatile int ShaftCountRight = 0;
volatile int ShaftCountLeft = 0;
unsigned char ADC_Conversion(unsigned char);
unsigned char ADC_Value;
unsigned char sharp,sharp1, distance,distance1, adc_reading;
unsigned int value,value1;
unsigned char data;
unsigned char flag = 0;
unsigned char Left_white_line = 0;
unsigned char Center_white_line = 0;
unsigned char Right_white_line = 0;
unsigned char Front_Sharp_Sensor=0;
unsigned char Front_IR_Sensor=0;
unsigned char flag1 = 0;
unsigned char flag2 = 0;

void adc_pin_config (void)
{
 DDRF = 0x00; //set PORTF direction as input
 PORTF = 0x00; //set PORTF pins floating
 DDRK = 0x00; //set PORTK direction as input
 PORTK = 0x00; //set PORTK pins floating
}


void buzzer_pin_config (void)
{
 DDRC = DDRC | 0x08;		//Setting PORTC 3 as outpt
 PORTC = PORTC & 0xF7;		//Setting PORTC 3 logic low to turnoff buzzer
}


void lcd_port_config (void)
{
DDRC = DDRC | 0xF7; //all the LCD pin's direction set as output
PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 except PORTC 7
}



void motion_pin_config (void)
{
DDRA = DDRA | 0x0F;
PORTA = PORTA & 0xF0;
DDRL = DDRL | 0x18; //Setting PL3 and PL4 pins as output for PWM generation
PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}

void adc_init()
{
	ADCSRA = 0x00;
	ADCSRB = 0x00;		//MUX5 = 0
	ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

void buzzer_on (void)
{
 unsigned char port_restore = 0;
 port_restore = PINC;
 port_restore = port_restore | 0x08;
 PORTC = port_restore;
}

void buzzer_off (void)
{
 unsigned char port_restore = 0;
 port_restore = PINC;
 port_restore = port_restore & 0xF7;
 PORTC = port_restore;
}





void servo_1(unsigned char degrees)
{
 	float PositionPanServo = 0;
 	PositionPanServo = ((float)degrees / 2.25) + 21.0;
 	OCR1AH = 0x00;
 	OCR1AL = (unsigned char) PositionPanServo;
}

//Function to rotate Servo 2 by a specified angle in the multiples of 2.25 degrees
void servo_2(unsigned char degrees)
{
 	float PositionTiltServo = 0;
 	PositionTiltServo = ((float)degrees / 2.25) + 21.0;
 	OCR1BH = 0x00;
 	OCR1BL = (unsigned char) PositionTiltServo;
}


//Function to rotate Servo 3 by a specified angle in the multiples of 2.25 degrees
void servo_3(unsigned char degrees)
{
 	float PositionTiltServo = 0;
 	PositionTiltServo = ((float)degrees / 2.25) + 21.0;
 	OCR1CH = 0x00;
 	OCR1CL = (unsigned char) PositionTiltServo;
}


void servo_1_free (void) 	//makes servo 1 free rotating
{
 	OCR1AH = 0x03;
 	OCR1AL = 0xFF; 			//Servo 1 off
}

void servo_2_free (void) 	//makes servo 2 free rotating
{
 	OCR1BH = 0x03;
 	OCR1BL = 0xFF; 			//Servo 2 off
}

void servo_3_free (void) 	//makes servo 3 free rotating
{
 	OCR1CH = 0x03;
 	OCR1CL = 0xFF; 			//Servo 3 off
}



//Function To Initialize UART0
// desired baud rate:9600
// actual baud rate:9600 (error 0.0%)
// char size: 8 bit
// parity: Disabled
void uart0_init(void)
{
 UCSR0B = 0x00; //disable while setting baud rate
 UCSR0A = 0x00;
 UCSR0C = 0x06;
 UBRR0L = 0x5F; //set baud rate lo
 UBRR0H = 0x00; //set baud rate hi
 UCSR0B = 0x98;
}


void timer1_init(void)
{
 	TCCR1B = 0x00; 				//stop
 	TCNT1H = 0xFC; 				//Counter high value to which OCR1xH value is to be compared with
 	TCNT1L = 0x01;				//Counter low value to which OCR1xH value is to be compared with
 	OCR1AH = 0x03;				//Output compare eegister high value for servo 1
 	OCR1AL = 0xFF;				//Output Compare Register low Value For servo 1
 	OCR1BH = 0x03;				//Output compare eegister high value for servo 2
 	OCR1BL = 0xFF;				//Output Compare Register low Value For servo 2
 	OCR1CH = 0x03;				//Output compare eegister high value for servo 3
 	OCR1CL = 0xFF;				//Output Compare Register low Value For servo 3
 	ICR1H  = 0x03;
 	ICR1L  = 0xFF;
 	TCCR1A = 0xAB; 				/*{COM1A1=1, COM1A0=0; COM1B1=1, COM1B0=0; COM1C1=1 COM1C0=0}
 									For Overriding normal port functionalit to OCRnA outputs.
				  				{WGM11=1, WGM10=1} Along With WGM12 in TCCR1B for Selecting FAST PWM Mode*/
 	TCCR1C = 0x00;
 	TCCR1B = 0x0C; 				//WGM12=1; CS12=1, CS11=0, CS10=0 (Prescaler=256)
}


void angle_rotate(unsigned int Degrees)
{
float ReqdShaftCount = 0;
unsigned long int ReqdShaftCountInt = 0;
ReqdShaftCount = (float) Degrees/ 4.090; // division by resolution to get shaft count
ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
ShaftCountRight = 0;
ShaftCountLeft = 0;
while (1)
{
if((ShaftCountRight >= ReqdShaftCountInt) | (ShaftCountLeft >= ReqdShaftCountInt))
break;
}
stop(); //

}



void linear_distance_mm(unsigned int DistanceInMM)
{
float ReqdShaftCount = 0;
unsigned long int ReqdShaftCountInt = 0;
ReqdShaftCount = DistanceInMM / 5.338; // division by resolution to get shaft count
ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;
ShaftCountRight = 0;
while(1)
{
if(ShaftCountRight > ReqdShaftCountInt)
{
break;
}
}
stop(); //Stop robot
}

void port_init_servo(void)
{
	servo1_pin_config();
	servo2_pin_config();
	servo3_pin_config();
}

void servo1_pin_config (void)
{
 	DDRB  = DDRB | 0x20;  		//making PORTB 5 pin output
 	PORTB = PORTB | 0x20; 		//setting PORTB 5 pin to logic 1
}

//Configure PORTB 6 pin for servo motor 2 operation
void servo2_pin_config (void)
{
 	DDRB  = DDRB | 0x40;  		//making PORTB 6 pin output
 	PORTB = PORTB | 0x40; 		//setting PORTB 6 pin to logic 1
}

//Configure PORTB 7 pin for servo motor 3 operation
void servo3_pin_config (void)
{
 	DDRB  = DDRB | 0x80;  		//making PORTB 7 pin output
 	PORTB = PORTB | 0x80; 		//setting PORTB 7 pin to logic 1
}

void init_devices_servo(void)
{
 	cli(); 					//disable all interrupts
 	port_init_servo();
 	timer1_init();
 	sei(); 						//re-enable interrupts
}










void timer5_init()
{
TCCR5A = 0xa9;
TCCR5B = 0x0b;
TCNT5H = 0xff;
TCNT5L = 0x00;
OCR5AH = 0x00;
OCR5AL = 0xff;
OCR5BH = 0x00;
OCR5BL = 0xff;


}


void velocity (unsigned char l , unsigned char r)  //to specify velocity for both wheels
{
OCR5AL = ( unsigned char ) l ;
OCR5BL = ( unsigned char ) r ;
}

void position_control_interuppt()
{
SREG = 0x80;
EICRB = 0x30;

}


void forward()
{
PORTA = 0x06;

}

void back()
{
PORTA = 0x09;
}
void left()
{
PORTA = 0x0a;
}
void right()
{
PORTA = 0x05;
}

void softleft()
{
PORTA = 0x04;
}
void softright()
{
PORTA = 0x02;
}

void stop()
{
PORTA = 0x00;
}


ISR(INT5_vect)
{
   ShaftCountRight++;
}


ISR(INT4_vect)
{
   ShaftCountLeft++;
}


void port_init()
{
lcd_port_config();
motion_pin_config();
left_encoder_pin_config();
right_encoder_pin_config();
adc_pin_config();
buzzer_pin_config();

}




void left_encoder_pin_config()
{
  DDRE = DDRE & 0xEF ;
  PORTE = PORTE | 0x10 ;
}

void right_encoder_pin_config()
{
 DDRE = DDRE & 0xDF ;
 PORTE = PORTE | 0x20 ;
}



void left_position_encoder_interrupt_init()
{
cli();//clear the global interuppt
EICRB = EICRB | 0X02 ;// INT4 is set to trigger with falling edge
EIMSK = EIMSK | 0x10;//Enable Interrupt
sei();

}

void white_line(){            //to follow black line
	while(1)
	{

		Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
		Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
		Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor

		flag=0;

		print_sensor(1,1,3);	//Prints value of White Line Sensor1
		print_sensor(1,5,2);	//Prints Value of White Line Sensor2
		print_sensor(1,9,1);	//Prints Value of White Line Sensor3
		
		

		if(Center_white_line>0x28)
		{
			flag=1;
			forward();
			velocity(220,220);
		}

		if((Left_white_line<0x28) && (flag==0))
		{
			flag=1;
			forward();
			velocity(220,50);
		}

		if((Right_white_line<0x28) && (flag==0))
		{
			flag=1;
			forward();
			velocity(50,220);
		}

		if(Center_white_line<0x28 && Left_white_line<0x28 && Right_white_line<0x28)
		{
			forward();
			velocity(0,0);
			break;
		}

	}
}



void white_line1()        //follow blackline and take a right turn when it ends
{
int flag3=0;
	while(1)
	{

		Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
		Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
		Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor

		flag=0;

		print_sensor(1,1,3);	//Prints value of White Line Sensor1
		print_sensor(1,5,2);	//Prints Value of White Line Sensor2
		print_sensor(1,9,1);	//Prints Value of White Line Sensor3
		
		

		if(Center_white_line>0x28 )
		{
			flag=1;
			forward();
			velocity(220,220);
		}

		if((Left_white_line<0x28) && (flag==0))
		{
			flag=1;
			forward();
			velocity(220,50);
		}

		if((Right_white_line<0x28) && (flag==0) )
		{
			flag=1;
			forward();
			velocity(50,220);
		}

		if(Center_white_line<0x28 && Left_white_line<0x28 && Right_white_line<0x28 )
		{
			while(Center_white_line<0x28)
			{
				Center_white_line = ADC_Conversion(2);
			  left_degrees(10);
			_delay_ms(200);
		
			//velocity(220,220);
			}
			stop();
			_delay_ms(2000);
			right_degrees(10);
				//forward_mm(100);
			velocity(250,250);	
				break;

	
		}

		


	}

}

void white_line2()
{
int flag3=0;
	while(1)
	{

		Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
		Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
		Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor

		flag=0;

		print_sensor(1,1,3);	//Prints value of White Line Sensor1
		print_sensor(1,5,2);	//Prints Value of White Line Sensor2
		print_sensor(1,9,1);	//Prints Value of White Line Sensor3
		
		

		if(Center_white_line>0x28 )
		{
			flag=1;
			forward();
			velocity(220,220);
		}

		if((Left_white_line<0x28) && (flag==0))
		{
			flag=1;
			forward();
			velocity(220,50);
		}

		if((Right_white_line<0x28) && (flag==0) )
		{
			flag=1;
			forward();
			velocity(50,220);
		}

		if(Center_white_line<0x28 && Left_white_line<0x28 && Right_white_line<0x28 )
		{
			while(Center_white_line<0x28)
			{
				Center_white_line = ADC_Conversion(2);
			  right_degrees(10);
			_delay_ms(200);
			
		
			//velocity(220,220);
			}
			stop();
			_delay_ms(2000);
			
			
			left_degrees(10);
				//forward_mm(100);
			velocity(250,250);	
				break;

	
		}

		


	}

}






void right_position_encoder_interrupt_init (void) //Interrupt 5 enable
{
cli(); //Clears the global interrupt
EICRB = EICRB | 0x08; // INT5 is set to trigger with falling edge
EIMSK = EIMSK | 0x20; // Enable Interrupt INT5 for right position encoder
sei();
}


void print_sensor(char row, char coloumn,unsigned char channel)
{
	ADC_Value = ADC_Conversion(channel);
	lcd_print(row, coloumn, ADC_Value, 3);
}


unsigned char ADC_Conversion(unsigned char Ch)
{
	unsigned char a;
	if(Ch>7)
	{
		ADCSRB = 0x08;
	}
	Ch = Ch & 0x07;  			
	ADMUX= 0x20| Ch;	   		
	ADCSRA = ADCSRA | 0x40;		//Set start conversion bit
	while((ADCSRA&0x10)==0);	//Wait for ADC conversion to complete
	a=ADCH;
	ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
	ADCSRB = 0x00;
	return a;
}





void back_mm(unsigned int DistanceInMM)
{
back();
linear_distance_mm(DistanceInMM);
}

void forward_mm(unsigned int DistanceInMM)
{
forward();
linear_distance_mm(DistanceInMM);
}

void left_degrees(unsigned int Degrees)
{// 88 pulses for 360 degrees rotation 4.090 degrees per count
left(); //Turn left
angle_rotate(Degrees);
}


void right_degrees(unsigned int Degrees)
{
// 88 pulses for 360 degrees rotation 4.090 degrees per count
right(); //Turn right
angle_rotate(Degrees);
}

void init_devices()
{

cli();
port_init();
adc_init();
//motion_init();
left_position_encoder_interrupt_init();
right_position_encoder_interrupt_init();
uart0_init();
sei();
}



unsigned int Sharp_GP2D12_estimation(unsigned char adc_reading)
{
	float distance;
	unsigned int distanceInt;
	distance = (int)(10.00*(2799.6*(1.00/(pow(adc_reading,1.1546)))));
	distanceInt = (int)distance;
	if(distanceInt>800)
	{
		distanceInt=800;
	}
	return distanceInt;
}


SIGNAL(SIG_USART0_RECV) 		// ISR for receive complete interrupt
{
	data = UDR0; 				//making copy of data from UDR0 in 'data' variable

	UDR0 = data; 				//echo data back to PC
}

void afterpick()   //function to find blackline and follow it after picking the tree
{
int count =0;
left_degrees(160);
Center_white_line = ADC_Conversion(2);
while(Center_white_line<0x28)
{	Center_white_line = ADC_Conversion(2);
	forward();
}
stop();
while(count<=100)
{
Center_white_line = ADC_Conversion(2);
	while(Center_white_line>0x28)
	{	Center_white_line = ADC_Conversion(2);
		forward();
	
	}
	stop();
	right_degrees(10);
	_delay_ms(500);
	count += 10;
}
//Center_white_line = ADC_Conversion(2);
//while(Center_white_line<0x28)
//{
//left_degrees(10);
//_delay_ms(500);
//}
white_line();


}


int main ()
{

unsigned int value;
unsigned int min;
unsigned int i;
 botId = 12;

    init_devices();
    timer5_init();
	lcd_set_4bit();
	lcd_init();
    init_devices_servo();
//	forward_mm(100);
	stop();
	//white_line1();
	


//	forward_mm(100);
	while(1)
{
				//Stores Distance calsulated in a variable "value".
	//lcd_cursor(1,3);
	//lcd_string("ravi");
	sharp = ADC_Conversion(11);						//Stores the Analog value of front sharp connected to ADC channel 11 into variable "sharp"
		value = Sharp_GP2D12_estimation(sharp);	
	
		lcd_print(2,14,value,3);	
	sharp1 = ADC_Conversion(10);						//Stores the Analog value of front sharp connected to ADC channel 11 into variable "sharp"
		value1 = Sharp_GP2D12_estimation(sharp1);	
	
		lcd_print(1,1,value1,3);	

if(data == 0x31)   // to start the bot press 1 for replantation
{
	min=800;
	_delay_ms(1000);

for (i = 10; i <90; i++)
 {
  servo_2(i);
  _delay_ms(100);
  }
  _delay_ms(500);
  servo_2_free();

//open mouth
for (i = 180; i > 0; i--)
 {
  servo_3(i);
  _delay_ms(100);
  }

  	_delay_ms(500);
	servo_3_free();
	_delay_ms(500);
	//servo_3_free();
	servo_1(90);


right_degrees(110);

	while(1)
	{

		left_degrees(5);
		_delay_ms(1000);

	sharp1 = ADC_Conversion(11);						//Stores the Analog value of front sharp connected to ADC channel 11 into variable "sharp"
		value1 = Sharp_GP2D12_estimation(sharp1);	
	
		lcd_print(2,14,value1,3);		
	    sharp = ADC_Conversion(10);						//Stores the Analog value of front sharp connected to ADC channel 11 into variable "sharp"
		value = Sharp_GP2D12_estimation(sharp);	
		lcd_print(1,1,value,3);		
		
		if(value > min && min < 400)
		{
			right_degrees(5);
			if(min >150)
			{
				forward_mm(50);
				_delay_ms(1000);
				right_degrees(10);
				min = 400;
			}
			else
				{	if(min<150 )
						forward_mm(min-20);
						_delay_ms(1000);
						break;
				}
		}
		if(min>value )
		{
			min=value;

		}
		//if(min<230)
		//	break;
		
	}
		
for (i = 0; i <150; i++)
 {
  servo_3(i);
  _delay_ms(30);
  }

	_delay_ms(3000);
	servo_3_free();

for (i = 80; i > 9; i--)
 {
  servo_2(i);
  _delay_ms(100);
  }
  	_delay_ms(3000);
	servo_3_free();

//	servo_2(20);
//	_delay_ms(6000);
	//servo_2(10);
	_delay_ms(1000);
	afterpick();
		velocity(220,220);
	white_line1();
	velocity(250,250);
	forward_mm(100);
for(int j = 80;j > 0 ; j--)
{		
	servo_1(j);
	_delay_ms(40);
}	
	_delay_ms(500);
	servo_1_free();
	
for(int j = 10;j < 80 ; j++)
{		
	servo_2(j);
	_delay_ms(40);
}	
	
	_delay_ms(500);
	
	servo_2_free();
for (i = 150; i > 0; i--)
 {
  servo_3(i);
  _delay_ms(30);
  }
	_delay_ms(2000);
	servo_3_free();
	servo_2(20);
	_delay_ms(2000);
	servo_2_free();

	left_degrees(150);
Center_white_line = ADC_Conversion(2);
while(Center_white_line<0x28)
{
left_degrees(10);
_delay_ms(200);
Center_white_line = ADC_Conversion(2);

}
	

	white_line();
	velocity(220,220);
	forward_mm(40);
	right_degrees(80);
Center_white_line = ADC_Conversion(2);
while(Center_white_line<0x28)
{
right_degrees(10);
_delay_ms(200);
Center_white_line = ADC_Conversion(2);

}
	
//	white_line();
	_delay_ms(1000);
	white_line();

//take position again by reversing its mouth
velocity(220,220);
for(int i= 0;i< 16;i++)
{
right_degrees(10);

}
Center_white_line = ADC_Conversion(2);
while(Center_white_line<0x28)
{
right_degrees(10);
_delay_ms(200);
Center_white_line = ADC_Conversion(2);

}

	data =0x00;
}



		if(data == 0x38) //ASCII value of 8
		{
			forward_mm(20);  //forward
			data = 0x00;
		}

		if(data == 0x33) //ASCII value of 8
		{
			servo_3(180);  //forward
			data = 0x00;
		}

		if(data == 0x37) 
		{
			servo_3(90); 
			data = 0x00;
		}
	
		if(data == 0x39) //ASCII value of 8
		{
			servo_3(0);  //forward
			data = 0x00;
		}

		if(data == 0x32) //ASCII value of 2
		{
		 	
			 for (i = 0; i <180; i++)
 				{
 					 servo_2(i);
					  _delay_ms(100);
 // servo_2(i);
 				}
					  _delay_ms(3000);

 // servo_3(i);
 // _delay_ms(30);
 			for (i = 180; i > 0; i--)
 				{
  					servo_2(i);
					  _delay_ms(100);
  				}

 		}
		
 

		if(data == 0x34) //ASCII value of 4
		{
			PORTA=0x05;  //left
			data = 0x00;
		}

		if(data == 0x36) //ASCII value of 6
		{
			PORTA=0x0A; //right
			data = 0x00;
		}

		if(data == 0x35) //ASCII value of 5
		{
			PORTA=0x00; //stop
			data = 0x00;
		}

//		if(data == 0x37) //ASCII value of 7
//		{
//			white_line();
			//buzzer_on();
//			data = 0x00;
//		}

//		if(data == 0x39) //ASCII value of 9
//		{
//			buzzer_off();
//			data = 0x00;
//		}


}


}
