/*
 * 
 *
 * 
 *  
 */



#define F_CPU 14745600
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "lcd.h"
#include <math.h>
#define pi 3.14157

volatile int Shaft_Counter_Right_Wheel = 0;
volatile int Shaft_Counter_Left_Wheel = 0;

double reference_distance=100;
double current_x=0,current_y=0,current_theta = 0;
double init_x=0, init_y=0;
unsigned char data;


//Function to configure INT4 (PORTE 4) pin as input for the left position encoder
void Left_Encoder_Pin_Configuration(void)
{
    DDRE  = DDRE & 0xEF;  //Set the direction of the PORTE 4 pin as input
    PORTE = PORTE | 0x10; //Enable internal pull-up for PORTE 4 pin
}


//Function to configure INT5 (PORTE 5) pin as input for the right position encoder
void Right_Encoder_Pin_Configuration(void)
{
    DDRE  = DDRE & 0xDF;  //Set the direction of the PORTE 4 pin as input
    PORTE = PORTE | 0x20; //Enable internal pull-up for PORTE 4 pin
}


void Left_Wheel_Interrupt_Pin(void) //Interrupt 4 enable for the Left Wheel
{
    cli(); //Clears the global interrupt
    EICRB = EICRB | 0x02; // Interrupt 4 is set to trigger at each falling edge only
    EIMSK = EIMSK | 0x10; // Enable Interrupt INT4 for left position encoder
    sei();   // Enables the global interrupt
}


void Right_Wheel_Interrupt_Pin(void) //Interrupt 5 enable for the Right Wheel
{
    cli(); //Clears the global interrupt
    EICRB = EICRB | 0x08; // INT5 is set to trigger with falling edge
    EIMSK = EIMSK | 0x20; // Enable Interrupt INT5 for right position encoder
    sei();   // Enables the global interrupt
}

//Function for interrupt for the Left Wheel
ISR(INT4_vect)
{
    Shaft_Counter_Left_Wheel ++;//It Increases the Left Wheel Counter Shaft value when the Left Wheel Interrupt increases. 
}

//Function for interrupt for the Left Wheel
ISR(INT5_vect)
{
    Shaft_Counter_Right_Wheel ++;//It Increases the Right Wheel Counter Shaft value when the Right Wheel Interrupt increases.
}

//Function To Configure all the Motion Configurations.
void Motion_Configurations()
{
    DDRA = 0x0F;//The Last 4 pins of the register A are set as output
    PORTA = 0x00;//Both wheels at stop;
    DDRL = 0x18;//For Enabling Motor Driver IC
    PORTL = 0x18;

}


void ADC_enable()
{
    DDRF = 0x00;
    DDRK = 0x00;
    ADCSRA = 0x86;
    ADCSRB = 0x00;
    ADMUX = 0x20;
    ACSR = 0x80;
}


void init_devices()
{
    DDRC = 0xF7;
    PORTC = 0x28;
}


void uart0_init(void)
{
    UCSR0B = 0x00; //disable while setting baud rate
    UCSR0A = 0x00;
    UCSR0C = 0x06;
    UBRR0L = 0x5F; //set baud rate lo
    UBRR0H = 0x00; //set baud rate hi
    UCSR0B = 0x98;
}


void initialize()
{
    Motion_Configurations();
    ADC_enable();
    init_devices();

    cli();

    Left_Encoder_Pin_Configuration();
    Right_Encoder_Pin_Configuration();
    Right_Wheel_Interrupt_Pin();
    Left_Wheel_Interrupt_Pin();

    sei();

}


void init_xbee()
{
    cli();
    uart0_init(); //Initialize UART1 for serial communication
    sei();
}



unsigned char Read_Sensor(unsigned char channel)
{
    unsigned char reading;

    if(channel>7)
    {
        ADCSRB = 0x08;
    }

    channel = channel & 0x07;

    ADMUX= 0x20 | channel;

    ADCSRA = ADCSRA | 0x40; //Set start conversion bit

    while((ADCSRA&0x10)==0) {} //Wait for ADC conversion to complete

    reading=ADCH;

    ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it

    ADCSRB = 0x00;

    return reading;

}



void forward_motion()
{
    PORTA = 0x06;
}

void backward_motion()
{
    PORTA = 0x09;
}

void left_motion()                            // Motion functions: moving the bot forward, reverse, left and right
{
    PORTA = 0x05;
}

void right_motion()
{
    PORTA = 0x0A;
}

void stop_motion()
{
    PORTA = 0x00;
}



void velocity (unsigned char left_motor, unsigned char right_motor)
{
    OCR5AL = (unsigned char)left_motor;
    OCR5BL = (unsigned char)right_motor;
}



double get_angle()                               // Return the angle turned by the bot
{
    double angle = 4.090*(Shaft_Counter_Right_Wheel + Shaft_Counter_Left_Wheel)/2;
    return angle;
}



void Left_Rotation_Degrees(int Degrees)
{
    init_x = current_x;
    init_y = current_y;

    /*************************
    88 pulses for 360 degrees rotation 4.090 degrees per count

    *************************/

    left_motion(); //Turn left

    float Reqd_Shaft_Counter = (float) Degrees/ 4.090; // division by resolution to get shaft count
    Reqd_Shaft_Counter = (unsigned int) Reqd_Shaft_Counter;
    Shaft_Counter_Left_Wheel = 0;
    Shaft_Counter_Right_Wheel = 0;
    double initial_theta = current_theta;                      // Rotate bot to the left by a specified angle
    // The angle rotated is measured using data from the shaft encoders

    while (1)
    {
        current_theta = initial_theta - get_angle();
        if(current_theta<0)
        {
            lcd_cursor(1,1);
            lcd_string("-");
            lcd_print(1,2,(-1 * current_theta),4);
        }
        else
        {
            lcd_cursor(1,1);
            lcd_string("+");
            lcd_print(1,2,current_theta,4);
        }
        if((Shaft_Counter_Right_Wheel+Shaft_Counter_Left_Wheel)/2 >= Reqd_Shaft_Counter)
            break;
    }
    stop_motion();
}



void Right_Rotation_Degrees(int Degrees)
{
    init_x = current_x;
    init_y = current_y;

    /*******************
    88 pulses for 360 degrees rotation 4.090 degrees per count

    ********************/

    right_motion(); //Turn right

    float Reqd_Shaft_Counter = (float) Degrees/ 4.090; // division by resolution to get shaft count
    Reqd_Shaft_Counter = (unsigned int) Reqd_Shaft_Counter;
    Shaft_Counter_Left_Wheel = 0;
    Shaft_Counter_Right_Wheel = 0;
    double initial_theta = current_theta;
    // Rotate bot to the right by a specified angle
    // The angle rotated is measured using data from the shaft encoders
    while (1)
    {
        current_theta = initial_theta + get_angle();
        if(current_theta<0)
        {
            lcd_cursor(1,1);
            lcd_string("-");
            lcd_print(1,2,(-1 * current_theta),4);
        }
        else
        {
            lcd_cursor(1,1);
            lcd_string("+");
            lcd_print(1,2,current_theta,4);
        }
        if((Shaft_Counter_Right_Wheel+Shaft_Counter_Left_Wheel)/2 >= Reqd_Shaft_Counter)
            break;
    }
    stop_motion();

}



unsigned int convert(unsigned char reading)                   // Convert the character reading from the ADC to intger value
{
    int dist;
    dist = (int)(10.00*(2799.6*(1.00/(pow(reading,1.1546)))));
    return dist;

}



void coordinate_calculation(double r)
{
    current_x =  init_x + r*sin(current_theta*pi/180.0);             // Calculate coordinates of the bot
    current_y =  init_y + r*cos(current_theta*pi/180.0);
    if(current_x >= 0)
    {
        lcd_cursor(1,13);
        lcd_string("+");
        lcd_print(1,13,current_x,4);
    }
    if(current_x < 0)
    {
        lcd_cursor(1,13);
        lcd_string("-");
        lcd_print(1,13,(-1 * current_x),4);
    }
    if(current_y >= 0)
    {
        lcd_cursor(2,13);
        lcd_string("+");
        lcd_print(2,13,current_y,4);
    }
    if(current_y < 0)
    {
        lcd_cursor(2,13);
        lcd_string("-");
        lcd_print(2,13,(-1 * current_y),4);
    }

}



double get_dist()
{
    /*********************************************

    Radius of circle along which bot moves = 7.6cm
      88 pulses for 360 degree rotation  =>  88 pulses for 2*pi*7.6 = 47.8 cm
    Thus, 0.54 cm per count

    *********************************************/

    double distance_travelled_till_yet = 0.54*(Shaft_Counter_Left_Wheel+Shaft_Counter_Right_Wheel)/2;           // Calculate distance covered using data from the shaft encoders

    coordinate_calculation(distance_travelled_till_yet);                    // Also update coordinates of the bot

    lcd_print(1,2,distance_travelled_till_yet,5);

    return distance_travelled_till_yet;
}



void check_dist_travelled(unsigned int dist)
{
    Shaft_Counter_Left_Wheel = 0;

    Shaft_Counter_Right_Wheel = 0;                      // Keep track of the distance traveled by the bot from the starting point
    // Also continuously keep checking for obstacles
    while (1)
    {
        unsigned char reading=Read_Sensor(11);
        double distance =convert(reading);
        if (distance<reference_distance)
        {
            avoiding_obstacle(distance);
        }

        if (get_dist()>dist)
            break;

    }

    stop_motion();
}



void move_forward(unsigned int dist)
{
    forward_motion();                     // Move the bot forward till it covers the specified distance
    check_dist_travelled(dist);
}



void line_move(double dist, double angle)               // Move the bot along the line previously calculated
{
    if((angle - current_theta)>0)
        Right_Rotation_Degrees(angle - current_theta);

    else if((angle - current_theta)<0)
        Left_Rotation_Degrees(current_theta-angle);

    move_forward((unsigned int)dist);

}



void line_calc(double xfinal,double yfinal)
{
    double slopeangle, dist;

    slopeangle = atan2(xfinal - current_x , yfinal - current_y) * (180/pi);           // Calculate the slope of line and distance to be moved along it
    dist = sqrt(pow(yfinal-current_y,2) + pow(xfinal-current_x,2));

    velocity (100,100);

    line_move(dist, slopeangle);

}



void avoiding_obstacle(double distance)
{
    init_x = current_x;
    init_y = current_y;

    int counter=0;

    while((unsigned int)distance<reference_distance+30)
    {
        Left_Rotation_Degrees(25);                           // Turn the bot 25 degrees repeatedly till line of motion gets clear
        unsigned char reading=Read_Sensor(11);
        distance =convert(reading);

        counter++;
    }

    double move_dist = 10/cos(25*pi*counter/180) + 5;

    line_move(move_dist,current_theta);                 // Move the bot forward till obstacle is cleared

    line_calc(0,0);                                   // Recalculate the line to be traversed
}



void backtracking()
{
    sei();
    line_calc(0,0);
    cli();
}




SIGNAL(SIG_USART0_RECV) 		// ISR for receive complete interrupt
{

    data = UDR0; 				//making copy of data from UDR0 in 'data' variable

    UDR0 = data; 				//echo data back to PC

    Shaft_Counter_Left_Wheel = 0;

    Shaft_Counter_Right_Wheel = 0;

    unsigned char reading=Read_Sensor(11);
    double distance =convert(reading);


    if(data == 0x38 && distance>reference_distance) //ASCII value of 8
    {
        PORTA=0x06;

        init_x = current_x;
        init_y = current_y;

        sei();
        _delay_ms(80);

        PORTA=0x00;

        _delay_ms(10);

        double dist_travelled = (Shaft_Counter_Left_Wheel+Shaft_Counter_Right_Wheel)*0.54;
        cli();

        coordinate_calculation(dist_travelled);
    }


    if(data == 0x32) //ASCII value of 2
    {
        backward_motion(); //back

        sei();
        _delay_ms(80);

        stop_motion();

        _delay_ms(10);

        double dist_travelled = ((Shaft_Counter_Left_Wheel+Shaft_Counter_Right_Wheel)/2)*0.54*2;
        cli();

        coordinate_calculation(-dist_travelled);

        init_x = current_x;
        init_y = current_y;
    }


    if(data == 0x34) //ASCII value of 4
    {
        left_motion();  //left
        sei();
        _delay_ms(50);
        stop_motion();
        _delay_ms(10);
		current_theta-=(get_angle()*3);
        //cli();
		if(current_theta>=0)
        {
            lcd_cursor(1,2);
            lcd_string("+");
            lcd_print(1,3,current_theta,4);
        }
        else
        {
            lcd_cursor(1,2);
            lcd_string("-");
            lcd_print(1,3, (-1 * current_theta),4);
        }
    }


    if(data == 0x36) //ASCII value of 6
    {

        right_motion();  //right

        sei();
        _delay_ms(50);
        stop_motion();
        _delay_ms(10);
		current_theta+=(get_angle()*3);
        //cli();
		
		/*
		To Print the Current theta ( The angle from the Y-Axis )
		The LCD cannot print negative values so that is taken care of
		by the positive and negative signs that are displayed on the LCD
		in case of the respective positive and negative values.		
		*/
		
        if(current_theta>=0)
        {
            lcd_cursor(1,2);
            lcd_string("+");
            lcd_print(1,3,current_theta,4);
        }
        else
        {
            lcd_cursor(1,2);
            lcd_string("-");
            lcd_print(1,3, (-1 * current_theta),4);
        }
    }


    if(data == 0x37) //ASCII value of 7
    {
        backtracking();		//The Backtracking function is called which tells the bot to return to (0,0) coordinates in real space.
    }


}




int main()
{
    initialize();              // Initializes all the ports
    lcd_init();				   // Initializes the LCD
    init_xbee();			   // Initializes the X-Bee
    while(1);					
}