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


void Left_Wheel_Interrupt_Pin(void) //Interrupt 4 enable
{
    cli(); //Clears the global interrupt
    EICRB = EICRB | 0x02; // INT4 is set to trigger with falling edge
    EIMSK = EIMSK | 0x10; // Enable Interrupt INT4 for left position encoder
    sei();   // Enables the global interrupt
}


void Right_Wheel_Interrupt_Pin(void) //Interrupt 5 enable
{
    cli(); //Clears the global interrupt
    EICRB = EICRB | 0x08; // INT5 is set to trigger with falling edge
    EIMSK = EIMSK | 0x20; // Enable Interrupt INT5 for right position encoder
    sei();   // Enables the global interrupt
}

//Functions for incrementing the shaft encoder value
ISR(INT4_vect)
{
    Shaft_Counter_Left_Wheel ++;
}

ISR(INT5_vect)
{
    Shaft_Counter_Right_Wheel ++;
}


void Motion_Configurations()
{
    DDRA = 0x0F;
    PORTA = 0x00;
    DDRL = 0x18;
    PORTL = 0x18;

}


void ADC_enable()         
{
	// Function to enable the ADC and initialize the required registers
	
    DDRF = 0x00;
    DDRK = 0x00;
    ADCSRA = 0x86;
    ADCSRB = 0x00;
    ADMUX = 0x20;
    ACSR = 0x80;
}


void init_devices()          // 
{
    DDRC = 0xF7;
    PORTC = 0x28;
}


/* Function To Initialize UART0
   desired baud rate:9600
   actual baud rate:9600 (error 0.0%)
   char size: 8 bit
   parity: Disabled  */

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
	// Function to call all the functions initializing the ports
	
    Motion_Configurations();
    ADC_enable();
    init_devices();

    cli();       // Clears the global interrupts

    Left_Encoder_Pin_Configuration();
    Right_Encoder_Pin_Configuration();
    Right_Wheel_Interrupt_Pin();
    Left_Wheel_Interrupt_Pin();

    sei();       // Enables the global interrupts

}


void init_xbee()               
{   
    // Calling the function to initialize serial communication via XBee
	
	cli();
    uart0_init(); //Initialize UART1 for serial communication
    sei();
}



unsigned char Read_Sensor(unsigned char channel)       
{  
	// Function to read the value from ADC and return it as an unsigned character
    
	unsigned char reading;

    if(channel>7)
    { ADCSRB = 0x08;
       }

    channel = channel & 0x07;
    ADMUX= 0x20 | channel;
    ADCSRA = ADCSRA | 0x40;     //Set start conversion bit

    while((ADCSRA&0x10)==0);  //Wait for ADC conversion to complete

    reading=ADCH;
    ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
    ADCSRB = 0x00;

    return reading;

}


//------------------------------------------------------------------------
// Motion functions: moving the bot forward, reverse, left and right

void forward_motion()
{
    PORTA = 0x06;
}

void backward_motion()
{
    PORTA = 0x09;
}

void left_motion()                           
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

//-----------------------------------------------------------------------



void velocity (unsigned char left_motor, unsigned char right_motor)       // Function to initialize the registers for motion control using PWM
{
    OCR5AL = (unsigned char)left_motor;
    OCR5BL = (unsigned char)right_motor;
}



double get_angle()                               // Return the angle turned by the bot
{
   
    /*************************
    88 pulses for 360 degrees => 4.090 degrees per count
	The angle rotated is calculated by measuring the number of counts 
	and multiplying it by 4.090 to get the degrees

    *************************/

    double angle = 4.090*(Shaft_Counter_Right_Wheel + Shaft_Counter_Left_Wheel)/2;
    return angle;
}



void Left_Rotation_Degrees(int Degrees)
{
    /* Function to rotate bot to the left by a specified angle
       The angle rotated is measured using data from the shaft encoders    
	  */
	
	init_x = current_x;
    init_y = current_y;

    left_motion(); //Turn left

    float Reqd_Shaft_Counter = (float) Degrees/ 4.090;                // division by resolution to get shaft count
    Reqd_Shaft_Counter = (unsigned int) Reqd_Shaft_Counter;
    Shaft_Counter_Left_Wheel = 0;
    Shaft_Counter_Right_Wheel = 0;
    double initial_theta = current_theta;                      

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
     /* Rotate bot to the right by a specified angle
        The angle rotated is measured using data from the shaft encoders
       */	 
	 
	init_x = current_x;
    init_y = current_y;

    right_motion(); //Turn right

    float Reqd_Shaft_Counter = (float) Degrees/ 4.090; // division by resolution to get shaft count
    Reqd_Shaft_Counter = (unsigned int) Reqd_Shaft_Counter;
    Shaft_Counter_Left_Wheel = 0;
    Shaft_Counter_Right_Wheel = 0;
    double initial_theta = current_theta;
    
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



unsigned int convert(unsigned char reading)                  
 {
     // Function to convert the character reading from the ADC to integer value
    
	int dist;
    dist = (int)(10.00*(2799.6*(1.00/(pow(reading,1.1546)))));
    return dist;

}



void coordinate_calculation(double r)
{
    current_x =  init_x + r*sin(current_theta*pi/180.0);             // Function to calculate coordinates of the bot and changing coordinates to Cartesian system from polar coordinates 
    current_y =  init_y + r*cos(current_theta*pi/180.0);
    if(current_x >= 0)
    {
        lcd_cursor(1,13);											 //Printing the x-coordinate on the LCD
        lcd_string("+");
        lcd_print(1,13,current_x,4);
    }
    if(current_x < 0)
    {
        lcd_cursor(1,13);
        lcd_string("-");
        lcd_print(1,13,(-1 * current_x),4);
    }
    if(current_y >= 0)												 //Printing the y-coordinate on the LCD
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
    if((angle - current_theta)>0)                      //if rotation angle is positive it starts right rotation.
        Right_Rotation_Degrees(angle - current_theta);  //aligns the bot such that it face towards the final point.

    else if((angle - current_theta)<0)                  //if rotation angle is negative it starts left rotation.
        Left_Rotation_Degrees(current_theta-angle);     //aligns the bot such that it face towards the final point.

    move_forward((unsigned int)dist);                   //bot starts moving forward towards the final point.

}



void line_calc(double xfinal,double yfinal)
{
    double slopeangle, dist;

    slopeangle = atan2(xfinal - current_x , yfinal - current_y) * (180/pi);  // Calculate the slope of line between the current position of the bot and the final point.  
    dist = sqrt(pow(yfinal-current_y,2) + pow(xfinal-current_x,2));          //Calculates distance to be moved along the line calculated above.

/**************************************************************************************************************************************************
While retreating back to its original position bot was showing around 15% error which was not acceptable.
So after large number of hit and trials we realized that if we reduce the speed of bot error reduces 
significantly.
So we reduced the Pulse Width Modulation(PWM) of the motor which very much increased the accuracy of the bot.
Thus we reduced the speed of the bot and set it to (100,100)
**************************************************************************************************************************************************/

    velocity (100,100);                                                            

    line_move(dist, slopeangle);                                                    //bot starts moving along the calculated line.  

}



void avoiding_obstacle(double distance)
{
    init_x = current_x;                                       //sets the initial co-ordinates to the the co-ordinates where the bot detected the obstacle.
    init_y = current_y;                                       // new initial co-ordinated are the new node.distance travelled will now be measured from this point.
/*********************************************************************************************************************************
To minimize the returning path of the bot it was necessary that bot clears the obstacle proportional to the size of the obstacle.
To implement this we defined a counter which measures how many times bot have to turn to get clear from obstacle.Now the bot is
moved proportional to the counter thus clearing the obstacle and also reducing the path length at the same time.
*********************************************************************************************************************************/
    int counter=0;                                         //initializing the counter value to zero.   

    while((unsigned int)distance<reference_distance+30)
    {
        Left_Rotation_Degrees(25);                           // Turn the bot 25 degrees repeatedly till line of motion gets clear
        unsigned char reading=Read_Sensor(11);
        distance =convert(reading);

        counter++;                                           //updating the counter.
    }

    double move_dist = 10/cos(25*pi*counter/180) + 5;        //moving the distance proportional to the counter.

    line_move(move_dist,current_theta);                 // Move the bot forward till obstacle is cleared

    line_calc(0,0);                                   // Recalculate the line to be traversed
}

<<<<<<< HEAD

/*
This function activates the BCAS ( Bot Collision Avoidance System ) and thereby tells the bot to return to (0,0) 
=======
/*************************************************
This function activates the ARA ( Auto Return Algorithm ) and thereby tells the bot to return to (0,0) 
>>>>>>> origin/master
Therefore the function line_calc is called which calculates the required direction and distance to move to (0,0)
and then tells the bot to rotate till that direction and start moving that required distance
in that direction.
*************************************************/

void backtracking()
{
    sei();
    line_calc(0,0);
    cli();
}




SIGNAL(SIG_USART0_RECV) 		// ISR for receive complete interrupt
{

    data = UDR0; 				//making copy of data from UDR0 in 'data' variable

    UDR0 = data; 				//echo data back to PC so that we get to know that the data is recieved at the bot

    Shaft_Counter_Left_Wheel = 0;

    Shaft_Counter_Right_Wheel = 0;

    unsigned char reading=Read_Sensor(11);
    double distance =convert(reading);

		/*
		The Forward/Backward motion is switched on for 50 ms for a little forward/backward motion on pressing 8/2 on the keyboard once.
		After 50 ms the motor is stopped but still a delay of 10 ms is provided to let the motor die down completely.
		The required transversed distance is calculated from the shaft counters.
		Then the new coordinates are calculated using the coordinate_calculation function in which distance travelled from the previous
		node is mentioned
		A new node is formed after that which is further used for any further motions.
		*/

    if(data == 0x38 && distance>reference_distance) //ASCII value of 8
    {
        forward_motion(); // Forward motion starts.

        init_x = current_x;
        init_y = current_y;

        sei();
        _delay_ms(80);
        stop_motion();
		_delay_ms(10);
        double dist_travelled = (Shaft_Counter_Left_Wheel+Shaft_Counter_Right_Wheel)*0.54;
        cli();

        coordinate_calculation(dist_travelled);
    }


    if(data == 0x32) //ASCII value of 2
    {
        backward_motion(); //Backward Motion starts

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

		/*
		The Left/Right motion is switched on for 50 ms for a little left/right rotation on pressing 4/6 on the keyboard once.
		After 50 ms the motor is stopped but still a delay of 10 ms is provided to let the motor die down completely.
		The required turned angle is called on from the get_angle function and the calibrated value is subtracted/added to the current theta.
		Thus the value of the global variable current_theta is updated.
		*/

    if(data == 0x34) //ASCII value of 4
    {
        left_motion();  // Left Motion starts.
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

        right_motion();  // Right motion starts.
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
