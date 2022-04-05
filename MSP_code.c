#include "io430.h"
#include <math.h>
#include <intrinsics.h> 

void Delay(long n)
{
while (n--);
}

void putc(float c)
{
while (!IFG2_bit.UCA0TXIFG);
UCA0TXBUF = c;
}

unsigned char getc(void)
{
while(!IFG2_bit.UCA0RXIFG);
return(UCA0RXBUF);
}

float PID(void)
{
  int dt = 0.01;
  int Kp,Ki,Kd,error,r,integral,temperature,integral_old,derivative,error_old;
  Kp = getc(); // Get proportional parameter
  Ki = getc(); // Get integral parameter
  Kd = getc(); // Get derivative parameter
  //r=getc();//Get desired temp from MATLAB (setting point) 
  error = r - temperature; // calculate error signal  //y' is temperatue 
  integral = integral_old + error*dt; // integrate error signal
  derivative = (error - error_old)/dt; // differentiate error signal
  
return (Kp*error + Ki*integral + Kd*derivative)/100; // output of PID
}

void PWM(float d)
{
TA1CCR1 = (unsigned int)(d*1000); //The period in number of clock cycles ...

}

int main( void )
{
  // Stop watchdog timer to prevent time out reset
  WDTCTL = WDTPW + WDTHOLD;
 // set system clock to calibrated values
DCOCTL = CALDCO_16MHZ;
BCSCTL1 = CALBC1_16MHZ;

  //Initialze MSP430 and PWM pin
     //Init();
     //Init_UART();
     //Init_ADC();
     P2SEL = 0;                
     P2OUT = 0;
     P2DIR = BIT6|BIT7;
     P2DIR |= BIT1; //set P2.1 for current direction
     P2SEL |= BIT5; //set P2.5 as PWM pin
     
     //Initilize ADC module
     ADC10CTL0 = SREF_0 | ADC10SHT_2 | ADC10ON;
     ADC10CTL1 = INCH_1 | SHS_0 | ADC10DIV_0 | ADC10SSEL_0 | CONSEQ_0;
     ADC10AE0 = BIT1;            // Enable analog input on channel 1 // on P1.1 ????
     ADC10CTL0_bit.ENC = 1;            // Enable conversions
     
     //Initialize UART
     UCA0BR1 = 0;
     UCA0BR0 = 139;
     // select UART clock source 
     UCA0CTL1_bit.UCSSEL1 = 1; 
     UCA0CTL1_bit.UCSSEL0 = 0;
     // release UART RESET
     UCA0CTL1_bit.UCSWRST = 0;
     
     
     float y,v,tem,rth,temperature;
     int r,u,duty,error_old , error,integral_old , integral;
  
  while (1)
  {
    
    //Receive r(t), set point temp from matlab gui 
    r = getc();
    //value of ADC
    ADC10CTL0_bit.ADC10SC = 1;
    putc(ADC10MEM >> 2); //bitshfit we have to do to go to matlab  (ADC temp going to matlab) 
    
    //Below has to be incorportated in MATLAB. Only send the raw 8 bit (prev 10 bit but shifted by 2) data to MATLAB from the ADC pin that connects to thermosistor ////////////////////
    
    y =  ADC10MEM;  //ADC input from thermistor, regisiter that contains value bw 0 to 1023 (ADC temp in C) /// ADC10MEM is the regisiter that saves  ADC10AE0 = BIT1 from the thermosistor  
    //convert y to voltage
    v =  (y*3.3/1023);
    //determine thermestor resistance value
    int R = 10000; //10 kohm resistor
    tem = (1/3.3-1/R);
    rth =  (v/(3.3-v))*(1/tem);

    temperature = (-1000/41*log(rth/35.963));  //the trend we see after we plot the data in excel, relationship w resistance, uses 10 bits  MEASURED TEMP   WHAT WE NEED TO SEND TO MATLAB 
    
 
    
    ///////////////////
    
    //temperature = (35.964*exp(-0.041*rth));
    
    u = PID(); // get PID values and compute the correction signal
    
    if (u>100) // correction signal should not be greater than 100%
        duty = 0.99;
    else if (u<0) // correction signal should not be less than 0%
        duty = 0.01;
    else // convert correction signal from percentage to decimal fraction
        duty = u/100;
    
    error_old = error; // save current error for next differentiation
    integral_old = integral; // save current integral for next integration
    PWM(duty);
    
    Delay(1000);
    
    
    
  }

  return 0;
}