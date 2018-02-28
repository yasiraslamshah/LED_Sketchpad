/*
*This is the code for LED_sketchpad Project 
* Project was done with a team of two people
* Author : Touchscreen Driver : Gautham K A
*	   LED Dot Matrix : Yasir Aslam shah
* Date : Dec 2017
* Final Project for Embedded System Design Course 2017
*/ 

#include "msp.h"
#include <stdint.h>



#include <stdbool.h>

volatile uint16_t TXData = 1;            // for UART
volatile uint8_t RXData = 0;
volatile uint16_t store_x[10000];               // Store ADC VALUES
volatile uint16_t store_y[10000];
volatile uint16_t num;                          // COUNT OF NUMNER OF VALID TOUCH SAMPLES STORED
volatile uint16_t ta;                           // for initial polling for display module
uint16_t disp,i ,j ;
#define SETTLING 100                            // settling delay
#define DEBOUNCE 400                            // debounce delay
#define numb_samples 5
volatile uint16_t x_values [numb_samples];
volatile uint16_t y_values [numb_samples];
void read_xval();
void read_yval();
void read_ch_xval();
void fun();
void matrix_map(int a,int b);
void matrix_ini();
void read_ch_yval();
void led_green (void);
void led_red (void);
//void  send_data();
unsigned int touch_detect();
void timer_delay( unsigned int count);
void error(void);

/*UART CONFIGURATION using DRIVERLIB from TI for MSP432 */
/*
const eUSCI_UART_Config uartConfig =
{
        EUSCI_A_UART_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
        156,                                      // BRDIV = 13
        4,                                       // UCxBRF = 0
        0,                                      // UCxBRS = 37
        EUSCI_A_UART_NO_PARITY,                  // No Parity
        EUSCI_A_UART_LSB_FIRST,                  // lSB First
        EUSCI_A_UART_ONE_STOP_BIT,               // One stop bit
        EUSCI_A_UART_MODE,                       // UART mode
        EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION  // Oversampling
};
*/

void main(void)
    {
                uint32_t currentPowerState;
                uint16_t x,y;
                //uint8_t x1,x2,y1,y2;
                uint16_t count1 = 0;

               uint16_t x1[10000],y1[10000];
                num = 0;
                ta =0;
                disp =0;
                i=0;
                j=0;
                WDT_A->CTL = WDT_A_CTL_PW |             // Stop watchdog timer
                             WDT_A_CTL_HOLD;
                currentPowerState = PCM->CTL0 & PCM_CTL0_CPM_MASK;       // Checking powerstates as it runs on higher clock freq

                if (currentPowerState != PCM_CTL0_CPM_0)
                        error();

                while ((PCM->CTL1 & PCM_CTL1_PMR_BUSY));
                PCM->CTL0 = PCM_CTL0_KEY_VAL | PCM_CTL0_AMR_1;
                while ((PCM->CTL1 & PCM_CTL1_PMR_BUSY));
                if (PCM->IFG & PCM_IFG_AM_INVALID_TR_IFG)
                    error();                            // Error if transition was not successful
                if ((PCM->CTL0 & PCM_CTL0_CPM_MASK) != PCM_CTL0_CPM_1)
                    error();                            // Erro0r if device is not in AM1_LDO mode

                /* Step 2: Configure Flash wait-state to 1 for both banks 0 & 1 */
                FLCTL->BANK0_RDCTL = (FLCTL->BANK0_RDCTL & ~(FLCTL_BANK0_RDCTL_WAIT_MASK)) |
                        FLCTL_BANK0_RDCTL_WAIT_1;
                FLCTL->BANK1_RDCTL  = (FLCTL->BANK0_RDCTL & ~(FLCTL_BANK1_RDCTL_WAIT_MASK)) |
                        FLCTL_BANK1_RDCTL_WAIT_1;
                //MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
                          //GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
                 // MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
                //  MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
                 //  CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_48);     // frequency for UART module


                CS->KEY = CS_KEY_VAL ;                  // Unlock CS module for register access
                CS->CTL0 = 0;                           // Reset tuning parameters
                CS->CTL0 |= ( CS_CTL0_DCORSEL_5 | CS_CTL0_DCOEN );   // DCO (oscillator ) frequency range
                CS->CTL1 = 0;
                CS->CTL1 |= (CS_CTL1_SELM__DCOCLK | CS_CTL1_DIVM_0 ) ; // clock source for MCLK : DCO
                CS->KEY = 0;


                touch_detect();                   // initialise pins for touch detect operation
                matrix_ini();                        // initialise pins for Led matrix dispaly
                //P5->SEL0 |= BIT6  ;      // Enable vref+ vref- for adc (external ref voltage)
                //P5->SEL0 |= BIT7  ;
                //P5->SEL1 |= BIT6  ;
                //P5->SEL1 |= BIT7  ;

                P5->IFG = 0;
                P5->IES |= BIT2;                         // Interrupt on high-to-low transition
                                          // Clear all P1 interrupt flags
                P5->IE |= BIT2;

                P1->DIR = ~(uint8_t) BIT1;               // enable pin for switch operation - clear LED display
                P1->OUT = BIT1;
                P1->REN = BIT1;                         // Enable pull-up resistor (P1.1 output high)
                P1->SEL0 = 0;
                P1->SEL1 = 0;
                P1->IES = BIT1;                         // Interrupt on high-to-low transition
                P1->IFG = 0;                            // Clear all P1 interrupt flags
                P1->IE = BIT1;                          // Enable interrupt for P1.1
               // MAP_UART_initModule(EUSCI_A0_BASE, &uartConfig);

               /* Enable UART module                 */
               // MAP_UART_enableModule(EUSCI_A0_BASE);
                /* Enabling interrupts */
              //  MAP_UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
              // MAP_Interrupt_enableInterrupt(INT_EUSCIA0);
                ADC14->CTL1 = 0;
                ADC14->CTL1 |=  ADC14_CTL1_RES__10BIT ;
                // Configuration of ADC registers
                ADC14->CTL0 = ADC14_CTL0_ON |
                ADC14_CTL0_MSC |
                ADC14_CTL0_SHT0__192 |
                ADC14_CTL0_SHP |
                ADC14_CTL0_CONSEQ_0 | ADC14_CTL0_SSEL__MCLK ;
                // Enable Port 1 interrupt on the NVIC
                NVIC->ISER[1] = 1 << ((PORT5_IRQn) & 31);
                //NVIC->ISER[1] = 1 << ((PORT1_IRQn) & 31);
                // Enable global interrupt
                __enable_irq();



    while(1)
    {
         while(ta == 0);                   // polling for touch

                    while((disp+1) < num){            // storing values for display
                    x = store_x[disp];
                    y= store_y[disp];
                    disp++;
                    y= y /100;
                    y =y+1;

                    if(y>8)
                        y = 8;
                    y1[i] = y ;
                    x = x/100;
                       x =x+1;
                       if(x>8)
                           x =8;
                    x1[i]= x;
                    i++;
                }

                    matrix_map(y1[j],x1[j]);              // mapping values in to LED matrix
                    timer_delay(1);                  // delay to show pattern as animation : reducing it to 1 will make the image stable
                    matrix_ini();
                    j++;
                    if( j == num){
                        j =0;
                    }







    }
}

/*  Interrupt routine used for clearing LED display*/

void PORT1_IRQHandler(void)
{
    __disable_irq();
    volatile uint32_t i;

    ta =0;
    num =0;
    disp =0;
    i =0;
    j =0;

    // Delay for switch debounce
    for(i = 0; i < 10000; i++)

    P1->IFG &= ~BIT1;
    __enable_irq();
}


/*  interrupt routine used for touch operation*/
void PORT5_IRQHandler(void)
{
    uint16_t x_val ,y_val, k,a;

    //P5->IFG = 0;
    timer_delay(DEBOUNCE);                    // debouncing delay for touchscreen
    if( touch_detect() )                        // if touch is detected enter for further processing
    {
       do{
           //led_red();                       // led blinking used for testing
           read_ch_yval();                     // to read Y coordinate
           timer_delay(SETTLING);               // settling delay for pin configuration
           read_ch_xval();                          // to read x value
           x_val = 0 ;             //initial value
           for(k =0 ; k < numb_samples; k++)
           {
               x_val = x_val + x_values[k];

           }
           x_val = (x_val / numb_samples);


           y_val = 0 ;             //initial value
           for(k =0 ; k < numb_samples; k++)
           {
               y_val = y_val + y_values[k];

           }
           y_val = y_val / numb_samples;

           store_x[num] = x_val;
           store_y[num] = y_val;
           num++;
           a = touch_detect();
           ta = 1;

           }while(a == 1);              // loop if touchscreen is touched still

     }


    P5->IFG = 0;             // clear interrupt flag
    //led_green();



}


/* Pin configuration used for reading Y Value*/
void read_yval()
{

                P5->SEL1 |= BIT2  ;      // Enable A/D channel A1 pin 5.2 x+
                P5->SEL0 |= BIT2  ;

                P5->SEL1 &= ~BIT5  ;
                P5->SEL0 &= ~BIT5 ;
                P5->DIR |= BIT5;
                P5->DS |= BIT5;
                P5->OUT |= BIT5;         // PIN 5.5 IS y+ - vcc


                P4->SEL1 &= ~BIT2  ;
                P4->SEL0 &= ~BIT2 ;
                P4->DIR &= ~BIT2;
                P4->REN &= ~BIT2;
                P4->DS |= BIT2;
                P4->OUT &= ~BIT2;         // PIN 4.2 IS X- - GND -- open


                P2->SEL1 &= ~BIT5  ;
                P2->SEL0 &= ~BIT5 ;
                P2->DIR |= BIT5;
                P2->DS |= BIT5;
                P2->OUT &= ~BIT5;   // PIN 2.5 IS y- - GND

                timer_delay(SETTLING);




}

/*
void EUSCIA0_IRQHandler(void)
{
    uint32_t status = MAP_UART_getEnabledInterruptStatus(EUSCI_A0_BASE);

    MAP_UART_clearInterruptFlag(EUSCI_A0_BASE, status);

    if(status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
    {
        RXData = MAP_UART_receiveData(EUSCI_A0_BASE);

        if(RXData != TXData)              // Check value
        {
            MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
            while(1);                       // Trap CPU
        }
        TXData++;
        MAP_Interrupt_disableSleepOnIsrExit();
    }

}

*/
/* Pin configuration used for reading X Value*/

void read_xval()
{
                P5->SEL1 &= ~BIT2 ;
                P5->SEL0 &= ~BIT2 ;
                P5->DIR |= BIT2;
                P5->DS |= BIT2;
                P5->OUT |= BIT2;         // PIN 4.5 IS X+ - vcc - or

                P5->SEL1 |= BIT5  ;      // Enable A/D channel A0 pin 5.5 -yel
                P5->SEL0 |= BIT5  ;

                P4->SEL1 &= ~BIT2  ;
                P4->SEL0 &= ~BIT2 ;
                P4->DIR |= BIT2;
                P4->DS |= BIT2;
                P4->OUT &= ~BIT2;         // PIN 2.5 IS X- - GND - gre


                P2->SEL1 &= ~BIT5  ;
                P2->SEL0 &= ~BIT5 ;
                P2->DIR &= ~BIT5;
                P2->REN &= ~BIT5;
                P2->DS |= BIT5;
                P2->OUT &= ~BIT5;            // PIN 2.6 IS y- - GND - red -- open


                timer_delay(SETTLING);

}

/* Pin configuration used for touch detection interrupt*/
unsigned int touch_detect()
{
                   P5->SEL1 &= ~BIT2  ;
                   P5->SEL0 &= ~BIT2 ;
                   P5->DIR &= ~BIT2;
                   P5->REN |= BIT2;
                   P5->DS |= BIT2;
                   P5->OUT |= BIT2;            // x+ - measure


                   P5->SEL1 &= ~BIT5  ;
                   P5->SEL0 &= ~BIT5 ;
                   P5->DIR &= ~BIT5;
                   P5->REN &= ~BIT5;
                   //P5->DS |= BIT5;
                   P5->OUT &= ~BIT5;            //y+ open

                   P4->SEL1 &= ~BIT2  ;
                   P4->SEL0 &= ~BIT2 ;
                   P4->REN &= ~BIT2;             //x- open
                   P4->DIR &= ~BIT2;
                   //P4->DS |= BIT2;
                   P4->OUT &= ~BIT2;


                   P2->SEL1 &= ~BIT5  ;
                   P2->SEL0 &= ~BIT5 ;
                   P2->DIR |= BIT5;
                   P2->REN |= BIT5;
                   P2->DS |= BIT5;                     //y- gnd
                   P2->OUT &= ~BIT5;


                timer_delay(SETTLING);

                if( (P5IN & 0x04) == 0 )
                    return 1;
                else
                    return 0;



}



void timer_delay( unsigned int count)
{

    TIMER32_1-> LOAD = count ;
    TIMER32_1->CONTROL |= TIMER32_CONTROL_ONESHOT | ~ (TIMER32_CONTROL_IE)   ;
    TIMER32_1->CONTROL |= TIMER32_CONTROL_ENABLE ;
    while(TIMER32_1-> VALUE > 0);
    TIMER32_1->CONTROL |= ~(TIMER32_CONTROL_ENABLE) ;
}



/* to start ADC conversion process after reading X cordinate analog value */

void read_ch_xval()
{
    unsigned int i;
    read_xval();
    timer_delay(SETTLING);
    ADC14->MCTL[0] |= ADC14_MCTLN_INCH_0;
    ADC14->CTL1 |= (ADC14_CTL1_CSTARTADD_MASK & 0x0) ;
    ADC14->CTL0 |=  ADC14_CTL0_SC | ADC14_CTL0_ENC ;
    for(i =0 ; i< numb_samples ; i++)
    {
        ADC14->CTL0 |=  ADC14_CTL0_ENC ;
        x_values[i] = ADC14->MEM[0];
    }

    ADC14->CTL0 |=  ~(ADC14_CTL0_ENC) ;               // stop ADC conversion
}

/* to start ADC conversion process after reading Y coordinate analog value */
void read_ch_yval()
{
        unsigned int i;
        read_yval();
        timer_delay(SETTLING);
        ADC14->MCTL[3] |= ADC14_MCTLN_INCH_3  ;
        ADC14->CTL1 |= (ADC14_CTL1_CSTARTADD_MASK & 0x03);
        ADC14->CTL0 |=  ADC14_CTL0_SC | ADC14_CTL0_ENC ;
        for(i =0 ; i< numb_samples ; i++)
        {
            ADC14->CTL0 |=  ADC14_CTL0_ENC ;
            y_values[i] = ADC14->MEM[3];
        }

        ADC14->CTL0 |=  ~(ADC14_CTL0_ENC) ;            // stop ADC conversion
}

void led_red()
{

    P2->DIR |= BIT0;
    P2->OUT |= BIT0;
    timer_delay(5);
    P2->OUT &=  ~BIT0;

}
void led_green()
{
    P2->DIR |= BIT1;
    P2->OUT |= BIT1;
    timer_delay(5);
    P2->OUT &=  ~BIT1;

}

/* error condition used for powerstate condition check*/

void error(void)
{
    volatile uint32_t i;

    while (1)
    {
        P1->OUT ^= BIT0;
        for(i = 20000; i > 0; i--);           // Blink LED forever
    }
}


/* Pin config initialisation for driving LED matrix */

void matrix_ini()
{
                        P6->SEL1 &= ~BIT0  ;//R8 IS 6.0
                        P6->SEL0 &= ~BIT0 ;
                        P6->DIR |= BIT0;
                        P6->REN |= BIT0;
                        P6->DS |= BIT0;
                        P6->OUT |= BIT0;

                        P3->SEL1 &= ~BIT2  ; //R7 IS 3.2
                        P3->SEL0 &= ~BIT2 ;
                        P3->DIR |= BIT2;
                        P3->REN |= BIT2;
                        P3->DS |= BIT2;
                        P3->OUT |= BIT2;

                        P3->SEL1 &= ~BIT3  ; //3.3 IS C2
                        P3->SEL0 &= ~BIT3 ;
                        P3->DIR &=  ~BIT3;
                        P3->DS |= BIT3;
                        P3->OUT &= ~BIT3;

                        P4->SEL1 &= ~BIT1  ; //R1 IS 4.1
                        P4->SEL0 &= ~BIT1 ;
                        P4->DIR |= BIT1;
                        P4->REN |= BIT1;
                        P4->DS |= BIT1;
                        P4->OUT |= BIT1;

                        P4->SEL1 &= ~BIT3  ; //4.3 IS C4
                        P4->SEL0 &= ~BIT3 ;
                        P4->DIR &=  ~BIT3;
                        P4->DS |= BIT3;
                        P4->OUT &= ~BIT3;

                        P1->SEL1 &= ~BIT5  ; //R6 IS 1.5
                        P1->SEL0 &= ~BIT5 ;
                        P1->DIR |= BIT5;
                        P1->REN |= BIT5;
                        P1->DS |= BIT5;
                        P1->OUT |= BIT5;

                        P4->SEL1 &= ~BIT6  ; //R4 IS 4.6
                        P4->SEL0 &= ~BIT6 ;
                        P4->DIR |= BIT6;
                        P4->REN |= BIT6;
                        P4->DS |= BIT6;
                        P4->OUT |= BIT6;

                        P6->SEL1 &= ~BIT5  ; //C1 IS 6.5
                        P6->SEL0 &= ~BIT5 ;
                        P6->DIR &=  ~BIT5;
                        P6->DS |= BIT5;
                        P6->OUT &= ~BIT5;

                        P6->SEL1 &= ~BIT4  ; //C5 IS 6.4
                        P6->SEL0 &= ~BIT4 ;
                        P6->DIR &=  ~BIT4;
                        P6->DS |= BIT4;
                        P6->OUT &= ~BIT4;

                        P2->SEL1 &= ~BIT7  ; //C7 IS 2.7
                        P2->SEL0 &= ~BIT7 ;
                        P2->DIR &=  ~BIT7;
                        P2->DS |= BIT7;
                        P2->OUT &= ~BIT7;

                        P5->SEL1 &= ~BIT1  ; //R2 IS 5.1
                        P5->SEL0 &= ~BIT1 ;
                        P5->DIR |= BIT1;
                        P5->REN |= BIT1;
                        P5->DS |= BIT1;
                        P5->OUT |= BIT1;

                        P2->SEL1 &= ~BIT4  ; //R3 IS 2.4
                        P2->SEL0 &= ~BIT4 ;
                        P2->DIR |= BIT4;
                        P2->REN |= BIT4;
                        P2->DS |= BIT4;
                        P2->OUT |= BIT4;

                        P5->SEL1 &= ~BIT6  ; //C8 IS 5.6
                        P5->SEL0 &= ~BIT6 ;
                        P5->DIR &=  ~BIT6;
                        P5->DS |= BIT6;
                        P5->OUT &= ~BIT6;

                        P6->SEL1 &= ~BIT6  ; //R5 IS 6.6
                        P6->SEL0 &= ~BIT6 ;
                        P6->DIR |= BIT6;
                        P6->REN |= BIT6;
                        P6->DS |= BIT6;
                        P6->OUT |= BIT6;

                        P6->SEL1 &= ~BIT7  ; //C6 IS 6.7
                        P6->SEL0 &= ~BIT7 ;
                        P6->DIR &=  ~BIT7;
                        P6->DS |= BIT7;
                        P6->OUT &= ~BIT7;

                        P2->SEL1 &= ~BIT3  ; //C3 IS 2.3
                        P2->SEL0 &= ~BIT3 ;
                        P2->DIR &=  ~BIT3;
                        P2->DS |= BIT3;
                        P2->OUT &= ~BIT3;

}

/* function used for mapping which takes parameters as x (column) and y(row) coordinate */

void matrix_map(int a,int b)
{
   int column=0,row=0;
    column=a; row=b;


     if(column==1)                              //each column has switch cases for 8 different LED's
    {
        matrix_ini();
                        P6->SEL1 &= ~BIT5  ; //C1 IS HIGH
                        P6->SEL0 &= ~BIT5 ;
                        P6->DIR |= BIT5;
                        P6->REN |= BIT5;
                        P6->DS |= BIT5;
                        P6->OUT |= BIT5;

        if     (row==1){P4->SEL1 &= ~BIT1  ; //R1
                        P4->SEL0 &= ~BIT1 ;
                        P4->DIR &=  ~BIT1;
                        P4->DS |= BIT1;
                        P4->OUT &= ~BIT1;}

        else if(row==2){ P5->SEL1 &= ~BIT1  ; //R2
                        P5->SEL0 &= ~BIT1 ;
                        P5->DIR &=  ~BIT1;
                        P5->DS |= BIT1;
                        P5->OUT &= ~BIT1;}

        else if(row==3){ P2->SEL1 &= ~BIT4  ; //R3
                        P2->SEL0 &= ~BIT4 ;
                        P2->DIR &=  ~BIT4;
                        P2->DS |= BIT4;
                        P2->OUT &= ~BIT4;}

        else if(row==4){ P4->SEL1 &= ~BIT6  ; //R4
                        P4->SEL0 &= ~BIT6 ;
                        P4->DIR &=  ~BIT6;
                        P4->DS |= BIT6;
                        P4->OUT &= ~BIT6;}

        else if(row==5){ P6->SEL1 &= ~BIT6  ; //R5
                        P6->SEL0 &= ~BIT6 ;
                        P6->DIR &=  ~BIT6;
                        P6->DS |= BIT6;
                        P6->OUT &= ~BIT6;}

        else if(row==6){ P1->SEL1 &= ~BIT5  ; //R6
                        P1->SEL0 &= ~BIT5 ;
                        P1->DIR &=  ~BIT5;
                        P1->DS |= BIT5;
                        P1->OUT &= ~BIT5;}

        else if(row==7){ P3->SEL1 &= ~BIT2  ; //R7
                        P3->SEL0 &= ~BIT2 ;
                        P3->DIR &=  ~BIT2;
                        P3->DS |= BIT2;
                        P3->OUT &= ~BIT2;}

        else if(row==8){ P6->SEL1 &= ~BIT0  ; //R8
                        P6->SEL0 &= ~BIT0 ;
                        P6->DIR &=  ~BIT0;
                        P6->DS |= BIT0;
                        P6->OUT &= ~BIT0;}
      }
     else if(column==2)                          // column 2
    {
        matrix_ini();
                        P3->SEL1 &= ~BIT3  ; //C2 IS HIGH
                        P3->SEL0 &= ~BIT3 ;
                        P3->DIR |= BIT3;
                        P3->REN |= BIT3;
                        P3->DS |= BIT3;
                        P3->OUT |= BIT3;

        if     (row==1){P4->SEL1 &= ~BIT1  ; //R1
                        P4->SEL0 &= ~BIT1 ;
                        P4->DIR &=  ~BIT1;
                        P4->DS |= BIT1;
                        P4->OUT &= ~BIT1;}

        else if(row==2){ P5->SEL1 &= ~BIT1  ; //R2
                        P5->SEL0 &= ~BIT1 ;
                        P5->DIR &=  ~BIT1;
                        P5->DS |= BIT1;
                        P5->OUT &= ~BIT1;}

        else if(row==3){ P2->SEL1 &= ~BIT4  ; //R3
                        P2->SEL0 &= ~BIT4 ;
                        P2->DIR &=  ~BIT4;
                        P2->DS |= BIT4;
                        P2->OUT &= ~BIT4;}

        else if(row==4){ P4->SEL1 &= ~BIT6  ; //R4
                        P4->SEL0 &= ~BIT6 ;
                        P4->DIR &=  ~BIT6;
                        P4->DS |= BIT6;
                        P4->OUT &= ~BIT6;}

        else if(row==5){ P6->SEL1 &= ~BIT6  ; //R5
                        P6->SEL0 &= ~BIT6 ;
                        P6->DIR &=  ~BIT6;
                        P6->DS |= BIT6;
                        P6->OUT &= ~BIT6;}

        else if(row==6){ P1->SEL1 &= ~BIT5  ; //R6
                        P1->SEL0 &= ~BIT5 ;
                        P1->DIR &=  ~BIT5;
                        P1->DS |= BIT5;
                        P1->OUT &= ~BIT5;}

        else if(row==7){ P3->SEL1 &= ~BIT2  ; //R7
                        P3->SEL0 &= ~BIT2 ;
                        P3->DIR &=  ~BIT2;
                        P3->DS |= BIT2;
                        P3->OUT &= ~BIT2;}

        else if(row==8){ P6->SEL1 &= ~BIT0  ; //R8
                        P6->SEL0 &= ~BIT0 ;
                        P6->DIR &=  ~BIT0;
                        P6->DS |= BIT0;
                        P6->OUT &= ~BIT0;}
      }

      else if(column==3)              // column 3
    {
        matrix_ini();
                        P2->SEL1 &= ~BIT3  ; //C2 IS HIGH
                        P2->SEL0 &= ~BIT3 ;
                        P2->DIR |= BIT3;
                        P2->REN |= BIT3;
                        P2->DS |= BIT3;
                        P2->OUT |= BIT3;

        if     (row==1){P4->SEL1 &= ~BIT1  ; //R1
                        P4->SEL0 &= ~BIT1 ;
                        P4->DIR &=  ~BIT1;
                        P4->DS |= BIT1;
                        P4->OUT &= ~BIT1;}

        else if(row==2){ P5->SEL1 &= ~BIT1  ; //R2
                        P5->SEL0 &= ~BIT1 ;
                        P5->DIR &=  ~BIT1;
                        P5->DS |= BIT1;
                        P5->OUT &= ~BIT1;}

        else if(row==3){ P2->SEL1 &= ~BIT4  ; //R3
                        P2->SEL0 &= ~BIT4 ;
                        P2->DIR &=  ~BIT4;
                        P2->DS |= BIT4;
                        P2->OUT &= ~BIT4;}

        else if(row==4){ P4->SEL1 &= ~BIT6  ; //R4
                        P4->SEL0 &= ~BIT6 ;
                        P4->DIR &=  ~BIT6;
                        P4->DS |= BIT6;
                        P4->OUT &= ~BIT6;}

        else if(row==5){ P6->SEL1 &= ~BIT6  ; //R5
                        P6->SEL0 &= ~BIT6 ;
                        P6->DIR &=  ~BIT6;
                        P6->DS |= BIT6;
                        P6->OUT &= ~BIT6;}

        else if(row==6){ P1->SEL1 &= ~BIT5  ; //R6
                        P1->SEL0 &= ~BIT5 ;
                        P1->DIR &=  ~BIT5;
                        P1->DS |= BIT5;
                        P1->OUT &= ~BIT5;}

        else if(row==7){ P3->SEL1 &= ~BIT2  ; //R7
                        P3->SEL0 &= ~BIT2 ;
                        P3->DIR &=  ~BIT2;
                        P3->DS |= BIT2;
                        P3->OUT &= ~BIT2;}

        else if(row==8){ P6->SEL1 &= ~BIT0  ; //R8
                        P6->SEL0 &= ~BIT0 ;
                        P6->DIR &=  ~BIT0;
                        P6->DS |= BIT0;
                        P6->OUT &= ~BIT0;}
      }
      else if(column==4)                // column 4
    {
        matrix_ini();
                        P4->SEL1 &= ~BIT3  ; //C2 IS HIGH
                        P4->SEL0 &= ~BIT3 ;
                        P4->DIR |= BIT3;
                        P4->REN |= BIT3;
                        P4->DS |= BIT3;
                        P4->OUT |= BIT3;

        if     (row==1){P4->SEL1 &= ~BIT1  ; //R1
                        P4->SEL0 &= ~BIT1 ;
                        P4->DIR &=  ~BIT1;
                        P4->DS |= BIT1;
                        P4->OUT &= ~BIT1;}

        else if(row==2){ P5->SEL1 &= ~BIT1  ; //R2
                        P5->SEL0 &= ~BIT1 ;
                        P5->DIR &=  ~BIT1;
                        P5->DS |= BIT1;
                        P5->OUT &= ~BIT1;}

        else if(row==3){ P2->SEL1 &= ~BIT4  ; //R3
                        P2->SEL0 &= ~BIT4 ;
                        P2->DIR &=  ~BIT4;
                        P2->DS |= BIT4;
                        P2->OUT &= ~BIT4;}

        else if(row==4){ P4->SEL1 &= ~BIT6  ; //R4
                        P4->SEL0 &= ~BIT6 ;
                        P4->DIR &=  ~BIT6;
                        P4->DS |= BIT6;
                        P4->OUT &= ~BIT6;}

        else if(row==5){ P6->SEL1 &= ~BIT6  ; //R5
                        P6->SEL0 &= ~BIT6 ;
                        P6->DIR &=  ~BIT6;
                        P6->DS |= BIT6;
                        P6->OUT &= ~BIT6;}

        else if(row==6){ P1->SEL1 &= ~BIT5  ; //R6
                        P1->SEL0 &= ~BIT5 ;
                        P1->DIR &=  ~BIT5;
                        P1->DS |= BIT5;
                        P1->OUT &= ~BIT5;}

        else if(row==7){ P3->SEL1 &= ~BIT2  ; //R7
                        P3->SEL0 &= ~BIT2 ;
                        P3->DIR &=  ~BIT2;
                        P3->DS |= BIT2;
                        P3->OUT &= ~BIT2;}

        else if(row==8){ P6->SEL1 &= ~BIT0  ; //R8
                        P6->SEL0 &= ~BIT0 ;
                        P6->DIR &=  ~BIT0;
                        P6->DS |= BIT0;
                        P6->OUT &= ~BIT0;}
      }
      else if(column==5)            // column 5
    {
        matrix_ini();
                        P6->SEL1 &= ~BIT4  ; //C2 IS HIGH
                        P6->SEL0 &= ~BIT4 ;
                        P6->DIR |= BIT4;
                        P6->REN |= BIT4;
                        P6->DS |= BIT4;
                        P6->OUT |= BIT4;

        if     (row==1){P4->SEL1 &= ~BIT1  ; //R1
                        P4->SEL0 &= ~BIT1 ;
                        P4->DIR &=  ~BIT1;
                        P4->DS |= BIT1;
                        P4->OUT &= ~BIT1;}

        else if(row==2){ P5->SEL1 &= ~BIT1  ; //R2
                        P5->SEL0 &= ~BIT1 ;
                        P5->DIR &=  ~BIT1;
                        P5->DS |= BIT1;
                        P5->OUT &= ~BIT1;}

        else if(row==3){ P2->SEL1 &= ~BIT4  ; //R3
                        P2->SEL0 &= ~BIT4 ;
                        P2->DIR &=  ~BIT4;
                        P2->DS |= BIT4;
                        P2->OUT &= ~BIT4;}

        else if(row==4){ P4->SEL1 &= ~BIT6  ; //R4
                        P4->SEL0 &= ~BIT6 ;
                        P4->DIR &=  ~BIT6;
                        P4->DS |= BIT6;
                        P4->OUT &= ~BIT6;}

        else if(row==5){ P6->SEL1 &= ~BIT6  ; //R5
                        P6->SEL0 &= ~BIT6 ;
                        P6->DIR &=  ~BIT6;
                        P6->DS |= BIT6;
                        P6->OUT &= ~BIT6;}

        else if(row==6){ P1->SEL1 &= ~BIT5  ; //R6
                        P1->SEL0 &= ~BIT5 ;
                        P1->DIR &=  ~BIT5;
                        P1->DS |= BIT5;
                        P1->OUT &= ~BIT5;}

        else if(row==7){ P3->SEL1 &= ~BIT2  ; //R7
                        P3->SEL0 &= ~BIT2 ;
                        P3->DIR &=  ~BIT2;
                        P3->DS |= BIT2;
                        P3->OUT &= ~BIT2;}

        else if(row==8){ P6->SEL1 &= ~BIT0  ; //R8
                        P6->SEL0 &= ~BIT0 ;
                        P6->DIR &=  ~BIT0;
                        P6->DS |= BIT0;
                        P6->OUT &= ~BIT0;}
      }
      else if(column==6)                // column 6
    {
        matrix_ini();
                        P6->SEL1 &= ~BIT7  ; //C2 IS HIGH
                        P6->SEL0 &= ~BIT7 ;
                        P6->DIR |= BIT7;
                        P6->REN |= BIT7;
                        P6->DS |= BIT7;
                        P6->OUT |= BIT7;

        if     (row==1){P4->SEL1 &= ~BIT1  ; //R1
                        P4->SEL0 &= ~BIT1 ;
                        P4->DIR &=  ~BIT1;
                        P4->DS |= BIT1;
                        P4->OUT &= ~BIT1;}

        else if(row==2){ P5->SEL1 &= ~BIT1  ; //R2
                        P5->SEL0 &= ~BIT1 ;
                        P5->DIR &=  ~BIT1;
                        P5->DS |= BIT1;
                        P5->OUT &= ~BIT1;}

        else if(row==3){ P2->SEL1 &= ~BIT4  ; //R3
                        P2->SEL0 &= ~BIT4 ;
                        P2->DIR &=  ~BIT4;
                        P2->DS |= BIT4;
                        P2->OUT &= ~BIT4;}

        else if(row==4){ P4->SEL1 &= ~BIT6  ; //R4
                        P4->SEL0 &= ~BIT6 ;
                        P4->DIR &=  ~BIT6;
                        P4->DS |= BIT6;
                        P4->OUT &= ~BIT6;}

        else if(row==5){ P6->SEL1 &= ~BIT6  ; //R5
                        P6->SEL0 &= ~BIT6 ;
                        P6->DIR &=  ~BIT6;
                        P6->DS |= BIT6;
                        P6->OUT &= ~BIT6;}

        else if(row==6){ P1->SEL1 &= ~BIT5  ; //R6
                        P1->SEL0 &= ~BIT5 ;
                        P1->DIR &=  ~BIT5;
                        P1->DS |= BIT5;
                        P1->OUT &= ~BIT5;}

        else if(row==7){ P3->SEL1 &= ~BIT2  ; //R7
                        P3->SEL0 &= ~BIT2 ;
                        P3->DIR &=  ~BIT2;
                        P3->DS |= BIT2;
                        P3->OUT &= ~BIT2;}

        else if(row==8){ P6->SEL1 &= ~BIT0  ; //R8
                        P6->SEL0 &= ~BIT0 ;
                        P6->DIR &=  ~BIT0;
                        P6->DS |= BIT0;
                        P6->OUT &= ~BIT0;}
      }
      else if(column==7)                // column 7
    {
        matrix_ini();
                        P2->SEL1 &= ~BIT7  ; //C2 IS HIGH
                        P2->SEL0 &= ~BIT7 ;
                        P2->DIR |= BIT7;
                        P2->REN |= BIT7;
                        P2->DS |= BIT7;
                        P2->OUT |= BIT7;

        if     (row==1){P4->SEL1 &= ~BIT1  ; //R1
                        P4->SEL0 &= ~BIT1 ;
                        P4->DIR &=  ~BIT1;
                        P4->DS |= BIT1;
                        P4->OUT &= ~BIT1;}

        else if(row==2){ P5->SEL1 &= ~BIT1  ; //R2
                        P5->SEL0 &= ~BIT1 ;
                        P5->DIR &=  ~BIT1;
                        P5->DS |= BIT1;
                        P5->OUT &= ~BIT1;}

        else if(row==3){ P2->SEL1 &= ~BIT4  ; //R3
                        P2->SEL0 &= ~BIT4 ;
                        P2->DIR &=  ~BIT4;
                        P2->DS |= BIT4;
                        P2->OUT &= ~BIT4;}

        else if(row==4){ P4->SEL1 &= ~BIT6  ; //R4
                        P4->SEL0 &= ~BIT6 ;
                        P4->DIR &=  ~BIT6;
                        P4->DS |= BIT6;
                        P4->OUT &= ~BIT6;}

        else if(row==5){ P6->SEL1 &= ~BIT6  ; //R5
                        P6->SEL0 &= ~BIT6 ;
                        P6->DIR &=  ~BIT6;
                        P6->DS |= BIT6;
                        P6->OUT &= ~BIT6;}

        else if(row==6){ P1->SEL1 &= ~BIT5  ; //R6
                        P1->SEL0 &= ~BIT5 ;
                        P1->DIR &=  ~BIT5;
                        P1->DS |= BIT5;
                        P1->OUT &= ~BIT5;}

        else if(row==7){ P3->SEL1 &= ~BIT2  ; //R7
                        P3->SEL0 &= ~BIT2 ;
                        P3->DIR &=  ~BIT2;
                        P3->DS |= BIT2;
                        P3->OUT &= ~BIT2;}

        else if(row==8){ P6->SEL1 &= ~BIT0  ; //R8
                        P6->SEL0 &= ~BIT0 ;
                        P6->DIR &=  ~BIT0;
                        P6->DS |= BIT0;
                        P6->OUT &= ~BIT0;}
      }
else if(column==8)              // column 8
    {
        matrix_ini();
                        P5->SEL1 &= ~BIT6  ; //C2 IS HIGH
                        P5->SEL0 &= ~BIT6 ;
                        P5->DIR |= BIT6;
                        P5->REN |= BIT6;
                        P5->DS |= BIT6;
                        P5->OUT |= BIT6;

        if     (row==1){P4->SEL1 &= ~BIT1  ; //R1
                        P4->SEL0 &= ~BIT1 ;
                        P4->DIR &=  ~BIT1;
                        P4->DS |= BIT1;
                        P4->OUT &= ~BIT1;}

        else if(row==2){ P5->SEL1 &= ~BIT1  ; //R2
                        P5->SEL0 &= ~BIT1 ;
                        P5->DIR &=  ~BIT1;
                        P5->DS |= BIT1;
                        P5->OUT &= ~BIT1;}

        else if(row==3){ P2->SEL1 &= ~BIT4  ; //R3
                        P2->SEL0 &= ~BIT4 ;
                        P2->DIR &=  ~BIT4;
                        P2->DS |= BIT4;
                        P2->OUT &= ~BIT4;}

        else if(row==4){ P4->SEL1 &= ~BIT6  ; //R4
                        P4->SEL0 &= ~BIT6 ;
                        P4->DIR &=  ~BIT6;
                        P4->DS |= BIT6;
                        P4->OUT &= ~BIT6;}

        else if(row==5){ P6->SEL1 &= ~BIT6  ; //R5
                        P6->SEL0 &= ~BIT6 ;
                        P6->DIR &=  ~BIT6;
                        P6->DS |= BIT6;
                        P6->OUT &= ~BIT6;}

        else if(row==6){ P1->SEL1 &= ~BIT5  ; //R6
                        P1->SEL0 &= ~BIT5 ;
                        P1->DIR &=  ~BIT5;
                        P1->DS |= BIT5;
                        P1->OUT &= ~BIT5;}

        else if(row==7){ P3->SEL1 &= ~BIT2  ; //R7
                        P3->SEL0 &= ~BIT2 ;
                        P3->DIR &=  ~BIT2;
                        P3->DS |= BIT2;
                        P3->OUT &= ~BIT2;}

        else if(row==8){ P6->SEL1 &= ~BIT0  ; //R8
                        P6->SEL0 &= ~BIT0 ;
                        P6->DIR &=  ~BIT0;
                        P6->DS |= BIT0;
                        P6->OUT &= ~BIT0;}
      }
}

/* function to test if all led's ar working */
/*
void fun()
{
          matrix9(1,1);
          timer_delay(40000);
          matrix_ini();
          matrix9(1,2);
          timer_delay(40000);
          matrix_ini();
          matrix9(1,3);
          timer_delay(40000);
          matrix_ini();
          matrix9(1,4);
          timer_delay(40000);
          matrix_ini();
          matrix9(1,5);
          timer_delay(40000);
          matrix_ini();
          matrix9(1,6);
          timer_delay(40000);
          matrix_ini();
          matrix9(1,7);
          timer_delay(40000);
          matrix_ini();
          matrix9(1,8);
          timer_delay(40000);
          matrix_ini();
          matrix9(2,8);
          timer_delay(40000);
          matrix_ini();
          matrix9(2,7);
          timer_delay(40000);
          matrix_ini();
          matrix9(2,6);
          timer_delay(40000);
          matrix_ini();
          matrix9(2,5);
          timer_delay(40000);
          matrix_ini();
          matrix9(2,4);
          timer_delay(40000);
          matrix_ini();
          matrix9(2,3);
          timer_delay(40000);
          matrix_ini();
          matrix9(2,2);
          timer_delay(40000);
          matrix_ini();
          matrix9(2,1);
          timer_delay(40000);
          matrix_ini();
          matrix9(3,1);
          timer_delay(40000);
          matrix_ini();
          matrix9(3,2);
          timer_delay(40000);
          matrix_ini();
          matrix9(3,3);
          timer_delay(40000);
          matrix_ini();
          matrix9(3,4);
          timer_delay(40000);
          matrix_ini();
          matrix9(3,5);
          timer_delay(40000);
          matrix_ini();
          matrix9(3,6);
          timer_delay(40000);
          matrix_ini();
          matrix9(3,7);
          timer_delay(40000);
          matrix_ini();
          matrix9(3,8);
          timer_delay(40000);
          matrix_ini();
          matrix9(4,8);
          timer_delay(40000);
          matrix_ini();
          matrix9(4,7);
          timer_delay(40000);
          matrix_ini();
          matrix9(4,6);
          timer_delay(40000);
          matrix_ini();
          matrix9(4,5);
          timer_delay(40000);
          matrix_ini();
          matrix9(4,4);
          timer_delay(40000);
          matrix_ini();
          matrix9(4,3);
          timer_delay(40000);
          matrix_ini();
          matrix9(4,2);
          timer_delay(40000);
          matrix_ini();
          matrix9(4,1);
          timer_delay(40000);
          matrix_ini();
          matrix9(5,1);
          timer_delay(40000);
          matrix_ini();
          matrix9(5,2);
          timer_delay(40000);
          matrix_ini();
          matrix9(5,3);
          timer_delay(40000);
          matrix_ini();
          matrix9(5,4);
          timer_delay(40000);
          matrix_ini();
          matrix9(5,5);
          timer_delay(40000);
          matrix_ini();
          matrix9(5,6);
          timer_delay(40000);
          matrix_ini();
          matrix9(5,7);
          timer_delay(40000);
          matrix_ini();
          matrix9(5,8);
          timer_delay(40000);
          matrix_ini();
          matrix9(6,8);
          timer_delay(40000);
          matrix_ini();
          matrix9(6,7);
          timer_delay(40000);
          matrix_ini();
          matrix9(6,6);
          timer_delay(40000);
          matrix_ini();
          matrix9(6,5);
          timer_delay(40000);
          matrix_ini();
          matrix9(6,4);
          timer_delay(40000);
          matrix_ini();
          matrix9(6,3);
          timer_delay(40000);
          matrix_ini();
          matrix9(6,2);
          timer_delay(40000);
          matrix_ini();
          matrix9(6,1);
          timer_delay(40000);
          matrix_ini();
          matrix9(7,1);
          timer_delay(40000);
          matrix_ini();
          matrix9(7,2);
          timer_delay(40000);
          matrix_ini();
          matrix9(7,3);
          timer_delay(40000);
          matrix_ini();
          matrix9(7,4);
          timer_delay(40000);
          matrix_ini();
          matrix9(7,5);
          timer_delay(40000);
          matrix_ini();
          matrix9(7,6);
          timer_delay(40000);
          matrix_ini();
          matrix9(7,7);
          timer_delay(40000);
          matrix_ini();
          matrix9(7,7);
          timer_delay(40000);
          matrix_ini();
          matrix9(8,8);
          timer_delay(40000);
          matrix_ini();
          matrix9(8,8);
          timer_delay(40000);
          matrix_ini();
          matrix9(8,7);
          timer_delay(40000);
          matrix_ini();
          matrix9(8,6);
          timer_delay(40000);
          matrix_ini();
          matrix9(8,5);
          timer_delay(40000);
          matrix_ini();
          matrix9(8,4);
          timer_delay(40000);
          matrix_ini();
          matrix9(8,3);
          timer_delay(40000);
          matrix_ini();
          matrix9(8,2);
          timer_delay(40000);
          matrix_ini();
          matrix9(8,1);
          timer_delay(40000);
          matrix_ini();

}

*/


