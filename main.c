/*------------------------------------------------
 *   Authors = Dixit Gurung
 *   EGR 326 901
 *   Date = 10/01/2020
 *   Lab_4
 *
 *   Description:The goal of the program was to detect the nob direction of the rotary encoder and
 *   incorporate the use of keypad with the rotary encoder.
 *   The keypad reads two-digit number from the user and illuminate the BLUE LED when
 *   the CCW count is met and illuminates the RED LED when the CW count is met.
 *
 *---------------------------------------------------*/

//----------------------------------------------------- PART I II & III ------------------------------------------------------------
#include "msp.h"
#include <stdio.h>
#include <stdlib.h>
#include "keypad.h"

void Rotary_Encoder_init();
void LED_init();

int keypress;  //flag for keypad input
int input = 0; //2digit input from the keypad
int CW = 0; //CW count
int CCW = 0; //CCW count
int final_count = 0; //store final count
int Tens_place = 0; // Store tens place value from keypad
int Ones_place = 0; // Store ones place value from keypad

void main(void)
{

    __disable_irq();
    Rotary_Encoder_init();
    LED_init();
    NVIC_EnableIRQ(PORT2_IRQn);
    __enable_irq();

    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;           // stop watchdog timer

    while (1)
    {
        keypress = getKeypress();

        if (!(P2->IN & BIT5 ))
        {
            printf("Counter Clock wise count = %d\n", CCW);
            printf("Clock wise count = %d\n", CW);

            if (final_count < 0)
            {
                final_count = -final_count;
                printf("final_count CCW = %d", final_count);
            }

            else
            {
                printf("final_count CW = %d", final_count);
            }
        }

        if (keypress != -1)
        {
            Tens_place = keypress;
            keypress = getKeypress();

            while (keypress == -1)
            {
                keypress = getKeypress();
            }

            Ones_place = keypress;
            input = (Tens_place * 10) + Ones_place;
            printf("\nInput = %d\n", input);
        }

        if (CCW == input)
        {
            P5->OUT |= BIT6;
        }

        else
        {
            P5->OUT &= ~ BIT6;
        }

        if (CW == input)
        {
            P5->OUT |= BIT7;
        }

        else
            P5->OUT &= ~ BIT7;
    }
}

//Initializing pins for Rotary encoder
void Rotary_Encoder_init()
{
    //Switch init as input
    P2->SEL0 &= ~ BIT5;
    P2->SEL1 &= ~ BIT5;
    P2->DIR &= ~ BIT5;
    P2->REN |= BIT5;
    P2->OUT |= BIT5;
    P2->IES |= BIT5;//falling edge interrupt
    P2->IE |= BIT5;//enable interrupt
    P2->IFG = 0;//clear IFG flag

//Input setup for DT pin
    P2->SEL0 &= ~ BIT4;
    P2->SEL1 &= ~ BIT4;
    P2->DIR &= ~ BIT4;
    P2->REN |= BIT4;
    P2->OUT |= BIT4;
    P2->IES &= ~ BIT4;        //Rising edge interrupt
    P2->IE |= BIT4;           //enable interrupt
    P2->IFG = 0;              //clear IFG flag

    //Interrupt setup for SW pin
    P2->SEL0 &= ~ BIT6;
    P2->SEL1 &= ~ BIT6;
    P2->DIR &= ~ BIT6;
    P2->REN |= BIT6;
    P2->OUT |= BIT6;
}

//Initializing two leds for partIII
void LED_init()
{
    P5->SEL0 &= ~(BIT7 | BIT6 );
    P5->SEL1 &= ~(BIT7 | BIT6 );
    P5->DIR |= (BIT7 | BIT6 );
    P5->OUT &= ~(BIT7 | BIT6 );

}

void PORT2_IRQHandler()
{
    if (P2->IFG & BIT4)
    {
        if (P2->IN & BIT6)
        {
            if (P2->IN & BIT6)
            {
                CW++;
            }
        }

        if (!(P2->IN & BIT6 ))
        {
            CCW++;
        }

        P2->IFG = 0x00;

    }

    //If the button is pressed clear the counter for both CW and CCW
    if (P2->IFG & BIT5)
    {
        final_count = CW - CCW;
        CCW = 0;
        CW = 0;
    }
    P2->IFG = 0x00;                     //Clear interrupt flags
}
