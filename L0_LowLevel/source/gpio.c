/*
 * gpio.c
 *
 *  Created on: Feb 13, 2017
 *      Author: Revthy
 */
#include "gpio.h"
#include "utilities.h"

/*********Function Definition***********/
/*
 * Function Interfaces to set pin configurations
 * these functions can be called to configure and use any gpio port
 * @para : portnum = 0 to 3
 *         pin = refer LPC74XX manual for available gpio pins
 *         func,mode,imode,omode = refer LPC74XX manual for config register setting
 */

void configurePortPin(uint8_t portnum,
		              uint8_t pin,
					  pin_func_t func,
					  pin_mode_t mode,
					  pin_input_mode_t imode,
					  pin_output_mode_t omode)
{
   switch(portnum)
   {
      case 0:
      {
         if(pin < REG_L)
         {
        	 /* Set pin function */
            LPC_PINCON->PINSEL0 &= CLR_2BITS(pin);
             /* Set pin Input Mode & Direction Bit for Input*/
            if(mode == input)
            {
            	LPC_GPIO0->FIODIR &= CLR(pin);
            	LPC_PINCON->PINMODE0 &= CLR_2BITS(pin);
				LPC_PINCON->PINMODE0 |= SET_2BITS(imode,pin);
            }
         }
         else if((pin > REG_L) && (pin < REG_H))
         {
        	 LPC_PINCON->PINSEL1 &= CLR_2BITS(pin);
        	 if(mode == input)
        	 {
        		 LPC_GPIO0->FIODIR &= CLR(pin);
        		 LPC_PINCON->PINMODE1 &= CLR_2BITS(pin);
        		 LPC_PINCON->PINMODE1 |= SET_2BITS(imode,pin);
        	 }
         }

         /* Set pin Output Mode & Direction Bit for Output*/

		if (mode == output  && omode == opendrain )
		{
			LPC_GPIO0->FIODIR |= SET(pin);
			LPC_PINCON->PINMODE_OD0 |= SET(pin);
		}
		else if (mode == output  && omode == pushpull )
		{
			LPC_GPIO0->FIODIR |= SET(pin);
			LPC_PINCON->PINMODE_OD0 &= CLR(pin);
		}

		break;
      }
      case 1:
      {
    	  if(pin < REG_L)
    	  {
    		  LPC_PINCON->PINSEL2 &= CLR_2BITS(pin);
    		  if(mode == input)
    		  {
    			  LPC_GPIO1->FIODIR &= CLR(pin);
    			  LPC_PINCON->PINMODE2 &= CLR_2BITS(pin);
    			  LPC_PINCON->PINMODE2 |= SET_2BITS(imode,pin);
    		  }
    	  }
    	  else if((pin > REG_L) && (pin < REG_H))
    	  {
    		  LPC_PINCON->PINSEL3 &= CLR_2BITS(pin);
    		  if(mode == input)
    		  {
    			  LPC_GPIO1->FIODIR &= CLR(pin);
    			  LPC_PINCON->PINMODE3 &= CLR_2BITS(pin);
    			  LPC_PINCON->PINMODE3 |= SET_2BITS(imode,pin);
    		  }
    	  }
    	  if (mode == output  && omode == opendrain )
    	  {
    		  LPC_GPIO1->FIODIR |= SET(pin);
    		  LPC_PINCON->PINMODE_OD1 |= SET(pin);
    	  }
    	  else if (mode == output  && omode == pushpull )
    	  {
    		  LPC_GPIO1->FIODIR |= SET(pin);
    		  LPC_PINCON->PINMODE_OD1 &= CLR(pin);
    	  }
    	  break;
      }
      case 2:
      {
    	  if(pin < REG_L)
    	  {
    		  LPC_PINCON->PINSEL4 &= CLR_2BITS(pin);
    		  if(mode == input)
    		  {
    			  LPC_GPIO2->FIODIR &= CLR(pin);
    			  LPC_PINCON->PINMODE4 &= CLR_2BITS(pin);
    			  LPC_PINCON->PINMODE4 |= SET_2BITS(imode,pin);
    		  }
    	  }
    	  else if((pin > REG_L) && (pin < REG_H))
    	  {
    		  LPC_PINCON->PINSEL5 &= CLR_2BITS(pin);
    		  if(mode == input)
    		  {
    			  LPC_GPIO2->FIODIR &= CLR(pin);
    			  LPC_PINCON->PINMODE5 &= CLR_2BITS(pin);
    			  LPC_PINCON->PINMODE5 |= SET_2BITS(imode,pin);
    		  }
    	  }

    	  if (mode == output  && omode == opendrain )
    	  {
    		  LPC_GPIO2->FIODIR |= SET(pin);
    		  LPC_PINCON->PINMODE_OD2 |= SET(pin);
    	  }
    	  else if (mode == output  && omode == pushpull )
    	  {
    		  LPC_GPIO2->FIODIR |= SET(pin);
    		  LPC_PINCON->PINMODE_OD2 &= CLR(pin);
    	  }
    	  break;
      }
      case 3:
      {
    	  if(pin < REG_L)
    	  {
    		  LPC_PINCON->PINSEL6 &= CLR_2BITS(pin);
    		  if(mode == input)
    		  {
    			  LPC_GPIO3->FIODIR &= CLR(pin);
    			  LPC_PINCON->PINMODE6 &= CLR_2BITS(pin);
    			  LPC_PINCON->PINMODE6 |= SET_2BITS(imode,pin);
    		  }
    	  }
    	  else if((pin > REG_L) && (pin < REG_H))
    	  {
    		  LPC_PINCON->PINSEL7 &= CLR_2BITS(pin);
    		  if(mode == input)
    		  {
    			  LPC_GPIO3->FIODIR &= CLR(pin);
    			  LPC_PINCON->PINMODE7 &= CLR_2BITS(pin);
    			  LPC_PINCON->PINMODE7 |= SET_2BITS(imode,pin);
    		  }
    	  }

    	  if (mode == output  && omode == opendrain )
    	  {
    		  LPC_GPIO3->FIODIR |= SET(pin);
    		  LPC_PINCON->PINMODE_OD3 |= SET(pin);
    	  }
    	  else if (mode == output  && omode == pushpull )
    	  {
    		  LPC_GPIO3->FIODIR |= SET(pin);
    		  LPC_PINCON->PINMODE_OD3 &= CLR(pin);
    	  }
    	  break;
      }
   }
}
