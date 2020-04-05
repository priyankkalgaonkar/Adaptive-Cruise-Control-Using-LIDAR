/*******************************************************************************
 * Includes
 *******************************************************************************/
#include "Cpu.h"
#include "pin_mux.h"
#include "osif1.h"
#include "dmaController1.h"
#include "clockMan1.h"
#include "lpuart1.h"
#include "S32K144.h" 
#include "MPL3115A2.h"
#include "I2C.h"
/*******************************************************************************
 * Local function prototypes
 *******************************************************************************/
void SIRC_div(void);
void PORT_conf(void);
/*******************************************************************************
 * Global variables
 *******************************************************************************/
uint8_t  ready;
uint8_t  err_0, err_1, err_2, err_3, err_4, err_5, err_6;
uint8_t  status[1],altitude[2];

uint16_t distance_lidar;

volatile int exit_code = 0;
/* User includes (#include below this line is not maintained by Processor Expert) */
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

int main(void)
{ 
  SIRC_div();
  PORT_conf();
  LPI2C0_clock();
  LPI2C0_IRQs();
  LPI2C0_init();

  bool strReceived = false;
  uint8_t	 buffer [255]	={0,};
  uint8_t  i = 0;
  uint32_t bytesRemaining;
#ifdef PEX_RTOS_INIT
  PEX_RTOS_INIT(); 
#endif

  CLOCK_SYS_Init(g_clockManConfigsArr, FSL_CLOCK_MANAGER_CONFIG_CNT,
                 g_clockManCallbacksArr, FSL_CLOCK_MANAGER_CALLBACK_CNT);
  CLOCK_SYS_UpdateConfiguration(0U, CLOCK_MANAGER_POLICY_FORCIBLE);

  PINS_DRV_Init(NUM_OF_CONFIGURED_PINS, g_pin_mux_InitConfigArr);
  PORT_conf();
  /* Initialize LPUART instance */
  LPUART_DRV_Init(FSL_LPUART1, &lpuart1_State, &lpuart1_InitConfig0);

  while(LPUART_DRV_GetTransmitStatus(FSL_LPUART1, &bytesRemaining) != LPUART_STAT_SUCCESS);

  /* Infinite loop:
   * 	- Receive data from user
   * 	- Echo the received data back
   */

  SIRC_div();
  while (1)
    {
	    err_0 = LPI2C0_read(MPL3115A2_R, DR_STATUS_REG, status, 1);
        distance_lidar =0;
        if(!(status[0] & 0x01))
        {	err_2 = LPI2C0_write(MPL3115A2_W, 0x00, 0x04);
            if(!(err_5 = LPI2C0_read(MPL3115A2_R, OUT_P_MSB_REG, altitude, 2)))
            {
                
                distance_lidar     |= (altitude[0] << 8);
                distance_lidar     |= (altitude[1]);
                sprintf(buffer, "%u\r\n", distance_lidar);
                i = strlen((char *)buffer);
                LPUART_DRV_SendData(FSL_LPUART1, buffer, i);
            }
        }
     buffer[i] = 0;
     while(LPUART_DRV_GetTransmitStatus(FSL_LPUART1, &bytesRemaining) != LPUART_STAT_SUCCESS);
     strReceived = false;
     i = 0;
    }
  #ifdef PEX_RTOS_START
    PEX_RTOS_START();                  /* Startup of the selected RTOS. Macro is defined by the RTOS component. */
  #endif
  for(;;) {
    if(exit_code != 0) {
      break;
    }
  }
  return exit_code;
  
} 


/*******************************************************************************
Function Name : SIRC_div_8MHz
Notes         : SIRCDIV2 divide by 1 (8MHz)
                SIRCDIV1 divide by 1 (8MHz)
 *******************************************************************************/
void SIRC_div(void)
{

    SCG->SIRCCSR &= ~ (1 << 24);
    SCG->SIRCCSR &= ~ (1 << 0);
    SCG->SIRCDIV |= 0x0101;
    SCG->SIRCCSR |= (1 << 0);
    while((SCG->SIRCCSR & (1 << 24)) == 0); 
}

/*******************************************************************************
Function Name : init_port
 *******************************************************************************/
void PORT_conf(void)
{
    // Peripheral Clock Controller
    PCC-> PCCn[PCC_PORTA_INDEX] = PCC_PCCn_CGC_MASK;

    PORTA->PCR[2] |= PORT_PCR_MUX(3);
    PORTA->PCR[2] |= PORT_PCR_PS(1);
    PORTA->PCR[2] |= PORT_PCR_PE(1);
    PORTA->PCR[3] |= PORT_PCR_MUX(3);
    PORTA->PCR[3] |= PORT_PCR_PS(1);
    PORTA->PCR[3] |= PORT_PCR_PE(1);

    PCC-> PCCn[PCC_PORTD_INDEX] = PCC_PCCn_CGC_MASK; 

    PORTD->PCR[0] = 0x00000100; 
    PTD-> PSOR |= (1 << 0);             
    PTD->PDDR |= (1 << 0);              
}