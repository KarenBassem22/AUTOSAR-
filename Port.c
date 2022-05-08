/******************************************************************************
 * Module: Port
 *
 * File Name: Port.c
 *
 * Description: Source file for TM4C123GH6PM Microcontroller - Port Driver.
 *
 * Author: Karen Bassem
 ******************************************************************************/

#include "Port.h"
#include "Port_Regs.h"
#include "tm4c123gh6pm_registers.h"

/************************************************************************************
 * Service Name: Port_GetVersionInfo
 * Service ID[hex]: 0x03
 * Sync/Async: Synchronous
 * Reentrancy: Non Reentrant
 * Parameters (in): None
 * Parameters (inout): None
 * Parameters (out): VersionInfo - Pointer to where to store the version information of this module.
 * Return value: None
 * Description: Function to get the version information of this module.
 ************************************************************************************/
#if (PORT_VERSION_INFO_API == STD_ON)
void Port_GetVersionInfo(Std_VersionInfoType *versioninfo)
{
#if (PORT_DEV_ERROR_DETECT == STD_ON)
	/* Check if input pointer is not Null pointer */
	if(NULL_PTR == versioninfo)
	{
		/* Report to DET  */
		Det_ReportError(PORT_MODULE_ID, 
				PORT_INSTANCE_ID,
				PORT_GET_VERSION_INFO_SID, 
				PORT_E_PARAM_POINTER);
	}
	else
	{

	}

	/* Check if the Driver is initialized before using this function */
	if (PORT_NOT_INITIALIZED == Port_Status)
	{
		Det_ReportError(PORT_MODULE_ID, 
				PORT_INSTANCE_ID,
				PORT_GET_VERSION_INFO_SID, 
				PORT_E_UNINIT);
		//error = TRUE;
	}
	else
	{
		/* No Action Required */
	}

#endif /* (PORT_DEV_ERROR_DETECT == STD_ON) */
	{
		/* Copy the vendor Id */
		versioninfo->vendorID = (uint16)PORT_VENDOR_ID;
		/* Copy the module Id */
		versioninfo->moduleID = (uint16)PORT_MODULE_ID;
		/* Copy Software Major Version */
		versioninfo->sw_major_version = (uint8)PORT_SW_MAJOR_VERSION;
		/* Copy Software Minor Version */
		versioninfo->sw_minor_version = (uint8)PORT_SW_MINOR_VERSION;
		/* Copy Software Patch Version */
		versioninfo->sw_patch_version = (uint8)PORT_SW_PATCH_VERSION;
	}
}
#endif


/* Set the module state to initialized and point to the PB configuration structure using a global pointer.
 * This global pointer is global to be used by other functions to read the PB configuration structures */
STATIC const Port_ConfigType * Port_ConfigPtr = NULL_PTR;  
uint8 Port_Status = PORT_NOT_INITIALIZED;

/************************************************************************************
 * Service Name: Port_Init
 * Sync/Async: Synchronous
 * Reentrancy: Non-reentrant
 * Parameters (in): ConfigPtr - Pointer to post-build configuration data
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: None
 * Description: Function to Setup the pin configuration:
 *              - Setup the pin as Digital or Analog pin
 *              - Setup the pin mode 
 *              - Setup the direction of the pin
 *              - Provide initial value for o/p pin
 *              - Setup the internal resistor for i/p pin
 ************************************************************************************/
void Port_Init(const Port_ConfigType * ConfigPtr)
{
	volatile uint32 * PortBase_Ptr = NULL_PTR; /* point to the required Port Registers base address */
	volatile uint32 delay = 0;

	//
#if (PORT_DEV_ERROR_DETECT == STD_ON)
	/* check if the input configuration pointer is not a NULL_PTR */
	if (NULL_PTR == ConfigPtr)
	{
		Det_ReportError(PORT_MODULE_ID, 
				PORT_INSTANCE_ID, 
				PORT_INIT_SID,
				PORT_E_PARAM_CONFIG);
	}
	else
#endif
	{
		/*
		 * Set the module state to initialized and point to the PB configuration structure using a global pointer.
		 * This global pointer is global to be used by other functions to read the PB configuration structures
		 */
		Port_Status = PORT_INITIALIZED;
		Port_ConfigPtr = ConfigPtr; /* address of the first structure */
	}
	
	for(uint8 i=0;i<PORT_CONFIGURED_CHANNLES;i++)  /* to loop on all structures in the array of all configured pins */
        {
		switch(Port_ConfigPtr[i].port_num)
		{
		case  0: PortBase_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
		break;
		case  1: PortBase_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
		break;
		case  2: PortBase_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
		break;
		case  3: PortBase_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
		break;
		case  4: PortBase_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
		break;
		case  5: PortBase_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
		break;
		}

		/* Enable clock for PORT and allow time for clock to start*/
		SYSCTL_REGCGC2_REG |= (1<<Port_ConfigPtr[i].port_num);
		delay = SYSCTL_REGCGC2_REG;

		if( ((Port_ConfigPtr[i].port_num == PORTD) && (Port_ConfigPtr[i].pin_num == PIN7)) || ((Port_ConfigPtr[i].port_num == PORTF) && (Port_ConfigPtr[i].pin_num == PIN0)) ) /* PD7 or PF0 */
		{
			*(volatile uint32 *)((volatile uint8 *)PortBase_Ptr + PORT_LOCK_REG_OFFSET) = PORT_LOCK_VALUE;                     /* Unlock the GPIOCR register */   

			SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortBase_Ptr + PORT_COMMIT_REG_OFFSET) , Port_ConfigPtr[i].pin_num);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
		}

		else if( (Port_ConfigPtr[i].port_num == PORTC) && (Port_ConfigPtr[i].pin_num <= PIN3) ) /* PC0 to PC3 */
		{
			/* Do Nothing ...  this is the JTAG pins */
		}

		else
		{
			/* Do Nothing ... No need to unlock the commit register for this pin */
		}


		/* GPIO mode */
                if(Port_ConfigPtr[i].mode == GPIO){ 
			SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortBase_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_ConfigPtr[i].pin_num);         /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */

			CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortBase_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_ConfigPtr[i].pin_num);      /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */

			CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortBase_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_ConfigPtr[i].pin_num);             /* Disable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */ 

			*(volatile uint32 *)((volatile uint8 *)PortBase_Ptr + PORT_CTL_REG_OFFSET) &= ~(PORT_PCTL_REG_MASK << (Port_ConfigPtr[i].pin_num * 4));     /* Clear the PMCx bits for this pin */
		} 

		/* ADC mode */
                else if(Port_ConfigPtr[i].mode == ADC){ 
			CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortBase_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_ConfigPtr[i].pin_num);        /* Clear the corresponding bit in the DEN register to enable digital functionality on this pin */

			SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortBase_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_ConfigPtr[i].pin_num);         /* Set the corresponding bit in the AMSEL register to disable analog functionality on this pin */

			CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortBase_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_ConfigPtr[i].pin_num);              /* Disable Alternative function for this pin by clear the corresponding bit in AFSEL register */ 

			*(volatile uint32 *)((volatile uint8 *)PortBase_Ptr + PORT_CTL_REG_OFFSET) &= ~(PORT_PCTL_REG_MASK << (Port_ConfigPtr[i].pin_num * 4));      /* Clear the PMCx bits for this pin */
		} 

		/* Other modes */
                else{ 
			SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortBase_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_ConfigPtr[i].pin_num);         /* Set the corresponding bit in the DEN register to enable digital functionality on this pin */

			CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortBase_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_ConfigPtr[i].pin_num);      /* Clear the corresponding bit in the AMSEL register to disable analog functionality on this pin */

			SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortBase_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_ConfigPtr[i].pin_num);               /* Enable Alternative function for this pin by clear the corresponding bit in AFSEL register */ 

			*(volatile uint32 *)((volatile uint8 *)PortBase_Ptr + PORT_CTL_REG_OFFSET) &= ~(PORT_PCTL_REG_MASK << (Port_ConfigPtr[i].pin_num * 4));
			
                          *(volatile uint32 *)((volatile uint8 *)PortBase_Ptr + PORT_CTL_REG_OFFSET) |=(Port_ConfigPtr[i].mode << (Port_ConfigPtr[i].pin_num * 4));    /* Put the PMCx bits for this pin and put corresponding pctl value */
		} 


		/* Output direction */
                if(Port_ConfigPtr[i].direction == PORT_PIN_OUT)
		{
			SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortBase_Ptr + PORT_DIR_REG_OFFSET) , Port_ConfigPtr[i].pin_num);                /* Set the corresponding bit in the GPIODIR register to configure it as output pin */

			if(Port_ConfigPtr[i].initial_value == STD_HIGH)
			{
				SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortBase_Ptr + PORT_DATA_REG_OFFSET) , Port_ConfigPtr[i].pin_num);          /* Set the corresponding bit in the GPIODATA register to provide initial value 1 */
			}
			else
			{
				CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortBase_Ptr + PORT_DATA_REG_OFFSET) , Port_ConfigPtr[i].pin_num);        /* Clear the corresponding bit in the GPIODATA register to provide initial value 0 */
			}
		}
                
                /* Input direction */
		else if(Port_ConfigPtr[i].direction == PORT_PIN_IN)
		{
			CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortBase_Ptr + PORT_DIR_REG_OFFSET) , Port_ConfigPtr[i].pin_num);             /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */

			if(Port_ConfigPtr[i].resistor == PULL_UP)
			{
				SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortBase_Ptr + PORT_PULL_UP_REG_OFFSET) , Port_ConfigPtr[i].pin_num);       /* Set the corresponding bit in the GPIOPUR register to enable the internal pull up pin */
			}
			else if(Port_ConfigPtr[i].resistor == PULL_DOWN)
			{
				SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortBase_Ptr + PORT_PULL_DOWN_REG_OFFSET) , Port_ConfigPtr[i].pin_num);     /* Set the corresponding bit in the GPIOPDR register to enable the internal pull down pin */
			}
			else
			{
				CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortBase_Ptr + PORT_PULL_UP_REG_OFFSET) , Port_ConfigPtr[i].pin_num);     /* Clear the corresponding bit in the GPIOPUR register to disable the internal pull up pin */
				CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortBase_Ptr + PORT_PULL_DOWN_REG_OFFSET) , Port_ConfigPtr[i].pin_num);   /* Clear the corresponding bit in the GPIOPDR register to disable the internal pull down pin */
			}
		}
		else
		{
			/* Do Nothing */
		}

	}
}

/************************************************************************************
 * Service Name: Port_SetPinDirection
 * Service ID[hex]: 0x01
 * Sync/Async: Synchronous
 * Reentrancy: Reentrant
 * Parameters (in): Pin - Direction
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: None
 * Description: Sets the port pin direction
 ************************************************************************************/
void Port_SetPinDirection(Port_PinType Pin,Port_PinDirectionType Direction){

#if (PORT_DEV_ERROR_DETECT == STD_ON)
	/* check if the input configuration pointer is not a NULL_PTR */
	if (Port_ConfigPtr[Pin].change_dir == FALSE) 
	{
		Det_ReportError(PORT_MODULE_ID, 
				PORT_INSTANCE_ID, 
				PORT_SET_PIN_DIRECTION_SID,
				PORT_E_DIRECTION_UNCHANGEABLE);
	}
	else
	{
		/* No Action Required */
	}


	/* Check if the Driver is initialized before using this function */
	if (PORT_NOT_INITIALIZED == Port_Status)
	{
		Det_ReportError(PORT_MODULE_ID, 
				PORT_INSTANCE_ID,
				PORT_SET_PIN_DIRECTION_SID, 
				PORT_E_UNINIT);

	}
	else
	{
		/* No Action Required */
	}


	/* Check if the used Pin is within the valid range */
	if (PORT_CONFIGURED_CHANNLES <= Pin)
	{

		Det_ReportError(PORT_MODULE_ID, 
				PORT_INSTANCE_ID,
				PORT_SET_PIN_DIRECTION_SID, 
				PORT_E_PARAM_PIN);
		
	}
	else
	{
		/* No Action Required */
	}


#endif

	volatile uint32 * PortBase_Ptr = NULL_PTR; /* point to the required Port Registers base address */
	volatile uint32 delay = 0;

	switch(Port_ConfigPtr[Pin].port_num)
	{
	case  0: PortBase_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
	break;
	case  1: PortBase_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
	break;
	case  2: PortBase_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
	break;
	case  3: PortBase_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
	break;
	case  4: PortBase_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
	break;
	case  5: PortBase_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
	break;
	}

	/* In case of output */
        if(Direction == PORT_PIN_OUT)
	{
		SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortBase_Ptr + PORT_DIR_REG_OFFSET) , Port_ConfigPtr[Pin].pin_num);                /* Set the corresponding bit in the GPIODIR register to configure it as output pin */
	}
        
        /* In case of input */
	else if(Direction == PORT_PIN_IN)
	{
		CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortBase_Ptr + PORT_DIR_REG_OFFSET) , Port_ConfigPtr[Pin].pin_num);             /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */
	}
	else
	{
		/* Do Nothing */
	}

}

/************************************************************************************
 * Service Name: Port_SetPinMode
 * Service ID[hex]: 0x04
 * Sync/Async: Synchronous
 * Reentrancy: Reentrant
 * Parameters (in): Pin - Mode
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: None
 * Description: Sets the port pin mode
 ************************************************************************************/
void Port_SetPinMode(Port_PinType Pin,Port_PinModeType Mode){

#if (PORT_DEV_ERROR_DETECT == STD_ON)
	/* check if the input configuration pointer is not a NULL_PTR */
	if (Port_ConfigPtr[Pin].change_mode == FALSE) 
	{
		Det_ReportError(PORT_MODULE_ID, 
				PORT_INSTANCE_ID, 
				PORT_SET_PIN_MODE_SID,
				PORT_E_MODE_UNCHANGEABLE);
	}
	else
	{
		/* No Action Required */
	}


	/* Check if the Driver is initialized before using this function */
	if (PORT_NOT_INITIALIZED == Port_Status)
	{
		Det_ReportError(PORT_MODULE_ID, 
				PORT_INSTANCE_ID,
				PORT_SET_PIN_MODE_SID, 
				PORT_E_UNINIT);
	}
	else
	{
		/* No Action Required */
	}


	/* Check if the used Pin is within the valid range */
	if (PORT_CONFIGURED_CHANNLES <= Pin)
	{

		Det_ReportError(PORT_MODULE_ID, 
				PORT_INSTANCE_ID,
				PORT_SET_PIN_MODE_SID, 
				PORT_E_PARAM_PIN);

	}
	else
	{
		/* No Action Required */
	}


	/* Check if the used mode is available */
	if (PORT_MAX_MODE_NUMBER < Mode)
	{

		Det_ReportError(PORT_MODULE_ID, 
				PORT_INSTANCE_ID,
				PORT_SET_PIN_MODE_SID, 
				PORT_E_PARAM_INVALID_MODE);

	}
	else
	{
		/* No Action Required */
	}

#endif

	volatile uint32 * PortBase_Ptr = NULL_PTR; /* point to the required Port Registers base address */
	volatile uint32 delay = 0;

	switch(Port_ConfigPtr[Pin].port_num)
	{
	case  0: PortBase_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
	break;
	case  1: PortBase_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
	break;
	case  2: PortBase_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
	break;
	case  3: PortBase_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
	break;
	case  4: PortBase_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
	break;
	case  5: PortBase_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
	break;
	}

	/* GPIO Mode */
        if(Mode == GPIO){ 
		SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortBase_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_ConfigPtr[Pin].pin_num);         /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */

		CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortBase_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_ConfigPtr[Pin].pin_num);      /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */

		CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortBase_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_ConfigPtr[Pin].pin_num);             /* Disable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */ 

		*(volatile uint32 *)((volatile uint8 *)PortBase_Ptr + PORT_CTL_REG_OFFSET) &= ~(PORT_PCTL_REG_MASK << ((Port_ConfigPtr[Pin].pin_num) * 4));     /* Clear the PMCx bits for this pin */
	} 

	/* ADC Mode */
        else if(Mode == ADC){ 
		CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortBase_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_ConfigPtr[Pin].pin_num);        /* Clear the corresponding bit in the GPIODEN register to enable digital functionality on this pin */

		SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortBase_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_ConfigPtr[Pin].pin_num);         /* Set the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */

		CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortBase_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_ConfigPtr[Pin].pin_num);              /* Disable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */ 

		*(volatile uint32 *)((volatile uint8 *)PortBase_Ptr + PORT_CTL_REG_OFFSET) &= ~(PORT_PCTL_REG_MASK << ((Port_ConfigPtr[Pin].pin_num) * 4));      /* Clear the PMCx bits for this pin */
	} 

	/* Other Modes */
        else{ 
		SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortBase_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_ConfigPtr[Pin].pin_num);         /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */

		CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortBase_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_ConfigPtr[Pin].pin_num);      /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */

		SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortBase_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_ConfigPtr[Pin].pin_num);               /* Enable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */ 

		*(volatile uint32 *)((volatile uint8 *)PortBase_Ptr + PORT_CTL_REG_OFFSET) &= ~(PORT_PCTL_REG_MASK << ((Port_ConfigPtr[Pin].pin_num) * 4));
		*(volatile uint32 *)((volatile uint8 *)PortBase_Ptr + PORT_CTL_REG_OFFSET) |= (Mode << ((Port_ConfigPtr[Pin].pin_num) * 4));          /* put the PMCx bits for this pin */
	} 
}

/************************************************************************************
 * Service Name: Port_RefreshPortDirection
 * Service ID[hex]: 0x02
 * Sync/Async: Synchronous
 * Reentrancy: Non Reentrant
 * Parameters (in): None
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: None
 * Description: Refreshes port direction
 ************************************************************************************/
void Port_RefreshPortDirection(void){

#if (PORT_DEV_ERROR_DETECT == STD_ON)

	/* Check if the Driver is initialized before using this function */
	if (PORT_NOT_INITIALIZED == Port_Status)
	{
		Det_ReportError(PORT_MODULE_ID, 
				PORT_INSTANCE_ID,
				PORT_REFRESH_PORT_DIRECTION_SID, 
				PORT_E_UNINIT);
	}
	else
	{
		/* No Action Required */
	}
#endif

	volatile uint32 * PortBase_Ptr = NULL_PTR; /* point to the required Port Registers base address */
	volatile uint32 delay = 0;

	for(uint8 i=0;i<PORT_CONFIGURED_CHANNLES;i++)  /* To loop on all configured pins */
        {
		switch(Port_ConfigPtr[i].port_num)
		{
		case  0: PortBase_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
		break;
		case  1: PortBase_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
		break;
		case  2: PortBase_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
		break;
		case  3: PortBase_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
		break;
		case  4: PortBase_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
		break;
		case  5: PortBase_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
		break;
		}

		if((Port_ConfigPtr[i].port_num == PORTC) && (Port_ConfigPtr[i].pin_num <= PIN3))
		{
			/* Do Nothing */ /* JTAG PINS */
		}

		else
		{

			if((Port_ConfigPtr[i].direction == PORT_PIN_OUT) && (Port_ConfigPtr[i].change_dir == UNCHANGEABLE))
			{
				SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortBase_Ptr + PORT_DIR_REG_OFFSET) , Port_ConfigPtr[i].pin_num);                /* Set the corresponding bit in the GPIODIR register to configure it as output pin */
			}

			else if((Port_ConfigPtr[i].direction == PORT_PIN_IN) && (Port_ConfigPtr[i].change_dir == UNCHANGEABLE))
			{
				CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortBase_Ptr + PORT_DIR_REG_OFFSET) , Port_ConfigPtr[i].pin_num);             /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */
			}

			else
			{
				/* Do Nothing */
			}
		}
	} 
}