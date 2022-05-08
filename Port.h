/******************************************************************************
 * Module: Port
 *
 * File Name: Port.h
 *
 * Description: Header file for TM4C123GH6PM Microcontroller - Port Driver.
 *
 * Author: Karen Bassem
 ******************************************************************************/

#ifndef PORT_H
#define PORT_H

/* Non - AUTOSAR */
#include "Common_Macros.h"


/* Id for the company in the AUTOSAR*/
#define PORT_VENDOR_ID                 (1000U)

/* Port Module Id */
#define PORT_MODULE_ID                       (124U)

/* Instance Id */
#define PORT_INSTANCE_ID                      (0U)

/* Macros for Port Status */
#define PORT_INITIALIZED                      (1U)
#define PORT_NOT_INITIALIZED                  (0U)


/* Port Software Module Version 1.0.0 */
#define PORT_SW_MAJOR_VERSION                 (1U)
#define PORT_SW_MINOR_VERSION                 (0U)
#define PORT_SW_PATCH_VERSION                 (0U)

/* AUTOSAR Version 4.0.3 */
#define PORT_AR_RELEASE_MAJOR_VERSION         (4U)
#define PORT_AR_RELEASE_MINOR_VERSION         (0U)
#define PORT_AR_RELEASE_PATCH_VERSION         (3U)


#include "Std_Types.h"

/* AUTOSAR checking between Std Types and Port Modules */
#if ((STD_TYPES_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
		||  (STD_TYPES_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
		||  (STD_TYPES_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
#error "The AR version of Std_Types.h does not match the expected version"
#endif


/* Port Pre-Compile Configuration Header file */
#include "Port_Cfg.h"

/* AUTOSAR Version checking between Port_Cfg.h and Port.h files */
#if ((PORT_CFG_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
		||  (PORT_CFG_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
		||  (PORT_CFG_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
#error "The AR version of Port_Cfg.h does not match the expected version"
#endif

/* Software Version checking between Port_Cfg.h and Port.h files */
#if ((PORT_CFG_SW_MAJOR_VERSION != PORT_SW_MAJOR_VERSION)\
		||  (PORT_CFG_SW_MINOR_VERSION != PORT_SW_MINOR_VERSION)\
		||  (PORT_CFG_SW_PATCH_VERSION != PORT_SW_PATCH_VERSION))
#error "The SW version of Port_Cfg.h does not match the expected version"
#endif

/******************************************************************************
 *                      API Service Id Macros                                 *
 ******************************************************************************/

/* service ID for PORT set pin direction */
#define PORT_SET_PIN_DIRECTION_SID                 (0x01)

/* service ID for PORT init */
#define PORT_INIT_SID                              (0x00)

/* service ID for PORT refresh port direction */
#define PORT_REFRESH_PORT_DIRECTION_SID            (0x02)

/* service ID for PORT get version info */
#define PORT_GET_VERSION_INFO_SID                  (0x03)

/* service ID for PORT set pin mode */
#define PORT_SET_PIN_MODE_SID                      (0x04)

/*******************************************************************************
 *                      DET Error Codes                                        *
 *******************************************************************************/
/* Invalid Port Pin ID requested */
#define PORT_E_PARAM_PIN               (0x0A)

/* Port Pin not configured as changeable */
#define PORT_E_DIRECTION_UNCHANGEABLE  (0x0B)

/* API Port_Init service called with wrong parameter */
#define PORT_E_PARAM_CONFIG            (0x0C)

/* API Port_SetPinMode service called when mode is unchangeable */
#define PORT_E_PARAM_INVALID_MODE      (0x0D)
#define PORT_E_MODE_UNCHANGEABLE       (0x0E)

/* API service called without module initialization */
#define PORT_E_UNINIT                  (0x0F)

/* APIs called with a Null Pointer */
#define PORT_E_PARAM_POINTER           (0x10)

/*******************************************************************************
 *                              Module Definitions                             *
 *******************************************************************************/

/* Some mode values */
#define GPIO 0000
#define ADC  1111
#define U1RX 0001
#define U1TX 0001

#define CHANGEABLE 1
#define UNCHANGEABLE 0  

#define PORT_LOCK_VALUE        0x4C4F434B
#define PORT_PCTL_REG_MASK     0x0000000F

/*******************************************************************************
 *                              Module Data Types                              *
 *******************************************************************************/

/* Description: Enum to hold PIN direction */
typedef enum
{
	PORT_PIN_IN,PORT_PIN_OUT
}Port_PinDirectionType;

/* Description: Enum to hold internal resistor type for PIN */
typedef enum
{
	OFF,PULL_UP,PULL_DOWN
}Port_InternalResistor;

/* Description: Enum to hold all port pins available */
typedef enum
{
	PA0,PA1,PA2,PA3,PA4,PA5,PA6,PA7,PB0,PB1,PB2,PB3,PB4,PB5,PB6,PB7,PC0,PC1,
	PC2,PC3,PC4,PC5,PC6,PC7,PD0,PD1,PD2,PD3,PD4,PD5,PD6,PD7,PE0,PE1,PE2,PE3,
	PE4,PE5,PF0,PF1,PF2,PF3,PF4
}Port_PinID;

/* Description: Enum to hold port ID of the microcontroller */
typedef enum{
	PORTA,PORTB,PORTC,PORTD,PORTE,PORTF
};

/* Description: Enum to hold pin ID of the microcontroller */
typedef enum{
	PIN0,PIN1,PIN2,PIN3,PIN4,PIN5,PIN6,PIN7
};

/* Type definition for Port_Pin_Mode_Type used by Port API */
typedef uint8 Port_PinModeType;  

/* Type definition for Port_Pin ID number used by Port API */
typedef uint8 Port_PinType; 

/* Type definition for Port_PinDirectionChangeable used by Port API */
typedef uint8 Port_PinDirectionChangeable;

/* Type definition for Port_PinModeChangeable used by Port API */
typedef uint8 Port_PinModeChangeable; 


/* Description: Structure to configure each individual PIN:
        1. the PORT Which the pin belongs to. 0, 1, 2, 3, 4 or 5
 	2. the number of the pin in the PORT.
        3. the direction of pin --> INPUT or OUTPUT
        4. the internal resistor --> Disable, Pull up or Pull down
        5. the initial value of pin --> HIGH or LOW
        6. the mode of pin --> GPIO,ADC,...
        7. the direction is changeable or not
        8. the mode is changeable or not */

typedef struct 
{
	uint8 port_num; 
	uint8 pin_num; 
	Port_PinDirectionType direction;
	Port_InternalResistor resistor;
	uint8 initial_value;
	Port_PinModeType mode; 
	Port_PinDirectionChangeable change_dir; 
	Port_PinModeChangeable change_mode; 
}Port_ConfigType;


/* struct{
	Port_ConfigType ports[PORT_CONFIGURED_CHANNLES];
}Port_ConfigTypeArray;*/

/*******************************************************************************
 *                      Function Prototypes                                    *
 *******************************************************************************/

/* Function for PORT initialization */
void Port_Init(const Port_ConfigType* ConfigPtr); 

/* Function for PORT Set Pin Direction API */
#if( PORT_SET_PIN_DIRECTION_API == TRUE)
void Port_SetPinDirection(Port_PinType Pin,Port_PinDirectionType Direction); 
#endif

/* Function for PORT to set pin mode */
void Port_SetPinMode(Port_PinType Pin,Port_PinModeType Mode); 

/* Function for PORT Get Version Info API */
#if (PORT_VERSION_INFO_API == STD_ON)
void Port_GetVersionInfo(Std_VersionInfoType* versioninfo); 
#endif

/* Function for PORT to refresh port direction */
void Port_RefreshPortDirection(void); 

/*******************************************************************************
 *                       External Variables                                    *
 *******************************************************************************/

/* Extern PB array of structures to be used by Port and other modules */
extern const Port_ConfigType Port_Configuration[PORT_CONFIGURED_CHANNLES];

#endif /* PORT_H */