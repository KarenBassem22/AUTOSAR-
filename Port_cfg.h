/******************************************************************************
 * Module: PORT
 *
 * File Name: PORT_Cfg.h
 *
 * Description: Pre-Compile Configuration Header file for TM4C123GH6PM Microcontroller - PORT Driver
 *
 * Author: Karen Bassem
 ******************************************************************************/

#ifndef PORT_CFG_H
#define PORT_CFG_H

/* Module Version 1.0.0 */
#define PORT_CFG_SW_MAJOR_VERSION              (1U)
#define PORT_CFG_SW_MINOR_VERSION              (0U)
#define PORT_CFG_SW_PATCH_VERSION              (0U)

/* AUTOSAR Version 4.0.3 */
#define PORT_CFG_AR_RELEASE_MAJOR_VERSION     (4U)
#define PORT_CFG_AR_RELEASE_MINOR_VERSION     (0U)
#define PORT_CFG_AR_RELEASE_PATCH_VERSION     (3U)

/* Pre-compile option for Development Error Detect */
#define PORT_DEV_ERROR_DETECT                (STD_ON)

/* Pre-compile option for Version Info API */
#define PORT_VERSION_INFO_API                (STD_OFF)

/* Pre-compile option for Set Pin Direction API */
#define PORT_SET_PIN_DIRECTION_API           (TRUE)

/* Pre-compile option for Set Pin Mode API */
//#define PORT_PIN_MODE_CHANGEABLE           (TRUE)

/* Number of the configured PORT Channels */
#define PORT_CONFIGURED_CHANNLES              (39U)

/* Max number of available modes */
#define PORT_MAX_MODE_NUMBER                  (15U)


/* PORT Configured Port and Pin ID's  */

/* LEDS */
#define PORT_LED1_PORT_NUM       PORTF 
#define PORT_LED1_PIN_NUM        PIN1  
#define PORT_LED2_PORT_NUM       PORTF 
#define PORT_LED2_PIN_NUM        PIN2  
#define PORT_LED3_PORT_NUM       PORTF 
#define PORT_LED3_PIN_NUM        PIN3  

/* Switches */
#define PORT_SW1_PORT_NUM        PORTF
#define PORT_SW1_PIN_NUM         PIN4
#define PORT_SW2_PORT_NUM        PORTF
#define PORT_SW2_PIN_NUM         PIN0

/* Channel Index in the array of structures in Port_PBcfg.c */ 
#define PORT_LED1_PIN_ID_INDEX   (uint8)0x23 /*35*/
#define PORT_LED2_PIN_ID_INDEX   (uint8)0x24 /*36*/
#define PORT_LED3_PIN_ID_INDEX   (uint8)0x25 /*37*/
#define PORT_SW1_PIN_ID_INDEX    (uint8)0x26 /*38*/
#define PORT_SW2_PIN_ID_INDEX    (uint8)0x22 /*34*/

#endif /* PORT_H */