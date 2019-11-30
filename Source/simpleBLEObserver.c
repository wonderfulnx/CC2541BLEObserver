/**************************************************************************************************
  Filename:       simpleBLEObserver.c
  Revised:        $Date: 2011-06-20 11:57:59 -0700 (Mon, 20 Jun 2011) $
  Revision:       $Revision: 28 $

  Description:    This file contains the Simple BLE Observer sample application 
                  for use with the CC2540 Bluetooth Low Energy Protocol Stack.

  Copyright 2011 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED �AS IS�?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "OnBoard.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_lcd.h"
#include "ll.h"
#include "hci.h"

#include "observer.h"
// #include <ioCC2540.h>
#include <string.h>

#include "simpleBLEObserver.h"

/*********************************************************************
 * MACROS
 */

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

/*********************************************************************
 * CONSTANTS
 */

// Maximum number of scan responses
#define DEFAULT_MAX_SCAN_RES                  32

// Scan duration in ms
#define DEFAULT_SCAN_DURATION                 50

// Discovey mode (limited, general, all)
#define DEFAULT_DISCOVERY_MODE                DEVDISC_MODE_ALL

// TRUE to use active scan
#define DEFAULT_DISCOVERY_ACTIVE_SCAN         FALSE

// TRUE to use white list during discovery
#define DEFAULT_DISCOVERY_WHITE_LIST          FALSE

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
static char HEX[] = "0123456789ABCDEF";

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

// Task ID for internal task/event processing
static uint8 simpleBLETaskId;

// GAP GATT Attributes
//static const uint8 simpleBLEDeviceName[GAP_DEVICE_NAME_LEN] = "Simple BLE Observer";

// Scanning state
static uint8 simpleBLEScanning = FALSE;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void simpleBLEObserverEventCB( gapObserverRoleEvent_t *pEvent );
static void simpleBLEObserver_HandleKeys( uint8 shift, uint8 keys );
static void simpleBLEObserver_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void simpleBLEAddDeviceInfo( uint8 *pAddr, uint8 *data, uint8 len );
char *bdAddr2Str ( uint8 *pAddr );
char *data2str( uint8 *data, uint8 len );
char *num2str(uint32 v);
void uartSendString(char* data, int len);
void uartSendConstString(char* data);
void analyzePakcet(uint8 *pData, uint8 dataLen);
void processBeacon(uint8 *uuid, uint8 *marjor, uint8* minor, uint8* tx_power);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static const gapObserverRoleCB_t simpleBLERoleCB =
{
  NULL,                     // RSSI callback
  simpleBLEObserverEventCB  // Event callback
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleBLEObserver_Init
 *
 * @brief   Initialization function for the Simple BLE Observer App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notification).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void SimpleBLEObserver_Init( uint8 task_id )
{
  simpleBLETaskId = task_id;

  // Setup Observer Profile
  {
    uint8 scanRes = DEFAULT_MAX_SCAN_RES;
    GAPObserverRole_SetParameter ( GAPOBSERVERROLE_MAX_SCAN_RES, sizeof( uint8 ), &scanRes );
  }
  
  // Setup GAP
  GAP_SetParamValue( TGAP_GEN_DISC_SCAN, DEFAULT_SCAN_DURATION );
  GAP_SetParamValue( TGAP_LIM_DISC_SCAN, DEFAULT_SCAN_DURATION );
  GAP_SetParamValue( TGAP_FILTER_ADV_REPORTS, FALSE);

  // Register for all key events - This app will handle all key events
  RegisterForKeys( simpleBLETaskId );
  
  // makes sure LEDs are off
  HalLedSet( (HAL_LED_1 | HAL_LED_2), HAL_LED_MODE_OFF );
  
  // Setup a delayed profile startup
  osal_set_event( simpleBLETaskId, START_DEVICE_EVT );
  
  // Set up serial port
  PERCFG = 0x00;
  P0SEL = 0x0c;
  P2DIR &= ~0xC0;
  
  U0CSR |= 0x80;
  U0GCR |= 11;
  U0BAUD |= 216;
  UTX0IF = 0;
  U0CSR &= ~0x40;
  IEN0 |= 0x84;

  uartSendConstString("Initiated.\r\n");
}

/*********************************************************************
 * @fn      SimpleBLEObserver_ProcessEvent
 *
 * @brief   Simple BLE Observer Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 SimpleBLEObserver_ProcessEvent( uint8 task_id, uint16 events )
{
  
  VOID task_id; // OSAL required parameter that isn't used in this function
  
  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( simpleBLETaskId )) != NULL )
    {
      simpleBLEObserver_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & START_DEVICE_EVT )
  {
    // Start the Device
    VOID GAPObserverRole_StartDevice( (gapObserverRoleCB_t *) &simpleBLERoleCB );

    return ( events ^ START_DEVICE_EVT );
  }
  
  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      simpleBLEObserver_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void simpleBLEObserver_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
    case KEY_CHANGE:
      simpleBLEObserver_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
      break;

    case GATT_MSG_EVENT:
      break;
  }
}

/*********************************************************************
 * @fn      simpleBLEObserver_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
uint8 gStatus;
static void simpleBLEObserver_HandleKeys( uint8 shift, uint8 keys )
{
  (void)shift;  // Intentionally unreferenced parameter

  if ( keys & HAL_KEY_UP )
  {
    // Start or stop discovery
    if ( !simpleBLEScanning )
    {
      simpleBLEScanning = TRUE;
      
      uartSendConstString("Discovering...\r\n");
      
      GAPObserverRole_StartDiscovery( DEFAULT_DISCOVERY_MODE,
                                      DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                      DEFAULT_DISCOVERY_WHITE_LIST );
    }
    else
    {
      GAPObserverRole_CancelDiscovery();
      uartSendConstString("discovery complete\r\n");
      simpleBLEScanning = FALSE;
    }
  }

  if ( keys & HAL_KEY_LEFT )
  {
  }

  if ( keys & HAL_KEY_RIGHT )
  {
  }
  
  if ( keys & HAL_KEY_CENTER )
  {
  }
  
  if ( keys & HAL_KEY_DOWN )
  {
  }
}

/*********************************************************************
 * @fn      simpleBLEObserverEventCB
 *
 * @brief   Observer event callback function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  none
 */
static void simpleBLEObserverEventCB( gapObserverRoleEvent_t *pEvent )
{
  switch ( pEvent->gap.opcode )
  {
    case GAP_DEVICE_INIT_DONE_EVENT:  
      {
        uartSendConstString("Init done, BLE Observer: ");
        uartSendConstString(bdAddr2Str( pEvent->initDone.devAddr));
        uartSendConstString("\r\n");
      }
      break;

    case GAP_DEVICE_INFO_EVENT:
      {
        if (simpleBLEScanning) {
          simpleBLEAddDeviceInfo( pEvent->deviceInfo.addr, pEvent->deviceInfo.pEvtData, pEvent->deviceInfo.dataLen );
        }
      }
      break;
      
    case GAP_DEVICE_DISCOVERY_EVENT:
      {
        // finish a round of scanning, start another
        GAPObserverRole_StartDiscovery( DEFAULT_DISCOVERY_MODE,
                                      DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                      DEFAULT_DISCOVERY_WHITE_LIST );
      }
      break;
      
    default:
      break;
  }
}

static void simpleBLEAddDeviceInfo( uint8 *pAddr, uint8 *data, uint8 len )
{
  // filter the wanted beacon signal here
  // char* add = bdAddr2Str(pAddr);
  // uartSendConstString(add);
  // if (len < 40){
  //   char* s = data2str(data, len);
  //   uartSendConstString("  ");
  //   uartSendConstString(s);
  // }
  // uartSendConstString("\r\n");
  analyzePakcet(data, len);
}

/*********************************************************************
 * @fn      bdAddr2Str
 *
 * @brief   Convert Bluetooth address to string
 *
 * @return  none
 */
char *bdAddr2Str( uint8 *pAddr )
{
  uint8       i;
  static char str[B_ADDR_STR_LEN];
  char        *pStr = str;
  
  *pStr++ = '0';
  *pStr++ = 'x';
  
  // Start from end of addr
  pAddr += B_ADDR_LEN;
  
  for ( i = B_ADDR_LEN; i > 0; i-- )
  {
    *pStr++ = HEX[*--pAddr >> 4];
    *pStr++ = HEX[*pAddr & 0x0F];
  }
  
  *pStr = 0;
  
  return str;
}

char *data2str( uint8 *data, uint8 len)
{
  uint8 i;
  static char str[80];
  char *pStr = str;
  
  // Start from end of addr
  // data += len;
  
  for ( i = len; i > 0; i-- )
  {
    *pStr++ = HEX[*data >> 4];
    *pStr++ = HEX[*data & 0x0F];
    data++;
  }
  
  *pStr = 0;
  
  return str;
}

char *num2str(uint32 v) {
  static char str_rev[16];
  static char str[16];
  char *pStr = str_rev;
  char *qStr = str;
  
  *pStr++ = '0' + (v % 10); v /= 10;
  while(v!=0) { *pStr++ = '0' + (v % 10); v /= 10; }
  while (pStr > str_rev) { *qStr++ = *--pStr; }
  *qStr = 0;

  return str;
}


void processBeacon(uint8 *uuid, uint8 *marjor, uint8* minor, uint8* tx_power) {
  uartSendConstString(num2str(osal_GetSystemClock()));
  uartSendConstString(" ");
  uartSendConstString(data2str(uuid, 16));
  uartSendConstString(" ");
  uartSendConstString(data2str(marjor, 2));
  uartSendConstString(" ");
  uartSendConstString(data2str(minor, 2));
  uartSendConstString(" ");
  uartSendConstString(data2str(tx_power, 1));
  uartSendConstString("\r\n");
}
void analyzePakcet(uint8 *pData, uint8 dataLen) {
  uint8 adLen;
  uint8 adType;
  uint8 *pEnd;
  pEnd = pData + dataLen - 1;
  
  // While end of data not reached
  while ( pData < pEnd ) {
    // Get length of next AD item
    adLen = *pData++;
    if ( adLen > 0 ) {
      adType = *pData;
      
      // If AD type is Manufacturer Specific Data
      if ( adType == GAP_ADTYPE_MANUFACTURER_SPECIFIC) {
        pData += 3;
        // ibeacon
        if (*pData == 0x02 && *(pData+1) == 0x15) processBeacon(pData+2, pData+18, pData+20, pData+22);
        // altbeacon
        if (*pData == 0xBE && *(pData+1) == 0xAC) processBeacon(pData+2, pData+18, pData+20, pData+22);
      }
      else pData += adLen;
    }
  }
  
}

void uartSendString(char* data, int len) {
  int i;
  for (i = 0; i < len; i++) {
    U0DBUF = *data++;
    while (UTX0IF == 0);
    UTX0IF = 0;
  }
}

void uartSendConstString(char* data) {
  while (*data) {
    U0DBUF = *data++;
    while (UTX0IF == 0);
    UTX0IF = 0;
  }
}
/*********************************************************************
*********************************************************************/
