/* Copyright (c) Nordic Semiconductor ASA
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 *   1. Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * 
 *   2. Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 * 
 *   3. Neither the name of Nordic Semiconductor ASA nor the names of other
 *   contributors to this software may be used to endorse or promote products
 *   derived from this software without specific prior written permission.
 * 
 *   4. This software must only be used in a processor manufactured by Nordic
 *   Semiconductor ASA, or in a processor manufactured by a third party that
 *   is used in combination with a processor manufactured by Nordic Semiconductor.
 * 
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */

#include <stdint.h>
#include <string.h>
#include "nrfcloud_tutorial_service.h"
#include "ble_srv_common.h"
#include "app_error.h"
#include "SEGGER_RTT.h"
#include "bsp.h"

#define TUT_LED_0_MASK      1
#define TUT_LED_1_MASK      2
#define TUT_LED_2_MASK      4
#define TUT_LED_3_MASK      8

void ble_tutorial_service_on_ble_evt(ble_os_t * p_our_service, ble_evt_t * p_ble_evt)
{
 uint32_t            err_code;   
	
	switch (p_ble_evt->header.evt_id)
{
    case BLE_GAP_EVT_CONNECTED:
      p_our_service->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
			SEGGER_RTT_WriteString(0, "nRFCloud Demo device connected.\n");
				 
		// Read nRFCloud Demo characteristic value
			ble_gatts_value_t      temp;								// declare and reserve memory for charcateristic structure
			memset(&temp, 0, sizeof(ble_gatts_value_t));
			uint8_t tutorial_char_value;								// define a variable to hold char value when read
			temp.p_value   = &tutorial_char_value;			// add pointer to characteristic structure
			temp.len      = 4;
			temp.offset   = 0;					
							
			err_code = sd_ble_gatts_value_get(p_our_service->conn_handle, p_our_service->char_handles.value_handle, &temp); // DONE: remove hard coded handle value
			APP_ERROR_CHECK(err_code);
			SEGGER_RTT_printf(0, " Characteristic READ on CONNECT EVENT=  %x\n",tutorial_char_value);
		
			LEDS_OFF(LEDS_MASK);	// first turn off all 4 LEDs and then turn them on according to lowest 4 bits of characteristic value
			if(tutorial_char_value&TUT_LED_0_MASK){LEDS_ON(BSP_LED_0_MASK);}
			if(tutorial_char_value&TUT_LED_1_MASK){LEDS_ON(BSP_LED_1_MASK);}
			if(tutorial_char_value&TUT_LED_2_MASK){LEDS_ON(BSP_LED_2_MASK);}
			if(tutorial_char_value&TUT_LED_3_MASK){LEDS_ON(BSP_LED_3_MASK);}	
		  break;
		
    case BLE_GAP_EVT_DISCONNECTED:
        p_our_service->conn_handle = BLE_CONN_HANDLE_INVALID;
				SEGGER_RTT_WriteString(0, "nRFCloud Demo disconnected.\n");
        break;
		
		case BLE_EVT_TX_COMPLETE:
         SEGGER_RTT_WriteString(0, "Notification TX complete for Demo.\n");  
         break;
			
		case BLE_GATTS_EVT_WRITE:
         SEGGER_RTT_WriteString(0, "Demo handler for write event.\n"); 

	// Read nRFCloud demo characteristic value
//			ble_gatts_value_t      temp;								// declare and reserve memory for charcateristic structure
			memset(&temp, 0, sizeof(ble_gatts_value_t));
//			uint8_t tutorial_char_value;								// define a variable to hold char value when read
			temp.p_value   = &tutorial_char_value;			// add pointer to characteristic structure
			temp.len      = 4;
			temp.offset   = 0;					
							
			err_code = sd_ble_gatts_value_get(p_our_service->conn_handle, p_our_service->char_handles.value_handle, &temp); // DONE: remove hard coded handle value
			APP_ERROR_CHECK(err_code);
			SEGGER_RTT_printf(0, " Characteristic READ after WRITE EVENT=  %x\n",tutorial_char_value);
		
		LEDS_OFF(LEDS_MASK);	// first turn off all 4 LEDs and then turn them on according to lowest 4 bits of characteristic value
		if(tutorial_char_value&TUT_LED_0_MASK){LEDS_ON(BSP_LED_0_MASK);}
		if(tutorial_char_value&TUT_LED_1_MASK){LEDS_ON(BSP_LED_1_MASK);}
		if(tutorial_char_value&TUT_LED_2_MASK){LEDS_ON(BSP_LED_2_MASK);}
		if(tutorial_char_value&TUT_LED_3_MASK){LEDS_ON(BSP_LED_3_MASK);}

		
         break;


    default:
        // No implementation needed.
        break;
}

}

/**@brief Function for adding our new characterstic 
 *
 * @param[in]   p_our_service        Our Service structure.
 *
 */
static uint32_t our_char_add(ble_os_t * p_our_service)
{
    // Add a custom 128 bit characteristic UUID for demo
		uint32_t            err_code; 
		ble_uuid_t          char_uuid;
		ble_uuid128_t       base_uuid = BLE_UUID_OUR_BASE_UUID;
		char_uuid.uuid      = BLE_UUID_TUTORIAL_CHARACTERISTC_UUID;
		err_code = sd_ble_uuid_vs_add(&base_uuid, &char_uuid.type);
		APP_ERROR_CHECK(err_code);
	    
    // Add read/write properties to our characteristic
    ble_gatts_char_md_t char_md;
    memset(&char_md, 0, sizeof(char_md));
		char_md.char_props.read = 1; // added
		char_md.char_props.write = 1; //added
    
    // Configuring Client Characteristic Configuration Descriptor metadata and add to char_md structure
    ble_gatts_attr_md_t cccd_md;
    memset(&cccd_md, 0, sizeof(cccd_md));
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm); // added
		cccd_md.vloc                = BLE_GATTS_VLOC_STACK;    
		char_md.p_cccd_md           = &cccd_md;
		char_md.char_props.notify   = 1;

    
    // Configure the attributes of the characteristic
    ble_gatts_attr_md_t attr_md;
    memset(&attr_md, 0, sizeof(attr_md)); 
    attr_md.vloc        = BLE_GATTS_VLOC_STACK; // added
    
    // Set read/write security levels to our characteristic
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    
    // Configure the characteristic value attribute
    ble_gatts_attr_t    attr_char_value;
    memset(&attr_char_value, 0, sizeof(attr_char_value));
   
		attr_char_value.p_uuid      = &char_uuid; // added
		attr_char_value.p_attr_md   = &attr_md; // added

    
    // Set characteristic length in number of bytes
		attr_char_value.max_len     = 2;
		attr_char_value.init_len    = 2;
		uint8_t value[4]            = {0x00,0x00};
		attr_char_value.p_value     = value;


    // Add our new characteristic to the service
		// added lines
		err_code = sd_ble_gatts_characteristic_add(p_our_service->service_handle,
                                   &char_md,
                                   &attr_char_value,
                                   &p_our_service->char_handles);

			APP_ERROR_CHECK(err_code);



    return NRF_SUCCESS;
}

/**@brief Function for initiating our new service.
 *
 * @param[in]   p_our_service        Our Service structure.
 *
 */
void tut_service_init(ble_os_t * p_our_service)
{
    // Declare 16 bit service and 128 bit base UUIDs and add them to BLE stack table 

	uint32_t   err_code;			
	ble_uuid_t        service_uuid;
	ble_uuid128_t     base_uuid = BLE_UUID_OUR_BASE_UUID;  // 128 bit base with 0000s for nRFCloud Demo UUID
	service_uuid.uuid = BLE_UUID_TUTORIAL;									// the 1111s that are added to the base UUID
	err_code = sd_ble_uuid_vs_add(&base_uuid, &service_uuid.type);	// Adding serice UUID (128bit) to BLE stack
	
	APP_ERROR_CHECK(err_code);
	
	
	    // Set service connection handle to default value. I.e. an invalid handle since we are not yet in a connection.
	p_our_service->conn_handle = BLE_CONN_HANDLE_INVALID;
		  
    //  Add nRFCloud Demo service to BLE stack
	
	err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                    &service_uuid,
                                    &p_our_service->service_handle);
	APP_ERROR_CHECK(err_code);
	
    // Call the function our_char_add() to add our new characteristic to the service. 
    our_char_add(p_our_service);
		

		SEGGER_RTT_WriteString(0, "Exectuing our_service_init().\n"); // Print message to RTT to the application flow
    SEGGER_RTT_printf(0, "Service UUID: 0x%#04x\n", service_uuid.uuid); // Print service UUID should match definition BLE_UUID_OUR_SERVICE
    SEGGER_RTT_printf(0, "Service UUID type: 0x%#02x\n", service_uuid.type); // Print UUID type. Should match BLE_UUID_TYPE_VENDOR_BEGIN. Search for BLE_UUID_TYPES in ble_types.h for more info
    SEGGER_RTT_printf(0, "Service handle: 0x%#04x\n", p_our_service->service_handle); // Print out the service handle. Should match service handle shown in MCP under Attribute values
}




