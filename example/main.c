/**
 * Copyright (c) 2014 - 2020, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 * @defgroup uart_example_main main.c
 * @{
 * @ingroup uart_example
 * @brief UART Example Application main file.
 *
 * This file contains the source code for a sample application using UART.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_uart.h"
#include "app_uart.h"

#include "transfer_handler.h"
////#include "app_uart.h"
////#include "app_error.h"
////#include "nrf_delay.h"
////#include "nrf.h"
////#include "bsp.h"
////#if defined (UART_PRESENT)
////#include "nrf_uart.h"
////#endif
////#if defined (UARTE_PRESENT)
////#include "nrf_uarte.h"
////#endif


////#define ENABLE_LOOPBACK_TEST  /**< if defined, then this example will be a loopback test, which means that TX should be connected to RX to get data loopback. */

//#define MAX_TEST_DATA_BYTES     (15U)                /**< max number of test bytes to be used for tx and rx. */
//#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
//#define UART_RX_BUF_SIZE 256                         /**< UART RX buffer size. */

//void uart_error_handle(app_uart_evt_t * p_event)
//{
//    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
//    {
//        APP_ERROR_HANDLER(p_event->data.error_communication);
//    }
//    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
//    {
//        APP_ERROR_HANDLER(p_event->data.error_code);
//    }
//}


//#ifdef ENABLE_LOOPBACK_TEST
///* Use flow control in loopback test. */
//#define UART_HWFC APP_UART_FLOW_CONTROL_ENABLED

///** @brief Function for setting the @ref ERROR_PIN high, and then enter an infinite loop.
// */
//static void show_error(void)
//{

//    bsp_board_leds_on();
//    while (true)
//    {
//        // Do nothing.
//    }
//}


///** @brief Function for testing UART loop back.
// *  @details Transmitts one character at a time to check if the data received from the loopback is same as the transmitted data.
// *  @note  @ref TX_PIN_NUMBER must be connected to @ref RX_PIN_NUMBER)
// */
//static void uart_loopback_test()
//{
//    uint8_t * tx_data = (uint8_t *)("\r\nLOOPBACK_TEST\r\n");
//    uint8_t   rx_data;

//    // Start sending one byte and see if you get the same
//    for (uint32_t i = 0; i < MAX_TEST_DATA_BYTES; i++)
//    {
//        uint32_t err_code;
//        while (app_uart_put(tx_data[i]) != NRF_SUCCESS);

//        nrf_delay_ms(10);
//        err_code = app_uart_get(&rx_data);

//        if ((rx_data != tx_data[i]) || (err_code != NRF_SUCCESS))
//        {
//            show_error();
//        }
//    }
//    return;
//}
//#else
///* When UART is used for communication with the host do not use flow control.*/
//#define UART_HWFC APP_UART_FLOW_CONTROL_DISABLED
//#endif

static uint8_t     rx_buf[128];                                                  
static uint8_t     tx_buf[128];  

#define STEP_1 1
#define STEP_2 2
#define STEP_3 3
#define STEP_4 4
#define STEP_5 5


int Parse_Payload( unsigned char *payload, unsigned char pLength ) {
    unsigned char bytesParsed = 0;
    unsigned char code;
    unsigned char length;
    unsigned char extendedCodeLevel;
		ret_code_t err_code;
		
		int16_t ECGTemp;
		static uint8_t poor_signal, heart_rate;
		
		
	
    /* Loop until all bytes are parsed from the payload[] array... */
    bool sensor_contact_detected = false;
		while( bytesParsed < pLength ) {
        /* Parse the extendedCodeLevel, code, and length */
        extendedCodeLevel = 0;
        while( payload[bytesParsed] == 0x55 ) {
            extendedCodeLevel++;
            bytesParsed++;
        }
        code = payload[bytesParsed++];
        if( code & 0x80 ) length = payload[bytesParsed++];
        else length = 1;
				
				//NRF_LOG_INFO("CODE:0x%02X,Payload:0x%02X,0x%02X",code,payload[bytesParsed],payload[bytesParsed + 1]);
				
				switch(code){
					case 0x02:
						poor_signal = payload[bytesParsed + 1];
						NRF_LOG_INFO("POOR_SIGNAL Quality:%03d",poor_signal);
#ifdef HRS_CONTACT					
            if(poor_signal > 100)
							sensor_contact_detected = true;
						else
							sensor_contact_detected = false;

					if(is_connected)
						ble_hrs_sensor_contact_detected_update(&m_hrs, sensor_contact_detected);
#endif
					break;
					case 0x03:
						heart_rate = payload[bytesParsed + 1];
						NRF_LOG_INFO("HEART_RATE Value:%03d",heart_rate);
#ifdef HRS_CONTACT
						if(is_connected){
							err_code = ble_hrs_heart_rate_measurement_send(&m_hrs, heart_rate);
								if ((err_code != NRF_SUCCESS) &&
										(err_code != NRF_ERROR_INVALID_STATE) &&
										(err_code != NRF_ERROR_RESOURCES) &&
										(err_code != NRF_ERROR_BUSY) &&
										(err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
									 )
								{
										APP_ERROR_HANDLER(err_code);
								}
						}
#endif
					break;
					
					case 0x80:
						ECGTemp = payload[bytesParsed] << 8 | payload[bytesParsed + 1];
						NRF_LOG_INFO("ECG Value:%04d",ECGTemp);
						//nrf_queue_push(&m_ecg_queue,&ECGTemp);
					break;
					default:
						break;
				
				}
				
        bytesParsed += length;
    }
    return( 0 );
}

void parseECG(uint8_t recv_char){
	

		static uint8_t STEP = STEP_1;
		static uint8_t offset_t = 0;
		static uint8_t plength = 0;
		static uint8_t recv_payload[170];
		int checksum = 0;
		int i;
    switch(STEP){

        case STEP_1: //SYNC-1
						//NRF_LOG_INFO("SYNC-1,%x",recv_char);
            if(recv_char == 0xaa)
                STEP = STEP_2;
            return;
        case STEP_2: //SYNC-2
						//NRF_LOG_INFO("SYNC-2,%x",recv_char);
            if(recv_char == 0xaa){
                STEP = STEP_3;
            } else {
                STEP = STEP_1;
							NRF_LOG_INFO("BMD101 Parse Error");
            }
            return;
        case STEP_3: //PLENGTH
					//NRF_LOG_INFO("PLENGTH,%x",recv_char);
            if(recv_char == 0xaa){
                STEP = STEP_3;
            } else if (recv_char > 170){
                STEP = STEP_1;
							NRF_LOG_INFO("BMD101 Parse Error(Too many chars)");
            } else {
                plength = recv_char;
                memset(recv_payload,0,sizeof(recv_payload));
                offset_t = 0;
                STEP = STEP_4;
            }
            return;
        case STEP_4: //Recv PLENGTH
					//NRF_LOG_INFO("Recv PLENGTH,%x",recv_char);
						recv_payload[offset_t] = recv_char;
						offset_t++;
            if( offset_t == plength ){
                STEP = STEP_5;
            }
            return;
        case STEP_5: //Checksum
					//NRF_LOG_INFO("Checksum,%x",recv_char);
						checksum = 0;
            for(i=0;i<plength;i++){
                checksum += recv_payload[i];
            }
            checksum &= 0xff;
            checksum = ~checksum & 0xff;
            if(checksum == recv_char){
              Parse_Payload(recv_payload,plength);  
							STEP = STEP_1;
            }else{
                STEP = STEP_1;
							NRF_LOG_INFO("BMD101 Parse Error(Checksum)");
            }
            return;
    }
}

void uart_event_handle(app_uart_evt_t *p_event)
{
  uint32_t err_code;
	uint8_t receive_char;

  switch (p_event->evt_type)
  {
		case APP_UART_DATA_READY:
			UNUSED_VARIABLE(app_uart_get(&receive_char));
			parseECG(receive_char);
			break;

		default:
			NRF_LOG_INFO("BMD101 Error %d", p_event->data.error_communication );
			break;
  }
}

void uart_on(void)
{
	ret_code_t err_code;
	
	app_uart_comm_params_t const comm_115200_params =
	{
			.rx_pin_no    = BMD101_TX,
			.tx_pin_no    = UART_PIN_DISCONNECTED,
			.rts_pin_no   = UART_PIN_DISCONNECTED,
			.cts_pin_no   = UART_PIN_DISCONNECTED,
			.flow_control = APP_UART_FLOW_CONTROL_DISABLED,
			.use_parity   = false,
			.baud_rate    = NRF_UART_BAUDRATE_115200
	};
	
	app_uart_buffers_t buffers = {
	
			.rx_buf      = rx_buf,                                                            
			.rx_buf_size = sizeof (rx_buf),                                                
			.tx_buf      = tx_buf,                 
			.tx_buf_size = sizeof (tx_buf)
	
	};
	
	err_code = app_uart_init(&comm_115200_params, &buffers, uart_event_handle, APP_IRQ_PRIORITY_LOWEST);
	
	APP_ERROR_CHECK(err_code);
	
	pinMode(BMD101_EN, OUTPUT);
	digitalWrite(BMD101_EN, 1);
		
}


/**
 * @brief Function for main application entry.
 */
int main(void)
{
    uint32_t err_code;
	
		err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
	
	NRF_LOG_INFO("BMD101 Parse Error(Checksum)");
	
		uart_on();
	
		for (;;)
    {
				if (NRF_LOG_PROCESS() == false)
				{
						__WFE();
				}
    }
	
		

//    bsp_board_init(BSP_INIT_LEDS);

//    const app_uart_comm_params_t comm_params =
//      {
//          RX_PIN_NUMBER,
//          TX_PIN_NUMBER,
//          RTS_PIN_NUMBER,
//          CTS_PIN_NUMBER,
//          UART_HWFC,
//          false,
//#if defined (UART_PRESENT)
//          NRF_UART_BAUDRATE_115200
//#else
//          NRF_UARTE_BAUDRATE_115200
//#endif
//      };

//    APP_UART_FIFO_INIT(&comm_params,
//                         UART_RX_BUF_SIZE,
//                         UART_TX_BUF_SIZE,
//                         uart_error_handle,
//                         APP_IRQ_PRIORITY_LOWEST,
//                         err_code);

//    APP_ERROR_CHECK(err_code);

//#ifndef ENABLE_LOOPBACK_TEST
//    printf("\r\nUART example started.\r\n");

//    while (true)
//    {
//        uint8_t cr;
//        while (app_uart_get(&cr) != NRF_SUCCESS);
//        while (app_uart_put(cr) != NRF_SUCCESS);

//        if (cr == 'q' || cr == 'Q')
//        {
//            printf(" \r\nExit!\r\n");

//            while (true)
//            {
//                // Do nothing.
//            }
//        }
//    }
//#else

//    // This part of the example is just for testing the loopback .
//    while (true)
//    {
//        uart_loopback_test();
//    }
//#endif
}


/** @} */
