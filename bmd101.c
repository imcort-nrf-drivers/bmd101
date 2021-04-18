#include "bmd101.h"

#include "nrf_uart.h"
#include "app_uart.h"

#include "transfer_handler.h"

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

void bmd101_begin(void)
{
	ret_code_t err_code;
	
	app_uart_comm_params_t const bmd101_comm_params =
	{
			.rx_pin_no    = BMD101_TX,
			.tx_pin_no    = UART_PIN_DISCONNECTED,
			.rts_pin_no   = UART_PIN_DISCONNECTED,
			.cts_pin_no   = UART_PIN_DISCONNECTED,
			.flow_control = APP_UART_FLOW_CONTROL_DISABLED,
			.use_parity   = false,
			.baud_rate    = NRF_UART_BAUDRATE_57600
	};
	
	app_uart_buffers_t buffers = {
	
			.rx_buf      = rx_buf,                                                            
			.rx_buf_size = sizeof (rx_buf),                                                
			.tx_buf      = tx_buf,                 
			.tx_buf_size = sizeof (tx_buf)
	
	};
	
	err_code = app_uart_init(&bmd101_comm_params, &buffers, uart_event_handle, APP_IRQ_PRIORITY_LOWEST);
	
	APP_ERROR_CHECK(err_code);
	
	pinMode(BMD101_EN, OUTPUT);
	digitalWrite(BMD101_EN, 1);
		
}
