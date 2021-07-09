#include "bmd101.h"

#include "nrf_uart.h"
#include "app_uart.h"

#define STEP_1 1
#define STEP_2 2
#define STEP_3 3
#define STEP_4 4
#define STEP_5 5

int16_t ecg;
uint8_t poor_signal, heart_rate;

int Parse_Payload( unsigned char *payload, unsigned char pLength ) 
{
    
    ret_code_t err_code;
    unsigned char bytesParsed = 0;
    unsigned char code;
    unsigned char length;
    unsigned char extendedCodeLevel;

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
                poor_signal = payload[bytesParsed];
                //NRF_LOG_INFO("POOR_SIGNAL Quality:%03d",poor_signal);
                break;
            
            case 0x03:
                heart_rate = payload[bytesParsed];
                //NRF_LOG_INFO("HEART_RATE Value:%03d",heart_rate);
                break;
                
            case 0x80:
                ecg = payload[bytesParsed] << 8 | payload[bytesParsed + 1];
                //NRF_LOG_INFO("ECG Value:%04d",ecg);
                break;
            default:
                break;
            
        }
            
        bytesParsed += length;
    }
    return 0;
}

void parseECG(uint8_t recv_char)
{
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
	uart_init();
    
	pinMode(BMD101_CS, OUTPUT);
    digitalWrite(BMD101_CS, 0);
    
    
    
	digitalWrite(BMD101_CS, 1);
		
}

void bmd101_sleep(void)
{
	
	digitalWrite(BMD101_CS, 0);
		
}

int16_t bmd101_getECG(void)
{

    return ecg;
    
}    

uint8_t bmd101_getHeartRate(void)
{

    return heart_rate;
    
}    

uint8_t bmd101_getSignalRate(void)
{

    return poor_signal;
    
}    
