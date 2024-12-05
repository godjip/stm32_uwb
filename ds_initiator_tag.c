/*
 * ss_tag.c
 *
 *  Created on: Jan 18, 2021
 *      Author: kostasdeligiorgis
 */

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <DWM_functions.h>
#include "main.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "stm32f1xx_hal.h"
#include "mlx90614.h"
#include "DFPLAYER_MINI.h"
/* Default communication configuration. We use here EVK1000's mode 4. See NOTE 1 below. */
static dwt_config_t config = {
    3,               /* Channel number. */
    DWT_PRF_64M,     /* Pulse repetition frequency. */
    DWT_PLEN_128,    /* Preamble length. Used in TX only. */
    DWT_PAC8,        /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    0,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_6M8,      /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (129 + 8 - 8)    /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

/* Default antenna delay values for 64 MHz PRF. See NOTE 2 below. */
#define TX_ANT_DLY 16505
#define RX_ANT_DLY 16505
#define tag 1	//tag 1, 2,3,4 ...
#define correction 0.1 //temperature correction value
#define NUM_READINGS 10
float temperature_readings[NUM_READINGS];
uint8_t current_index = 0;

/* Frames used in the ranging process. See NOTE 3 below. */
static uint8 rx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0};
static uint8 tx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, tag, 0, 0};

/* Length of the common part of the message (up to and including the function code, see NOTE 3 below). */
#define ALL_MSG_COMMON_LEN 10
/* Index to access some of the fields in the frames involved in the process. */
#define ALL_MSG_SN_IDX 2
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define RESP_MSG_TS_LEN 4
/* Frame sequence number, incremented after each transmission. */
static uint8 frame_seq_nb = 0;

/* Buffer to store received messages.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 100
static uint8 rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
uint32 status_reg = 0;

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
 * 1 uus = 512 / 499.2 Βs and 1 Βs = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536

/* Delay between frames, in UWB microseconds. See NOTE 1 below. */
//#define POLL_RX_TO_RESP_TX_DLY_UUS 330
//#define POLL_RX_TO_RESP_TX_DLY_UUS 1000 // it works
//#define POLL_RX_TO_RESP_TX_DLY_UUS 850  // it works
#define POLL_RX_TO_RESP_TX_DLY_UUS 1300

/* Timestamps of frames transmission/reception.
 * As they are 40-bit wide, we need to define a 64-bit int type to handle them. */
typedef unsigned long long uint64;
static uint64 poll_rx_ts;
static uint64 resp_tx_ts;

/* Declaration of static functions. */
static uint64 get_rx_timestamp_u64(void);
static void resp_msg_set_ts(uint8 *ts_field, const uint64 ts);

extern float temp;
extern uint8_t wear_flag;
extern uint8_t sos_flag;
extern uint8_t server_shock_flag;

float temperature_readings[10] = {26.5,26.5,26.5,26.5,26.532,26.5,26.5,26.5,26.5,26.5};
uint8 temp_flag = 0;
uint8 wear_flag = 0;
uint8 previous_wear_flag = 0;
float avr_temp = 26.5;
uint8 before_temp_flag = 0;
void ss_resp_main(void)
{
    /* Reset and initialise DW1000.
     * For initialisation, DW1000 clocks must be temporarily set to crystal speed. After initialisation SPI rate can be increased for optimum
     * performance. */
    deca_reset(); /* Target specific drive of RSTn line into DW1000 low for a period. */

    port_set_dw1000_slowrate();

    if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)
    {
        while (1)
        { };
    }

    port_set_dw1000_fastrate();

    /* Configure DW1000. See NOTE 5 below. */
    dwt_configure(&config);

    /* Apply default antenna delay value. See NOTE 2 below. */
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);

    /****Debug Counters****/
//    int k1 = 0;   // start_tx_delayed failed
//    int k2 = 0;   // start_tx_delayed successed
    /**********************/

}
int count123;
/* Loop forever responding to ranging requests. */
void ss_resp_start(void)
{
	/* Activate reception immediately. */

	status_reg = dwt_read32bitreg(SYS_STATUS_ID);

	/* Poll for reception of a frame or error/timeout. See NOTE 6 below. */
	dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
	if (status_reg & SYS_STATUS_RXFCG)
	{
		uint32 frame_len;

		/* Clear good RX frame event in the DW1000 status register. */
	   // dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

		/* A frame has been received, read it into the local buffer. */
		frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
		if (frame_len <= RX_BUFFER_LEN)
		{
			dwt_readrxdata(rx_buffer, frame_len, 0);
		}

		/* Check that the frame is a poll sent by "SS TWR initiator" example.
		 * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
		rx_buffer[ALL_MSG_SN_IDX] = 0;
		if (memcmp(rx_buffer, rx_poll_msg, ALL_MSG_COMMON_LEN) == 0)
		{
			uint32 resp_tx_time;
			int ret;

			/* Retrieve poll reception timestamp. */
			poll_rx_ts = get_rx_timestamp_u64();

			/* Compute final message transmission time. See NOTE 7 below. */
			resp_tx_time = (poll_rx_ts + ((POLL_RX_TO_RESP_TX_DLY_UUS + (tag-1)*5000 ) * UUS_TO_DWT_TIME)) >> 8;

			dwt_setdelayedtrxtime(resp_tx_time);

			/* Response TX timestamp is the transmission time we programmed plus the antenna delay. */
			resp_tx_ts = (((uint64)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

			/* Write all timestamps in the final message. See NOTE 8 below. */
			resp_msg_set_ts(&tx_resp_msg[RESP_MSG_POLL_RX_TS_IDX], poll_rx_ts);
			resp_msg_set_ts(&tx_resp_msg[RESP_MSG_RESP_TX_TS_IDX], resp_tx_ts);

			//temp
			memcpy(&tx_resp_msg[18], &avr_temp, sizeof(float));	//float 4byte [18 19 20 21]
			//wear detect flag
			tx_resp_msg[23] = wear_flag;
			//sos_flag
			tx_resp_msg[24] = sos_flag;
			//shock detect flag
			tx_resp_msg[25] = server_shock_flag;

			/* Write and send the response message. See NOTE 9 below. */
			tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb;	//int type 1byte [ALL_MSG_SN_IDX]
			dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0); /* Zero offset in TX buffer. */
			dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1); /* Zero offset in TX buffer, ranging. */
			ret = dwt_starttx(DWT_START_TX_DELAYED);


			if (ret == DWT_ERROR)
			{

			}

			/* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. See NOTE 10 below. */
			if (ret == DWT_SUCCESS)
			{
				count123++;
				/* Poll DW1000 until TX frame sent event set. See NOTE 6 below. */
				while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
				{ };

				/* Clear TXFRS event. */
				dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

				/* Increment frame sequence number after transmission of the poll message (modulo 256). */
				frame_seq_nb++;
			}

			if (current_index >= NUM_READINGS)
			    {
			        current_index = 0;
			    }

			temp = MLX90614_ReadTemp(MLX90614_DEFAULT_SA, MLX90614_TOBJ1) + 5;  // temp read
			temperature_readings[current_index] = temp;  // save list
			current_index++;  // move to next index

			float sum = 0;
		    for (uint8_t i = 0; i < NUM_READINGS; i++)
		    {
		        sum += temperature_readings[i];
		    }
		    avr_temp = (sum / NUM_READINGS) + correction;  //calculate average +n

		    // High fever detection flag server no
		    if (avr_temp > 38.5) {
				temp_flag = 1;
				if(before_temp_flag == 0){
					DF_Play(0x06); //High temperature detected
				}
			} else if (avr_temp < 37) {
				temp_flag = 0;
			}

		    before_temp_flag=temp_flag;
		    // Wearing detection flag
		    if (avr_temp > 33) {
		    	wear_flag = 1;
		    } else if (avr_temp < 30){
		    	wear_flag = 0;
		    }

		    //wear sound
		    if (previous_wear_flag == 0 && wear_flag == 1) {
		    	DF_Play(0x05); //wear detected sound play
			}
//		    else if (previous_wear_flag == 1 && wear_flag == 0) {
//		    	DF_Play(0x02); //unwear detected sound play
//		    }

			// update value
			previous_wear_flag = wear_flag;

		}
	}
	dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_rx_timestamp_u64()
 *
 * @brief Get the RX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
static uint64 get_rx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readrxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn final_msg_set_ts()
 *
 * @brief Fill a given timestamp field in the response message with the given value. In the timestamp fields of the
 *        response message, the least significant byte is at the lower address.
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to fill
 *         ts  timestamp value
 *
 * @return none
 */
static void resp_msg_set_ts(uint8 *ts_field, const uint64 ts)
{
    int i;
    for (i = 0; i < RESP_MSG_TS_LEN; i++)
    {
        ts_field[i] = (ts >> (i * 8)) & 0xFF;
    }
}
