/*
 * ss_anchor.c
 *
 *  Created on: Jan 18, 2021
 *      Author: kostasdeligiorgis
 */

#include <stdio.h>
#include <string.h>

#include "deca_device_api.h"
#include "deca_regs.h"
#include "stdio.h"
#include <DWM_functions.h>
#include "main.h"

//#define RNG_DELAY_MS 1000
#define RNG_DELAY_MS 850

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
    (128 +1 + 8 - 8)    /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

/* Default antenna delay values for 64 MHz PRF. See NOTE 2 below. */
#define TX_ANT_DLY 16505
#define RX_ANT_DLY 16505

/* Frames used in the ranging process. See NOTE 3 below. */
static uint8 tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0};
static uint8 rx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

/* Length of the common part of the message (up to and including the function code, see NOTE 3 below). */
#define ALL_MSG_COMMON_LEN 10
/* Indexes to access some of the fields in the frames defined above. */
#define ALL_MSG_SN_IDX 2
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define RESP_MSG_TS_LEN 4
#define RESP_MSG_TEMP_IDX 18 // 온도 데이터가 있는 위치
/* Frame sequence number, incremented after each transmission. */
static uint8 frame_seq_nb = 0;

/* Buffer to store received response message.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 100
uint8 rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32 status_reg = 0;

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
 * 1 uus = 512 / 499.2 Βs and 1 Βs = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536

/* Delay between frames, in UWB microseconds. See NOTE 1 below. */
#define POLL_TX_TO_RESP_RX_DLY_UUS 0

/* Receive response timeout. See NOTE 5 below. */
//#define RESP_RX_TIMEOUT_UUS 210
#define RESP_RX_TIMEOUT_UUS 1000	  // it works good 08/03

/* Speed of light in air, in metres per second. */
#define SPEED_OF_LIGHT 299702547

/* Hold copies of computed time of flight and distance here for reference so that it can be examined at a debug breakpoint. */
static double tof;
float distance ,dist1 ,dist2;
float temp,temp1,temp2;

uint32_t start_time,time1;
uint8_t dist[30];
uint8_t wear_flag, wear_flag1, wear_flag2;
uint8_t sos_flag, sos_flag1, sos_flag2;
uint8_t server_shock_flag, server_shock_flag1, server_shock_flag2;
/* Declaration of static functions. */
static void resp_msg_get_ts(uint8 *ts_field, uint32 *ts);

void ss_init_main(void)
{
    /* Reset and initialise DW1000.
     * For initialisation, DW1000 clocks must be temporarily set to crystal speed. After initialisation SPI rate can be increased for optimum
     * performance. */
	deca_reset();/* Target specific drive of RSTn line into DW1000 low for a period. */

	port_set_dw1000_slowrate();

    if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)
    {
        while (1)
        { };
    }

    port_set_dw1000_fastrate();

    /* Configure DW1000. See NOTE 6 below. */
    dwt_configure(&config);

    /* Apply default antenna delay value. See NOTE 2 below. */
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);

    /* Set expected response's delay and timeout. See NOTE 1 and 5 below.
     * As this example only handles one incoming frame with always the same delay and timeout, those values can be set here once for all. */
    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(100000);

    /****Debug Counters****/
//    int k1 = 0;   // received a frame
//    int k2 = 0;   // the received frame is correct
//    int k3 = 0;   // receive fails
    /**********************/
}
    /* Loop forever initiating ranging exchanges. */
void ss_init_start()
    {
        /* Write frame data to DW1000 and prepare transmission. See NOTE 7 below. */
        tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
        dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0); /* Zero offset in TX buffer. */
        dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1); /* Zero offset in TX buffer, ranging. */


        /* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
         * set by dwt_setrxaftertxdelay() has elapsed. */
        dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
        start_time = HAL_GetTick();
        /* We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout. See NOTE 8 below. */
        while((HAL_GetTick() - start_time) < 100){

			while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
			{
				if((HAL_GetTick() - start_time) > 100){break; }
			};
			if((HAL_GetTick() - start_time) > 100){break;}
			/* Increment frame sequence number after transmission of the poll message (modulo 256). */
			frame_seq_nb++;

			if (status_reg & SYS_STATUS_RXFCG)
			{
	//        	k1++;
				uint32 frame_len;

				/* Clear good RX frame event in the DW1000 status register. */
				dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

				/* A frame has been received, read it into the local buffer. */
				frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
				if (frame_len <= RX_BUF_LEN)
				{
					dwt_readrxdata(rx_buffer, frame_len, 0);
				}

				/* Check that the frame is the expected response from the companion "SS TWR responder" example.
				 * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
				rx_buffer[ALL_MSG_SN_IDX] = 0;
				if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0)
				{
	//            	k2++;

					uint32 poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
					int32 rtd_init, rtd_resp;
					float clockOffsetRatio ;

					/* Retrieve poll transmission and response reception timestamps. See NOTE 9 below. */
					poll_tx_ts = dwt_readtxtimestamplo32();
					resp_rx_ts = dwt_readrxtimestamplo32();

					/* Read carrier integrator value and calculate clock offset ratio. See NOTE 11 below. */
					clockOffsetRatio = dwt_readcarrierintegrator() * (FREQ_OFFSET_MULTIPLIER * HERTZ_TO_PPM_MULTIPLIER_CHAN_2 / 1.0e6) ;

					/* Get timestamps embedded in response message. */
					resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts);
					resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts);

					/* Compute time of flight and distance, using clock offset ratio to correct for differing local and remote clock rates */
					rtd_init = resp_rx_ts - poll_tx_ts;
					rtd_resp = resp_tx_ts - poll_rx_ts;

					tof = ((rtd_init - rtd_resp * (1 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
					distance = tof * SPEED_OF_LIGHT;

					// Extract temperature data from the response message

					memcpy(&temp, &rx_buffer[RESP_MSG_TEMP_IDX], sizeof(float));

					wear_flag = rx_buffer[23];
					sos_flag = rx_buffer[24];
					server_shock_flag = rx_buffer[25];

					if(rx_buffer[26]==1){
						dist1=distance;
						temp1=temp;
						wear_flag1 = wear_flag;
						sos_flag1 = sos_flag;
						server_shock_flag1 = server_shock_flag;
					} else if(rx_buffer[26]==2){
						dist2=distance;
						temp2=temp;
						wear_flag2 = wear_flag;
						sos_flag2 = sos_flag;
						server_shock_flag2 = server_shock_flag;
					}

				}
				// + all buffer getthering
				memset(rx_buffer, 0, RX_BUF_LEN);
				dwt_rxenable(DWT_START_RX_IMMEDIATE);
			}
			else
			{
				/* Clear RX error/timeout events in the DW1000 status register. */
				dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

				/* Reset RX to properly reinitialise LDE operation. */
				dwt_rxreset();
			}

        }
        /* Execute a delay between ranging exchanges. */
        //server?
        //memset(all_buffer, 0, ALL_BUF_LEN);
    }

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn resp_msg_get_ts()
 *
 * @brief Read a given timestamp value from the response message. In the timestamp fields of the response message, the
 *        least significant byte is at the lower address.
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to get
 *         ts  timestamp value
 *
 * @return none
 */
static void resp_msg_get_ts(uint8 *ts_field, uint32 *ts)
{
    int i;
    *ts = 0;
    for (i = 0; i < RESP_MSG_TS_LEN; i++)
    {
        *ts += ts_field[i] << (i * 8);
    }
}
