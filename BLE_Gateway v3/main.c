/**
 * Copyright (c) 2014 - 2019, Nordic Semiconductor ASA
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
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */


#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "app_timer.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp_btn_ble.h"
#include "nrf_pwr_mgmt.h"

#include "fds.h"
//#include "boards.h"
#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "cmd.h"

#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */

#define DEFAULT_DEVICE_NAME             "AA"                               /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */

#define APP_ADV_DURATION                BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED                                     /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(20000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

#define LINK_TOTAL                      NRF_SDH_BLE_PERIPHERAL_LINK_COUNT + \
                                        NRF_SDH_BLE_CENTRAL_LINK_COUNT

#define MAX_APP_TIMER             NRF_SDH_BLE_PERIPHERAL_LINK_COUNT
#define TIMEOUT_WAIT_PASS         30000        // ms

static app_timer_t m_timer_data[MAX_APP_TIMER] = { {0} };
static app_timer_id_t m_timer_id[MAX_APP_TIMER];
volatile  static bool flag_request_dis[MAX_APP_TIMER];


static uint8_t m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;                   /**< Advertising handle used to identify an advertising set. */
static uint8_t m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];                    /**< Buffer for storing an encoded advertising set. */
static uint8_t m_enc_scan_response_data[BLE_GAP_ADV_SET_DATA_SIZE_MAX];         /**< Buffer for storing an encoded scan data. */

/**@brief Struct that contains pointers to the encoded advertising data. */
static ble_gap_adv_data_t m_adv_data =
{
    .adv_data =
    {
        .p_data = m_enc_advdata,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
    },
    .scan_rsp_data =
    {
        .p_data = m_enc_scan_response_data,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX

    }
};

BLE_NUS_DEF(m_nus, NRF_SDH_BLE_PERIPHERAL_LINK_COUNT);                                   /**< BLE NUS service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
NRF_BLE_QWRS_DEF(m_qwr, NRF_SDH_BLE_PERIPHERAL_LINK_COUNT);                                                               /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */

static uint16_t   m_conn_handle          = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current connection. */
static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static ble_uuid_t m_adv_uuids[]          =                                          /**< Universally unique service identifier. */
{
    {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}
};

#define ADVERTISING_LED                 BSP_BOARD_LED_0                         /**< Is on when device is advertising. */
#define CONNECTED_LED                   BSP_BOARD_LED_1                         /**< Is on when device has connected. */
#define LEDBUTTON_LED                   BSP_BOARD_LED_2                         /**< LED to be toggled with the help of the LED Button Service. */
#define LEDBUTTON_BUTTON                BSP_BUTTON_0                            /**< Button that will trigger the notification event with the LED Button Service */

#define CONNECTED_LED1                   BSP_BOARD_LED_2
#define CONNECTED_LED2                   BSP_BOARD_LED_3
typedef struct __attribute__((packed))
{
  uint8_t   nick_name[LEN_ID_APP];
  uint8_t  dir;
  uint16_t  len_payload;
  uint8_t   payload[LEN_PAYLOAD];
} _packet_t;

typedef union __attribute__((packed))
{
    _packet_t packet_struct;
    uint8_t   packet[LEN_PACKET];
} uart_packet_t;

typedef enum
{
	RX_HEADER,
        RX_ID_APP,
        RX_DIRECTION,
	RX_LEN_PAYLOAD,
	RX_PAYLOAD
} rx_state_t;

typedef enum 
{
	ERROR,
	UNCOMPLETE,
	COMPLETE
} flag_rx_status;


static flag_rx_status 	flag_rx;
static rx_state_t	rx_state;
static uart_packet_t      rx_packet;


void restart_receive(void)
{
	flag_rx 		= UNCOMPLETE;
	rx_state 		= RX_HEADER;
}

flag_rx_status receive_packet(uint8_t data)
{
    static uint16_t 	i;

    if(flag_rx == COMPLETE)
    return COMPLETE;

    switch(rx_state)
    {
        case RX_HEADER:
            if( data == HEADER_PACKET)
            {
                rx_state                  = RX_ID_APP;	
                i                         = 0;
                NRF_LOG_INFO("HEADR = %2x", data);
            }
        break;

        case RX_ID_APP:
           rx_packet.packet_struct.nick_name[i++] = data;
           if( i == LEN_ID_APP )
           {
                rx_state                  = RX_DIRECTION;
                i                         = 0;
                rx_packet.packet_struct.dir = 0;
                NRF_LOG_INFO("ID = %2x", data);
           }
        break;

        case RX_DIRECTION:
            rx_packet.packet_struct.dir   = data;
            rx_state                      = RX_LEN_PAYLOAD;
            i                             = 0;
            rx_packet.packet_struct.len_payload = 0;
            NRF_LOG_INFO("Dir = %2x", rx_packet.packet_struct.dir);      
        break;

        case RX_LEN_PAYLOAD:
            rx_packet.packet_struct.len_payload = (rx_packet.packet_struct.len_payload << 8) | data;
            i++;
            if( i == LEN_LEN_PAYLOAD )
            {
                if( rx_packet.packet_struct.len_payload >  LEN_PAYLOAD )
                {
                      rx_packet.packet_struct.len_payload = LEN_PAYLOAD;
                }
                rx_state                = RX_PAYLOAD;  
                i                       = 0;		
                NRF_LOG_INFO("Len = %d", rx_packet.packet_struct.len_payload);
            }
        break;

        case RX_PAYLOAD:
            rx_packet.packet_struct.payload[i++] 	= data;

            if( i == rx_packet.packet_struct.len_payload)
            {
                    flag_rx         = COMPLETE;
                     NRF_LOG_INFO("RX Complete");
            }
        break;

        default:
        break;
    }

    return flag_rx;
}


#define   MAX_LEN_NAME      BLE_GAP_ADV_SET_DATA_SIZE_MAX
#define FILE_ID         0x0001  /* The ID of the file to write the records into. */
#define RECORD_NAME     0x1111  /* A key for the name record. */

static uint8_t     m_name[MAX_LEN_NAME];
static uint8_t     len_name = MAX_LEN_NAME;

// Simple event handler to handle errors during initialization.
static void fds_evt_handler(fds_evt_t const * p_fds_evt)
{
    switch (p_fds_evt->id)
    {
        case FDS_EVT_INIT:
            if (p_fds_evt->result != FDS_SUCCESS)
            {
                NRF_LOG_INFO("init fds: error");
            }
            break;
        
        case FDS_EVT_WRITE:
        {
            if (p_fds_evt->result == FDS_SUCCESS)
            {
                NRF_LOG_INFO("Record ID:\t0x%04x",  p_fds_evt->write.record_id);
                NRF_LOG_INFO("File ID:\t0x%04x",    p_fds_evt->write.file_id);
                NRF_LOG_INFO("Record key:\t0x%04x", p_fds_evt->write.record_key);
            }
        } break;

        default:
            break;
    }
}

void fds_init_data(void)
{
    ret_code_t ret = fds_register(fds_evt_handler);
    APP_ERROR_CHECK(ret);

    ret = fds_init();
    APP_ERROR_CHECK(ret);
}

void fds_wirte_data(void)
{
    fds_record_t record;
    fds_record_desc_t   record_desc;

    record.file_id           = FILE_ID;
    record.key               = RECORD_NAME;
    record.data.p_data       = &m_name;
    /* The length of a record is always expressed in 4-byte units (words). */
    record.data.length_words = (MAX_LEN_NAME + 3) / sizeof(uint32_t);

    ret_code_t rc;
    rc = fds_record_write(&record_desc, &record);
     NRF_LOG_INFO("fds_record_write: rc =%d", rc);
    APP_ERROR_CHECK(rc);
}

void record_update(void)
{
    fds_record_desc_t desc = {0};
    fds_find_token_t  ftok = {0};
    fds_record_t record;

    if (fds_record_find(FILE_ID, RECORD_NAME, &desc, &ftok) == FDS_SUCCESS)
    {
        fds_record_t   record;
        record.file_id           = FILE_ID;
        record.key               = RECORD_NAME;
        record.data.p_data       = &m_name;
        /* The length of a record is always expressed in 4-byte units (words). */
        record.data.length_words = (MAX_LEN_NAME + 3) / sizeof(uint32_t);

        ret_code_t rc = fds_record_update(&desc, &record);
        NRF_LOG_INFO("fds_record_update: rc =%d", rc);
        APP_ERROR_CHECK(rc);
    }
    else
    {
        NRF_LOG_INFO("error: could not find config file.\n");
    }
}

void fds_read_data(void)
{
    fds_flash_record_t  flash_record;
    fds_record_desc_t   record_desc;
    fds_find_token_t    ftok;
    uint8_t i;
    uint8_t *ptr;

    /* It is required to zero the token before first use. */
    memset(&ftok, 0x00, sizeof(fds_find_token_t));
    /* Loop until all records with the given key and file ID have been found. */
    if (fds_record_find(FILE_ID, RECORD_NAME, &record_desc, &ftok) == FDS_SUCCESS)
    {
        if (fds_record_open(&record_desc, &flash_record) != FDS_SUCCESS)
        {
            /* Handle error. */
            NRF_LOG_INFO("fds_record_open: error");
        }
        
        ptr = (uint8_t*)(flash_record.p_data);
        for(i = MAX_LEN_NAME; i > 0; i-- )
        {
            if( ptr[i-1] != 0 )
              break;
        }
        
        if( ptr[0] == 0 )
        {
            NRF_LOG_INFO("use default name");
            len_name = strlen(DEFAULT_DEVICE_NAME);
            memcpy(&m_name, DEFAULT_DEVICE_NAME, len_name);
        }
        else
        {
            len_name = i;
            memcpy(&m_name, flash_record.p_data, MAX_LEN_NAME);
            NRF_LOG_INFO("use flash name");
        }
        /* Access the record through the flash_record structure. */
        /* Close the record when done. */
        if (fds_record_close(&record_desc) != FDS_SUCCESS)
        {
            /* Handle error. */
             NRF_LOG_INFO("fds_record_close: error");
        }
    }
    else
    {
        /* System config not found; write a new one. */
        NRF_LOG_INFO("Writing config file...");
        memset(m_name, 0, MAX_LEN_NAME);
        fds_wirte_data();

        NRF_LOG_INFO("use default name");
        len_name = strlen(DEFAULT_DEVICE_NAME);
        memcpy(&m_name, DEFAULT_DEVICE_NAME, len_name);
    }
}

#define ADDR_DEVICE_INVAILD 0xFFFF

typedef struct
{
    uint16_t  conn_handle;
    uint16_t  addr_device;
} status_devive_t;

volatile static status_devive_t   list_device[NRF_SDH_BLE_PERIPHERAL_LINK_COUNT];
                          


void put_packet(uint16_t lenght, uint8_t* p_data)
{
    uint32_t err_code;
    
    uint8_t temp = p_data[LEN_ID_APP + LEN_DIRECTION];
    p_data[LEN_ID_APP + LEN_DIRECTION] =  p_data[LEN_ID_APP + LEN_DIRECTION + 1];
    p_data[LEN_ID_APP + LEN_DIRECTION + 1] = temp;

    do
    {
        err_code = app_uart_put(HEADER_PACKET);
        NRF_LOG_INFO("UART: HEADER : %d",HEADER_PACKET);
        if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
        {
            NRF_LOG_ERROR("Failed receiving NUS message. Error 0x%x. ", err_code);
            APP_ERROR_CHECK(err_code);
        }
    } while (err_code == NRF_ERROR_BUSY);

    for(uint16_t count = 0; count < lenght; count++)
    {
        do
        {
            err_code = app_uart_put(p_data[count]);
            NRF_LOG_INFO("UART: %d",p_data[count]);
            if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
            {
                NRF_LOG_ERROR("Failed receiving NUS message. Error 0x%x. ", err_code);
                APP_ERROR_CHECK(err_code);
            }
        } while (err_code == NRF_ERROR_BUSY);
    }
}

void name_change(uint8_t* p_payload, uint16_t len);

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for initializing the timer module.
 */
static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    for(uint8_t i = 0; i < MAX_APP_TIMER; i++)
    {
        m_timer_id[i] = &m_timer_data[i];
        flag_request_dis[i] = false;
    }
}

void start_timer(const uint16_t* conn_handle)
{
    uint32_t err_code;
    uint16_t   connect_handle = (uint16_t) * ((uint16_t*) conn_handle);

    if( connect_handle >= NRF_SDH_BLE_PERIPHERAL_LINK_COUNT )
        return;
    NRF_LOG_INFO("timer start: handle: %x", connect_handle);
    
    flag_request_dis[connect_handle] = true;
    err_code = app_timer_start(m_timer_id[connect_handle], APP_TIMER_TICKS(TIMEOUT_WAIT_PASS), (void*) conn_handle);
    APP_ERROR_CHECK(err_code);
}

void stop_timer(uint16_t connect_handle)
{
    uint32_t err_code;

    if( flag_request_dis[connect_handle] == false )
        return;
    NRF_LOG_INFO("stop timer: handle: %x", connect_handle);

    err_code = app_timer_stop(m_timer_id[connect_handle]);        
    APP_ERROR_CHECK(err_code);
}
/**@brief Timeout handler for the single shot timer.
 */
static void timer_handler(void * p_context)
{
    uint32_t err_code;
    uint16_t   connect_handle = (uint16_t) * ((uint16_t*) p_context);
    uint16_t check_conn = m_qwr[connect_handle].conn_handle;

    stop_timer(connect_handle);

    if( connect_handle >= NRF_SDH_BLE_PERIPHERAL_LINK_COUNT )
        return;
    if( flag_request_dis[connect_handle] == false ) 
        return;
    NRF_LOG_INFO("timeout_wait_pass -> disconnect handle: %x", connect_handle);
    
    err_code = sd_ble_gap_disconnect(connect_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    APP_ERROR_CHECK(err_code);
//    flag_request_dis[connect_handle] = false;
}

/**@brief Create timers.
 */
static void create_timers()
{
    ret_code_t err_code;
    
    for(uint8_t i = 0; i < MAX_APP_TIMER; i++)
    {
        err_code = app_timer_create(&m_timer_id[i], APP_TIMER_MODE_SINGLE_SHOT, timer_handler);
        APP_ERROR_CHECK(err_code);
    }
}

/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    
    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) m_name,
                                          len_name);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
//    uint32_t err_code = ble_advertising_start(&m_advertising, APP_BLE_CONN_CFG_TAG);
//    APP_ERROR_CHECK(err_code);
    ret_code_t err_code;

    err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);

    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_evt_t * p_evt)
{
    uint8_t cmd;
    uint16_t addr = 0x0000;
    uart_packet_t tx_packet;

    memset( (uint8_t*)&tx_packet, 0, sizeof(uart_packet_t) );
    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
        uint32_t err_code;

        NRF_LOG_INFO("Received data from BLE NUS.");
//        NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);
            
        cmd = p_evt->params.rx_data.p_data[0];
        if( cmd == CMD_STATUS_APP || cmd == CMD_STATUS_NODE )
        {             
            stop_timer(p_evt->conn_handle);

            if( cmd == CMD_STATUS_NODE )
                addr = (uint16_t) ( p_evt->params.rx_data.p_data[1] << 8 | p_evt->params.rx_data.p_data[2] );
            for (uint32_t i = 0; i < NRF_SDH_BLE_PERIPHERAL_LINK_COUNT; i++)
            {
                if (m_qwr[i].conn_handle == p_evt->conn_handle)
                {
                    list_device[i].conn_handle  = p_evt->conn_handle;
                    list_device[i].addr_device  = addr;                   
                    NRF_LOG_INFO("nus_data_handler: addr = %x  conn_handle = %x", addr, p_evt->conn_handle);
                    break;
                }
            }
        }
        
        if(addr != 0x0000 )    // node send: notify_status to ble
            tx_packet.packet_struct.dir   = (uint8_t) NODE_2_GATE;
        else
            tx_packet.packet_struct.dir   = (uint8_t) APP_BLE_2_GATE;

        tx_packet.packet_struct.len_payload       = p_evt->params.rx_data.length;

        for(uint16_t i = 0; i < tx_packet.packet_struct.len_payload ; i++)
        {
            tx_packet.packet_struct.payload[i] = p_evt->params.rx_data.p_data[i];
            NRF_LOG_INFO("%2x", tx_packet.packet_struct.payload[i]);
        }

        put_packet( tx_packet.packet_struct.len_payload + LEN_ID_APP + LEN_DIRECTION  + LEN_LEN_PAYLOAD , (uint8_t*)&tx_packet ); 
    }
}
/**@snippet [Handling the data received over BLE] */


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t           err_code;
    ble_nus_init_t     nus_init;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    for (uint32_t i = 0; i < LINK_TOTAL; i++)
    {
        err_code = nrf_ble_qwr_init(&m_qwr[i], &qwr_init);
        APP_ERROR_CHECK(err_code);
    }

    // Initialize NUS.
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;
        default:
            break;
    }
}


/**@brief Function for handling the Connected event.
 *
 * @param[in] p_gap_evt GAP event received from the BLE stack.
 */
static void on_connected(const ble_gap_evt_t * const p_gap_evt)
{
    ret_code_t  err_code;
    uint32_t    periph_link_cnt = ble_conn_state_peripheral_conn_count(); // Number of peripheral links.

    NRF_LOG_INFO("Connection with link 0x%x established.", p_gap_evt->conn_handle);

    // Assign connection handle to available instance of QWR module.
    for (uint32_t i = 0; i < NRF_SDH_BLE_PERIPHERAL_LINK_COUNT; i++)
    {
        if (m_qwr[i].conn_handle == BLE_CONN_HANDLE_INVALID)
        {
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr[i], p_gap_evt->conn_handle);                 
            APP_ERROR_CHECK(err_code);

            start_timer( &(p_gap_evt->conn_handle) );
            break;
        }
    }

    err_code = app_button_enable();
    APP_ERROR_CHECK(err_code);
    
    // Update LEDs
    if ( periph_link_cnt == 1 )
    {
        bsp_board_led_on(CONNECTED_LED1);
        bsp_board_led_off(CONNECTED_LED2);
    }
    else if (periph_link_cnt == 2)
    {
         bsp_board_led_on(CONNECTED_LED1);
         bsp_board_led_on(CONNECTED_LED2);
    }

    if (periph_link_cnt == NRF_SDH_BLE_PERIPHERAL_LINK_COUNT)
    {
        bsp_board_led_off(ADVERTISING_LED);
    }
    else
    {
        // Continue advertising. More connections can be established because the maximum link count has not been reached.
        advertising_start();
    }
}



/**@brief Function for handling the Disconnected event.
 *
 * @param[in] p_gap_evt GAP event received from the BLE stack.
 */
static void on_disconnected(ble_gap_evt_t const * const p_gap_evt)
{
    uart_packet_t   tx_packet;
    uint16_t        addr_dis;
    ret_code_t      err_code;
    uint32_t        periph_link_cnt = ble_conn_state_peripheral_conn_count(); // Number of peripheral links.

    NRF_LOG_INFO("Connection 0x%x has been disconnected. Reason: 0x%X",
                 p_gap_evt->conn_handle,
                 p_gap_evt->params.disconnected.reason);
    
    for (uint32_t i = 0; i < NRF_SDH_BLE_PERIPHERAL_LINK_COUNT; i++)
    {
        if (p_gap_evt->conn_handle == list_device[i].conn_handle)
        { 
            flag_request_dis[p_gap_evt->conn_handle] = false;
            memset( (uint8_t*)&tx_packet, 0, sizeof(uart_packet_t) );
            
            addr_dis  = list_device[i].addr_device;
            
            if( addr_dis != 0 )       // node disconnect
            {
                tx_packet.packet_struct.dir     =   NODE_2_GATE;
                tx_packet.packet_struct.payload[0] = CMD_STATUS_NODE;
            }
            else
            {
                tx_packet.packet_struct.dir     =   APP_BLE_2_GATE;
                tx_packet.packet_struct.payload[0] = CMD_STATUS_APP;
            }

            tx_packet.packet_struct.len_payload     = sizeof(uint8_t) + sizeof(uint16_t) + sizeof(uint32_t);           
            tx_packet.packet_struct.payload[1] = (uint8_t)(addr_dis >> 8);
            tx_packet.packet_struct.payload[2] = (uint8_t)(addr_dis & 0x00FF);
            tx_packet.packet_struct.payload[3] = 0;
            tx_packet.packet_struct.payload[4] = 0;
            tx_packet.packet_struct.payload[5] = 0;
            tx_packet.packet_struct.payload[6] = 0;

            put_packet( tx_packet.packet_struct.len_payload + LEN_ID_APP + LEN_DIRECTION  + LEN_LEN_PAYLOAD , (uint8_t*)&tx_packet);

            list_device[i].addr_device = ADDR_DEVICE_INVAILD;
            list_device[i].conn_handle  = BLE_CONN_HANDLE_INVALID;
            break;
        }
    }
    
    // Update LEDs
    if ( periph_link_cnt == 1 )
    {
        bsp_board_led_on(CONNECTED_LED1);
        bsp_board_led_off(CONNECTED_LED2);
    }
    else if (periph_link_cnt == 2)
    {
         bsp_board_led_on(CONNECTED_LED1);
         bsp_board_led_on(CONNECTED_LED2);
    }

    if (periph_link_cnt == 0)
    {
        bsp_board_led_off(CONNECTED_LED1);
        bsp_board_led_off(CONNECTED_LED2);
        err_code = app_button_disable();
        APP_ERROR_CHECK(err_code);
    }

    if (periph_link_cnt == (NRF_SDH_BLE_PERIPHERAL_LINK_COUNT - 1))
    {
        // Advertising is not running when all connections are taken, and must therefore be started.
        advertising_start();
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t err_code;
uint32_t    periph_link_cnt = ble_conn_state_peripheral_conn_count();
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connected(&p_ble_evt->evt.gap_evt);
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);       
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnected(&p_ble_evt->evt.gap_evt);
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                  p_gatt->att_mtu_desired_central,
                  p_gatt->att_mtu_desired_periph);
}


/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;

        default:
            break;
    }
}


/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint8_t index = 0;
    uint32_t       err_code;

    ble_conn_state_conn_handle_list_t conn_handles = ble_conn_state_periph_handles();

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            break;

        case APP_UART_COMMUNICATION_ERROR:
            NRF_LOG_ERROR("Communication error occurred while handling UART: %08X", p_event->data.error_communication );
            if( p_event->data.error_communication & UART_ERRORSRC_BREAK_Msk )
            {
                NRF_LOG_ERROR("   Break");
            }
            if( p_event->data.error_communication & UART_ERRORSRC_FRAMING_Msk )
            {
                NRF_LOG_ERROR("   Framing");
            }
            if( p_event->data.error_communication & UART_ERRORSRC_PARITY_Msk )
            {
                NRF_LOG_ERROR("   Parity");
            }
            if( p_event->data.error_communication & UART_ERRORSRC_OVERRUN_Msk )
            {
                NRF_LOG_ERROR("   Overrun");
            }
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}
/**@snippet [Handling the data received over UART] */


/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void)
{
    uint32_t                     err_code;
    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
#if defined (UART_PRESENT)
        .baud_rate    = NRF_UART_BAUDRATE_9600//NRF_UART_BAUDRATE_115200
#else
        .baud_rate    = NRF_UARTE_BAUDRATE_115200
#endif
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);
}
/**@snippet [UART Initialization] */


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{

    ret_code_t           err_code;
    ble_advdata_t        advdata;
    ble_advdata_t        srdata;
    ble_gap_adv_params_t adv_params;


    ble_uuid_t adv_uuids[] = {{BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}};

    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = true;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;


    memset(&srdata, 0, sizeof(srdata));
    srdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    srdata.uuids_complete.p_uuids  = adv_uuids;

    err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
    APP_ERROR_CHECK(err_code);

    err_code = ble_advdata_encode(&srdata, m_adv_data.scan_rsp_data.p_data, &m_adv_data.scan_rsp_data.len);
    APP_ERROR_CHECK(err_code);

    // Start advertising.
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.p_peer_addr   = NULL;
    adv_params.filter_policy = BLE_GAP_ADV_FP_ANY;
    adv_params.interval      = APP_ADV_INTERVAL;

    adv_params.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
    adv_params.duration        = APP_ADV_DURATION;
    adv_params.primary_phy     = BLE_GAP_PHY_1MBPS;

    err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &adv_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}

uint16_t  find_conn_handle_from_addr(uint16_t addr)
{
    for (uint32_t i = 0; i < NRF_SDH_BLE_PERIPHERAL_LINK_COUNT; i++)
    {
        if( (list_device[i].addr_device == addr)  && (list_device[i].conn_handle != BLE_CONN_HANDLE_INVALID) )
        {
            NRF_LOG_INFO("find: addr = %4x - handle = %4x", addr,list_device[i].conn_handle );
            return list_device[i].conn_handle;
         }
    }
    return BLE_CONN_HANDLE_INVALID;
}


void process(void)
{
    uint16_t dir   = rx_packet.packet_struct.dir;
    uint16_t len   = rx_packet.packet_struct.len_payload;    
    uint8_t cmd   = rx_packet.packet_struct.payload[0];

    uint16_t addr_node;
    uint16_t node_conn_handle;
    uint16_t app_conn_handle;
    uint32_t       err_code;
    
    if(dir & GATE_2_GATE)
    {
        NRF_LOG_INFO("process: gate to gate");
        if( cmd == CMD_CHANGE_NAME )
        {
            NRF_LOG_INFO("cmd_change_name");
            name_change( (uint8_t*)rx_packet.packet_struct.payload, len );
        }
    }

    if(dir & GATE_2_NODE)
    {
        NRF_LOG_INFO("process: gate to node");
        addr_node   = (uint16_t) ( (rx_packet.packet_struct.payload[1] << 8) | rx_packet.packet_struct.payload[2] ) ;
        node_conn_handle = find_conn_handle_from_addr(addr_node);
        NRF_LOG_INFO("addr_node = 0x%x  conn_handle = 0x%x", addr_node, node_conn_handle);

        do
        {
            err_code = ble_nus_data_send(&m_nus, (uint8_t*)rx_packet.packet_struct.payload, &len, node_conn_handle);
            if ( (err_code != NRF_ERROR_INVALID_STATE) && (err_code != NRF_ERROR_BUSY) &&
                 (err_code != NRF_ERROR_NOT_FOUND) )
            {
                APP_ERROR_CHECK(err_code);
            }
         } while (err_code == NRF_ERROR_BUSY);
    }

    if( dir &  GATE_2_APP_BLE )
    {
        NRF_LOG_INFO("process: gate to app");
        app_conn_handle = find_conn_handle_from_addr(0x0000);
        do
        {
            err_code = ble_nus_data_send(&m_nus, (uint8_t*) rx_packet.packet_struct.payload, &len, app_conn_handle);
            if ( (err_code != NRF_ERROR_INVALID_STATE) && (err_code != NRF_ERROR_BUSY) &&
                 (err_code != NRF_ERROR_NOT_FOUND) )
            {
                APP_ERROR_CHECK(err_code);
            }
         } while (err_code == NRF_ERROR_BUSY);  
     }
}

void name_change(uint8_t* p_payload, uint16_t len)
{
    ret_code_t err_code;
    ble_gap_conn_sec_mode_t sec_mode;
    
    memset(m_name, 0, MAX_LEN_NAME);
    if( p_payload[1] == 0 )  // name = null --> reset name --> use DEFAULT_DEVICE_NAME
    {
        
        record_update();
        len_name = strlen(DEFAULT_DEVICE_NAME);
        memcpy(m_name, (const uint8_t *)DEFAULT_DEVICE_NAME, len_name);
    }
    else
    {
        len_name = ((len-1) >= MAX_LEN_NAME)? MAX_LEN_NAME : (len-1);
        memcpy(m_name, &p_payload[1], len_name);        
        record_update();
     }
     
    
    (void)sd_ble_gap_adv_stop(m_adv_handle);
    m_adv_data.adv_data.len = BLE_GAP_ADV_SET_DATA_SIZE_MAX;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                            (const uint8_t *)m_name,
                                             len_name);
    APP_ERROR_CHECK(err_code);
    advertising_init();
    advertising_start();
}

void init_list_device(void)
{
    for (uint32_t i = 0; i < NRF_SDH_BLE_PERIPHERAL_LINK_COUNT; i++)
        {
            list_device[i].conn_handle  = BLE_CONN_HANDLE_INVALID;
            list_device[i].addr_device  = ADDR_DEVICE_INVAILD;        
        }
}
/**@brief Application main function.
 */
int main(void)
{
    bool erase_bonds;
    uint8_t data;
    uint8_t number_message;
  
    log_init();
    uart_init();

    fds_init_data();
    fds_read_data();
    
    timers_init();
    create_timers();

    buttons_leds_init(&erase_bonds);
    power_management_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();

    init_list_device();
//    printf("\r\nUART started.\r\n");
    advertising_start();
    
    // Enter main loop.
    restart_receive();
    for (;;)
    {   
        idle_state_handle();
        if( app_uart_get(&data) == NRF_SUCCESS )
        {
            NRF_LOG_INFO("%02x", data);
            if( receive_packet(data) == COMPLETE )
            {
                process();
                restart_receive();
            }
        }
            
    }
}


/**
 * @}
 */
