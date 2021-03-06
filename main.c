#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "nrf51_bitfields.h"
#include "nrf_gpio.h"
#include "ble.h"
#include "ble_hci.h"  
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "app_gpiote.h"
//#include "bsp.h"
#ifdef BLE_DFU_APP_SUPPORT
#include "ble_dfu.h"
#include "dfu_app_handler.h"
#endif // BLE_DFU_APP_SUPPORT

#include "spi_master.h"
#include "simple_uart.h"
#include "nrf_temp.h"
//#include "ble_hrs.h"
//#include "ble_dis.h"
#include "ble_bas.h"
static ble_bas_t                             m_bas;                                     /**< Structure used to identify the battery service. */
//static ble_hrs_t                             m_hrs;      
#include "nrf_delay.h"
#include "pstorage.h"
//static bool                                  m_memory_access_in_progress = false;       /**< Flag to keep track of ongoing operations on persistent memory. */
//static pstorage_handle_t      m_storage_handle;                                     /**< Persistent storage handle for blocks requested by the module. */

#define IS_SRVC_CHANGED_CHARACT_PRESENT     0                                       /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/

// User-modifiable configuration parameters.
//      The following values shall be altered when doing power profiling.

#define APP_CFG_NON_CONN_ADV_TIMEOUT  30                                            /**< Time for which the device must be advertising in non-connectable mode (in seconds). */
#define APP_CFG_CHAR_NOTIF_TIMEOUT    5000                                          /**< Time for which the device must continue to send notifications once connected to central (in milli seconds). */
#define APP_CFG_ADV_DATA_LEN          31                                            /**< Required length of the complete advertisement packet. This should be atleast 8 in order to accommodate flag field and other mandatory fields and one byte of manufacturer specific data. */
#define APP_CFG_CONNECTION_INTERVAL   10                                            /**24 < Connection interval used by the central (in milli seconds). This application will be sending one notification per connection interval. A repeating timer will be started with timeout value equal to this value and one notification will be sent everytime this timer expires. */
#define APP_CFG_CHAR_LEN              20                                            /**< Size of the characteristic value being notified (in bytes). */

//#define BLE_DFU_APP_SUPPORT
#ifdef BLE_DFU_APP_SUPPORT
#define DFU_REV_MAJOR                        0x00                                       /** DFU Major revision number to be exposed. */
#define DFU_REV_MINOR                        0x01                                       /** DFU Minor revision number to be exposed. */
#define DFU_REVISION                         ((DFU_REV_MAJOR << 8) | DFU_REV_MINOR)     /** DFU Revision number to be exposed. Combined of major and minor versions. */
#endif // BLE_DFU_APP_SUPPORT

#define HTONL(A)  ((((uint32_t)(A) & 0xff000000) >> 24)  | \
                   (((uint32_t)(A) & 0x00ff0000) >> 8 )  | \
                   (((uint32_t)(A) & 0x0000ff00) << 8 )  | \
                   (((uint32_t)(A) & 0x000000ff) << 24)) 

// Fixed configuration parameters:
//      The following parameters are not meant to be changed while using this application for power
//      profiling.

//#define NOTIF_BUTTON_ID               0                                             /**< Button used for initializing the application in connectable mode. */
//#define NON_CONN_ADV_BUTTON_ID        1                                             /**< Button used for initializing the application in non-connectable mode. */

//#define DEVICE_NAME                   "CSTNet_5TM__7__"                           /**< Name of device. Will be included in the advertising data. */
//#define MANUFACTURER_NAME                    "CSTNET"                      /**< Manufacturer. Will be passed to Device Information Service. */

#define APP_TIMER_PRESCALER           0                                             /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS          5                  /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE       6                                           /**< Size of timer operation queues. */

#define CHAR_NOTIF_TIMEOUT_IN_TKS     APP_TIMER_TICKS(APP_CFG_CHAR_NOTIF_TIMEOUT,\
                                                      APP_TIMER_PRESCALER)          /**< Time for which the device must continue to send notifications once connected to central (in ticks). */

#define CONNECTABLE_ADV_INTERVAL      MSEC_TO_UNITS(TIME_STEP*1000, UNIT_0_625_MS)              /**< The advertising interval for connectable advertisement (20 ms). This value can vary between 20ms to 10.24s. */
#define NON_CONNECTABLE_ADV_INTERVAL  MSEC_TO_UNITS(TIME_STEP*1000, UNIT_0_625_MS)             /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */
#define CONNECTABLE_ADV_TIMEOUT       0   //5  //30                                            /**< Time for which the device must be advertising in connectable mode (in seconds). */

#define SLAVE_LATENCY                 0                                             /**< Slave latency. */
#define CONN_SUP_TIMEOUT              MSEC_TO_UNITS(100, UNIT_10_MS)               /**< Connection supervisory timeout (4 seconds). */

#define ADV_ENCODED_AD_TYPE_LEN       1                                             /**< Length of encoded ad type in advertisement data. */
#define ADV_ENCODED_AD_TYPE_LEN_LEN   1                                             /**< Length of the 'length field' of each ad type in advertisement data. */
#define ADV_FLAGS_LEN                 1                                             /**< Length of flags field that will be placed in advertisement data. */
#define ADV_ENCODED_FLAGS_LEN         (ADV_ENCODED_AD_TYPE_LEN +       \
                                       ADV_ENCODED_AD_TYPE_LEN_LEN +   \
                                       ADV_FLAGS_LEN)                               /**< Length of flags field in advertisement packet. (1 byte for encoded ad type plus 1 byte for length of flags plus the length of the flags itself). */
#define ADV_ENCODED_COMPANY_ID_LEN    2     
#define BLE_ADVDATA_SHORT_NAME_LEN     4                                   /**< Length of the encoded Company Identifier in the Manufacturer Specific Data part of the advertisement data. */
#define ADV_ADDL_MANUF_DATA_LEN       (APP_CFG_ADV_DATA_LEN -                \
                                       (                                     \
                                           ADV_ENCODED_FLAGS_LEN +           \
                                           (                                 \
                                               ADV_ENCODED_AD_TYPE_LEN +     \
                                               ADV_ENCODED_AD_TYPE_LEN_LEN + \
                                               ADV_ENCODED_COMPANY_ID_LEN    \
                                           )                                 \
                                       )                                     \
                                      )                                             /**< Length of Manufacturer Specific Data field that will be placed on the air during advertisement. This is computed based on the value of APP_CFG_ADV_DATA_LEN (required advertisement data length). */

#if APP_CFG_ADV_DATA_LEN > BLE_GAP_ADV_MAX_SIZE
    #error "The required advertisement data size (APP_CFG_ADV_DATA_LEN) is greater than the value allowed by stack (BLE_GAP_ADV_MAX_SIZE). Reduce the value of APP_CFG_ADV_DATA_LEN and recompile."
#endif 

// Check whether the maximum characteristic length + opcode length (1) + handle length (2) is not
// greater than default MTU size.
#if (APP_CFG_CHAR_LEN + 1 + 2) > BLE_L2CAP_MTU_DEF
    #error "The APP_CFG_CHAR_LEN is too large for the maximum MTU size."
#endif 

#if ADV_ADDL_MANUF_DATA_LEN < 1
    #error "The required length of additional manufacturer specific data computed based on the user configured values is computed to be less than 1. Consider increasing the value of APP_CFG_ADV_DATA_LEN."
#endif

#define COMPANY_IDENTIFIER            0x5881                                        /**< 0x0059 Company identifier for Nordic Semiconductor ASA as per www.bluetooth.org. */

#define LOCAL_SERVICE_UUID            0x1523                                        /**< Proprietary UUID for local service. */
#define LOCAL_CHAR_UUID               0x1524                                        /**< Proprietary UUID for local characteristic. */

#define DEAD_BEEF                     0xDEADBEEF                                    /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

/*------------------------------------------------------------------------------------------------*/
//#define V5TM              			// for 5TM sensor  data log,  300Secs period.  Comment for timecounter test, 5Secs period.
//#define BEACON   							  // For iBeacon mode test
//#define SHENZHEN
//#define MS49SF1
#define AAAPCB
#ifdef BEACON
	#define UART_POWER  				12   //5TM Power
	#define UART_RX							15   //17
	#define UART_TX							16   //18
	#define LED_PIN             20   //19
	#define ADC_AIN						  ADC_CONFIG_PSEL_AnalogInput4
	#define TIME_PERIOD	 				300      /*For test  time period in seconds*/
	static uint8_t adv_name[22] = "BCON_Time:" ;

#elif defined(MS49SF1)
	#define UART_POWER  				8   //5TM Power
	#define UART_RX							16
	#define UART_TX							17
	#define ADC_AIN						  ADC_CONFIG_PSEL_AnalogInput5
	#define LED_PIN             22
	#define TIME_PERIOD	 				300      /*For test  time period in seconds*/
	static uint8_t adv_name[22] = "GPS_5TM:  " ;

#elif defined(SHENZHEN)
	#define UART_POWER  				12   //5TM Power
	#define UART_RX							15
	#define UART_TX							16
	#define ADC_AIN						  ADC_CONFIG_PSEL_AnalogInput4
	#define LED_PIN             20
	#define TIME_PERIOD	 				300      /*For test  time period in seconds*/
	static uint8_t adv_name[22] = "SZ_5TM0:  " ;


#elif defined(AAAPCB)
	#define UART_POWER  				13   //5TM Power
	#define UART_RX							12
	#define UART_TX							10
	#define ADC_AIN						  ADC_CONFIG_PSEL_AnalogInput4
	#define LED_PIN             20
	#define TIME_PERIOD	 				300      /*For test  time period in seconds*/
	static uint8_t adv_name[22] = "AAA_5TM:  " ;

#else
	#define UART_POWER  				13   //5TM Power
	#define UART_RX							12
	#define UART_TX							10
	#define ADC_AIN						  ADC_CONFIG_PSEL_AnalogInput4
	#define LED_PIN             20
	#define TIME_PERIOD	 				300      /*For test  time period in seconds*/
	static uint8_t adv_name[22] = "XXX_Time:" ;

#endif


#define TIME_STEP 				     10      
#if (TIME_PERIOD % TIME_STEP) 
	#error "timer step not TIME_PERIOD int times"
#endif
  
#define PSTORAGE_PAGE_SIZE		1024

/**@brief 128-bit UUID base List. */
//static const ble_uuid128_t m_base_uuid128 =
//{
//   {
//       0x23, 0xD1, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x15,
//       0xDE, 0xEF, 0x12, 0x12, 0x00, 0x00, 0x00, 0x00
//   }
//};

static ble_gap_adv_params_t      m_adv_params;                                       /**< Parameters to be passed to the stack when starting advertising. */
//static uint8_t                  m_char_value[APP_CFG_CHAR_LEN];                     /**< Value of the characteristic that will be sent as a notification to the central. */
static uint8_t                   m_addl_adv_manuf_data[ADV_ADDL_MANUF_DATA_LEN];     /**< Value of the additional manufacturer specific data that will be placed in air (initialized to all zeros). */
//static ble_gatts_char_handles_t m_char_handles;                                     /**< Handles of local characteristic (as provided by the BLE stack).*/
static uint16_t                  m_conn_handle = BLE_CONN_HANDLE_INVALID;            /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection).*/
static uint16_t                  m_service_handle;                                   /**< Handle of local service (as provided by the BLE stack).*/
static bool                      m_is_notifying_enabled = false;                     /**< Variable to indicate whether the notification is enabled by the peer.*/
static app_timer_id_t            m_conn_int_timer_id;                                /**< Connection interval timer. */
static app_timer_id_t            m_notif_timer_id;                                   /**< Notification timer. */
/////////////////////////added  by yelun
//static app_timer_id_t           m_s5tm_timer_id;  
static app_timer_id_t            m_timecounter_id;  
static uint32_t 							   flash_page_data[256+4];
static uint32_t 								 timer_counter = 0;
static uint32_t 							   timer_shifting = 0;
static pstorage_handle_t 				 flash_handle_last;

static ble_advdata_t             advdata;
static ble_advdata_manuf_data_t  manuf_data;
static uint8_t                   flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE; //BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

//static char 									 	 *advname = "Time";
//#define START_ADDRESS 0x1B000	  /*Data recordes start address, 12KB program flash from 0x160005*/
static uint16_t  Vtm_humi=0, Vtm_unknow=0, Vtm_temp=0;  //sensor data;
static int32_t   nrf_temp=0;

static bool                                  m_memory_access_in_progress = false;       /**< Flag to keep track of ongoing operations on persistent memory. */

#ifdef BLE_DFU_APP_SUPPORT    
static ble_dfu_t                             m_dfus;                                    /**< Structure used to identify the DFU service. */
#endif // BLE_DFU_APP_SUPPORT    

static uint8_t pstorage_wait_flag = 0;
static pstorage_block_t pstorage_wait_handle = 0;
static pstorage_handle_t       flash_base_handle;
static uint32_t 	flash_block_num;
// YOUR_JOB: Modify these according to requirements (e.g. if other event types are to pass through
//           the scheduler).
#define SCHED_MAX_EVENT_DATA_SIZE       sizeof(app_timer_event_t)                   /**< Maximum size of scheduler events. Note that scheduler BLE stack events do not contain any data, as the events are being pulled from the stack in the event handler. */
#define SCHED_QUEUE_SIZE                10                                          /**< Maximum number of events in the scheduler queue. */

// Persistent storage system event handler
void pstorage_sys_event_handler (uint32_t p_evt);


static void application_timers_stop(void);
static void application_timers_start(void);

////////////////////////////////////////////////
/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze 
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for the Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);

}


uint16_t    batt_lvl_in_milli_volts;

#define ADC_REF_VOLTAGE_IN_MILLIVOLTS        1200                                      /**< Reference voltage (in milli volts) used by ADC while doing conversion. */
#define ADC_PRE_SCALING_COMPENSATION         3                                         /**< The ADC is configured to use VDD with 1/3 prescaling as input. And hence the result of conversion is to be multiplied by 3 to get the actual value of the battery voltage.*/
#define DIODE_FWD_VOLT_DROP_MILLIVOLTS       270                                       /**< Typical forward voltage drop of the diode (Part no: SD103ATW-7-F) that is connected in series with the voltage supply. This is the voltage drop when the forward current is 1mA. Source: Data sheet of 'SURFACE MOUNT SCHOTTKY BARRIER DIODE ARRAY' available at www.diodes.com. */
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE)\
        ((((ADC_VALUE) * ADC_REF_VOLTAGE_IN_MILLIVOLTS) / 255) * ADC_PRE_SCALING_COMPENSATION)
void battery_start(uint32_t AnalogInput)
{
//    uint32_t err_code=0;
//	sd_nvic_SetPriority(ADC_IRQn, NRF_APP_PRIORITY_LOW);  
//	sd_nvic_EnableIRQ(ADC_IRQn);
//	

//		if (AnalogInput==4)  AnalogInput=ADC_CONFIG_PSEL_AnalogInput4 ;
//		else if (AnalogInput==1)  AnalogInput=ADC_CONFIG_PSEL_AnalogInput1 ;
//		else  AnalogInput=ADC_CONFIG_PSEL_AnalogInput6 ;
    // Configure ADC
    NRF_ADC->INTENSET   = AAR_INTENSET_END_Disabled;   				//ADC  Interrupt disabled. Must be disable.
    NRF_ADC->CONFIG     = (ADC_CONFIG_RES_8bit                        << ADC_CONFIG_RES_Pos)     |
                          (ADC_CONFIG_INPSEL_AnalogInputOneThirdPrescaling << ADC_CONFIG_INPSEL_Pos)  |
                          (ADC_CONFIG_REFSEL_VBG                      << ADC_CONFIG_REFSEL_Pos)  |
                          (AnalogInput                   << ADC_CONFIG_PSEL_Pos)    |
                          (ADC_CONFIG_EXTREFSEL_None                  << ADC_CONFIG_EXTREFSEL_Pos);
    NRF_ADC->ENABLE     = ADC_ENABLE_ENABLE_Enabled;

//			uint32_t p_is_running = 0;

//			sd_clock_hfclk_request();
//			while(! p_is_running) {  							//wait for the hfclk to be available
//				sd_clock_hfclk_is_running((&p_is_running));
//			}               
		NRF_ADC->TASKS_START = 1;							//Start ADC sampling
    NRF_ADC->EVENTS_END  = 0;    // Stop any running conversions.
				uint8_t adc_delay=0;
        while (!NRF_ADC->EVENTS_END && adc_delay<0xff)  adc_delay++;
    NRF_ADC->EVENTS_END  = 0;    // Stop any running conversions.

        NRF_ADC->TASKS_STOP     = 1;
				NRF_ADC->ENABLE     = ADC_ENABLE_ENABLE_Disabled;
        batt_lvl_in_milli_volts = ADC_RESULT_IN_MILLI_VOLTS(NRF_ADC->RESULT); // + DIODE_FWD_VOLT_DROP_MILLIVOLTS;
//		sd_clock_hfclk_release();
}

//void ADC_IRQHandler(void)
//{
//	/* Clear dataready event */
//  NRF_ADC->EVENTS_END = 0;	
//   batt_lvl_in_milli_volts = ADC_RESULT_IN_MILLI_VOLTS(NRF_ADC->RESULT); // + DIODE_FWD_VOLT_DROP_MILLIVOLTS;
//	
//	//Use the STOP task to save current. Workaround for PAN_028 rev1.5 anomaly 1.
//  NRF_ADC->TASKS_STOP = 1;
//	NRF_ADC->ENABLE     = ADC_ENABLE_ENABLE_Disabled;

//	//Release the external crystal
//	sd_clock_hfclk_release();
//}	


/**@brief Function for the Characteristic notification.
 *
 * @details Sends one characteristic value notification to peer if connected to it and the
 *          notification is enabled.
 */
static void char_notify(void)
{
    uint32_t err_code;
    uint16_t len = APP_CFG_CHAR_LEN;
		err_code = NRF_SUCCESS;

    // Send value if connected and notifying.
    if ((m_conn_handle != BLE_CONN_HANDLE_INVALID) && m_is_notifying_enabled)
    {
				ble_gatts_hvx_params_t 		hvx_params;
        memset(&hvx_params, 0, sizeof(hvx_params));
        hvx_params.handle   = m_bas.Vtm_date_time_handles.value_handle;  //   m_char_handles.value_handle;
//        hvx_params.handle   = m_bas.battery_level_handles.value_handle;  //   m_char_handles.value_handle;
        hvx_params.type     = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset   = 0;
        hvx_params.p_len    = &len;
//        hvx_params.p_data   = (uint8_t *) flash_load;  //m_char_value;
//				pstorage_handle_t flash_handle;

				while(true){
					hvx_params.p_data   = ((uint8_t *) &flash_page_data[(uint8_t)timer_shifting]);  //m_char_value;
					err_code = sd_ble_gatts_hvx(m_conn_handle, &hvx_params);
					if ((err_code == BLE_ERROR_NO_TX_BUFFERS) ||
							(err_code == NRF_ERROR_INVALID_STATE) ||
							(err_code == BLE_ERROR_GATTS_SYS_ATTR_MISSING)
					)
							{
									break;
							}
					else if ((err_code != NRF_SUCCESS))
							{
									APP_ERROR_HANDLER(err_code);
							}
					else 	if (timer_shifting > 254)
							{
								m_is_notifying_enabled =  false;
								application_timers_start();
								break;
							}
					else {
									timer_shifting += 5;
								}
					}
		}
}
 char char_hex( char bHex){
		bHex&=0x0f;
    if((bHex<10))
        bHex += '0';
    else bHex += 'A'-10;
//    else bHex = 0xff;
    return bHex;
}

/**@brief Function for the GAP initialization.
 *
 * @details This function shall be used to setup all the necessary GAP (Generic Access Profile)
 *          parameters of the device. It also sets the permissions.
 *
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
		uint8_t 								i;
//    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
		ble_gap_addr_t  	p_addr;
    sd_ble_gap_address_get	(	&p_addr	);
//		sprintf(adv_name, "CSTNET_%4s:%x%x%x%x%x%x",sadvname,p_addr.addr[5],p_addr.addr[4],p_addr.addr[3],p_addr.addr[2],p_addr.addr[1],p_addr.addr[0]);
//    for (i=7;sadvname[i-7];i++) adv_name[i]=sadvname[i-7];
		i = 10;
		adv_name[i++] = char_hex(p_addr.addr[5]>>4);		adv_name[i++] = char_hex(p_addr.addr[5]);
		adv_name[i++] = char_hex(p_addr.addr[4]>>4);		adv_name[i++] = char_hex(p_addr.addr[4]);
		adv_name[i++] = char_hex(p_addr.addr[3]>>4);		adv_name[i++] = char_hex(p_addr.addr[3]);
		adv_name[i++] = char_hex(p_addr.addr[2]>>4);		adv_name[i++] = char_hex(p_addr.addr[2]);
		adv_name[i++] = char_hex(p_addr.addr[1]>>4);		adv_name[i++] = char_hex(p_addr.addr[1]);
		adv_name[i++] = char_hex(p_addr.addr[0]>>4);		adv_name[i++] = char_hex(p_addr.addr[0]);
		err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)adv_name ,
                                          22);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the connectable advertisement parameters.
 *
 * @details This function initializes the advertisement parameters to values that will put 
 *          the application in connectable mode.
 *
 */
static void connectable_adv_init(void)
{
    // Initialize advertising parameters (used when starting advertising).
    memset(&m_adv_params, 0, sizeof(m_adv_params));
    
    m_adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND ;
    m_adv_params.p_peer_addr = NULL;                               // Undirected advertisement
    m_adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval    = CONNECTABLE_ADV_INTERVAL;
    m_adv_params.timeout     = CONNECTABLE_ADV_TIMEOUT;
}

/**@brief Function for initializing the Advertisement packet.
 *
 * @details This function initializes the data that is to be placed in an advertisement packet in 
 *          both connectable and non-connectable modes.
 *
 */

#ifdef BLE_DFU_APP_SUPPORT    
static void advertising_stop(void)
{
    uint32_t err_code;

    err_code = sd_ble_gap_adv_stop();
    APP_ERROR_CHECK(err_code);

//    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
//    APP_ERROR_CHECK(err_code);
}


/** @snippet [DFU BLE Reset prepare] */
static void reset_prepare(void)
{
    uint32_t err_code;
    
    if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        // Disconnect from peer.
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
//        err_code = bsp_indication_set(BSP_INDICATE_IDLE);
//        APP_ERROR_CHECK(err_code);
    }
    else
    {
        // If not connected, then the device will be advertising. Hence stop the advertising.
        advertising_stop();
    }

    err_code = ble_conn_params_stop();
    APP_ERROR_CHECK(err_code);
}
/** @snippet [DFU BLE Reset prepare] */
#endif // BLE_DFU_APP_SUPPORT    

static void service_add(void)
{
    uint32_t       			err_code;
//    ble_bas_init_t 			bas_init;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
//    uint8_t             init_len;

    // Initialize Battery Service.
//    memset(&bas_init, 0, sizeof(bas_init));

    // Here the sec level for the Battery Service can be changed/increased.
//    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.cccd_write_perm);
//    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.read_perm);
//    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&bas_init.battery_level_char_attr_md.write_perm);

//    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_report_read_perm);


    // Add service
    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_ALERT_NOTIFICATION_SERVICE);
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &m_service_handle);
    APP_ERROR_CHECK(err_code);


    // cccd_md init characteristic
        memset(&cccd_md, 0, sizeof(cccd_md));
        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
//        cccd_md.write_perm = bas_init.battery_level_char_attr_md.cccd_write_perm;
        cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

				memset(&char_md, 0, sizeof(char_md));
//				char_md.char_props.broadcast = 1;
				char_md.char_props.read   = 1;
		    char_md.char_props.write  = 1;
				char_md.char_props.notify = 1;
				char_md.p_char_user_desc  = NULL;
				char_md.p_char_pf         = NULL;
				char_md.p_user_desc_md    = NULL;
				char_md.p_cccd_md         = (char_md.char_props.notify) ? &cccd_md : NULL;
				char_md.p_sccd_md         = (char_md.char_props.broadcast) ? &cccd_md : NULL;

				memset(&attr_md, 0, sizeof(attr_md));
				BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
				BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

//				attr_md.read_perm  = bas_init.battery_level_char_attr_md.read_perm;
////				attr_md.write_perm = bas_init.battery_level_char_attr_md.write_perm;
				attr_md.vloc       = BLE_GATTS_VLOC_STACK;
				attr_md.rd_auth    = 0;
				attr_md.wr_auth    = 0;
				attr_md.vlen       = 1;  //0;

				memset(&attr_char_value, 0, sizeof(attr_char_value));

				attr_char_value.p_attr_md = &attr_md;
				attr_char_value.init_len  = sizeof(uint32_t);
				attr_char_value.init_offs = 0;
				attr_char_value.max_len   = 20; //sizeof(uint32_t);
				attr_char_value.p_value   = NULL; 
// char data_time
				BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_DATE_TIME_CHAR);
				attr_char_value.p_uuid    = &ble_uuid;
				err_code = sd_ble_gatts_characteristic_add(m_bas.service_handle, &char_md,
																									 &attr_char_value,
																									 &m_bas.Vtm_date_time_handles);
//				char_md.char_props.broadcast = 0;
//				char_md.char_props.read   = 1;
				attr_char_value.init_len  = sizeof(uint32_t);
				char_md.char_props.write   = 0;
				char_md.char_props.notify = 0;
				char_md.p_cccd_md         = (char_md.char_props.notify) ? &cccd_md : NULL;
				char_md.p_sccd_md         = (char_md.char_props.broadcast) ? &cccd_md : NULL;

				attr_char_value.max_len   = sizeof(uint32_t);
// char battery_level
				BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_BATTERY_LEVEL_CHAR);
				attr_char_value.p_uuid    = &ble_uuid;
				err_code = sd_ble_gatts_characteristic_add(m_bas.service_handle, &char_md,
																									 &attr_char_value,
																									 &m_bas.battery_level_handles);

				BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_TEMPERATURE_MEASUREMENT_CHAR);
				attr_char_value.p_uuid    = &ble_uuid;
				err_code = sd_ble_gatts_characteristic_add(m_bas.service_handle, &char_md,
																									 &attr_char_value,
																									 &m_bas.Vtm_temp_level_handles);

				BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_HID_INFORMATION_CHAR);
				attr_char_value.p_uuid    = &ble_uuid;
				err_code = sd_ble_gatts_characteristic_add(m_bas.service_handle, &char_md,
																									 &attr_char_value,
																									 &m_bas.Vtm_humi_level_handles);

				BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_TEMPERATURE_TYPE_CHAR);
				attr_char_value.p_uuid    = &ble_uuid;
				err_code = sd_ble_gatts_characteristic_add(m_bas.service_handle, &char_md,
																									 &attr_char_value,
																									 &m_bas.Nrf_temp_level_handles);

				char_md.char_props.write   = 1;
				BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_REPORT_CHAR);
				attr_char_value.p_uuid    = &ble_uuid;
				err_code = sd_ble_gatts_characteristic_add(m_bas.service_handle, &char_md,
																									 &attr_char_value,
																									 &m_bas.Nrf_name_handles);

#ifdef BLE_DFU_APP_SUPPORT    
    /** @snippet [DFU BLE Service initialization] */
    ble_dfu_init_t   dfus_init;

    // Initialize the Device Firmware Update Service.
    memset(&dfus_init, 0, sizeof(dfus_init));

    dfus_init.evt_handler    = dfu_app_on_dfu_evt;
    dfus_init.error_handler  = NULL; //service_error_handler - Not used as only the switch from app to DFU mode is required and not full dfu service.
    dfus_init.evt_handler    = dfu_app_on_dfu_evt;
    dfus_init.revision       = DFU_REVISION;

    err_code = ble_dfu_init(&m_dfus, &dfus_init);
    APP_ERROR_CHECK(err_code);
    
    dfu_app_reset_prepare_set(reset_prepare);
    /** @snippet [DFU BLE Service initialization] */
#endif // BLE_DFU_APP_SUPPORT    
}


/**@brief Function for handling the Notification timeout.
 *
 * @details This function will be called when the notification timer expires. This will stop the
 *          timer for connection interval and disconnect from the peer.
 *
 * @param[in]   p_context   Pointer used for passing some arbitrary information (context) from the
 *                          app_start_timer() call to the timeout handler.
 */
static void notif_timeout_handler(void * p_context)
{
    uint32_t err_code;
    
    UNUSED_PARAMETER(p_context);

    // Stop all notifications (by stopping the timer for connection interval that triggers 
    // notifications and disconnecting from peer).
    err_code = app_timer_stop(m_conn_int_timer_id); 
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the Connection interval timeout.
 *
 * @details This function will be called when the connection interval timer expires. This will
 *          trigger another characteristic notification to the peer.
 *
 * @param[in]   p_context   Pointer used for passing some arbitrary information (context) from the
 *                          app_start_timer() call to the timeout handler.
 */
static void connection_interval_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);

    // Into next connection interval. Send one notification.
				char_notify();
}
static void example_cb_handler(pstorage_handle_t  * handle,
															 uint8_t              op_code,
                               uint32_t             result,
                               uint8_t            * p_data,
                               uint32_t             data_len)
{
		if(handle->block_id == pstorage_wait_handle) { pstorage_wait_flag = 0; }  //If we are waiting for this callback, clear the wait flag.
}

static void s5tm_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
		uint8_t data_id = 1, data_pos = false;
    static uint32_t data_saved ;  //must be STATIC for pstorage_store()
		uint32_t err_code=0;
		uint8_t timer_flash_id = ((uint8_t)( timer_counter/TIME_PERIOD));
//		uint16_t len = 4;
//		timer_counter+=TIME_PERIOD;
				battery_start(ADC_AIN);	 //For 3.3V boster
//					battery_start(6);  //For 3.7V Li Battery 
	
//				ble_bas_battery_level_update(&m_bas,batt_lvl_in_milli_volts/100);  //For display 3.0V*100 on IOS
				
//				nrf_gpio_pin_clear (UART_POWER);												//Open power and UART for 5TM 
				uint8_t cr; //rx_count=10;
				uint8_t adv_i = 4;
				Vtm_humi=0,Vtm_unknow=0,Vtm_temp=0;
				NRF_UART0->POWER = (UART_POWER_POWER_Enabled << UART_POWER_POWER_Pos);
				simple_uart_config(NULL, UART_TX, NULL, UART_RX, false);
		  	nrf_gpio_pin_set (UART_POWER);												//Open power and UART for 5TM 
				while (simple_uart_get_with_timeout(4, &cr));					//clear RX buffer, must be!
				for (uint8_t i=0;i<50;i++){										//Get date from 5tm 
//						m_addl_adv_manuf_data[rx_count++]=cr;  //simple_uart_put(cr);
					if (simple_uart_get_with_timeout(4, &cr)) {
							if (cr==0x0D) break;
//							m_addl_adv_manuf_data[adv_i++] = cr;
							if (cr >='0' && cr<= '9') 
//							if ((cr>>4) == 3 ) 
								{ 
								data_pos = true;    //5TM data start postion.
								switch (data_id){
									case 1: {
										Vtm_humi = Vtm_humi<<4;
										Vtm_humi |= (cr & 0x0F) ;
										break;
										}
									case 2:  {
										Vtm_unknow = Vtm_unknow<<4;
										Vtm_unknow |= (cr & 0x0F) ;
										break;
										}
									case 3: {
										Vtm_temp = Vtm_temp<<4;
										Vtm_temp |= (cr & 0x0F) ;
										break;
										}
									default: break;
								}
							}
						else if(data_pos && cr==' ') data_id++;
					}
		}	
				NRF_UART0->POWER = (UART_POWER_POWER_Disabled << UART_POWER_POWER_Pos);
//				nrf_gpio_cfg_output(UART_RX);   //Add this line will be more power.

				nrf_gpio_pin_clear (UART_POWER);
//				nrf_gpio_pin_set (UART_POWER);
				sd_temp_get(&nrf_temp);  													//Get Cpu tempreature
#ifdef BEACON
				data_saved = timer_counter ; //test flash write
#else
				data_saved = (Vtm_humi << 16) + Vtm_temp ;
#endif
						pstorage_handle_t 		flash_handle;
//						pstorage_block_identifier_get(&flash_base_handle,((((timer_counter/TIME_PERIOD)>>8))%PSTORAGE_MAX_APPLICATIONS), &flash_handle);
						pstorage_block_identifier_get(&flash_base_handle,((timer_counter/(TIME_PERIOD*(PSTORAGE_PAGE_SIZE/4)))%PSTORAGE_MAX_APPLICATIONS), &flash_handle);
						if (flash_handle.block_id != flash_handle_last.block_id || flash_handle.module_id != flash_handle_last.module_id) {
								flash_handle_last = flash_handle;
								err_code = pstorage_clear(&flash_handle,PSTORAGE_PAGE_SIZE);
							}
						err_code = pstorage_store(&flash_handle, (uint8_t * )&data_saved, 4, timer_flash_id*4);

			 *( (uint32_t *)     m_addl_adv_manuf_data)  = timer_counter ;
			 *( (uint16_t *) (m_addl_adv_manuf_data+4))  = Vtm_temp ;
			 *( (uint16_t *) (m_addl_adv_manuf_data+6))  = Vtm_unknow ;
			 *( (uint16_t *) (m_addl_adv_manuf_data+8))  = Vtm_humi;
			 *( (uint16_t *) (m_addl_adv_manuf_data+10)) = batt_lvl_in_milli_volts ;
			 *( (uint32_t *) (m_addl_adv_manuf_data+12)) = nrf_temp ;
				err_code = ble_advdata_set(&advdata, NULL);
				APP_ERROR_CHECK(err_code);
}

static void timecounter_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
		timer_counter += TIME_STEP;
		if (timer_counter%TIME_PERIOD){  
				*( (uint32_t *) m_addl_adv_manuf_data) = timer_counter ;
			uint32_t	err_code = ble_advdata_set(&advdata, NULL);
//				APP_ERROR_CHECK(err_code);
			}
		else s5tm_timeout_handler(NULL);
}




/**@brief Function for starting application timers.
 *
 * @details This function will be start two timers - one for the time duration for which 
 *          notifications have to be sent to the peer and another for the interval between two 
 *          notifications (which is also the connection interval).
 */
static void application_timers_start(void)
{
    uint32_t err_code;

    // Start connection interval timer.     
//    err_code = app_timer_start(m_conn_int_timer_id,
//                               APP_TIMER_TICKS(APP_CFG_CONNECTION_INTERVAL, APP_TIMER_PRESCALER),
//                               NULL);
//    APP_ERROR_CHECK(err_code);

    // Start characteristic notification timer.
    err_code = app_timer_start(m_notif_timer_id, CHAR_NOTIF_TIMEOUT_IN_TKS, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for Stopping application timers.
 */
static void application_timers_stop(void)
{
    uint32_t err_code;
    
    err_code = app_timer_stop(m_notif_timer_id);
    APP_ERROR_CHECK(err_code);

//    err_code = app_timer_stop(m_conn_int_timer_id);
//    APP_ERROR_CHECK(err_code);

//    err_code = app_timer_create(&m_s5tm_timer_id,
//                                APP_TIMER_MODE_REPEATED,
//                                s5tm_timeout_handler);
//    APP_ERROR_CHECK(err_code);

}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t err_code;
    uint32_t count;

    // Verify if there is any flash access pending, if yes delay starting advertising until
    // it's complete.
		err_code = pstorage_access_status_get(&count);
		APP_ERROR_CHECK(err_code);
		if (count) nrf_delay_ms(20);
    err_code = sd_ble_gap_adv_start(&m_adv_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the Timer initialization.
 *
* @details Initializes the timer module.
*/
static void timers_init(void)
{
    uint32_t err_code;

    // Initialize timer module
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);

    // Create timers
    err_code = app_timer_create(&m_conn_int_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                connection_interval_timeout_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_notif_timer_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                notif_timeout_handler);
    APP_ERROR_CHECK(err_code);


//    err_code = app_timer_create(&m_s5tm_timer_id,
//                                APP_TIMER_MODE_REPEATED,
//                                s5tm_timeout_handler);
//    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_timecounter_id,
                                APP_TIMER_MODE_REPEATED,
                                timecounter_handler);
    APP_ERROR_CHECK(err_code);

}


/**@brief Function for evaluating the value written in CCCD 
 *
 * @details This shall be called when there is a write event received from the stack. This 
 *          function will evaluate whether or not the peer has enabled notifications and
 *          start/stop notifications accordingly.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_write(ble_evt_t * p_ble_evt)
{
//		uint32_t err_code;
    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
    if ((p_evt_write->handle == m_bas.Vtm_date_time_handles.value_handle) && (p_evt_write->len == 1))
			flash_block_num = *p_evt_write->data;
		else if ((p_evt_write->handle == m_bas.Nrf_name_handles.value_handle) && (p_evt_write->len == 4))
			{
				adv_name[0] = *p_evt_write->data;
				adv_name[1] = *(p_evt_write->data+1);
				adv_name[2] = *(p_evt_write->data+2);
				adv_name[3] = *(p_evt_write->data+3);
				gap_params_init();
				m_addl_adv_manuf_data[16] = adv_name[0];
				m_addl_adv_manuf_data[17] = adv_name[1];
				m_addl_adv_manuf_data[18] = adv_name[2];
				m_addl_adv_manuf_data[19] = adv_name[3];
			  ble_advdata_set(&advdata, NULL);

			}
    else if ((p_evt_write->handle == m_bas.Vtm_date_time_handles.cccd_handle) && (p_evt_write->len == 2))
//    if ((p_evt_write->handle == m_bas.battery_level_handles.cccd_handle) && (p_evt_write->len == 2))
    {
        // CCCD written. Start notifications
        m_is_notifying_enabled = ble_srv_is_notification_enabled(p_evt_write->data);

        if (m_is_notifying_enabled)
        {
						application_timers_stop();
//						timer_shifting = (timer_counter/TIME_PERIOD)& 0xfffffffC;  // intergreat /4
						timer_shifting = 0;
						pstorage_handle_t flash_handle;
						uint16_t len=4;
//						sd_ble_gatts_value_get(m_bas.Vtm_date_time_handles.value_handle,0, &len, (uint8_t *)&flash_block_num);
						if (flash_block_num > PSTORAGE_MAX_APPLICATIONS) flash_block_num = PSTORAGE_MAX_APPLICATIONS-1;
						pstorage_block_identifier_get(&flash_base_handle,flash_block_num, &flash_handle);
						pstorage_load((uint8_t *)flash_page_data, &flash_handle,1024,0);
						flash_page_data[259] = flash_block_num-- ;
						flash_page_data[258] = PSTORAGE_MAX_APPLICATIONS;
						flash_page_data[257] = PSTORAGE_PAGE_SIZE;
						flash_page_data[256] = TIME_PERIOD;
						ble_gatts_value_t gatts_value;
						memset(&gatts_value, 0, sizeof(gatts_value));
						gatts_value.len     = sizeof(uint32_t);
						gatts_value.offset  = 0;
						gatts_value.p_value = (uint8_t *)&flash_block_num;

						sd_ble_gatts_value_set(m_bas.conn_handle, m_bas.Vtm_date_time_handles.value_handle, &gatts_value);
            char_notify();
						application_timers_start();
        }
    }
}


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code = NRF_SUCCESS;
		uint32_t ss;
		uint16_t len = 4;
    
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
						nrf_gpio_pin_set (LED_PIN);												//Led on.
						application_timers_start();
						ble_gap_conn_params_t   gap_conn_params;
						gap_conn_params.min_conn_interval = ((4 * APP_CFG_CONNECTION_INTERVAL) / 5);  //= 10
						gap_conn_params.max_conn_interval = (4 * 24) / 5;
						gap_conn_params.slave_latency     = 4;		//SLAVE_LATENCY;
						gap_conn_params.conn_sup_timeout  = MSEC_TO_UNITS(400, UNIT_10_MS) ;	//CONN_SUP_TIMEOUT;
//						gap_conn_params.min_conn_interval = (4 * 20) / 5;	//Interval Max * (Slave Latency + 1) <= 2 seconds
//						gap_conn_params.max_conn_interval = (4 * 40) / 5;	//Interval Min + 20 ms >= Interval Max Slave Latency = 4
//						gap_conn_params.slave_latency     = 8;		//connSupervisionTimeout <= 6 seconds
//						gap_conn_params.conn_sup_timeout  = MSEC_TO_UNITS(1330, UNIT_10_MS) ;	//Interval Max * (Slave Latency + 1) * 3 < connSupervisionTimeout
						err_code=sd_ble_gap_conn_param_update(m_conn_handle,&gap_conn_params);
            APP_ERROR_CHECK(err_code);
//						flash_block_num = ((timer_shifting)>>8)%PSTORAGE_MAX_APPLICATIONS;
						flash_block_num = (timer_counter/(TIME_PERIOD*(PSTORAGE_PAGE_SIZE/4)))%PSTORAGE_MAX_APPLICATIONS;
//						ss = HTONL(timer_counter);
						ble_gatts_value_t gatts_value;
						memset(&gatts_value, 0, sizeof(gatts_value));
						gatts_value.len     = sizeof(uint32_t);
						gatts_value.offset  = 0;
						gatts_value.p_value = (uint8_t *)&flash_block_num;

						sd_ble_gatts_value_set(m_bas.conn_handle, m_bas.Vtm_date_time_handles.value_handle, &gatts_value);
// Updata server data for display  
						ss = HTONL(batt_lvl_in_milli_volts);
						gatts_value.p_value = (uint8_t *)&ss;

						sd_ble_gatts_value_set(m_bas.conn_handle, m_bas.battery_level_handles.value_handle, &gatts_value);
						ss = HTONL(Vtm_temp);
						sd_ble_gatts_value_set(m_bas.conn_handle,m_bas.Vtm_temp_level_handles.value_handle, &gatts_value);
						ss = HTONL(Vtm_humi);
						sd_ble_gatts_value_set(m_bas.conn_handle,m_bas.Vtm_humi_level_handles.value_handle, &gatts_value);
						ss = HTONL(nrf_temp);
						sd_ble_gatts_value_set(m_bas.conn_handle,m_bas.Nrf_temp_level_handles.value_handle, &gatts_value);
						gatts_value.p_value = (uint8_t *)adv_name;

						err_code=sd_ble_gatts_value_set(m_bas.conn_handle,m_bas.Nrf_name_handles.value_handle, &gatts_value);
//						application_timers_start();

            break;

        case BLE_EVT_TX_COMPLETE:
            char_notify();
						break;

        case BLE_GAP_EVT_DISCONNECTED:
//					spi_flash_pages_count=0;
						nrf_gpio_pin_clear(LED_PIN);						//LED off.
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            application_timers_stop();
 						advertising_start();
            
            // Go to system-off mode            
//            err_code = sd_power_system_off();
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0
																				,BLE_GATTS_SYS_ATTR_FLAG_SYS_SRVCS | BLE_GATTS_SYS_ATTR_FLAG_USR_SRVCS);
            break;

        case BLE_GAP_EVT_TIMEOUT:
            if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_ADVERTISING)
            { 
								advertising_start();
                // Go to system-off mode (this function will not return; wakeup will cause a reset).
//                err_code = sd_power_system_off(); 
            }
            break;
            
        case BLE_GATTS_EVT_WRITE:
            on_write(p_ble_evt);
            break;



        case BLE_GATTC_EVT_TIMEOUT:
        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server and Client timeout events.
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;
        default:
            break;
    }
    
    APP_ERROR_CHECK(err_code);

}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the BLE Stack event interrupt handler after a BLE stack
 *          event has been received.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
//    dm_ble_evt_handler(p_ble_evt);
//    ble_hrs_on_ble_evt(&m_hrs, p_ble_evt);
//    ble_bas_on_ble_evt(&m_bas, p_ble_evt);
//    ble_conn_params_on_ble_evt(p_ble_evt);
#ifdef BLE_DFU_APP_SUPPORT    
    /** @snippet [Propagating BLE Stack events to DFU Service] */
    ble_dfu_on_ble_evt(&m_dfus, p_ble_evt);
    /** @snippet [Propagating BLE Stack events to DFU Service] */
#endif // BLE_DFU_APP_SUPPORT    

    on_ble_evt(p_ble_evt);
}


/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in]   sys_evt   System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
    pstorage_sys_event_handler(sys_evt);

    switch(sys_evt)
    {
        case NRF_EVT_FLASH_OPERATION_SUCCESS:
						break;
        case NRF_EVT_FLASH_OPERATION_ERROR:

            if (m_memory_access_in_progress)
            {
                m_memory_access_in_progress = false;
                advertising_start();
            }
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;
    // Initialize the SoftDevice handler module.
//#ifdef BEACON
//    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_RC_250_PPM_250MS_CALIBRATION, false);
//#else
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, NULL);
//#endif
    // Enable BLE stack 
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));
    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
    
    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}



//static void dm_pstorage_cb_handler(pstorage_handle_t * p_handle,
//                                   uint8_t             op_code,
//                                   uint32_t            result,
//                                   uint8_t           * p_data,
//                                   uint32_t            data_len)
//{
//}

/**@brief Function for application main entry.
 */
/**@brief Function for the Event Scheduler initialization.
 */
static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}

int main(void)
{
//		timer_counter = 0x007DCEA4;
    uint32_t err_code = NRF_SUCCESS;
		NRF_GPIO->PIN_CNF[LED_PIN] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
                                            | (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
                                            | (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)
                                            | (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)
                                            | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
		nrf_gpio_pin_set (LED_PIN);												//Led on.
		pstorage_module_param_t		 param;
//		sd_power_dcdc_mode_set  ( NRF_POWER_DCDC_ENABLE);
//			NRF_GPIO->PIN_CNF[UART_POWER] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
//                                            | (GPIO_PIN_CNF_DRIVE_H0S1 << GPIO_PIN_CNF_DRIVE_Pos)
//                                            | (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)
//                                            | (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)
//                                            | (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos);
//    nrf_gpio_cfg_input(UART_POWER, NRF_GPIO_PIN_PULLUP);

//#endif
		NRF_GPIO->PIN_CNF[UART_POWER] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
                                            | (GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos)
                                            | (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)
                                            | (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)
                                            | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
//						nrf_gpio_pin_clear (UART_POWER);


    timers_init();
    ble_stack_init();
    scheduler_init();    

		pstorage_init();
		param.block_size  = PSTORAGE_PAGE_SIZE;                   //Select block size of 16 bytes
		param.block_count = PSTORAGE_MAX_APPLICATIONS;           	//Select 10 blocks, total of 160 bytes
		param.cb          = example_cb_handler;   								//Set the pstorage callback handler
		err_code = pstorage_register(&param, &flash_base_handle);
    gap_params_init();
    sd_ble_gap_tx_power_set(4);
		service_add();
		connectable_adv_init();
//    advertising_data_init();
		{
			APP_ERROR_CHECK_BOOL(sizeof(flags) == ADV_FLAGS_LEN);  // Assert that these two values of the same.
			// Build and set advertising data
			memset(&advdata, 0, sizeof(advdata));
//			*( (uint32_t *) (m_addl_adv_manuf_data+16)) = *(uint32_t *) adv_name;
			m_addl_adv_manuf_data[16] = adv_name[0];
			m_addl_adv_manuf_data[17] = adv_name[1];
			m_addl_adv_manuf_data[18] = adv_name[2];
			m_addl_adv_manuf_data[19] = adv_name[3];
			manuf_data.company_identifier = COMPANY_IDENTIFIER;
			manuf_data.data.size          = ADV_ADDL_MANUF_DATA_LEN;
			manuf_data.data.p_data        = m_addl_adv_manuf_data;
//			advdata.name_type							= BLE_ADVDATA_SHORT_NAME;
//			advdata.short_name_len				= BLE_ADVDATA_SHORT_NAME_LEN;
			advdata.flags.size            = sizeof(flags);
			advdata.flags.p_data          = &flags;
			advdata.p_manuf_specific_data = &manuf_data;
			err_code = ble_advdata_set(&advdata, NULL);
			APP_ERROR_CHECK(err_code);
			}
		advertising_start();
//    err_code = app_timer_start(m_s5tm_timer_id,  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER), NULL);
    err_code = app_timer_start(m_timecounter_id,  APP_TIMER_TICKS(TIME_STEP*1000, APP_TIMER_PRESCALER), NULL);

		nrf_gpio_pin_clear (LED_PIN);												//Led off.
		s5tm_timeout_handler(NULL);
		for(;;)	{
      app_sched_execute();
			power_manage();
		}
}

/** 
 * @}
 */
