/* Copyright (c) 2013 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @defgroup ble_sdk_app_beacon_main main.c
 * @{
 * @ingroup ble_sdk_app_beacon
 * @brief Beacon Transmitter Sample Application main file.
 *
 * This file contains the source code for an Beacon transmitter sample application.
 */

#include <stdbool.h>
#include <stdint.h>
#include "ble_advdata.h"
#include "boards.h"
#include "nordic_common.h"
#include "softdevice_handler.h"
#include "ble_debug_assert_handler.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "ble_radio_notification.h"
#include "cc110l.h"
#include "spi_master_mnhs.h"

//Changeable content of packets
uint8_t __SOIL_HUMIDITY;
uint8_t PACKET_INCR;
uint16_t __SUPERCAP_VOLTAGE = 0;


#define SENSORS_ON 15					

#define GPIO2 5
#define GPIO3 4

#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                                 /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/
#define APP_CFG_NON_CONN_ADV_TIMEOUT    0                                 /**< Time for which the device must be advertising in non-connectable mode (in seconds). 0 disables timeout. */
#define NON_CONNECTABLE_ADV_INTERVAL    MSEC_TO_UNITS(8000, UNIT_0_625_MS) /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */

#define APP_BEACON_INFO_LENGTH          14                                /**< Total length of information advertised by the Beacon. */
#define APP_ADV_DATA_LENGTH             0x15                              /**< Length of manufacturer specific data in the advertisement. */
#define APP_MEASURED_RSSI               0xC3                              /**< The Beacon's measured RSSI at 1 meter distance in dBm. */
#define APP_COMPANY_IDENTIFIER          0x0000                            /**< IMPORTANT - company identifier as per www.bluetooth.org, 0x0000 added just for reference. */

#define DEAD_BEEF                       0xDEADBEEF                        /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

//Definitions for handling ADC calculations
#define ADC_REF_VOLTAGE_IN_MILLIVOLTS 1250
#define ADC_PRE_SCALING_COMPENSATION 1
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE)\
(((((ADC_VALUE) * ADC_REF_VOLTAGE_IN_MILLIVOLTS) / 255) * ADC_PRE_SCALING_COMPENSATION)*100/20)

//Non-changeable parameters of the packets
#define DEVICE_UUID 0x20,0x20,0x20,0x03
#define LASTPACKET_INCR_DEFINE 0
#define DEVICE_TYPE_DEFINE 89
#define DEVICE_DATA 0x00,0x00,0x00,0x00,0x00,0x00
#define SUPERCAP_VOLTAGE 0x00,0x00

#define _SUPERCAP_MEASURE 1
#define _SOILHUMIDITY_MEASURE 2

uint8_t _ADC_measurement = 0;

static ble_gap_adv_params_t m_adv_params;                               /**< Parameters to be passed to the stack when starting advertising. */
static uint8_t m_beacon_info[APP_BEACON_INFO_LENGTH] =                  /**< Information advertised by the 2.4 Ghz BLE Beacon. */
{
    DEVICE_UUID,            // UUID of the sensor 
    LASTPACKET_INCR_DEFINE, // Used to determine if any packet were lost during transmission 
    DEVICE_TYPE_DEFINE,     // Type of sensor (see documentation for more) 
    DEVICE_DATA,            // Custom sensor data - maximum of 6 bytes 
    SUPERCAP_VOLTAGE        // Supercapacitor voltage in millivolts - 2 bytes 
};

uint8_t CC110L_PACKETCONTENT[16] = {0x7F,0x07,DEVICE_UUID,LASTPACKET_INCR_DEFINE,DEVICE_TYPE_DEFINE,DEVICE_DATA,SUPERCAP_VOLTAGE};


/** Watchdog initialization.
 * Triggered on pin P0.00 change
 */
void wdt_init(void)
{
	NRF_WDT->CONFIG = (WDT_CONFIG_HALT_Pause << WDT_CONFIG_HALT_Pos) | ( WDT_CONFIG_SLEEP_Run << WDT_CONFIG_SLEEP_Pos);   //Configure Watchdog. a) Pause watchdog while the CPU is halted by the debugger.  b) Keep the watchdog running while the CPU is sleeping.
	NRF_WDT->CRV = 10*32768;             //ca 3 sek. timout
	NRF_WDT->RREN |= WDT_RREN_RR0_Msk;  //Enable reload register 0
	NRF_WDT->TASKS_START = 1;           //Start the Watchdog timer
}

//ADC interrupt handler
void ADC_IRQHandler(void)
{
 if (NRF_ADC->EVENTS_END != 0)
 {
  uint8_t adc_result;
  NRF_ADC->EVENTS_END = 0;
  adc_result = NRF_ADC->RESULT;
  NRF_ADC->TASKS_STOP = 1;
	 if(_ADC_measurement==_SUPERCAP_MEASURE)
	 {
	  __SUPERCAP_VOLTAGE = ADC_RESULT_IN_MILLI_VOLTS(adc_result);
   }
	 else
	 {
	  __SOIL_HUMIDITY = adc_result;
	  nrf_gpio_pin_clear(GPIO2);
	 }
 }
}

//Start supercapacitor voltage measurement
void supercap_measure_start(void)
{
	// Configure ADC
	/* Enable interrupt on ADC sample ready event*/		
	NRF_ADC->INTENSET = ADC_INTENSET_END_Msk;   
	sd_nvic_SetPriority(ADC_IRQn, NRF_APP_PRIORITY_LOW);  
	sd_nvic_EnableIRQ(ADC_IRQn);
	
	NRF_ADC->CONFIG	= (ADC_CONFIG_EXTREFSEL_None << ADC_CONFIG_EXTREFSEL_Pos) /* Bits 17..16 : ADC external reference pin selection. */
									| (ADC_CONFIG_PSEL_AnalogInput7 << ADC_CONFIG_PSEL_Pos)					/*!< Use analog input 2 as analog input. */
									| (ADC_CONFIG_REFSEL_VBG << ADC_CONFIG_REFSEL_Pos)							/*!< Use internal 1.2V bandgap voltage as reference for conversion. */
									| (ADC_CONFIG_INPSEL_AnalogInputNoPrescaling << ADC_CONFIG_INPSEL_Pos) /*!< Analog input specified by PSEL with no prescaling used as input for the conversion. */
									| (ADC_CONFIG_RES_8bit << ADC_CONFIG_RES_Pos);									/*!< 8bit ADC resolution. */ 
  sd_nvic_ClearPendingIRQ(ADC_IRQn);
	NRF_ADC->EVENTS_END = 0;	
	/* Enable ADC*/
	NRF_ADC->ENABLE = ADC_ENABLE_ENABLE_Enabled;
	NRF_ADC->TASKS_START = 1;
}

//Start supercapacitor voltage measurement
void soilhumidity_measure_start(void)
{
	// Configure ADC
	/* Enable interrupt on ADC sample ready event*/		
	NRF_ADC->INTENSET = ADC_INTENSET_END_Msk;   
	sd_nvic_SetPriority(ADC_IRQn, NRF_APP_PRIORITY_LOW);  
	sd_nvic_EnableIRQ(ADC_IRQn);
	
	NRF_ADC->CONFIG	= (ADC_CONFIG_EXTREFSEL_None << ADC_CONFIG_EXTREFSEL_Pos) /* Bits 17..16 : ADC external reference pin selection. */
									| (ADC_CONFIG_PSEL_AnalogInput5 << ADC_CONFIG_PSEL_Pos)					/*!< Use analog input 2 as analog input. */
									| (ADC_CONFIG_REFSEL_VBG << ADC_CONFIG_REFSEL_Pos)							/*!< Use internal 1.2V bandgap voltage as reference for conversion. */
									| (ADC_CONFIG_INPSEL_AnalogInputNoPrescaling << ADC_CONFIG_INPSEL_Pos) /*!< Analog input specified by PSEL with no prescaling used as input for the conversion. */
									| (ADC_CONFIG_RES_8bit << ADC_CONFIG_RES_Pos);									/*!< 8bit ADC resolution. */ 
  sd_nvic_ClearPendingIRQ(ADC_IRQn);
	NRF_ADC->EVENTS_END = 0;	
	/* Enable ADC*/
	NRF_ADC->ENABLE = ADC_ENABLE_ENABLE_Enabled;
	NRF_ADC->TASKS_START = 1;
}

/**@brief Function for error handling, which is called when an error has occurred.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of error.
 *
 * @param[in] error_code  Error code supplied to the handler.
 * @param[in] line_num    Line number where the handler is called.
 * @param[in] p_file_name Pointer to the file name.
 */
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    // This call can be used for debug purposes during application development.
    // @note CAUTION: Activating this code will write the stack to flash on an error.
    //                This function should NOT be used in a final product.
    //                It is intended STRICTLY for development/debugging purposes.
    //                The flash write will happen EVEN if the radio is active, thus interrupting
    //                any communication.
    //                Use with care. Un-comment the line below to use.
    // ble_debug_assert_handler(error_code, line_num, p_file_name);

    // On assert, the system can only recover on reset.
    NVIC_SystemReset();
}


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


ble_advdata_t   advdata;
ble_advdata_manuf_data_t manuf_specific_data;
/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t        err_code;
    
    uint8_t         flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;

    
    manuf_specific_data.company_identifier = APP_COMPANY_IDENTIFIER;

    manuf_specific_data.data.p_data        = (uint8_t *) m_beacon_info;
    manuf_specific_data.data.size          = APP_BEACON_INFO_LENGTH;

    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type               = BLE_ADVDATA_NO_NAME;
    advdata.flags.size              = sizeof(flags);
    advdata.flags.p_data            = &flags;
    advdata.p_manuf_specific_data   = &manuf_specific_data;

    err_code = ble_advdata_set(&advdata, NULL);
    APP_ERROR_CHECK(err_code);

    // Initialize advertising parameters (used when starting advertising).
    memset(&m_adv_params, 0, sizeof(m_adv_params));

    m_adv_params.type        = BLE_GAP_ADV_TYPE_ADV_NONCONN_IND;
    m_adv_params.p_peer_addr = NULL;                             // Undirected advertisement.
    m_adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval    = NON_CONNECTABLE_ADV_INTERVAL;
    m_adv_params.timeout     = APP_CFG_NON_CONN_ADV_TIMEOUT;
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t err_code;

    err_code = sd_ble_gap_adv_start(&m_adv_params);
    APP_ERROR_CHECK(err_code);

}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, false);

    // Enable BLE stack 
    uint32_t err_code;
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));
    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for doing power management.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

uint8_t cnt = 0;
uint8_t incr_cnt = 0;

void SensorsInit()
{
  	
}

void PacketData_Update()
{
        uint8_t         flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;
			  PACKET_INCR++; //Incrementing the value
	      
	      incr_cnt++;
	      if(incr_cnt==5)
				{
					incr_cnt = 0;
					_ADC_measurement = _SOILHUMIDITY_MEASURE;
	        nrf_gpio_pin_set(GPIO2);
					nrf_delay_ms(2);
					soilhumidity_measure_start();
				}
        else
				{
					_ADC_measurement = _SUPERCAP_MEASURE;
					supercap_measure_start();
        }      	
				m_beacon_info[6]  = __SOIL_HUMIDITY; //Putting data into 2.4 Ghz beacon packet and update advertising data
	      m_beacon_info[12] = __SUPERCAP_VOLTAGE>>8;
	      m_beacon_info[13] = __SUPERCAP_VOLTAGE&0xFF;
    
     	  manuf_specific_data.data.p_data        = (uint8_t *) m_beacon_info;

        // Build and set advertising data.
        memset(&advdata, 0, sizeof(advdata));

        advdata.flags.size              = sizeof(flags);
        advdata.flags.p_data            = &flags;
        advdata.p_manuf_specific_data   = &manuf_specific_data;

        ble_advdata_set(&advdata, NULL);
				
				CC110L_PACKETCONTENT[2+6]  = __SOIL_HUMIDITY;   //Putting data into 868 Mhz packet
	      CC110L_PACKETCONTENT[2+12] = __SUPERCAP_VOLTAGE>>8;
	      CC110L_PACKETCONTENT[2+13] = __SUPERCAP_VOLTAGE&0xFF;
}

uint8_t packet868_i = 0;

void radio_notification_evt_handler(bool radio_evt)
{
 if(radio_evt)
 {
	NRF_WDT->RR[0] = WDT_RR_RR_Reload; 
  PacketData_Update();
	packet868_i++;
	if(packet868_i==8)
	{
   packet868_i = 0;		
   CC110L_RFSendPacket(CC110L_PACKETCONTENT, 16); //Sending the packet via 868 Mhz link
  }
 }
}

uint32_t spi_base_addr;

/**
 * @brief Function for application main entry.
 */
int main(void)
{
	 // wdt_init();
    nrf_gpio_cfg_output(SENSORS_ON);
    nrf_gpio_pin_set(SENSORS_ON);  
	  nrf_gpio_cfg_output(GPIO2);
	  nrf_gpio_pin_clear(GPIO2);
	
		ble_stack_init(); // Initialize Bluetooth stack
    spi_base_addr = (uint32_t)spi_master_init(SPI0, SPI_MODE0, false);
	  CC110L_setupspibaseaddr(spi_base_addr);
	  setup_cc110l();
   	
	  advertising_init();
    // Start execution.
	  PacketData_Update();
    advertising_start();
	  ble_radio_notification_init(3, NRF_RADIO_NOTIFICATION_DISTANCE_5500US, radio_notification_evt_handler);

    // Enter main loop.
    for (;;)
    {
        power_manage();
    }
}

/**
 * @}
 */
