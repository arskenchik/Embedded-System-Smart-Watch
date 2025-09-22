#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

#include "nrf.h"
#include "app_error.h"
#include "app_scheduler.h"
#include "app_timer.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_twi.h"
#include "nrf_twi_mngr.h"
#include "nrf_twi_sensor.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_drv_clock.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "ble.h"
#include "ble_cts_c.h"
#include "ble_db_discovery.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_conn_state.h"
#include "ble_gattc.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "fds.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"

#include "boards.h"
#include "bsp_btn_ble.h"
#include "app_util_platform.h"

#include <stdio.h>

/* ===== BLE Settings ===== */
#define DEVICE_NAME                     "EE414"                       /**< Name that will appear in advertising. */
#define APP_BLE_OBS_PRIORITY           3                             /**< BLE observer priority. */
#define APP_BLE_TAG            1                             /**< SoftDevice BLE configuration tag. */

#define MIN_INTERVAL               MSEC_TO_UNITS(500, UNIT_1_25_MS)   /**< Minimum acceptable connection interval (0.5 s). */
#define MAX_INTERVAL               MSEC_TO_UNITS(1000, UNIT_1_25_MS)  /**< Maximum acceptable connection interval (1 s). */
#define SLAVE_LATENCY                   0                                     /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)      /**< Connection supervisory timeout (4 s). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                /**< Delay before first conn params update attempt (5 s). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)               /**< Delay between successive conn params updates (30 s). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                    /**< Number of conn params update attempts. */

#define NRF_BLE_GQ_QUEUE_SIZE           4     /**< GATT Queue size. */
#define NRF_SDH_BLE_GAP_CONN_COUNT      1     /**< How many simultaneous connections (central/peripheral) we support. */

/* ===== I2C Settings ===== */
#define SENSOR_TWI_INSTANCE_ID          0

/* ===== Accelerometer / Step Counter ===== */
#define ACC_CTRL_REG                        0x20      /**< Accelerometer control register. */
#define BUF_SIZE                        20        /**< Number of samples used for calibration & step detection. */
#define THRESHOLD_TO_DETECT                  5.5f      /**< Threshold (in m/s²) to detect a step. */
#define ACC_ADDR                        0x6B      /**< I²C address of the accelerometer. */
#define ACC_STATUS                      0x17      /**< Status register (data-ready bit). */

/* ===== OLED Settings ===== */
#define OLED_ADDR                       0x3C
#define OLED_TWI_INSTANCE_ID            1
#define OLED_SCL_PIN                    18
#define OLED_SDA_PIN                    20
#define OLED_DATA_MODE                  0x40
#define OLED_CMD_MODE                   0x00

#define OLED_CMD_DISP_OFF               0xAE
#define OLED_CMD_DISP_ON                0xAF
#define OLED_CMD_MEM_ADDR_MODE          0x20
#define OLED_CMD_SET_COL_ADDR           0x21
#define OLED_CMD_SET_PAGE_ADDR          0x22
#define OLED_CMD_SET_START_LINE         0x40
#define OLED_CMD_SET_CONTRAST           0x81
#define OLED_CMD_CHARGE_PUMP            0x8D

#define OLED_WIDTH                      128
#define OLED_HEIGHT                     64
#define OLED_PAGE_COUNT                 8
#define OLED_BUFFER_SIZE                (OLED_WIDTH * OLED_PAGE_COUNT)

/* ===== Power Pins ===== */
#define PIN_VDD_ENV                     22 // 32*0+22
#define PIN_R_PULLUP                    32 // 32*1+0

/* ===== APDS-9960 (Gesture/Proximity) ===== */
#define APDS9960_I2C_ADDR               0x39
#define APDS9960_ENABLE                 0x80

#define MAX_PENDING_TRANSACTIONS        20

/* ===== APDS-9960 Gesture Registers ===== */
#define APDS9960_GSTATUS       0xAF  
#define APDS9960_GFLVL         0xAE
#define APDS9960_GFIFO_U       0xFC
#define APDS9960_GFIFO_D       0xFD
#define APDS9960_GFIFO_L       0xFE
#define APDS9960_GFIFO_R       0xFF
#define APDS9960_GPENTH        0xA0
#define APDS9960_GEXTH         0xA1
#define APDS9960_GCONF1        0xA2
#define APDS9960_GCONF2        0xA3
#define APDS9960_GCONF3        0xAA
#define APDS9960_GCONF4        0xAB
#define APDS9960_GPULSE        0xA6
#define APDS9960_ID            0x92

#define BIT_PON                (1 << 0)  
#define AIEN                   (1 << 1)  
#define PEN                    (1 << 2)  
#define PIEN                   (1 << 5)                  
#define BIT_GEN                (1 << 6)  
#define GVALID                 (1 << 0)  

/* ===== Gesture to LED Mapping ===== */
#define LED_RED                24  // For UP
#define LED_BLUE               6   // For DOWN
#define LED_GREEN              16  // For LEFT
// For RIGHT, we turn on all three (R+G+B)

/* ===== TWI Instance ===== */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(SENSOR_TWI_INSTANCE_ID);
static volatile bool m_xfer_done = false;

/* ===== Gesture State ===== */
enum {
  GESTURE_NONE = -1,
  GESTURE_UP = 0,
  GESTURE_DOWN = 1,
  GESTURE_LEFT = 2,
  GESTURE_RIGHT = 3
};

static bool _gestureEnabled = false;
static bool _gestureIn = false;
static int  _gestureDirectionX = 0;
static int  _gestureDirectionY = 0;
static int  _gestureDirInX = 0;
static int  _gestureDirInY = 0;
static int  _gestureSensitivity = 30; 
static int  _detectedGesture = GESTURE_NONE;

/* ===== Display Modes ===== */
typedef enum {
    DISPLAY_LOGO,
    DISPLAY_CLOCK,
    DISPLAY_INVERTED_LOGO,
    DISPLAY_STEPS
} display_mode_t;

static display_mode_t current_mode = DISPLAY_CLOCK;
static bool display_inverted = false;

/* ===== Timing Variables ===== */
static uint8_t sec = 0;
static uint8_t min = 0;
static uint8_t hour = 12;

/* ===== OLED and Step Counter Buffers ===== */
static uint8_t  acc_data[BUF_SIZE * 3];   /**< Raw bytes from accelerometer (X, Y, Z). */
static float    acc_x[BUF_SIZE];          /**< Recent X samples. */
static float    acc_y[BUF_SIZE];          /**< Recent Y samples. */
static float    acc_z[BUF_SIZE];          /**< Recent Z samples. */
static float    xavg, yavg, zavg;          /**< Averages (gravity offset). */
static int      acc_flag = 0;              /**< State‐machine flag for step detection. */
static int      step_count = 0;            /**< Number of steps counted so far. */
char step_str[10];

static volatile bool gesture_processing_active = false;
static volatile uint8_t elapsed_inactive_seconds = 0;
static volatile bool oled_active = true;
static volatile bool time_updated = false;
static volatile uint8_t gesture_timeout_counter = 0;

/* ===== TWI Managers ===== */
NRF_TWI_MNGR_DEF(m_sensor_twi_mngr, MAX_PENDING_TRANSACTIONS, SENSOR_TWI_INSTANCE_ID);
NRF_TWI_MNGR_DEF(m_oled_twi_mngr,  MAX_PENDING_TRANSACTIONS, OLED_TWI_INSTANCE_ID);

NRF_TWI_SENSOR_DEF(twi_APDS9960, &m_sensor_twi_mngr, MAX_PENDING_TRANSACTIONS);
NRF_TWI_SENSOR_DEF(twi_OLED,     &m_oled_twi_mngr,  MAX_PENDING_TRANSACTIONS); 

/* ===== Timer ===== */
APP_TIMER_DEF(m_timer);
const nrf_drv_timer_t timer_inst = NRF_DRV_TIMER_INSTANCE(1);

/* ===== BLE Module Instances ===== */
BLE_CTS_C_DEF(      m_cts_c);          
NRF_BLE_GATT_DEF(   m_gatt);            
NRF_BLE_GQ_DEF(     m_ble_gatt_q,
                    NRF_SDH_BLE_PERIPHERAL_LINK_COUNT,
                    NRF_BLE_GQ_QUEUE_SIZE);  
NRF_BLE_QWR_DEF(    m_qwr);     
BLE_DB_DISCOVERY_DEF(m_db_disc);      
BLE_ADVERTISING_DEF(m_adv);       

static uint16_t m_conn_handle_periph = BLE_CONN_HANDLE_INVALID; 
static bool     m_is_advertising     = false;            

/* ===== Peer Manager Globals ===== */
static pm_peer_id_t  m_peer_id;                                         
static pm_peer_id_t  m_whitelist_peers[BLE_GAP_WHITELIST_ADDR_MAX_COUNT]; 
static uint32_t      m_whitelist_peer_cnt;                            

/* ===== Security Parameters ===== */
#define SEC_PARAM_BOND                 1   
#define SEC_PARAM_MITM                 0     
#define SEC_PARAM_LESC                 0      
#define SEC_PARAM_KEYPRESS             0     
#define SEC_PARAM_IO_CAPABILITIES      BLE_GAP_IO_CAPS_NONE  
#define SEC_PARAM_OOB                  0    
#define SEC_PARAM_MIN_KEY_SIZE         7     
#define SEC_PARAM_MAX_KEY_SIZE         16      

/* ===== Scheduler Settings ===== */
#define SCHED_MAX_EVENT_DATA_SIZE      APP_TIMER_SCHED_EVENT_DATA_SIZE
#ifdef SVCALL_AS_NORMAL_FUNCTION
#define SCHED_QUEUE_SIZE               20
#else
#define SCHED_QUEUE_SIZE               10
#endif

uint8_t first_buf[15][14] = {{0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x20, 0x20, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00}, //0
                                      {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
                                      {0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x20, 0xa0, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00},
                                      {0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x20, 0xa0, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00},
                                      {0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x40, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
                                      {0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0xa0, 0xa0, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00},
                                      {0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0xa0, 0xa0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, //6
                                      {0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x20, 0x20, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00},
                                      {0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0xa0, 0xa0, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00},
                                      {0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0xa0, 0xa0, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00},
                                      {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // :
                                      {0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x40, 0x40, 0x40, 0x80, 0x00, 0x00, 0x00, 0x00},
                                      {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0xF0, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00},
                                      {0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x40, 0x40, 0x40, 0x40, 0x80, 0x00, 0x00, 0x00},
                                      {0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x80, 0x40, 0x40, 0x40, 0x80, 0x00, 0x00, 0x00}
                                      };
uint8_t second_buf[15][14] = {{0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x02, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00},//0
                                       {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x03, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00},
                                       {0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x03, 0x02, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00},
                                       {0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00},
                                       {0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00},
                                       {0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x02, 0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00},
                                       {0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00}, //6
                                       {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
                                       {0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00},
                                       {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00},
                                       {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // :
                                       {0x00, 0x00, 0x00, 0x00, 0x00, 0x09, 0x12, 0x12, 0x12, 0x0C, 0x00, 0x00, 0x00, 0x00},
                                       {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00},
                                       {0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x12, 0x12, 0x12, 0x12, 0x0B, 0x00, 0x00, 0x00},
                                       {0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x08, 0x10, 0x10, 0x10, 0x0F, 0x00, 0x00, 0x00}
                                       };
/* ===== I2C Write/Read Helpers ===== */
bool writeRegister(uint8_t devAddr, uint8_t regAddr, uint8_t data)
{
    ret_code_t err_code;
    uint8_t tx_data[2] = { regAddr, data };

    NRF_LOG_INFO("Writing to reg 0x%02X: data=0x%02X", regAddr, data);
    m_xfer_done = false;  // Clear transfer flag
    err_code = nrf_drv_twi_tx(&m_twi, devAddr, tx_data, sizeof(tx_data), false);
    while (!m_xfer_done) {
    }
    if (err_code != NRF_SUCCESS) {
        NRF_LOG_INFO("writeRegister failed, error: %d", err_code);
        return false;
    }
    NRF_LOG_INFO("Write complete to reg 0x%02X", regAddr);
    return true;
}

bool readRegister(uint8_t devAddr, uint8_t regAddr, uint8_t *data)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Reading from reg 0x%02X", regAddr);
    m_xfer_done = false;  // Clear transfer flag for TX
    err_code = nrf_drv_twi_tx(&m_twi, devAddr, &regAddr, 1, true);
    while (!m_xfer_done) {
    }
    if (err_code != NRF_SUCCESS) {
        NRF_LOG_INFO("readRegister TX failed, error: %d", err_code);
        return false;
    }
    NRF_LOG_INFO("TX of reg address complete");

    m_xfer_done = false;  // Clear flag for RX
    err_code = nrf_drv_twi_rx(&m_twi, devAddr, data, 1);
    while (!m_xfer_done) {
    }
    if (err_code != NRF_SUCCESS) {
        NRF_LOG_INFO("readRegister RX failed, error: %d", err_code);
        return false;
    }
    NRF_LOG_INFO("Read complete from reg 0x%02X: data=0x%02X", regAddr, *data);
    return true;
}

bool readRegisters(uint8_t devAddr, uint8_t regAddr, uint8_t *data, size_t length)
{
    ret_code_t err_code;
    
    NRF_LOG_INFO("readRegisters: Requesting %d bytes from reg 0x%02X", (int)length, regAddr);
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&m_twi, devAddr, &regAddr, 1, true);
    while (!m_xfer_done) {
    }
    if (err_code != NRF_SUCCESS) {
        NRF_LOG_INFO("readRegisters TX failed, error: %d", err_code);
        return false;
    }
    NRF_LOG_INFO("readRegisters: TX successful");

    m_xfer_done = false;
    err_code = nrf_drv_twi_rx(&m_twi, devAddr, data, length);
    while (!m_xfer_done) {
    }
    if (err_code != NRF_SUCCESS) {
        NRF_LOG_INFO("readRegisters RX failed, error: %d", err_code);
        return false;
    }
    NRF_LOG_INFO("readRegisters: RX successful. Data:");
    for (size_t i = 0; i < length; i++) {
        NRF_LOG_INFO("  Byte %d: 0x%02X", (int)i, data[i]);
    }
    return true;
}

/* ===== Gesture Sensor Helpers ===== */
bool getGSTATUS(uint8_t* r)
{
    NRF_LOG_INFO("Getting GSTATUS");
    if (!readRegister(APDS9960_I2C_ADDR, APDS9960_GSTATUS, r)) {
        NRF_LOG_INFO("getGSTATUS failed");
        return false;
    }
    NRF_LOG_INFO("GSTATUS = 0x%02X", *r);
    return true;
}

bool getGFLVL(uint8_t* r)
{
    NRF_LOG_INFO("Getting GFLVL");
    if (!readRegister(APDS9960_I2C_ADDR, APDS9960_GFLVL, r)) {
        NRF_LOG_INFO("getGFLVL failed");
        return false;
    }
    NRF_LOG_INFO("GFLVL = 0x%02X", *r);
    return true;
}

bool getENABLE(uint8_t* r) 
{
    NRF_LOG_INFO("Getting ENABLE");
    if (!readRegister(APDS9960_I2C_ADDR, APDS9960_ENABLE, r)) {
        NRF_LOG_INFO("getENABLE failed");
        return false;
    }
    NRF_LOG_INFO("ENABLE = 0x%02X", *r);
    return true;
}

bool setENABLE(uint8_t r)
{
    NRF_LOG_INFO("Setting ENABLE to 0x%02X", r);
    if (!writeRegister(APDS9960_I2C_ADDR, APDS9960_ENABLE, r)) {
        NRF_LOG_INFO("setENABLE failed");
        return false;
    }
    NRF_LOG_INFO("setENABLE succeeded");
    return true;
}
int readGesture()
{
    int gesture = _detectedGesture;
    _detectedGesture = GESTURE_NONE;
    NRF_LOG_INFO("readGesture: %d", gesture);
    return gesture;
}

int gestureFIFOAvailable() {
    uint8_t r;
    nrf_delay_ms(10);
    NRF_LOG_INFO("GSTATUS = 0x%02X, GFLVL = %d", r, r);

    if (!getGSTATUS(&r)) {
        NRF_LOG_INFO("gestureFIFOAvailable: getGSTATUS failed");
        return -1;
    }
    nrf_delay_ms(10);
    if ((r & GVALID) == 0) {
        NRF_LOG_INFO("gestureFIFOAvailable: No valid FIFO data (GSTATUS=0x%02X)", r);
        return -2;
    }
    nrf_delay_ms(10);
    if (!getGFLVL(&r)) {
        NRF_LOG_INFO("gestureFIFOAvailable: getGFLVL failed");
        return -3;
    }
    NRF_LOG_INFO("gestureFIFOAvailable: %d entries available", r);
    return r;
}

bool enableGesture()
{
    uint8_t r;
    nrf_delay_ms(10);
    if (!getENABLE(&r)) return false;
    nrf_delay_ms(10);

    if ((r & BIT_GEN) != 0) {
        _gestureEnabled = true;
        NRF_LOG_INFO("Gesture already enabled (ENABLE=0x%02X)", r);
        return true;
    }
    r |= BIT_GEN | BIT_PON | PEN;
    nrf_delay_ms(10);
    bool res = setENABLE(r);
    nrf_delay_ms(10);
    _gestureEnabled = res;
    NRF_LOG_INFO("Gesture enable set, result: %d", res);
    return res;
}

int handleGesture() {
    const int gestureThreshold = 30;
    nrf_delay_ms(10);

    int available = gestureFIFOAvailable();
    if (available <= 0) {
        NRF_LOG_INFO("handleGesture: No FIFO data available (available=%d)", available);
        return 0;
    }

    uint8_t fifo_data[128];
    size_t bytes_to_read = available * 4;  // 4 bytes per gesture set (U, D, L, R)
    NRF_LOG_INFO("handleGesture: Attempting to read %d bytes from FIFO", (int)bytes_to_read);

    if (!readRegisters(APDS9960_I2C_ADDR, APDS9960_GFIFO_U, fifo_data, bytes_to_read)) {
        NRF_LOG_INFO("handleGesture: Failed to read gesture FIFO data.");
        return 0;
    }
    nrf_delay_ms(10);
    for (int i = 0; i + 3 < (int)bytes_to_read; i += 4) {
        uint8_t u = fifo_data[i];
        uint8_t d = fifo_data[i + 1];
        uint8_t l = fifo_data[i + 2];
        uint8_t r = fifo_data[i + 3];
        NRF_LOG_INFO("FIFO[%d]: U=0x%02X, D=0x%02X, L=0x%02X, R=0x%02X", i/4, u, d, l, r);

        if ((u < gestureThreshold) && (d < gestureThreshold) &&
            (l < gestureThreshold) && (r < gestureThreshold)) {
            _gestureIn = true;
            if (_gestureDirInX != 0 || _gestureDirInY != 0) {
                int totalX = _gestureDirInX - _gestureDirectionX;
                int totalY = _gestureDirInY - _gestureDirectionY;
                NRF_LOG_INFO("Gesture delta: totalX=%d, totalY=%d", totalX, totalY);

                if (totalX < -_gestureSensitivity) { _detectedGesture = GESTURE_LEFT; }
                if (totalX > _gestureSensitivity) { _detectedGesture = GESTURE_RIGHT; }
                if (totalY < -_gestureSensitivity) { _detectedGesture = GESTURE_DOWN; }
                if (totalY > _gestureSensitivity) { _detectedGesture = GESTURE_UP; }

                NRF_LOG_INFO("Detected gesture: %d", _detectedGesture);
                _gestureDirectionX = 0;
                _gestureDirectionY = 0;
                _gestureDirInX = 0;
                _gestureDirInY = 0;
            }
            continue;
        }

        _gestureDirectionX = r - l;
        _gestureDirectionY = u - d;
        NRF_LOG_INFO("Intermediate gesture directions: X=%d, Y=%d", _gestureDirectionX, _gestureDirectionY);
        if (_gestureIn) {
            _gestureIn = false;
            _gestureDirInX = _gestureDirectionX;
            _gestureDirInY = _gestureDirectionY;
            NRF_LOG_INFO("Setting gesture initial directions: X=%d, Y=%d", _gestureDirInX, _gestureDirInY);
        }
    }
    return 1;
}

/********************************************************
 * APDS9960_init()
 ********************************************************/
bool APDS9960_init(void)
{
    uint8_t r = 0;


    if (!getENABLE(&r)) {
        NRF_LOG_INFO("APDS9960_init: Failed to read ENABLE register.");
        return false;
    }
    NRF_LOG_INFO("APDS9960_init: Initial ENABLE = 0x%02X", r);
    r |= BIT_PON;
    if (!setENABLE(r)) {
        NRF_LOG_INFO("APDS9960_init: Failed to write ENABLE register.");
        return false;
    }
    if (!getENABLE(&r)) {
        NRF_LOG_INFO("APDS9960_init: Failed to re-read ENABLE register.");
        return false;
    }
    NRF_LOG_INFO("APDS9960_init: ENABLE register after PON = 0x%02X", r);

    // Set Gesture Proximity Entry Threshold (GPENTH, 0xA0) to 0x15. <- Key problem was found here.
    // When we set GPENTH to be 0x30 and GEXTH to be 0x10 our sensor couldn't read any gestures
    if (!writeRegister(APDS9960_I2C_ADDR, APDS9960_GPENTH, 0x15)) {
         NRF_LOG_INFO("APDS9960_init: Failed to write GPENTH register.");
         return false;
    }
    NRF_LOG_INFO("APDS9960_init: GPENTH set to 0x15");

    if (!writeRegister(APDS9960_I2C_ADDR, APDS9960_GEXTH, 0x05)) {
         NRF_LOG_INFO("APDS9960_init: Failed to write GEXTH register.");
         return false;
    }
    NRF_LOG_INFO("APDS9960_init: GEXTH set to 0x05");

    if (!writeRegister(APDS9960_I2C_ADDR, APDS9960_GCONF1, 0x40)) {
         NRF_LOG_INFO("APDS9960_init: Failed to write GCONF1 register.");
         return false;
    }
    NRF_LOG_INFO("APDS9960_init: GCONF1 set to 0x40");

    if (!writeRegister(APDS9960_I2C_ADDR, APDS9960_GCONF2, 0x41)) {
         NRF_LOG_INFO("APDS9960_init: Failed to write GCONF2 register.");
         return false;
    }
    NRF_LOG_INFO("APDS9960_init: GCONF2 set to 0x41");

    if (!writeRegister(APDS9960_I2C_ADDR, APDS9960_GPULSE, 0x8F)) {
         NRF_LOG_INFO("APDS9960_init: Failed to write GPULSE register.");
         return false;
    }
    NRF_LOG_INFO("APDS9960_init: GPULSE set to 0x8F");

    if (!writeRegister(APDS9960_I2C_ADDR, APDS9960_GCONF4, 0x03)) {
         NRF_LOG_INFO("APDS9960_init: Failed to write GCONF4 register.");
         return false;
    }
    NRF_LOG_INFO("APDS9960_init: GCONF4 set to 0x03");


    return true;
}

/********************************************************
 * LED Setup & Control
 ********************************************************/
static void leds_init(void)
{
    NRF_LOG_INFO("Initializing LEDs");
    nrf_gpio_cfg_output(LED_RED);
    nrf_gpio_cfg_output(LED_GREEN);
    nrf_gpio_cfg_output(LED_BLUE);

    nrf_gpio_pin_clear(LED_RED);
    nrf_gpio_pin_clear(LED_GREEN);
    nrf_gpio_pin_clear(LED_BLUE);

    nrf_gpio_pin_set(LED_RED);
    nrf_gpio_pin_set(LED_GREEN);
    nrf_gpio_pin_set(LED_BLUE);
}

static void setLEDColor(int gesture)
{
    nrf_gpio_pin_set(LED_RED);
    nrf_gpio_pin_set(LED_GREEN);
    nrf_gpio_pin_set(LED_BLUE);
    switch (gesture)
    {
        case GESTURE_UP:
            nrf_gpio_pin_clear(LED_RED);
            NRF_LOG_INFO("LED => RED (UP)");
            nrf_delay_ms(1000);
            nrf_gpio_pin_set(LED_RED);
            break;

        case GESTURE_DOWN:
            nrf_gpio_pin_clear(LED_BLUE);
            NRF_LOG_INFO("LED => BLUE (DOWN)");
            nrf_delay_ms(1000);
            nrf_gpio_pin_set(LED_BLUE);
            break;

        case GESTURE_LEFT:
            nrf_gpio_pin_clear(LED_GREEN);
            NRF_LOG_INFO("LED => GREEN (LEFT)");
            nrf_delay_ms(1000);
            nrf_gpio_pin_set(LED_GREEN);
            break;

        case GESTURE_RIGHT:
            nrf_gpio_pin_clear(LED_RED);
            nrf_gpio_pin_clear(LED_GREEN);
            nrf_gpio_pin_clear(LED_BLUE);
            NRF_LOG_INFO("LED => WHITE (RIGHT)");
            nrf_delay_ms(1000);
            nrf_gpio_pin_set(LED_RED);
            nrf_gpio_pin_set(LED_GREEN);
            nrf_gpio_pin_set(LED_BLUE);
            break;

        default:
            NRF_LOG_INFO("LED => OFF (No Gesture)");
            break;
    }
}

/* ===== Accelerometer Helpers ===== */
static bool accelerometerAvailable(void)
{
    uint8_t status = 0;
    /* Read STATUS register from LSM */
    if (!readRegister(ACC_ADDR, ACC_STATUS, &status)) {
        return false;
    }
    return (status & 0x01) != 0;  // ZYXDA bit (data ready)
}
static void sampleAccelerometer(float *x, float *y, float *z)
{
    if (!accelerometerAvailable()) {
        return;
    }

    /* Read 6 bytes starting at 0x28 (OUT_X_L) with auto-increment */
    uint8_t sub_address = 0x28 | 0x80; // auto-increment bit
    if (!readRegisters(ACC_ADDR, sub_address, acc_data, 6)) {
        return;
    }

    int16_t raw_x = (int16_t)((acc_data[1] << 8) | acc_data[0]);
    int16_t raw_y = (int16_t)((acc_data[3] << 8) | acc_data[2]);
    int16_t raw_z = (int16_t)((acc_data[5] << 8) | acc_data[4]);

    if (raw_x > 32767) raw_x -= 65536;
    if (raw_y > 32767) raw_y -= 65536;
    if (raw_z > 32767) raw_z -= 65536;

    /* Full‐scale ±2g → raw / 32768 * 2g */
    *x = ((float)raw_x * 2.0f) / 32768.0f;
    *y = ((float)raw_y * 2.0f) / 32768.0f;
    *z = ((float)raw_z * 2.0f) / 32768.0f;
}
static void initAccelerometer(void)
{
    uint8_t config = 0x57;
    (void)writeRegister(ACC_ADDR, ACC_CTRL_REG, config);
    nrf_delay_ms(10);
}
static void calibrateAccelerometer(void)
{
    float xsum = 0, ysum = 0, zsum = 0;
    for (int i = 0; i < BUF_SIZE; i++)
    {
        sampleAccelerometer(&acc_x[i], &acc_y[i], &acc_z[i]);
        xsum += acc_x[i];
        ysum += acc_y[i];
        zsum += acc_z[i];
        nrf_delay_ms(10);
    }
    xavg = xsum / (float)BUF_SIZE;
    yavg = ysum / (float)BUF_SIZE;
    zavg = zsum / (float)BUF_SIZE;
}

static void computeStepCount(void)
{
    float x_window[BUF_SIZE];
    float y_window[BUF_SIZE];
    float z_window[BUF_SIZE];
    float total_acc[BUF_SIZE];
    float avg_acc[BUF_SIZE];

    for (int i = 0; i < BUF_SIZE; i++)
    {
        sampleAccelerometer(&x_window[i], &y_window[i], &z_window[i]);

        /* Compute magnitude minus gravity */
        float mag = sqrtf(x_window[i]*x_window[i]
                        + y_window[i]*y_window[i]
                        + z_window[i]*z_window[i]);
        float grav = sqrtf(xavg*xavg + yavg*yavg + zavg*zavg);
        total_acc[i] = mag - grav;

        /* Simple smoothing */
        if (i == 0) {
            avg_acc[i] = total_acc[i];
        } else {
            avg_acc[i] = (total_acc[i] + total_acc[i-1]) / 2.0f;
        }

        /* Convert to m/s² */
        avg_acc[i] *= 9.8f;

        /* Rising‐edge detection over threshold */
        if ((avg_acc[i] > THRESHOLD_TO_DETECT) && (acc_flag == 0)) {
            step_count++;
            acc_flag = 1;
        } else if ((avg_acc[i] < THRESHOLD_TO_DETECT) && (acc_flag == 1)) {
            acc_flag = 0;
        }

        nrf_delay_ms(10);
    }
}

/* ===== OLED I2C Helpers ===== */
void OLED_write_cmd(uint8_t cmd) {
    nrf_twi_sensor_reg_write(
        &twi_OLED,
        OLED_ADDR,
        OLED_CMD_MODE,
        &cmd,
        1
    );
}

ret_code_t OLED_write_data(uint8_t *data, uint16_t length) {
    ret_code_t err_code = nrf_twi_sensor_reg_write(
        &twi_OLED,
        OLED_ADDR,
        OLED_DATA_MODE,
        &data[0],
        length
    );

    nrf_delay_ms(1);
    return err_code;
}

const uint8_t zeros[1024] = {0};

void paste_zero(void) {
    OLED_write_cmd(OLED_CMD_MEM_ADDR_MODE);          
    OLED_write_cmd(0x00);          

    OLED_write_cmd(OLED_CMD_SET_COL_ADDR);          
    OLED_write_cmd(0x00);          
    OLED_write_cmd(OLED_WIDTH - 1); 
    
    OLED_write_cmd(OLED_CMD_SET_PAGE_ADDR);          
    OLED_write_cmd(0x00);          
    OLED_write_cmd(OLED_PAGE_COUNT - 1); 

    const size_t CHUNK_SIZE = 64; 
    uint16_t bytes_sent = 0;
    
    while(bytes_sent < OLED_BUFFER_SIZE) {
        uint16_t remaining = OLED_BUFFER_SIZE - bytes_sent;
        uint16_t send_size = (remaining > CHUNK_SIZE) ? CHUNK_SIZE : remaining;
        
        OLED_write_data((uint8_t*)(zeros + bytes_sent), send_size);
        bytes_sent += send_size;
        nrf_delay_us(50);
    }

    OLED_write_cmd(OLED_CMD_MEM_ADDR_MODE);          
    OLED_write_cmd(0x02);         
}

/* Bitmap of logo (1024 bytes) */
const uint8_t logo_bitmap[1024] = {
    
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x0f, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0xe0, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x03, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xfc, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x1f, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 
0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xfe, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x07, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 
0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x7f, 
0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x03, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0xfe, 0xf0, 0x80, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x07, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0x1f, 0x03, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x07, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x80, 0x00, 0x80, 0xf8, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 
0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x0f, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x07, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x3f, 0x07, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xfe, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x3f, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x80, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x3f, 0xff, 0xff, 0xff, 0xff, 0x1f, 0x03, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x0f, 0x7f, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0xc0, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x1f, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x3f, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0x3f, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x07, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xfc, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x0f, 0x7f, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0x0f, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x03, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xfe, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

void paste_logo(void) {
    OLED_write_cmd(OLED_CMD_MEM_ADDR_MODE);          
    OLED_write_cmd(0x00);          

    OLED_write_cmd(OLED_CMD_SET_COL_ADDR);          
    OLED_write_cmd(0x00);          
    OLED_write_cmd(OLED_WIDTH - 1); 

    OLED_write_cmd(OLED_CMD_SET_PAGE_ADDR);          
    OLED_write_cmd(0x00);          
    OLED_write_cmd(OLED_PAGE_COUNT - 1); 

    const size_t CHUNK_SIZE = 64;  
    uint16_t bytes_sent = 0;
    
    while(bytes_sent < OLED_BUFFER_SIZE) {
        uint16_t remaining = OLED_BUFFER_SIZE - bytes_sent;
        uint16_t send_size = (remaining > CHUNK_SIZE) ? CHUNK_SIZE : remaining;
        
        OLED_write_data((uint8_t*)(logo_bitmap + bytes_sent), send_size);
        bytes_sent += send_size;
        nrf_delay_us(50);
    }

    OLED_write_cmd(OLED_CMD_MEM_ADDR_MODE);          
    OLED_write_cmd(0x02);          
}





void OLED_clear() {
    uint8_t zero_buffer[128] = {0}; 

    OLED_write_cmd(OLED_CMD_MEM_ADDR_MODE); 
    OLED_write_cmd(0x00);

    OLED_write_cmd(OLED_CMD_SET_COL_ADDR); 
    OLED_write_cmd(0x00); 
    OLED_write_cmd(0x7F); 
    OLED_write_cmd(OLED_CMD_SET_PAGE_ADDR); 
    OLED_write_cmd(0x00); 
    OLED_write_cmd(0x07); 

    for (uint8_t page = 0; page < 8; page++) {
        OLED_write_data(zero_buffer, sizeof(zero_buffer));
    }
    OLED_write_cmd(OLED_CMD_MEM_ADDR_MODE); 
    OLED_write_cmd(0x02); 
}

void OLED_init() {
    OLED_write_cmd(0xA8); 
    OLED_write_cmd(0x3F); 
    OLED_write_cmd(0xD3); 
    OLED_write_cmd(0x00); 
    OLED_write_cmd(0x40); 
    OLED_write_cmd(0xA1); 
    OLED_write_cmd(0xC8); 
    OLED_write_cmd(0xDA); 
    OLED_write_cmd(0x12);
    OLED_write_cmd(OLED_CMD_MEM_ADDR_MODE); 
    OLED_write_cmd(0x00); 
    OLED_write_cmd(0x81); 
    OLED_write_cmd(0x7F); 
    OLED_write_cmd(0xA4); 
    OLED_write_cmd(0xA6); 
    OLED_write_cmd(0xD5);
    OLED_write_cmd(0x80);
    OLED_write_cmd(0x8D); 
    OLED_write_cmd(0x14);
    OLED_write_cmd(0xAF);
}

void OLED_set_invert(bool invert) {
    OLED_write_cmd(invert ? 0xA7 : 0xA6);
}

void choose_column_to_write(uint8_t page, uint8_t column){
    page = (page > 7) ? 7 : page;
    column = (column > 127) ? 127 : column;
    OLED_write_cmd(0xB0 | page);
    OLED_write_cmd(0x00 | (column & 0x0F));    
    OLED_write_cmd(0x10 | (column >> 4));      
}
void write_numbers_OLED(char *str, uint8_t page, uint8_t column){

while(*str){
    choose_column_to_write(page, column);
    if(*str == '0')
      OLED_write_data(first_buf[0], 14);
    else if(*str == '1')
      OLED_write_data(first_buf[1], 14);
    else if(*str == '2')
      OLED_write_data(first_buf[2], 14);
    else if(*str == '3')
      OLED_write_data(first_buf[3], 14);
    else if(*str == '4')
      OLED_write_data(first_buf[4], 14);
    else if(*str == '5')
      OLED_write_data(first_buf[5], 14);
    else if(*str == '6')
      OLED_write_data(first_buf[6], 14);
    else if(*str == '7')
      OLED_write_data(first_buf[7], 14);
    else if(*str == '8')
      OLED_write_data(first_buf[8], 14);
    else if(*str == '9')
      OLED_write_data(first_buf[9], 14);
    else if(*str == 's')
      OLED_write_data(first_buf[11], 14);
    else if(*str == 't')
      OLED_write_data(first_buf[12], 14);
    else if(*str == 'e')
      OLED_write_data(first_buf[13], 14);
    else if(*str == 'p')
      OLED_write_data(first_buf[14], 14);
    else
      OLED_write_data(first_buf[10], 14);
    nrf_delay_ms(4);
    
    choose_column_to_write(page + 1, column);
    if(*str == '0')
      OLED_write_data(second_buf[0], 14);
    else if(*str == '1')
      OLED_write_data(second_buf[1], 14);
    else if(*str == '2')
      OLED_write_data(second_buf[2], 14);
    else if(*str == '3')
      OLED_write_data(second_buf[3], 14);
    else if(*str == '4')
      OLED_write_data(second_buf[4], 14);
    else if(*str == '5')
      OLED_write_data(second_buf[5], 14);
    else if(*str == '6')
      OLED_write_data(second_buf[6], 14);
    else if(*str == '7')
      OLED_write_data(second_buf[7], 14);
    else if(*str == '8')
      OLED_write_data(second_buf[8], 14);
    else if(*str == '9')
      OLED_write_data(second_buf[9], 14);
    else if(*str == 's')
      OLED_write_data(second_buf[11], 14);
    else if(*str == 't')
      OLED_write_data(second_buf[12], 14);
    else if(*str == 'e')
      OLED_write_data(second_buf[13], 14);
    else if(*str == 'p')
      OLED_write_data(second_buf[14], 14);
    else
      OLED_write_data(second_buf[10], 14);
    nrf_delay_ms(4);

    column += 14;
    str++;
  }
}
static void renderStepCount(void)
{
    // “step:” is always in indices 0..4
    step_str[0] = 's';
    step_str[1] = 't';
    step_str[2] = 'e';
    step_str[3] = 'p';
    step_str[4] = ':';

    // Now fill digits at step_str[5..8]:
    int temp = step_count;
    for (int i = 8; i >= 5; i--) {
        step_str[i] = '0' + (temp % 10);
        temp /= 10;
    }
    step_str[9] = '\0';  // terminate

    // That gives you exactly “step:0123” or “step:0000”, etc.
    write_numbers_OLED(step_str, 3, 2);
}


/* ===== TWI Initialization ===== */
static void twi_event_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    if (p_event->type == NRF_DRV_TWI_EVT_DONE)
        m_xfer_done = true;
        NRF_LOG_INFO("TWI event: DONE");
}


static void twi_config(void)
{
    nrf_drv_twi_config_t const oled_cfg = {
        .scl = 2,  .sda = 31,
        .frequency = NRF_DRV_TWI_FREQ_100K,
        .interrupt_priority = APP_IRQ_PRIORITY_LOWEST,
        .clear_bus_init = false
    };
    APP_ERROR_CHECK(nrf_twi_mngr_init(&m_oled_twi_mngr, &oled_cfg));

    nrf_drv_twi_config_t const apds_cfg = {
        .scl = 15, .sda = 14,
        .frequency = NRF_DRV_TWI_FREQ_100K,
        .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
        .clear_bus_init = false
    };
    APP_ERROR_CHECK(nrf_drv_twi_init(&m_twi, &apds_cfg,
                                     twi_event_handler, NULL));

    nrf_gpio_cfg(PIN_R_PULLUP,
        NRF_GPIO_PIN_DIR_OUTPUT,
        NRF_GPIO_PIN_INPUT_CONNECT,
        NRF_GPIO_PIN_NOPULL,
        NRF_GPIO_PIN_S0H1,
        NRF_GPIO_PIN_NOSENSE);
    nrf_gpio_pin_set(PIN_R_PULLUP);
    nrf_drv_twi_enable(&m_twi);

}

/* ===== Logger Initialization ===== */
void log_init(void)
{
    ret_code_t err_code;

    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/* ===== Update Display by Mode ===== */
void update_display(void)
{
    OLED_clear();
    OLED_write_cmd(0x20);  
    OLED_write_cmd(0x00);  

    switch (current_mode)
    {
        case DISPLAY_INVERTED_LOGO:
            OLED_write_cmd(0xA7);  
            break;

        case DISPLAY_STEPS:
            OLED_write_cmd(0xA7);
            break;

        case DISPLAY_LOGO:
        case DISPLAY_CLOCK:
        default:
            OLED_write_cmd(0xA6);  
            break;
    }

    switch (current_mode)
    {
        case DISPLAY_LOGO:
            paste_logo();
            break;

        case DISPLAY_INVERTED_LOGO:
            paste_logo();
            break;

        case DISPLAY_STEPS:
            // Draw the literal "step:XXXX"
            paste_zero();
            nrf_delay_ms(100);
            renderStepCount();
            break;

        case DISPLAY_CLOCK:
        default:
            {
                char time_str[9];
                snprintf(time_str, sizeof(time_str), "%02d:%02d:%02d",
                         hour, min, sec);
                paste_zero();        
                nrf_delay_ms(100);   
                write_numbers_OLED(time_str, 3, 8);
            }
            break;
    }
    OLED_write_cmd(0xAF);
}

/* ===== Timer Handler ===== */
void timer_handler(nrf_timer_event_t event_type, void *p_context) {
    if (event_type == NRF_TIMER_EVENT_COMPARE0) {
        sec++;
        if (sec >= 60) {
            sec = 0;
            min++;
            if (min >= 60) {
                min = 0;
                hour++;
                if (hour >= 24) hour = 0;
            }
        }
        time_updated = true;

        // Inactivity handling
        if (oled_active) {
            elapsed_inactive_seconds++;
            if (elapsed_inactive_seconds >= 15) {
                OLED_write_cmd(OLED_CMD_DISP_OFF);
                oled_active = false;
                elapsed_inactive_seconds = 0;
            }
        }

        if (gesture_processing_active) {
            gesture_timeout_counter++;
        }
    }
}

void update_clock_display() {
    char time_str[9];
    snprintf(time_str, sizeof(time_str), "%02d:%02d:%02d", hour, min, sec);
    OLED_clear();
    write_numbers_OLED(time_str, 3, 8);
}

// Proximity interrupt handler
void proximity_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
    uint8_t pdata;
    readRegister(APDS9960_I2C_ADDR, 0x9C, &pdata); 

    uint8_t en_reg;
    getENABLE(&en_reg);
    NRF_LOG_INFO("ENABLE after gesture activation: 0x%02X", en_reg);

    en_reg &= ~(PIEN | PEN);       
    en_reg |= BIT_GEN | AIEN;  

    setENABLE(en_reg);           
    // Debug
    NRF_LOG_INFO("Prox Int! ENABLE: 0x%02X", en_reg);
    gesture_processing_active = true;
    gesture_timeout_counter = 0;
    elapsed_inactive_seconds = 0;

    if (!oled_active) {
        OLED_write_cmd(OLED_CMD_DISP_ON);
        oled_active = true;
        update_display();
    }
}

/* ===== BLE Peer Manager Helpers ===== */
static void peer_list_get(pm_peer_id_t * p_peers, uint32_t * p_size)
{
    pm_peer_id_t peer_id;
    uint32_t     peers_to_copy = (*p_size < BLE_GAP_WHITELIST_ADDR_MAX_COUNT)
                                 ? *p_size
                                 : BLE_GAP_WHITELIST_ADDR_MAX_COUNT;

    peer_id = pm_next_peer_id_get(PM_PEER_ID_INVALID);
    *p_size = 0;

    while ((peer_id != PM_PEER_ID_INVALID) && (peers_to_copy--))
    {
        p_peers[(*p_size)++] = peer_id;
        peer_id = pm_next_peer_id_get(peer_id);
    }
}

static void delete_bonds(void)
{
    ret_code_t err_code;
    NRF_LOG_INFO("Erase bonds!");
    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}

/* ===== Connection Parameters ===== */
static void conn_params_init(void)
{
    ret_code_t err_code;
    ble_conn_params_init_t cp_init;
    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.disconnect_on_fail             = true;
    cp_init.evt_handler                    = NULL;
    cp_init.error_handler                  = NULL;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

static void advertising_start(bool erase_bonds)
{
    ret_code_t err_code;

    if (erase_bonds == true)
    {
        delete_bonds();
    }
    else
    {
        memset(m_whitelist_peers, PM_PEER_ID_INVALID, sizeof(m_whitelist_peers));
        m_whitelist_peer_cnt = (sizeof(m_whitelist_peers) / sizeof(pm_peer_id_t));
        peer_list_get(m_whitelist_peers, &m_whitelist_peer_cnt);

        err_code = pm_whitelist_set(m_whitelist_peers, m_whitelist_peer_cnt);
        APP_ERROR_CHECK(err_code);

        err_code = pm_device_identities_list_set(m_whitelist_peers, m_whitelist_peer_cnt);
        if (err_code != NRF_ERROR_NOT_SUPPORTED)
        {
            APP_ERROR_CHECK(err_code);
        }

        err_code = ble_advertising_start(&m_adv, BLE_ADV_MODE_FAST);
        APP_ERROR_CHECK(err_code);
    }
}


/* ===== Peer Manager Event Handler ===== */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    ret_code_t err_code;

    pm_handler_on_pm_evt(p_evt);
    pm_handler_disconnect_on_sec_failure(p_evt);
    pm_handler_flash_clean(p_evt);

    switch (p_evt->evt_id)
    {
        case PM_EVT_CONN_SEC_SUCCEEDED:
        {
            m_peer_id = p_evt->peer_id;

            // Discover peer's services.
            err_code  = ble_db_discovery_start(&m_db_disc, p_evt->conn_handle);
            APP_ERROR_CHECK(err_code);
        } break;

        case PM_EVT_PEERS_DELETE_SUCCEEDED:
        {
            advertising_start(false);
        } break;

        case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
        {
            // Only if bonding data in flash actually changed:
            if (p_evt->params.peer_data_update_succeeded.flash_changed
             && (p_evt->params.peer_data_update_succeeded.data_id == PM_PEER_DATA_ID_BONDING))
            {
                if (m_whitelist_peer_cnt < BLE_GAP_WHITELIST_ADDR_MAX_COUNT)
                {
                    m_whitelist_peers[m_whitelist_peer_cnt++] = m_peer_id;

                    err_code = pm_device_identities_list_set(m_whitelist_peers,
                                                             m_whitelist_peer_cnt);
                    APP_ERROR_CHECK(err_code);

                    err_code = pm_whitelist_set(m_whitelist_peers, m_whitelist_peer_cnt);
                    APP_ERROR_CHECK(err_code);

                    NRF_LOG_INFO("Added peer 0x%02x to whitelist (now %u total)",
                                 m_peer_id, m_whitelist_peer_cnt);
                }
                else
                {
                    NRF_LOG_WARNING("Cannot add peer 0x%02x: whitelist is full", m_peer_id);
                }
            }
        } break;

        default:
            break;
    }
}
/* ===== Peer Manager Initialization ===== */
static void peer_manager_init(void)
{
    ret_code_t           err_code;
    ble_gap_sec_params_t sec_param;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);
    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}
/* ===== DB Discovery Handler ===== */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    ble_cts_c_on_db_disc_evt(&m_cts_c, p_evt);
}
static void advertising_evt(ble_adv_evt_t ble_adv_evt)
{
    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast advertising");
            break;

        case BLE_ADV_EVT_SLOW:
            NRF_LOG_INFO("Slow advertising");
            break;

        case BLE_ADV_EVT_FAST_WHITELIST:
            NRF_LOG_INFO("Fast adv. with whitelist");
            break;

        case BLE_ADV_EVT_SLOW_WHITELIST:
            NRF_LOG_INFO("Slow adv. with whitelist");
            {
                ret_code_t err_code = ble_advertising_restart_without_whitelist(&m_adv);
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BLE_ADV_EVT_IDLE:
            // Optionally go to sleep here
            // sleep_mode_enter();
            break;

        case BLE_ADV_EVT_WHITELIST_REQUEST:
        {
            ble_gap_addr_t whitelist_addrs[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
            ble_gap_irk_t  whitelist_irks[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
            uint32_t       addr_cnt = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;
            uint32_t       irk_cnt  = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;

            ret_code_t err_code = pm_whitelist_get(whitelist_addrs, &addr_cnt,
                                                   whitelist_irks,  &irk_cnt);
            APP_ERROR_CHECK(err_code);

            err_code = ble_advertising_whitelist_reply(&m_adv,
                                                       whitelist_addrs, addr_cnt,
                                                       whitelist_irks,  irk_cnt);
            APP_ERROR_CHECK(err_code);
        }
        break;

        default:
            break;
    }
}

/* ===== DB Discovery ===== */
static void db_disc_init(void)
{
    ble_db_discovery_init_t db_init = {0};
    db_init.evt_handler  = db_disc_handler;
    db_init.p_gatt_queue = &m_ble_gatt_q;
    ret_code_t err_code = ble_db_discovery_init(&db_init);
    APP_ERROR_CHECK(err_code);
}

/* ===== GAP Parameters ===== */
static void gap_params_init(void)
{
    ret_code_t err_code;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    ble_gap_conn_params_t gap_conn_params;
    memset(&gap_conn_params, 0, sizeof(gap_conn_params));
    gap_conn_params.min_conn_interval = MIN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

/* ===== GATT Module Initialization ===== */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}


/* UUIDs to advertise (CTS) */
static ble_uuid_t m_adv_uuids[] = {{BLE_UUID_CURRENT_TIME_SERVICE, BLE_UUID_TYPE_BLE}};

/* ===== Advertising ===== */
static void advertising_init(void)
{
    ret_code_t             err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type                = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance       = true;
    init.advdata.flags                    = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;
    init.advdata.uuids_solicited.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_solicited.p_uuids  = m_adv_uuids;

    init.config.ble_adv_whitelist_enabled = true;
    init.config.ble_adv_fast_enabled      = true;
    init.config.ble_adv_fast_interval     = 0x0028;
    init.config.ble_adv_fast_timeout      = 3000;
    init.config.ble_adv_slow_enabled      = true;
    init.config.ble_adv_slow_interval     = 0x0C80;
    init.config.ble_adv_slow_timeout      = 18000;

    init.evt_handler = advertising_evt;

    err_code = ble_advertising_init(&m_adv, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_adv, APP_BLE_TAG);
}


/* ===== GATT Queue Error Handler ===== */
static void gatt_error_handler(uint32_t nrf_error, void * p_ctx)
{
    NRF_LOG_INFO("gatt error");
    ble_cts_c_t * p_cts = (ble_cts_c_t *)p_ctx;
    uint16_t conn_handle = p_cts->conn_handle;
    NRF_LOG_ERROR("GATT error 0x%08X on conn_handle %d", nrf_error, conn_handle);
}

/* ===== CTS Client Event Handler ===== */
static void cts_c_evt_handler(ble_cts_c_t * p_cts, ble_cts_c_evt_t * p_evt)
{
NRF_LOG_INFO("cts c evt");
    ret_code_t err_code;

    switch (p_evt->evt_type)
    {
      case BLE_CTS_C_EVT_DISCOVERY_COMPLETE:
            err_code = ble_cts_c_handles_assign(&m_cts_c,
                                                p_evt->conn_handle,
                                                &p_evt->params.char_handles);
            APP_ERROR_CHECK(err_code);
            if (m_cts_c.conn_handle != BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_cts_c_current_time_read(&m_cts_c);
                if (err_code == NRF_ERROR_NOT_FOUND)
                {
                    NRF_LOG_INFO("Current Time Service is not discovered.");
                }
            }
            break;


        case BLE_CTS_C_EVT_DISCOVERY_FAILED:
            NRF_LOG_INFO("Current Time Service not found on server. ");
            if (p_evt->conn_handle != BLE_CONN_HANDLE_INVALID)
            {
                err_code = sd_ble_gap_disconnect(p_evt->conn_handle,
                                                 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BLE_CTS_C_EVT_DISCONN_COMPLETE:
            NRF_LOG_INFO("Disconnect Complete.");
            break;

        case BLE_CTS_C_EVT_CURRENT_TIME:
            NRF_LOG_INFO("Current Time received.");
            hour = p_evt->params.current_time.exact_time_256.day_date_time.date_time.hours;
            min  = p_evt->params.current_time.exact_time_256.day_date_time.date_time.minutes;
            sec  = p_evt->params.current_time.exact_time_256.day_date_time.date_time.seconds;
            NRF_LOG_INFO("\tHours   %i", hour);
            NRF_LOG_INFO("\tMinutes %i", min);
            NRF_LOG_INFO("\tSeconds %i", sec);
            NRF_LOG_FLUSH();
            break;

        case BLE_CTS_C_EVT_INVALID_TIME:
            NRF_LOG_INFO("Invalid Time received.");
            break;

        default:
            break;
    }
}

/* ===== BLE Event Handler ===== */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_ctx)
{
NRF_LOG_INFO("ble evt");


    ret_code_t err_code;

    pm_handler_secure_on_connection(p_ble_evt);

NRF_LOG_INFO("ble evt 2");


    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected.");
            m_conn_handle_periph = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle_periph);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected.");
            m_conn_handle_periph = BLE_CONN_HANDLE_INVALID;
            if (p_ble_evt->evt.gap_evt.conn_handle == m_cts_c.conn_handle)
            {
                m_cts_c.conn_handle = BLE_CONN_HANDLE_INVALID;
            }
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

        case BLE_GATTC_EVT_TIMEOUT:
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            break;
    }
}

/* ===== BLE Observer Registration ===== */
NRF_SDH_BLE_OBSERVER(m_ble_observer,
                     APP_BLE_OBS_PRIORITY,
                     ble_evt_handler,
                     NULL);

/* ===== SoftDevice and BLE Stack Initialization ===== */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    // 1) Bring up the SoftDevice
    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("SoftDevice enabled and BLE stack up.");
}

static void conn_params_error_handler(uint32_t nrf_error)
{

NRF_LOG_INFO("conn params evt");

    APP_ERROR_HANDLER(nrf_error);
}
static void current_time_error_handler(uint32_t nrf_error)
{

NRF_LOG_INFO("current time evt");

    APP_ERROR_HANDLER(nrf_error);
}

/* ===== CTS Client Initialization ===== */
static void cts_c_init(void)
{
    ret_code_t         err_code;
    ble_cts_c_init_t   cts_init = {0};
    nrf_ble_qwr_init_t qwr_init = {0};

    qwr_init.error_handler = conn_params_error_handler;
    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    cts_init.evt_handler   = cts_c_evt_handler;
    cts_init.error_handler = current_time_error_handler;
    cts_init.p_gatt_queue  = &m_ble_gatt_q;
    err_code = ble_cts_c_init(&m_cts_c, &cts_init);
    APP_ERROR_CHECK(err_code);
}

/* ===== Scheduler Initialization ===== */
static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}

/* ===== Power Management Initialization ===== */
static void power_management_init(void)
{
    ret_code_t err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

/* ===== Main ===== */
int main(void)
{
    nrf_gpio_pin_set(PIN_VDD_ENV);
    nrf_gpio_pin_set(PIN_R_PULLUP);
    nrf_gpio_cfg(PIN_VDD_ENV, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, 
                NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0H1, NRF_GPIO_PIN_NOSENSE);
    nrf_gpio_cfg(PIN_R_PULLUP, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, 
                NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0H1, NRF_GPIO_PIN_NOSENSE);
    nrf_delay_ms(4);

    int gesture = GESTURE_NONE;
    
    log_init();
    NRF_LOG_RAW_INFO("\r\nSystem starting...\r\n");
    NRF_LOG_FLUSH();

    scheduler_init();
    power_management_init();

    leds_init();
    NRF_LOG_INFO("LEDs initialized");

    twi_config();
    NRF_LOG_INFO("TWI initialized");

    nrf_twi_sensor_init(&twi_OLED);
    OLED_init();
    OLED_clear();
    update_display();
    NRF_LOG_INFO("OLED initialized");
    NRF_LOG_FLUSH();
    nrf_delay_ms(1000);

    APP_ERROR_CHECK(nrf_drv_gpiote_init());
    NRF_LOG_INFO("GPIOTE initialized");

    if (!APDS9960_init() || !enableGesture()) {
        NRF_LOG_ERROR("Gesture sensor initialization failed!");
        APP_ERROR_HANDLER(NRF_ERROR_INTERNAL);
    }

    initAccelerometer();
    calibrateAccelerometer();

    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer_cfg.frequency = NRF_TIMER_FREQ_1MHz;
    APP_ERROR_CHECK(nrf_drv_timer_init(&timer_inst, &timer_cfg, timer_handler));

    uint32_t ticks = nrf_drv_timer_ms_to_ticks(&timer_inst, 1000);
    nrf_drv_timer_extended_compare(&timer_inst, NRF_TIMER_CC_CHANNEL0, ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
    nrf_drv_timer_enable(&timer_inst);

    // Initialize GPIOTE for proximity interrupt
    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
    in_config.pull = NRF_GPIO_PIN_PULLUP;
    APP_ERROR_CHECK(nrf_drv_gpiote_in_init(19, &in_config, proximity_handler));
    nrf_drv_gpiote_in_event_enable(19, true);

        {
        uint8_t en_reg;
        if (getENABLE(&en_reg))
        {
            en_reg |= BIT_GEN | BIT_PON | PEN;
            setENABLE(en_reg);
        }
    }

    ble_stack_init();         
    gap_params_init();
    gatt_init();
    db_disc_init();
    advertising_init();       
    NRF_LOG_INFO("BLE stack initialized");
    NRF_LOG_FLUSH();

    fds_register(NULL);
    ret_code_t err_code = fds_init();
    APP_ERROR_CHECK(err_code);

    peer_manager_init();       
    cts_c_init();           
    conn_params_init();       

    NRF_LOG_INFO("Current Time Service client started.");
    NRF_LOG_FLUSH();


    bool erase_bonds = true;  
    advertising_start(erase_bonds); 
    NRF_LOG_INFO("adv_start finished");

    {
        uint8_t reg = 0;
        getENABLE(&reg);
        NRF_LOG_INFO("ENABLE register at startup: 0x%02X", reg);
    }

while (true)
    {
        uint8_t en_reg = 0;
        getENABLE(&en_reg);
        bool gesture_engine_enabled = (en_reg & BIT_GEN);

        if (!gesture_processing_active && gesture_engine_enabled)
        {
            uint8_t gstatus = 0;
            if (getGSTATUS(&gstatus) && (gstatus & GVALID))
            {
                gesture_processing_active = true;
                gesture_timeout_counter = 0;
                NRF_LOG_INFO("Gesture data available: entering gesture processing");
            }
        }

        if (gesture_processing_active)
        {
            handleGesture();
            int gesture = readGesture();

            if (gesture != GESTURE_NONE)
            {
                gesture_timeout_counter = 0;

                if (!oled_active)
                {
                    OLED_write_cmd(OLED_CMD_DISP_ON);
                    oled_active = true;
                    update_display();
                    if (current_mode == DISPLAY_CLOCK)
                    {
                        update_clock_display();
                    }
                }
                elapsed_inactive_seconds = 0;

                bool mode_changed = false;
                switch (gesture)
                {
                    case GESTURE_LEFT:
                        current_mode = DISPLAY_LOGO;
                        mode_changed = true;
                        break;
                    case GESTURE_RIGHT:
                        current_mode = DISPLAY_CLOCK;
                        mode_changed = true;
                        break;
                    case GESTURE_UP:
                        display_inverted = false;
                        current_mode = DISPLAY_STEPS;
                        mode_changed = true;
                        break;
                    case GESTURE_DOWN:
                        display_inverted = false;
                        OLED_set_invert(display_inverted);
                        OLED_write_cmd(0xAF);
                        break;
                }

                if (mode_changed)
                {
                    OLED_clear();
                    update_display();
                    if (current_mode == DISPLAY_CLOCK)
                    {
                        update_clock_display();
                    }
                    else if (current_mode == DISPLAY_STEPS)
                    {
                        renderStepCount();
                    }
                }

                setLEDColor(gesture);
            }

            if (gesture_timeout_counter >= 15)
            {
                gesture_processing_active = false;
                uint8_t new_en = en_reg;
                new_en &= ~BIT_GEN;      
                new_en |= (PEN | PIEN);   
                setENABLE(new_en);
                gesture_timeout_counter = 0;
            }
        }

        if (!gesture_processing_active && !oled_active)
        {
            uint8_t pdata = 0;
            if (readRegister(APDS9960_I2C_ADDR, 0x9C, &pdata))
            {
                if (pdata > 10)
                {
                    NRF_LOG_INFO("Proximity detected: enabling gesture engine");
                    uint8_t new_en = en_reg;
                    new_en &= ~(PIEN | PEN);  
                    new_en |= (BIT_GEN | BIT_PON);
                    setENABLE(new_en);
                    gesture_processing_active = true;
                    gesture_timeout_counter = 0;

                    OLED_write_cmd(OLED_CMD_DISP_ON);
                    oled_active = true;
                    update_display();
                }
            }
        }

        if (time_updated)
        {
            if (current_mode == DISPLAY_CLOCK)
            {
                update_clock_display();
            }
            else if (current_mode == DISPLAY_STEPS)
            {
                computeStepCount();    
                renderStepCount();    
            }
            time_updated = false;
        }

        nrf_delay_ms(50);
        app_sched_execute();
        NRF_LOG_PROCESS();
        nrf_pwr_mgmt_run();
    }
}
