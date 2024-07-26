#ifndef _BHI160B_H_
#define _BHI160B_H_
#include <linux/workqueue.h>

#define BHI160B_SUCCESS	        (0)
#define I2C_SUCCESS             (0)
#define I2C_READ_ERROR          (-1)
#define I2C_WRITE_ERROR         (-2)
#define BHI160B_ERROR           (-3)
#define BHI160B_OUT_OF_RANGE    (-3)
#define BHI160B_COMM_RES        (-1)
#define BHI160B_INIT_VALUE      (0)

#define BHI160B_BUFFER_OUT           (0x00)
#define BHI160B_I2C_ADDR             (0x28)
#define BHI160B_CHIP_CONTROL         (0x34)
#define BHI160B_BYTES_REMAINING      (0x38)
#define BHI160B_PRODUCT_ID_ADDR      (0x90)
#define BHI160B_REVSION_ID_ADDR      (0x91)

#define MAX_SENSOR_ID                (0x20)
#define SENSOR_PARAMETER_WRITE       (0xC0)
#define VS_NON_WAKEUP                (0)
#define VS_WAKEUP                    (32)
#define VS_FLUSH_NONE                (0x00)
#define VS_FLUSH_ALL                 (0xFF)
#define VS_FLUSH_SINGLE              (0x01)

#define BHI160B_RESET_ENABLE         (0x01)
#define BHI160B_UPLOAD_DATA          (0x00)
#define BHI160B_CHIP_CTRL_ENABLE_1   (0x02)
#define BHI160B_CHIP_CTRL_ENABLE_2   (0x01)
#define BHI160B_ROM_VERSION_ADDR     (0x70)
#define BHI160B_ROM_VERSION_MSB_DATA (1)
#define BHI160B_ROM_VERSION_LSB_DATA (0)
#define BHI160B_RAM_WRITE_LENGTH     (4)
#define BHI160B_RAM_WRITE_LENGTH_API (32)
#define BHI160B_SIGNATURE_1          (0)
#define BHI160B_SIGNATURE_2          (1)
#define BHI160B_SIGNATURE_LENGTH     (16)
#define BHI160B_SIGNATURE_MEM_LEN    (17)
#define BHI160B_IMAGE_SIGNATURE1     (0x2A)
#define BHI160B_IMAGE_SIGNATURE2     (0x65)
#define BHI160B_SIG_FLAG_1_POS       (2)
#define BHI160B_SIG_FLAG_2_POS       (3)
#define BHI160B_ROM_VER_DI01         (0x01)
#define BHI160B_ROM_VER_DI03         (0x03)
#define BHI160B_ROM_VERSION_DI01     (uint16_t)(0x2112)
#define BHI160B_ROM_VERSION_DI03     (uint16_t)(0x2DAD)
#define BHI160B_RAMPATCH_NOT_MATCH   (-4)
#define BHI160B_RAMPATCH_NOT_SUPPORT (-5)
#define BHI160B_CRC_ERROR            (-6)
#define BHI160B_CRC_HOST_MSB         (3)
#define BHI160B_CRC_HOST_LSB         (0)
#define BHI160B_CRC_HOST_XLSB        (1)
#define BHI160B_CRC_HOST_XXLSB       (2)
#define BHI160B_CRC_HOST_FILE_MSB    (7)
#define BHI160B_CRC_HOST_FILE_LSB    (4)
#define BHI160B_CRC_HOST_FILE_XLSB   (5)
#define BHI160B_CRC_HOST_FILE_XXLSB  (6)
#define BHI160B_SHIFT_BIT_POSITION_BY_08_BITS   (8)
#define BHI160B_SHIFT_BIT_POSITION_BY_16_BITS   (16)
#define BHI160B_SHIFT_BIT_POSITION_BY_24_BITS   (24)
#define BHI160B_GEN_READ_WRITE_LENGTH           (1)
#define BHI160B_PARAMETER_ACK_LENGTH            (250)
#define BHI160B_PARAMETER_ACK_DELAY             (50)
#define BHI160B_READ_BUFFER_LENGTH              (16)
#define BHI160B_GET_ROMVEREXP(bhi160b_sig_flag)     ((bhi160b_sig_flag>>11)&0x03)

#define BHI160B_I2C_REG_BUFFER_ZERO_ADDR        (0x00)
#define BHI160B_I2C_REG_BUFFER_END_ADDR         (0x31)
#define BHI160B_I2C_REG_FIFO_FLUSH_ADDR         (0x32)
#define BHI160B_I2C_REG_CHIP_CONTROL_ADDR       (0x34)
#define BHI160B_I2C_REG_INT_STATUS_ADDR         (0x36)
#define BHI160B_I2C_REG_UPLOAD_0_ADDR           (0x94)
#define BHI160B_I2C_REG_UPLOAD_1_ADDR           (0x95)
#define BHI160B_I2C_REG_UPLOAD_DATA_ADDR        (0x96)
#define BHI160B_I2C_REG_CRC_HOST_ADDR           (0x97)
#define BHI160B_I2C_REG_PARAMETER_REQUEST_ADDR           (0x64)
#define BHI160B_I2C_REG_PARAMETER_PAGE_SELECT_ADDR       (0x54)
#define BHI160B_I2C_REG_PARAMETER_WRITE_BUFFER_ZERO      (0x5C)
#define BHI160B_I2C_REG_PARAMETER_READ_BUFFER_ZERO       (0x3B)
#define BHI160B_I2C_REG_PARAMETER_ACKNOWLEDGE_ADDR       (0x3A)
/**< CRC register*/
#define BHI160B_I2C_REG_RESET_REQUEST_ADDR      (0x9B)
#define BHI160B_I2C_REG_BUFFER_LENGTH           ((BHI160B_I2C_REG_BUFFER_END_ADDR) - (BHI160B_I2C_REG_BUFFER_ZERO_ADDR) + 1)
#define PRODUCT_ID_7183             (0x83)

#define FIFO_SIZE                      (69)
#define MAX_PACKET_LENGTH              (18)
#define BHI160B_INIT_RETRY_COUNT       (3)
#define BHI160B_DELAY_DEFAULT          (200)
#define BHI160B_ENABLE_DEBUG_MSG

#define BHI160B_SET_BITSLICE(regvar, bitname, val)\
        ((regvar & ~bitname##__MSK) | \
        ((val<<bitname##__POS)&bitname##__MSK))

#define BHI160B_GET_BITSLICE(regvar, bitname)\
        ((regvar & bitname##__MSK) >> bitname##__POS)

#define VS_ID_PADDING                           (0)
#define VS_ID_ACCELEROMETER                     (1)
#define VS_ID_MAGNETOMETER                      (2)
#define VS_ID_ORIENTATION                       (3)
#define VS_ID_GYROSCOPE                         (4)
#define VS_ID_LIGHT                             (5)
#define VS_ID_BAROMETER                         (6)
#define VS_ID_TEMPERATURE                       (7)
#define VS_ID_PROXIMITY                         (8)
#define VS_ID_GRAVITY                           (9)
#define VS_ID_LINEAR_ACCELERATION               (10)
#define VS_ID_ROTATION_VECTOR                   (11)
#define VS_ID_HUMIDITY                          (12)
#define VS_ID_AMBIENT_TEMPERATURE               (13)
#define VS_ID_UNCALIBRATED_MAGNETOMETER         (14)
#define VS_ID_GAME_ROTATION_VECTOR              (15)
#define VS_ID_UNCALIBRATED_GYROSCOPE            (16)
#define VS_ID_SIGNIFICANT_MOTION                (17)
#define VS_ID_STEP_DETECTOR                     (18)
#define VS_ID_STEP_COUNTER                      (19)
#define VS_ID_GEOMAGNETIC_ROTATION_VECTOR       (20)
#define VS_ID_HEART_RATE                        (21)
#define VS_ID_TILT_DETECTOR                     (22)
#define VS_ID_WAKE_GESTURE                      (23)
#define VS_ID_GLANCE_GESTURE                    (24)
#define VS_ID_PICKUP_GESTURE                    (25)
#define VS_ID_CUS1                              (26)
#define VS_ID_CUS2                              (27)
#define VS_ID_CUS3                              (28)
#define VS_ID_CUS4                              (29)
#define VS_ID_CUS5                              (30)
#define VS_ID_ACTIVITY                          (31)

#define VS_ID_ACCELEROMETER_WAKEUP                  (VS_ID_ACCELEROMETER+32)
#define VS_ID_MAGNETOMETER_WAKEUP                   (VS_ID_MAGNETOMETER+32)
#define VS_ID_ORIENTATION_WAKEUP                    (VS_ID_ORIENTATION+32)
#define VS_ID_GYROSCOPE_WAKEUP                      (VS_ID_GYROSCOPE+32)
#define VS_ID_LIGHT_WAKEUP                          (VS_ID_LIGHT+32)
#define VS_ID_BAROMETER_WAKEUP                      (VS_ID_BAROMETER+32)
#define VS_ID_TEMPERATURE_WAKEUP                    (VS_ID_TEMPERATURE+32)
#define VS_ID_PROXIMITY_WAKEUP                      (VS_ID_PROXIMITY+32)
#define VS_ID_GRAVITY_WAKEUP                        (VS_ID_GRAVITY+32)
#define VS_ID_LINEAR_ACCELERATION_WAKEUP            (VS_ID_LINEAR_ACCELERATION+32)
#define VS_ID_ROTATION_VECTOR_WAKEUP                (VS_ID_ROTATION_VECTOR+32)
#define VS_ID_HUMIDITY_WAKEUP                       (VS_ID_HUMIDITY+32)
#define VS_ID_AMBIENT_TEMPERATURE_WAKEUP            (VS_ID_AMBIENT_TEMPERATURE+32)
#define VS_ID_UNCALIBRATED_MAGNETOMETER_WAKEUP      (VS_ID_UNCALIBRATED_MAGNETOMETER+32)
#define VS_ID_GAME_ROTATION_VECTOR_WAKEUP           (VS_ID_GAME_ROTATION_VECTOR+32)
#define VS_ID_UNCALIBRATED_GYROSCOPE_WAKEUP         (VS_ID_UNCALIBRATED_GYROSCOPE+32)
#define VS_ID_SIGNIFICANT_MOTION_WAKEUP             (VS_ID_SIGNIFICANT_MOTION+32)
#define VS_ID_STEP_DETECTOR_WAKEUP                  (VS_ID_STEP_DETECTOR+32)
#define VS_ID_STEP_COUNTER_WAKEUP                   (VS_ID_STEP_COUNTER+32)
#define VS_ID_GEOMAGNETIC_ROTATION_VECTOR_WAKEUP    (VS_ID_GEOMAGNETIC_ROTATION_VECTOR+32)
#define VS_ID_HEART_RATE_WAKEUP                     (VS_ID_HEART_RATE+32)
#define VS_ID_TILT_DETECTOR_WAKEUP                  (VS_ID_TILT_DETECTOR+32)
#define VS_ID_WAKE_GESTURE_WAKEUP                   (VS_ID_WAKE_GESTURE+32)
#define VS_ID_GLANCE_GESTURE_WAKEUP                 (VS_ID_GLANCE_GESTURE+32)
#define VS_ID_PICKUP_GESTURE_WAKEUP                 (VS_ID_PICKUP_GESTURE+32)
#define VS_ID_CUS1_WAKEUP              				(VS_ID_CUS1+32)
#define VS_ID_CUS2_WAKEUP 			                (VS_ID_CUS2+32)
#define VS_ID_CUS3_WAKEUP           			    (VS_ID_CUS3+32)
#define VS_ID_CUS4_WAKEUP                 			(VS_ID_CUS4+32)
#define VS_ID_CUS5_WAKEUP                 			(VS_ID_CUS5+32)
#define VS_ID_ACTIVITY_WAKEUP                       (VS_ID_ACTIVITY+32)

#define VS_ID_DEBUG                         245
#define VS_ID_TIMESTAMP_LSW_WAKEUP          246
#define VS_ID_TIMESTAMP_MSW_WAKEUP          247
#define VS_ID_META_EVENT_WAKEUP             248
#define VS_ID_BSX_C                         249
#define VS_ID_BSX_B                         250
#define VS_ID_BSX_A                         251
#define VS_ID_TIMESTAMP_LSW                 252
#define VS_ID_TIMESTAMP_MSW                 253
#define VS_ID_META_EVENT                    254

#define BHI160B_I2C_REG_FIFO_FLUSH__POS             (0)
#define BHI160B_I2C_REG_FIFO_FLUSH__MSK             (0xFF)
#define BHI160B_I2C_REG_FIFO_FLUSH__LEN             (8)
#define BHI160B_I2C_REG_FIFO_FLUSH__REG             (BHI160B_I2C_REG_FIFO_FLUSH_ADDR)

#define BHI160B_I2C_REG_INT_STATUS__POS             (0)
#define BHI160B_I2C_REG_INT_STATUS__MSK             (0xFF)
#define BHI160B_I2C_REG_INT_STATUS__LEN             (8)
#define BHI160B_I2C_REG_INT_STATUS__REG             (BHI160B_I2C_REG_INT_STATUS_ADDR)

#define BHI160B_I2C_REG_BHI160B_INT_STATUS_HOST_INTR__POS  (0)
#define BHI160B_I2C_REG_BHI160B_INT_STATUS_HOST_INTR__MSK  (0x01)
#define BHI160B_I2C_REG_BHI160B_INT_STATUS_HOST_INTR__LEN  (1)
#define BHI160B_I2C_REG_BHI160B_INT_STATUS_HOST_INTR__REG  (BHI160B_I2C_REG_INT_STATUS_ADDR)

#define BHI160B_I2C_REG_BHI160B_INT_STATUS_WAKEUP_WM__POS    (1)
#define BHI160B_I2C_REG_BHI160B_INT_STATUS_WAKEUP_WM__MSK    (0x02)
#define BHI160B_I2C_REG_BHI160B_INT_STATUS_WAKEUP_WM__LEN    (1)
#define BHI160B_I2C_REG_BHI160B_INT_STATUS_WAKEUP_WM__REG    \
(BHI160B_I2C_REG_INT_STATUS_ADDR)

#define BHI160B_I2C_REG_BHI160B_INT_STATUS_WAKEUP_LATENCY__POS   (2)
#define BHI160B_I2C_REG_BHI160B_INT_STATUS_WAKEUP_LATENCY__MSK   (0x04)
#define BHI160B_I2C_REG_BHI160B_INT_STATUS_WAKEUP_LATENCY__LEN   (1)
#define BHI160B_I2C_REG_BHI160B_INT_STATUS_WAKEUP_LATENCY__REG   \
(BHI160B_I2C_REG_INT_STATUS_ADDR)

#define BHI160B_I2C_REG_BHI160B_INT_STATUS_WAKEUP_IMMEDIATE__POS   (3)
#define BHI160B_I2C_REG_BHI160B_INT_STATUS_WAKEUP_IMMEDIATE__MSK   (0x08)
#define BHI160B_I2C_REG_BHI160B_INT_STATUS_WAKEUP_IMMEDIATE__LEN   (1)
#define BHI160B_I2C_REG_BHI160B_INT_STATUS_WAKEUP_IMMEDIATE__REG   \
(BHI160B_I2C_REG_INT_STATUS_ADDR)

#define BHI160B_I2C_REG_BHI160B_INT_STATUS_NON_WAKEUP_WM__POS   (4)
#define BHI160B_I2C_REG_BHI160B_INT_STATUS_NON_WAKEUP_WM__MSK   (0x10)
#define BHI160B_I2C_REG_BHI160B_INT_STATUS_NON_WAKEUP_WM__LEN   (1)
#define BHI160B_I2C_REG_BHI160B_INT_STATUS_NON_WAKEUP_WM__REG   \
(BHI160B_I2C_REG_INT_STATUS_ADDR)

#define BHI160B_I2C_REG_INT_STATUS_NON_WAKEUP_LATENCY__POS   (5)
#define BHI160B_I2C_REG_INT_STATUS_NON_WAKEUP_LATENCY__MSK   (0x20)
#define BHI160B_I2C_REG_INT_STATUS_NON_WAKEUP_LATENCY__LEN   (1)
#define BHI160B_I2C_REG_INT_STATUS_NON_WAKEUP_LATENCY__REG   \
(BHI160B_I2C_REG_INT_STATUS_ADDR)

#define BHI160B_I2C_REG_INT_STATUS_NON_WAKEUP_IMMEDIATE__POS   (6)
#define BHI160B_I2C_REG_INT_STATUS_NON_WAKEUP_IMMEDIATE__MSK   (0x40)
#define BHI160B_I2C_REG_INT_STATUS_NON_WAKEUP_IMMEDIATE__LEN   (1)
#define BHI160B_I2C_REG_INT_STATUS_NON_WAKEUP_IMMEDIATE__REG   \
(BHI160B_I2C_REG_INT_STATUS_ADDR)

#define BHI160B_PARAMETER_ACK_CHECK     (0x80)
#define BHI160B_MASK_LSB_DATA           (0x00FF)
#define BHI160B_MASK_MSB_DATA           (0xFF00)
#define BHI160B_SIC_MASK_MSB_DATA       (0x000000FF)
#define BHI160B_SIC_MASK_LSB_DATA       (0x0000FF00)
#define BHI160B_SIC_MASK_LSB1_DATA      (0x00FF0000)
#define BHI160B_SIC_MASK_LSB2_DATA      (0xFF000000)
#define BHI160B_MASK_META_EVENT         (0xFF)

#define BHI160B_PAGE_1  (0x01)
#define BHI160B_PAGE_2  (0x02)
#define BHI160B_PAGE_3  (0x03)
#define BHI160B_PAGE_15 (0x0F)

#define BHI160B_WRITE_BUFFER_SIZE   (8)
#define BHI160B_WRITE_BUFFER_1_REG  (0)
#define BHI160B_WRITE_BUFFER_2_REG  (1)
#define BHI160B_WRITE_BUFFER_3_REG  (2)
#define BHI160B_WRITE_BUFFER_4_REG  (3)
#define BHI160B_WRITE_BUFFER_5_REG  (4)
#define BHI160B_WRITE_BUFFER_6_REG  (5)
#define BHI160B_WRITE_BUFFER_7_REG  (6)
#define BHI160B_WRITE_BUFFER_8_REG  (7)

#define BHI160B_READ_BUFFER_SIZE    (16)
#define BHI160B_READ_BUFFER_1_REG   (0)
#define BHI160B_READ_BUFFER_2_REG   (1)
#define BHI160B_READ_BUFFER_3_REG   (2)
#define BHI160B_READ_BUFFER_4_REG   (3)
#define BHI160B_READ_BUFFER_5_REG   (4)
#define BHI160B_READ_BUFFER_6_REG   (5)
#define BHI160B_READ_BUFFER_7_REG   (6)
#define BHI160B_READ_BUFFER_8_REG   (7)
#define BHI160B_READ_BUFFER_9_REG   (8)
#define BHI160B_READ_BUFFER_10_REG  (9)
#define BHI160B_READ_BUFFER_11_REG  (10)
#define BHI160B_READ_BUFFER_12_REG  (11)
#define BHI160B_READ_BUFFER_13_REG  (12)
#define BHI160B_READ_BUFFER_14_REG  (13)
#define BHI160B_READ_BUFFER_15_REG  (14)
#define BHI160B_READ_BUFFER_16_REG  (15)

#define BHI160B_PARAMETER_REQUEST_READ_PARAMETER_31     (0x1F)

#define BHI160B_DATA_SIZE_PADDING                       (1)
#define BHI160B_DATA_SIZE_QUATERNION                    (11)
#define BHI160B_DATA_SIZE_VECTOR                        (8)
#define BHI160B_DATA_SIZE_SCALAR_U8                     (2)
#define BHI160B_DATA_SIZE_SCALAR_U16                    (3)
#define BHI160B_DATA_SIZE_SCALAR_S16                    (3)
#define BHI160B_DATA_SIZE_SCALAR_U24                    (4)
#define BHI160B_DATA_SIZE_SENSOR_EVENT                  (1)
#define BHI160B_DATA_SIZE_UNCALIB_VECTOR                (14)
#define BHI160B_DATA_SIZE_META_EVENT                    (4)
#define BHI160B_DATA_SIZE_BSX                           (17)
#define BHI160B_DATA_SIZE_DEBUG                         (14)
#define BHI160B_DATA_SIZE_CUS1                          (1)
#define BHI160B_DATA_SIZE_CUS2                          (1)
#define BHI160B_DATA_SIZE_CUS3                          (1)
#define BHI160B_DATA_SIZE_CUS4                          (1)
#define BHI160B_DATA_SIZE_CUS5                          (1)

enum bhi_log_level{
    BHI_MSG_VERBOSE     = 0x0,
    BHI_MSG_INFO        = 0x1,
    BHI_MSG_DBG         = 0x2,
    BHI_MSG_WARNING     = 0x3,
    BHI_MSG_ERROR       = 0x4,
    BHI_MSG_CRITICAL    = 0x5,
    BHI_MSG_reserved    = 0x8000000
};

typedef struct bhi160b_raw_data
{
    uint8_t  sensor_id;
    int16_t x;
    int16_t y;
    int16_t z;
    uint8_t  status;
} bhi160b_raw_data;

extern enum bhi_log_level bhi_log_lvl;

#define BHI_INFO(fmt, arg...) do {         \
	if (bhi_log_lvl >= BHI_MSG_INFO)       \
		printk(KERN_INFO "[Thundersoft][%s][%d]: " fmt, __func__,  __LINE__, ## arg);    \
} while (0)

#define BHI_DBG(fmt, arg...) do {          \
    if (bhi_log_lvl >= BHI_MSG_DBG)        \
        printk(KERN_DEBUG "[Thundersoft][%s][%d]: " fmt, __func__,  __LINE__, ##arg);   \
} while (0)

#define BHI_WARN(fmt, arg...) do {          \
	if (bhi_log_lvl >= BHI_MSG_WARNING)     \
		printk(KERN_WARNING "[Thundersoft][%s][%d]: " fmt, __func__,  __LINE__, ## arg);   \
} while (0)

#define BHI_ERR(fmt, arg...) do {           \
	if (bhi_log_lvl >= BHI_MSG_ERROR)       \
		printk(KERN_ERR "[Thundersoft][%s][%d]: " fmt, __func__,  __LINE__, ## arg);    \
} while (0)

#ifdef BHI160B_ENABLE_DEBUG_MSG
#define DBG_LOG 1
#else
#define DBG_LOG 0
#endif

typedef enum {
    VS_TYPE_ACCELEROMETER               = VS_ID_ACCELEROMETER,
    VS_TYPE_GEOMAGNETIC_FIELD           = VS_ID_MAGNETOMETER,
    VS_TYPE_ORIENTATION                 = VS_ID_ORIENTATION,
    VS_TYPE_GYROSCOPE                   = VS_ID_GYROSCOPE,
    VS_TYPE_LIGHT                       = VS_ID_LIGHT,
    VS_TYPE_PRESSURE                    = VS_ID_BAROMETER,
    VS_TYPE_TEMPERATURE                 = VS_ID_TEMPERATURE,
    VS_TYPE_PROXIMITY                   = VS_ID_PROXIMITY,
    VS_TYPE_GRAVITY                     = VS_ID_GRAVITY,
    VS_TYPE_LINEAR_ACCELERATION         = VS_ID_LINEAR_ACCELERATION,
    VS_TYPE_ROTATION_VECTOR             = VS_ID_ROTATION_VECTOR,
    VS_TYPE_RELATIVE_HUMIDITY           = VS_ID_HUMIDITY,
    VS_TYPE_AMBIENT_TEMPERATURE         = VS_ID_AMBIENT_TEMPERATURE,
    VS_TYPE_MAGNETIC_FIELD_UNCALIBRATED = VS_ID_UNCALIBRATED_MAGNETOMETER,
    VS_TYPE_GAME_ROTATION_VECTOR        = VS_ID_GAME_ROTATION_VECTOR,
    VS_TYPE_GYROSCOPE_UNCALIBRATED      = VS_ID_UNCALIBRATED_GYROSCOPE,
    VS_TYPE_SIGNIFICANT_MOTION          = VS_ID_SIGNIFICANT_MOTION,
    VS_TYPE_STEP_DETECTOR               = VS_ID_STEP_DETECTOR,
    VS_TYPE_STEP_COUNTER                = VS_ID_STEP_COUNTER,
    VS_TYPE_GEOMAGNETIC_ROTATION_VECTOR = VS_ID_GEOMAGNETIC_ROTATION_VECTOR,
    VS_TYPE_HEART_RATE                  = VS_ID_HEART_RATE,
    VS_TYPE_TILT                        = VS_ID_TILT_DETECTOR,
    VS_TYPE_WAKEUP                      = VS_ID_WAKE_GESTURE,
    VS_TYPE_GLANCE                      = VS_ID_GLANCE_GESTURE,
    VS_TYPE_PICKUP                      = VS_ID_PICKUP_GESTURE,
    VS_TYPE_CUS1                        = VS_ID_CUS1,
    VS_TYPE_CUS2                        = VS_ID_CUS2,
    VS_TYPE_CUS3                        = VS_ID_CUS3,
    VS_TYPE_CUS4                        = VS_ID_CUS4,
    VS_TYPE_CUS5                        = VS_ID_CUS5,
    VS_TYPE_ACTIVITY_RECOGNITION        = VS_ID_ACTIVITY
} bhi160b_virtual_sensor_t;

typedef struct sensor_configuration_wakeup_t
{
        uint16_t wakeup_sample_rate;
        uint16_t wakeup_max_report_latency;
        uint16_t wakeup_change_sensitivity;
        uint16_t wakeup_dynamic_range;
} sensor_configuration_wakeup_t;

typedef struct sensor_configuration_non_wakeup_t
{
        uint16_t non_wakeup_sample_rate;
        uint16_t non_wakeup_max_report_latency;
        uint16_t non_wakeup_change_sensitivity;
        uint16_t non_wakeup_dynamic_range;
} sensor_configuration_non_wakeup_t;

struct parameter_read_buffer_t
{
        uint8_t parameter_1;
        uint8_t parameter_2;
        uint8_t parameter_3;
        uint8_t parameter_4;
        uint8_t parameter_5;
        uint8_t parameter_6;
        uint8_t parameter_7;
        uint8_t parameter_8;
        uint8_t parameter_9;
        uint8_t parameter_10;
        uint8_t parameter_11;
        uint8_t parameter_12;
        uint8_t parameter_13;
        uint8_t parameter_14;
        uint8_t parameter_15;
        uint8_t parameter_16;
};

typedef struct parameter_write_buffer_t
{
        uint8_t write_parameter_byte1;
        uint8_t write_parameter_byte2;
        uint8_t write_parameter_byte3;
        uint8_t write_parameter_byte4;
        uint8_t write_parameter_byte5;
        uint8_t write_parameter_byte6;
        uint8_t write_parameter_byte7;
        uint8_t write_parameter_byte8;
} parameter_write_buffer_t;

typedef struct accel_physical_status_t
{
        uint16_t accel_sample_rate;
        uint16_t accel_dynamic_range;
        uint8_t accel_flag;
} accel_physical_status_t;

typedef struct gyro_physical_status_t
{
        uint16_t gyro_sample_rate;
        uint16_t gyro_dynamic_range;
        uint8_t gyro_flag;
} gyro_physical_status_t;

typedef enum {
    BHI160B_DATA_TYPE_PADDING               = 0,
    BHI160B_DATA_TYPE_QUATERNION            = 1,
    BHI160B_DATA_TYPE_VECTOR                = 2,
    BHI160B_DATA_TYPE_SCALAR_U8             = 3,
    BHI160B_DATA_TYPE_SCALAR_U16            = 4,
    BHI160B_DATA_TYPE_SCALAR_S16            = 5,
    BHI160B_DATA_TYPE_SCALAR_U24            = 6,
    BHI160B_DATA_TYPE_SENSOR_EVENT          = 7,
    BHI160B_DATA_TYPE_UNCALIB_VECTOR        = 8,
    BHI160B_DATA_TYPE_META_EVENT            = 9,
    BHI160B_DATA_TYPE_BSX                   = 10,
    BHI160B_DATA_TYPE_DEBUG                 = 11,
    BHI160B_DATA_TYPE_CUS1                  = 12,
    BHI160B_DATA_TYPE_CUS2                  = 13,
    BHI160B_DATA_TYPE_CUS3                  = 14,
    BHI160B_DATA_TYPE_CUS4                  = 15,
    BHI160B_DATA_TYPE_CUS5                  = 16,
} bhi160b_data_type_t;

typedef enum {
    BHI160B_META_EVENT_TYPE_NOT_USED                = 0,
    BHI160B_META_EVENT_TYPE_FLUSH_COMPLETE          = 1,
    BHI160B_META_EVENT_TYPE_SAMPLE_RATE_CHANGED     = 2,
    BHI160B_META_EVENT_TYPE_POWER_MODE_CHANGED      = 3,
    BHI160B_META_EVENT_TYPE_ERROR                   = 4,
    BHI160B_META_EVENT_TYPE_ALGORITHM               = 5,
    /* IDs 6-10 are reserved */
    BHI160B_META_EVENT_TYPE_SENSOR_ERROR            = 11,
    BHI160B_META_EVENT_TYPE_FIFO_OVERFLOW           = 12,
    BHI160B_META_EVENT_TYPE_DYNAMIC_RANGE_CHANGED   = 13,
    BHI160B_META_EVENT_TYPE_FIFO_WATERMARK          = 14,
    BHI160B_META_EVENT_TYPE_SELF_TEST_RESULTS       = 15,
    BHI160B_META_EVENT_TYPE_INITIALIZED             = 16,

} bhi160b_meta_event_type_t;

typedef struct bhi160b_data_padding_t
{
    uint8_t sensor_id;
} bhi160b_data_padding_t;

typedef struct bhi160b_data_quaternion_t
{
    uint8_t  sensor_id;
    int16_t x;
    int16_t y;
    int16_t z;
    int16_t w;
    int16_t estimated_accuracy;
} bhi160b_data_quaternion_t;

typedef struct bhi160b_data_vector_t
{
    uint8_t  sensor_id;
    int16_t x;
    int16_t y;
    int16_t z;
    uint8_t  status;
} bhi160b_data_vector_t;

typedef struct bhi160b_data_scalar_u8_t
{
    uint8_t sensor_id;
    uint8_t data;
} bhi160b_data_scalar_u8_t;

typedef struct bhi160b_data_scalar_u16_t
{
    uint8_t  sensor_id;
    uint16_t data;
} bhi160b_data_scalar_u16_t;

typedef struct bhi160b_data_scalar_s16_t
{
    uint8_t  sensor_id;
    int16_t data;
} bhi160b_data_scalar_s16_t;

typedef struct bhi160b_data_scalar_u24_t
{
    uint8_t  sensor_id;
    uint32_t data;
} bhi160b_data_scalar_u24_t;

typedef struct bhi160b_data_sensor_event_t
{
    uint8_t sensor_id;
} bhi160b_data_sensor_event_t;

typedef struct bhi160b_data_uncalib_vector_t
{
    uint8_t  sensor_id;
    int16_t x;
    int16_t y;
    int16_t z;
    int16_t x_bias;
    int16_t y_bias;
    int16_t z_bias;
    uint8_t  status;
} bhi160b_data_uncalib_vector_t;

typedef struct bhi160b_data_meta_event_t
{
    uint8_t meta_event_id;
    bhi160b_meta_event_type_t event_number;
    uint8_t sensor_type;
    uint8_t event_specific;
} bhi160b_data_meta_event_t;

typedef struct bhi160b_data_bsx_t
{
    uint8_t sensor_id;
    int32_t x;
    int32_t y;
    int32_t z;
    uint32_t timestamp;
} bhi160b_data_bsx_t;

typedef struct bhi160b_data_debug_t
{
    uint8_t sensor_id;
    uint8_t data[13];
} bhi160b_data_debug_t;

typedef struct bhi160b_data_pdr_t
{
	uint8_t  sensor_id;
	int16_t deltaX;
	int16_t deltaY;
	int16_t deltaZ;
	int16_t confidencelevel;
	uint16_t direction;
	uint16_t stepCount;
} bhi160b_data_pdr_t;

typedef struct bhi160b_data_custom_t
{
	uint8_t  sensor_id;
	uint8_t  data[16];
} bhi160b_data_custom_t;

typedef union bhi160b_data_generic_t
{
    bhi160b_data_padding_t          data_padding;
    bhi160b_data_quaternion_t       data_quaternion;
    bhi160b_data_vector_t           data_vector;
    bhi160b_data_scalar_u8_t        data_scalar_u8;
    bhi160b_data_scalar_u16_t       data_scalar_u16;
    bhi160b_data_scalar_s16_t       data_scalar_s16;
    bhi160b_data_scalar_u24_t       data_scalar_u24;
    bhi160b_data_sensor_event_t     data_sensor_event;
    bhi160b_data_uncalib_vector_t   data_uncalib_vector;
    bhi160b_data_meta_event_t       data_meta_event;
    bhi160b_data_bsx_t              data_bsx;
    bhi160b_data_debug_t            data_debug;
    bhi160b_data_custom_t           data_custom;
    bhi160b_data_pdr_t              data_pdr;
} bhi160b_data_generic_t;

typedef struct bhi160b_accel_info
{
  bhi160b_virtual_sensor_t          sensor_type;
  bhi160b_raw_data                  raw_data;
  sensor_configuration_wakeup_t     wakeup_status;
  sensor_configuration_non_wakeup_t non_wakeup_status;
  accel_physical_status_t           physical_status;
} bhi160b_accel_info;

typedef struct bhi160b_gyro_info
{
  bhi160b_virtual_sensor_t          sensor_type;
  bhi160b_raw_data                  raw_data;
  sensor_configuration_wakeup_t     wakeup_status;
  sensor_configuration_non_wakeup_t non_wakeup_status;
  gyro_physical_status_t            physical_status;
} bhi160b_gyro_info;

typedef struct bhi160b_common_info
{
  bhi160b_virtual_sensor_t          sensor_type;
  bhi160b_raw_data                  raw_data;
  sensor_configuration_wakeup_t     wakeup_status;
  sensor_configuration_non_wakeup_t non_wakeup_status;
  gyro_physical_status_t            physical_status;
} bhi160b_common_info;

typedef struct bhi160b_data
{
	struct i2c_client *client;
    struct input_polled_dev *input_polled;
	struct input_dev *input;
    struct bhi160b_accel_info accel_info;
    struct bhi160b_gyro_info gyro_info;
    struct delayed_work work;
    atomic_t delay;
    bool is_enable;
	int device_id;
} bhi160b_data;

#endif /* _BHI160B_H_ */