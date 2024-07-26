#include <linux/delay.h>

#include "bhi160b.h"

extern int read_reg(uint8_t reg, uint8_t* buffer, uint16_t length);
extern int write_reg(uint8_t reg, uint8_t* buffer, uint16_t length);
extern int bhi160b_read_fifo(uint8_t *buffer, uint16_t buffer_size, uint16_t *bytes_read, uint16_t *bytes_left);

static struct parameter_read_buffer_t read_buffer;
static struct parameter_write_buffer_t write_buffer;

uint8_t fifo[FIFO_SIZE];
uint8_t  _fifoSizes[] = {
    BHI160B_DATA_SIZE_PADDING,
    BHI160B_DATA_SIZE_QUATERNION,
    BHI160B_DATA_SIZE_VECTOR,
    BHI160B_DATA_SIZE_SCALAR_U8,
    BHI160B_DATA_SIZE_SCALAR_U16,
    BHI160B_DATA_SIZE_SCALAR_S16,
    BHI160B_DATA_SIZE_SCALAR_U24,
    BHI160B_DATA_SIZE_SENSOR_EVENT,
    BHI160B_DATA_SIZE_UNCALIB_VECTOR,
    BHI160B_DATA_SIZE_META_EVENT,
    BHI160B_DATA_SIZE_BSX,
    BHI160B_DATA_SIZE_DEBUG,
    BHI160B_DATA_SIZE_CUS1,
    BHI160B_DATA_SIZE_CUS2,
    BHI160B_DATA_SIZE_CUS3,
    BHI160B_DATA_SIZE_CUS4,
    BHI160B_DATA_SIZE_CUS5,
};

static int bhi160b_set_reset_request(uint8_t v_reset_request_u8)
{
    int com_rslt = BHI160B_COMM_RES;
    uint8_t v_data_u8 = BHI160B_INIT_VALUE;
    v_data_u8 = v_reset_request_u8;
    com_rslt = write_reg(BHI160B_I2C_REG_RESET_REQUEST_ADDR, &v_data_u8, 1);
    return com_rslt;
}

static int bhi160b_set_fifo_flush(uint8_t v_fifo_flush_u8)
{
    int com_rslt = BHI160B_COMM_RES;
    uint8_t v_data_u8 = BHI160B_INIT_VALUE;
    com_rslt = read_reg(BHI160B_I2C_REG_FIFO_FLUSH_ADDR, &v_data_u8, 1);
    if (BHI160B_SUCCESS == com_rslt)
    {
        v_data_u8 = BHI160B_SET_BITSLICE(v_data_u8, BHI160B_I2C_REG_FIFO_FLUSH, v_fifo_flush_u8);
        com_rslt += write_reg(BHI160B_I2C_REG_FIFO_FLUSH_ADDR, &v_data_u8, 1);
    }
    return com_rslt;
}

static int bhi160b_set_parameter_request(uint8_t v_parameter_request_u8)
{
    int com_rslt = BHI160B_COMM_RES;
    uint8_t v_data_u8 = BHI160B_INIT_VALUE;
    v_data_u8 = v_parameter_request_u8;
    com_rslt = write_reg(BHI160B_I2C_REG_PARAMETER_REQUEST_ADDR, &v_data_u8, 1);
    return com_rslt;
}

static int bhi160b_set_parameter_page_select(uint8_t v_page_select_u8)
{
    int com_rslt = BHI160B_COMM_RES;
    uint8_t v_data_u8 = BHI160B_INIT_VALUE;
    v_data_u8 = v_page_select_u8;
    com_rslt = write_reg(BHI160B_I2C_REG_PARAMETER_PAGE_SELECT_ADDR, &v_data_u8, 1);
    return com_rslt;
}

static int bhi160b_get_parameter_acknowledge(uint8_t *v_parameter_acknowledge_u8)
{
    int com_rslt = BHI160B_COMM_RES;
    uint8_t v_data_u8 = BHI160B_INIT_VALUE;
    com_rslt = read_reg(BHI160B_I2C_REG_PARAMETER_ACKNOWLEDGE_ADDR, &v_data_u8, 1);
    *v_parameter_acknowledge_u8 = v_data_u8;
    return com_rslt;
}

static int bhi160b_get_rom_version(uint16_t *v_rom_version_u16)
{
    int com_rslt = BHI160B_COMM_RES;
    uint8_t v_data_u8[] = {0, 0};
    com_rslt = read_reg(BHI160B_ROM_VERSION_ADDR, v_data_u8, 2);
    *v_rom_version_u16 = (uint16_t)((v_data_u8[BHI160B_ROM_VERSION_MSB_DATA]
    << BHI160B_SHIFT_BIT_POSITION_BY_08_BITS)
    | (v_data_u8[BHI160B_ROM_VERSION_LSB_DATA]));
    return com_rslt;
}

static int bhi160b_get_crc_host(uint32_t *v_crc_host_u32)
{
    int com_rslt = BHI160B_COMM_RES;
    uint8_t a_data_u8[] = {0, 0, 0, 0};
    com_rslt = read_reg(BHI160B_I2C_REG_CRC_HOST_ADDR, a_data_u8, 4);
    *v_crc_host_u32 = (uint32_t)
    (((uint32_t)a_data_u8[BHI160B_CRC_HOST_MSB]
    << BHI160B_SHIFT_BIT_POSITION_BY_24_BITS) |
    ((uint32_t)a_data_u8[BHI160B_CRC_HOST_XXLSB]
    << BHI160B_SHIFT_BIT_POSITION_BY_16_BITS)
    |(a_data_u8[BHI160B_CRC_HOST_XLSB]
    << BHI160B_SHIFT_BIT_POSITION_BY_08_BITS)
    | (a_data_u8[BHI160B_CRC_HOST_LSB]));
    return com_rslt;
}

static int bhi160b_read_parameter_bytes(uint8_t v_page_select_u8, uint8_t v_parameter_request_u8)
{
    int com_rslt = BHI160B_COMM_RES;
    uint8_t v_parameter_ack_u8 = BHI160B_INIT_VALUE;
    uint8_t init_array_data = BHI160B_INIT_VALUE;
    uint8_t a_read_data_u8[BHI160B_READ_BUFFER_SIZE];
    uint8_t v_parameter_ack_check_u8 = BHI160B_INIT_VALUE;

    for (; init_array_data < BHI160B_READ_BUFFER_SIZE; init_array_data++)
        a_read_data_u8[init_array_data] = BHI160B_INIT_VALUE;

    /* select the page*/
    com_rslt = bhi160b_set_parameter_page_select(v_page_select_u8);

    /* select the parameter*/
    com_rslt += bhi160b_set_parameter_request(v_parameter_request_u8);

    /* read the values*/
    for (v_parameter_ack_check_u8 = BHI160B_INIT_VALUE;
    v_parameter_ack_check_u8 < BHI160B_PARAMETER_ACK_LENGTH;
    v_parameter_ack_check_u8++)
    {
        /* read acknowledgement*/
        com_rslt = bhi160b_get_parameter_acknowledge(&v_parameter_ack_u8);
        if (v_parameter_ack_u8 == v_parameter_request_u8)
        {
                break;
        }
        else if (v_parameter_ack_u8 == BHI160B_PARAMETER_ACK_CHECK)
        {
            mdelay(BHI160B_PARAMETER_ACK_DELAY);
            com_rslt = BHI160B_ERROR;
        }
        else
        {
                /* device not ready yet */
                mdelay(1);
        }
    }
    com_rslt =
    read_reg(BHI160B_I2C_REG_PARAMETER_READ_BUFFER_ZERO,
    a_read_data_u8, BHI160B_READ_BUFFER_LENGTH);

    read_buffer.parameter_1 =
    a_read_data_u8[BHI160B_READ_BUFFER_1_REG];
    read_buffer.parameter_2 =
    a_read_data_u8[BHI160B_READ_BUFFER_2_REG];
    read_buffer.parameter_3 =
    a_read_data_u8[BHI160B_READ_BUFFER_3_REG];
    read_buffer.parameter_4 =
    a_read_data_u8[BHI160B_READ_BUFFER_4_REG];
    read_buffer.parameter_5 =
    a_read_data_u8[BHI160B_READ_BUFFER_5_REG];
    read_buffer.parameter_6 =
    a_read_data_u8[BHI160B_READ_BUFFER_6_REG];
    read_buffer.parameter_7 =
    a_read_data_u8[BHI160B_READ_BUFFER_7_REG];
    read_buffer.parameter_8 =
    a_read_data_u8[BHI160B_READ_BUFFER_8_REG];
    read_buffer.parameter_9 =
    a_read_data_u8[BHI160B_READ_BUFFER_9_REG];
    read_buffer.parameter_10 =
    a_read_data_u8[BHI160B_READ_BUFFER_10_REG];
    read_buffer.parameter_11 =
    a_read_data_u8[BHI160B_READ_BUFFER_11_REG];
    read_buffer.parameter_12 =
    a_read_data_u8[BHI160B_READ_BUFFER_12_REG];
    read_buffer.parameter_13 =
    a_read_data_u8[BHI160B_READ_BUFFER_13_REG];
    read_buffer.parameter_14 =
    a_read_data_u8[BHI160B_READ_BUFFER_14_REG];
    read_buffer.parameter_15 =
    a_read_data_u8[BHI160B_READ_BUFFER_15_REG];
    read_buffer.parameter_16 =
    a_read_data_u8[BHI160B_READ_BUFFER_16_REG];

    return com_rslt;
}

static int bhi160b_get_physical_sensor_status(
    struct accel_physical_status_t *accel_status,
    struct gyro_physical_status_t *gyro_status)
{
    int com_rslt = BHI160B_COMM_RES;
    /* input as page 1 and parameter 31 for system page*/
    com_rslt = bhi160b_read_parameter_bytes(
    BHI160B_PAGE_1, BHI160B_PARAMETER_REQUEST_READ_PARAMETER_31);

    /* accel physical status*/
    accel_status->accel_sample_rate = (uint16_t)((read_buffer.parameter_2
    << BHI160B_SHIFT_BIT_POSITION_BY_08_BITS) | (read_buffer.parameter_1));
    accel_status->accel_dynamic_range = (uint16_t)((read_buffer.parameter_4
    << BHI160B_SHIFT_BIT_POSITION_BY_08_BITS) | (read_buffer.parameter_3));
    accel_status->accel_flag = read_buffer.parameter_5;

    /* gyro physical status*/
    gyro_status->gyro_sample_rate = (uint16_t)((read_buffer.parameter_7
    << BHI160B_SHIFT_BIT_POSITION_BY_08_BITS) | (read_buffer.parameter_6));
    gyro_status->gyro_dynamic_range = (uint16_t)((read_buffer.parameter_9
    << BHI160B_SHIFT_BIT_POSITION_BY_08_BITS) | (read_buffer.parameter_8));
    gyro_status->gyro_flag = (uint8_t)(read_buffer.parameter_10);

    return com_rslt;
}

static int bhi160b_write_parameter_bytes(uint8_t v_page_select_u8, uint8_t v_parameter_request_u8)
{
    int com_rslt = BHI160B_COMM_RES;
    uint8_t v_parameter_ack_u8 = BHI160B_INIT_VALUE;
    uint8_t v_parameter_ack_check_u8 = BHI160B_INIT_VALUE;
    uint8_t v_write_parameter_byte_u8[BHI160B_WRITE_BUFFER_SIZE];
    uint8_t init_array_data = BHI160B_INIT_VALUE;

    for (; init_array_data < BHI160B_WRITE_BUFFER_SIZE; init_array_data++)
        v_write_parameter_byte_u8[init_array_data] = BHI160B_INIT_VALUE;

    /* Assign the load parameters*/
    v_write_parameter_byte_u8[BHI160B_WRITE_BUFFER_1_REG]
    = write_buffer.write_parameter_byte1;
    v_write_parameter_byte_u8[BHI160B_WRITE_BUFFER_2_REG]
    = write_buffer.write_parameter_byte2;
    v_write_parameter_byte_u8[BHI160B_WRITE_BUFFER_3_REG]
    = write_buffer.write_parameter_byte3;
    v_write_parameter_byte_u8[BHI160B_WRITE_BUFFER_4_REG]
    = write_buffer.write_parameter_byte4;
    v_write_parameter_byte_u8[BHI160B_WRITE_BUFFER_5_REG]
    = write_buffer.write_parameter_byte5;
    v_write_parameter_byte_u8[BHI160B_WRITE_BUFFER_6_REG]
    = write_buffer.write_parameter_byte6;
    v_write_parameter_byte_u8[BHI160B_WRITE_BUFFER_7_REG]
    = write_buffer.write_parameter_byte7;
    v_write_parameter_byte_u8[BHI160B_WRITE_BUFFER_8_REG]
    = write_buffer.write_parameter_byte8;

    /* write values to the load address*/
    com_rslt = write_reg(BHI160B_I2C_REG_PARAMETER_WRITE_BUFFER_ZERO,
    &v_write_parameter_byte_u8[BHI160B_WRITE_BUFFER_1_REG],
    BHI160B_WRITE_BUFFER_SIZE);

    /* select the page*/
    com_rslt += bhi160b_set_parameter_page_select(v_page_select_u8);

    /* select the parameter*/
    com_rslt += bhi160b_set_parameter_request(v_parameter_request_u8);

    for (v_parameter_ack_check_u8 = BHI160B_INIT_VALUE;
    v_parameter_ack_check_u8 < BHI160B_PARAMETER_ACK_LENGTH;
    v_parameter_ack_check_u8++)
    {
        /* read the acknowledgement*/
        com_rslt += bhi160b_get_parameter_acknowledge(&v_parameter_ack_u8);
        if (v_parameter_ack_u8 == v_parameter_request_u8)
        {
            com_rslt += BHI160B_SUCCESS;
            break;
        }
        else if (v_parameter_ack_u8 == BHI160B_PARAMETER_ACK_CHECK)
        {
            mdelay(BHI160B_PARAMETER_ACK_DELAY);
            com_rslt += BHI160B_ERROR;
        }
        else
        {
            /* device not ready yet */
            mdelay(1);
        }
    }

    return com_rslt;
}

static int bhi160b_set_non_wakeup_sensor_configuration(
    struct sensor_configuration_non_wakeup_t *sensor_configuration,
    uint8_t v_parameter_request_u8)
{
    int com_rslt = BHI160B_COMM_RES;

    /* write sample rate*/
    write_buffer.write_parameter_byte1 = (uint8_t)(
    sensor_configuration->non_wakeup_sample_rate & BHI160B_MASK_LSB_DATA);
    write_buffer.write_parameter_byte2 = (uint8_t)(
    (sensor_configuration->non_wakeup_sample_rate
    & BHI160B_MASK_MSB_DATA) >> BHI160B_SHIFT_BIT_POSITION_BY_08_BITS);

    /* write maximum report latency*/
    write_buffer.write_parameter_byte3 = (uint8_t)(
    sensor_configuration->non_wakeup_max_report_latency
    & BHI160B_MASK_LSB_DATA);
    write_buffer.write_parameter_byte4 = (uint8_t)(
    (sensor_configuration->non_wakeup_max_report_latency
    & BHI160B_MASK_MSB_DATA) >> BHI160B_SHIFT_BIT_POSITION_BY_08_BITS);

    /* write sensitivity*/
    write_buffer.write_parameter_byte5 = (uint8_t)(
    sensor_configuration->non_wakeup_change_sensitivity
     & BHI160B_MASK_LSB_DATA);
    write_buffer.write_parameter_byte6 = (uint8_t)(
    (sensor_configuration->non_wakeup_change_sensitivity
    & BHI160B_MASK_MSB_DATA) >> BHI160B_SHIFT_BIT_POSITION_BY_08_BITS);

    /* write dynamic range*/
    write_buffer.write_parameter_byte7 = (uint8_t)(
    sensor_configuration->non_wakeup_dynamic_range & BHI160B_MASK_LSB_DATA);
    write_buffer.write_parameter_byte8 = (uint8_t)(
    (sensor_configuration->non_wakeup_dynamic_range
    & BHI160B_MASK_MSB_DATA) >> BHI160B_SHIFT_BIT_POSITION_BY_08_BITS);

    /* load the parameter of non wakeup sensor configuration*/
    com_rslt = bhi160b_write_parameter_bytes(
    BHI160B_PAGE_3, v_parameter_request_u8);

    return com_rslt;
}

static int bhi160b_set_wakeup_sensor_configuration(
    struct sensor_configuration_wakeup_t *sensor_configuration,
    uint8_t v_parameter_request_u8)
{
    int com_rslt = BHI160B_COMM_RES;

    /* write sample rate*/
    write_buffer.write_parameter_byte1 = (uint8_t)(
    sensor_configuration->wakeup_sample_rate & BHI160B_MASK_LSB_DATA);
    write_buffer.write_parameter_byte2 = (uint8_t)(
    (sensor_configuration->wakeup_sample_rate
    & BHI160B_MASK_MSB_DATA) >> BHI160B_SHIFT_BIT_POSITION_BY_08_BITS);

    /* write maximum report latency*/
    write_buffer.write_parameter_byte3 = (uint8_t)(
    sensor_configuration->wakeup_max_report_latency & BHI160B_MASK_LSB_DATA);
    write_buffer.write_parameter_byte4 = (uint8_t)(
    (sensor_configuration->wakeup_max_report_latency
    & BHI160B_MASK_MSB_DATA) >> BHI160B_SHIFT_BIT_POSITION_BY_08_BITS);

    /* write change sensitivity*/
    write_buffer.write_parameter_byte5 = (uint8_t)(
    sensor_configuration->wakeup_change_sensitivity & BHI160B_MASK_LSB_DATA);
    write_buffer.write_parameter_byte6 = (uint8_t)(
    (sensor_configuration->wakeup_change_sensitivity
    & BHI160B_MASK_MSB_DATA) >> BHI160B_SHIFT_BIT_POSITION_BY_08_BITS);

    /* write dynamic range*/
    write_buffer.write_parameter_byte7 = (uint8_t)(
    sensor_configuration->wakeup_dynamic_range & BHI160B_MASK_LSB_DATA);
    write_buffer.write_parameter_byte8 = (uint8_t)(
    (sensor_configuration->wakeup_dynamic_range
    & BHI160B_MASK_MSB_DATA) >> BHI160B_SHIFT_BIT_POSITION_BY_08_BITS);

    /* load the parameter of non wakeup sensor configuration*/
    com_rslt = bhi160b_write_parameter_bytes(
    BHI160B_PAGE_3, v_parameter_request_u8);

    return com_rslt;
}

static int bhi160b_get_interrupt_status(uint8_t *v_host_int_u8,
                                uint8_t *v_wakeup_water_mark_u8, uint8_t *v_wakeup_latency_u8,
                                uint8_t *v_wakeup_immediate_u8, uint8_t *v_non_wakeup_water_mark_u8,
                                uint8_t *v_non_wakeup_latency_u8, uint8_t *v_non_wakeup_immediate_u8)
{
    int com_rslt = BHI160B_COMM_RES;
    uint8_t v_data_u8 = BHI160B_INIT_VALUE;

    /* read the interrupt status*/
    com_rslt =
    read_reg(BHI160B_I2C_REG_INT_STATUS__REG, &v_data_u8, BHI160B_GEN_READ_WRITE_LENGTH);

    /* read the host interrupt status*/
    *v_host_int_u8 = BHI160B_GET_BITSLICE(v_data_u8,
    BHI160B_I2C_REG_BHI160B_INT_STATUS_HOST_INTR);
    /* read the wakeup watermark interrupt status*/
    *v_wakeup_water_mark_u8 = BHI160B_GET_BITSLICE(v_data_u8,
    BHI160B_I2C_REG_BHI160B_INT_STATUS_WAKEUP_WM);
    /* read the wakeup latency interrupt status*/
    *v_wakeup_latency_u8 = BHI160B_GET_BITSLICE(v_data_u8,
    BHI160B_I2C_REG_BHI160B_INT_STATUS_WAKEUP_LATENCY);
    /* read the wakeup immediate interrupt status*/
    *v_wakeup_immediate_u8 = BHI160B_GET_BITSLICE(v_data_u8,
    BHI160B_I2C_REG_BHI160B_INT_STATUS_WAKEUP_IMMEDIATE);
    /* read the non wakeup watermark interrupt status*/
    *v_non_wakeup_water_mark_u8 = BHI160B_GET_BITSLICE(v_data_u8,
    BHI160B_I2C_REG_BHI160B_INT_STATUS_NON_WAKEUP_WM);
    /* read the non wakeup latency interrupt status*/
    *v_non_wakeup_latency_u8 = BHI160B_GET_BITSLICE(v_data_u8,
    BHI160B_I2C_REG_INT_STATUS_NON_WAKEUP_LATENCY);
    /* read the non wakeup immediate status*/
    *v_non_wakeup_immediate_u8 = BHI160B_GET_BITSLICE(v_data_u8,
    BHI160B_I2C_REG_INT_STATUS_NON_WAKEUP_IMMEDIATE);

    return com_rslt;
}

static uint8_t interruptFired(void)
{
    uint8_t v_host_int_u8;
    uint8_t v_wakeup_water_mark_u8;
    uint8_t v_wakeup_latency_u8;
    uint8_t v_wakeup_immediate_u8;
    uint8_t v_non_wakeup_water_mark_u8;
    uint8_t v_non_wakeup_latency_u8;
    uint8_t v_non_wakeup_immediate_u8;
    bhi160b_get_interrupt_status(&v_host_int_u8, &v_wakeup_water_mark_u8, &v_wakeup_latency_u8,
    &v_wakeup_immediate_u8, &v_non_wakeup_water_mark_u8, &v_non_wakeup_latency_u8, &v_non_wakeup_immediate_u8);
    return v_host_int_u8 > 0;
}

static int bhi160b_initialize_from_rom( const uint8_t *memory, const uint32_t v_file_length_u32)
{
    int com_rslt = BHI160B_COMM_RES;
    uint8_t v_upload_addr = BHI160B_UPLOAD_DATA;
    uint8_t v_chip_control_u8 = BHI160B_CHIP_CTRL_ENABLE_1;
    uint32_t v_crc_from_memory_u32 = BHI160B_INIT_VALUE;
    uint32_t v_crc_host_u32 = BHI160B_INIT_VALUE;
    uint32_t write_data = BHI160B_INIT_VALUE;
    uint8_t data_from_mem[BHI160B_SIGNATURE_MEM_LEN];
    uint8_t data_byte[BHI160B_RAM_WRITE_LENGTH_API];
    uint32_t read_index_u8 = BHI160B_INIT_VALUE;
    uint32_t reverse_index_u32 = BHI160B_INIT_VALUE;
    uint32_t reverse_block_index_u32 = BHI160B_INIT_VALUE;
    uint32_t write_length = BHI160B_INIT_VALUE;
    uint32_t data_to_process = BHI160B_INIT_VALUE;
    uint32_t packet_length = BHI160B_INIT_VALUE;
    uint16_t signature_flag = 0;
    uint16_t rom_version = 0;
    uint8_t rom_ver_exp = 0;
    int i = BHI160B_INIT_VALUE;

    /* initialize the array*/
    for (i = 0; i < BHI160B_SIGNATURE_MEM_LEN; i++)
    {
        data_from_mem[i] = BHI160B_INIT_VALUE;
    }
    for (i = BHI160B_INIT_VALUE; i < BHI160B_RAM_WRITE_LENGTH; i++)
    {
        data_byte[i] = BHI160B_INIT_VALUE;
    }

    /* Assign the memory data into the local array*/
    for (read_index_u8 = BHI160B_INIT_VALUE;read_index_u8 <= BHI160B_SIGNATURE_LENGTH; read_index_u8++)
    {
      data_from_mem[read_index_u8] = *(memory+read_index_u8);
    }

    /* Verify the signature of the data*/
    if ((data_from_mem[BHI160B_SIGNATURE_1] == BHI160B_IMAGE_SIGNATURE1)
    && (data_from_mem[BHI160B_SIGNATURE_2] == BHI160B_IMAGE_SIGNATURE2))
    {
        com_rslt = BHI160B_SUCCESS;
    }
    else
    {
        com_rslt = BHI160B_ERROR;
        goto bhi160b_init_from_rom_return;
    }

    /* Verify the signature of the data*/
    signature_flag = data_from_mem[BHI160B_SIG_FLAG_1_POS] + ((uint16_t)data_from_mem[BHI160B_SIG_FLAG_2_POS]<<8);
    rom_ver_exp = BHI160B_GET_ROMVEREXP (signature_flag);
    bhi160b_get_rom_version(&rom_version);
    if(BHI160B_ROM_VER_DI01 == rom_ver_exp)
    {
        if(BHI160B_ROM_VERSION_DI01 == rom_version)
        {
            com_rslt = BHI160B_SUCCESS;
        }
        else
        {
            com_rslt = BHI160B_RAMPATCH_NOT_MATCH;
            goto bhi160b_init_from_rom_return;
        }
    }
    else if(BHI160B_ROM_VER_DI03 == rom_ver_exp)
    {
        if(BHI160B_ROM_VERSION_DI03 == rom_version)
        {
            com_rslt = BHI160B_SUCCESS;
        }
        else
        {
            com_rslt = BHI160B_RAMPATCH_NOT_MATCH;
            goto bhi160b_init_from_rom_return;
        }
    }
    else
    {
        com_rslt = BHI160B_RAMPATCH_NOT_SUPPORT;
        goto bhi160b_init_from_rom_return;
    }
    /* read the CRC data from memory */
    v_crc_from_memory_u32 = (uint32_t)
    (((uint32_t)data_from_mem[BHI160B_CRC_HOST_FILE_MSB]
    << BHI160B_SHIFT_BIT_POSITION_BY_24_BITS) |
    ((uint32_t)data_from_mem[BHI160B_CRC_HOST_FILE_XXLSB]
    << BHI160B_SHIFT_BIT_POSITION_BY_16_BITS)
    |(data_from_mem[BHI160B_CRC_HOST_FILE_XLSB]
    << BHI160B_SHIFT_BIT_POSITION_BY_08_BITS)
    | (data_from_mem[BHI160B_CRC_HOST_FILE_LSB]));
    /* Remove the first 16 bytes*/
    data_to_process = v_file_length_u32 - BHI160B_SIGNATURE_LENGTH;

    /* set the reset as 0x01*/
    com_rslt = bhi160b_set_reset_request(BHI160B_RESET_ENABLE);
    com_rslt += write_reg(BHI160B_I2C_REG_CHIP_CONTROL_ADDR, &v_chip_control_u8, BHI160B_GEN_READ_WRITE_LENGTH);

    /* set the upload data*/
    com_rslt += write_reg(BHI160B_I2C_REG_UPLOAD_0_ADDR, &v_upload_addr, BHI160B_GEN_READ_WRITE_LENGTH);
    com_rslt += write_reg(BHI160B_I2C_REG_UPLOAD_1_ADDR, &v_upload_addr, BHI160B_GEN_READ_WRITE_LENGTH);

    /* write the chip control register as 0x02*/
    write_length = data_to_process / BHI160B_RAM_WRITE_LENGTH_API;
    read_index_u8 = BHI160B_INIT_VALUE;

    /* write the memory of data */
    /*skips first 16 bytes*/
    write_data += 16;
    if (com_rslt == BHI160B_SUCCESS)
    {
        for (read_index_u8 = BHI160B_INIT_VALUE; read_index_u8 <= write_length; read_index_u8++)
        {
            packet_length = (read_index_u8 == write_length) ?
            (data_to_process % BHI160B_RAM_WRITE_LENGTH_API) / BHI160B_RAM_WRITE_LENGTH :
            BHI160B_RAM_WRITE_LENGTH_API / BHI160B_RAM_WRITE_LENGTH;

            /*reverse the data*/
            for (reverse_block_index_u32 = 1; reverse_block_index_u32 <= packet_length; reverse_block_index_u32++)
            {
                for (reverse_index_u32 = 0; reverse_index_u32 < BHI160B_RAM_WRITE_LENGTH; reverse_index_u32++)
                {
                    data_byte[reverse_index_u32 + ((reverse_block_index_u32-1) * BHI160B_RAM_WRITE_LENGTH)] =
                    *(memory + write_data + BHI160B_RAM_WRITE_LENGTH * reverse_block_index_u32 - (reverse_index_u32 + 1));
                }
            }

            if(packet_length != 0)
                com_rslt += write_reg(BHI160B_I2C_REG_UPLOAD_DATA_ADDR, data_byte, packet_length * BHI160B_RAM_WRITE_LENGTH);

            write_data = write_data + (packet_length * BHI160B_RAM_WRITE_LENGTH);
        }
        read_reg(BHI160B_I2C_REG_UPLOAD_DATA_ADDR, data_byte, 1);
    }

    /* Check the CRC success*/
    com_rslt = bhi160b_get_crc_host(&v_crc_host_u32);
    if (v_crc_from_memory_u32 == v_crc_host_u32)
    {
        com_rslt = BHI160B_SUCCESS;
    }
    else
    {
        com_rslt = BHI160B_CRC_ERROR;
        goto bhi160b_init_from_rom_return;
    }
    /* disable upload mode*/
    v_chip_control_u8 = BHI160B_CHIP_CTRL_ENABLE_2;
    /* write the chip control register as 0x02*/
    com_rslt += write_reg(BHI160B_I2C_REG_CHIP_CONTROL_ADDR, &v_chip_control_u8, BHI160B_GEN_READ_WRITE_LENGTH);

bhi160b_init_from_rom_return:
    return com_rslt;
}

static int bhi160b_enable_virtual_sensor(bhi160b_virtual_sensor_t sensor_id, uint8_t wakeup_status,
                                                    uint16_t sample_rate, uint16_t max_report_latency_ms,
                                                    uint8_t flush_sensor, uint16_t change_sensitivity,
                                                    uint16_t dynamic_range)
{
    int result = BHI160B_SUCCESS;
    union
    {
        struct sensor_configuration_wakeup_t sensor_configuration_wakeup;
        struct sensor_configuration_non_wakeup_t sensor_configuration_non_wakeup;
    } sensor_configuration;

    /* checks if sensor id is in range */
    if ((uint8_t)sensor_id >= MAX_SENSOR_ID)
    {
        return BHI160B_OUT_OF_RANGE;
    }

    /*computes the sensor id */
    sensor_id += wakeup_status;

    /* flush the fifo if requested */
    switch (flush_sensor)
    {
        case VS_FLUSH_SINGLE:
            result += bhi160b_set_fifo_flush(sensor_id);
            break;
        case VS_FLUSH_ALL:
            result += bhi160b_set_fifo_flush(VS_FLUSH_ALL);
            break;
        case VS_FLUSH_NONE:
            break;
        default:
            return BHI160B_OUT_OF_RANGE;
    }

    /* computes the param page as sensor_id + 0xC0 (sensor parameter write)*/
    sensor_id += SENSOR_PARAMETER_WRITE;

    /*calls the right function */
    switch (wakeup_status)
    {
        case VS_NON_WAKEUP:
            sensor_configuration.sensor_configuration_non_wakeup.non_wakeup_sample_rate = sample_rate;
            sensor_configuration.sensor_configuration_non_wakeup.non_wakeup_max_report_latency = max_report_latency_ms;
            sensor_configuration.sensor_configuration_non_wakeup.non_wakeup_change_sensitivity = change_sensitivity;
            sensor_configuration.sensor_configuration_non_wakeup.non_wakeup_dynamic_range = dynamic_range;
            result += bhi160b_set_non_wakeup_sensor_configuration( &sensor_configuration.sensor_configuration_non_wakeup, sensor_id);
            break;
        case VS_WAKEUP:
            sensor_configuration.sensor_configuration_wakeup.wakeup_sample_rate = sample_rate;
            sensor_configuration.sensor_configuration_wakeup.wakeup_max_report_latency = max_report_latency_ms;
            sensor_configuration.sensor_configuration_wakeup.wakeup_change_sensitivity = change_sensitivity;
            sensor_configuration.sensor_configuration_wakeup.wakeup_dynamic_range = dynamic_range;
            result += bhi160b_set_wakeup_sensor_configuration(&sensor_configuration.sensor_configuration_wakeup, sensor_id);
            break;
        default:
            return BHI160B_OUT_OF_RANGE;
    }
    return result;
}

int bhi160b_initialize(const uint8_t *bhi160b_hub_data)
{
    uint32_t tmp_fw_len               = 0;
    int8_t   init_retry_count    = BHI160B_INIT_RETRY_COUNT;
    int result   = BHI160B_SUCCESS;

    /* get fw lenght */
    tmp_fw_len = 16 + bhi160b_hub_data[12] + (256 * bhi160b_hub_data[13]);

    /* retry BHI160B_INIT_RETRY_COUNT times to avoid firmware download fail*/
    while (init_retry_count > 0)
    {
        bhi160b_set_reset_request(BHI160B_RESET_ENABLE);

        /* downloads the ram patch to the BHI160B */
        result += bhi160b_initialize_from_rom(bhi160b_hub_data, /*bhi160b_fw_len*/tmp_fw_len);

        if (result == BHI160B_SUCCESS)
        {
            if (DBG_LOG) BHI_DBG("bhi160b initialize successfully\n");
            break;
        }

        init_retry_count--;
    }
    return result;
}

static int bhi160b_parse_next_fifo_packet (uint8_t **fifo_buffer, uint16_t *fifo_buffer_length,
                                                        bhi160b_data_generic_t * fifo_data_output,
                                                        bhi160b_data_type_t * fifo_data_type)
{
    uint8_t backupFifo;
    int i = 0;
    if ((*fifo_buffer_length) == 0)
    {
        /* there are no more bytes in the fifo buffer to read */
        if (DBG_LOG) BHI_DBG("fifo buffer length = 0\n");
        return BHI160B_SUCCESS;
    }

    /* the first fifo byte should be a known virtual sensor ID */
    backupFifo = **fifo_buffer;
    switch (backupFifo)
    {
        case VS_ID_PADDING:
            (*fifo_data_type) = BHI160B_DATA_TYPE_PADDING;
            fifo_data_output->data_padding.sensor_id = (**fifo_buffer);
            break;

        case VS_ID_ROTATION_VECTOR:
            BHI_INFO("rotation buffer.\n");
            break;
        case VS_ID_ROTATION_VECTOR_WAKEUP:
        case VS_ID_GAME_ROTATION_VECTOR:
        case VS_ID_GAME_ROTATION_VECTOR_WAKEUP:
        case VS_ID_GEOMAGNETIC_ROTATION_VECTOR:
        case VS_ID_GEOMAGNETIC_ROTATION_VECTOR_WAKEUP:
            if ((*fifo_buffer_length) < _fifoSizes[BHI160B_DATA_TYPE_QUATERNION])
                return BHI160B_OUT_OF_RANGE;
            (*fifo_data_type) = BHI160B_DATA_TYPE_QUATERNION;
            fifo_data_output->data_quaternion.sensor_id = (**fifo_buffer);
            fifo_data_output->data_quaternion.x =
            (int16_t)(((uint16_t)*((*fifo_buffer) + 1)) | ((uint16_t)*((*fifo_buffer) + 2) << 8));
            fifo_data_output->data_quaternion.y =
            (int16_t)(((uint16_t)*((*fifo_buffer) + 3)) | ((uint16_t)*((*fifo_buffer) + 4) << 8));
            fifo_data_output->data_quaternion.z =
            (int16_t)(((uint16_t)*((*fifo_buffer) + 5)) | ((uint16_t)*((*fifo_buffer) + 6) << 8));
            fifo_data_output->data_quaternion.w =
            (int16_t)(((uint16_t)*((*fifo_buffer) + 7)) | ((uint16_t)*((*fifo_buffer) + 8) << 8));
            fifo_data_output->data_quaternion.estimated_accuracy =
            (int16_t)(((uint16_t)*((*fifo_buffer) + 9)) | ((uint16_t)*((*fifo_buffer) + 10) << 8));
            break;

        case VS_ID_ACCELEROMETER:
        case VS_ID_ACCELEROMETER_WAKEUP:
        case VS_ID_MAGNETOMETER:
        case VS_ID_MAGNETOMETER_WAKEUP:
        case VS_ID_ORIENTATION:
        case VS_ID_ORIENTATION_WAKEUP:
        case VS_ID_GYROSCOPE:
        case VS_ID_GYROSCOPE_WAKEUP:
        case VS_ID_GRAVITY:
        case VS_ID_GRAVITY_WAKEUP:
        case VS_ID_LINEAR_ACCELERATION:
        case VS_ID_LINEAR_ACCELERATION_WAKEUP:
            if ((*fifo_buffer_length) < _fifoSizes[BHI160B_DATA_TYPE_VECTOR])
            {
                return BHI160B_OUT_OF_RANGE;
            }

            (*fifo_data_type) = BHI160B_DATA_TYPE_VECTOR;
            fifo_data_output->data_vector.sensor_id = (**fifo_buffer);
            fifo_data_output->data_vector.x =
            (int16_t)(((uint16_t)*((*fifo_buffer) + 1)) | ((uint16_t)*((*fifo_buffer) + 2) << 8));
            fifo_data_output->data_vector.y =
            (int16_t)(((uint16_t)*((*fifo_buffer) + 3)) | ((uint16_t)*((*fifo_buffer) + 4) << 8));
            fifo_data_output->data_vector.z =
            (int16_t)(((uint16_t)*((*fifo_buffer) + 5)) | ((uint16_t)*((*fifo_buffer) + 6) << 8));
            fifo_data_output->data_vector.status = *((*fifo_buffer) + 7);
            break;

        case VS_ID_HEART_RATE:
        case VS_ID_HEART_RATE_WAKEUP:
            if ((*fifo_buffer_length) < _fifoSizes[BHI160B_DATA_TYPE_SCALAR_U8])
            {
                return BHI160B_OUT_OF_RANGE;
            }

            (*fifo_data_type) = BHI160B_DATA_TYPE_SCALAR_U8;
            fifo_data_output->data_scalar_u8.sensor_id = (**fifo_buffer);
            fifo_data_output->data_scalar_u8.data = *((*fifo_buffer) + 1);
            break;

        case VS_ID_LIGHT:
        case VS_ID_LIGHT_WAKEUP:
        case VS_ID_PROXIMITY:
        case VS_ID_PROXIMITY_WAKEUP:
        case VS_ID_HUMIDITY:
        case VS_ID_HUMIDITY_WAKEUP:
        case VS_ID_STEP_COUNTER:
        case VS_ID_STEP_COUNTER_WAKEUP:
        case VS_ID_ACTIVITY:
        case VS_ID_ACTIVITY_WAKEUP:
        case VS_ID_TIMESTAMP_LSW:
        case VS_ID_TIMESTAMP_LSW_WAKEUP:
        case VS_ID_TIMESTAMP_MSW:
        case VS_ID_TIMESTAMP_MSW_WAKEUP:
            if ((*fifo_buffer_length) < _fifoSizes[BHI160B_DATA_TYPE_SCALAR_U16])
            {
                return BHI160B_OUT_OF_RANGE;
            }

            (*fifo_data_type) = BHI160B_DATA_TYPE_SCALAR_U16;
            fifo_data_output->data_scalar_u16.sensor_id = (**fifo_buffer);
            fifo_data_output->data_scalar_u16.data =
            (uint16_t)(((uint16_t)*((*fifo_buffer) + 1)) | ((uint16_t)*((*fifo_buffer) + 2) << 8));
            break;

        case VS_ID_TEMPERATURE:
        case VS_ID_TEMPERATURE_WAKEUP:
        case VS_ID_AMBIENT_TEMPERATURE:
        case VS_ID_AMBIENT_TEMPERATURE_WAKEUP:
            if ((*fifo_buffer_length) < _fifoSizes[BHI160B_DATA_TYPE_SCALAR_S16])
            {
                return BHI160B_OUT_OF_RANGE;
            }

            (*fifo_data_type) = BHI160B_DATA_TYPE_SCALAR_S16;
            fifo_data_output->data_scalar_s16.sensor_id = (**fifo_buffer);
            fifo_data_output->data_scalar_s16.data =
            (int16_t)(((uint16_t)*(*fifo_buffer + 1)) | ((uint16_t)*(*fifo_buffer + 2) << 8));
            break;

        case VS_ID_BAROMETER:
        case VS_ID_BAROMETER_WAKEUP:
            if ((*fifo_buffer_length) < _fifoSizes[BHI160B_DATA_TYPE_SCALAR_U24])
            {
                return BHI160B_OUT_OF_RANGE;
            }

            (*fifo_data_type) = BHI160B_DATA_TYPE_SCALAR_U24;
            fifo_data_output->data_scalar_u24.sensor_id = (**fifo_buffer);
            fifo_data_output->data_scalar_u24.data =
            (uint32_t)(((uint32_t)*((*fifo_buffer) + 1)) | ((uint32_t)*((*fifo_buffer) + 2) << 8) |
              ((uint32_t)*((*fifo_buffer) + 3) << 16));
            break;

        case VS_ID_SIGNIFICANT_MOTION:
        case VS_ID_SIGNIFICANT_MOTION_WAKEUP:
        case VS_ID_STEP_DETECTOR:
        case VS_ID_STEP_DETECTOR_WAKEUP:
        case VS_ID_TILT_DETECTOR:
        case VS_ID_TILT_DETECTOR_WAKEUP:
        case VS_ID_WAKE_GESTURE:
        case VS_ID_WAKE_GESTURE_WAKEUP:
        case VS_ID_GLANCE_GESTURE:
        case VS_ID_GLANCE_GESTURE_WAKEUP:
        case VS_ID_PICKUP_GESTURE:
        case VS_ID_PICKUP_GESTURE_WAKEUP:
            (*fifo_data_type) = BHI160B_DATA_TYPE_SENSOR_EVENT;
            fifo_data_output->data_sensor_event.sensor_id = (**fifo_buffer);
            break;

        case VS_ID_UNCALIBRATED_MAGNETOMETER:
        case VS_ID_UNCALIBRATED_MAGNETOMETER_WAKEUP:
        case VS_ID_UNCALIBRATED_GYROSCOPE:
        case VS_ID_UNCALIBRATED_GYROSCOPE_WAKEUP:
            if ((*fifo_buffer_length) < _fifoSizes[BHI160B_DATA_TYPE_UNCALIB_VECTOR])
            {
                return BHI160B_OUT_OF_RANGE;
            }

            (*fifo_data_type) = BHI160B_DATA_TYPE_UNCALIB_VECTOR;
            fifo_data_output->data_uncalib_vector.sensor_id = (**fifo_buffer);
            fifo_data_output->data_uncalib_vector.x =
            (int16_t)(((uint16_t)*((*fifo_buffer) + 1)) | ((uint16_t)*((*fifo_buffer) + 2) << 8));
            fifo_data_output->data_uncalib_vector.y =
            (int16_t)(((uint16_t)*((*fifo_buffer) + 3)) | ((uint16_t)*((*fifo_buffer) + 4) << 8));
            fifo_data_output->data_uncalib_vector.z =
            (int16_t)(((uint16_t)*((*fifo_buffer) + 5)) | ((uint16_t)*((*fifo_buffer) + 6) << 8));
            fifo_data_output->data_uncalib_vector.x_bias =
            (int16_t)(((uint16_t)*((*fifo_buffer) + 7)) | ((uint16_t)*((*fifo_buffer) + 8) << 8));
            fifo_data_output->data_uncalib_vector.y_bias =
            (int16_t)(((uint16_t)*((*fifo_buffer) + 9)) | ((uint16_t)*((*fifo_buffer) + 10) << 8));
            fifo_data_output->data_uncalib_vector.z_bias =
            (int16_t)(((uint16_t)*((*fifo_buffer) + 11)) | ((uint16_t)*((*fifo_buffer) + 12) << 8));
            fifo_data_output->data_uncalib_vector.status = *((*fifo_buffer)+13);
            break;

        case VS_ID_META_EVENT:
        case VS_ID_META_EVENT_WAKEUP:
            if ((*fifo_buffer_length) < _fifoSizes[BHI160B_DATA_TYPE_META_EVENT])
            {
                return BHI160B_OUT_OF_RANGE;
            }

            (*fifo_data_type) = BHI160B_DATA_TYPE_META_EVENT;
            fifo_data_output->data_meta_event.meta_event_id    = (**fifo_buffer);
            fifo_data_output->data_meta_event.event_number     = (bhi160b_meta_event_type_t)(*((*fifo_buffer) + 1));
            fifo_data_output->data_meta_event.sensor_type      = *((*fifo_buffer) + 2);
            fifo_data_output->data_meta_event.event_specific   = *((*fifo_buffer) + 3);
            break;
        case VS_ID_DEBUG:
            if ((*fifo_buffer_length) < _fifoSizes[BHI160B_DATA_TYPE_DEBUG])
            {
                return BHI160B_OUT_OF_RANGE;
            }

            (*fifo_data_type) = BHI160B_DATA_TYPE_DEBUG;
            fifo_data_output->data_debug.sensor_id   = (**fifo_buffer);
            fifo_data_output->data_debug.data[0]     = *((*fifo_buffer) + 1);
            fifo_data_output->data_debug.data[1]     = *((*fifo_buffer) + 2);
            fifo_data_output->data_debug.data[2]     = *((*fifo_buffer) + 3);
            fifo_data_output->data_debug.data[3]     = *((*fifo_buffer) + 4);
            fifo_data_output->data_debug.data[4]     = *((*fifo_buffer) + 5);
            fifo_data_output->data_debug.data[5]     = *((*fifo_buffer) + 6);
            fifo_data_output->data_debug.data[6]     = *((*fifo_buffer) + 7);
            fifo_data_output->data_debug.data[7]     = *((*fifo_buffer) + 8);
            fifo_data_output->data_debug.data[8]     = *((*fifo_buffer) + 9);
            fifo_data_output->data_debug.data[9]     = *((*fifo_buffer) + 10);
            fifo_data_output->data_debug.data[10]    = *((*fifo_buffer) + 11);
            fifo_data_output->data_debug.data[11]    = *((*fifo_buffer) + 12);
            fifo_data_output->data_debug.data[12]    = *((*fifo_buffer) + 13);
            break;
        case VS_ID_BSX_C:
        case VS_ID_BSX_B:
        case VS_ID_BSX_A:
            if ((*fifo_buffer_length) < _fifoSizes[BHI160B_DATA_TYPE_BSX])
            {
                return BHI160B_OUT_OF_RANGE;
            }

            (*fifo_data_type) = BHI160B_DATA_TYPE_BSX;
            fifo_data_output->data_bsx.sensor_id =  (**fifo_buffer);
            fifo_data_output->data_bsx.x =
            (uint32_t)(((uint32_t)*((*fifo_buffer) + 1)) | ((uint32_t)*((*fifo_buffer) + 2) << 8) |
              ((uint32_t)*((*fifo_buffer) + 3) << 16) | ((uint32_t)*((*fifo_buffer) + 4) << 24));
            fifo_data_output->data_bsx.y =
            (uint32_t)(((uint32_t)*((*fifo_buffer) + 5)) | ((uint32_t)*((*fifo_buffer) + 6) << 8) |
              ((uint32_t)*((*fifo_buffer) + 7) << 16) | ((uint32_t)*((*fifo_buffer) + 8) << 24));
            fifo_data_output->data_bsx.z =
            (uint32_t)(((uint32_t)*((*fifo_buffer) + 9)) | ((uint32_t)*((*fifo_buffer) + 10) << 8) |
              ((uint32_t)*((*fifo_buffer) + 11) << 16) | ((uint32_t)*((*fifo_buffer) + 12) << 24));
            fifo_data_output->data_bsx.timestamp =
            (uint32_t)(((uint32_t)*((*fifo_buffer) + 13)) | ((uint32_t)*((*fifo_buffer) + 14) << 8) |
              ((uint32_t)*((*fifo_buffer) + 15) << 16) | ((uint32_t)*((*fifo_buffer) + 16) << 24));
            break;

       case VS_ID_CUS1:
       case VS_ID_CUS2:
       case VS_ID_CUS3:
       case VS_ID_CUS4:
       case VS_ID_CUS5:
            (*fifo_data_type) = BHI160B_DATA_TYPE_CUS1+ **fifo_buffer - VS_ID_CUS1;

            if ((*fifo_buffer_length) < _fifoSizes[*fifo_data_type])
            {
                return BHI160B_OUT_OF_RANGE;
            }

            fifo_data_output->data_pdr.sensor_id   = (**fifo_buffer);

            for(i = 0; i < _fifoSizes[*fifo_data_type] - 1; i++)
            {
                fifo_data_output->data_custom.data[i] = *((*fifo_buffer) + i);
            }
            break;

       case VS_ID_CUS1_WAKEUP:
       case VS_ID_CUS2_WAKEUP:
       case VS_ID_CUS3_WAKEUP:
       case VS_ID_CUS4_WAKEUP:
       case VS_ID_CUS5_WAKEUP:
            (*fifo_data_type) = BHI160B_DATA_TYPE_CUS1+ **fifo_buffer - VS_ID_CUS1_WAKEUP;

            if ((*fifo_buffer_length) < _fifoSizes[*fifo_data_type])
            {
                return BHI160B_OUT_OF_RANGE;
            }

            fifo_data_output->data_pdr.sensor_id   = (**fifo_buffer);

            for(i = 0; i < _fifoSizes[*fifo_data_type]-1; i++)
                fifo_data_output->data_custom.data[i] = *((*fifo_buffer) + i);
            {
                fifo_data_output->data_custom.data[i] = *((*fifo_buffer) + i);
            }
            break;

        /* the VS sensor ID is unknown. Either the sync has been lost or the */
        /* ram patch implements a new sensor ID that this driver doesn't yet */
        /* support                               */
        default:
            return BHI160B_OUT_OF_RANGE;
    }

    (*fifo_buffer)         += _fifoSizes[*fifo_data_type];
    (*fifo_buffer_length)  -= _fifoSizes[*fifo_data_type];

    return BHI160B_SUCCESS;
};

int bhi160b_open(struct bhi160b_data *bhi160b)
{
    int result = BHI160B_SUCCESS;
    unsigned long rate;
    rate = 1000/atomic_read(&bhi160b->delay);

    result += bhi160b_enable_virtual_sensor(VS_TYPE_ACCELEROMETER, VS_WAKEUP, rate, 0, VS_FLUSH_NONE, 0, 0);
    if(result)
    {
        if (DBG_LOG) BHI_ERR("Fail to enable sensor id: %d\n", VS_TYPE_ACCELEROMETER);
    }
    result += bhi160b_enable_virtual_sensor(VS_TYPE_GYROSCOPE, VS_WAKEUP, rate, 0, VS_FLUSH_NONE, 0, 0);
    if(result)
    {
        if (DBG_LOG) BHI_ERR("Fail to enable sensor id: %d\n", VS_TYPE_GYROSCOPE);
    }
    if(!result)
    {
        BHI_INFO("Open virtual sensor successfully\n");
    }

    result += bhi160b_get_physical_sensor_status(&bhi160b->accel_info.physical_status, &bhi160b->gyro_info.physical_status);
    if (DBG_LOG) BHI_DBG("accelerometer present: %d, accelerometer active: %d, gyroscope present: %d, gyroscope active: %d\n",
                                                                  (bhi160b->accel_info.physical_status.accel_flag & 0x01),
                                                                  (bhi160b->accel_info.physical_status.accel_flag & 0xE0) >> 5,
                                                                  (bhi160b->gyro_info.physical_status.gyro_flag & 0x01),
                                                                  (bhi160b->gyro_info.physical_status.gyro_flag & 0xE0) >> 5);
    return result;
}

int bhi160b_close(struct bhi160b_data *bhi160b)
{
    int result = BHI160B_SUCCESS;
    unsigned long rate;
    rate = 1000/atomic_read(&bhi160b->delay);

    bhi160b->accel_info.non_wakeup_status.non_wakeup_sample_rate = rate;
    bhi160b->accel_info.non_wakeup_status.non_wakeup_max_report_latency = 0;
    bhi160b->accel_info.non_wakeup_status.non_wakeup_change_sensitivity = 0;
    bhi160b->accel_info.non_wakeup_status.non_wakeup_dynamic_range = 0;
    bhi160b->accel_info.sensor_type = VS_TYPE_ACCELEROMETER;
    bhi160b->accel_info.sensor_type += VS_WAKEUP + SENSOR_PARAMETER_WRITE;
    result += bhi160b_set_non_wakeup_sensor_configuration(&bhi160b->accel_info.non_wakeup_status, bhi160b->accel_info.sensor_type);
    if(result)
    {
        if (DBG_LOG) BHI_WARN("Fail to set the sensor to not wakeup, sensor_type: %d\n", VS_TYPE_ACCELEROMETER);
    }
    bhi160b->gyro_info.non_wakeup_status.non_wakeup_sample_rate = rate;
    bhi160b->gyro_info.non_wakeup_status.non_wakeup_max_report_latency = 0;
    bhi160b->gyro_info.non_wakeup_status.non_wakeup_change_sensitivity = 0;
    bhi160b->gyro_info.non_wakeup_status.non_wakeup_dynamic_range = 0;
    bhi160b->gyro_info.sensor_type = VS_TYPE_GYROSCOPE;
    bhi160b->gyro_info.sensor_type += VS_WAKEUP + SENSOR_PARAMETER_WRITE;
    result += bhi160b_set_non_wakeup_sensor_configuration(&bhi160b->gyro_info.non_wakeup_status, bhi160b->gyro_info.sensor_type);
    if(result)
    {
        if (DBG_LOG) BHI_WARN("Fail to set the sensor to not wakeup, sensor_type: %d\n", VS_TYPE_GYROSCOPE);
    }
    if(!result)
    {
        BHI_INFO("Close virtual sensor successfully\n");
    }

	return result;
}

int bhi160b_get_data(struct bhi160b_data *bhi160b)
{
    int count;
    int result = BHI160B_SUCCESS;
    uint8_t                    *fifoptr           = NULL;
    uint8_t                    bytes_left_in_fifo = 0;
    uint16_t                   bytes_read         = 0;
    uint16_t                   bytes_remaining    = 0;
    bhi160b_data_generic_t     fifo_packet;
    bhi160b_data_type_t        packet_type;
    bhi160b->accel_info.sensor_type = 0;
    bhi160b->gyro_info.sensor_type = 0;

    do
    {
        while(!interruptFired() && !bytes_remaining);

        result = bhi160b_read_fifo(fifo + bytes_left_in_fifo, FIFO_SIZE - bytes_left_in_fifo, &bytes_read, &bytes_remaining);
        bytes_read           += bytes_left_in_fifo;
        fifoptr              = fifo;
        packet_type          = BHI160B_DATA_TYPE_PADDING;

        do{
            result += bhi160b_parse_next_fifo_packet(&fifoptr, &bytes_read, &fifo_packet, &packet_type);
            if ((fifo_packet.data_vector.sensor_id) == VS_ID_ACCELEROMETER ||
                (fifo_packet.data_vector.sensor_id) == VS_ID_ACCELEROMETER_WAKEUP)
            {
                bhi160b->accel_info.sensor_type = VS_TYPE_ACCELEROMETER;
                bhi160b->accel_info.raw_data.x = fifo_packet.data_vector.x;
                bhi160b->accel_info.raw_data.y = fifo_packet.data_vector.y;
                bhi160b->accel_info.raw_data.z = fifo_packet.data_vector.z;
                if (DBG_LOG) BHI_DBG("accel raw data: %d %d %d  (result: %d)\n",
                bhi160b->accel_info.raw_data.x, bhi160b->accel_info.raw_data.y, bhi160b->accel_info.raw_data.z, result);
            }
            if ((fifo_packet.data_vector.sensor_id) == VS_ID_GYROSCOPE ||
                (fifo_packet.data_vector.sensor_id) == VS_ID_GYROSCOPE_WAKEUP)
            {
                bhi160b->gyro_info.sensor_type = VS_TYPE_GYROSCOPE;
                bhi160b->gyro_info.raw_data.x = fifo_packet.data_vector.x;
                bhi160b->gyro_info.raw_data.y = fifo_packet.data_vector.y;
                bhi160b->gyro_info.raw_data.z = fifo_packet.data_vector.z;
                if (DBG_LOG) BHI_DBG("gyro raw data: %d %d %d  (result: %d)\n",
                bhi160b->gyro_info.raw_data.x, bhi160b->gyro_info.raw_data.y, bhi160b->gyro_info.raw_data.z, result);
            }
        } while ((result == BHI160B_SUCCESS) && (bytes_read > (bytes_remaining ? MAX_PACKET_LENGTH : 0)));

        bytes_left_in_fifo = 0;

        if (bytes_remaining)
        {
            while (bytes_left_in_fifo < bytes_read)
            {
                fifo[bytes_left_in_fifo++] = 0;
            }
        }
    } while (((bhi160b->accel_info.sensor_type == 0) && (bhi160b->gyro_info.sensor_type == 0)));
    return result;
}
