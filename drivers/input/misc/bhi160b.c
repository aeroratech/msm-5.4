#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/sysfs.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/fs.h>

#include "bhi160b_hub.h"
#include "bhi160b.h"

extern int bhi160b_initialize(const uint8_t *bhi160b_hub_data);
extern int bhi160b_get_data(struct bhi160b_data *bhi160b);
extern int bhi160b_open(struct bhi160b_data *bhi160b);
extern int bhi160b_close(struct bhi160b_data *bhi160b);

struct mutex lock;
struct bhi160b_data *bhi160b;
enum bhi_log_level bhi_log_lvl = BHI_MSG_ERROR;

static ssize_t bhi160b_enable_store(struct device *dev, struct device_attribute *attr,
                const char *buf, size_t size)
{
    unsigned long enable;
    unsigned long delay;
    int ret;

    struct input_dev *input = to_input_dev(dev);
    bhi160b = input_get_drvdata(input);

    ret = kstrtoul(buf, 10, &enable);
    if (ret < 0)
    {
        if (DBG_LOG) BHI_ERR("bhi160b enable store error\n");
    }
    else
    {
        if (enable)
        {
            if (!bhi160b->is_enable)
            {
                ret =  bhi160b_open(bhi160b);
                bhi160b->is_enable = true;
                delay = msecs_to_jiffies(atomic_read(&bhi160b->delay));
                schedule_delayed_work(&bhi160b->work, delay);
            }
        }
        else
        {
            if (bhi160b->is_enable)
            {
                ret =  bhi160b_close(bhi160b);
                cancel_delayed_work_sync(&bhi160b->work);
                bhi160b->is_enable = false;
            }
        }
        ret = size;
    }

    return ret;
}

static ssize_t bhi160b_sample_period_store(struct device *dev, struct device_attribute *attr,
                const char *buf, size_t count)
{
    unsigned long delay;
    int ret;

    struct input_dev *input = to_input_dev(dev);
    struct bhi160b_data *data = input_get_drvdata(input);

    ret = kstrtoul(buf, 10, &delay);
    if (ret < 0) {
        if (DBG_LOG) BHI_ERR("bhi160b_sample_period_store kstrtoul error %d\n", ret);
        return ret;
    }

    mutex_lock(&lock);
    atomic_set(&data->delay, delay);
    mutex_unlock(&lock);

    return count;
}

static DEVICE_ATTR(bhi160b_enable, S_IWUSR, NULL, bhi160b_enable_store);
static DEVICE_ATTR(bhi160b_sample_period, S_IWUSR, NULL, bhi160b_sample_period_store);

static int bhi160b_i2cdev_ioctl(struct i2c_client *client,
        uint8_t read_write, uint8_t command, uint32_t size,
        union i2c_smbus_data *data)
{
    int datasize, res;

    /* Note that I2C_SMBUS_READ and I2C_SMBUS_WRITE are 0 and 1*/
    /* so the check is valid if size==I2C_SMBUS_QUICK too. */
    if ((read_write != I2C_SMBUS_READ) && (read_write != I2C_SMBUS_WRITE))
    {
        if (DBG_LOG) BHI_ERR("read_write out of range (%x) in ioctl I2C_SMBUS.\n", read_write);
        return -EINVAL;
    }

    if ((size == I2C_SMBUS_BYTE_DATA) || (size == I2C_SMBUS_BYTE))
        datasize = sizeof(data->byte);
    else if ((size == I2C_SMBUS_WORD_DATA) || (size == I2C_SMBUS_PROC_CALL))
        datasize = sizeof(data->word);
    else
        datasize = sizeof(data->block);
    /* size == smbus block, i2c block, or block proc. call */

    if ((size == I2C_SMBUS_PROC_CALL) || (size == I2C_SMBUS_BLOCK_PROC_CALL) ||
        (size == I2C_SMBUS_I2C_BLOCK_DATA) || (read_write == I2C_SMBUS_WRITE))
    {
        res = i2c_smbus_xfer(client->adapter, client->addr, client->flags,
          read_write, command, size, data);
    }

    return res;
}

static int i2c_rdwr_block(struct bhi160b_data *bhi160b, uint8_t reg, uint8_t read_write, uint16_t length, uint8_t* buffer)
{
    int i, rv;
    struct i2c_smbus_ioctl_data ioctl_data;
    union i2c_smbus_data smbus_data;

    if(length > I2C_SMBUS_BLOCK_MAX)
    {
        if (DBG_LOG) BHI_ERR("Requested Length is greater than the maximum specified\n");
        return -1;
    }

    smbus_data.block[0] = (uint8_t)length;

    if ( read_write != I2C_SMBUS_READ )
    {
        for(i = 0; i < length; i++)
        {
            smbus_data.block[i + 1] = buffer[i];
        }
    }

    ioctl_data.read_write = read_write;
    ioctl_data.command = reg;
    ioctl_data.size = I2C_SMBUS_I2C_BLOCK_DATA;
    ioctl_data.data = &smbus_data;

    rv = bhi160b_i2cdev_ioctl(bhi160b->client, ioctl_data.read_write, ioctl_data.command, ioctl_data.size, ioctl_data.data);
    if (rv < 0)
    {
        if (DBG_LOG) BHI_ERR("Accessing I2C Read/Write failed!\n");
        return rv;
    }

    if (read_write == I2C_SMBUS_READ)
    {
        for(i = 0; i < length; i++)
        {
            // Skip the first byte, which is the length of the rest of the block.
            buffer[i] = smbus_data.block[i+1];
        }
    }

    return rv;
}

int read_reg(uint8_t reg, uint8_t* buffer, uint16_t length)
{
    int block_num = 0, reg_addr = reg;
    int remaining_bytes = length;
    char temp_buf[I2C_SMBUS_BLOCK_MAX] = {0,};
    int i, rv;

    do
    {
        int bytes_to_read = 0;
        if (remaining_bytes > I2C_SMBUS_BLOCK_MAX)
        {
            bytes_to_read = I2C_SMBUS_BLOCK_MAX;
        }
        else{
            bytes_to_read = remaining_bytes;
        }

        if (reg < 50)
        {
            // if (DBG_LOG) BHI_DBG("read %u bytes @ FIFO offset 0x%x", bytes_to_read, reg_addr\n);
        }
        rv = i2c_rdwr_block(bhi160b, reg_addr, I2C_SMBUS_READ, bytes_to_read, temp_buf);
        if (rv < 0)
        {
            if (DBG_LOG) BHI_ERR("Reading I2C Bus Error...\n");
            return -1;
        }

        for (i = 0; i < bytes_to_read; i++)
        {
            buffer[I2C_SMBUS_READ * block_num + i] = temp_buf[i];
        }
        block_num++;
        remaining_bytes -= I2C_SMBUS_BLOCK_MAX;
        reg_addr += I2C_SMBUS_BLOCK_MAX;
        reg_addr %= BHI160B_I2C_REG_BUFFER_LENGTH;
    } while (remaining_bytes > 0);

    return rv;
}

int write_reg(uint8_t reg, uint8_t* buffer, uint16_t length)
{
    int rv = i2c_rdwr_block(bhi160b, reg, I2C_SMBUS_WRITE, length, buffer);
    if (rv < 0)
    {
        if (DBG_LOG) BHI_DBG("Writing I2C Bus Error...\n");
        return -1;
    }
    return rv;
}

static int bhi160b_read_bytes_remaining(uint16_t *v_bytes_remaining_u16)
{
    int com_rslt = BHI160B_COMM_RES;
    uint8_t v_data_u8[] = {0,0};

    com_rslt = read_reg(BHI160B_BYTES_REMAINING, v_data_u8, 2);
    /* get the bytes remaining data*/
    *v_bytes_remaining_u16 = (uint16_t)((v_data_u8[1]<< 8) | (v_data_u8[0]));
    return com_rslt;
}

int bhi160b_read_fifo(uint8_t *buffer, uint16_t buffer_size, uint16_t *bytes_read, uint16_t *bytes_left)
{
    int result = BHI160B_SUCCESS;
    static uint16_t current_index = 0;
    static uint16_t current_transaction_size = 0;

    // if (DBG_LOG) BHI_DBG("called with buffer=0x%x buflen=%u, initial index=%u transaction size=%u\n",
    //        *buffer, buffer_size, current_index, current_transaction_size);

    if (buffer_size <= BHI160B_I2C_REG_BUFFER_LENGTH)
    {
        return BHI160B_OUT_OF_RANGE;
    }

    /* gets the number of bytes left in the fifo either from memory of from */
    /* the register                                                         */
    if (current_transaction_size == 0)
    {
        result = bhi160b_read_bytes_remaining(&current_transaction_size);
        if (DBG_LOG) BHI_DBG("new transaction size %u (result %d)\n", current_transaction_size, result);
    }

    /* if there are bytes in the fifo to read */
    if (current_transaction_size)
    {
        /* calculates the number of bytes to read. either the number of     */
        /* bytes left, or the buffer size, or just enough so the last page  */
        /* does not get turned                                              */
        if (buffer_size >= current_transaction_size - current_index)
        {
            *bytes_read = current_transaction_size - current_index;
            if (DBG_LOG) BHI_DBG("buffer can hold %u remaining bytes, grab them all (got %u so far)\n",
                  *bytes_read, current_index);
        }
        else if (current_transaction_size - (current_index+buffer_size) <= BHI160B_I2C_REG_BUFFER_LENGTH)
        {
            *bytes_read = (current_transaction_size - (BHI160B_I2C_REG_BUFFER_LENGTH + 1)) - current_index;
            if (DBG_LOG) BHI_DBG("buffer too small, partial fill with %u (got %u so far)\n",
                  *bytes_read, current_index);
        }
        else
        {
            *bytes_read = buffer_size;
            if (DBG_LOG) BHI_DBG("buffer is too small, fill it with %u (got %u so far)\n",
                  *bytes_read, current_index);
        }

        if (DBG_LOG) BHI_DBG("READ %u bytes starting at offset %u (bytes_left=%u)\n",
              *bytes_read, current_index % BHI160B_I2C_REG_BUFFER_LENGTH,
              current_transaction_size - current_index);
        result += read_reg(current_index % BHI160B_I2C_REG_BUFFER_LENGTH, buffer, *bytes_read);

        current_index += *bytes_read;

        (*bytes_left) = current_transaction_size - current_index;

        if ((*bytes_left) == 0)
        {
            current_index = 0;
            current_transaction_size = 0;
        }

    }
    else
    {
        (*bytes_read) = 0;
        (*bytes_left) = 0;
        return result;
    }

    return result;
}

static int bhi160b_check_device(struct bhi160b_data *bhi160b)
{
    int result = BHI160B_ERROR;
    uint8_t bhi160b_val = BHI160B_INIT_VALUE;

    result = i2c_rdwr_block(bhi160b, BHI160B_PRODUCT_ID_ADDR, I2C_SMBUS_READ, 1, &bhi160b_val);
    if(result < 0)
    {
        if (DBG_LOG) BHI_ERR("i2c read error\n");
        return result;
    }
    bhi160b->device_id = bhi160b_val;

    return result;
}

static void bhi160b_work_func(struct work_struct *work)
{
    int32_t ret;
    unsigned long delay;
    bhi160b = container_of((struct delayed_work *)work, struct bhi160b_data, work);
    delay = msecs_to_jiffies(atomic_read(&bhi160b->delay));

    ret = bhi160b_get_data(bhi160b);
    if ((ret != BHI160B_SUCCESS) || ((bhi160b->accel_info.sensor_type != VS_TYPE_ACCELEROMETER) ||
       (bhi160b->gyro_info.sensor_type != VS_TYPE_GYROSCOPE)))
        goto skip_report;

    input_event(bhi160b->input, EV_MSC, MSC_SERIAL, bhi160b->accel_info.raw_data.x);
    input_event(bhi160b->input, EV_MSC, MSC_PULSELED, bhi160b->accel_info.raw_data.y);
    input_event(bhi160b->input, EV_MSC, MSC_GESTURE, bhi160b->accel_info.raw_data.z);
    input_event(bhi160b->input, EV_MSC, MSC_RAW, bhi160b->gyro_info.raw_data.x);
    input_event(bhi160b->input, EV_MSC, MSC_SCAN, bhi160b->gyro_info.raw_data.y);
    input_event(bhi160b->input, EV_MSC, MSC_TIMESTAMP, bhi160b->gyro_info.raw_data.z);
    input_sync(bhi160b->input);

skip_report:
    schedule_delayed_work(&bhi160b->work, delay);
}

static void bhi160b_init_input_device(struct bhi160b_data *bhi160b,
                        struct input_dev *idev)
{
    idev->name = "BHI160B";
    idev->phys = "BHI160B/input0";
    idev->id.bustype = BUS_I2C;
    idev->dev.parent = &bhi160b->client->dev;

    set_bit(EV_MSC, idev->evbit);
    set_bit(MSC_SERIAL, idev->mscbit);
    set_bit(MSC_PULSELED, idev->mscbit);
    set_bit(MSC_GESTURE, idev->mscbit);
    set_bit(MSC_RAW, idev->mscbit);
    set_bit(MSC_SCAN, idev->mscbit);
    set_bit(MSC_TIMESTAMP, idev->mscbit);
}

static int bhi160b_register_input_device(struct bhi160b_data *bhi160b)
{
    struct input_dev *idev;
    int error;

    idev = input_allocate_device();
    if (!idev)
        return -ENOMEM;

    bhi160b_init_input_device(bhi160b, idev);

    input_set_drvdata(idev, bhi160b);
    bhi160b->input = idev;

    error = input_register_device(idev);
    if (error) {
        input_free_device(idev);
        if (DBG_LOG) BHI_ERR("register input device error\n");
        return error;
    }

    return 0;
}

static int bhi160b_probe(struct i2c_client *client,
                        const struct i2c_device_id *id)
{
    int result = BHI160B_ERROR;

    if (!i2c_check_functionality(client->adapter, (I2C_FUNC_I2C | I2C_FUNC_SMBUS_BYTE)))
    {
        if (DBG_LOG) BHI_ERR("device doesn't support I2C\n");
        return -ENODEV;
    }

    bhi160b = devm_kzalloc(&client->dev,
        sizeof(struct bhi160b_data), GFP_KERNEL);
    if (!bhi160b)
        return -ENOMEM;

    i2c_set_clientdata(client, bhi160b);
    bhi160b->client = client;
    result = bhi160b_check_device(bhi160b);
    if(result < 0)
    {
        if (DBG_LOG) BHI_ERR("Can not get device id\n");
    }

    if (bhi160b->device_id != PRODUCT_ID_7183)
    {
        if (DBG_LOG) BHI_ERR("BHI160B device id error: %d\n", bhi160b->device_id);
        return -EINVAL;
    }

    result += device_create_file(&(client->dev), &dev_attr_bhi160b_enable);
    result += device_create_file(&(client->dev), &dev_attr_bhi160b_sample_period);
    if (result)
    {
        if (DBG_LOG) BHI_ERR("sys file create failed\n");
        device_remove_file(&(client->dev), &dev_attr_bhi160b_enable);
        device_remove_file(&(client->dev), &dev_attr_bhi160b_sample_period);
    }

    result += bhi160b_initialize(bhi160b_di03);
    if(result)
    {
        if (DBG_LOG) BHI_ERR("Fail to init bhi160b_sensorhub\n");
        return result;
    }

    result += bhi160b_register_input_device(bhi160b);
    if(result)
    {
        if (DBG_LOG) BHI_ERR("Fail to register input device\n");
        return result;
    }
    mutex_init(&lock);
    bhi160b->is_enable = false;

    INIT_DELAYED_WORK(&bhi160b->work, bhi160b_work_func);
    atomic_set(&bhi160b->delay, BHI160B_DELAY_DEFAULT);
    return 0;
}

static struct i2c_device_id bhi160b_id[] = {
    { "bhi160b", 0  },
    { }
};

static const struct of_device_id bhi160b_match_table[] = {
    {.compatible = "bosch-sensortec,bhi160b"},
    {.compatible = "bhi,bhi160b"},
    {.compatible = "bhi160b"},
    { }
};

MODULE_DEVICE_TABLE(of, bhi160b_match_table);

static struct i2c_driver bhi160b_driver = {
    .probe = bhi160b_probe,
    .driver    = {
        .name = "BHI160B",
        .of_match_table    = bhi160b_match_table,
    },
    .id_table = bhi160b_id,
};

module_i2c_driver(bhi160b_driver);
