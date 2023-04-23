#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <asm/div64.h>
#include <linux/math64.h>
#include "bme280.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Aleksei Rogov");
MODULE_DESCRIPTION("BME280 driver");
// MODULE_SUPPORTED_DEVICE("NONE");

static dev_t dev_num;
static struct class *my_class;
static struct cdev my_device;
#define DRIVER_NAME "bme280"
#define DRIVER_CLASS "bme280_class"

static struct i2c_adapter *bme_i2c_adapter = NULL;
static struct i2c_client *bme280_i2c_client = NULL;

#define I2C_BUS_AVAILABLE 1
#define SLAVE_DEVICE_NAME "BME280"
#define BME280_SLAVE_ADDRESS 0x76

static const struct i2c_device_id bme_id[] = {
    { SLAVE_DEVICE_NAME, 0},
    {}
};

static struct i2c_driver bme_driver = {
    .driver = {
        .name = SLAVE_DEVICE_NAME,
        .owner = THIS_MODULE
    }
};

static struct i2c_board_info bme_board_info = {
    I2C_BOARD_INFO(SLAVE_DEVICE_NAME, BME280_SLAVE_ADDRESS)
};

struct bme280_calibration_t bme280_calibration_t;
int32_t t_fine;

// Returns temperature in DegC, resolution is 0.01 DegC.
// Output value of “5123” equals 51.23 DegC.
int32_t bme280_read_compensate_temperature_int32(void) {
    int32_t adc_T, var1, var2, t;
    int32_t d1, d2, d3;

    // Read temperature register
    d1 = i2c_smbus_read_byte_data(bme280_i2c_client, BME280_REGISTER_TEMP_MSB);
    d2 = i2c_smbus_read_byte_data(bme280_i2c_client, BME280_REGISTER_TEMP_LSB);
    d3 = i2c_smbus_read_byte_data(bme280_i2c_client, BME280_REGISTER_TEMP_XLSB);
    adc_T = (d1 << 12) | (d2 << 4) | (d3 >> 4);

    // Calculate temperature according the "4.2.3 Compensation formulas"
    var1 = ((((adc_T >> 3) - ((int32_t)bme280_calibration_t.dig_T1 << 1))) * ((int32_t)bme280_calibration_t.dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)bme280_calibration_t.dig_T1)) * ((adc_T >> 4) - ((int32_t)bme280_calibration_t.dig_T1))) >> 12) * ((int32_t)bme280_calibration_t.dig_T3)) >> 14;
    t_fine = var1 + var2;
    t = (t_fine * 5 + 128) >> 8;
    return t;
}


// Returns pressure in Pa as unsigned 32 bit integer.
// Output value of “96386” equals 96386 Pa = 963.86 hPa
uint32_t bme280_read_compensate_pressure_int32(void) {
    uint32_t p;
    int32_t adc_P, d1, d2, d3, var1, var2;

    d1 = i2c_smbus_read_byte_data(bme280_i2c_client, BME280_REGISTER_PRESS_MSB);
    d2 = i2c_smbus_read_byte_data(bme280_i2c_client, BME280_REGISTER_PRESS_LSB);
    d3 = i2c_smbus_read_byte_data(bme280_i2c_client, BME280_REGISTER_PRESS_XLSB);
    adc_P = (d1 << 12) | (d2 << 4) | (d3 >> 4);

    var1 = (((int32_t)t_fine) >> 1) - (int32_t)64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11 ) * ((int32_t)bme280_calibration_t.dig_P6);
    var2 = var2 + ((var1 * ((int32_t)bme280_calibration_t.dig_P5)) << 1);
    var2 = (var2 >> 2) + (((int32_t)bme280_calibration_t.dig_P4) << 16);
    var1 = (((bme280_calibration_t.dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13 )) >> 3) + ((((int32_t)bme280_calibration_t.dig_P2) * var1) >> 1)) >> 18;
    var1 =((((32768 + var1)) * ((int32_t)bme280_calibration_t.dig_P1)) >> 15);
    if (var1 == 0) {
        return 0;
    }
    p = (((uint32_t)(((int32_t)1048576) - adc_P) - (var2 >> 12))) * 3125;
    if (p < 0x80000000) {
        p = (p << 1) / ((uint32_t)var1);
    }
    else {
        p = (p / (uint32_t)var1) * 2;
    }
    var1 = (((int32_t)bme280_calibration_t.dig_P9) * ((int32_t)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
    var2 = (((int32_t)(p >> 2)) * ((int32_t)bme280_calibration_t.dig_P8)) >> 13;
    p = (uint32_t)((int32_t)p + ((var1 + var2 + bme280_calibration_t.dig_P7) >> 4));
    return p;
}


// Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format (22 integer and 10fractional bits).
// Output value of “47445” represents 47445/1024 = 46.333 %RH
uint32_t bme280_read_compensate_humidity_int32(void) {
    int32_t v_x1_u32r, adc_H;
    int32_t d1, d2;

    d1 = i2c_smbus_read_byte_data(bme280_i2c_client, BME280_REGISTER_HUM_MSB);
    d2 = i2c_smbus_read_byte_data(bme280_i2c_client, BME280_REGISTER_HUM_LSB);
    adc_H = (d1 << 8) | d2;

    v_x1_u32r = (t_fine - ((int32_t)76800));
    v_x1_u32r = (((((adc_H << 14) - (((int32_t)bme280_calibration_t.dig_H4) << 20) - (((int32_t)bme280_calibration_t.dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)bme280_calibration_t.dig_H6)) >> 10) * (((v_x1_u32r * ((int32_t)bme280_calibration_t.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)bme280_calibration_t.dig_H2) + 8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)bme280_calibration_t.dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    return (uint32_t)(v_x1_u32r >> 12);
}

static ssize_t driver_read(struct file *fd, char *user_buffer, size_t count, loff_t *offs) {
    int to_copy, not_copied, delta;
    int32_t temperature, pressure, humidity, bytes;
    char buff[512];

    to_copy = min(sizeof(buff), count);

    temperature = bme280_read_compensate_temperature_int32(); // 1234 = 12.34
    pressure = bme280_read_compensate_pressure_int32();
    humidity = bme280_read_compensate_humidity_int32();

    bytes = snprintf(buff, sizeof(buff), "{\"temperature\": %d.%d, \"pressure\": %d, \"humidity\": %d.%d}\n",\
             temperature / 100, temperature % 100, pressure, humidity >> 10, ((humidity % 1024) * 100) >> 10 );

    //not_copied = copy_to_user(user_buffer, buff, to_copy);
    not_copied = copy_to_user(user_buffer, buff, bytes);

    // delta = to_copy - not_copied;
    delta = bytes - not_copied;
    printk("driver_read has been called. count: %d, to_copy: %d, not_copied: %d, delta: %d\n", count, to_copy, not_copied, delta);
    return delta;
}


static int driver_open(struct inode *device_file, struct file *instance) {
    printk("bme280 driver_open called\n");
    return 0;
}


static int driver_close(struct inode *device_file, struct file *instance) {
    printk("bme280 driver_close called\n");
    return 0;
}


static struct file_operations fops = {
    .owner = THIS_MODULE,
    .open = driver_open,
    .release = driver_close,
    .read = driver_read,
};


static int __init bme280_init(void) {
    int ret;
    uint8_t id, e4, e5, e6;
    ret = -1;
    printk(KERN_INFO "BME280 module loaded\n");
    if(alloc_chrdev_region(&dev_num, 0, 1, DRIVER_NAME) < 0){
        printk("Device Nr could not be allocated\n");
        return -1;
    }
    printk("read_write - device Nr Major: %d, minor: %d has been registred\n", dev_num >> 20, dev_num && 0xfffff);

    if((my_class = class_create(THIS_MODULE, DRIVER_CLASS)) == NULL) {
        printk("Device class can not be crated\n");
        goto ClassError;
    }

    if(device_create(my_class, NULL, dev_num, NULL, DRIVER_NAME) == NULL) {
        printk("Can not create device file\n");
        goto FileError;
    }

    cdev_init(&my_device, &fops);

    if(cdev_add(&my_device, dev_num, 1) == -1) {
        printk("Registration of device to kernel failed\n");
        goto AddError;
    }

    bme_i2c_adapter = i2c_get_adapter(I2C_BUS_AVAILABLE);

    if(bme_i2c_adapter != NULL) {
        bme280_i2c_client = i2c_new_client_device(bme_i2c_adapter, &bme_board_info);
        if(bme280_i2c_client != NULL) {
            if(i2c_add_driver(&bme_driver) != -1) {
                ret = 0;
            }
            else {
                printk("Can't add driver\n");
            }
        }
        i2c_put_adapter(bme_i2c_adapter);
    }
    printk("BME280 driver added\n");

    id = i2c_smbus_read_byte_data(bme280_i2c_client, 0xD0);
    printk("ID: 0x%X\n", id);

    // Read calibration Values
    bme280_calibration_t.dig_T1 = i2c_smbus_read_word_data(bme280_i2c_client, CALIBRATION_REGISTER_DIG_T1);
    bme280_calibration_t.dig_T2 = i2c_smbus_read_word_data(bme280_i2c_client, CALIBRATION_REGISTER_DIG_T2);
    bme280_calibration_t.dig_T3 = i2c_smbus_read_word_data(bme280_i2c_client, CALIBRATION_REGISTER_DIG_T3);

    bme280_calibration_t.dig_P1 = i2c_smbus_read_word_data(bme280_i2c_client, CALIBRATION_REGISTER_DIG_P1);
    bme280_calibration_t.dig_P2 = i2c_smbus_read_word_data(bme280_i2c_client, CALIBRATION_REGISTER_DIG_P2);
    bme280_calibration_t.dig_P3 = i2c_smbus_read_word_data(bme280_i2c_client, CALIBRATION_REGISTER_DIG_P3);
    bme280_calibration_t.dig_P4 = i2c_smbus_read_word_data(bme280_i2c_client, CALIBRATION_REGISTER_DIG_P4);
    bme280_calibration_t.dig_P5 = i2c_smbus_read_word_data(bme280_i2c_client, CALIBRATION_REGISTER_DIG_P5);
    bme280_calibration_t.dig_P6 = i2c_smbus_read_word_data(bme280_i2c_client, CALIBRATION_REGISTER_DIG_P6);
    bme280_calibration_t.dig_P7 = i2c_smbus_read_word_data(bme280_i2c_client, CALIBRATION_REGISTER_DIG_P7);
    bme280_calibration_t.dig_P8 = i2c_smbus_read_word_data(bme280_i2c_client, CALIBRATION_REGISTER_DIG_P8);
    bme280_calibration_t.dig_P9 = i2c_smbus_read_word_data(bme280_i2c_client, CALIBRATION_REGISTER_DIG_P9);

    bme280_calibration_t.dig_H1 = i2c_smbus_read_byte_data(bme280_i2c_client, CALIBRATION_REGISTER_DIG_H1);
    bme280_calibration_t.dig_H2 = i2c_smbus_read_word_data(bme280_i2c_client, CALIBRATION_REGISTER_DIG_H2);
    bme280_calibration_t.dig_H3 = i2c_smbus_read_byte_data(bme280_i2c_client, CALIBRATION_REGISTER_DIG_H3);

    e4 = i2c_smbus_read_byte_data(bme280_i2c_client, CALIBRATION_REGISTER_DIG_H4);
    e5 = i2c_smbus_read_byte_data(bme280_i2c_client, CALIBRATION_REGISTER_DIG_H5);
    e6 = i2c_smbus_read_byte_data(bme280_i2c_client, CALIBRATION_REGISTER_DIG_H5 + 1);
    bme280_calibration_t.dig_H4 = (e4 << 4) | (e5 & 0xF);
    bme280_calibration_t.dig_H5 = (e6 << 4) | ((e5 >> 4) & 0xF);

    bme280_calibration_t.dig_H6 = i2c_smbus_read_byte_data(bme280_i2c_client, CALIBRATION_REGISTER_DIG_H6);

    printk("H1: %d, H2: %d, H3: %d, H4: %d, H5: %d, H6: %d\n", bme280_calibration_t.dig_H1, bme280_calibration_t.dig_H2,\
                                                               bme280_calibration_t.dig_H3, bme280_calibration_t.dig_H4,\
                                                               bme280_calibration_t.dig_H5, bme280_calibration_t.dig_H6);

    // Initialize device
    i2c_smbus_write_byte_data(bme280_i2c_client, BME280_REGISTER_CTRL_HUM, BME280_OSRS_H_1);
    i2c_smbus_write_byte_data(bme280_i2c_client, BME280_REGISTER_CTRL_MEAS, BME280_OSRS_MODE_NORMAL | BME280_OSRS_P_1 | BME280_OSRS_T_1);
    i2c_smbus_write_byte_data(bme280_i2c_client, BME280_REGISTER_CONFIG, BME280_T_SB_1000);

    return ret;

    AddError:
        device_destroy(my_class, dev_num);
    FileError:
        class_destroy(my_class);
    ClassError:
        unregister_chrdev(dev_num, DRIVER_NAME);
        return -1;
}


static void __exit bme280_exit(void) {
    i2c_unregister_device(bme280_i2c_client);
    i2c_del_driver(&bme_driver);
    device_destroy(my_class, dev_num);
    class_destroy(my_class);
    cdev_del(&my_device);
    unregister_chrdev(dev_num, DRIVER_NAME);
    printk(KERN_INFO "bme280 module unloaded\n");
}


module_init(bme280_init);
module_exit(bme280_exit);
