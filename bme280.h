// Temperature calibration registers
#define CALIBRATION_REGISTER_DIG_T1 0x88
#define CALIBRATION_REGISTER_DIG_T2 0x8A
#define CALIBRATION_REGISTER_DIG_T3 0x8C

// Pressure calibration registers
#define CALIBRATION_REGISTER_DIG_P1 0x8E
#define CALIBRATION_REGISTER_DIG_P2 0x90
#define CALIBRATION_REGISTER_DIG_P3 0x92
#define CALIBRATION_REGISTER_DIG_P4 0x94
#define CALIBRATION_REGISTER_DIG_P5 0x96
#define CALIBRATION_REGISTER_DIG_P6 0x98
#define CALIBRATION_REGISTER_DIG_P7 0x9A
#define CALIBRATION_REGISTER_DIG_P8 0x9C
#define CALIBRATION_REGISTER_DIG_P9 0x9E

// Humidity calibration registers
#define CALIBRATION_REGISTER_DIG_H1 0xA1
#define CALIBRATION_REGISTER_DIG_H2 0xE1
#define CALIBRATION_REGISTER_DIG_H3 0xE3
#define CALIBRATION_REGISTER_DIG_H4 0xE4
#define CALIBRATION_REGISTER_DIG_H5 0xE5
#define CALIBRATION_REGISTER_DIG_H6 0xE7

// Contains the chip identification number chip_id[7:0] which is 0x60
#define BME280_REGISTER_ID 0xD0
#define BME280_CHIP_ID     0x60

// Contains the soft reset word reset[7:0].
// If the value 0xB6 is written to the register,
// the device is reset using the complete power-on-reset procedure
#define BME280_REGISTER_RESET 0xE0
#define BME280_RESET          0xB6

// Sets the humidity data acquisition options of the device. osrs_h[2:0]
// Changes to this register only become effective after a write operation to “ctrl_meas”.
#define BME280_REGISTER_CTRL_HUM 0xF2
#define BME280_OSRS_H_0   0x0
#define BME280_OSRS_H_1   0x1
#define BME280_OSRS_H_2   0x2
#define BME280_OSRS_H_4   0x3
#define BME280_OSRS_H_8   0x4
#define BME280_OSRS_H_16  0x5

// Contains two bits which indicate the status of the device.
#define BME280_REGISTER_STATUS  0xF3
#define BME280_STATUS_MEASURING 0x8
#define BME280_STATUS_IMUPDATE  0x1

// Sets the pressure and temperature data acquisition options of the device.
// The register needs to be written after changing “ctrl_hum” for the changes to become effective.
#define BME280_REGISTER_CTRL_MEAS 0xF4
#define BME280_OSRS_MODE_SLEEP    0x0
#define BME280_OSRS_MODE_FORCED   0x1
#define BME280_OSRS_MODE_NORMAL   0x3
#define BME280_OSRS_P_0  0x0
#define BME280_OSRS_P_1  0x4
#define BME280_OSRS_P_2  0x8
#define BME280_OSRS_P_4  0xC
#define BME280_OSRS_P_8  0x10
#define BME280_OSRS_P_16 0x14
#define BME280_OSRS_T_0  0x0
#define BME280_OSRS_T_1  0x20
#define BME280_OSRS_T_2  0x40
#define BME280_OSRS_T_4  0x60
#define BME280_OSRS_T_8  0x80
#define BME280_OSRS_T_16 0xA0

// Sets the rate, filter and interface options of the device
#define BME280_REGISTER_CONFIG 0xF5
#define BME280_SPI3W_EN   0x1
#define BME280_FILTER_OFF 0x0
#define BME280_FILTER_2   0x4
#define BME280_FILTER_4   0x8
#define BME280_FILTER_8   0xC
#define BME280_FILTER_16  0x10
#define BME280_T_SB_0_5   0x0
#define BME280_T_SB_62_5  0x20
#define BME280_T_SB_125   0x40
#define BME280_T_SB_250   0x60
#define BME280_T_SB_500   0x80
#define BME280_T_SB_1000  0xA0
#define BME280_T_SB_10    0xC0
#define BME280_T_SB_20    0xE0

// Contains the raw pressure measurement output data up[19:0]
#define BME280_REGISTER_PRESS_MSB  0xF7
#define BME280_REGISTER_PRESS_LSB  0xF8
#define BME280_REGISTER_PRESS_XLSB 0xF9

// Contains the raw temperature measurement output data ut[19:0]
#define BME280_REGISTER_TEMP_MSB  0xFA
#define BME280_REGISTER_TEMP_LSB  0xFB
#define BME280_REGISTER_TEMP_XLSB 0xFC

// Contains the raw temperature measurement output data ut[19:0]
#define BME280_REGISTER_HUM_MSB 0xFD
#define BME280_REGISTER_HUM_LSB 0xFE

struct bme280_calibration_t {
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;

    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;

    uint8_t dig_H1;
    int16_t dig_H2;
    uint8_t dig_H3;
    int16_t dig_H4;
    int16_t dig_H5;
    uint8_t dig_H6;
};
