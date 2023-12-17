/*******************************************************************************
 * @file    ads1115.c
 * @brief   Implements functions for the ADS1115 module (analog-to-digital converter)
 * @author  Eng. Fabiano Bozzi D'Acunti
 * @li      fabiano.dcti@gmail.com
 * @version 0.1.0
 * @warning These features are under development and may not provide accurate results.
 ******************************************************************************/

/*******************************************************************************
* Include
*******************************************************************************/
#include "ads1115.h"

/******************************************************************************
* Private Function Prototypes
*******************************************************************************/
static int null_handle_check(const ads1115_handle_t *dev);
//static int null_info_check(const ads1115_info_t *info);
static int i2c_read_chip_register(ads1115_handle_t *dev, uint8_t reg, uint16_t *data);
static int i2c_write_chip_register(ads1115_handle_t *dev, uint8_t reg, uint16_t data);

/*******************************************************************************
* Private Macros
*******************************************************************************/
/**
 * @brief Chip information definition
 */
#define CHIP_NAME           "ADS1115"           /**< Chip name */
#define MANUFACTURER_NAME   "Texas Instruments" /**< Manufacturer name */
#define INTERFACE_TYPE      "I2C"               /**< Interface type */
#define SUPPLY_VOLTAGE_MIN  (2.0f)              /**< Chip min supply voltage */
#define SUPPLY_VOLTAGE_MAX  (5.5f)              /**< Chip max supply voltage */
#define CURRENT_MAX         (0.2f)              /**< Chip max current */
#define TEMPERATURE_MIN     (-40.0f)            /**< Chip min operating temperature */
#define TEMPERATURE_MAX     (125.0f)            /**< Chip max operating temperature */

/**
 * @brief I2C slave address definition
 */
#define ADS1115_ADDRESS_GND     (0x48 << 1U) /**< Slave Address = 0b1001000 (0x48) */
#define ADS1115_ADDRESS_VDD     (0x49 << 1U) /**< Slave Address = 0b1001001 (0x49) */
#define ADS1115_ADDRESS_SDA     (0x4A << 1U) /**< Slave Address = 0b1001010 (0x4A) */
#define ADS1115_ADDRESS_SCL     (0x4B << 1U) /**< Slave Address = 0b1001011 (0x4B) */

/**
 * @brief Device registers definition
 */
#define ADS1115_REG_CONVERSION  0x00    /**< ADC result register */
#define ADS1115_REG_CONFIG      0x01    /**< Chip config register */
#define ADS1115_REG_LO_THRESH   0x02    /**< Interrupt low threshold register */
#define ADS1115_REG_HI_THRESH   0x03    /**< Interrupt high threshold register */

/*******************************************************************************
* Chip config register
*******************************************************************************/
/* 
 * OS[15]: Operational status/single-shot conversion start
 * - write:
 * 0: No effect
 * 1: Begins a single conversion (when in power-down mode)
 * - read:
 * 0: Device is currently performing a conversion
 * 1: Device conversion completed
 */
#define ADS1115_OS_POS          (15U)
#define ADS1115_OS_MASK         (0x01)
#define ADS1115_START_SINGLE_CONVERSION (0x01 << ADS1115_OS_POS)

/*
 * MUX[14:12]: Input multiplexer configuration
 * 000: AINp = AIN0 and AINn = AIN1 (default)
 * 001: AINp = AIN0 and AINn = AIN3
 * 010: AINp = AIN1 and AINn = AIN3
 * 011: AINp = AIN2 and AINn = AIN3
 * 100: AINp = AIN0 and AINn = GND
 * 101: AINp = AIN1 and AINn = GND
 * 110: AINp = AIN2 and AINn = GND
 * 111: AINp = AIN3 and AINn = GND
 */
#define ADS1115_MUX_POS         (12U)
#define ADS1115_MUX_MASK        (0x07)

#define ADS1115_MUX_AIN0_AIN1   (0x00 << ADS1115_MUX_POS)
#define ADS1115_MUX_AIN0_AIN3   (0x01 << ADS1115_MUX_POS)
#define ADS1115_MUX_AIN1_AIN3   (0x02 << ADS1115_MUX_POS)
#define ADS1115_MUX_AIN2_AIN3   (0x03 << ADS1115_MUX_POS)
#define ADS1115_MUX_AIN0_GND    (0x04 << ADS1115_MUX_POS)
#define ADS1115_MUX_AIN1_GND    (0x05 << ADS1115_MUX_POS)
#define ADS1115_MUX_AIN2_GND    (0x06 << ADS1115_MUX_POS)
#define ADS1115_MUX_AIN3_GND    (0x07 << ADS1115_MUX_POS)

/*
 * PGA[11:9]: Programmable gain amplifier configuration
 * 000: 6.144V PGA gain
 * 001: 4.096V PGA gain
 * 010: 2.048V PGA gain (default)
 * 011: 1.024V PGA gain
 * 100: 0.512V PGA gain
 * 101: 0.256V PGA gain
 * 110: 0.256V PGA gain
 * 111: 0.256V PGA gain
 */
#define ADS1115_PGA_POS         (9U)
#define ADS1115_PGA_MASK        (0x07)

#define ADS1115_PGA_6144        (0x00 << ADS1115_PGA_POS)
#define ADS1115_PGA_4096        (0x01 << ADS1115_PGA_POS)
#define ADS1115_PGA_2048        (0x02 << ADS1115_PGA_POS)
#define ADS1115_PGA_1024        (0x03 << ADS1115_PGA_POS)
#define ADS1115_PGA_0512        (0x04 << ADS1115_PGA_POS)
#define ADS1115_PGA_0256        (0x05 << ADS1115_PGA_POS)
#define ADS1115_PGA_0256_110    (0x06 << ADS1115_PGA_POS)
#define ADS1115_PGA_0256_111    (0x07 << ADS1115_PGA_POS)

/* 
 * MODE[8]: Device operating mode
 * 0: Continuous conversion mode
 * 1: Power-down single-shot mode(default)
 */
#define ADS1115_MODE_POS        (8U)
#define ADS1115_MODE_MASK       (0x01)

#define ADS1115_MODE_CONTINUOUS (0x00 << ADS1115_MODE_POS)
#define ADS1115_MODE_SINGLESHOT (0x01 << ADS1115_MODE_POS)

/*
 * DR[7:5]: Data rate
 * These bits control the data rate setting.
 * 000: 8 samples per second
 * 001: 16 samples per second
 * 010: 32 samples per second
 * 011: 64 samples per second
 * 100: 128 samples per second (default)
 * 101: 250 samples per second
 * 110: 475 samples per second
 * 111: 860 samples per second
 */
#define ADS1115_DR_POS       (5U)
#define ADS1115_DR_MASK      (0x07)

#define ADS1115_DR_8SPS      (0x00 << ADS1115_DR_POS)
#define ADS1115_DR_16SPS     (0x01 << ADS1115_DR_POS)
#define ADS1115_DR_32SPS     (0x02 << ADS1115_DR_POS)
#define ADS1115_DR_64SPS     (0x03 << ADS1115_DR_POS)
#define ADS1115_DR_128SPS    (0x04 << ADS1115_DR_POS)
#define ADS1115_DR_250SPS    (0x05 << ADS1115_DR_POS)
#define ADS1115_DR_475SPS    (0x06 << ADS1115_DR_POS)
#define ADS1115_DR_860SPS    (0x07 << ADS1115_DR_POS)

/*
 * COMP_MODE[4]: Comparator mode
 * 0: Traditional comparator with hysteresis (default)
 * 1: Window comparator
 */
#define ADS1115_COMP_MODE_POS           (4U)
#define ADS1115_COMP_MODE_MASK          (0x01)

#define ADS1115_COMP_MODE_HYSTERESIS    (0x00 << ADS1115_COMP_MODE_POS)
#define ADS1115_COMP_MODE_WINDOW        (0x01 << ADS1115_COMP_MODE_POS)

/*
 * COMP_POL[3]: Comparator polarity (ALERT/RDY pin)
 * 0: ALERT/RDY pin active low (default)
 * 1: ALERT/RDY active high
 */
#define ADS1115_COMP_POL_POS    (3U)
#define ADS1115_COMP_POL_MASK   (0x01)

#define ADS1115_COMP_POL_LOW    (0x00 << ADS1115_COMP_POL_POS)
#define ADS1115_COMP_POL_HIGH   (0x01 << ADS1115_COMP_POL_POS)

/*
 * COMP_LAT[2]: Latching comparator (ALERT/RDY pin)
 * 0: ALERT/RDY pin does not latch when asserted (default)
 * 1: ALERT/RDY pin remains latched until conversion data are read by the master
 */
#define ADS1115_COMP_LAT_POS            (2U)
#define ADS1115_COMP_LAT_MASK           (0x01)

#define ADS1115_COMP_LAT_NO_LATCHING    (0x00 << ADS1115_COMP_LAT_POS)
#define ADS1115_COMP_LAT_LATCHING       (0x01 << ADS1115_COMP_LAT_POS)

/*
 * COMP_QUE[1:0]: Latching comparator
 * These bits perform two functions.
 * When set to `11`, they disable the comparator function and put the ALERT/RDY pin into a high state.
 * When set to any other value, they controls the number of successive conversions exceeding the
 * upper or lower thresholds required before asserting the ALERT/RDY pin.
 * 00: Assert after one conversion
 * 01: Assert after two conversions
 * 10: Assert after four conversions
 * 11: Disable comparator(default)
 */
#define ADS1115_COMP_QUEUE_POS          (0U)
#define ADS1115_COMP_QUEUE_MASK         (0x03)

#define ADS1115_COMPARATOR_QUEUE1       (0x00 << ADS1115_COMP_QUEUE_POS)
#define ADS1115_COMPARATOR_QUEUE2       (0x01 << ADS1115_COMP_QUEUE_POS)
#define ADS1115_COMPARATOR_QUEUE4       (0x02 << ADS1115_COMP_QUEUE_POS)
#define ADS1115_COMPARATOR_QUEUE_OFF    (0x03 << ADS1115_COMP_QUEUE_POS)

/*******************************************************************************
* Private Types
*******************************************************************************/

/*******************************************************************************
* Private Variables
*******************************************************************************/

/*******************************************************************************
* Private Functions
*******************************************************************************/
/**
 * @brief  Checks if the pointer to the ADS1115 device structure is valid.
 * 
 * This private function checks if the pointer to the ADS1115 device structure (`dev`)
 * and its associated functions are not NULL.
 *
 * @param[in] dev Pointer to the ADS1115 device handle structure.
 * 
 * @return 0 if the pointer and associated functions are valid, -1 otherwise.
 */
static int null_handle_check(const ads1115_handle_t *dev)
{
    if ((dev == NULL) ||
        (dev->i2c_init == NULL)  || (dev->i2c_deinit == NULL) ||
        (dev->i2c_read == NULL)  || (dev->i2c_write == NULL)  ||
        (dev->delay_ms == NULL)  || (dev->debug_print == NULL)) {
        /* device structure pointer is not valid */
        return -1;
    }
    return 0;
}

/**
 * @brief Reads a 16-bit value from a specified device register.
 * 
 * This private function reads a 16-bit value from a specified register of the ADS1115 device.
 * 
 * @param[in] dev Pointer to the ADS1115 device handle.
 * @param[in] reg The register address to read from.
 * @param[out] data Pointer to where the read data will be stored.
 * 
 * @return 0 if the operation is successful, -1 on failure.
 */
static int i2c_read_chip_register(ads1115_handle_t *dev, uint8_t reg, uint16_t *data)
{
    /* Checks if the given device handle (`dev`) is valid */
    if(null_handle_check(dev) != 0)
    {
        return -1;
    }

    /* Reads 2-byte data from the specified register */
    uint8_t buf[2] = {0};
    if (dev->i2c_read(dev->i2c_addr, reg, (uint8_t *)buf, 2) != 0)
    {
        dev->debug_print("[%s] i2c_read_chip_register(): failed to read register 0x%02X\n", CHIP_NAME, reg);
        return -1;
    }

    /* Reorders the bytes and stores the 16-bit data in the data pointer */
    *data = ((uint16_t)buf[0] << 8) | (uint16_t)buf[1];

    return 0;
}

/**
 * @brief Writes a 16-bit value to a specified device register.
 *
 * This private function writes a 16-bit value to a specified register of the ADS1115 device
 *
 * @param[in] dev Pointer to the ADS1115 device handle.
 * @param[in] reg The register address to write to.
 * @param[in] data The 16-bit data to be written.
 *
 * @return 0 if the operation is successful, -1 on failure.
 */
static int i2c_write_chip_register(ads1115_handle_t *dev, uint8_t reg, uint16_t data)
{
    /* Checks if the given device handle (`dev`) is valid */
    if(null_handle_check(dev) != 0)
    {
        return -1;
    }

    /* Prepares a 2-byte buffer `buf` to hold the data to be written */
    uint8_t buf[2] = {0};
    buf[0] = (uint8_t)((data & 0xFF00) >> 8);
    buf[1] = (uint8_t)(data & 0x00FF);

    /* Writes the data to the specified register */
    if (dev->i2c_write(dev->i2c_addr, reg, (uint8_t *)buf, 2) != 0)
    {
        dev->debug_print("[%s] i2c_write_chip_register(): failed to write register 0x%02X\n", CHIP_NAME, reg);
        return -1;
    }

    return 0;
}

/*******************************************************************************
* API Implementation
*******************************************************************************/
int ads1115_init(ads1115_handle_t *dev)
{
    /* Checks if the given device handle (`dev`) is valid */
    if(null_handle_check(dev) != 0)
    {
        return ADS1115_FAIL;
    }

    /* Initializes I2C communication */
    if(dev->i2c_init() != 0)
    {
        dev->debug_print("[%s] ads1115_init(): i2c init failed\n", CHIP_NAME);
        return ADS1115_FAIL;
    }

    /* Performs a device reset, setting all device configurations to default values */
    uint16_t config_default =
            DiffIn0In1          |   /* AINp = AIN0 and AINn = AIN1 */
            Gain2p048           |   /* 2.048V PGA gain */
            SingleShotMode      |   /* Power-down single-shot mode */
            DataRate128SPS      |   /* 128 samples per second */
            CompareHysteresis   |   /* Threshold compare interrupt mode */
            CompareActiveLow    |   /* Active low - The comparator output is active low */
            CompareNoLatching   |   /* Non-latching comparator */
            CompareDisabled;        /* Comparator disabled */

    if(i2c_write_chip_register(dev, ADS1115_REG_CONFIG, config_default) != 0)
    {
        dev->debug_print("[%s] ads1115_init(): device reset failed\n", CHIP_NAME);
        return ADS1115_FAIL;
    }

    /* Checks whether device configuration was successful */
    uint16_t config_check;
    int res = i2c_read_chip_register(dev, ADS1115_REG_CONFIG, &config_check);
    
    if((res != 0) || (config_check != config_default))
    {
        dev->debug_print("[%s] ads1115_init(): device initialization failed\n", CHIP_NAME);
    }
    
    return ADS1115_SUCCESS;
}

int ads1115_deinit(ads1115_handle_t *dev)
{    
    /* Checks if the given device handle (`dev`) is valid */
    if(null_handle_check(dev) != 0)
    {
        return ADS1115_FAIL;
    }

    /* Ensures continuous reading stops */
    uint16_t config;
    config &= ~(ADS1115_MODE_MASK << ADS1115_MODE_POS);
    config |= (SingleShotMode & ADS1115_MODE_MASK) << ADS1115_MODE_POS;

    if(i2c_write_chip_register(dev, ADS1115_REG_CONFIG, config) != 0)
    {
        dev->debug_print("[%s] ads1115_deinit(): failure to stop the continuous reading process", CHIP_NAME);
        return ADS1115_FAIL;  
    }

    /* De-initializes I2C communication */
    if(dev->i2c_init() != 0)
    {
        dev->debug_print("[%s] ads1115_deinit(): i2c deinit failed\n", CHIP_NAME);
        return ADS1115_FAIL;
    }

    return ADS1115_SUCCESS;
}

int ads1115_set_mux(ads1115_handle_t *dev, ads1115_mux_t mux)
{
    /* Checks if the given device handle (`dev`) is valid */
    if(null_handle_check(dev) != 0)
    {
        return ADS1115_FAIL;
    }

    /* Reads current data from the config register  */
    uint16_t config;
    if(i2c_read_chip_register(dev, ADS1115_REG_CONFIG, &config) != 0)
    {
        dev->debug_print("[%s] ads1115_set_mux(): read config failed\n", CHIP_NAME);
        return ADS1115_FAIL;
    }
    
    static const uint16_t mux_values[] = {
        DiffIn0In1,
        DiffIn0In3,
        DiffIn1In3,
        DiffIn2In3,
        SingleIn0,
        SingleIn1,
        SingleIn2,
        SingleIn3,
    };

    config &= ~(ADS1115_MUX_MASK << ADS1115_MUX_POS);
    config |= mux_values[mux & ADS1115_MUX_MASK] << ADS1115_MUX_POS;

    /* Updates the config register with the new multiplex value */
    if(i2c_write_chip_register(dev, ADS1115_REG_CONFIG, config) != 0)
    {
        dev->debug_print("[%s] ads1115_set_mux(): write config failed\n", CHIP_NAME);
        return ADS1115_FAIL;        
    }

    return ADS1115_SUCCESS;
}

int ads1115_get_mux(ads1115_handle_t *dev, ads1115_mux_t *mux)
{
    /* Check argument validity and whether the provided handle (dev) is valid */
    if((null_handle_check(dev) != 0) || (mux == NULL))
    {
        return ADS1115_FAIL;
    }

    /* Reads current data from the config register */
    uint16_t config;
    if(i2c_read_chip_register(dev, ADS1115_REG_CONFIG, &config) != 0)
    {
        dev->debug_print("[%s] ads1115_get_mux(): read config failed.\n", CHIP_NAME);
        return ADS1115_FAIL;
    }

    /* Gets the current input multiplex configuration */
    *mux = (ads1115_mux_t)((config >> ADS1115_MUX_POS) & ADS1115_MUX_MASK);

    return ADS1115_SUCCESS;
}

int ads1115_info(ads1115_info_t *info)
{
    /* Checks if the provided handle (info) is valid */
    if (info == NULL)
    {
        return ADS1115_FAIL; 
    }

    memset(info, 0, sizeof(ads1115_info_t));            /* Initialize ads1115 info structure */
    strncpy(info->chip_name, CHIP_NAME, 32);            /* Copy chip name */
    strncpy(info->manufacturer, MANUFACTURER_NAME, sizeof(info->manufacturer));  /* Copy manufacturer name */
    strncpy(info->interface_type, INTERFACE_TYPE, sizeof(info->interface_type)); /* Copy interface type */
    info->voltage_min_v = SUPPLY_VOLTAGE_MIN;           /* Set minimal supply voltage */
    info->voltage_max_v = SUPPLY_VOLTAGE_MAX;           /* Set maximum supply voltage */
    info->current_max_ma = CURRENT_MAX;                 /* Set maximum current */
    info->temperature_max = TEMPERATURE_MAX;            /* Set minimal temperature */
    info->temperature_min = TEMPERATURE_MIN;            /* Set maximum temperature */

    return ADS1115_SUCCESS;
}
