/*******************************************************************************
 * @file    ads1115.h
 * @brief   Defines functions for ADS1115 module (analog-to-digital converter).
 * @author  Eng. Fabiano Bozzi D'Acunti
 * @li      fabiano.dcti@gmail.com
 * @version 0.1.0
 * @warning These features are under development and may not provide accurate results.
 ******************************************************************************/

#ifndef ADS1115_H
#define ADS1115_H

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdio.h>
#include <stdint.h>
#include <string.h>

/*******************************************************************************
 * Public Macros
 ******************************************************************************/
/**
 *  @brief Device errors
 */
#define ADS1115_SUCCESS      (0)   /**< Successful operation */
#define ADS1115_FAIL        (-1)   /**< Failed operation */

/**
 * @brief     Clears the device handle structure.
 * @param[in] HANDLE Points to ads1115 handle structure.
 * @param[in] STRUCTURE is ads1115_handle_t
 * @note      None
 */
#define ADS1115_CLEAR_HANDLE(HANDLE, STRUCTURE) memset(HANDLE, 0, sizeof(STRUCTURE))

/**
 * @brief     Assigns a user defined i2c bus initialization function to the specified handler.
 * @param[in] HANDLE Points to ads1115 handle structure.
 * @param[in] FUNCTION Points to `i2c_init` function address.
 * @note      None
 */
#define ADS1115_ASSIGN_I2C_INIT(HANDLE, FUNCTION) (HANDLE)->i2c_init = FUNCTION

/**
 * @brief     Assigns a user defined i2c bus de-initialization function to the specified handler.
 * @param[in] HANDLE Points to ads1115 handle structure.
 * @param[in] FUNCTION Points to `i2c_deinit` function address.
 * @note      None
 */
#define ADS1115_ASSIGN_I2C_DEINIT(HANDLE, FUNCTION) (HANDLE)->i2c_deinit = FUNCTION

/**
 * @brief     Assigns a user defined i2c read function to the specified handler.
 * @param[in] HANDLE Points to ads1115 handle structure.
 * @param[in] FUNCTION Points to `i2c_read` function address.
 * @note      None
 */
#define ADS1115_ASSIGN_I2C_READ(HANDLE, FUNCTION) (HANDLE)->i2c_read = FUNCTION

/**
 * @brief     Assigns a user defined i2c write function to the specified handler.
 * @param[in] HANDLE Points to ads1115 handle structure.
 * @param[in] FUNCTION Points to `i2c_write` function address.
 * @note      None
 */
#define ADS1115_ASSIGN_I2C_WRITE(HANDLE, FUNCTION) (HANDLE)->i2c_write = FUNCTION

/**
 * @brief     Assigns a user defined delay function to the specified handler.
 * @param[in] HANDLE Points to ads1115 handle structure.
 * @param[in] FUNCTION Points to a `delay_ms` function address.
 * @note      None
 */
#define ADS1115_ASSIGN_DELAY_MS(HANDLE, FUNCTION) (HANDLE)->delay_ms = FUNCTION

/**
 * @brief     Assigns a user defined debug print function to the specified handler.
 * @param[in] HANDLE Points to ads1115 handle structure.
 * @param[in] FUNCTION Points to a `debug_print` function address.
 * @note      None
 */
#define ADS1115_ASSIGN_DEBUG_PRINT(HANDLE, FUNCTION) (HANDLE)->debug_print = FUNCTION

/*******************************************************************************
 * Public Types
 ******************************************************************************/
 /**
 * @enum ads1115_address
 * @brief ADS1115 I2C slave address.
 */
typedef enum ads1115_address {
    SlaveAddressGnd = 0U,   /**< ADDR pin connected to GND - Slave Address = 0b1001000 (0x48) */
    SlaveAddressVcc,        /**< ADDR pin connected to VCC - Slave Address = 0b1001001 (0x49) */
    SlaveAddressSda,        /**< ADDR pin connected to SDA - Slave Address = 0b1001010 (0x4A) */
    SlaveAddressScl,        /**< ADDR pin connected to SCL - Slave Address = 0b1001011 (0x4B) */
} ads1115_address_t;

/**
 * @enum ads1115_mux
 * @brief ADS1115 input multiplexer.
 */
typedef enum ads1115_mux {
    DiffIn0In1 = 0U,/**< AINp = AIN0 and AINn = AIN1 (default) */
    DiffIn0In3,     /**< AINp = AIN0 and AINn = AIN3 */
    DiffIn1In3,     /**< AINp = AIN1 and AINn = AIN3 */
    DiffIn2In3,     /**< AINp = AIN2 and AINn = AIN3 */
    SingleIn0,      /**< AINp = AIN0 and AINn = GND  */
    SingleIn1,      /**< AINp = AIN1 and AINn = GND  */
    SingleIn2,      /**< AINp = AIN2 and AINn = GND  */
    SingleIn3,      /**< AINp = AIN3 and AINn = GND  */
} ads1115_mux_t;

/**
 * @enum ads1115_pga
 * @brief ADS1115 programmable gain amplifier range.
 */
typedef enum ads1115_pga {
    Gain6p144 = 0U, /**< 6.144V PGA gain - Full Scale = ±6.144V */
    Gain4p096,      /**< 4.096V PGA gain - Full Scale = ±4.096V */
    Gain2p048,      /**< 2.048V PGA gain - Full Scale = ±2.048V */
    Gain1p024,      /**< 1.024V PGA gain - Full Scale = ±1.024V */
    Gain0p541,      /**< 0.512V PGA gain - Full Scale = ±0.512V */
    Gain0p256,      /**< 0.256V PGA gain - Full Scale = ±0.256V */
} ads1115_pga_t;

/**
 * @enum ads111x_mode
 * @brief ADS1115 operating mode.
 */
typedef enum ads111x_mode {
	ContinuousMode = 0U, /**< Continuous conversion mode */
	SingleShotMode,      /**< Power-down single-shot mode (default) */
} ads111x_mode_t;

/**
 * @enum ads1115_rate
 * @brief ADS1115 data rate.
 */
typedef enum ads1115_rate {
    DataRate8SPS = 0U,  /**< 8 samples per second  */
    DataRate16SPS,      /**< 16 samples per second */
    DataRate32SPS,      /**< 32 samples per second */
    DataRate64SPS,      /**< 64 samples per second */
    DataRate128SPS,     /**< 128 samples per second (default) */
    DataRate250SPS,     /**< 250 samples per second */
    DataRate475SPS,     /**< 475 samples per second */
    DataRate860SPS,     /**< 860 samples per second */
} ads1115_rate_t;

/**
 * @enum ads1115_compare
 * @brief ADS1115 compare mode.
 */
typedef enum ads1115_compare {
    CompareHysteresis = 0U,	/**< Traditional comparator with hysteresis (default) */
	CompareWindow		    /**< Window comparator */
} ads1115_compare_t;

/**
 * @enum ads111x_polarity
 * @brief ADS1115 comparator polarity of the ALERT/RDY pin.
 */
typedef enum ads111x_polarity {
	CompareActiveLow = 0U,  /**< Active low - The comparator output is active low (default) */
	CompareActiveHigh,	    /**< Active high - The ALERT/RDY pin is active high */
} ads111x_polarity_t;

/**
 * @enum ads111x_latch
 * @brief ADS1115 ALERT/RDY pin latching comparator.
 */
typedef enum ads111x_latch {
	CompareNoLatching = 0U, /**< Non-latching comparator (default) */
	CompareLatching,        /**< The asserted ALERT/RDY pin remains latched */
} ads111x_latch_t;

/**
 * @enum ads1115_comparator_queue
 * @brief ADS1115 comparator queue.
 */
typedef enum ads1115_comparator_queue {
    CompareQueue1 = 0U, /**< Comparator queue has 1 conversion  */
    CompareQueue2,      /**< Comparator queue has 2 conversions */
    CompareQueue4,      /**< Comparator queue has 4 conversions */
    CompareDisabled,    /**< Comparator disabled (default) */
} ads1115_comparator_queue_t;

/**
 * @enum ads1115_pin
 * @brief ADS1115 ALERT/RDY pin state.
 */
typedef enum ads1115_pin {
    AlertRdyPinLow  = 0U,   /**< ALERT/RDY pin low  */
    AlertRdyPinHigh,        /**< ALERT/RDY pin high */
} ads1115_pin_t;

/**
 * @struct ads1115_handle
 * @brief ADS1115 handle structure definition.
 */
typedef struct ads1115_handle {
    uint8_t i2c_addr;                                                           /**< I2C device address */
    int (*i2c_init)(void);                                                      /**< Pointer to an i2c_init function. */
    int (*i2c_deinit)(void);                                                    /**< Pointer to an i2c_deinit function */
    int (*i2c_read)(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len);     /**< Pointer to an i2c_read function */
    int (*i2c_write)(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len);    /**< Pointer to an i2c_write function */
    void (*delay_ms)(uint32_t ms);                                              /**< Pointer to a delay_ms function */
    void (*debug_print)(const char *const fmt, ...);                            /**< Pointer to a debug_print function */
} ads1115_handle_t;

/**
 * @struct ads1115_info
 * @brief ADS1115 information structure definition
 */
typedef struct ads1115_info {
    char chip_name[32];     /**< Chip name */
    char manufacturer[32];  /**< Manufacturer name */
    char interface_type[8]; /**< Chip interface type */
    float voltage_min_v;    /**< Chip min supply voltage */
    float voltage_max_v;	/**< Chip max supply voltage */
    float current_max_ma;   /**< Chip max current */
    float temperature_min;	/**< Chip min operating temperature */
    float temperature_max;	/**< Chip max operating temperature */
} ads1115_info_t;


/*******************************************************************************
 * API Specification
 ******************************************************************************/
/**
 * @brief ADS1115 device initialization.
 * This function initializes the ADS1115 analog-to-digital converter.
 * 
 * @param[in] dev Pointer to the ADS1115 device handle structure.
 *
 * @return
 * - ::ADS1115_SUCCESS if the initialization is successful.
 * - ::ADS1115_FAIL if any of the initialization steps fail.
 *
 * @note
 * - The default configuration includes settings such as single-shot mode, 2.048V PGA gain, 128 SPS,
 *   low active comparator output, non-latching comparator, and comparator queue disabled.
 * - The function may print debug messages using the `debug_print` function for errors reporting.
 */
int ads1115_init(ads1115_handle_t *dev);

/**
 * @brief ADS1115 device de-initialization.
 * This function de-initializes the ADS1115 device.
 *
 * @param[in] dev Pointer to the ADS1115 device handle structure.
 *
 * @return
 * - ::ADS1115_SUCCESS if the de-initialization is successful.
 * - ::ADS1115_FAIL if any of the de-initialization steps fail.
 */
int ads1115_deinit(ads1115_handle_t *dev);

/**
 * @brief Sets the ADS1115 input multiplexer configuration.
 * This function sets the input multiplexer configuration (`mux`) of the ADS1115 device.
 * 
 * @param[in] dev Pointer to the ADS1115 device handle structure.
 *
 * @return
 * - ::ADS1115_SUCCESS if the operation is successful.
 * - ::ADS1115_FAIL if the operation fails.
 */
int ads1115_set_mux(ads1115_handle_t *dev, ads1115_mux_t channel);

/**
 * @brief Gets the ADS1115 input multiplexer configuration.
 * This function gets the input multiplexer configuration (`mux`) of the ADS1115 device.
 * 
 * @param[in] dev Pointer to the ADS1115 device handle structure.
 *
 * @return
 * - ::ADS1115_SUCCESS if the operation is successful.
 * - ::ADS1115_FAIL if the operation fails.
 */
int ads1115_get_mux(ads1115_handle_t *dev, ads1115_mux_t *mux);

/**
 * @brief Retrieves information about the ADS1115 device.
 *
 * This function populates the provided `info` structure with information about the ADS1115 device,
 * including its name, manufacturer, interface type, supply voltage range, maximum current, and temperature range.
 *
 * @param[out] info Pointer to the structure where the ADS1115 information will be stored.
 *
 * @return
 * - ::ADS1115_SUCCESS if the operation is successful.
 * - ::ADS1115_FAIL if the operation fails.
 */
int ads1115_info(ads1115_info_t *info);

#ifdef __cplusplus
}
#endif

#endif /* ADS1115_H */
