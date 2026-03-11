/**
 *
 * @defgroup nrfx_wdt_config WDT peripheral driver configuration
 * @{
 * @ingroup nrfx_wdt
 */

/** @brief Enable WDT driver
 *
 *  Set to 1 to activate.
 *
 * @note This is an NRF_CONFIG macro.
 */
#define NRFX_WDT_ENABLED

/** @brief Remove WDT IRQ handling from WDT driver
 *
 *  Following options are available:
 * - 0 - Include WDT IRQ handling
 * - 1 - Remove WDT IRQ handling
 *
 * @note This is an NRF_CONFIG macro.
 */
#define NRFX_WDT_CONFIG_NO_IRQ

/** @brief Interrupt priority
 *
 *  Following options are available:
 * - 0 - 0 (highest)
 * - 1 - 1
 * - 2 - 2
 * - 3 - 3
 * - 4 - 4
 * - 5 - 5
 * - 6 - 6
 * - 7 - 7
 *
 * @note This is an NRF_CONFIG macro.
 */
#define NRFX_WDT_CONFIG_IRQ_PRIORITY

/** @brief Enables logging in the module.
 *
 *  Set to 1 to activate.
 *
 * @note This is an NRF_CONFIG macro.
 */
#define NRFX_WDT_CONFIG_LOG_ENABLED

/** @brief Default Severity level
 *
 *  Following options are available:
 * - 0 - Off
 * - 1 - Error
 * - 2 - Warning
 * - 3 - Info
 * - 4 - Debug
 *
 * @note This is an NRF_CONFIG macro.
 */
#define NRFX_WDT_CONFIG_LOG_LEVEL

/** @} */
