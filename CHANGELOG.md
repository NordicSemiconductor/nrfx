# Changelog
All notable changes to this project are documented in this file.

## [3.14.0] - 2025-08-22
### Added
- Added support for nRF54LS05B Eng A.
- Added support for using the CRACEN driver without CRYPTOMASTER module.

### Changed
- Updated MDK to version 8.72.2.

### Fixed
- Fixed missing event clearing when setting limits in the SAADC driver.
- Fixed missing disabling of the internal timer in case of switching modes in the SAADC driver.
- Fixed missing brackets in NRFX_BITMASK_TO_BITPOS() macro.
- Fixed incorrect offset in the PUBLISH and SUBSCRIBE register getters in the RADIO HAL.

## [3.13.0] - 2025-07-18
### Added
- Added support for calibrating the LFRC peripheral on the nRF7120 Eng A SoC in the CLOCK driver.
- Added support for the GPIOTE0 instance on the nRF54H20 SoC radio core in the GPIOTE driver.
- Added function for checking whether the XO is tuned in the CLOCK driver.
- Added support for trimming in the SAADC HAL, HALY, and the driver for supported devices.
- Added functions for committing the write buffer, checking whether the write buffer is empty, and waking up the controller in the RRAMC driver.
- Added functions for validating address range correctness in the MRAMC driver.
- Added functions for accessing the SYSCOUNTER array by index in the GRTC HAL and HALY.
- Added individual functions for setting buffer length and buffer pointer in the UARTE HAL.
- Added missing sampling period values in the QDEC HAL.
- Added functions for getting SAADC trim values in the FICR HAL.

### Changed
- Updated MDK to version 8.72.1.
- Changed the function for reading MRAM to use bytes instead of 32-bit words as length in the MRAMC driver.

### Fixed
- Fixed incorrect interrupt initialization in the RRAMC driver.
- Fixed waiting for STOPPED event in case of stopping the peripheral without ongoing transfer in the SPIM driver.
- Fixed incorrect size assertions in the MRAMC HAL.
- Fixed the nRF54LM20A memory map description in the nrfx_ram_ctrl helper. Now, it corresponds to the actual memory sections layout.

## [3.12.1] - 2025-06-09
### Fixed
- Fixed missing event clearing in case of NULL user handler in the GRTC driver.
- Fixed the workaround condition for nRF54L Series anomaly 55 in the SPIM driver.

## [3.12.0] - 2025-06-02
### Added
- Added a driver, HALY and HAL for MRAMC.
- Added support for 24 MHz clock on the nRF54LM20A Eng A SoC in the CLOCK driver.
- Added the workaround for nRF54L Series anomaly 55 in the SPIM driver.
- Added functions for setting compare value in an optimized way in the GRTC driver.
- Added a function for retrieving compare value in the GRTC driver.
- Added a function for setting and getting an event handler in the TWIM driver.
- Added support for chopping and high-speed analog pin configuration on the affected SoCs in the SAADC driver.
- Added a function for getting a driver instance associated with a DPPIC instance located on the same APB as the specified peripheral in the DPPI driver.
- Added functions for managing all RAM blocks at once in the ``nrfx_ram_ctrl`` helper.
- Added a function for getting a pin drive setting in the GPIO HAL.
- Added a function for checking whether the XIP is active in the QSPI HAL.
- Added an enumerator for frequency divider ratio in the AUXPLL HAL.
- Added new functions for enabling, disabling and checking status of voltage regulators in POWER HAL. Previous functions are deprecated.
- Added functions for getting the SUBSCRIBE and PUBLISH registers in all HALs.

### Changed
- Updated MDK to version 8.71.1.
- Optimized counter setting and reading procedures to make them more performant in the GRTC driver.
- Restored USBPWRRDY event clearing before initialization in the USBREG driver.

### Fixed
- Fixed 8-bit samples handling for the nRF54 Series in the SAADC driver.
- Fixed missing DONE event clearing after a calibration in the SAADC driver.
- Fixed the processing order when events arrive simultaneously in the SPIS driver.
- Fixed pin idle state handling for the nRF54 Series in the PWM driver.
- Fixed the counter reading procedure for VPR cores in the GRTC HALY.
- Fixed the custom ratio setting procedure in the PDM HALY.
- Fixed missing array list functionality for the nRF54 Series in the SPIM HAL.
- Fixed a workaround for the anomaly 161 on the nRF5340 SoC in non-secure variant in the RESET HAL.

## [3.11.0] - 2025-02-24
### Added
- Added the workaround for nRF54L Series anomaly 39 in the CLOCK driver.
- Added the optional workarounds for nRF52 Series anomaly 219 and nRF53 Series anomaly 47 in the TWIM driver.
- Added support for nRF54LM20A Eng A in the PDM driver.
- Added support for trimming in the COMP HAL and driver for supported devices.
- Added support for frame size and address in the UARTE HAL.
- Added support for nRF54L Series in the CTRLAP HAL.
- Added configuration option for skipping clock selection in the GRTC HALY.
- Added auxiliary macro for generating 32-bit counter value in the VPR CSR VTIM HAL.
- Added functions for getting task endpoint associated with given channel or fork in the PPI HAL.
- Expanded functions for setting RAM power and retention configuration in the MEMCONF HAL.

### Changed
- Updated MDK to version 8.70.0.
- Renamed nRF54L20 Eng A to nRF54LM20A Eng A.
- Renamed nRF54L09 Eng A to nRF54LV10A Eng A.
- Removed Sphinx-based documentation. Use existing Doxygen documentation instead.

### Fixed
- Fixed incorrect value of power-on-reset reset reason bitmask in the RESETINFO HAL.

## [3.10.0] - 2025-01-21
### Added
- Added support for nRF7120 Eng A.
- Added support for XO tuning in the CLOCK driver.
- Added HALs and driver for the CryptoMaster and RNG components of the CRACEN peripheral.
- Added the workaround for nRF54L anomaly 8 and nRF54H20 anomaly 212 in the SPIM driver.
- Added functions for buffered input and output shift control in the VPR CSR VIO HAL.
- Added functions for setting and clearing bits in the VPR CSR VIO HAL.
- Added functions for setting and getting individual peripheral pins in the following HALs: I2S, PDM, PWM, QDEC, QSPI, SPI, SPIM, SPIS, TWI, TWIM, TWIS, UART, UARTE.
- Added functions for accessing memory contents in the RRAMC HAL.
- Added function for handling data whitening polynomial configuration in the RADIO HAL.

### Changed
- Updated MDK to version 8.69.1.
- Changed the system counter reading procedure for VPR cores in the GRTC driver. Now, individual high and low 32-bit parts are read separately to preserve the order of accessing the registers. ARM cores retained existing procedure based on double-word read instruction.
- Removed remaining references to obsolete nRF54H20 Eng A, nRF54H20 Eng B, and nRF54L15 Eng A.

### Fixed
- Fixed the local reset reason handling for nRF54H20 in the nrfx_reset_reason helper. Now, the local reset reasons can be cleared.
- Fixed the missing bitshift of retention mask for VMC peripheral in the nrfx_ram_ctrl helper.
- Fixed the configuration of data whitening initialization vector in the RADIO HAL.

## [3.9.0] - 2024-11-19
### Added
- Added support for nRF54L05, nRF54L09 Eng A, and nRF54L10.
- Added NRFX_RELEASE_VER_AT_LEAST() macro for conditional statements based on current nrfx release version.
- Added functions for handling pin retention in the GPIO HAL.
- Added HAL for the BICR peripheral.
- Added support for nRF54H20 in the nrfx_reset_reason helper.

### Changed
- Updated MDK to version 8.68.1.
- Removed support for nRF54H20 Eng A, nRF54H20 Eng B, and nRF54L15 Eng A.
- Removed event clearing in the initialization of the USBREG driver. Now, the USB stack built upon the driver can react to the cable being attached when the device is powered from the USB bus.
- Changed the default driver configuration for the COMP driver. Now, the description and code matches.
- Improved RX FIFO flush handling in the UARTE driver.

### Fixed
- Fixed the nRF54L15 memory map description in the nrfx_ram_ctrl helper. Now, it corresponds to the actual memory sections layout.
- Fixed the LIMIT events interrupt bitmask definition in the SAADC HAL. Now, the LIMIT events 4 to 7 are working properly.

## [3.8.0] - 2024-10-17
### Added
- Added multi-instance support in the DPPI driver.
- Added multi-instance driver for the PPIB.
- Added a function for interconnecting two DPPI domains in the GPPI helper.

## [3.7.0] - 2024-09-27
### Added
- Added support for nRF54H20 Eng B.
- Added multi-instance support in the PDM driver.
- Added option to keep SYSCOUNTER active whenever any of SoC CPUs is not sleeping in the GRTC driver.
- Added new functions in the LPCOMP driver to better match COMP driver. Previous functions are deprecated.
- Added support for the IDLEOUT mechanism in the PWM driver.
- Added support for configuring clock using prescaler in the PDM driver.
- Added missing decimation ratios in the PDM HAL.
- Added missing TX power settings in the RADIO HAL.
- Added temperature coefficient trim setting in the HSFLL HAL.
- Added HAL for the TDDCONF peripheral.

### Changed
- Updated MDK to version 8.67.0.

### Fixed
- Fixed capacitance calculation macros in the OSCILLATORS HAL. Now they are aligned with the Product Specification.

## [3.6.0] - 2024-07-26
### Added
- Added support for nRF54L20 Eng A and nRF9230 Eng B.
- Added support for enabling and disabling the SPIM peripheral instance for nRF54H20 outside of driver context in the SPIM driver.
- Added support for FRAMETIMEOUT event in the UARTE HAL.
- Added support for SOFTRESET task in the RADIO HAL.
- Added optimized functions for reading inputs, writing outputs and configuring direction of GPIO ports in the GPIO HAL.
- Added getters for BLE address, BLE IR, and BLE ER in the FICR HAL.
- Implemented a workaround for the anomaly 89 on the nRF52832 SoC in the TWIM and SPIM HALs.

### Changed
- Updated MDK to version 8.66.0.
- Removed unsupported GPIO pin drive settings from the GPIO HAL.

### Fixed
- Fixed spurious assertion in the TIMER driver. Now, the nrfx_timer_extended_compare() function can be called after the TIMER has started.
- Fixed potential data corruption when cache bounce buffer and RX FIFO buffer were shared in the UARTE driver.
- Fixed handling of events having respective interrupts disabled in the UARTE driver.
- Fixed dropping of data in case of RX abort in the UARTE driver.
- Fixed frequency symbol setting in case of non-secure build on the nRF54L15 SoC.

## [3.5.0] - 2024-05-10
### Added
- Added function for freeing all allocated channels in the WDT driver.
- Added HAL for AUXPLL peripheral.
- Added support for nRF54H20 and nRF54L15 in the AAR HAL.

### Changed
- Updated MDK to version 8.64.0.
- Changed default implementation of the core-dependent delay loop for the VPR cores. Now, the VTIM peripheral is utilized instead of CPU spinning.

### Fixed
- Fixed 1 MHz clock frequency configuration setting in the TWIM driver. Now, the supported pins are checked only if GPIO or PSEL configuration is specified.
- Fixed overridden write mode in the RRAMC HALY. Now, the configured write mode is preserved before the write operation and restored afterwards.

## [3.4.0] - 2024-03-08
### Added
- Added support for nRF54H20 and nRF54L15.
- Added support for changing the sub-power mode in the POWER driver.
- Added support for sleep configuration in the GRTC driver.
- Added HAL for TDM peripheral.
- Implemented a workaround for the anomaly 159 on the nRF5340 SoC in the QSPI driver.

### Changed
- Updated MDK to version 8.63.2.

### Fixed
- Fixed extended configuration setting in the SPIM driver. Now extended configuration is applied correctly when only a subset of extended features is available.

## [3.3.0] - 2023-12-21
### Added
- Added support for nRF54H20 Eng A and nRF54L15 Eng A.
- Added HALs for BELLBOARD, CRACEN, CTRLAP, EXMIF, GLITCHDET, GRTC, HSFLL, IPCT, LRCCONF, MEMCONF, MPC, MVDMA, PPIB, RAMC, RESETINFO, RRAMC, STM, TAMPC, TBM, UICR and, VPR.
- Added HALYs for BELLBOARD, GRTC, MVDMA, RRAMC, TBM, and VPR.
- Added drivers for BELLBOARD, GRTC, RRAMC, TBM, and VEVIF.
- Added the nrfx_ids helper layer to facilitate developing generic code that performs inter-domain signalling.
- Added possibility to use TX link feature without (D)PPI connection in the UARTE driver.
- Added possibility to get callback called on each byte received in the UARTE driver.

### Changed
- Updated MDK to version 8.60.3.
- Changed the default nrfx API version to 3.1.0.
- Changed how the buffers are provided to the I2S driver. Now, the buffer size must be passed alongside pointers to the next buffers.
- Improved peripheral power management in the PWM driver. Now, the peripheral is enabled only during playback.

### Fixed
- Fixed the workaround for nRF52840 anomaly 215 and nRF5340 anomaly 215 in the QSPI driver. Now, the custom instruction functions do not risk triggering anomalies.
- Fixed a missing CTRLAP reset reason for nRF91 in the POWER HAL.
- Fixed incorrect assertion in the TWIS driver. Now, the driver status can be checked when the driver is idle.
- Restored workaround for nRF52 anomaly 55 in the I2S driver.

## [3.2.0] - 2023-10-02
### Added
- Added multi-instance support for the GPIOTE driver.
- Added the nrfx_ram_ctrl helper layer to facilitate developing generic code that controls the power and retention settings of RAM blocks.
- Introduced the NRFX_TIMER_BASE_FREQUENCY_GET() and NRFX_SPIM_BASE_FREQUENCY_GET() macros for getting base frequencies in Hz for the specified TIMER and SPIM instances.
- Added a function for checking whether the specified voltage regulator is enabled in the REGULATORS HAL.
- Added a function for checking whether an inductor is connected to the DCC pin in the REGULATORS HAL.
- Added a function for checking if the detected voltage is below or above the threshold of VPOF (POF Comparator's threshold voltage) in the REGULATORS HAL.
- Added a function for powering on all RAM blocks in the VMC HAL.
- Added missing entries in enumerators for RAM sections in the VMC HAL.
- Added the new nrfx_wdt_stop() function to the WDT driver that stops the watchdog.
- Added the new nrfx_wdt_uninit() function to the WDT driver that uninitializes the watchdog.
- Added the new nrfy_wdt_task_stop_enable_set() function to the WDT HALY that enables or disables stopping the watchdog.
- Added functions for retrieving pointers to the receive and transmit buffers in the TWIS HAL.
- Added functions for checking whether given instance is initialized to all of the drivers.
- Added individual functions for setting reference selection, external reference, detection type and enabling hysteresis in the LPCOMP HAL. Now they are supposed to be used instead of nrf_lpcomp_configure() function.
- Added a blocking mode to the CLOCK driver. The mode is initialized when a user handler is set to NULL.
- Added a new member to the configuration structure of the QSPI driver. The new configuration member determines time in milliseconds when a timeout of the QSPI operation occurs.
- Added the new nrfx_qspi_timeout_signal() function to the QSPI driver that signals premature QSPI operation timeout.

### Changed
- Updated MDK to version 8.58.0.
- Overhauled the UARTE driver. Extended the UARTE driver with various modes of operations including: mixing of blocking and non-blocking transfers, linking transfers, continuous reception, low power operation.
- Changed all functions' parameters in the GPIOTE driver. A pointer to nrfx_gpiote_t structure was added as the first parameter in all functions. Now functions without the pointer to nrfx_gpiote_t structure as the first argument are deprecated.
- Changed functions for setting and reading the GPREGRET registers in the POWER HAL. Now there are only two functions, nrf_power_gpregret_set() for setting and nrf_power_gpregret_get() for reading. A new `reg_num` argument was added to both of them.
- Replaced nrf_regulators_dcdcen_set() and nrf_regulators_dcdcen_vddh_set() functions with nrf_regulators_vreg_enable_set().
- Replaced nrf_regulators_pofcon_set() and nrf_regulators_pofcon_vddh_set() functions with nrf_regulators_config_set().
- Replaced nrf_regulators_pofcon_get() function with nrf_regulators_pof_config_get().
- Renamed nrf_regulators_mainregstatus_get() function to nrf_regulators_main_status_get().
- Renamed NRF_REGULATORS_POTHR_V{XY} macros to NRF_REGULATORS_POF_THR_{X}V{Y}.
- Renamed NRF_REGULATORS_PORTHRVDDH_V{XY} macros to NRF_REGULATORS_POF_THR_VDDH_{X}V{Y}.
- Renamed nrf_regulators_pof_thrvddh_t structure to nrf_regulators_pof_thr_vddh_t.
- Renamed macros indicating features presence in the REGULATORS HAL.
- Changed a default value of the source reference structure member from 1.8V to 1.2V in the COMP driver configuration structure.
- Changed a prototype of an event handler in the WDT driver. The new prototype accepts an event type, requests and a pointer to the context. A previous version is deprecated.
- Changed a prototype of the nrfx_wdt_init() function in the WDT driver. The new prototype accepts a pointer to the instance, a pointer to the configuration, an event handler and a pointer to the context. A previous version is deprecated.
- Replaced the nrf_wdt_task_stop_enable() function with nrf_wdt_task_stop_enable_set(). Now it takes a boolean parameter.
- Replaced the deprecated NRFX_ERROR_ALREADY_INITIALIZED error code with the new NRFX_ERROR_ALREADY one.
- Changed the organization of configuration structure members in the LPCOMP driver.
- Changed the definition of nrf_saadc_value_t type in the SAADC HAL. Now pointer to the buffer holding conversion results is a void* instead of int16_t*.
- Changed the return value in all drivers in case initialization functions have already been executed. Now the NRFX_ERROR_ALREADY error code will be returned instead of NRFX_ERROR_INVALID_STATE.
- Changed the way that the QSPI peripheral is activated. Now the driver activates the peripheral instance before the first transfer or when the nrfx_qspi_activate() function is called.

### Fixed
- Fixed the device failing to reconnect to the host after USB cable being reconnected in the USBD driver.
- Fixed configuration of the external reference in nrfx_comp_init() and nrfx_comp_reconfigure() functions. Now the differential mode can be enabled in the COMP driver.
- Fixed support for P0.18, P0.20, and P0.28-P0.30 pins in the GPIOTE driver for nRF52820 SoC.
- Fixed events being forwarded to the user callback despite having their interrupts disabled in the QDEC driver.

## [3.1.0] - 2023-06-28
### Added
- Added the HALY layer for the NFCT. HALY is an extension of the HAL layer that aggregates basic hardware use cases within single functions. Now it is used instead of HAL in the NFCT drivers.
- Added the NRFX_IN_RANGE() macro for checking if a given value is in a given range.
- Added functions for writing a word to the flash and reading a buffer, word, halfword, and byte in the NVMC HAL.

### Changed
- Updated MDK to version 8.55.0.
- Modified the power management in the SPIM driver. Now the nrfx_spim_abort() function must be called once all expected transactions using the NRFX_SPIM_FLAG_NO_XFER_EVT_HANDLER, NRFX_SPIM_FLAG_HOLD_XFER, or NRFX_SPIM_FLAG_REPEATED_XFER option flags are finalized.

### Fixed
- Fixed a workaround for the anomaly 109 on the nRF52 family in the SPIM and SPIS drivers.
- Fixed the NRFX_TIMER_FREQUENCY_STATIC_CHECK() and NRFX_SPIM_FREQUENCY_STATIC_CHECK() macros.
- Fixed the nrfx_wdt_reconfigure() function that was returning the NRFX_ERROR_INVALID_STATE error code when the driver instance has been initialized.
- Fixed the nrfx_pwm_stopped_check() function when used without user's handler function.

## [3.0.0] - 2023-04-25
### Added
- Added the HALY layer for the following peripherals: COMP, DPPI, GPIO, GPIOTE, I2S, LPCOMP, PDM, PWM, QDEC, RTC, SAADC, SPIM, TEMP, TIMER, TWIM, UARTE, WDT. HALY is an extension of the HAL layer that aggregates basic hardware use cases within single functions. Now it is used instead of HAL in the corresponding drivers.
- Added functions for reconfiguring a peripheral outside of the initialization process in the following drivers: COMP, PDM, PWM, QDEC, QSPI, SPI, SPIM, SPIS, TIMER, TWI, TWIM, TWIS, UART, UARTE, WDT.
- Added multi-instance support for the I2S and QDEC drivers. Now a pointer to the driver instance needs to be specified in all relevant functions.
- Added the reportper_inten member to the configuration structure for the QDEC driver. It allows to explicitly enable interrupts associated with the REPORTRDY peripheral event.
- Added the NRFX_ERROR_FORBIDDEN error being returned from the nrfx_spim_init() function when a user attempts to configure a hardware-controlled and software-controlled chip select simultaneously.
- Added the NRFX_TIMER_FREQUENCY_STATIC_CHECK() macro to statically check if the given frequency is achievable for the specified TIMER instance.
- Added the NRFX_TWIM_XFER_DESC() macro to support creation of any type of TWIM transaction for the TWIM driver.
- Added auxiliary macros NRFX_KHZ_TO_HZ() and NRFX_MHZ_TO_HZ() for converting specified frequency value to Hertz.
- Added auxiliary macros for sophisticated handling of the preprocessor symbols to `nrfx_utils.h`.
- Added functions for getting the number of available channels and groups for a given peripheral instance in the DPPIC HAL.
- Added functions for getting the SEQSTART tasks and SEQEND events associated with a specified sequence index in the PWM HAL.
- Added the nrf_rtc_capture_task_get() function for getting the CAPTURE task associated with the specified CC channel in the RTC HAL.
- Added functions for managing the non-maskable watchdog interrupts in the WDT HAL.
- Added functions for getting the address of peripheral tasks and events in the EGU HAL.
- Added a function for enabling the watchdog STOP task in the WDT HAL.
- Added a function for getting the bitmask of all watchdog requests' statuses in the WDT HAL.
- Added a function for clearing the event and task endpoints for a given channel in the GPPI helper.

### Changed
- Renamed a macro from NRFX_VOLTAGE_THRESHOLD_TO_INT() to NRFX_COMP_VOLTAGE_THRESHOLD_TO_INT() in the COMP HAL.
- Renamed functions for getting the task of event address in the following drivers: CLOCK, GPIOTE, PPI, SPIM, TWIM. Now they follow a common scheme.
- Renamed configuration structure members `hal` and `hal_config`, containing low-level peripheral settings, to `config` in the LPCOMP and UARTE drivers.
- Renamed a function from nrfx_pwm_is_stopped() to nrfx_pwm_stopped_check() in the PWM driver.
- Renamed a symbol from NRF_QDEC_LED_NOT_CONNECTED to NRF_QDEC_PIN_NOT_CONNECTED in the QDEC HAL. Now it is consistent with other, similar symbols.
- Changed the way the API version to be used is specified. Now the following dedicated symbols shall be defined to form the API version as a numerical value: NRFX_CONFIG_API_VER_MAJOR, NRFX_CONFIG_API_VER_MINOR, NRFX_CONFIG_API_VER_MICRO.
- Changed prototypes of several functions in the UARTE driver. Now all functions have a return value, and some of them accept additional parameters.
- Changed a type of configuration structure members that hold a pin number from `uint8_t` to `uint32_t` in the following drivers: I2S, PWM, SPIM.
- Changed the prototypes of the nrf_i2s_pins_set() and nrf_i2s_configure() functions in the I2S HAL. Now they accept a pointer to the configuration structure instead of several individual parameters.
- Changed a default value of the `gain` structure member from `1/6` to `1` in the SAADC driver configuration structure.
- Changed a type of the configuration structure member specifying the clock frequency from an enumeration to an unsigned integer in the TIMER driver. Now the timer clock frequency must be specified in Hertz. Use the NRFX_TIMER_FREQUENCY_STATIC_CHECK() macro to check if the requested frequency is achievable for a given TIMER instance.
- Changed a type of the configuration structure member specifying the clock frequency from an enumeration to an unsigned integer in the SPIM driver. Now the clock signal frequency must be specified in Hertz. Use the NRFX_SPIM_FREQUENCY_STATIC_CHECK() macro to check if the requested frequency is achievable for a given SPIM instance.
- Changed the prototype of the NRFX_TIMER_DEFAULT_CONFIG() macro in the TIMER driver. Now the desired timer clock frequency in Hertz needs to be specified as a parameter.
- Changed a prototype of the WDT driver callback. Now a bitmask specifying which reload request caused the timeout is provided.
- Changed the prototype of the initialization and callback functions of the QDEC driver. Now the user context can be provided and accessed during the interrupt handler execution.
- Removed the deprecated nrf_gpio_pin_mcu_select() function and the corresponding nrf_gpio_pin_mcusel_t enumerator. Use the nrf_gpio_pin_control_select() function and the nrf_gpio_pin_sel_t enumerator instead.
- Removed the deprecated and spurious NRF_QDEC_REPORTPER_DISABLED symbol from the QDEC HAL.
- Removed the deprecated nrf_qdec_pio_assign() function from the QDEC HAL. Use the nrf_qdec_pins_set() function instead.
- Removed the deprecated nrf_timer_frequency_set() and nrf_timer_frequency_get() functions in the TIMER HAL. Use nrf_timer_prescaler_set() and nrf_timer_prescaler_get() instead.
- Removed symbols that mark a pin as unused in the following drivers: I2S, PWM, SPIM, SPIS. Now the HAL variant should be used instead.
- Removed deprecated functions from the GPIOTE driver. Use new functions instead.
- Removed the deprecated callback prototype in the IPC driver. Use a new variant instead.
- Removed the following redundant functions from the PWM driver: nrfx_pwm_sequence_values_update(), nrfx_pwm_sequence_length_update(), nrfx_pwm_sequence_repeats_update(), nrfx_pwm_sequence_end_delay_update(). Use the nrfx_pwm_sequence_update() function instead.
- Moved static inline functions from the header file to the source file in the GPPI helper.
- Refactored the method of configuring pins that should use inverted polarity in the PWM driver. Now an array inside driver configuration structure needs to be used instead of masking the pin number value. As a consequence, the NRFX_PWM_PIN_INVERTED symbol was removed.
- Renamed configuration structure members used for setting peripheral pins in the following drivers: PDM, TWIM, TWIS, UARTE. Now they use a common scheme.
- Renamed the nrf_wdt_started() and nrf_wdt_request_status() functions in the WDT HAL. Now they are named as nrf_wdt_started_check() and nrf_wdt_request_status_check() respectively.

### Fixed
- Fixed a misleading description for the nrf_rtc_event_disable() function in the RTC HAL. It accepts a bitmask of events instead of a single event.

## [2.11.0] - 2023-04-07
### Added
- Added support for the nRF9161 and nRF9131 SiPs. Use `NRF9120_XXAA` as the compilation symbol.
- Implemented a workaround for the anomaly 161 on the nRF5340 SoC in the RESET HAL.

### Changed
- Updated MDK to version 8.53.1.

### Fixed
- Removed a spurious function call causing aborting of endpoints in the SUSPEND event for the USBD driver.

## [2.10.0] - 2022-10-25
### Added
- Added NRFX_CONFIG_API_VER_2_9 and NRFX_CONFIG_API_VER_2_10 symbols that guard API-breaking changes. Deprecated API is used by default.
- Added new signature for the event handler in the IPC driver that accepts event index instead of event bitmask. The previous signature is deprecated.
- Added nrfx_ipc_gpmem_get() function that is used to get data from the GPMEM register in IPC. It should be now used instead of nrfx_ipc_mem_get(), which is deprecated.
- Added samples for the following drivers: EGU, PWM, RNG, SAADC, SPIM, SPIS, TEMP, TIMER, TWIM, TWIS, UARTE. They are a part of `zephyrproject-rtos/hal_nordic/nrfx` repository.
- Introduced the NRFX_{PERIPH}_INST_HANDLER_GET() macro for getting interrupt handler associated with the specified driver instance.
- Introduced the NRFX_{PERIPH}_INST_GET() macro for getting pointer to the structure of the registers of the specified peripheral.
- Introduced the NRF_TIMER_PRESCALER_CALCULATE() macro for computing prescaler value for given TIMER base frequency and desired frequency.
- Introduced the NRF_TIMER_BASE_FREQUENCY_GET() macro for getting base frequency in Hz for the specified TIMER instance.
- Added missing NRFX_RESET_REASON_CTRLAP_MASK in nrfx_reset_reason_mask_t for nRF9160.
- Added missing NRFX_RESET_REASON_SREQ_MASK in nrfx_reset_reason_mask_t.

### Changed
- Updated MDK to version 8.51.0.
- Refactored the TIMER driver to allow user handler to be NULL.
- Removed magic numbers from nrf_twim_event_t type in TWIM HAL.

### Fixed
- Fixed the NRFX_ROUNDED_DIV() macro for negative numbers.
- Disabled array bounds warning for nrf_clock_is_running() function that was false positive in GCC 12 and above.

## [2.9.0] - 2022-07-19
### Added
- Added configuration parameter NRFX_NFCT_PARAM_ID_FDT_MIN that allows settting the value of the FRAMEDELAYMIN register.
- Added nrf_gpio_pin_sel_t enumerator. It should be now used instead of nrf_gpio_pin_mcusel_t, which is deprecated.
- Added nrf_gpio_pin_control_select() function. It should be now used instead of nrf_gpio_pin_mcu_select(), which is deprecated.
- Added NRF_TIMER_CC_COUNT_MAX symbol that specifies maximum number of available compare channels.
- Added NRF_TIMER_ALL_CHANNELS_INT_MASK symbol that is used for creating the interrupt bitmask for all compare channels.
- Added nrf_timer_prescaler_set() and nrf_timer_prescaler_get() functions that are used for setting and retrieving the prescaler factor. They should be now used instead of nrf_timer_frequency_set() and nrf_timer_frequency_get(), which are deprecated.
- Added NRFX_MAX(), NRFX_MIN() and NRFX_ABS() macros.

### Changed
- Updated MDK to version 8.47.0.

### Fixed
- Fixed logging level in nrfx_uart_init() and nrfx_uarte_init() functions. It was reduced from warning to info.
- Fixed IO3 level used in nrfx_qspi_mem_busy_check(). The line is now kept high to prevent memory status from being read incorrectly.

## [2.8.0] - 2022-04-05
### Added
- Added function for getting the currently configured channels in the SAADC driver.
- Added callback for the RXFRAMESTART event in the NFCT driver.
- Added function for transmitting the NFC frame with specified number of bits in the NFCT driver.
- Added functions for getting and setting SEL_RES auto response configuration in the NFCT HAL.

### Changed
- Refactored calibration procedure in the SAADC driver, so it retains previously configured mode.
- Improved management of low frequency clock source during initialization in the CLOCK driver. Compatible source that already runs or is starting during initialization is now used without reconfiguration.
- Improved software-managed triggering of the START task on an END event in the SAADC driver.

### Fixed
- Fixed event processing order in the SAADC driver. Previously, incorrect buffer might have been filled when double-buffered sampling was used with END event and START task being connected through (D)PPI.
- Fixed the limits feature that could be spuriously triggered during calibration procedure in the SAADC driver.

## [2.7.0] - 2021-12-16
### Added
- Added new fields in the driver configuration structures to allow skipping GPIO and/or PSEL register configuration. Affected drivers: I2S, PDM, PWM, QDEC, QSPI, SPI, SPIM, SPIS, TWI, TWIM, TWIS, UART, UARTE.

### Changed
- Updated MDK to version 8.44.2.

### Fixed
- Fixed the inability to start subsequent simple mode conversion from the user callback context in the SAADC driver.

## [2.6.0] - 2021-11-22
### Added
- Added new functions for the GPIOTE peripheral management in the GPIOTE driver. Legacy API is now deprecated.
- Added support for the TIMER CC channels 6 and 7.
- Added the nrfx_flag32_allocator helper layer to unify resource allocation mechanism across several drivers.
- Added functions for getting shortcut bitmask associated with specified channel in the TIMER HAL.
- Added functions for channel allocation and deallocation in the nrfx_gppi helper layer.
- Added the nrf_gpio_reconfigure() function for selective reconfiguration of the specified pin.
- Introduced the NRFX_CLOCK_CONFIG_CT_ENABLED symbol for enabling the calibration timer support in the CLOCK driver. The symbol is enabled by default on supported SoCs.
- Introduced the NRFX_CTZ and NRFX_CLZ macros to allow providing custom implementation for these functionalities. By default `__RBIT()` and `__CLZ()` CMSIS macros are used.
- Introduced the NRFX_ATOMIC_CAS macro to allow providing custom implementation for atomic compare-and-swap functionality.
- Implemented workaround for the nRF9160 anomaly 7 in the NVMC driver.

### Changed
- Updated MDK to version 8.44.1.

### Fixed
- Fixed missing disabling of interrupts in the nrfx_qspi_lfm_start() and nrfx_qspi_lfm_xfer() functions.
- Fixed manual sample fetching when REPORTPER is disabled in the QDEC driver.
- Fixed a race condition in the NFCT driver when starting the transfer.

## [2.5.0] - 2021-05-05
### Added
- Added support for double-buffered transfers in the QSPI driver.
- Added support for one-by-one channel configuration in the SAADC driver.
- Added critical sections for channel allocation and deallocation in the DPPI driver.
- Added support for generating documentation in the RST format using Sphinx tool.
- Added high-drive pin configuration in the QSPI, TWIM, and SPIM drivers.
- Added report period setting of 1 sample in the QDEC HAL.
- Implemented workaround for nRF53 Series anomaly 121 in the QSPI driver.

### Changed
- Updated MDK to version 8.40.2.
- Removed implicit channel deallocation during initialization in the GPIOTE driver.
- Restricted pins that are supported for specific configuration on nRF5340 in the QSPI, TWIM, and SPIM drivers.
- Aligned IRQ handler symbols to new naming scheme for nRF53 Series.

### Fixed
- Fixed register access sequence in the COMP driver.
- Fixed memory corruption when uninitialized channel was freed in the GPIOTE driver.
- Fixed spurious access of DCX pin for every instance of the peripheral when extended features were enabled in the SPIM driver.
- Fixed overwrite of MCUSEL configuration in the nrf_gpio_cfg() function.
- Fixed driver behavior for zero-length transfer in the TWIM driver.
- Fixed internal state handling when transfer is aborted before finishing in the TWI and TWIM drivers.
- Corrected assertions for MPS requirements in the USBD driver.
- Corrected symbol describing maximum data payload for a full-speed isochronous endpoint in the USBD driver.

## [2.4.0] - 2020-11-13
### Added
- Added the channel allocator feature in the GPIOTE driver.
- Added support for registers associated with the Direction Finding feature in the RADIO HAL.
- Added support for the SUBSCRIBE and PUBLISH registers in the following HALs: AAR, CCM, ECB, RADIO, and RNG.
- Added support for the LATENCY register present on nRF5340 in the GPIOTE driver and the GPIOTE HAL.
- Implemented workaround for nRF53 Series anomaly 53 in the POWER driver.

### Changed
- Updated MDK to 8.37.0.

### Fixed
- Fixed unused parameters in the PPI, CLOCK, SPIM and POWER drivers to resolve compilation warnings, such as 'flags' in the SPIM driver.
- Fixed a race condition in the COMP driver when interrupt occured before update of the driver internal state.
- Fixed minor C++ compilation warnings in the drivers.
- Fixed implementation of workaround for nRF52 Series anomaly 197 in the POWER HAL, which was applied in incorrect configuration.
- Fixed implementation of workaround for nRF53 Series anomaly 4 in the CLOCK driver, which was applied in incorrect configuration.
- Fixed the incorrect signature of the nrf_clock_alwaysrun_set() function.

## [2.3.0] - 2020-08-19
### Added
- Added support for nRF52805.
- Implemented workaround for nRF52 Series anomaly 197 in the POWER HAL.
- Implemented workaround for nRF52 Series anomalies 211 and 223 in the USBD driver.
- Added support for the nRF53 Series in the QSPI HAL.
- Added function in the GPIO HAL that returns port index.
- Introduced a QSPI HAL symbol that indicates the availability of the QSPI mode 1.
- Introduced shortcut functionality in the CCM HAL.
- Added function in the TIMER HAL for setting the specified shortcuts.
- Added optional two-stage start procedure of the LFXO in the CLOCK driver.

### Changed
- Updated MDK to 8.35.0.
- Divided network and application core-specific functionalities for nRF5340 in the RESET HAL and the reset reason helper.

### Fixed
- Corrected assertions in the NVMC driver.
- Corrected return types in the CCM HAL.
- Fixed setting of program memory access mode for secure code in the NVMC driver.
- Removed usage of the NRF_UICR symbol on non-secure targets in address validity checks in the NVMC driver.
- Fixed an error message in the SPIM driver that prevented successful compilation with logging enabled.
- Fixed unused parameters in the PPI HAL.

## [2.2.0] - 2020-04-28
### Added
- Added support for nRF52820.
- Added possibility to use the nRF52833 DK in development for nRF52820. LEDs and buttons in this DK use GPIOs that are not present in the actual nRF52820 device, so accesses to these need special handling in the GPIO HAL.
- Added the nrfx_reset_reason helper layer, to facilitate developing generic code that uses the RESETREAS register.
- Implemented workarounds for nRF52 Series anomalies 170 and 196 in the I2S driver.
- Added implicit casting of uint32_t to unsigned long in log messages, to prevent compiler warnings.
- Added missing secure and non-secure peripheral symbol translations for nRF5340 and nRF9160.
- Added functions in the NVMC driver for reading and writing halfword data from the OTP region of the UICR.

### Changed
- Updated MDK to 8.32.1.
- Improved documentation regarding double-buffered reception in the UARTE driver.
- Unified policy of pin cleanup procedure during uninitialization of the drivers. Now every driver restores utilized pins to default setting.
- Removed support for revision Engineering A of nRF52840 in the USBD driver.
- Changed selected while() loops to NRFX_WAIT_FOR() macro in the SPIM, UARTE and CLOCK drivers to improve robustness.

### Fixed
- Fixed assertions that check write addresses in the NVMC driver. Now the UICR memory region is handled properly.
- Fixed a race condition in the CLOCK driver when requested clock was stopped during ramp-up from high priority interrupt.
- Fixed assertions that check pin numbers in the GPIO HAL and GPIOTE driver. Now noncontiguous groups of available pin numbers are handled properly.
- Fixed assertion that checks correctness of limit setting in the SAADC driver.
- Fixed spurious callback invocation when the SPIM driver was reinitialized to blocking mode from non-blocking mode.
- Fixed missing workaround for I2S STOP anomaly for nRF52833.
- Fixed handling of NRFX_SPIM_FLAG_HOLD_XFER setting in the blocking mode of the SPIM driver.
- Fixed void pointer cast in the CLOCK HAL, which could cause memory corruption with specific compiler settings.
- Fixed definition of the NRF_CLOCK_LFCLK_Synth value, to make it available also in builds for nRF5340.

## [2.1.0] - 2020-01-24
### Added
- Added HALs for DCNF, OSCILLATORS, USBREG, and VREQCTRL.
- Added support for 1-MHz clock frequency in TWIM.
- Introduced the NRFX_I2S_STATUS_TRANSFER_STOPPED flag in the I2S driver.
- Introduced the nrfx_power_compat layer that allows use of the nrfx_power API with new SoC.
- Added encryption support in the QSPI driver.
- Added support for USBD in nRF5340.
- Expanded HALs to cover new functions in nRF5340: GPIO, I2S, PDM, POWER, QSPI, and REGULATORS.
- Introduced new clock management system in the CLOCK driver.
- Introduced new audio clock configuration settings in the I2S and PDM drivers for nRF5340.
- Implemented workaround for nRF5340 anomaly 4 in the CLOCK driver.
- Implemented workaround for nRF5340 anomaly 10 in the CCM HAL.
- Implemented workaround for nRF9160 anomaly 21 and nRF5340 anomaly 6 in the NVMC HAL.
- Implemented workaround for nRF9160 anomaly 23 and nRF5340 anomaly 44 in the UARTE driver.
- Introduced the NRFX_TWIM_NO_SPURIOUS_STOP_CHECK flag in the TWIM driver.
- Added functions for getting shortcut configuration in the TWIM HAL.

### Changed
- Updated MDK to 8.30.2.
- Reorganized templates of nrfx_config header files for different SoCs. Now they are included through one common file according to the selected SoC.
- Improved the UARTE driver to consume less current after the driver uninitialization. Now all clocks are disabled properly after uninitialization.
- Improved the GPIOTE driver robustness by setting the LATCH functionality to be used by default.
- Changed names of the frequency divider symbols in the QSPI HAL to reflect the new frequencies in nRF5340. Old API names were preserved and are still supported.
- Improved spurious STOP condition handling in the TWIM driver.
- Improved sampling procedure in the advanced blocking mode in the SAADC driver.
- Improved calibration procedure in the SAADC driver for nRF5340 and nRF9160.

### Fixed
- Fixed address assertions in NVMC driver for the nRF5340 network core.
- Fixed an issue in the TWI driver that would make the driver stuck when a premature STOP condition was generated by a slave device. The driver now handles this situation properly and signals that a bus error occurred.
- Fixed the stopping procedure in the PWM driver. Previously in very specific circumstances the PWM output might be not stopped at all or might be immediately restarted.

## [2.0.0] - 2019-11-06
### Added
- Added support for nRF5340.
- Added HALs for: CACHE, FPU, MUTEX, and RESET.
- Added driver and HAL for IPC.
- Added possibility to configure in UART and UARTE the number of stop bits and the type of parity, when a given SoC allows it.
- Added function in the GPIO HAL for selecting the MCU to control the specified pin.
- Added support for ONESHOT register in the TIMER HAL.
- Added support for LIST feature in HALs for SPIS and TWIS.
- Added possibility to choose TIMER instance used for workarounds in the NFCT driver.

### Changed
- Updated MDK to 8.29.0.
- Enhanced PWM driver API: added the "p_context" parameter to the event handler.
- Updated address and task getters in all HALs to return values as uint32_t type.
- Updated all HAL functions to take the pointer to the structure of registers of the peripheral as their first argument.
- Changed __STATIC_INLINE symbol to NRF_STATIC_INLINE for HALs and NRFX_STATIC_INLINE for drivers.
- Refactored the SAADC driver and HAL.
- Refactored the WDT driver and HAL to support multiple instances.
- Changed nrfx_gpiote_init() function to take the interrupt priority as its parameter. Previously this priority was an nrfx_config option.
- Changed nrf_usbd_ep_all_disable() function to disable really all endpoints. Use nrf_usbd_ep_default_config() to restore the default endpoint configuration.
- Updated nrfx_gpiote_out_init() and nrfx_gpiote_in_init() return codes. Now NRFX_ERROR_INVALID_STATE is changed to NRFX_ERROR_BUSY.
- Replaced the SWI/EGU driver with one for EGU only.
- Aligned symbol names for default IRQ priority in nrfx_config. These symbols are now adhering to the following standard: NRFX_xxx_DEFAULT_CONFIG_IRQ_PRIORITY.
- Changed the way of configuring the MISO pin pull setting in SPI and SPIM drivers. Now it can be set separately for each instance.

### Removed
- Removed deprecated functions from drivers: TWI and TWIM. See migration guide for details.
- Removed deprecated functions from HALs: ECB, NVMC, and TEMP. See migration guide for details.
- Removed redundant bariers in the nrfx_usbd driver.
- Removed the default configuration values for drivers from the nrfx_config header files.

## [1.8.1] - 2019-10-21
### Added
- Added functions in the GPIOTE driver for getting task or event for the specified GPIO pin.

### Changed
- Updated MDK to version 8.27.1.
- Moved the nrfx_gppi helper from helpers/nrfx_gppi/ to helpers/.
- Changed the interrupt initialization in the GPIOTE driver, so that mapping of the GPIOTEx_IRQn enumeration values is no longer needed for nRF9160.

## [1.8.0] - 2019-08-27
### Added
- Added support for nRF52833.
- Added bus recovery feature in the TWI and TWIM drivers.
- Added the nrfx_gppi helper layer to facilitate developing generic code that can utilize PPI or DPPI, depending on which interface is available in a given SoC.

### Changed
- Updated MDK to version 8.27.0.

### Fixed
- Fixed an issue in the TWIM driver that would make the driver stuck when a premature STOP condition was generated by a slave device. The driver now handles this situation properly and signals that a bus error occurred.
- Fixed a frame timing bug in the NFCT driver. Previously, the timing of the SENS_RES response could be violated after the NFCT peripheral was put to the Sleep state with the SLP_REQ command.

## [1.7.2] - 2019-07-25
### Added
- Added functions in the DPPI, GPIOTE, PPI, RTC, and TIMER HALs for getting tasks and events specified by index.
- Added the possibility of suspending transfers in the TWI driver. This allows combining several transfers into one continuous TWI transaction.
- Added termination of transfers at deinitialization of the UARTE driver.
- Added buffer alignment checks in the QSPI driver.
- Introduced the NRFX_OFFSETOF macro that duplicates the functionality of the built-in offsetof() mechanism, but can be used without issues also with non-constant expressions.
- Added an alternative way of ending the DMA transfer loop in the USBD driver.
- Added the CTSTARTED and CTSTOPPED events to the CLOCK HAL.

### Changed
- Removed an assertion that prevented setting the data payload size of isochronous endpoints to zero, to fulfill requirements of the USB 2.0 specification, paragraph 5.6.3.
- Declared the tx_buffer_length field in the UART driver's control block as volatile to prevent issues in case of compilation with high optimization level.

### Fixed
- Fixed an incorrect conversion of frequency values in the RADIO HAL.
- Fixed an incorrectly enabled interrupt in the QSPI driver.
- Corrected the LFCLK source selection values in the template configuration file for nRF9160.
- Fixed support for external LFCLK sources for nRF52811.

## [1.7.1] - 2019-04-08
### Added
- Added functions in the NVMC driver for getting the flash page size, the count of pages and the total flash size.

### Fixed
- Fixed handling of short unaligned write requests (1 or 2 bytes in length) in the nrfx_nvmc_bytes_write() function.

## [1.7.0] - 2019-03-29
### Added
- Added drivers for NVMC and TEMP.
- Added HALs: AAR and FICR.
- Added support for the custom instruction long frame mode in the QSPI driver.

### Changed
- Reworked HAL for NVMC. Now it can be used for all SoCs supported by nrfx.
- Reworked HAL for TEMP.
- Improved documentation. Now it is more precise and can be generated without warnings with newer versions of doxygen.
- Improved the UARTE driver to consume less current after the TX operation. Now at the end of the transmission the transmitter is turned off by the STOPTX task.
- Improved C++ support in drivers. Now fields in structures are filled up in the correct order.
- Changed to size_t the type used for holding the amount of data in the TWIS driver.

### Fixed
- Fixed a race condition in the USBD driver. It could occur when an IN transfer was interrupted by an OUT transaction, which in turn was interrupted by a process with a higher priority.

## [1.6.2] - 2019-02-12
### Added
- Added the possibility to use the macro NRFX_COREDEP_DELAY_US_LOOP_CYCLES to specify the number of cycles consumed by one iteration of the internal loop in the function nrfx_coredep_delay_us().

### Changed
- Updated MDK to version 8.24.1.

## [1.6.1] - 2019-01-29
### Fixed
- Fixed an issue in the NFCT driver that caused a performance loss on nRF52832. The interrupt configuration is now properly restored after the NRFX_NFCT_EVT_FIELD_LOST event.

## [1.6.0] - 2019-01-18
### Added
- Added support for nRF52811.
- Added support for the legacy peripherals SPI, TWI, and UART in nRF52810.
- Added support for SAMPLERATE in nrf_saadc.h.
- Added clearing of the STOPPED event in the nrfx_saadc_init() function to prevent driver deadlock in some cases.
- Added HALs: BPROT, MPU, MWU.
- Added function for reading the pin input buffer configuration in the GPIO HAL.
- Implemented workaround for nRF9160 anomaly 1 in the I2S driver.

### Changed
- Improved handling of hardware anomalies in the USBD driver.
- Updated MDK to version 8.23.1.

### Fixed
- Fixed the condition in NRFX_WAIT_FOR in the nrfx_saadc_abort() function. The macro now correctly waits for a stop of the driver.
- Fixed the pending interrupt clearing in NVIC in the nrfx_usbd_stop() function. The driver now correctly handles power management.
- Fixed the case when nrfx_uarte_tx_in_progress() function would return an incorrect value. The driver now correctly updates the tx_buffer_length variable internally.

## [1.5.0] - 2018-12-12
### Added
- Added support for nRF9160.
- Added allocator for DPPI.
- Added HALs: DPPI, KMU, REGULATORS, SPU, VMC.
- Added support for DPPI subscription and publishing in HALs related to nRF9160.
- Added support for instances 2 and 3 in SPIS, TWIM, TWIS, and UARTE drivers.

### Changed
- Updated MDK to version 8.21.1.

### Fixed
- Corrected NRFX_I2S_CONFIG_RATIO value in nrfx_config.h. It now correctly uses supported value.

## [1.4.0] - 2018-11-30
### Added
- Added the nrfx_is_word_aligned() function for checking whether an address is word-aligned.
- Added HAL for ACL.
- Added functions for disabling and re-enabling interrupts in the SWI driver.
- Added possibility to completely remove interrupt handling from the WDT driver.

### Changed
- Updated the documentation for the nrfx_uarte_rx() function. It now correctly reflects the actual behavior of the function.

### Fixed
- Corrected the type of the nrfx_uarte_xfer_evt_t structure field that holds the amount of transferred bytes.
- Corrected the way of disabling interrupts in the NFCT driver when moving the peripheral to the disabled state.
- Fixed a typo in the name of the bmRequest field in the nrfx_usbd_setup_t structure. The new correct name is bRequest.
- Fixed the nrfx_ppi_channel_fork_assign() function. It now accepts also pre-programmed channels.
- Fixed handling of long custom instruction responses in the QSPI driver.
- Fixed a bug affecting the conversion of time to ticks in the TIMER HAL.

## [1.3.1] - 2018-09-28
### Fixed
- Corrected the type of nrfx_usbd_ep_status_get() return value.
- Corrected calls to undefined macros in NFCT and USBD drivers.

## [1.3.0] - 2018-09-21
### Added
- Added HAL and driver for NFCT.
- Added driver for USBD.
- Added function for setting the burst mode in the SAADC HAL.
- Added the NRFX_ARRAY_SIZE macro.

### Changed
- Moved the implementation of nrfx_power_clock_irq_handler() to nrfx_power.c, removed nrfx_power_clock.c.

### Fixed
- Replaced ARRAY_SIZE macro calls with NRFX_ARRAY_SIZE ones.

## [1.2.0] - 2018-09-06
### Added
- Added function for checking if a specific channel is enabled in the GPIOTE HAL.
- Added support for using the QDEC driver without LED.
- Added functions for modifying only the event endpoint or only the task endpoint in the PPI HAL.
- Added function for reading the pin pull configuration in the GPIO HAL.

### Changed
- Corrected ISOSPLIT enumerator names in the USBD HAL.

### Fixed
- Fixed a double buffering bug that occurred in the UARTE driver after the RX abort.
- Fixed the TXRX transfers in the TWIM driver. They can now be started after transfers that are not ended with the stop condition.

## [1.1.0] - 2018-06-15
### Added
- Implemented workaround for nRF52832 and nRF52840 anomaly 194 in the I2S driver.
- Implemented workaround for nRF52840 anomaly 195 in the SPIM driver.
- Added HALs for CCM, ECB, and RADIO.
- Extended HALs for GPIO, PPI, SAADC, and USBD.
- Added support for external LFCLK sources.

### Changed
- Corrected handling of transfer lengths in the TWI driver.
- Updated MDK to version 8.17.0.

### Fixed
- Fixed logging in the PPI driver.
- Fixed SPIM interrupt definition for nRF52810.
- Fixed Slave Select configuration in the SPIM driver.
- Corrected default settings for NRF_SPIM3.
- Fixed a typo in the UARTE TXDRDY event definition.
- Corrected the TIMEOUT event clearing in the WDT interrupt handler.

## [1.0.0] - 2018-03-21
### Added
- Added the NRFX_WAIT_FOR macro to improve the time-out functionality in QSPI and SAADC drivers.
- Added glue layer macros for checking and modifying the pending status of interrupts.
- Added new enumeration values for interrupts and events in the UARTE HAL.
- Implemented workarounds for nRF52 anomalies 192 and 201 in the CLOCK driver.
- Implemented workaround for nRF52840 anomaly 122 in the QSPI driver.
- Implemented workaround for nRF52840 anomaly 198 in the SPIM driver.

### Changed
- Updated MDK to 8.16.0.
- Extended input pin configuration in the GPIOTE driver.
- Unified the way of checking if a required event handler was provided. Now, all drivers do it with assertions.
- Changed the RNG bias correction configuration option to be enabled by default.
- Refactored the ADC driver and HAL.
- Corrected assertions in the TIMER driver to make it usable in debug version with PPI.
- Improved buffer handling in the I2S driver. The API of the driver has been slightly modified.
- Enhanced SPIS driver API: added the "p_context" parameter, allowed NULL pointers for zero-length buffers.

### Fixed
- Fixed result value casting in the TEMP HAL.
- Fixed types of conversion result and buffer size in the ADC HAL and driver.
- Fixed time-out in the SAADC driver in abort function.

## [0.8.0] - 2017-12-20
### Added
- Added XIP support in the QSPI driver.
- Implemented Errata 132 in the CLOCK driver.
- Added function for checking if a TIMER instance is enabled.
- Added extended SPIM support.

### Changed
- Updated MDK to 8.15.0. Introduced Segger Embedded Studio startup files.
- Updated drivers: COMP, PWM, QDEC, SAADC, SPIS, TIMER, TWI, TWIS.
- Changed the type used for transfer lengths to 'size_t' in drivers: SPI, SPIM, SPIS, TWI, TWIM, TWIS, UART, UARTE. Introduced checking of EasyDMA transfers limits.
- Updated HALs: COMP, NVMC, UART, UARTE, USBD.
- Updated template files and documentation of configuration options.

### Fixed
- Fixed TWI and TWIM drivers so that they now support GPIOs from all ports.
- Fixed definitions related to compare channels in the TIMER HAL.

### Removed
- Removed the possibility of passing NULL instead of configuration to get default settings during drivers initialization.
- Removed support for UART1 and PRS box #5.

## [0.7.0] - 2017-10-20
### Added
- This CHANGELOG.md file.
- README.md file with simple description and explanations.
- HAL for: ADC, CLOCK, COMP, ECB, EGU, GPIO, GPIOTE, I2S, LPCOMP, NVMC, PDM, POWER, PPI, PWM, QDEC, QSPI, RNG, RTC, SAADC, SPI, SPIM, SPIS, ARM(R) SysTick, TEMP, TIMER, TWI, TWIM, TWIS, UART, UARTE, USBD, WDT.
- Drivers for: ADC, CLOCK, COMP, GPIOTE, I2S, LPCOMP, PDM, POWER, PWM, QDEC, QSPI, RNG, RTC, SAADC, SPI, SPIM, SPIS, ARM(R) SysTick, TIMER, TWI, TWIM, TWIS, UART, UARTE, WDT.
- Allocators for: PPI, SWI/EGU.
- MDK in version 8.14.0.
- Offline documentation for every added driver and simple integration description.
- Template integration files.
