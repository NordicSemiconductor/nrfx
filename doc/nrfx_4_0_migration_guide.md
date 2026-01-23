# nrfx 4.0 migration notes

nrfx 4.0 introduces API and structure changes. This guide lists actions required to make your code compatible with these changes.

## BSP (Board Support Package)

- Added new directory that contains:
  - MDK
  - SoC-specific templates
  - SoC-specific defines from nrfx drivers
  - Soc-specific files from soc/ directory (IRQ handlers and interconnect)
<br>Action : Always include `nrfx.h` header globally, do not include sub-headers like:
- `nrf.h`
- `nrf_erratas.h`
- `nrf_mem.h`
- `nrf_peripherals.h`

## Errata

- Reworked the errata system. The workarounds for unsupported SoCs were removed. Summary table [Doxygen file](https://github.com/NordicSemiconductor/nrfx/blob/master/doc/errata.dox) was created, which presents the currently implemented workarounds in nrfx.
- Added the following errata workarounds:
  - nRF52 Series:
    - 58,
    - 173,
    - 174,
    - 214,
  - nRF53 Series:
    - 65,
    - 119,
  - nRF91 Series:
    - 7 (added nrfx_nvmc_uicr_word_write() and fixed nrfx_nvmc_otp_halfword_read())
<br>Action : Remove fixes for errata now handled internally by the nrfx.
- Some errata are now disabled by default, even if the chip should apply them. They should be carefully enabled by the user on a case-by-case manner. The list includes:
  - nRF52 Series erratum 58,
  - nRF52 Series erratum 109.
  - nRF54H Series erratum 62.
<br>Actions :
  - If required, explicitly enable nRF52 Series erratum 58 workaround, by defining `NRF52_ERRATA_58_ENABLE_WORKAROUND` to `1`.
  - If required, explicitly enable nRF52 Series erratum 109 workaround, by defining `NRF52_ERRATA_109_ENABLE_WORKAROUND` to `1`.
  - If required, explicitly enable nRF54H Series erratum 62 workaround, by defining `NRF54H_ERRATA_62_ENABLE_WORKAROUND` to `1`.

## Error codes

All nrfx drivers and helpers now return int errno values. Failed function execution is indicated by negative return value. `nrfx_err_t` type is deprecated. Expressions like `NRFX_ASSERT(err == NRFX_SUCCESS)` will no longer work as expected.
<br>Action : Update the affected code. Update implementation of `NRFX_LOG_ERROR_STRING_GET` macro to new error type.

## Instance interrupt handlers

Some nrfx drivers now expect instance specific interrupt handlers to be defined by users using `NRFX_INSTANCE_IRQ_HANDLER_DEFINE`. Macro expects three parameters - lowercase peripheral name, instance index and address of peripheral instance.
<br>Action : Define instance specific interrupt handler for affected drivers.

## Configuration

- Removed instance specific driver enablement configs, eg. `NRFX_SPIM20_ENABLED`.
    <br>Action : Do not redefine these symbols, nrfx now considers all present peripheral instances to be enabled.
- Drivers are no longer dependent on driver enablement configs, eg. `NRFX_SPIM_ENABLED`.
    <br>Action : Adjust build system to add appropriate driver source files to compilation accordingly, rather that relying on preprocessor symbols to disable driver source file contents.

## HALYs

HALY is now deprecated for all peripherals.
<br>Action : Use respective HAL functions instead or migrate to driver.

## Drivers

### CLOCK

- Split `nrfx_clock` driver into separate drivers for each clock type :
  - `nrfx_clock_hfclk`
  - `nrfx_clock_hfclk192m`
  - `nrfx_clock_hfclkaudio`
  - `nrfx_clock_lfclk`
  - `nrfx_clock_xo`
  - `nrfx_clock_xo24m`
- <br>Action : Adjust build system to add appropriate clock driver source files to compilation. Do not include specific clock driver headers separately.
- `nrfx_clock_hfclk_start`, `nrfx_clock_hfclk_stop`, `nrfx_clock_lfclk_start` and `nrfx_clock_lfclk_stop` functions are no longer deprecated.
    <br>Action : No action required from user.
- `nrfx_clock_hfclk_is_running` function has been renamed to `nrfx_clock_hfclk_running_check`.
    <br>Action : Update the affected code.
- `nrfx_clock_lfclk_is_running` function has been renamed to `nrfx_clock_lfclk_running_check`.
    <br>Action : Update the affected code.

### COMP

- Removed deprecated `nrfx_comp_short_mask_t` enumerator.
    <br>Action : Use `nrf_comp_short_mask_t` instead.
- Removed deprecated `nrfx_comp_evt_en_mask_t` enumerator.
    <br>Action : Use `nrf_comp_int_mask_t` instead.
- Changed type of `ext_ref` and `input` fields in `nrfx_comp_config_t` type to `nrfx_analog_input_t`.
    <br>Action : Update the affected code.
- `nrfx_comp_pin_select` now expects `nrfx_analog_input_t` parameter and returns `int` error value.
    <br>Action : Update the affected code.

### DPPI

- Removed DPPI driver. Use GPPI helper layer instead. If GPPI connection API is used then DPPI channels are handled internally. However, it is possible to allocate a specific group or channel and handle it outside of the GPPI driver, e.g. using HAL.

- Each domain has one unique DPPI instance and domain ID can be used to identify which DPPI instance shall be used to allocate resources. `nrfx_gppi_domain_id_get(addr)` is used to derive the domain ID from a peripheral register address.

    <br>Action : Replace following template code to allocate a specific DPPI channels
```
nrfx_dppi_t dppi = NRFX_DPPI_INSTANCE(20);
uint8_t chan;
nrf_dppi_channel_group_t group_chan;
nrfx_err_t err;

err = nrfx_dppi_channel_alloc(&dppi, &chan);
if (err != NRFX_SUCCESS) return err;

err = nrfx_dppi_group_alloc(&dppi, &group_chan);
if (err != NRFX_SUCCESS) return err;

...

nrfx_dppi_channel_free(&dppi, chan);
nrfx_dppi_group_free(&dppi, group_chan);
```

with

```
uint32_t domain_id = nrfx_gppi_domain_id_get(NRF_DPPIC20);
int chan = nrfx_gppi_channel_alloc(domain_id);
if (chan < 0) return chan; // Negative error code

int group_chan = nrfx_gppi_group_channel_alloc(domain_id);
if (chan < 0) return chan; // Negative error code

...

nrfx_gppi_channel_free(domain_id, (uint8_t)chan);
nrfx_gppi_group_channel_free(domain_id, (uint8_t)group_chan);
```

### EGU

- `NRFX_EGU_INSTANCE` macro for creating `nrfx_egu_t` driver instance now expects peripheral's base address instead of instance number.
    <br>Action : Update the affected code, e.g. from `NRFX_EGU_INSTANCE(20)` to `NRFX_EGU_INSTANCE(NRF_EGU20)`.
- Driver internal state is now kept within driver's instance object. Driver instance object must have static storage to remain valid throughout application runtime. Creating multiple driver instances associated with same hardware peripheral instance will not share peripheral state.
    <br>Action : Declare single driver instance within the system and share it among dependent software components, e.g. by using global scope.
- Some driver functions expect `nrfx_egu_t` parameter to be mutable.
    <br>Action : Update the affected code.
- Driver instance interrupt handler now has to be explicitly defined by user and call `nrfx_egu_irq_handler` with pointer to the driver instance as an argument. Removed `NRFX_EGU_INST_HANDLER_GET` macro.
    <br>Action : Use `NRFX_INSTANCE_IRQ_HANDLER_DEFINE`.

### GPIOTE

- Removed deprecated `nrfx_gpiote_input_config_t` type and single instance API variant.
    <br>Action : Use multi-instance API instead.
- `NRFX_GPIOTE_INSTANCE` macro for creating `nrfx_gpiote_t` driver instance now expects peripheral's base address instead of instance number.
    <br>Action : Update the affected code, e.g. from `NRFX_GPIOTE_INSTANCE(20)` to `NRFX_GPIOTE_INSTANCE(NRF_GPIOTE20)`.
- Driver internal state is now kept within driver's instance object. Driver instance object must have static storage to remain valid throughout application runtime. Creating multiple driver instances associated with same hardware peripheral instance will not share peripheral state.
    <br>Action : Declare single driver instance within the system and share it among dependent software components, e.g. by using global scope.
- Some driver functions expect `nrfx_gpiote_t` parameter to be mutable.
    <br>Action : Update the affected code.
- Driver instance interrupt handler now has to be explicitly defined by user and call `nrfx_gpiote_irq_handler` with pointer to the driver instance as an argument. Removed `NRFX_GPIOTE_INST_HANDLER_GET` macro.
    <br>Action : Use `NRFX_INSTANCE_IRQ_HANDLER_DEFINE`.

### GRTC

- `nrfx_grtc_syscounter_get` takes no arguments, it now returns `uint64_t` syscounter value instead of error code.
    <br>Action : Update the affected code.

### I2S

- Encapsulated `mck_setup`, `ratio` and `enable_bypass` fields from `nrfx_i2s_config_t` type into separate `nrfx_i2s_prescalers_t` type substructure.
    <br>Action : Update the affected code. Content of `nrfx_i2s_prescalers_t` substructure can now be acquired using `nrfx_i2s_prescalers_calc`.
- `NRFX_I2S_INSTANCE` macro for creating `nrfx_i2s_t` driver instance now expects peripheral's base address instead of instance number.
    <br>Action : Update the affected code, e.g. from `NRFX_I2S_INSTANCE(20)` to `NRFX_I2S_INSTANCE(NRF_I2S20)`.
- Driver internal state is now kept within driver's instance object. Driver instance object must have static storage to remain valid throughout application runtime. Creating multiple driver instances associated with same hardware peripheral instance will not share peripheral state.
    <br>Action : Declare single driver instance within the system and share it among dependent software components, e.g. by using global scope.
- Some driver functions expect `nrfx_i2s_t` parameter to be mutable.
    <br>Action : Update the affected code.
- Driver instance interrupt handler now has to be explicitly defined by user and call `nrfx_i2s_irq_handler` with pointer to the driver instance as an argument. Removed `NRFX_I2S_INST_HANDLER_GET` macro.
    <br>Action : Use `NRFX_INSTANCE_IRQ_HANDLER_DEFINE`.

### LPCOMP

- Removed deprecated `config` field in `nrfx_lpcomp_config_t` type.
    <br>Action : Use `reference`, `ext_ref`, `detection` and `hyst` fields.
- Changed type of `ext_ref` and `input` fields in `nrfx_lpcomp_config_t` type to `nrfx_analog_input_t`.
    <br>Action : Update the affected code.
- Removed deprecated `nrfx_lpcomp_enable`.
    <br>Action : Use `nrfx_lpcomp_start` instead.
- Removed deprecated `nrfx_lpcomp_disable`.
    <br>Action : Use `nrfx_lpcomp_stop` instead.

### PDM

- Removed deprecated single instance API variant.
    <br>Action : Use multi-instance API instead.
- Encapsulated `clock_freq`, `prescaler` and `ratio` fields from `nrfx_pdm_config_t` type into separate `nrfx_pdm_prescalers_t` type substructure.
    <br>Action : Update the affected code. Content of `nrfx_pdm_prescalers_t` substructure can now be acquired using `nrfx_pdm_prescalers_calc`.
- `NRFX_PDM_INSTANCE` macro for creating `nrfx_pdm_t` driver instance now expects peripheral's base address instead of instance number.
    <br>Action : Update the affected code, e.g. from `NRFX_PDM_INSTANCE(20)` to `NRFX_PDM_INSTANCE(NRF_PDM20)`.
- Driver internal state is now kept within driver's instance object. Driver instance object must have static storage to remain valid throughout application runtime. Creating multiple driver instances associated with same hardware peripheral instance will not share peripheral state.
    <br>Action : Declare single driver instance within the system and share it among dependent software components, e.g. by using global scope.
- Some driver functions expect `nrfx_pdm_t` parameter to be mutable.
    <br>Action : Update the affected code.
- Driver instance interrupt handler now has to be explicitly defined by user and call `nrfx_pdm_irq_handler` with pointer to the driver instance as an argument. Removed `NRFX_PDM_INST_HANDLER_GET` macro.
    <br>Action : Use `NRFX_INSTANCE_IRQ_HANDLER_DEFINE`.

### PPI

- Removed PPI driver. Use GPPI helper layer instead. If GPPI connection API is used then PPI channel is handled internally. However, it is possible to allocate a specific group or channel and handle it outside of the GPPI driver, e.g. using HAL.

    <br>Action : Replace following template code to allocate a specific PPI channel
```
uint8_t chan;
nrf_ppi_channel_group_t group_chan;
nrfx_err_t err;

err = nrfx_ppi_channel_alloc(&chan);
if (err != NRFX_SUCCESS) return err;

err = nrfx_ppi_group_alloc(&group_chan);
if (err != NRFX_SUCCESS) return err;

...

nrfx_ppi_channel_free(chan);
nrfx_ppi_group_free(group_chan);
```

with

```
int chan = nrfx_gppi_channel_alloc(0);
if (chan < 0) return chan; // Negative error code

int group_chan = nrfx_gppi_group_channel_alloc(0);
if (group_chan < 0) return chan; // Negative error code

...

nrfx_gppi_channel_free(0, (uint8_t)chan);
nrfx_gppi_group_channel_free(0, (uint8_t)group_chan);
```

### PPIB

- Removed PPIB driver. Use GPPI helper layer instead. If GPPI connection API is used then PPIB are handled internally. However, it is possible to allocate a channel in a SoC specific pair of PPIB instances and handle it outside of the GPPI driver, e.g. using HAL.

    <br>Action : Use following code to allocate a PPIB channel
```
#include <soc/interconnect/nrfx_gppi_lumos.h>

...

int chan = nrfx_gppi_channel_alloc(NRFX_GPPI_NODE_PPIB01_20);
if (chan < 0) return chan; // Negative error code

...

nrfx_gppi_channel_free(NRFX_GPPI_NODE_PPIB01_20, (uint8_t)chan);
```

### PWM

- Renamed symbols in `nrfx_pwm_event_t` from `NRFX_PWM_EVT_` to `NRFX_PWM_EVENT_`.
    <br>Action : Update the affected code.
- Renamed `nrfx_pwm_handler_t` to `nrfx_pwm_event_handler_t`.
    <br>Action : Update the affected code.
- Renamed `nrfx_pwm_evt_type_t` to `nrfx_pwm_event_t`.
    <br>Action : Update the affected code.
- `NRFX_PWM_INSTANCE` macro for creating `nrfx_pwm_t` driver instance now expects peripheral's base address instead of instance number.
    <br>Action : Update the affected code, e.g. from `NRFX_PWM_INSTANCE(20)` to `NRFX_PWM_INSTANCE(NRF_PWM20)`.
- Driver internal state is now kept within driver's instance object. Driver instance object must have static storage to remain valid throughout application runtime. Creating multiple driver instances associated with same hardware peripheral instance will not share peripheral state.
    <br>Action : Declare single driver instance within the system and share it among dependent software components, e.g. by using global scope.
- Driver instance interrupt handler now has to be explicitly defined by user and call `nrfx_pwm_irq_handler` with pointer to the driver instance as an argument. Removed `NRFX_PWM_INST_HANDLER_GET` macro.
    <br>Action : Use `NRFX_INSTANCE_IRQ_HANDLER_DEFINE`.
- Some driver functions expect `nrfx_pwm_t` parameter to be mutable.
    <br>Action : Update the affected code.

### QDEC

- `NRFX_QDEC_INSTANCE` macro for creating `nrfx_qdec_t` driver instance now expects peripheral's base address instead of instance number.
    <br>Action : Update the affected code, e.g. from `NRFX_QDEC_INSTANCE(20)` to `NRFX_QDEC_INSTANCE(NRF_QDEC20)`.
- Driver internal state is now kept within driver's instance object. Driver instance object must have static storage to remain valid throughout application runtime. Creating multiple driver instances associated with same hardware peripheral instance will not share peripheral state.
    <br>Action : Declare single driver instance within the system and share it among dependent software components, e.g. by using global scope.
- Some driver functions expect `nrfx_qdec_t` parameter to be mutable.
    <br>Action : Update the affected code.
- Driver instance interrupt handler now has to be explicitly defined by user and call `nrfx_qdec_irq_handler` with pointer to the driver instance as an argument. Removed `NRFX_QDEC_INST_HANDLER_GET` macro.
    <br>Action : Use `NRFX_INSTANCE_IRQ_HANDLER_DEFINE`.

### SAADC

- Changed type of `pin_p` and `pin_n` fields in `nrfx_saadc_channel_t` type to `nrfx_analog_input_t`.
    <br>Action : Update the affected code.

### SPIM

- Renamed `nrfx_spim_evt_type_t` to `nrfx_spim_event_type_t`.
    <br>Action : Update the affected code.
- Renamed `nrfx_spim_evt_t` to `nrfx_spim_event_t`.
    <br>Action : Update the affected code.
- Renamed `nrfx_spim_evt_handler_t` to `nrfx_spim_event_handler_t`.
    <br>Action : Update the affected code.
- Driver no longer validates whether an instance supports SPIM extended features.
    <br>Action : Provide proper configuration in `nrfx_spim_config_t` based on hardware capabilities of an instance. Refer to Product Specification for details.
- `NRFX_SPIM_INSTANCE` macro for creating `nrfx_spim_t` driver instance now expects peripheral's base address instead of instance number.
    <br>Action : Update the affected code, e.g. from `NRFX_SPIM_INSTANCE(20)` to `NRFX_SPIM_INSTANCE(NRF_SPIM20)`.
- Driver internal state is now kept within driver's instance object. Driver instance object must have static storage to remain valid throughout application runtime. Creating multiple driver instances associated with same hardware peripheral instance will not share peripheral state.
    <br>Action : Declare single driver instance within the system and share it among dependent software components, e.g. by using global scope.
- Some driver functions expect `nrfx_spim_t` parameter to be mutable.
    <br>Action : Update the affected code.
- Driver instance interrupt handler now has to be explicitly defined by user and call `nrfx_spim_irq_handler` with pointer to the driver instance as an argument. Removed `NRFX_SPIM_INST_HANDLER_GET` macro.
    <br>Action : Use `NRFX_INSTANCE_IRQ_HANDLER_DEFINE`.

### SPIS

- Renamed `nrfx_spis_evt_type_t` to `nrfx_spis_event_type_t`.
    <br>Action : Update the affected code.
- Renamed `nrfx_spis_evt_t` to `nrfx_spis_event_t`.
    <br>Action : Update the affected code.
- `NRFX_SPIS_INSTANCE` macro for creating `nrfx_spis_t` driver instance now expects peripheral's base address instead of instance number.
    <br>Action : Update the affected code, e.g. from `NRFX_SPIS_INSTANCE(20)` to `NRFX_SPIS_INSTANCE(NRF_SPIS20)`.
- Driver internal state is now kept within driver's instance object. Driver instance object must have static storage to remain valid throughout application runtime. Creating multiple driver instances associated with same hardware peripheral instance will not share peripheral state.
    <br>Action : Declare single driver instance within the system and share it among dependent software components, e.g. by using global scope.
- Some driver functions expect `nrfx_spis_t` parameter to be mutable.
    <br>Action : Update the affected code.
- Driver instance interrupt handler now has to be explicitly defined by user and call `nrfx_spis_irq_handler` with pointer to the driver instance as an argument. Removed `NRFX_SPIS_INST_HANDLER_GET` macro.
    <br>Action : Use `NRFX_INSTANCE_IRQ_HANDLER_DEFINE`.

### TIMER

- `NRFX_TIMER_INSTANCE` macro for creating `nrfx_timer_t` driver instance now expects peripheral's base address instead of instance number.
    <br>Action : Update the affected code, e.g. from `NRFX_TIMER_INSTANCE(20)` to `NRFX_TIMER_INSTANCE(NRF_TIMER20)`.
- Driver internal state is now kept within driver's instance object. Driver instance object must have static storage to remain valid throughout application runtime. Creating multiple driver instances associated with same hardware peripheral instance will not share peripheral state.
    <br>Action : Declare single driver instance within the system and share it among dependent software components, e.g. by using global scope.
- Some driver functions expect `nrfx_timer_t` parameter to be mutable.
    <br>Action : Update the affected code.
- Driver instance interrupt handler now has to be explicitly defined by user and call `nrfx_timer_irq_handler` with pointer to the driver instance as an argument. Removed `NRFX_TIMER_INST_HANDLER_GET` macro.
    <br>Action : Use `NRFX_INSTANCE_IRQ_HANDLER_DEFINE`.

### TWIM

- Renamed `nrfx_twim_evt_type_t` to `nrfx_twim_event_type_t`.
    <br>Action : Update the affected code.
- Renamed `nrfx_twim_evt_t` to `nrfx_twim_event_t`.
    <br>Action : Update the affected code.
- Renamed `nrfx_twim_evt_handler_t` to `nrfx_twim_event_handler_t`.
    <br>Action : Update the affected code.
- `NRFX_TWIM_INSTANCE` macro for creating `nrfx_twim_t` driver instance now expects peripheral's base address instead of instance number.
    <br>Action : Update the affected code, e.g. from `NRFX_TWIM_INSTANCE(20)` to `NRFX_TWIM_INSTANCE(NRF_TWIM20)`.
- Driver internal state is now kept within driver's instance object. Driver instance object must have static storage to remain valid throughout application runtime. Creating multiple driver instances associated with same hardware peripheral instance will not share peripheral state.
    <br>Action : Declare single driver instance within the system and share it among dependent software components, e.g. by using global scope.
- Some driver functions expect `nrfx_twim_t` parameter to be mutable.
    <br>Action : Update the affected code.
- Driver instance interrupt handler now has to be explicitly defined by user and call `nrfx_twim_irq_handler` with pointer to the driver instance as an argument. Removed `NRFX_TWIM_INST_HANDLER_GET` macro.
    <br>Action : Use `NRFX_INSTANCE_IRQ_HANDLER_DEFINE`.

### TWIS

- Renamed `nrfx_twis_evt_type_t` to `nrfx_twis_event_type_t`.
    <br>Action : Update the affected code.
- Renamed `nrfx_twis_evt_t` to `nrfx_twis_event_t`.
    <br>Action : Update the affected code.
- `NRFX_TWIS_INSTANCE` macro for creating `nrfx_twis_t` driver instance now expects peripheral's base address instead of instance number.
    <br>Action : Update the affected code, e.g. from `NRFX_TWIS_INSTANCE(20)` to `NRFX_TWIS_INSTANCE(NRF_TWIS20)`.
- Driver internal state is now kept within driver's instance object. Driver instance object must have static storage to remain valid throughout application runtime. Creating multiple driver instances associated with same hardware peripheral instance will not share peripheral state.
    <br>Action : Declare single driver instance within the system and share it among dependent software components, e.g. by using global scope.
- Some driver functions expect `nrfx_twis_t` parameter to be mutable.
    <br>Action : Update the affected code.
- Driver instance interrupt handler now has to be explicitly defined by user and call `nrfx_twis_irq_handler` with pointer to the driver instance as an argument. Removed `NRFX_TWIS_INST_HANDLER_GET` macro.
    <br>Action : Use `NRFX_INSTANCE_IRQ_HANDLER_DEFINE`.

### UARTE

- Renamed `nrfx_uarte_evt_type_t` to `nrfx_uarte_event_type_t`.
    <br>Action : Update the affected code.
- Removed deprecated `nrfx_uarte_rx` function.
    <br>Action : Use `nrfx_uarte_rx_enable` and `nrfx_uarte_rx_buffer_set` instead.
- `NRFX_UARTE_INSTANCE` macro for creating `nrfx_uarte_t` driver instance now expects peripheral's base address instead of instance number.
    <br>Action : Update the affected code, e.g. from `NRFX_UARTE_INSTANCE(20)` to `NRFX_UARTE_INSTANCE(NRF_UARTE20)`.
- Driver internal state is now kept within driver's instance object. Driver instance object must have static storage to remain valid throughout application runtime. Creating multiple driver instances associated with same hardware peripheral instance will not share peripheral state.
    <br>Action : Declare single driver instance within the system and share it among dependent software components, e.g. by using global scope.
- Some driver functions expect `nrfx_uarte_t` parameter to be mutable.
    <br>Action : Update the affected code.
- Driver instance interrupt handler now has to be explicitly defined by user and call `nrfx_uarte_irq_handler` with pointer to the driver instance as an argument. Removed `NRFX_UARTE_INST_HANDLER_GET` macro.
    <br>Action : Use `NRFX_INSTANCE_IRQ_HANDLER_DEFINE`.

### WDT

- Removed deprecated event handler type expecting single parameter.
    <br>Action : Use new variant instead.
- Removed deprecated variant of `nrfx_wdt_init` taking 3 parameters.
    <br>Action : Use new variant instead.
- `NRFX_WDT_INSTANCE` macro for creating `nrfx_wdt_t` driver instance now expects peripheral's base address instead of instance number.
    <br>Action : Update the affected code, e.g. from `NRFX_WDT_INSTANCE(20)` to `NRFX_WDT_INSTANCE(NRF_WDT20)`.
- Driver internal state is now kept within driver's instance object. Driver instance object must have static storage to remain valid throughout application runtime. Creating multiple driver instances associated with same hardware peripheral instance will not share peripheral state.
    <br>Action : Declare single driver instance within the system and share it among dependent software components, e.g. by using global scope.
- Some driver functions expect `nrfx_wdt_t` parameter to be mutable.
    <br>Action : Update the affected code.
- Driver instance interrupt handler now has to be explicitly defined by user and call `nrfx_wdt_irq_handler` with pointer to the driver instance as an argument. Removed `NRFX_WDT_INST_HANDLER_GET` macro.
    <br>Action : Use `NRFX_INSTANCE_IRQ_HANDLER_DEFINE`.

## Helpers

### GPPI

GPPI has been reworked to accommodate for a multi-instance DPPI architecture present in the newest SoC, e.g. nRF54 famlily. GPPI focuses on establishing and managing connections between endpoints or domains rather than a particular channels. Connection can spread across multiple and consists of resources in mutliple DPPIC and PPIB instances. Legacy architecture with a single PPI or DPPIC instance can be seen as the simplest example. GPPI API provides API for a basic one-to-one connection and API that allows to setup more complex many-to-many, spreading across multiple domains, connections.
- New GPPI API has limited support on nRF54H20. Full support requires a dedicated Ironside service which is planned in the near future. Support is limited to one-to-one connections: `nrfx_gppi_conn_alloc`, `nrfx_gppi_conn_free`, `nrfx_gppi_conn_enable`, `nrfx_gppi_conn_disable`. It is using legacy GPPI implementation underneath.
- Contrary to the legacy solution, GPPI needs to be initialized prior to using the GPPI API. Ensure
  that `nrfx_gppi_init` is called before calling any GPPI API.
- Removed `nrfx_gppi_channel_group_t` and `nrfx_gppi_task_t` enumerators and `nrfx_gppi_channel_check`, `nrfx_gppi_channels_disable_all`, `nrfx_gppi_task_endpoint_setup`, `nrfx_gppi_channel_endpoints_setup`, `nrfx_gppi_channel_endpoints_clear`, `nrfx_gppi_task_endpoint_clear`, `nrfx_gppi_fork_endpoint_setup`, `nrfx_gppi_fork_endpoint_clear`, `nrfx_gppi_fork_endpoint_clear` , `nrfx_gppi_task_trigger`, `nrfx_gppi_task_address_get`, `nrfx_gppi_group_disable_task_get`, `nrfx_gppi_group_enable_task_get` and `nrfx_gppi_edge_connection_setup` functions.
    <br>Action : Use new API.
- Removed `nrfx_gppi_event_endpoint_setup`.
    <br>Action : Use `nrfx_gppi_ep_attach` instead.
- Removed `nrfx_gppi_event_endpoint_clear`.
    <br>Action : Use nrfx_gppi_ep_clear.
- `nrfx_gppi_channels_enable` and `nrfx_gppi_channels_enable` functions now expects additional `domain_id` parameter which enables a particular channel in a specific domain.
    <br>Action : Update the affected code. Use domain ID returned by `nrfx_gppi_domain_id_get`. For controlling all channels within a connection, use `nrfx_gppi_conn_enable` and `nrfx_gppi_conn_disable`.

- Some API accepts domain ID as a parameter. Each domain has one unique DPPI instance and domain ID can be used to identify which DPPI instance shall be used to allocate resources. `nrfx_gppi_domain_id_get(addr)` is used to derive the domain ID from a peripheral register address.
- Reworked connection workflow using DPPI.
    <br>Action : Replace following template code
```
static uint8_t dppi_channel;
nrfx_dppi_t dppi = NRFX_DPPI_INSTANCE(0);

nrfx_dppi_channel_alloc(&dppi, &dppi_channel);

nrf_<a>_subscribe_set(p_reg_a, A_TASK, &dppi_channel);
nrf_<b>_publish_set(p_reg_b, B_EVENT, &dppi_channel);

nrfx_dppi_channel_enable(&dppi, dppi_channel);
...

nrfx_dppi_channel_disable(&dppi, dppi_channel);
```

with

```
uint32_t eep, tep;
nrfx_gppi_handle_t gppi_handle;

tep_a = nrf_<a>_task_address_get(p_reg_a, A_TASK);
eep_b = nrf_<b>_event_address_get(p_reg_b, B_EVENT);

nrfx_gppi_conn_alloc(eep_a, tep_b, &gppi_handle);

nrfx_gppi_conn_enable(gppi_handle);

...

nrfx_gppi_conn_disable(gppi_handle);
```

- Reworked multiple endpoints workflow using DPPI.
    <br>Action : Replace following template code
```
nrf_<c>_subscribe_set(p_reg_c, C_TASK, &dppi_channel);
```

with

```
tep_c = nrf_<c>_task_address_get(p_reg_c, C_TASK);

nrfx_gppi_ep_attach(tep_c, gppi_handle);
```

- Reworked channel grouping using GPPI.
    <br>Action : Replace following template code
```
uint8_t chan_a, chan_b;

nrfx_gppi_channel_alloc(&chan_a);
nrfx_gppi_channel_alloc(&chan_b);

eep_a = nrf_<a>_event_address_get(p_reg_a, A_EVENT);
eep_b = nrf_<b>_event_address_get(p_reg_b, B_EVENT);
tep_c = nrf_<c>_task_address_get(p_reg_c, C_TASK);

nrfx_gppi_group_clear(NRFX_GPPI_CHANNEL_GROUP0);
nrfx_gppi_group_disable(NRFX_GPPI_CHANNEL_GROUP0);

nrfx_gppi_channels_include_in_group(BIT(chan_a) | BIT(chan_b),
                                    NRFX_GPPI_CHANNEL_GROUP0);

nrfx_gppi_channel_endpoints_setup(chan_a, eep_a,
                                  nrfx_gppi_task_address_get(NRFX_GPPI_TASK_CHG0_EN));
nrfx_gppi_channel_endpoints_setup(chan_b, eep_b, tep_c);
nrfx_gppi_fork_endpoint_setup(chan_b,
                              nrfx_gppi_task_address_get(NRFX_GPPI_TASK_CHG0_DIS));

nrfx_gppi_channels_enable(BIT(chan_a) | BIT(chan_b));

...

nrfx_gppi_channels_disable_all(BIT(chan_a) | BIT(chan_b));

nrfx_gppi_channels_remove_from_group(BIT(chan_a) | BIT(chan_b),
                                     NRFX_GPPI_CHANNEL_GROUP0);

nrfx_gppi_fork_endpoint_clear(chan_b,
                              nrfx_gppi_task_address_get(NRFX_GPPI_TASK_CHG0_DIS));
nrfx_gppi_channel_endpoints_clear(chan_b, eep_b, tep_c);
nrfx_gppi_channel_endpoints_clear(chan_a, eep_a,
                                  nrfx_gppi_task_address_get(NRFX_GPPI_TASK_CHG0_EN));
```

with

```
nrfx_gppi_handle_t gppi_handle, gppi_handle2;
nrfx_gppi_group_handle_t gppi_group;

eep_a = nrf_<a>_event_address_get(p_reg_a, A_EVENT);
eep_b = nrf_<b>_event_address_get(p_reg_b, B_EVENT);
tep_c = nrf_<c>_task_address_get(p_reg_c, C_TASK);

nrfx_gppi_group_alloc(nrfx_gppi_domain_id_get(eep_a), &gppi_group);

nrfx_gppi_conn_alloc(eep_a, nrfx_gppi_group_task_en_addr(gppi_group), &gppi_handle);
nrfx_gppi_conn_alloc(eep_b, tep_c, &gppi_handle2);

nrfx_gppi_ep_attach(nrfx_gppi_group_task_dis_addr(gppi_group), gppi_handle2);

nrfx_gppi_group_ep_add(gppi_group, eep_a);
nrfx_gppi_group_ap_add(gppi_group, eep_b);

nrfx_gppi_conn_enable(gppi_handle);
nrfx_gppi_conn_enable(gppi_handle2);

...

nrfx_gppi_conn_disable(gppi_handl2);
nrfx_gppi_conn_disable(gppi_handle);

nrfx_gppi_ep_clear(eep_a);
nrfx_gppi_ep_clear(eep_b);
nrfx_gppi_ep_clear(tep_c);
```

## HALs

### CACHE

- `nrf_cache_data_get`, `nrf_cache_tag_get`, `nrf_cache_line_validity_check`, `nrf_cache_mru_get`, `nrf_cache_data_unit_validity_check` and `nrf_cache_is_data_unit_dirty_check` now expect `NRF_CACHE_Type` as their first parameter.
   <br>Action : Update the affected code by using `NRF_CACHE`, `NRF_DCACHE` or `NRF_ICACHE`.

### CLOCK

- Removed deprecated `nrf_clock_lf_actv_src_get`, `nrf_clock_lf_is_running` and `nrf_clock_hf_is_running` functions and `nrf_clock_start_task_status_t` enumerator.
    <br>Action: Use `nrf_clock_is_running` instead.
- Removed deprecated `nrf_clock_lf_start_task_status_get` and `nrf_clock_hf_start_task_status_get` functions.
    <br>Action : Use `nrf_clock_start_task_check` instead.

### COMP

- Renamed `nrf_isource_t` type to `nrf_comp_isource_t`.
    <br>Action : Update the affected code.

### LPCOMP

- Removed deprecated `NRF_LPCOMP_REF_EXT_REF0` and `NRF_LPCOMP_REF_EXT_REF1` symbols from `nrf_lpcomp_ref_t` enumerator.
    <br>Action : Use `NRF_LPCOMP_EXT_REF_REF0` or `NRF_LPCOMP_EXT_REF_REF1` symbols from `nrf_lpcomp_ext_ref_t` enumerator instead.
- Removed deprecated `nrf_lpcomp_configure` function and `nrf_lpcomp_config_t` type.
    <br>Action : Use dedicated functions to configure specific parameters.

### POWER

- Removed deprecated `nrf_power_rampower_t` enumerator.
    <br>Action : Use `NRF_POWER_RAMPOWER_S0POWER_POS` or `NRF_POWER_RAMPOWER_S0RETENTION_POS` symbols instead.
- Removed deprecated `nrf_power_vreg_enable` and `nrf_power_vreg_disable` functions.
    <br>Action : Use `nrf_power_vreg_set` instead.
- Removed deprecated `nrf_power_vreg_enable_check` function.
    <br>Action : Use `nrf_power_vreg_get` instead.

### RESET

- Removed `NRF_RESET_HAS_APPLICATION` symbol.
    <br>Action : Update the affected code.

### SAADC

- Removed deprecated `NRF_SAADC_8BIT_SAMPLE_WIDTH` symbol.
    <br>Action : Update the affected code, all SAADC samples are 16 bits wide now.
- Defined `nrf_saadc_value_t` type as `int16_t`. `nrf_saadc_value_min_get`, `nrf_saadc_value_max_get`, `nrf_saadc_value_min_get` and `nrf_saadc_value_max_get` functions now return `nrf_saadc_value_t` type.
    <br>Action : Update the affected code.

## Others

### nrfx_coredep

- Moved from `soc` directory to `lib` directory.
    <br>Action : Update the inclusion paths.
