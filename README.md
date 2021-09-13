# ESPHome Components 
This is my particular collection of ESPHome components to add support for unsupported devices on mainline ESPHome.
All content is published under the ESPHome license.

## MCP4728
MCP4728 is a quad channel, 12-bit voltage output Digital-to-Analog Converter with non-volatile memory and I2C compatible Serial Interface.
 * Supported features
   * Each channel is exported as a `float output`.
   * Output Vref can be selected between `vdd` and `internal` (2.048V).
   * For `internal` Vref, two levels of gain can be selected (`X1` (default) and `X2`).
   * Two write modes are supported:
     * MultiWrite: write all channel settings without writing to non-volatile memory (EEPROM). This is the default and recommended mode.
     * SequentialWrite: write all channel settings to non-volatile memory (EEPROM) and apply this changes.
 * Unsupported features
   * Power-down mode selection (`NORMAL` is selected for all channels).
   * FastWrite mode as this requires the LDAC pin.
   * SingleWrite mode.

Check [example_mcp4728.yaml](./example_mcp4728.yaml) for a reference usage file.

## MAX44009
The MAX44009 is an ambient light sensor featuring an I2C digital output.
 * Supported features
   * Measure lux intensity
   * Measure mode selection:
     * Low power: The IC measures lux intensity only once every 800ms regardless of integration time.
     * Continuous mode: The IC continuously measures lux intensity.
     * Auto: Continuous mode for an update interval < 800ms, low power mode otherwise.
   * The device always runs on auto mode (hardware default).
 * Unsupported features
   * Manual mode.
   * Advanced power saving features (timers, thresholds, INT pin).

Check [example_max44009.yaml](./example_max44009.yaml) for a reference usage file.

## SI1145/46/47
The SI1145 is a sensor IC with a calibrated light sensing algorithm that can calculate UV Index. It doesn't contain an actual UV sensing element, instead it approximates it based on visible & IR light from the sun.
 * Supported features
   * Measure visible light (`visible`)
   * Measure IR light (`infrared`)
   * Measure UV index (`uvindex`)
   * Combine visible and IR sensors to approximate total lux (`calculated_lux`)
   * Auto range and gain `mode: auto`
   * Manual range and gain `mode: manual`
 * Unsupported features
   * IR based proximity sensor
   * Relative temperature sensor

Check [example_si1145.yaml](./example_si1145.yaml) for a reference usage file.

## UARTPIN
This component is intended to support relay modules controllable through UART (serial) like the relay modules from LC Technology. Currently, the LC Technology modules based on Nuvoton microcontroller are not supported.
 * Supported features
   * Send init data after a delay.
   * Send data to set pin/relay high.
   * Send data to set pin/relay low.

Check [example_uartpin_lctech.yaml](./example_uartpin_lctech.yaml) for a reference usage file for an LC Technology Dual Relay module.
