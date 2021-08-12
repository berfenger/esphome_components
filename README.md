# ESPHome Components 
This is my particular collection of ESPHome components to add support for unsupported devices on mainline ESPHome.
All content is published under the ESPHome license.

## MCP4728
MCP4728 is a quad channel, 12-bit voltage output Digital-to-Analog Converter with non-volatile memory and I2C compatible Serial Interface.
 * Supported features
   * Each channel is exported as a `float output`.
   * Output Vref can be selected between VDD and internal 2.048V.
   * For internal 2.048V Vref, two levels of gain can be selected (X1 (default), X2).
   * Two write modes are supported:
     * MultiWrite: write all channel settings without writing to non-volatile memory (EEPROM). This is the default and recommended mode.
     * SequentialWrite: write all channel settings to non-volatile memory (EEPROM) and apply this changes.
 * Unsupported features
   * Power-down mode selection (`NORMAL` is selected for all channels).
   * FastWrite mode as this requires the LDAC pin.
   * SingleWrite mode.

See [example_mcp4728.yaml](./example_mcp4728.yaml) for a reference usage file.