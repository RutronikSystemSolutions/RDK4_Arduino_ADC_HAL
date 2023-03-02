# RDK4 Arduino ADC HAL Code Example

Rutronik Development Kit 4 Programmable System-on-Chip CY8C4149AZE-S598 "RDK3 Arduino ADC HAL" Code Example. 

This example demonstrates how to use the HAL library to measure all the ADC channels on the Arduino ADC header. 

 <img src="images/rdk4_top.jpg" style="zoom:25%;" />

## Requirements

- [ModusToolbox™ software](https://www.cypress.com/products/modustoolbox-software-environment) v3.0

### Using the code example with a ModusToolbox™ IDE:

1. Import the project: **File** > **Import...** > **General** > **Existing Projects into Workspace** > **Next**.
2. Select the directory where **"RDK4_Arduino_ADC_HAL"** resides and click  **Finish**.
3. Update the libraries using a **"Library Manager"** tool.
4. Select and build the project **Project ** > **Build Project**.

### Operation

The firmware example uses KitProg3 Debug UART for the debug output. The ADC peripheral measures all the Arduino ADC channels continuously in a callback function:

```
static void adc_continuous_event_handler(void* arg, cyhal_adc_event_t event)
```

The ADC initialization is done in a function:

```
cy_rslt_t app_hw_init(void)
```

The data is read from the data array and printed to the terminal every second:

<img src="images/debug_output.png" style="zoom:100%;" />

### Debugging

If you successfully have imported the example, the debug configurations are already prepared to use with a the KitProg3, MiniProg4, or J-link. Open the ModusToolbox™ perspective and find the Quick Panel. Click on the desired debug launch configuration and wait for the programming to complete and the debugging process to start.

<img src="images/debug_start.png" style="zoom:100%;" />

## Legal Disclaimer

The evaluation board including the software is for testing purposes only and, because it has limited functions and limited resilience, is not suitable for permanent use under real conditions. If the evaluation board is nevertheless used under real conditions, this is done at one’s responsibility; any liability of Rutronik is insofar excluded. 

<img src="images/rutronik_origin_kaunas.png" style="zoom:50%;" />



