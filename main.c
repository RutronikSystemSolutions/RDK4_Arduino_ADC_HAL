/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the RDK4_Arduino_ADC_HAL
*              Application for ModusToolbox.
*
* Related Document: See README.md
*
*
*  Created on: 2023-03-01
*  Company: Rutronik Elektronische Bauelemente GmbH
*  Address: Jonavos g. 30, Kaunas 44262, Lithuania
*  Author: GDR
*
*******************************************************************************
* (c) 2019-2021, Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*
* Rutronik Elektronische Bauelemente GmbH Disclaimer: The evaluation board
* including the software is for testing purposes only and,
* because it has limited functions and limited resilience, is not suitable
* for permanent use under real conditions. If the evaluation board is
* nevertheless used under real conditions, this is done at one’s responsibility;
* any liability of Rutronik is insofar excluded
*******************************************************************************/
#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

#define ARD_CH_NUM		6

void handle_error(void);
cy_rslt_t app_hw_init(void);
static void adc_continuous_event_handler(void* arg, cyhal_adc_event_t event);

/*ADC Clock*/
cyhal_clock_t adc_clock;

/* ADC Object */
cyhal_adc_t adc_obj;

/* ADC Channel Objects */
cyhal_adc_channel_t adc_channels[ARD_CH_NUM];

/* Default ADC configuration */
const cyhal_adc_config_t adc_config =
{
        .continuous_scanning=true,  	// Continuous Scanning is enabled
        .average_count=16,           	// Average count
        .vref=CYHAL_ADC_REF_VDDA,   	// VREF for Single ended channel set to VDDA
        .vneg=CYHAL_ADC_VNEG_VSSA,  	// VNEG for Single ended channel set to VSSA
        .resolution = 12u,          	// 12-bit resolution
        .ext_vref = NC,
        .bypass_pin = NC,
};

/* ADC channel configuration */
const cyhal_adc_channel_config_t channel_config =
{
		.enable_averaging = true,   // Disable averaging for channel
        .min_acquisition_ns = 60000, // Minimum acquisition time set to 60us
        .enabled = true
};

/* ADC data storage */
uint32_t adc_data[ARD_CH_NUM] = {0};

int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
    	handle_error();
    }
    __enable_irq();

    /*Enable debug output via KitProg UART*/
    result = cy_retarget_io_init( KITPROG_TX, KITPROG_RX, CY_RETARGET_IO_BAUDRATE);
    if (result != CY_RSLT_SUCCESS)
    {
        handle_error();
    }
    printf("\x1b[2J\x1b[;H");
    printf("RDK4 Arduino ADC HAL Example.\r\n");

    /*Initialize LEDs*/
    result = cyhal_gpio_init( USER_LED_RED, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    if (result != CY_RSLT_SUCCESS)
    {handle_error();}
    result = cyhal_gpio_init( USER_LED_GREEN, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    if (result != CY_RSLT_SUCCESS)
    {handle_error();}
    result = cyhal_gpio_init( USER_LED_BLUE, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    if (result != CY_RSLT_SUCCESS)
    {handle_error();}

    /*Configure the hardware*/
    result = app_hw_init();
    if (result != CY_RSLT_SUCCESS)
    {
        handle_error();
    }

    printf("Channel A1;  Channel A2;  Channel A3;  Channel A4;  Potentiometer; Channel A6;\r\n");
    for (;;)
    {
    	Cy_SysLib_Delay(1000);
    	printf("A1: %4ld mV, A2: %4ld mV, A3: %4ld mV, A4: %4ld mV, A5: %4ld mV,   A6: %4ld mV\r\n", adc_data[0], adc_data[1], adc_data[2], adc_data[3], adc_data[4], adc_data[5]);
    	cyhal_gpio_toggle(USER_LED_GREEN);
    }
}

void handle_error(void)
{
     /* Disable all interrupts. */
    __disable_irq();

    CY_ASSERT(0);
}

cy_rslt_t app_hw_init(void)
{
	cy_rslt_t result;
	uint32_t actual_sample_rate = 0;

	/*Initialize a Clock for the ADC Peripheral*/
	result = cyhal_clock_get(&adc_clock, &ADC_CLK_obj);
    if(result != CY_RSLT_SUCCESS)
    {
    	goto return_err;
    }
    if (!cyhal_clock_is_enabled(&adc_clock))
    {
    	result = cyhal_clock_set_enabled(&adc_clock, true, true);
    }

	/* Initialize the Arduino ADC Block.*/
	cyhal_adc_free(&adc_obj);
	result = cyhal_adc_init(&adc_obj, ARD_ADC1, &adc_clock);
    if(result != CY_RSLT_SUCCESS)
    {
    	goto return_err;
    }

    /* Initialize a channel A0 and configure it to scan P2_0 in single ended mode. */
    result  = cyhal_adc_channel_init_diff(&adc_channels[0], &adc_obj, ARD_ADC1, CYHAL_ADC_VNEG, &channel_config);
    if(result != CY_RSLT_SUCCESS)
    {goto return_err;}

    /* Initialize a channel A1 and configure it to scan P2_1 in single ended mode. */
    result  = cyhal_adc_channel_init_diff(&adc_channels[1], &adc_obj, ARD_ADC2, CYHAL_ADC_VNEG, &channel_config);
    if(result != CY_RSLT_SUCCESS)
    {goto return_err;}

    /* Initialize a channel A2 and configure it to scan P2_2 in single ended mode. */
    result  = cyhal_adc_channel_init_diff(&adc_channels[2], &adc_obj, ARD_ADC3, CYHAL_ADC_VNEG, &channel_config);
    if(result != CY_RSLT_SUCCESS)
    {goto return_err;}

    /* Initialize a channel A3 and configure it to scan P2_3 in single ended mode. */
    result  = cyhal_adc_channel_init_diff(&adc_channels[3], &adc_obj, ARD_ADC4, CYHAL_ADC_VNEG, &channel_config);
    if(result != CY_RSLT_SUCCESS)
    {goto return_err;}

    /* Initialize a channel A4 and configure it to scan P2_4 in single ended mode. */
    result  = cyhal_adc_channel_init_diff(&adc_channels[4], &adc_obj, ARD_ADC5, CYHAL_ADC_VNEG, &channel_config);
    if(result != CY_RSLT_SUCCESS)
    {goto return_err;}

    /* Initialize a channel A5 and configure it to scan P2_5 in single ended mode. */
    result  = cyhal_adc_channel_init_diff(&adc_channels[5], &adc_obj, ARD_ADC6, CYHAL_ADC_VNEG, &channel_config);
    if(result != CY_RSLT_SUCCESS)
    {goto return_err;}

    /*Set the sample rate*/
    result = cyhal_adc_set_sample_rate(&adc_obj, 10, &actual_sample_rate);
    if(result != CY_RSLT_SUCCESS)
    {goto return_err;}

    cyhal_adc_register_callback(&adc_obj, &adc_continuous_event_handler, NULL);
    cyhal_adc_enable_event(&adc_obj, CYHAL_ADC_EOS, 0, true);

    /* Update ADC configuration */
    result = cyhal_adc_configure(&adc_obj, &adc_config);
    if(result != CY_RSLT_SUCCESS)
    {goto return_err;}

	return_err:
	return result;
}

static void adc_continuous_event_handler(void* arg, cyhal_adc_event_t event)
{
    CY_UNUSED_PARAMETER(arg);
    CY_UNUSED_PARAMETER(event);

    /* Note: arg is configured below to be a pointer to the adc object */
    if(0u != (event & CYHAL_ADC_EOS))
    {
        for(int i = 0; i < ARD_CH_NUM; ++i)
        {
        	adc_data[i] = (int32_t)(cyhal_adc_read_uv(&adc_channels[i])/1000);
        }
        cyhal_gpio_toggle(USER_LED_RED);
    }
}

/* [] END OF FILE */
