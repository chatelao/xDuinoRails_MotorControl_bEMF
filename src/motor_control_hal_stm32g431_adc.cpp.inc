#if defined(ARDUINO_ARCH_STM32)

#include <Arduino.h>
#include "pinmap.h"

// --- Helper function to get DMA channel and IRQn ---
void get_dma_info(ADC_TypeDef* adc_instance, DMA_Channel_TypeDef** dma_channel, IRQn_Type* irqn) {
    if (adc_instance == ADC1) {
        *dma_channel = DMA1_Channel1;
        *irqn = DMA1_Channel1_IRQn;
    } else if (adc_instance == ADC2) {
        *dma_channel = DMA1_Channel2;
        *irqn = DMA1_Channel2_IRQn;
    }
    // Add other ADC instances as needed
}


void hal_motor_init_bemf_adc(uint8_t bemf_a_pin, uint8_t bemf_b_pin, uint8_t motor_id) {
    if (motor_id >= MAX_MOTORS) {
        return;
    }

    MotorContext* context = &motor_contexts[motor_id];

    // --- Pin Mapping ---
    PinName pin_a = digitalPinToPinName(bemf_a_pin);
    PinName pin_b = digitalPinToPinName(bemf_b_pin);

    ADC_TypeDef *ADC_Instance_a = (ADC_TypeDef *)pinmap_peripheral(pin_a, PinMap_ADC);
    ADC_TypeDef *ADC_Instance_b = (ADC_TypeDef *)pinmap_peripheral(pin_b, PinMap_ADC);

    if (ADC_Instance_a == NP || ADC_Instance_b == NP) {
        return;
    }
    // For now, we only support both pins on the same ADC
    if (ADC_Instance_a != ADC_Instance_b) {
        return;
    }

    context->hadc.Instance = ADC_Instance_a;

    // --- ADC Clock Enable ---
    // This is handled by the pinmap_function, so we don't need to do it manually.

    // --- ADC Configuration ---
    context->hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
    context->hadc.Init.Resolution = ADC_RESOLUTION_12B;
    context->hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    context->hadc.Init.ScanConvMode = ADC_SCAN_ENABLE;
    context->hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    context->hadc.Init.LowPowerAutoWait = DISABLE;
    context->hadc.Init.ContinuousConvMode = DISABLE;
    context->hadc.Init.NbrOfConversion = 2;
    context->hadc.Init.DiscontinuousConvMode = DISABLE;
    context->hadc.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO;
    context->hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
    context->hadc.Init.DMAContinuousRequests = ENABLE;
    context->hadc.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
    context->hadc.Init.OversamplingMode = DISABLE;
    HAL_ADC_Init(&context->hadc);


    // --- ADC Channel Configuration ---
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = STM_PIN_CHANNEL(pinmap_function(pin_a, PinMap_ADC));
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    HAL_ADC_ConfigChannel(&context->hadc, &sConfig);

    sConfig.Channel = STM_PIN_CHANNEL(pinmap_function(pin_b, PinMap_ADC));
    sConfig.Rank = ADC_REGULAR_RANK_2;
    HAL_ADC_ConfigChannel(&context->hadc, &sConfig);

    // --- GPIO Configuration ---
    pinmap_pinout(pin_a, PinMap_ADC);
    pinmap_pinout(pin_b, PinMap_ADC);

}


void hal_motor_init_bemf_dma(hal_bemf_update_callback_t callback, uint8_t motor_id) {
    if (motor_id >= MAX_MOTORS) {
        return;
    }

    MotorContext* context = &motor_contexts[motor_id];
    context->bemf_callback = callback;

    // --- DMA Clock Enable ---
    __HAL_RCC_DMA1_CLK_ENABLE();

    // --- DMA Configuration ---
    DMA_Channel_TypeDef* dma_channel;
    IRQn_Type irqn;
    get_dma_info(context->hadc.Instance, &dma_channel, &irqn);

    context->hdma_adc.Instance = dma_channel;
    context->hdma_adc.Init.Request = (context->hadc.Instance == ADC1) ? DMA_REQUEST_ADC1 : DMA_REQUEST_ADC2;
    context->hdma_adc.Init.Direction = DMA_PERIPH_TO_MEMORY;
    context->hdma_adc.Init.PeriphInc = DMA_PINC_DISABLE;
    context->hdma_adc.Init.MemInc = DMA_MINC_ENABLE;
    context->hdma_adc.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    context->hdma_adc.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    context->hdma_adc.Init.Mode = DMA_CIRCULAR;
    context->hdma_adc.Init.Priority = DMA_PRIORITY_HIGH;
    HAL_DMA_Init(&context->hdma_adc);

    __HAL_LINKDMA(&context->hadc, DMA_Handle, context->hdma_adc);

    // --- Start ADC with DMA ---
    HAL_ADC_Start_DMA(&context->hadc, (uint32_t*)context->bemf_buffer, BEMF_RING_BUFFER_SIZE * 2);

    // --- Configure Timer Trigger ---
    // The timer used for PWM will be used to trigger the ADC
    // This will be configured in the PWM init function
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&context->htim, &sMasterConfig);


    // --- Interrupt Configuration ---
    HAL_NVIC_SetPriority(irqn, 0, 0);
    HAL_NVIC_EnableIRQ(irqn);
}

// --- Interrupt Handlers ---
extern "C" {
    void DMA1_Channel1_IRQHandler(void) {
        for (int i = 0; i < MAX_MOTORS; i++) {
            if (motor_contexts[i].hdma_adc.Instance == DMA1_Channel1) {
                HAL_DMA_IRQHandler(&motor_contexts[i].hdma_adc);
            }
        }
    }
    void DMA1_Channel2_IRQHandler(void) {
        for (int i = 0; i < MAX_MOTORS; i++) {
            if (motor_contexts[i].hdma_adc.Instance == DMA1_Channel2) {
                HAL_DMA_IRQHandler(&motor_contexts[i].hdma_adc);
            }
        }
    }
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    for (int i = 0; i < MAX_MOTORS; i++) {
        MotorContext* context = &motor_contexts[i];
        if (hadc->Instance == context->hadc.Instance) {
            if (context->bemf_callback != NULL) {
                // For simplicity, we'll just send the last value
                int last_a = context->bemf_buffer[BEMF_RING_BUFFER_SIZE * 2 - 2];
                int last_b = context->bemf_buffer[BEMF_RING_BUFFER_SIZE * 2 - 1];
                context->bemf_callback(abs(last_a - last_b));
            }
        }
    }
}


#endif // ARDUINO_ARCH_STM32
