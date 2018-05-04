/*
    ChibiOS - Copyright (C) 2006..2016 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/*
 * The Clear BSD License
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 *  that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
    Copyright (C) 2018 Michael Walker <walkerstop@gmail.com)
*/

/**
 * @file    hal_dac_lld.c
 * @brief   PLATFORM DAC subsystem low level driver source.
 *
 * @addtogroup DAC
 * @{
 */

#include "hal.h"

#if (HAL_USE_DAC == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/
/*!
 * @brief Get instance number for DAC module.
 *
 * @param base DAC peripheral base address
 */

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/
/** @brief DAC0 driver identifier.*/
#if (KINETIS_DAC_USE_DAC0 == TRUE) || defined(__DOXYGEN__)
const DACConfig *lld_config0;
DACDriver DACD0;
#endif

/** @brief DAC1 driver identifier.*/
#if (KINETIS_DAC_USE_DAC1 == TRUE) || defined(__DOXYGEN__)
const DACConfig *lld_config1;
DACDriver DACD1;
#endif

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/
static uint32_t DAC_GetInstance(DAC_TypeDef *base);
/*! @brief Pointers to DAC bases for each instance. */
static DAC_TypeDef *const s_dacBases[] = DAC_BASE_PTRS;
#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
/*! @brief Pointers to DAC clocks for each instance. */
static const clock_ip_name_t s_dacClocks[] = DAC_CLOCKS;
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */
#if KINETIS_DAC_USE_DAC0
dac_config_t config0;
#endif
#if KINETIS_DAC_USE_DAC1
dac_config_t config1;
#endif
#if KINETIS_DAC_USE_DAC0
volatile uint32_t g_PdbDelayInterruptCounter0;
volatile uint32_t g_PdbDelayInterruptFlag0;
#if defined(FSL_FEATURE_DAC_HAS_WATERMARK_SELECTION) && FSL_FEATURE_DAC_HAS_WATERMARK_SELECTION
volatile bool g_DacBufferWatermarkInterruptFlag0;
#endif /* FSL_FEATURE_DAC_HAS_WATERMARK_SELECTION */
volatile bool g_DacBufferReadPointerTopPositionInterruptFlag0;
volatile bool g_DacBufferReadPointerBottomPositionInterruptFlag0;
volatile uint32_t g_PdbDelayInterruptCounter0;
#endif
#if KINETIS_DAC_USE_DAC1
volatile uint32_t g_PdbDelayInterruptFlag1;
#if defined(FSL_FEATURE_DAC_HAS_WATERMARK_SELECTION) && FSL_FEATURE_DAC_HAS_WATERMARK_SELECTION
volatile bool g_DacBufferWatermarkInterruptFlag1;
#endif /* FSL_FEATURE_DAC_HAS_WATERMARK_SELECTION */
volatile bool g_DacBufferReadPointerTopPositionInterruptFlag1;
volatile bool g_DacBufferReadPointerBottomPositionInterruptFlag1;
#endif

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/
#if KINETIS_DAC_USE_DAC0 == TRUE
/*!
 * @brief ISR for DAC interrupt function
 */
OSAL_IRQ_HANDLER(KINETIS_DAC0_IRQ_VECTOR)
{
  uint32_t flags = DAC_GetBufferStatusFlags(DAC0);

  OSAL_IRQ_PROLOGUE();
#if defined(FSL_FEATURE_DAC_HAS_WATERMARK_SELECTION) && FSL_FEATURE_DAC_HAS_WATERMARK_SELECTION
  if (kDAC_BufferWatermarkFlag == (kDAC_BufferWatermarkFlag & flags))
  {
      g_DacBufferWatermarkInterruptFlag0 = true;
  }
#endif /* FSL_FEATURE_DAC_HAS_WATERMARK_SELECTION */
  if (kDAC_BufferReadPointerTopPositionFlag == (kDAC_BufferReadPointerTopPositionFlag & flags))
  {
      g_DacBufferReadPointerTopPositionInterruptFlag0 = true;
  }
  if (kDAC_BufferReadPointerBottomPositionFlag == (kDAC_BufferReadPointerBottomPositionFlag & flags))
  {
      g_DacBufferReadPointerBottomPositionInterruptFlag0 = true;
  }
  /* Clear flags. */
  DAC_ClearBufferStatusFlags(DAC0, flags);
  /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
    exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
  __DSB();
#endif
  OSAL_IRQ_EPILOGUE();
}
#endif //KINETIS_DAC_USE_DAC0 == TRUE

#if KINETIS_DAC_USE_DAC1 == TRUE
/*!
 * @brief ISR for DAC interrupt function
 */
OSAL_IRQ_HANDLER(KINETIS_DAC1_IRQ_VECTOR)
{
  uint32_t flags = DAC_GetBufferStatusFlags(DAC1);

  OSAL_IRQ_PROLOGUE();
#if defined(FSL_FEATURE_DAC_HAS_WATERMARK_SELECTION) && FSL_FEATURE_DAC_HAS_WATERMARK_SELECTION
  if (kDAC_BufferWatermarkFlag == (kDAC_BufferWatermarkFlag & flags))
  {
      g_DacBufferWatermarkInterruptFlag1 = true;
  }
#endif /* FSL_FEATURE_DAC_HAS_WATERMARK_SELECTION */
  if (kDAC_BufferReadPointerTopPositionFlag == (kDAC_BufferReadPointerTopPositionFlag & flags))
  {
      g_DacBufferReadPointerTopPositionInterruptFlag1 = true;
  }
  if (kDAC_BufferReadPointerBottomPositionFlag == (kDAC_BufferReadPointerBottomPositionFlag & flags))
  {
      g_DacBufferReadPointerBottomPositionInterruptFlag1 = true;
  }
  /* Clear flags. */
  DAC_ClearBufferStatusFlags(DAC1, flags);
  /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
    exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
  __DSB();
#endif
  OSAL_IRQ_EPILOGUE();
}
#endif //KINETIS_DAC_USE_DAC0 == TRUE


/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

static uint32_t DAC_GetInstance(DAC_TypeDef *base)
{
    uint32_t instance;

    /* Find the instance index from base address mappings. */
    for (instance = 0; instance < ARRAY_SIZE(s_dacBases); instance++)
    {
        if (s_dacBases[instance] == base)
        {
            break;
        }
    }

    assert(instance < ARRAY_SIZE(s_dacBases));

    return instance;
}

void DAC_Init(DAC_TypeDef *base, const dac_config_t *config)
{
    assert(NULL != config);

    uint8_t tmp8;

#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    /* Enable the clock. */
    CLOCK_EnableClock(s_dacClocks[DAC_GetInstance(base)]);
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */

    /* Configure. */
    /* DACx_C0. */
    tmp8 = base->C0 & ~(DAC_C0_DACRFS_MASK | DAC_C0_LPEN_MASK);
    if (kDAC_ReferenceVoltageSourceVref2 == config->referenceVoltageSource)
    {
        tmp8 |= DAC_C0_DACRFS_MASK;
    }
    if (config->enableLowPowerMode)
    {
        tmp8 |= DAC_C0_LPEN_MASK;
    }
    base->C0 = tmp8;

    /* DAC_Enable(base, true); */
    /* Tip: The DAC output can be enabled till then after user sets their own available data in application. */
}

void DAC_Deinit(DAC_TypeDef *base)
{
    DAC_Enable(base, false);

#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    /* Disable the clock. */
    CLOCK_DisableClock(s_dacClocks[DAC_GetInstance(base)]);
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */
}

void DAC_GetDefaultConfig(dac_config_t *config)
{
    assert(NULL != config);

    config->referenceVoltageSource = kDAC_ReferenceVoltageSourceVref2;
    config->enableLowPowerMode = false;
}

void DAC_SetBufferConfig(DAC_TypeDef *base, const dac_buffer_config_t *config)
{
    assert(NULL != config);

    uint8_t tmp8;

    /* DACx_C0. */
    tmp8 = base->C0 & ~(DAC_C0_DACTRGSEL_MASK);
    if (kDAC_BufferTriggerBySoftwareMode == config->triggerMode)
    {
        tmp8 |= DAC_C0_DACTRGSEL_MASK;
    }
    base->C0 = tmp8;

    /* DACx_C1. */
    tmp8 = base->C1 &
           ~(
#if defined(FSL_FEATURE_DAC_HAS_WATERMARK_SELECTION) && FSL_FEATURE_DAC_HAS_WATERMARK_SELECTION
               DAC_C1_DACBFWM_MASK |
#endif /* FSL_FEATURE_DAC_HAS_WATERMARK_SELECTION */
               DAC_C1_DACBFMD_MASK);
#if defined(FSL_FEATURE_DAC_HAS_WATERMARK_SELECTION) && FSL_FEATURE_DAC_HAS_WATERMARK_SELECTION
    tmp8 |= DAC_C1_DACBFWM(config->watermark);
#endif /* FSL_FEATURE_DAC_HAS_WATERMARK_SELECTION */
    tmp8 |= DAC_C1_DACBFMD(config->workMode);
    base->C1 = tmp8;

    /* DACx_C2. */
    tmp8 = base->C2 & ~DAC_C2_DACBFUP_MASK;
    tmp8 |= DAC_C2_DACBFUP(config->upperLimit);
    base->C2 = tmp8;
}

void DAC_GetDefaultBufferConfig(dac_buffer_config_t *config)
{
    assert(NULL != config);

    config->triggerMode = kDAC_BufferTriggerBySoftwareMode;
#if defined(FSL_FEATURE_DAC_HAS_WATERMARK_SELECTION) && FSL_FEATURE_DAC_HAS_WATERMARK_SELECTION
    config->watermark = kDAC_BufferWatermark1Word;
#endif /* FSL_FEATURE_DAC_HAS_WATERMARK_SELECTION */
    config->workMode = kDAC_BufferWorkAsNormalMode;
    config->upperLimit = DAC_DATL_COUNT - 1U;
}

void DAC_SetBufferValue(DAC_TypeDef *base, uint8_t index, uint16_t value)
{
    assert(index < DAC_DATL_COUNT);

    base->DAT[index].DATL = (uint8_t)(0xFFU & value);         /* Low 8-bit. */
    base->DAT[index].DATH = (uint8_t)((0xF00U & value) >> 8); /* High 4-bit. */
}

void DAC_SetBufferReadPointer(DAC_TypeDef *base, uint8_t index)
{
    assert(index < DAC_DATL_COUNT);

    uint8_t tmp8 = base->C2 & ~DAC_C2_DACBFRP_MASK;

    tmp8 |= DAC_C2_DACBFRP(index);
    base->C2 = tmp8;
}

void DAC_EnableBufferInterrupts(DAC_TypeDef *base, uint32_t mask)
{
    mask &= (
#if defined(FSL_FEATURE_DAC_HAS_WATERMARK_DETECTION) && FSL_FEATURE_DAC_HAS_WATERMARK_DETECTION
        DAC_C0_DACBWIEN_MASK |
#endif /* FSL_FEATURE_DAC_HAS_WATERMARK_DETECTION */
        DAC_C0_DACBTIEN_MASK | DAC_C0_DACBBIEN_MASK);
    base->C0 |= ((uint8_t)mask); /* Write 1 to enable. */
}

void DAC_DisableBufferInterrupts(DAC_TypeDef *base, uint32_t mask)
{
    mask &= (
#if defined(FSL_FEATURE_DAC_HAS_WATERMARK_DETECTION) && FSL_FEATURE_DAC_HAS_WATERMARK_DETECTION
        DAC_C0_DACBWIEN_MASK |
#endif /* FSL_FEATURE_DAC_HAS_WATERMARK_DETECTION */
        DAC_C0_DACBTIEN_MASK | DAC_C0_DACBBIEN_MASK);
    base->C0 &= (uint8_t)(~((uint8_t)mask)); /* Write 0 to disable. */
}

uint32_t DAC_GetBufferStatusFlags(DAC_TypeDef *base)
{
    return (uint32_t)(base->SR & (
#if defined(FSL_FEATURE_DAC_HAS_WATERMARK_DETECTION) && FSL_FEATURE_DAC_HAS_WATERMARK_DETECTION
                                     DAC_SR_DACBFWMF_MASK |
#endif /* FSL_FEATURE_DAC_HAS_WATERMARK_DETECTION */
                                     DAC_SR_DACBFRPTF_MASK | DAC_SR_DACBFRPBF_MASK));
}

void DAC_ClearBufferStatusFlags(DAC_TypeDef *base, uint32_t mask)
{
    mask &= (
#if defined(FSL_FEATURE_DAC_HAS_WATERMARK_DETECTION) && FSL_FEATURE_DAC_HAS_WATERMARK_DETECTION
        DAC_SR_DACBFWMF_MASK |
#endif /* FSL_FEATURE_DAC_HAS_WATERMARK_DETECTION */
        DAC_SR_DACBFRPTF_MASK | DAC_SR_DACBFRPBF_MASK);
    base->SR &= (uint8_t)(~((uint8_t)mask)); /* Write 0 to clear flags. */
}

/**
 * @brief   Low level DAC driver initialization.
 *
 * @notapi
 */

void _dac_lld_init(DACDriver *dacp) {

#if KINETIS_DAC_USE_DAC0 == TRUE
  if(&DACD0 == dacp){
    dac_buffer_config_t dacBufferConfigStruct0;
    DAC_GetDefaultConfig(&config0);
    dacObjectInit(&DACD0);
    DACD0.config = (const DACConfig *)&lld_config0;
    DAC_Init(DAC0, &config0);
    DAC_EnableBufferDMA(DAC0, false);
    DAC_Enable(DAC0, true);

    /* Configure the DAC buffer. */
    DAC_GetDefaultBufferConfig(&dacBufferConfigStruct0);
    dacBufferConfigStruct0.triggerMode = kDAC_BufferTriggerByHardwareMode;
    DAC_SetBufferConfig(DAC0, &dacBufferConfigStruct0);
    DAC_SetBufferReadPointer(DAC0, 0U); /* Make sure the read pointer to the start. */
  }
#endif

#if KINETIS_DAC_USE_DAC1 == TRUE
  if(&DACD1 == dacp){
    dac_buffer_config_t dacBufferConfigStruct1;
    DAC_GetDefaultConfig(&config1);
    dacObjectInit(&DACD1);
    DACD1.config = (const DACConfig *)&lld_config1;
    DAC_Init(DAC1, &config1);
    DAC_EnableBufferDMA(DAC0, false);
    DAC_Enable(DAC0, true);

    /* Configure the DAC buffer. */
    DAC_GetDefaultBufferConfig(&dacBufferConfigStruct1);
    dacBufferConfigStruct1.triggerMode = kDAC_BufferTriggerByHardwareMode;
    DAC_SetBufferConfig(DAC1, &dacBufferConfigStruct1);
    DAC_SetBufferReadPointer(DAC1, 0U); /* Make sure the read pointer to the start. */
  }
#endif

}

void dac_lld_init(void) {
#if KINETIS_DAC_USE_DAC0 == TRUE
  _dac_lld_init(&DACD0);
#endif
#if KINETIS_DAC_USE_DAC1 == TRUE
  _dac_lld_init(&DACD1);
#endif
}

/**
 * @brief   Configures and activates the DAC peripheral.
 *
 * @param[in] dacp      pointer to the @p DACDriver object
 *
 * @notapi
 */
void dac_lld_start(DACDriver *dacp) {

  /* If the driver is in DAC_STOP state then a full initialization is
     required.*/

  if (dacp->state == DAC_STOP) {
    _dac_lld_init(dacp);
  }
}

/**
 * @brief   Deactivates the DAC peripheral.
 *
 * @param[in] dacp      pointer to the @p DACDriver object
 *
 * @notapi
 */
void dac_lld_stop(DACDriver *dacp) {

  /* If in ready state then disables the DAC clock.*/
  if (dacp->state == DAC_READY) {

#if KINETIS_DAC_USE_DAC0 == TRUE
    if (&DACD0 == dacp) {
      DAC_Enable(DAC0, false);
    }
#endif

#if KINETIS_DAC_USE_DAC1 == TRUE
    if (&DACD1 == dacp) {
      DAC_Enable(DAC1, false);
    }
#endif

  }
}

/**
 * @brief   Outputs a value directly on a DAC channel.
 *
 * @param[in] dacp      pointer to the @p DACDriver object
 * @param[in] channel   DAC channel number
 * @param[in] sample    value to be output
 *
 * @api
 */
void dac_lld_put_channel(DACDriver *dacp,
                         dacchannel_t channel,
                         dacsample_t sample) {

#if KINETIS_DAC_USE_DAC0 == TRUE
  if(&DACD0 == dacp){
    DAC_SetBufferValue(DAC0, channel, sample);
  }
#endif

#if KINETIS_DAC_USE_DAC1 == TRUE
  if(&DACD1 == dacp){
    DAC_SetBufferValue(DAC1, channel, sample);
  }
#endif

  (void)dacp;
  (void)channel;
  (void)sample;
}

/**
 * @brief   Starts a DAC conversion.
 * @details Starts an asynchronous conversion operation.
 * @note    In @p DAC_DHRM_8BIT_RIGHT mode the parameters passed to the
 *          callback are wrong because two samples are packed in a single
 *          dacsample_t element. This will not be corrected, do not rely
 *          on those parameters.
 * @note    In @p DAC_DHRM_8BIT_RIGHT_DUAL mode two samples are treated
 *          as a single 16 bits sample and packed into a single dacsample_t
 *          element. The num_channels must be set to one in the group
 *          conversion configuration structure.
 *
 * @param[in] dacp      pointer to the @p DACDriver object
 *
 * @notapi
 */
void dac_lld_start_conversion(DACDriver *dacp) {
  uint32_t mask;
  (void)mask;

#if KINETIS_DAC_USE_DAC0 == TRUE
  if(&DACD0 == dacp){

/* Clear flags. */
#if defined(FSL_FEATURE_DAC_HAS_WATERMARK_SELECTION) && FSL_FEATURE_DAC_HAS_WATERMARK_SELECTION
    g_DacBufferWatermarkInterruptFlag0 = false;
#endif /* FSL_FEATURE_DAC_HAS_WATERMARK_SELECTION */
    g_DacBufferReadPointerTopPositionInterruptFlag0 = false;
    g_DacBufferReadPointerBottomPositionInterruptFlag0 = false;

    /* Enable interrupts. */
    mask = 0U;
#if defined(FSL_FEATURE_DAC_HAS_WATERMARK_SELECTION) && FSL_FEATURE_DAC_HAS_WATERMARK_SELECTION
    mask |= kDAC_BufferWatermarkInterruptEnable;
#endif /* FSL_FEATURE_DAC_HAS_WATERMARK_SELECTION */
    mask |= kDAC_BufferReadPointerTopInterruptEnable | kDAC_BufferReadPointerBottomInterruptEnable;
    DAC_EnableBuffer(DAC0, true);
    DAC_EnableBufferInterrupts(DAC0, mask);
  }
#endif

#if KINETIS_DAC_USE_DAC1 == TRUE
  if(&DACD1 == dacp){

/* Clear flags. */
#if defined(FSL_FEATURE_DAC_HAS_WATERMARK_SELECTION) && FSL_FEATURE_DAC_HAS_WATERMARK_SELECTION
    g_DacBufferWatermarkInterruptFlag1 = false;
#endif /* FSL_FEATURE_DAC_HAS_WATERMARK_SELECTION */
    g_DacBufferReadPointerTopPositionInterruptFlag1 = false;
    g_DacBufferReadPointerBottomPositionInterruptFlag1 = false;

    /* Enable interrupts. */
    mask = 0U;
#if defined(FSL_FEATURE_DAC_HAS_WATERMARK_SELECTION) && FSL_FEATURE_DAC_HAS_WATERMARK_SELECTION
    mask |= kDAC_BufferWatermarkInterruptEnable;
#endif /* FSL_FEATURE_DAC_HAS_WATERMARK_SELECTION */
    mask |= kDAC_BufferReadPointerTopInterruptEnable | kDAC_BufferReadPointerBottomInterruptEnable;
    DAC_EnableBuffer(DAC1, true);
    DAC_EnableBufferInterrupts(DAC1, mask);
  }
#endif

  (void)dacp;
}

/**
 * @brief   Stops an ongoing conversion.
 * @details This function stops the currently ongoing conversion and returns
 *          the driver in the @p DAC_READY state. If there was no conversion
 *          being processed then the function does nothing.
 *
 * @param[in] dacp      pointer to the @p DACDriver object
 *
 * @iclass
 */
void dac_lld_stop_conversion(DACDriver *dacp) {

  (void)dacp;
}

#endif /* HAL_USE_DAC == TRUE */

/** @} */
