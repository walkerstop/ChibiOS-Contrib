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
 * @file    hal_dac_lld.h
 * @brief   MK66F18 DAC subsystem low level driver header.
 *
 * @addtogroup DAC
 * @{
 */

#ifndef HAL_DAC_LLD_H
#define HAL_DAC_LLD_H

#if HAL_USE_DAC || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/
#define CLK_GATE_REG_OFFSET_SHIFT 16U
#define CLK_GATE_REG_OFFSET_MASK 0xFFFF0000U
#define CLK_GATE_BIT_SHIFT_SHIFT 0U
#define CLK_GATE_BIT_SHIFT_MASK 0x0000FFFFU

#define assert osalDbgCheck

/**
 * @brief   Maximum number of DAC channels per unit.
 */
#define DAC_MAX_CHANNELS                    1

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */

#if defined(MK66F18)
#define FSL_FEATURE_DAC_HAS_WATERMARK_DETECTION          TRUE
#define FSL_FEATURE_DAC_HAS_WATERMARK_SELECTION          TRUE
#define FSL_FEATURE_DAC_HAS_WATERMARK_1_WORD             TRUE
#define FSL_FEATURE_DAC_HAS_WATERMARK_2_WORD             TRUE
#define FSL_FEATURE_DAC_HAS_WATERMARK_3_WORD             TRUE
#define FSL_FEATURE_DAC_HAS_WATERMARK_4_WORD             TRUE
#define FSL_FEATURE_DAC_HAS_BUFFER_SWING_MODE            TRUE
#define FSL_FEATURE_DAC_HAS_BUFFER_FIFO_MODE             TRUE
#define FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL             FALSE
#endif

/**
 * @brief   DAC0 CH1 driver enable switch.
 * @details If set to @p TRUE the support for DAC0 channel 1 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(KINETIS_DAC_USE_DAC0) || defined(__DOXYGEN__)
#define KINETIS_DAC_USE_DAC0               FALSE
#endif

/**
 * @brief   DAC1 CH1 driver enable switch.
 * @details If set to @p TRUE the support for DAC1 channel 1 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(KINETIS_DAC_USE_DAC1) || defined(__DOXYGEN__)
#define KINETIS_DAC_USE_DAC1               FALSE
#endif

/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/
#if !KINETIS_DAC_USE_DAC0 && !KINETIS_DAC_USE_DAC1
#error "HAL_USE_DAC defined but KINETIS_DAC_USE_DAC0 and KINETIS_DAC_USE_DAC1 false!"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Type of a DAC channel index.
 */
typedef uint32_t dacchannel_t;

/**
 * @brief   Type of a structure representing an DAC driver.
 */
typedef struct DACDriver DACDriver;

/**
 * @brief   Type representing a DAC sample.
 */
typedef uint16_t dacsample_t;

/**
 * @brief   Possible DAC failure causes.
 * @note    Error codes are architecture dependent and should not relied
 *          upon.
 */
typedef enum {
  DAC_ERR_DMAFAILURE = 0,                   /**< DMA operations failure.    */
  DAC_ERR_UNDERFLOW = 1                     /**< DAC overflow condition.    */
} dacerror_t;

/**
 * @brief   DAC notification callback type.
 *
 * @param[in] dacp      pointer to the @p DACDriver object triggering the
 * @param[in] buffer    pointer to the next semi-buffer to be filled
 * @param[in] n         number of buffer rows available starting from @p buffer
 *                      callback
 */
typedef void (*daccallback_t)(DACDriver *dacp, dacsample_t *buffer, size_t n);

/**
 * @brief   ADC error callback type.
 *
 * @param[in] dacp      pointer to the @p DACDriver object triggering the
 *                      callback
 * @param[in] err       ADC error code
 */
typedef void (*dacerrorcallback_t)(DACDriver *dacp, dacerror_t err);

/**
 * @brief   DAC Conversion group structure.
 */
typedef struct {
  /**
   * @brief   Number of DAC channels.
   */
  uint32_t                  num_channels;
  /**
   * @brief   Operation complete callback or @p NULL.
   */
  daccallback_t             end_cb;
  /**
   * @brief   Error handling callback or @p NULL.
   */
  dacerrorcallback_t        error_cb;
  /* End of the mandatory fields.*/
  /**
   * @brief   DAC initialization data.
   * @note    This field contains the (not shifted) value to be put into the
   *          TSEL field of the DAC CR register during initialization. All
   *          other fields are handled internally.
   */
  uint32_t                  trigger;
} DACConversionGroup;

/**
 * @brief   Samples alignment and size mode.
 */
typedef enum {
  DAC_DHRM_12BIT_RIGHT = 0,
  DAC_DHRM_12BIT_LEFT = 1,
  DAC_DHRM_8BIT_RIGHT = 2,
  DAC_DHRM_12BIT_RIGHT_DUAL = 3,
  DAC_DHRM_12BIT_LEFT_DUAL = 4,
  DAC_DHRM_8BIT_RIGHT_DUAL = 5
} dacdhrmode_t;

/**
 * @brief   Driver configuration structure.
 */
typedef struct {
  /* End of the mandatory fields.*/
  /**
   * @brief   Initial output on DAC channels.
   */
  dacsample_t               init;
  /**
   * @brief   DAC data holding register mode.
   */
  dacdhrmode_t              datamode;
  /**
   * @brief   DAC control register.
   */
  uint16_t                  cr;
} DACConfig;

/*!
 * @brief DAC reference voltage source.
 */
typedef enum _dac_reference_voltage_source
{
    kDAC_ReferenceVoltageSourceVref1 = 0U, /*!< The DAC selects DACREF_1 as the reference voltage. */
    kDAC_ReferenceVoltageSourceVref2 = 1U, /*!< The DAC selects DACREF_2 as the reference voltage. */
} dac_reference_voltage_source_t;

/*!
 * @brief DAC module configuration.
*/
typedef struct _dac_config
{
    dac_reference_voltage_source_t referenceVoltageSource; /*!< Select the DAC reference voltage source. */
    bool enableLowPowerMode;                               /*!< Enable the low-power mode. */
} dac_config_t;

/**
 * @brief   Structure representing a DAC driver.
 */
struct DACDriver {
  /**
   * @brief   Driver state.
   */
  dacstate_t                state;
  /**
   * @brief   Conversion group.
   */
  const DACConversionGroup  *grpp;
  /**
   * @brief   Samples buffer pointer.
   */
  dacsample_t               *samples;
  /**
   * @brief   Samples buffer size.
   */
  uint16_t                  depth;
  /**
   * @brief   Current configuration data.
   */
  const DACConfig         *config;
#if DAC_USE_WAIT || defined(__DOXYGEN__)
  /**
   * @brief   Waiting thread.
   */
  thread_reference_t        thread;
#endif /* DAC_USE_WAIT */
#if DAC_USE_MUTUAL_EXCLUSION || defined(__DOXYGEN__)
  /**
   * @brief   Mutex protecting the bus.
   */
  mutex_t                   mutex;
#endif /* DAC_USE_MUTUAL_EXCLUSION */
#if defined(DAC_DRIVER_EXT_FIELDS)
  DAC_DRIVER_EXT_FIELDS
#endif
  /* End of the mandatory fields.*/
};

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/
/*! @name Driver version */
/*@{*/
/*! @brief DAC driver version 2.0.1. */
#define FSL_DAC_DRIVER_VERSION (MAKE_VERSION(2, 0, 1))
/*@}*/

#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

#define CLK_GATE_DEFINE(reg_offset, bit_shift)                                  \
    ((((reg_offset) << CLK_GATE_REG_OFFSET_SHIFT) & CLK_GATE_REG_OFFSET_MASK) | \
     (((bit_shift) << CLK_GATE_BIT_SHIFT_SHIFT) & CLK_GATE_BIT_SHIFT_MASK))


/*! @brief Clock ip name array for DAC. */
#define DAC_CLOCKS               \
    {                            \
        CLK_GATE_DEFINE(0x102CU, 12U), CLK_GATE_DEFINE(0x102CU, 13U) \
    }

#define CLK_GATE_ABSTRACT_REG_OFFSET(x) (((x)&CLK_GATE_REG_OFFSET_MASK) >> CLK_GATE_REG_OFFSET_SHIFT)
#define CLK_GATE_ABSTRACT_BITS_SHIFT(x) (((x)&CLK_GATE_BIT_SHIFT_MASK) >> CLK_GATE_BIT_SHIFT_SHIFT)
/*! @brief Clock gate name used for CLOCK_EnableClock/CLOCK_DisableClock. */

typedef enum _clock_ip_name
{
    kCLOCK_IpInvalid = 0U,
    kCLOCK_I2c2 = CLK_GATE_DEFINE(0x1028U, 6U),
    kCLOCK_I2c3 = CLK_GATE_DEFINE(0x1028U, 7U),
    kCLOCK_Uart4 = CLK_GATE_DEFINE(0x1028U, 10U),

    kCLOCK_Enet0 = CLK_GATE_DEFINE(0x102CU, 0U),
    kCLOCK_Lpuart0 = CLK_GATE_DEFINE(0x102CU, 4U),
    kCLOCK_Tpm1 = CLK_GATE_DEFINE(0x102CU, 9U),
    kCLOCK_Tpm2 = CLK_GATE_DEFINE(0x102CU, 10U),
    kCLOCK_Dac0 = CLK_GATE_DEFINE(0x102CU, 12U),
    kCLOCK_Dac1 = CLK_GATE_DEFINE(0x102CU, 13U),

    kCLOCK_Rnga0 = CLK_GATE_DEFINE(0x1030U, 0U),
    kCLOCK_Usbhs0 = CLK_GATE_DEFINE(0x1030U, 1U),
    kCLOCK_UsbhsPhy0 = CLK_GATE_DEFINE(0x1030U, 2U),
    kCLOCK_UsbhsDcd0 = CLK_GATE_DEFINE(0x1030U, 3U),
    kCLOCK_Flexcan1 = CLK_GATE_DEFINE(0x1030U, 4U),
    kCLOCK_Spi2 = CLK_GATE_DEFINE(0x1030U, 12U),
    kCLOCK_Sdhc0 = CLK_GATE_DEFINE(0x1030U, 17U),
    kCLOCK_Ftm3 = CLK_GATE_DEFINE(0x1030U, 25U),
    kCLOCK_Adc1 = CLK_GATE_DEFINE(0x1030U, 27U),

    kCLOCK_Ewm0 = CLK_GATE_DEFINE(0x1034U, 1U),
    kCLOCK_Cmt0 = CLK_GATE_DEFINE(0x1034U, 2U),
    kCLOCK_I2c0 = CLK_GATE_DEFINE(0x1034U, 6U),
    kCLOCK_I2c1 = CLK_GATE_DEFINE(0x1034U, 7U),
    kCLOCK_Uart0 = CLK_GATE_DEFINE(0x1034U, 10U),
    kCLOCK_Uart1 = CLK_GATE_DEFINE(0x1034U, 11U),
    kCLOCK_Uart2 = CLK_GATE_DEFINE(0x1034U, 12U),
    kCLOCK_Uart3 = CLK_GATE_DEFINE(0x1034U, 13U),
    kCLOCK_Usbfs0 = CLK_GATE_DEFINE(0x1034U, 18U),
    kCLOCK_Cmp0 = CLK_GATE_DEFINE(0x1034U, 19U),
    kCLOCK_Cmp1 = CLK_GATE_DEFINE(0x1034U, 19U),
    kCLOCK_Cmp2 = CLK_GATE_DEFINE(0x1034U, 19U),
    kCLOCK_Cmp3 = CLK_GATE_DEFINE(0x1034U, 19U),
    kCLOCK_Vref0 = CLK_GATE_DEFINE(0x1034U, 20U),

    kCLOCK_Lptmr0 = CLK_GATE_DEFINE(0x1038U, 0U),
    kCLOCK_Tsi0 = CLK_GATE_DEFINE(0x1038U, 5U),
    kCLOCK_PortA = CLK_GATE_DEFINE(0x1038U, 9U),
    kCLOCK_PortB = CLK_GATE_DEFINE(0x1038U, 10U),
    kCLOCK_PortC = CLK_GATE_DEFINE(0x1038U, 11U),
    kCLOCK_PortD = CLK_GATE_DEFINE(0x1038U, 12U),
    kCLOCK_PortE = CLK_GATE_DEFINE(0x1038U, 13U),

    kCLOCK_Ftf0 = CLK_GATE_DEFINE(0x103CU, 0U),
    kCLOCK_Dmamux0 = CLK_GATE_DEFINE(0x103CU, 1U),
    kCLOCK_Flexcan0 = CLK_GATE_DEFINE(0x103CU, 4U),
    kCLOCK_Spi0 = CLK_GATE_DEFINE(0x103CU, 12U),
    kCLOCK_Spi1 = CLK_GATE_DEFINE(0x103CU, 13U),
    kCLOCK_Sai0 = CLK_GATE_DEFINE(0x103CU, 15U),
    kCLOCK_Crc0 = CLK_GATE_DEFINE(0x103CU, 18U),
    kCLOCK_Usbdcd0 = CLK_GATE_DEFINE(0x103CU, 21U),
    kCLOCK_Pdb0 = CLK_GATE_DEFINE(0x103CU, 22U),
    kCLOCK_Pit0 = CLK_GATE_DEFINE(0x103CU, 23U),
    kCLOCK_Ftm0 = CLK_GATE_DEFINE(0x103CU, 24U),
    kCLOCK_Ftm1 = CLK_GATE_DEFINE(0x103CU, 25U),
    kCLOCK_Ftm2 = CLK_GATE_DEFINE(0x103CU, 26U),
    kCLOCK_Adc0 = CLK_GATE_DEFINE(0x103CU, 27U),
    kCLOCK_Rtc0 = CLK_GATE_DEFINE(0x103CU, 29U),

    kCLOCK_Flexbus0 = CLK_GATE_DEFINE(0x1040U, 0U),
    kCLOCK_Dma0 = CLK_GATE_DEFINE(0x1040U, 1U),
    kCLOCK_Sysmpu0 = CLK_GATE_DEFINE(0x1040U, 2U),
    kCLOCK_Sdramc0 = CLK_GATE_DEFINE(0x1040U, 3U),
} clock_ip_name_t;

/*!
 * @brief Enable the clock for specific IP.
 *
 * @param name  Which clock to enable, see \ref clock_ip_name_t.
 */
static inline void CLOCK_EnableClock(clock_ip_name_t name)
{
    uint32_t regAddr = SIM_BASE + CLK_GATE_ABSTRACT_REG_OFFSET((uint32_t)name);
    (*(volatile uint32_t *)regAddr) |= (1U << CLK_GATE_ABSTRACT_BITS_SHIFT((uint32_t)name));
}

/*!
 * @brief Disable the clock for specific IP.
 *
 * @param name  Which clock to disable, see \ref clock_ip_name_t.
 */
static inline void CLOCK_DisableClock(clock_ip_name_t name)
{
    uint32_t regAddr = SIM_BASE + CLK_GATE_ABSTRACT_REG_OFFSET((uint32_t)name);
    (*(volatile uint32_t *)regAddr) &= ~(1U << CLK_GATE_ABSTRACT_BITS_SHIFT((uint32_t)name));
}

/*!
 * @brief DAC buffer flags.
 */
enum _dac_buffer_status_flags
{
#if defined(FSL_FEATURE_DAC_HAS_WATERMARK_DETECTION) && FSL_FEATURE_DAC_HAS_WATERMARK_DETECTION
    kDAC_BufferWatermarkFlag = DAC_SR_DACBFWMF_MASK,                  /*!< DAC Buffer Watermark Flag. */
#endif                                                                /* FSL_FEATURE_DAC_HAS_WATERMARK_DETECTION */
    kDAC_BufferReadPointerTopPositionFlag = DAC_SR_DACBFRPTF_MASK,    /*!< DAC Buffer Read Pointer Top Position Flag. */
    kDAC_BufferReadPointerBottomPositionFlag = DAC_SR_DACBFRPBF_MASK, /*!< DAC Buffer Read Pointer Bottom Position
                                                                           Flag. */
};

/*!
 * @brief DAC buffer interrupts.
 */
enum _dac_buffer_interrupt_enable
{
#if defined(FSL_FEATURE_DAC_HAS_WATERMARK_DETECTION) && FSL_FEATURE_DAC_HAS_WATERMARK_DETECTION
    kDAC_BufferWatermarkInterruptEnable = DAC_C0_DACBWIEN_MASK,         /*!< DAC Buffer Watermark Interrupt Enable. */
#endif                                                                  /* FSL_FEATURE_DAC_HAS_WATERMARK_DETECTION */
    kDAC_BufferReadPointerTopInterruptEnable = DAC_C0_DACBTIEN_MASK,    /*!< DAC Buffer Read Pointer Top Flag Interrupt
                                                                             Enable. */
    kDAC_BufferReadPointerBottomInterruptEnable = DAC_C0_DACBBIEN_MASK, /*!< DAC Buffer Read Pointer Bottom Flag
                                                                             Interrupt Enable */
};

/*!
 * @brief DAC buffer trigger mode.
 */
typedef enum _dac_buffer_trigger_mode
{
    kDAC_BufferTriggerByHardwareMode = 0U, /*!< The DAC hardware trigger is selected. */
    kDAC_BufferTriggerBySoftwareMode = 1U, /*!< The DAC software trigger is selected. */
} dac_buffer_trigger_mode_t;

#if defined(FSL_FEATURE_DAC_HAS_WATERMARK_SELECTION) && FSL_FEATURE_DAC_HAS_WATERMARK_SELECTION
/*!
 * @brief DAC buffer watermark.
 */
typedef enum _dac_buffer_watermark
{
#if defined(FSL_FEATURE_DAC_HAS_WATERMARK_1_WORD) && FSL_FEATURE_DAC_HAS_WATERMARK_1_WORD
    kDAC_BufferWatermark1Word = 0U, /*!< 1 word  away from the upper limit. */
#endif                              /* FSL_FEATURE_DAC_HAS_WATERMARK_1_WORD */
#if defined(FSL_FEATURE_DAC_HAS_WATERMARK_2_WORDS) && FSL_FEATURE_DAC_HAS_WATERMARK_2_WORDS
    kDAC_BufferWatermark2Word = 1U, /*!< 2 words away from the upper limit. */
#endif                              /* FSL_FEATURE_DAC_HAS_WATERMARK_2_WORDS */
#if defined(FSL_FEATURE_DAC_HAS_WATERMARK_3_WORDS) && FSL_FEATURE_DAC_HAS_WATERMARK_3_WORDS
    kDAC_BufferWatermark3Word = 2U, /*!< 3 words away from the upper limit. */
#endif                              /* FSL_FEATURE_DAC_HAS_WATERMARK_3_WORDS */
#if defined(FSL_FEATURE_DAC_HAS_WATERMARK_4_WORDS) && FSL_FEATURE_DAC_HAS_WATERMARK_4_WORDS
    kDAC_BufferWatermark4Word = 3U, /*!< 4 words away from the upper limit. */
#endif                              /* FSL_FEATURE_DAC_HAS_WATERMARK_4_WORDS */
} dac_buffer_watermark_t;
#endif /* FSL_FEATURE_DAC_HAS_WATERMARK_SELECTION */

/*!
 * @brief DAC buffer work mode.
 */
typedef enum _dac_buffer_work_mode
{
    kDAC_BufferWorkAsNormalMode = 0U, /*!< Normal mode. */
#if defined(FSL_FEATURE_DAC_HAS_BUFFER_SWING_MODE) && FSL_FEATURE_DAC_HAS_BUFFER_SWING_MODE
    kDAC_BufferWorkAsSwingMode,       /*!< Swing mode. */
#endif                                /* FSL_FEATURE_DAC_HAS_BUFFER_SWING_MODE */
    kDAC_BufferWorkAsOneTimeScanMode, /*!< One-Time Scan mode. */
#if defined(FSL_FEATURE_DAC_HAS_BUFFER_FIFO_MODE) && FSL_FEATURE_DAC_HAS_BUFFER_FIFO_MODE
    kDAC_BufferWorkAsFIFOMode, /*!< FIFO mode. */
#endif                         /* FSL_FEATURE_DAC_HAS_BUFFER_FIFO_MODE */
} dac_buffer_work_mode_t;

/*!
 * @brief DAC buffer configuration.
 */
typedef struct _dac_buffer_config
{
    dac_buffer_trigger_mode_t triggerMode; /*!< Select the buffer's trigger mode. */
#if defined(FSL_FEATURE_DAC_HAS_WATERMARK_SELECTION) && FSL_FEATURE_DAC_HAS_WATERMARK_SELECTION
    dac_buffer_watermark_t watermark; /*!< Select the buffer's watermark. */
#endif                                /* FSL_FEATURE_DAC_HAS_WATERMARK_SELECTION */
    dac_buffer_work_mode_t workMode;  /*!< Select the buffer's work mode. */
    uint8_t upperLimit;               /*!< Set the upper limit for the buffer index.
                                           Normally, 0-15 is available for a buffer with 16 items. */
} dac_buffer_config_t;

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/
#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name Initialization
 * @{
 */

/*!
 * @brief Initializes the DAC module.
 *
 * This function initializes the DAC module including the following operations.
 *  - Enabling the clock for DAC module.
 *  - Configuring the DAC converter with a user configuration.
 *  - Enabling the DAC module.
 *
 * @param base DAC peripheral base address.
 * @param config Pointer to the configuration structure. See "dac_config_t".
 */
void DAC_Init(DAC_TypeDef *base, const dac_config_t *config);

/*!
 * @brief De-initializes the DAC module.
 *
 * This function de-initializes the DAC module including the following operations.
 *  - Disabling the DAC module.
 *  - Disabling the clock for the DAC module.
 *
 * @param base DAC peripheral base address.
 */
void DAC_Deinit(DAC_TypeDef *base);

/*!
 * @brief Initializes the DAC user configuration structure.
 *
 * This function initializes the user configuration structure to a default value. The default values are as follows.
 * @code
 *   config->referenceVoltageSource = kDAC_ReferenceVoltageSourceVref2;
 *   config->enableLowPowerMode = false;
 * @endcode
 * @param config Pointer to the configuration structure. See "dac_config_t".
 */
void DAC_GetDefaultConfig(dac_config_t *config);
/*!
 * @brief Enables the DAC module.
 *
 * @param base DAC peripheral base address.
 * @param enable Enables or disables the feature.
 */
static inline void DAC_Enable(DAC_TypeDef *base, bool enable)
{
    if (enable)
    {
        base->C0 |= DAC_C0_DACEN_MASK;
    }
    else
    {
        base->C0 &= ~DAC_C0_DACEN_MASK;
    }
}

/* @} */

/*!
 * @name Buffer
 * @{
 */

/*!
 * @brief Enables the DAC buffer.
 *
 * @param base DAC peripheral base address.
 * @param enable Enables or disables the feature.
 */
static inline void DAC_EnableBuffer(DAC_TypeDef *base, bool enable)
{
    if (enable)
    {
        base->C1 |= DAC_C1_DACBFEN_MASK;
    }
    else
    {
        base->C1 &= ~DAC_C1_DACBFEN_MASK;
    }
}

/*!
 * @brief Configures the CMP buffer.
 *
 * @param base   DAC peripheral base address.
 * @param config Pointer to the configuration structure. See "dac_buffer_config_t".
 */
void DAC_SetBufferConfig(DAC_TypeDef *base, const dac_buffer_config_t *config);

/*!
 * @brief Initializes the DAC buffer configuration structure.
 *
 * This function initializes the DAC buffer configuration structure to default values. The default values are as follows.
 * @code
 *   config->triggerMode = kDAC_BufferTriggerBySoftwareMode;
 *   config->watermark   = kDAC_BufferWatermark1Word;
 *   config->workMode    = kDAC_BufferWorkAsNormalMode;
 *   config->upperLimit  = DAC_DATL_COUNT - 1U;
 * @endcode
 * @param config Pointer to the configuration structure. See "dac_buffer_config_t".
 */
void DAC_GetDefaultBufferConfig(dac_buffer_config_t *config);

/*!
 * @brief Enables the DMA for DAC buffer.
 *
 * @param base DAC peripheral base address.
 * @param enable Enables or disables the feature.
 */
static inline void DAC_EnableBufferDMA(DAC_TypeDef *base, bool enable)
{
    if (enable)
    {
        base->C1 |= DAC_C1_DMAEN_MASK;
    }
    else
    {
        base->C1 &= ~DAC_C1_DMAEN_MASK;
    }
}

/*!
 * @brief Sets the value for  items in the buffer.
 *
 * @param base  DAC peripheral base address.
 * @param index Setting the index for items in the buffer. The available index should not exceed the size of the DAC buffer.
 * @param value Setting the value for items in the buffer. 12-bits are available.
 */
void DAC_SetBufferValue(DAC_TypeDef *base, uint8_t index, uint16_t value);

/*!
 * @brief Triggers the buffer using software and updates the read pointer of the DAC buffer.
 *
 * This function triggers the function using software. The read pointer of the DAC buffer is updated with one step
 * after this function is called. Changing the read pointer depends on the buffer's work mode.
 *
 * @param base DAC peripheral base address.
 */
static inline void DAC_DoSoftwareTriggerBuffer(DAC_TypeDef *base)
{
    base->C0 |= DAC_C0_DACSWTRG_MASK;
}

/*!
 * @brief Gets the current read pointer of the DAC buffer.
 *
 * This function gets the current read pointer of the DAC buffer.
 * The current output value depends on the item indexed by the read pointer. It is updated either
 * by a software trigger or a hardware trigger.
 *
 * @param  base DAC peripheral base address.
 *
 * @return      The current read pointer of the DAC buffer.
 */
static inline uint8_t DAC_GetBufferReadPointer(DAC_TypeDef *base)
{
    return ((base->C2 & DAC_C2_DACBFRP_MASK) >> DAC_C2_DACBFRP_SHIFT);
}

/*!
 * @brief Sets the current read pointer of the DAC buffer.
 *
 * This function sets the current read pointer of the DAC buffer.
 * The current output value depends on the item indexed by the read pointer. It is updated either by a
 * software trigger or a hardware trigger. After the read pointer changes, the DAC output value also changes.
 *
 * @param base  DAC peripheral base address.
 * @param index Setting an index value for the pointer.
 */
void DAC_SetBufferReadPointer(DAC_TypeDef *base, uint8_t index);

/*!
 * @brief Enables interrupts for the DAC buffer.
 *
 * @param base DAC peripheral base address.
 * @param mask Mask value for interrupts. See "_dac_buffer_interrupt_enable".
 */
void DAC_EnableBufferInterrupts(DAC_TypeDef *base, uint32_t mask);

/*!
 * @brief Disables interrupts for the DAC buffer.
 *
 * @param base DAC peripheral base address.
 * @param mask Mask value for interrupts. See  "_dac_buffer_interrupt_enable".
 */
void DAC_DisableBufferInterrupts(DAC_TypeDef *base, uint32_t mask);

/*!
 * @brief Gets the flags of events for the DAC buffer.
 *
 * @param  base DAC peripheral base address.
 *
 * @return      Mask value for the asserted flags. See  "_dac_buffer_status_flags".
 */
uint32_t DAC_GetBufferStatusFlags(DAC_TypeDef *base);

/*!
 * @brief Clears the flags of events for the DAC buffer.
 *
 * @param base DAC peripheral base address.
 * @param mask Mask value for flags. See "_dac_buffer_status_flags_t".
 */
void DAC_ClearBufferStatusFlags(DAC_TypeDef *base, uint32_t mask);

/* @} */

#if defined(__cplusplus)
}
#endif

#if KINETIS_DAC_USE_DAC0 && !defined(__DOXYGEN__)
extern DACDriver DACD0;
#endif

#if KINETIS_DAC_USE_DAC1 && !defined(__DOXYGEN__)
extern DACDriver DACD1;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void dac_lld_init(void);
  void dac_lld_start(DACDriver *dacp);
  void dac_lld_stop(DACDriver *dacp);
  void dac_lld_put_channel(DACDriver *dacp,
                           dacchannel_t channel,
                           dacsample_t sample);
  void dac_lld_start_conversion(DACDriver *dacp);
  void dac_lld_stop_conversion(DACDriver *dacp);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_DAC */

#endif /* HAL_DAC_LLD_H */

/** @} */
