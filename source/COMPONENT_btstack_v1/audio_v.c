/*
 * Copyright 2016-2023, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
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
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

/*******************************************************************************
*
* File Name: hidd_audio.c
*
* This file implements the microphone for voice using Audio ADC
*******************************************************************************/
#ifdef SUPPORT_AUDIO
#include "wiced_hal_gpio.h"
#include "wiced_bt_event.h"
#include "app.h"

#if CHIP!=20829 && defined WICED_BT_TRACE_ENABLE
# undef WICED_BT_TRACE
# define WICED_BT_TRACE(format,...)   wiced_printf(NULL, 0, (format"\n"), ##__VA_ARGS__)
#endif

#define PCM_AUDIO_BUFFER_SIZE       120  // in 16-bit sample

// Definition of bitmap for for buffer state in hidd_voice_report_t
#define PCMBUFFSTATE_FIFO_MASK  0x0007  // 3 bits only
#define PCMBUFFSTATE_FIFO_INP_SHFT 0   // Bit 0-2
#define PCMBUFFSTATE_FIFO_OUT_SHFT 3   // Bit 3-5
#define PCMBUFFSTATE_FIFO_UNRFLOW_SHFT 6   // Bit 6-8
#define PCMBUFFSTATE_FIFO_OVRFLOW_SHFT 9   // Bit 9-11

static tMicAudio micAudioDriver;
uint32_t packetDelayCount;
uint16_t audioBufSize16; // in 16-bit sample
hidd_audio_encoding_t audioEncType;
uint8_t mic_stop_command_pending = 0;

//adc audio fifo number
#define BUFFER_ADC_AUDIO_FIFO_NUM	25

AdcAudioFifo wicedAdcAudioFifo[BUFFER_ADC_AUDIO_FIFO_NUM] = {0};
#ifdef SUPPORT_DIGITAL_MIC
AdcAudioFifo wicedAdcAudioFifo2[BUFFER_ADC_AUDIO_FIFO_NUM] = {0};
#else
#define wicedAdcAudioFifo2 NULL
#endif

extern void wiced_bt_allowPeripheralLatency(wiced_bool_t allow);

extern int32_t custom_gain_boost;
extern AdcAudioDrcSettings adc_audioDrcSettings;

#ifdef ENABLE_ADC_AUDIO_ENHANCEMENTS
extern AdcAudioFilterCfg_t adcAudioFilterData;
#define eq_filter_enabled() (adcAudioFilterData.eqFilter.coeff[0])
#endif


#ifdef FATORY_TEST_SUPPORT
extern uint8_t factory_mode;

extern uint8_t *audio_byte_pool;
extern uint32_t audio_byte_pool_size;
extern uint32_t index_in_byte_pool;
#endif

extern void audio_drc_loop_init(UINT8 pgaGain, uint32_t sampleRate);
extern void adc_assignFilterData(AdcAudioFilterCfg_t * data);
extern void adc_stopAudio_fix(void);

extern uint32_t appUtils_cpuIntDisable(void);
extern void   appUtils_cpuIntEnable(uint32_t _newPosture);

void micAudio_micAudioCallback(UINT8 *audioData, UINT32 receivedLength, UINT32 adcFIFOStatus);

typedef struct {
    wiced_bool_t  enabled:1;
    wiced_bool_t  disable_level:1;
    uint8_t       gpio;
} pin_en_mic_t;
static pin_en_mic_t pin_en_mic={0};

#ifdef SUPPORT_DIGITAL_MIC
extern void adc_pdm_pinconfig(UINT8 ch1, UINT8 risingEdge1, UINT8 ch2, UINT8 risingEdge2, UINT8 clk);
 #if CHIP==20835
static uint8_t gpioPDMInClk = WICED_P27;   // default PDM clk in case if app does not assign
static uint8_t gpioPDMInData = WICED_P26;   // default PDM data in case if app does not assign
 #elif CHIP==20721
static uint8_t gpioPDMInClk = WICED_P26;   // default PDM clk in case if app does not assign
static uint8_t gpioPDMInData = WICED_P27;   // default PDM data in case if app does not assign
 #endif
#define pdm_in_ch1_rising_edge 1
#define pdm_in_ch2_rising_edge 0

////////////////////////////////////////////////////////////////////////////////
/// Configure Audio pdm pins
///
/// \param clk, data gpio pins
///
/// \return none
////////////////////////////////////////////////////////////////////////////////
void mic_assign_mic_pdm_pins(uint8_t clk, uint8_t data)
{
 #if CHIP==20835
    extern uint8_t useWicedPDMAudio;
    useWicedPDMAudio = TRUE;
 #endif

    // The GPIO might have been configured for LED by platfrom.c at startup. We need to reconfigure the pin
    // to ensure the GPIO is configured propoerly.
    wiced_hal_gpio_configure_pin(clk, GPIO_INPUT_ENABLE, 0);
    wiced_hal_gpio_configure_pin(data, GPIO_INPUT_ENABLE, 0);

    gpioPDMInClk = clk;
    gpioPDMInData = data;
}
#endif

///////////////////////////////////////////////////////////////////////////////
/// Assgin MIC EN pin. By default is disabled until app assigns a pin
///
/// \param gpio
/// \param default level (disabled logic)
///
/// \return WICED_TRUE/WICED_FALSE
void mic_assign_mic_en_pin(uint8_t gpio, wiced_bool_t disable_level)
{
    pin_en_mic.enabled = TRUE;
    pin_en_mic.disable_level = disable_level;
    pin_en_mic.gpio = gpio;
    wiced_hal_gpio_configure_pin(gpio, (disable_level ? GPIO_PULL_UP : GPIO_PULL_DOWN) | GPIO_OUTPUT_ENABLE, disable_level);
    wiced_hal_gpio_slimboot_reenforce_cfg(gpio, GPIO_OUTPUT_ENABLE);
}

////////////////////////////////////////////////////////////////////////////////
/// Configure Audio ADC for microphone
///
/// \param config - configuration
///
/// \return none
////////////////////////////////////////////////////////////////////////////////
void mic_audio_config(hidd_microphone_config_t * config)
{
    adc_config();
    adc_audioConfig(wicedAdcAudioFifo, BUFFER_ADC_AUDIO_FIFO_NUM, wicedAdcAudioFifo2);

    micAudioDriver.codec        = NULL;
    micAudioDriver.audioAdcData = config->audio_fifo;
    micAudioDriver.dataCnt      = config->data_count;
    micAudioDriver.fifo_cnt     = config->fifo_count;
    micAudioDriver.pin_en_audio = config->enable;
    micAudioDriver.audio_gain   = config->audio_gain;
    micAudioDriver.audio_boost  = config->audio_boost;
    micAudioDriver.audio_delay  = config->audio_delay;
    micAudioDriver.codec_sampling_freq = config->codec_sampling_freq;

}

///////////////////////////////////////////////////////////////////////////////
// Set DRC parameters
///////////////////////////////////////////////////////////////////////////////
void micAudio_setDrcParameters(uint8_t *drcCustomConfigBuf)
{
    memcpy(&adc_audioDrcSettings, drcCustomConfigBuf, sizeof(AdcAudioDrcSettings));
}

///////////////////////////////////////////////////////////////////////////////
/// Enhanced configure Audio ADC for microphone (DRC, eq filter etc)
///
/// \param pConfig - pointer to the configuration data
///
/// \return none
///////////////////////////////////////////////////////////////////////////////
void mic_audio_config_enhanced(uint8_t *pConfig)
{
    hidd_microphone_enhanced_config_t * pMicAudioConfiguration = (hidd_microphone_enhanced_config_t *)pConfig;

    audioEncType = pMicAudioConfiguration->audioEncType;
    audioBufSize16 = AUDIO_SAMPLE_CNT;

    custom_gain_boost = pMicAudioConfiguration->custom_gain_boost;  //Increase gain 3.5 dB  1.496 = 10^(3.5/20)

    micAudio_setDrcParameters((uint8_t *)&pMicAudioConfiguration->drcSettings);

#ifdef ENABLE_ADC_AUDIO_ENHANCEMENTS
    adc_assignFilterData(&pMicAudioConfiguration->audioFilterData);
#endif

    audio_drc_loop_init(micAudioDriver.audio_gain, 16000);
}

////////////////////////////////////////////////////////////////////////////////
/// microphone init
///
/// \param callback - application callback function when audio data is generated and ready to send
/// \param context - pass back to the application as callback parameter as is passed in
///
/// \return none
////////////////////////////////////////////////////////////////////////////////
void  mic_audio_init(void (*callback)(void*), void* context)
{
#ifdef SBC_ENCODER
    SBC_ENC_PARAMS *p_mSBCEncParams =  &mSBCEncParams;
#endif

    // turn codec power off by default
    if (pin_en_mic.enabled)
    {
        wiced_hal_gpio_set_pin_output(pin_en_mic.gpio, pin_en_mic.disable_level);
    }

    micAudioDriver.userFn = callback;
    micAudioDriver.master = context;

    micAudioDriver.active = 0;


    micAudioDriver.voiceStarted = 1;
    mic_audio_stop();

#ifdef SBC_ENCODER
    //init SBC codec here
    p_mSBCEncParams->numOfSubBands = SBC_NUM_SUBBANDS_8;        //4,8
    p_mSBCEncParams->numOfBlocks = SBC_BLOCK_SIZE_15;               //4,8
    p_mSBCEncParams->allocationMethod  = SBC_ALLOC_METHOD_LOUDNESS;       //0,1
    p_mSBCEncParams->channelMode = SBC_CH_MODE_MONO;      //0,1,2,3
    p_mSBCEncParams->samplingFreq = SBC_SAMP_FREQ_16000;              //0,1,2,3
    p_mSBCEncParams->bitPool = WBS_MSBC_BIT_POOL_VAL;

    p_mSBCEncParams->sbc_mode = SBC_MODE_WB;
    //NOTE: we need to feed number of samples = numOfSubBands * numOfBlocks = 8*15=120 samples to the SBC encoder for it to function correctly.
#endif
}

///////////////////////////////////////////////////////////////////////////////
//  stop the audio and clear the buffer
///
/// \param none
///
/// \return none
///////////////////////////////////////////////////////////////////////////////
void mic_audio_stop()
{
    uint8_t i;

    mic_audio_set_active(0);

    if (micAudioDriver.voiceStarted)
    {
#if defined(SUPPORT_DIGITAL_MIC) && (CHIP==20835)
        adc_stopAudio_fix();
#else
        adc_stopAudio();
#endif

        // clear all data buffer
        for (i=0; i<micAudioDriver.fifo_cnt; i++)
        {
            memset(&micAudioDriver.audioAdcData[i], 0, sizeof(hidd_voice_report_t));
            micAudioDriver.dataCnt[i] = 0;
        }

        micAudioDriver.fifoOutIndex = micAudioDriver.fifoInIndex = 0;
        micAudioDriver.voiceStarted = FALSE;
        if(mic_stop_command_pending == FALSE)
        {
            wiced_bt_allowPeripheralLatency(TRUE);
        }
    }

}

///////////////////////////////////////////////////////////////////////////////
//  start Audio ADC
///////////////////////////////////////////////////////////////////////////////
#ifdef SUPPORT_DIGITAL_MIC
#define MIC_TYPE TRUE    // PDM MIC
#else
#define MIC_TYPE FALSE   // Analog MIC
#endif
void micAudio_startAudio()
{
    wiced_bt_allowPeripheralLatency(FALSE);
    micAudioDriver.voiceStarted = 1;

    micAudioDriver.audioCounter = micAudioDriver.underflowCounter = micAudioDriver.overflowCounter = 0;

#ifdef SUPPORT_DIGITAL_MIC
    adc_pdm_pinconfig(gpioPDMInData, pdm_in_ch1_rising_edge, gpioPDMInData, pdm_in_ch2_rising_edge, gpioPDMInClk);
#endif
    if (micAudioDriver.codec_sampling_freq == HIDD_CODEC_SAMP_FREQ_8K)
    {
        packetDelayCount = micAudioDriver.audio_delay / 15;  //15 mSec per packet
        adc_startAudio(8000, 16, micAudioDriver.audio_gain, micAudio_micAudioCallback, NULL, MIC_TYPE);
    }
    else
    {
        packetDelayCount = micAudioDriver.audio_delay * 2 / 15;  //7.5 mSec per packet
        adc_startAudio(16000, 16, micAudioDriver.audio_gain, micAudio_micAudioCallback, NULL, MIC_TYPE);
    }

#ifdef FATORY_TEST_SUPPORT
    if ((factory_mode == 12) && audio_byte_pool)
    {
        packetDelayCount +=6;
    }
#endif

    micAudioDriver.voicePktTime = 0; //hiddcfa_currentNativeBtClk();

#ifdef SBC_ENCODER
    SBC_Encoder_Init(&mSBCEncParams);
#endif
}

///////////////////////////////////////////////////////////////////////////////
/// poll microphone for audio activity
///
/// \param dataPtr - application callback function when audio data is generated and ready to send
/// \param context - pass back to the application as callback parameter as is passed in
///
/// \return 0 - no audio activity
///           otherwise - audio activity available
///////////////////////////////////////////////////////////////////////////////
uint8_t mic_audio_poll_activity(HidEventUserDefine * dataPtr)
{
    //WICED_BT_TRACE("micAudio_pollActivityUser");

    if (micAudioDriver.active == 1)
    {
        if (!micAudioDriver.voiceStarted)
        {
            micAudio_startAudio();
        }
        else if (micAudioDriver.audioAdcData[micAudioDriver.fifoOutIndex].reportId)
        {
            dataPtr->userDataPtr = &micAudioDriver.audioAdcData[micAudioDriver.fifoOutIndex];
            if (micAudioDriver.fifoInIndex == micAudioDriver.fifoOutIndex)
            {
                micAudioDriver.underflowCounter++;
            }

            if (++micAudioDriver.fifoOutIndex >= micAudioDriver.fifo_cnt)
            {
                micAudioDriver.fifoOutIndex = 0;
            }

            return (uint8_t)sizeof(hidd_voice_report_t);
        }
    }
    else
    {
        if (micAudioDriver.voiceStarted)
        {
            mic_audio_stop();
        }
    }

    return 0;
}

////////////////////////////////////////////////////////////////////////////////
// callback to application to send audio
////////////////////////////////////////////////////////////////////////////////
int micAudio_sendAudio(void* unused)
{
    (micAudioDriver.userFn)(micAudioDriver.master);
    return 1;
}

///////////////////////////////////////////////////////////////////////////////
// Audio ADC callback Handler
///////////////////////////////////////////////////////////////////////////////
void micAudio_micAudioCallback(UINT8 *audioData, UINT32 receivedLength, UINT32 adcFIFOStatus)
{
    int index;
    uint32_t originalLength;

#ifdef FATORY_TEST_SUPPORT
    if (factory_mode)
    {
        if ((factory_mode == 12) && audio_byte_pool) //factory_mode == 12, i.e. audio capture test.
        {
            if(packetDelayCount == 0)
            {
                if (receivedLength + index_in_byte_pool > audio_byte_pool_size)
                {
                    receivedLength = audio_byte_pool_size - index_in_byte_pool;
                }

                //copy audio FIFO data to audio byte pool
                memcpy(&audio_byte_pool[index_in_byte_pool], audioData, receivedLength);
                index_in_byte_pool += receivedLength;

                //check if audio byte pool is full
                if (index_in_byte_pool == audio_byte_pool_size)
                {
                    // trigger App for activity
                    wiced_app_event_serialize(micAudio_sendAudio, NULL);
                    factory_mode = 1;
                }
            }
            else
            {
                packetDelayCount--;
            }
        }

        return;
    }
#endif

    receivedLength /= 2; //16 bits per sample

    if(packetDelayCount == 0)
    {
        while (receivedLength > 0)
        {
            originalLength = receivedLength;

            // make sure the next copy is not over buffer size
            if (receivedLength + micAudioDriver.dataCnt[micAudioDriver.fifoInIndex] > audioBufSize16)
            {
                receivedLength = audioBufSize16 - micAudioDriver.dataCnt[micAudioDriver.fifoInIndex];
            }
            //Copy the data to our current buffer
            for(index=0; index<receivedLength; index++)
            {
                ((uint16_t *)&micAudioDriver.audioAdcData[micAudioDriver.fifoInIndex].dataBuffer[micAudioDriver.dataCnt[micAudioDriver.fifoInIndex]])[index] =
                    (audioData[2 * index + 1] << 8) + audioData[2 * index];
            }

            //Read into local byte buffer with an offset of the previous length
            //Check if close to filled up local buffer
            micAudioDriver.dataCnt[micAudioDriver.fifoInIndex] += receivedLength;
            audioData +=2*receivedLength;

            if(micAudioDriver.dataCnt[micAudioDriver.fifoInIndex] == audioBufSize16)
            {
                micAudioDriver.audioAdcData[micAudioDriver.fifoInIndex].reportId  = HIDD_VOICE_REPORT_ID;
                micAudioDriver.audioAdcData[micAudioDriver.fifoInIndex].format    = HIDD_AUDIO_DATA;
                micAudioDriver.audioAdcData[micAudioDriver.fifoInIndex].sqn       = micAudioDriver.audioCounter++;
                micAudioDriver.audioAdcData[micAudioDriver.fifoInIndex].ts        = 0; //(uint16_t) hiddcfa_BtClksSince(micAudioDriver.voicePktTime);

                //BitMap describing the state of the FIFO:
                // -------------------------------------------------------
                //FIFO_IN_INDEX:  Bit 0-2
                //FIFO_OUT_INDEX: Bit 3-5
                //OVERFLOW_COUNT: TBD
                //UNDERFLOW_COUNT: TBD
                micAudioDriver.audioAdcData[micAudioDriver.fifoInIndex].buf_state  = (micAudioDriver.fifoInIndex  & PCMBUFFSTATE_FIFO_MASK ) << PCMBUFFSTATE_FIFO_INP_SHFT;
                micAudioDriver.audioAdcData[micAudioDriver.fifoInIndex].buf_state |= (micAudioDriver.fifoOutIndex & PCMBUFFSTATE_FIFO_MASK ) << PCMBUFFSTATE_FIFO_OUT_SHFT;
                if (((micAudioDriver.fifoInIndex+1)%micAudioDriver.fifo_cnt) == micAudioDriver.fifoOutIndex)
                {
                    micAudioDriver.overflowCounter++;
                }
                micAudioDriver.audioAdcData[micAudioDriver.fifoInIndex].buf_state |= (micAudioDriver.underflowCounter & PCMBUFFSTATE_FIFO_MASK ) << PCMBUFFSTATE_FIFO_UNRFLOW_SHFT;
                micAudioDriver.audioAdcData[micAudioDriver.fifoInIndex].buf_state |= (micAudioDriver.overflowCounter  & PCMBUFFSTATE_FIFO_MASK ) << PCMBUFFSTATE_FIFO_OVRFLOW_SHFT;

                micAudioDriver.audioAdcData[micAudioDriver.fifoInIndex].dataCnt   = micAudioDriver.dataCnt[micAudioDriver.fifoInIndex];
                if (++micAudioDriver.fifoInIndex >= micAudioDriver.fifo_cnt)
                {
                    micAudioDriver.fifoInIndex = 0;
                }

                // start fresh
                micAudioDriver.dataCnt[micAudioDriver.fifoInIndex] = 0;
                micAudioDriver.audioAdcData[micAudioDriver.fifoInIndex].reportId = 0;

                if(adc_audioDrcSettings.enable == 1)
                {   //FW DRC and EQ filtering already serialize it to MPAF thread
                    micAudio_sendAudio(NULL);
                }
                else
                {
                    wiced_app_event_serialize(micAudio_sendAudio, NULL);
                }
            }

            receivedLength = originalLength - receivedLength;
        }
    }
    else
    {
        packetDelayCount--;
    }
}


///////////////////////////////////////////////////////////////////////////////
/// set audio to active/inactive
///
/// \param active - active/inactive
///
/// \return none
///////////////////////////////////////////////////////////////////////////////
void mic_audio_set_active(wiced_bool_t active)
{
    if (micAudioDriver.active == active)
        return;

    micAudioDriver.active = active;

    if (pin_en_mic.enabled)
    {
        wiced_hal_gpio_set_pin_output(pin_en_mic.gpio, active ? !pin_en_mic.disable_level : pin_en_mic.disable_level);
    }
}

////////////////////////////////////////////////////////////////////////////////
/// Configure codec sampling frequency
///
/// \param freq -- see sample_freq_e
///
/// \return none
////////////////////////////////////////////////////////////////////////////////
void mic_set_codec_sampling_freq(uint8_t freq)
{
    micAudioDriver.codec_sampling_freq = freq;
}

///////////////////////////////////////////////////////////////////////////////
/// is audio active?
///
/// \param none
///
/// \return WICED_TRUE/WICED_FALSE
///////////////////////////////////////////////////////////////////////////////
wiced_bool_t mic_audio_is_active(void)
{
    return ((micAudioDriver.active || micAudioDriver.voiceStarted) ? WICED_TRUE : WICED_FALSE);
}


///////////////////////////////////////////////////////////////////////////////
/// get the total audio FIFO number
///
/// \param none
///
/// \return the total audio FIFO number
///////////////////////////////////////////////////////////////////////////////
uint8_t mic_audio_FIFO_count(void)
{
    return micAudioDriver.fifo_cnt;
}

///////////////////////////////////////////////////////////////////////////////
/// increase internal audio overflow counter
///
/// \param none
///
/// \return none
///////////////////////////////////////////////////////////////////////////////
void mic_audio_overflow(void)
{
    //Because interrupt could also update the overflow flag , we need to ensure no race condition here.
    uint32_t oldPosture = appUtils_cpuIntDisable();

    micAudioDriver.overflowCounter++;

    // NOTE: Don't forget to re-enable interrupts.
    appUtils_cpuIntEnable(oldPosture);
}

///////////////////////////////////////////////////////////////////////////////
/// is audio overflow for the current session?
///
/// \param none
///
/// \return 0 - no overflow. otherwise indicates the number of lost audio packages
///////////////////////////////////////////////////////////////////////////////
uint16_t mic_audio_is_overflow(void)
{
    return micAudioDriver.overflowCounter;
}

#endif
