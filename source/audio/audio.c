/*
 * (c) 2016-2025, Infineon Technologies AG, or an affiliate of Infineon
 * Technologies AG. All rights reserved.
 * This software, associated documentation and materials ("Software") is
 * owned by Infineon Technologies AG or one of its affiliates ("Infineon")
 * and is protected by and subject to worldwide patent protection, worldwide
 * copyright laws, and international treaty provisions. Therefore, you may use
 * this Software only as provided in the license agreement accompanying the
 * software package from which you obtained this Software. If no license
 * agreement applies, then any use, reproduction, modification, translation, or
 * compilation of this Software is prohibited without the express written
 * permission of Infineon.
 *
 * Disclaimer: UNLESS OTHERWISE EXPRESSLY AGREED WITH INFINEON, THIS SOFTWARE
 * IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING, BUT NOT LIMITED TO, ALL WARRANTIES OF NON-INFRINGEMENT OF
 * THIRD-PARTY RIGHTS AND IMPLIED WARRANTIES SUCH AS WARRANTIES OF FITNESS FOR A
 * SPECIFIC USE/PURPOSE OR MERCHANTABILITY.
 * Infineon reserves the right to make changes to the Software without notice.
 * You are responsible for properly designing, programming, and testing the
 * functionality and safety of your intended application of the Software, as
 * well as complying with any legal requirements related to its use. Infineon
 * does not guarantee that the Software will be free from intrusion, data theft
 * or loss, or other breaches ("Security Breaches"), and Infineon shall have
 * no liability arising out of any Security Breaches. Unless otherwise
 * explicitly approved by Infineon, the Software may not be used in any
 * application where a failure of the Product or any consequences of the use
 * thereof can reasonably be expected to result in personal injury.
 */

/** @file
 *
 * audio.c
 *
 * This file contains audio functions
 *
 */

#ifdef SUPPORT_AUDIO
#include "app.h"
#include "wiced_timer.h"
#include "hci_control_api.h"

#if CHIP!=20829 && defined WICED_BT_TRACE_ENABLE
# undef WICED_BT_TRACE
# define WICED_BT_TRACE(format,...)   wiced_printf(NULL, 0, (format"\n"), ##__VA_ARGS__)
#endif

#if AUDIO_TRACE==1
 #define APP_AUDIO_TRACE        WICED_BT_TRACE
 #define APP_AUDIO_TRACE2(...)
#elif AUDIO_TRACE>1
 #define APP_AUDIO_TRACE        WICED_BT_TRACE
 #define APP_AUDIO_TRACE2       WICED_BT_TRACE
#else
 #define APP_AUDIO_TRACE(...)
 #define APP_AUDIO_TRACE2(...)
#endif

#define LED_AUDIO_ON()      led_on(RED_LED)
#define LED_AUDIO_OFF()     led_off(RED_LED)

/////////////////////////////////////////////////////////////////////////////////

enum
{
    STATE_IDLE,
    STATE_START_REQ,
    STATE_STOP_REQ,
    STATE_ACTIVE,
};


typedef struct
{
    uint8_t buttonHoldToTalk:1;       // 0: Press button acting as audio on or off. 1:Audio is on when pressed.
    uint8_t to_hci:1;                 // 0: audio data is to sent OTA,  1: audio data is send to HCI
    uint8_t from_hci:1;               // 0: audio data is from MICsend MIC, 1:audio data to coming from HCI
    uint8_t stop_requested:1;
    uint8_t encoded:1;                // 0: Send or Recive to HCI in PCM Raw data. 1: Send or Recive to HCI in compressed Codec encoded format.
    uint8_t enc_seq;                  // Encoded data sequence
    uint8_t auto_stop_in_sec;         // Automatically stop audio in sec. Use 0 to disable.
    uint8_t state;
    uint8_t timer_mode;
    wiced_timer_t audio_timer;
    HidEventUserDefine  voiceEvent;
    uint16_t data_max_size;
    hidd_voice_report_t rpt;

} tAudioState;

tAudioState audio = {
    .buttonHoldToTalk = TRUE,
    .auto_stop_in_sec = 7,          // max audio play time is 7 sec.
    .data_max_size = 20,            // not used. This value is dynamically reassigned with PDU size when audio starts. (see function audio_start)
};

#define CMD_RESP_TIMEOUT_IN_SEC 2              // 2 sec timeout for command

////////////////////////////////////////////////////////////////////////
/// Private Functions
////////////////////////////////////////////////////////////////////////

/**
 * audio_timer_timeout_handler
 *
 * This function handles audio timer timeout.
 */
static void audio_timer_timeout_handler( uint32_t arg )
{
    switch (audio.state) {
    case STATE_STOP_REQ:
        if (audio_is_active())
        {
            audio_stop();
        }
        break;

    case STATE_START_REQ:
        if (!audio_is_active())
        {
            audio.state = STATE_IDLE;
        }
        break;

    case STATE_ACTIVE:
        WICED_BT_TRACE("MIC Command Pending Timeout");
        if (audio_is_active())
        {
            audio_stop_request();
        }
        else
        {
            audio.state = STATE_IDLE;
        }
        break;
    }
}

/**
 * start_timer
 *
 * Calling this function based on the state to start audio timer
 *
 */
static void start_timer(uint8_t mode)
{
    if (wiced_is_timer_in_use(&audio.audio_timer))
    {
        wiced_stop_timer(&audio.audio_timer);
    }

    audio.state = mode;
    switch (mode) {
    case STATE_START_REQ:
    case STATE_STOP_REQ:
        if (!audio.from_hci)
        {
            wiced_start_timer(&audio.audio_timer, CMD_RESP_TIMEOUT_IN_SEC);
        }
        break;

    case STATE_ACTIVE:
        if (audio.auto_stop_in_sec && !audio.from_hci)
        {
            wiced_start_timer(&audio.audio_timer, audio.auto_stop_in_sec);
        }
        break;
    }
}

static void audio_congested_cb(wiced_bool_t congested)
{
    if (!congested)
    {
        if (audio.from_hci)
        {
            // We have room now, request more audio data from HCI
            hci_control_send_audio_data_req();
        }
        else if (audio.stop_requested)
        {
            protocol_stop_request();
            start_timer(STATE_STOP_REQ);
            audio.stop_requested = 0;
        }
    }
}

////////////////////////////////////////////////////////////////////////
/// Public Functions
////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////
/// Returns the input data data count in 16bit-data unit
/////////////////////////////////////////////////////////////////////////////////
uint16_t audio_wdata_cnt()
{
    return (audio.from_hci && audio.encoded) ? ENCODE_BUF_SIZE / 2 - 1: AUDIO_SAMPLE_CNT;
}

/////////////////////////////////////////////////////////////////////////////////
/// This function amplifies audio data
/////////////////////////////////////////////////////////////////////////////////
void audio_amplify_data(int16_t * p_data, uint16_t cnt, uint32_t multiplier)
{
    int32_t data;
    while (cnt--)
    {
        data = *p_data * multiplier;

        // check for data range
        if(data > 32767)
        {
            data = 32767;
        }
        else if(data < -32768)
        {
            data = -32768;
        }

        *p_data++ = data;
    }
}

// Set the state to send audio data to HCI
void audio_set_routing_to_hci(wiced_bool_t en, wiced_bool_t encoded )
{
    audio.to_hci = en;
    audio.encoded = en ? encoded : 0;
    APP_AUDIO_TRACE("Set audio output to %s %s", en ? "HCI" : "LE", audio.encoded ? "Codec" : "PCM Raw");
}

// Set the state to get the  audio data from HCI
void audio_set_routing_from_hci(wiced_bool_t en, wiced_bool_t encoded)
{
    audio.from_hci = en;
    audio.encoded = en ? encoded : 0;
    audio.enc_seq = 0;
    APP_AUDIO_TRACE("Set audio input from %s %s", en ? "HCI" : "MIC", audio.encoded ? "Codec" : "PCM Raw");
}

/////////////////////////////////////////////////////////////////////////////////
/// Returns supported features
/////////////////////////////////////////////////////////////////////////////////
uint8_t audio_capability(void)
{
    uint8_t capa = HCI_CONTROL_HIDD_AUDIO;

#ifdef SUPPORT_DIGITAL_MIC
    capa |= HCI_CONTROL_HIDD_AUDIO_DIGITAL_MIC;
#endif

    return capa;
}

/////////////////////////////////////////////////////////////////////////////////
/// This function polls for voice activity and queues any events in the
/// FW event queue.
/////////////////////////////////////////////////////////////////////////////////
void audio_pollActivityVoice(void)
{
    //voice is active, poll and queue voice data
    if ( audio_is_active() )
    {
        while (mic_audio_poll_activity((void *)&audio.voiceEvent))
        {
            int16_t * ptr = ((hidd_voice_report_t *)audio.voiceEvent.userDataPtr)->dataBuffer;
#ifdef SUPPORT_DIGITAL_MIC
            // PDM does not have hardware amplifier. We need amplify the audio
            audio_amplify_data(ptr, AUDIO_SAMPLE_CNT, AUDIO_PDM_MULTIPLIER);
#endif
            audio_process_input_data(ptr, AUDIO_SAMPLE_CNT);
        }
    }
}

/**
 * audio_process_input_data
 *
 * ptr16 is a pointer to uint16_t. When input if from MIC, it is pointing to PCM raw data.
 *                                 When the input is from HCI, it is PCM raw data if audio.encoded is not set. Otherwise, it is encoded data
 *
 */
void audio_process_input_data(int16_t * ptr16, uint16_t word_cnt)
{
    uint8_t frame[ENCODE_BUF_SIZE];
    uint8_t * p_data = (uint8_t *) ptr16;
    uint16_t len = word_cnt * 2;

    APP_AUDIO_TRACE2("%s: word_cnt %d, from_hci:%d, to_hci:%d, encoded:%d", __FUNCTION__, word_cnt, audio.from_hci, audio.to_hci, audio.encoded);

#if defined(TESTING_USING_HCI)
    if (audio.from_hci && audio.encoded)
    {
        frame[0] = audio.enc_seq++;
        frame[1] = 0;

        APP_AUDIO_TRACE2("coying encoded data, size %d to buffer, header data:%A, enc_seq:%d", len, p_data, ENC_HEADER_SIZE, audio.enc_seq);
        // we need to add sequence & channel in front of data
        memcpy(&frame[2], p_data, len);
        len += 2;
        p_data = frame;
    }
    else
    {
        // We reach here because the input is PCM raw data.
 #ifndef CODEC_DECODE_SUPPORT
        if (!audio.to_hci || audio.encoded)
 #endif
        {
#endif
            len = codec_encode(ptr16, word_cnt, frame);
            p_data = frame;
            APP_AUDIO_TRACE2("encoded data size is %d, seq=%d", len, frame[0]);

#if defined(TESTING_USING_HCI)
 #ifdef CODEC_DECODE_SUPPORT
            if (audio.to_hci && !audio.encoded)
            {
                // decode back to PCM
                len = codec_decode(frame, len, ptr16) * 2;
                p_data = (uint8_t *) ptr16;
                APP_AUDIO_TRACE2("decoded back data, size is %d", len);
            }
 #endif
        }
    }

    if (audio.to_hci)
    {
        APP_AUDIO_TRACE2("Forwarding data, len:%d, to HCI, encoded=%d", len, audio.encoded);
        if (audio.encoded)
        {
            p_data += 2;  // skip seq & channel bytes
            len -= 2;
        }

        // Send to HCI
        while (len)
        {
            uint16_t size = len >= MAX_SEND_HCI_BYTE_CNT ? MAX_SEND_HCI_BYTE_CNT : len;

            hci_control_send_data(HCI_CONTROL_HIDD_EVENT_AUDIO_DATA, p_data, size);
            len -= size;
            p_data += size;
        }
    }
    else
#endif
    // send data over the air
    {
        APP_AUDIO_TRACE2("Send audio data, len:%d, to the air", len);
        // Send over the air.
        // As long as the len is larger than audio MTU size, we fragement the data
        while (len)
        {
            uint16_t size = len >= audio.data_max_size ? audio.data_max_size : len;
            audio_send_data(p_data, size);
            len -= size;
            p_data += size;
        };
    }
}


/**
 * audio_stop_request
 *
 * Calling this function to send a request to host to stop audio
 *
 */
void audio_stop_request()
{
    APP_AUDIO_TRACE("%s", __FUNCTION__);

    audio.from_hci = 0;
    if (gatt.congested)
    {
        audio.stop_requested = 1;
    }
    else
    {
        protocol_stop_request();
        start_timer(STATE_STOP_REQ);
    }
}

/**
 * audio_start_request
 *
 * Calling this function to send a request to host to start audio
 *
 */
void audio_start_request()
{
    APP_AUDIO_TRACE("%s", __FUNCTION__);
    protocol_start_request();
    start_timer(STATE_START_REQ);
}

/**
 * audio_stop
 *
 * Calling this function to stop audio activity
 *
 */
void audio_stop(void)
{
    if (audio_is_active())
    {
        WICED_BT_TRACE("Audio Stop");
    }

    wiced_stop_timer(&audio.audio_timer);

    mic_stop();
//    audio_codec_stop();
    protocol_stop();

//    hidd_blelink_enable_poll_callback(WICED_TRUE);      // re-enable back
    audio.stop_requested = 0;

    LED_AUDIO_OFF();
}

/**
 * audio_start
 *
 * Calling this function to start audio activity
 *
 */
void audio_start(void)
{
    APP_AUDIO_TRACE("Audio Start");

    audio.data_max_size = protocol_audio_data_max_size();

    codec_start();  // Reset encoder
    protocol_start();

#if defined(TESTING_USING_HCI)
    if (audio.from_hci)
    {
        WICED_BT_TRACE("Audio input from HCI");
        audio.rpt.sqn = 0;
        // Request audio data from HCI
        hci_control_send_audio_data_req();
    }
    else
#endif
    {
        WICED_BT_TRACE("Audio input from MIC");
        mic_start();
        // kick start audio
        mic_audio_poll_activity((void *)&audio.voiceEvent);
    }

    LED_AUDIO_ON();
    start_timer(STATE_ACTIVE);
}

/**
 * audio_button
 *
 * Call this function when audio button is pressed or released.
 *
 */
void audio_button(wiced_bool_t key_pressed)
{
    APP_AUDIO_TRACE("%s %s", __FUNCTION__, key_pressed ? "down" : "up");

    if (key_pressed)
    {
        wiced_bool_t audio_active = audio_is_active();

        if (audio.buttonHoldToTalk || !audio_active)
        {
            audio_start_request();
        }
        // otherwise, audio button is toggle switch to start or to stop
        else if (!audio.buttonHoldToTalk && audio_active)
        {
            audio_stop_request();
        }
    }
    else
    {
        // if buttonHoldToTalk is true, it means audio is active while key is pressed down.
        // Since the button is released, we stop
        if (audio.buttonHoldToTalk)
        {
            audio_stop_request();
        }
    }
}

/**
 * audio_shutdown
 *
 * Calling this function to shutdown audio system
 *
 */
void audio_shutdown(void)
{
    //stop audio
    mic_audio_stop();
}

/**
 * audio_data_from_hci
 *
 */
wiced_bool_t audio_data_from_hci()
{
    return audio.from_hci;
}

/**
 * audio_init
 *
 * This function should be called at startup to initialize audio.
 * GATT must be initialized first before calling this function. (bt_init must called first)
 *
 */
void audio_init()
{
    APP_AUDIO_TRACE("%s", __FUNCTION__);

    audio.voiceEvent.eventInfo.eventType = HID_EVENT_VOICE_DATA_AVAILABLE;
    wiced_init_timer( &audio.audio_timer, audio_timer_timeout_handler, 0, WICED_SECONDS_TIMER );
    protocol_init();
    codec_init();
    mic_init(audio_pollActivityVoice); // mic_init should be called last after initializing protocl & codec
//    gatt_register_congested_callback(audio_congested_cb);
}

#endif // SUPPORT_AUDIO
