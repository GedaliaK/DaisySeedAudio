// one seed sends to a pi and receives back on a different nrf (same seed)

#include <Arduino.h>
#include "DaisyDuino.h"
#include "RF24.h"
#include <chrono>

/*
 * See documentation at https://nRF24.github.io/RF24
 * See License information at root directory of this library
 * Author: Brendan Doherty (2bndy5)
 */

// const uint16_t ce_pin = 7;
// const uint16_t csn_pin = 8; // chip select
// const uint8_t mosi_pin = 11;
// const uint8_t miso_pin = 10;
// const uint8_t sck_pin = 9;

const uint8_t mosi_pin = 11 - 1;
const uint8_t miso_pin = 10 - 1;
const uint8_t sck_pin = 9 - 1;
const uint16_t csn_pin = 8 - 1; // chip select
const uint16_t ce_pin = 7 - 1;
const u_int8_t channel_number = 0;
#define SAMP_RATE AUDIO_SR_48K
#define SAMP_RATE_HZ 48000.f // at 16k the 250 hz buzz went away
#define LUT_N 1024
float LUT[LUT_N];
const float INV_16MAX = 1.f / float(INT16_MAX);
bool use_synthetic_data = true;
enum sample_types
{
    float32 = 0,
    int32,
    int16,
    int8
};

enum nodes
{
    seed1 = 0, // tx
    seed2,     // rx
    pi         // rx and tx
};

// make sure these match
#define SAMP_TYPE int16_t
const sample_types samp_type = int16;
#define PAYLOAD_BYTES 32
#define SAMPS_PER_PAYLOAD int(PAYLOAD_BYTES / sizeof(SAMP_TYPE))
volatile SAMP_TYPE send_counter = 0;

// instantiate an object for the nRF24L01 transceiver
// RF24 radio(7, 8); // using pin 7 for the CE pin, and pin 8 for the CSN pin
RF24 radio_tx(ce_pin, csn_pin, int(5e6));         // using pin 7 for the CE pin, and pin 8 for the CSN pin
RF24 radio_rx(ce_pin - 2, csn_pin - 2, int(5e6)); // using pin 7 for the CE pin, and pin 8 for the CSN pin

// Let these addresses be used for the pair
// seed1 to pi at 1node, pi to seed2 at 2node
uint8_t address[][6] = {"1Node", "2Node", "3Node"};
// It is very helpful to think of an address as a path instead of as
// an identifying device destination

// Used to control whether this node is sending or receiving
// bool role = true; // true = TX role, false = RX role

// to use different addresses on a pair of radios, we need a variable to
// uniquely identify which address this radio will use to transmit
// bool radioNumber = role; // 0 uses address[0] to transmit, 1 uses address[1] to transmit
// int radioNumber = seed2;

// For this example, we'll be using a payload containing
// a single float number that will be incremented
// on every successful transmission
// float payload[8] = {0};
const float int16_factor = (1.f / 32767.f);

#define SAMPS_PER_BUF 64 // was 32
#define N_BUFS int(16)   // was 4
#define TOTAL_SAMPS SAMPS_PER_BUF *N_BUFS
#define CRC_LENGTH RF24_CRC_16
float audio_buffer[TOTAL_SAMPS] = {0};
SAMP_TYPE payload[SAMPS_PER_PAYLOAD] = {0};

float audio_in_buffer[TOTAL_SAMPS] = {0};
int audio_in_read_idx = 0;
int audio_in_write_idx = 0;
int audio_in_available_samps = 0;

volatile int read_idx = 0;
volatile int write_idx = 0;
volatile int available_samps = 0;
const bool use_auto_ack = false;
DaisyHardware hw;

size_t num_channels;

float x, t = 0.f, omega = 2.f * 3.14159f * 440.f, dt = 1.f / 16e3;
int sine_dx = float(LUT_N) / SAMP_RATE_HZ * 363.f;
int sine_idx = 0;
volatile bool overun = false;
bool listening_paused = false;

volatile int callback_state = LOW;
volatile bool enough_samples = false;
volatile bool error_weird = false;

void MyCallback(float **in, float **out, size_t size)
{
    // if tx node, copy to output buffer for nrf24
    for (size_t i = 0; i < size; i++)
    {
        // audio_in_buffer[audio_in_write_idx] = 0.5f * (in[0][i] + in[1][i]);
        audio_in_buffer[audio_in_write_idx] = in[0][i];

        // hack to make relative "zero level" ground
        // out[0][i] = 0.f;
        // out[1][i] = 0.f;
        audio_in_write_idx++;

        if (audio_in_write_idx >= (TOTAL_SAMPS))
        {
            audio_in_write_idx = 0;
        }
        audio_in_available_samps++;
    }

    // rx control
    callback_state = (callback_state == HIGH) ? LOW : HIGH;
    float amp = 0.9f;
    // Serial.print("size: ");
    // Serial.println(size);
    if (available_samps >= (TOTAL_SAMPS - SAMPS_PER_BUF)) // we want 1 buffer available
        enough_samples = true;

    if (enough_samples && (available_samps >= SAMPS_PER_BUF)) // we want 1 buffer available
    {
        overun = false;

        for (size_t i = 0; i < size; i++)
        {
            out[0][i] = 0;
            out[1][i] = 0.9f * audio_buffer[read_idx];

            // // hacky? skip over samples if we have too many
            // if (available_samps >= TOTAL_SAMPS)
            // {
            //     read_idx += 2;
            //     available_samps -= 2;
            // }
            // else
            // {
            //     read_idx++;
            //     available_samps--;
            // }

            read_idx++;
            available_samps--;
            if (read_idx >= (TOTAL_SAMPS))
            {
                read_idx = 0;
            }
        }
    }
    else
    {
        enough_samples = false;
        // Serial.println("OVERUN :(");
        overun = true;
        for (size_t i = 0; i < SAMPS_PER_BUF; i++)
        {
            for (size_t chn = 0; chn < num_channels; chn++)
            {
                out[chn][i] = .0f;
            }
        }
    }
}

void setup()
{
    for (int i = 0; i < LUT_N; i++)
    {
        LUT[i] = sin(2.f * 3.14159 * float(i) / float(LUT_N));
    }

    // hw = DAISY.init(DAISY_SEED, AUDIO_SR_48K);
    // DAISY.SetAudioBlockSize(48);
    hw = DAISY.init(DAISY_SEED, SAMP_RATE);
    DAISY.SetAudioBlockSize(SAMPS_PER_BUF);
    // DAISY.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_32KHZ);

    num_channels = hw.num_channels;
    // for (int i = 0; i < 8; i++)
    // {
    // 	payload[i] = 0;
    // }
    const int led = LED_BUILTIN;
    pinMode(led, OUTPUT);
    // pinMode(1, OUTPUT);
    // pinMode(2, OUTPUT);
    // pinMode(22, OUTPUT);
    // digitalWrite(23, HIGH);
    // digitalWrite(1, HIGH);

    Serial.begin();
    // while (!Serial)
    // {
    // 	// some boards need to wait to ensure access to serial over USB
    // }

    SPI.begin();
    SPI.setMOSI(mosi_pin);
    SPI.setMISO(miso_pin);
    SPI.setSCLK(sck_pin);
    // initialize the transceiver on the SPI bus
    if (!radio_tx.begin())
    {
        Serial.println(F("radio hardware is not responding!!"));
        while (1)
        {
            digitalWrite(led, HIGH);
            delay(1000);
            digitalWrite(led, LOW);
            delay(1000);

        } // hold in infinite loop
    }

    if (!radio_rx.begin())
    {
        Serial.println(F("radio hardware is not responding!!"));
        while (1)
        {
            digitalWrite(led, HIGH);
            delay(1000);
            digitalWrite(led, LOW);
            delay(1000);

        } // hold in infinite loop
    }

    radio_tx.setPALevel(RF24_PA_MAX, 1); // RF24_PA_MAX is default.
    radio_rx.setPALevel(RF24_PA_MAX, 1); // RF24_PA_MAX is default.

    radio_tx.setChannel(0);
    radio_rx.setChannel(125);

    radio_tx.setCRCLength(CRC_LENGTH);
    radio_rx.setCRCLength(CRC_LENGTH);
    // radio.disableCRC();
    // save on transmission time by setting the radio to only transmit the
    // number of bytes we need to transmit a float
    radio_tx.setPayloadSize(PAYLOAD_BYTES); // float datatype occupies 4 bytes
    radio_tx.setDataRate(RF24_2MBPS);

    radio_rx.setPayloadSize(PAYLOAD_BYTES); // float datatype occupies 4 bytes
    radio_rx.setDataRate(RF24_2MBPS);

    // seed1 is always a tx node, so it always write to the pi
    radio_tx.openWritingPipe(address[pi]); // always uses pipe 0

    // seed2 is always an rx node, so it always reads from the seed2
    radio_rx.openReadingPipe(1, address[seed2]); // using pipe 1

    radio_tx.setAutoAck(0);
    radio_rx.setAutoAck(0);

    // additional setup specific to the node's role

    radio_tx.stopListening();  // put radio in TX mode
    radio_rx.startListening(); // put radio in RX mode

    DAISY.StartAudio(MyCallback);

} // setup

int led_state = HIGH;

int last_read_us = micros();
const int delay_time_us = float(32) / float(sizeof(SAMP_TYPE)) / SAMP_RATE_HZ * 1e6;
int payload_state = LOW;

void loop()
{
    // tx control
    if (audio_in_available_samps >= SAMPS_PER_PAYLOAD)
    {
        // This device is a TX node
        for (int i = 0; i < SAMPS_PER_PAYLOAD; i++)
        {
            switch (samp_type)
            {
            case float32:
                payload[i] = audio_in_buffer[audio_in_read_idx];
                break;
            case int32:
                payload[i] = int32_t(audio_in_buffer[audio_in_read_idx] * float(INT32_MAX));
                break;
            case int16:
                if (use_synthetic_data)
                {
                    x = 0.75f * LUT[sine_idx]; // increment float payload
                    sine_idx += sine_dx;
                    if (sine_idx >= LUT_N)
                        sine_idx %= LUT_N;

                    payload[i] = int16_t(x * float(INT16_MAX));
                }
                else
                {
                    payload[i] = int16_t(audio_in_buffer[audio_in_read_idx] * float(INT16_MAX));
                }
                break;
            case int8:
                payload[i] = int8_t(audio_in_buffer[audio_in_read_idx] * float(INT8_MAX));
                break;
            }
            // payload[i] = send_counter;
            send_counter++;
            if (send_counter == INT16_MAX)
                send_counter = 0;
            audio_in_read_idx++;
            if (audio_in_read_idx >= (TOTAL_SAMPS))
                audio_in_read_idx = 0;
        }
        audio_in_available_samps -= SAMPS_PER_PAYLOAD;

        // unsigned long start_timer = micros();						// start the timer
        bool report = radio_tx.writeFast(&payload, PAYLOAD_BYTES); // transmit & save the report
                                                                   // unsigned long end_timer = micros();							// end the timer
    }
    // rx control
    uint8_t pipe;

    if (radio_rx.available(&pipe))
    { // is there a payload? get the pipe number that recieved it

        led_state = (led_state == HIGH) ? LOW : HIGH;
        uint8_t bytes = radio_rx.getPayloadSize(); // get the size of the payload
                                                   // radio.read(&audio_buffer[write_idx], 32); // fetch payload from FIFO

        // digitalWrite(1, HIGH);
        payload_state = (payload_state == HIGH) ? LOW : HIGH;
        radio_rx.read(&payload, PAYLOAD_BYTES); // fetch payload from FIFO
        // digitalWrite(1, LOW);
        float amp = 0.7f;
        for (int i = 0; i < SAMPS_PER_PAYLOAD; i++)
        {
            switch (samp_type)
            {
            case float32:
                audio_buffer[write_idx] = amp * payload[i];
                break;
            case int32:
                audio_buffer[write_idx] = amp * float(payload[i]) / float(INT32_MAX);
                break;
            case int16:
                audio_buffer[write_idx] = amp * float(payload[i]) * INV_16MAX;
                break;
            case int8:
                audio_buffer[write_idx] = amp * float(payload[i]) / float(INT8_MAX);
                break;
            }
            write_idx++;
            if (write_idx >= (TOTAL_SAMPS))
                write_idx = 0;
        }
        available_samps += SAMPS_PER_PAYLOAD;
    }

} // loop
