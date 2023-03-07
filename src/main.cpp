// #include <Arduino.h>
// #include "DaisyDuino.h"
// #include "RF24.h"
// #include <chrono>

// /*
//  * See documentation at https://nRF24.github.io/RF24
//  * See License information at root directory of this library
//  * Author: Brendan Doherty (2bndy5)
//  */

// // const uint16_t ce_pin = 7;
// // const uint16_t csn_pin = 8; // chip select
// // const uint8_t mosi_pin = 11;
// // const uint8_t miso_pin = 10;
// // const uint8_t sck_pin = 9;

// const uint8_t mosi_pin = 11 - 1;
// const uint8_t miso_pin = 10 - 1;
// const uint8_t sck_pin = 9 - 1;
// const uint16_t csn_tx_pin = 8 - 1; // chip select
// const uint16_t ce_tx_pin = 7 - 1;

// const uint16_t csn_rx_pin = 6 - 1; // chip select
// const uint16_t ce_rx_pin = 5 - 1;

// #define SAMP_RATE AUDIO_SR_48K
// #define SAMP_RATE_HZ 48000.f
// #define LUT_N 1024
// float LUT[LUT_N];

// enum sample_types
// {
//     float32 = 0,
//     int32,
//     int16,
//     int8
// };

// // make sure these match
// #define SAMP_TYPE int16_t
// const sample_types samp_type = int16;
// #define SAMPS_PER_PAYLOAD int(32 / sizeof(SAMP_TYPE))

// // instantiate an object for the nRF24L01 transceiver
// // RF24 radio(7, 8); // using pin 7 for the CE pin, and pin 8 for the CSN pin
// RF24 radio_rx(ce_rx_pin, csn_rx_pin, int(10e6)); // using pin 7 for the CE pin, and pin 8 for the CSN pin

// RF24 radio_tx(ce_tx_pin, csn_tx_pin, int(10e6)); // using pin 7 for the CE pin, and pin 8 for the CSN pin

// // Let these addresses be used for the pair
// uint8_t addresses_node[][8] = {"node_tx", "node_rx"}; // node tx, mixer tx
// uint8_t addresses_mixer[][8] = {"mix_tx", "mix_rx"};  // node tx, mixer tx
// // It is very helpful to think of an address as a path instead of as
// // an identifying device destination

// /***
//  * audio pathway
//  * src_tx ->  mixer_rx --- mixer_tx -> src_rx
//  */
// bool mixer = true; // mixer is true, source is false
// // Used to control whether this node is sending or receiving
// // bool role = false; // true = TX role, false = RX role

// // to use different addresses on a pair of radios, we need a variable to
// // uniquely identify which address this radio will use to transmit
// bool radioNumber = mixer; // 0 uses address[0] to transmit, 1 uses address[1] to transmit

// // For this example, we'll be using a payload containing
// // a single float number that will be incremented
// // on every successful transmission
// // float payload[8] = {0};
// const float int16_factor = (1.f / 32767.f);

// #define SAMPS_PER_BUF 64
// #define N_BUFS int(3)
// #define TOTAL_SAMPS SAMPS_PER_BUF *N_BUFS

// float audio_buffer[TOTAL_SAMPS] = {0};
// SAMP_TYPE payload[SAMPS_PER_PAYLOAD] = {0};

// float audio_in_buffer[TOTAL_SAMPS] = {0};
// int audio_in_read_idx = 0;
// int audio_in_write_idx = 0;
// int audio_in_available_samps = 0;

// volatile int read_idx = 0;
// volatile int write_idx = 0;
// volatile int available_samps = 0;
// const bool use_auto_ack = false;
// DaisyHardware hw;

// size_t num_channels;

// float x, t = 0.f, omega = 2.f * 3.14159f * 440.f, dt = 1.f / 16e3;
// int sine_dx = float(LUT_N) / 16e3 * 363.f;
// int sine_idx = 0;
// volatile bool overun = false;
// bool listening_paused = false;

// volatile int callback_state = LOW;
// volatile bool enough_samples = false;
// volatile bool error_weird = false;

// void MyCallback(float **in, float **out, size_t size)
// {
//     if (!mixer) // only the sources should be running this callback (for now...)
//     {

//         // 1. read from the mic and put it in the audio in buffer
//         {
//             for (size_t i = 0; i < size; i++)
//             {
//                 audio_in_buffer[audio_in_write_idx] = 0.5f * (in[0][i] + in[1][i]);

//                 audio_in_write_idx++;

//                 if (audio_in_write_idx >= (TOTAL_SAMPS))
//                 {
//                     audio_in_write_idx = 0;
//                 }
//                 audio_in_available_samps++;
//             }
//         }

//         // 2. write out the radio rx data to the speaker
//         {
//             callback_state = (callback_state == HIGH) ? LOW : HIGH;
//             float amp = 0.9f;
//             // Serial.print("size: ");
//             // Serial.println(size);
//             if (available_samps >= (TOTAL_SAMPS - SAMPS_PER_BUF)) // we want 1 buffer available
//                 enough_samples = true;

//             if (enough_samples && (available_samps >= SAMPS_PER_BUF)) // we want 1 buffer available
//             {
//                 overun = false;

//                 for (size_t i = 0; i < size; i++)
//                 {
//                     for (size_t chn = 0; chn < num_channels; chn++)
//                     {
//                         out[chn][i] = 0.9f * audio_buffer[read_idx];
//                         // out[chn][i] = .1f * audio_buffer[read_idx] + 1.f * in[chn][i];
//                         // out[chn][i] = in[chn][i];
//                     }
//                     read_idx++;

//                     if (read_idx >= (TOTAL_SAMPS))
//                     {
//                         read_idx = 0;
//                     }
//                     available_samps--;
//                 }
//             }
//             else
//             {
//                 enough_samples = false;
//                 // Serial.println("OVERUN :(");
//                 overun = true;
//                 for (size_t i = 0; i < SAMPS_PER_BUF; i++)
//                 {
//                     for (size_t chn = 0; chn < num_channels; chn++)
//                     {
//                         out[chn][i] = .0f;
//                     }
//                 }
//             }
//         }
//     }
// }

// void setup()
// {
//     for (int i = 0; i < LUT_N; i++)
//     {
//         LUT[i] = sin(2.f * 3.14159 * float(i) / float(LUT_N));
//     }

//     // hw = DAISY.init(DAISY_SEED, AUDIO_SR_48K);
//     // DAISY.SetAudioBlockSize(48);
//     hw = DAISY.init(DAISY_SEED, SAMP_RATE);
//     DAISY.SetAudioBlockSize(SAMPS_PER_BUF);
//     // DAISY.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_32KHZ);

//     num_channels = hw.num_channels;
//     // for (int i = 0; i < 8; i++)
//     // {
//     // 	payload[i] = 0;
//     // }
//     const int led = LED_BUILTIN;
//     pinMode(led, OUTPUT);
//     pinMode(1, OUTPUT);
//     pinMode(2, OUTPUT);
//     // digitalWrite(1, HIGH);

//     Serial.begin();
//     // while (!Serial)
//     // {
//     // 	// some boards need to wait to ensure access to serial over USB
//     // }

//     SPI.begin();
//     SPI.setMOSI(mosi_pin);
//     SPI.setMISO(miso_pin);
//     SPI.setSCLK(sck_pin);
//     // initialize the transceiver on the SPI bus
//     // if (!radio_tx.begin())
//     // {
//     // 	Serial.println(F("radio hardware is not responding!!"));
//     // 	while (1)
//     // 	{
//     // 		digitalWrite(led, HIGH);
//     // 		delay(1000);
//     // 		digitalWrite(led, LOW);
//     // 		delay(1000);

//     // 	} // hold in infinite loop
//     // }

//     if (!radio_rx.begin())
//     {
//         Serial.println(F("radio hardware is not responding!!"));
//         while (1)
//         {
//             digitalWrite(led, HIGH);
//             delay(1000);
//             digitalWrite(led, LOW);
//             delay(1000);

//         } // hold in infinite loop
//     }

//     Serial.print(F("radioNumber = "));
//     Serial.println((int)radioNumber);

//     // role variable is hardcoded to RX behavior, inform the user of this
//     Serial.println(F("*** PRESS 'T' to begin transmitting to the other node"));

//     // Set the PA Level low to try preventing power supply related problems
//     // because these examples are likely run with nodes in close proximity to
//     // each other.
//     radio_tx.setPALevel(RF24_PA_MAX, 1); // RF24_PA_MAX is default.
//     radio_rx.setPALevel(RF24_PA_MAX, 1); // RF24_PA_MAX is default.

//     // save on transmission time by setting the radio to only transmit the
//     // number of bytes we need to transmit a float
//     radio_tx.setPayloadSize(32); // float datatype occupies 4 bytes
//     radio_tx.setDataRate(RF24_1MBPS);
//     radio_rx.setPayloadSize(32); // float datatype occupies 4 bytes
//     radio_rx.setDataRate(RF24_1MBPS);

//     // set the TX address of the RX node into the TX pipe
//     /*
//     uint8_t address_mixer_to_node[][4] = {"tx0", "rx0"}; // node tx, mixer tx
//     uint8_t address_node_to_mixer[][4] = {"tx1", "rx1"}; // node rx, mixer rx
//     */
//     if (mixer)
//     {
//         radio_tx.openWritingPipe(addresses_node[1]);     // always uses pipe 0
//         radio_rx.openReadingPipe(1, addresses_mixer[0]); // always uses pipe 0
//     }
//     else // node
//     {
//         radio_tx.openWritingPipe(addresses_mixer[0]);   // mixer rx
//         radio_rx.openReadingPipe(1, addresses_node[1]); // always uses pipe 0
//     }
//     // // set the RX address of the TX node into a RX pipe
//     // radio_tx.openReadingPipe(1, address_tx[!radioNumber]); // using pipe 1
//     // radio_rx.openReadingPipe(1, address_rx[!radioNumber]); // using pipe 1

//     if (use_auto_ack)
//     {
//         // to use ACK payloads, we need to enable dynamic payload lengths (for all nodes)
//         radio_tx.enableDynamicPayloads(); // ACK payloads are dynamically sized
//         radio_rx.enableDynamicPayloads(); // ACK payloads are dynamically sized

//         // Acknowledgement packets have no payloads by default. We need to enable
//         // this feature for all nodes (TX & RX) to use ACK payloads.
//         radio_tx.enableAckPayload();
//         radio_rx.enableAckPayload();
//         radio_tx.setAutoAck(1);
//         radio_rx.setAutoAck(1);
//     }
//     else
//     {
//         radio_tx.setAutoAck(0);
//         radio_rx.setAutoAck(0);
//     }

//     // additional setup specific to the node's role
//     radio_tx.stopListening();
//     radio_rx.startListening();
//     // if (role)
//     // {
//     // 	radio.stopListening(); // put radio in TX mode
//     // }
//     // else
//     // {
//     // 	radio.startListening(); // put radio in RX mode
//     // }

//     // only the sources run the callback
//     if (!mixer)
//     {
//         DAISY.StartAudio(MyCallback);
//     }

// } // setup

// int led_state = HIGH;

// int last_read_us = micros();
// const int delay_time_us = float(32) / float(sizeof(SAMP_TYPE)) / SAMP_RATE_HZ * 1e6;
// int payload_state = LOW;

// void loop()
// {
//     // digitalWrite(2, callback_state);
//     // digitalWrite(1, payload_state);

//     int t0 = micros();
//     // digitalWrite(LED_BUILTIN, (overun) ? HIGH : LOW);
//     // digitalWrite(LED_BUILTIN, (available_samps <= SAMPS_PER_BUF) ? HIGH : LOW);
//     // digitalWrite(LED_BUILTIN, (error_weird) ? HIGH : LOW);
//     // digitalWrite(1, (overun) ? HIGH : LOW);
//     // digitalWrite(LED_BUILTIN, (available_samps >= TOTAL_SAMPS) ? HIGH : LOW);

//     if (mixer)
//     {
//         uint8_t pipe;
//         // if mixer, transmit back what we received
//         if (radio_rx.available(&pipe))
//         { // is there a payload? get the pipe number that recieved it

//             led_state = (led_state == HIGH) ? LOW : HIGH;
//             uint8_t bytes = radio_rx.getPayloadSize(); // get the size of the payload
//                                                        // radio.read(&audio_buffer[write_idx], 32); // fetch payload from FIFO

//             digitalWrite(LED_BUILTIN, HIGH);
//             payload_state = (payload_state == HIGH) ? LOW : HIGH;
//             radio_rx.read(&payload, 32); // fetch payload from FIFO

//             // send it back
//             bool report = radio_tx.writeFast(&payload, 32); // transmit & save the report
//         }
//         else
//         {
//             digitalWrite(LED_BUILTIN, LOW);
//         }
//     }
//     else
//     {
//         if (audio_in_available_samps >= SAMPS_PER_PAYLOAD)
//         {
//             // This device is a TX node
//             for (int i = 0; i < SAMPS_PER_PAYLOAD; i++)
//             {
//                 switch (samp_type)
//                 {
//                 case float32:
//                     payload[i] = audio_in_buffer[audio_in_read_idx];
//                     break;
//                 case int32:
//                     payload[i] = int32_t(audio_in_buffer[audio_in_read_idx] * float(INT32_MAX));
//                     break;
//                 case int16:
//                     payload[i] = int16_t(audio_in_buffer[audio_in_read_idx] * float(INT16_MAX));
//                     break;
//                 case int8:
//                     payload[i] = int8_t(audio_in_buffer[audio_in_read_idx] * float(INT8_MAX));
//                     break;
//                 }
//                 audio_in_read_idx++;
//                 if (audio_in_read_idx >= (TOTAL_SAMPS))
//                     audio_in_read_idx = 0;
//             }
//             audio_in_available_samps -= SAMPS_PER_PAYLOAD;

//             // unsigned long start_timer = micros();						// start the timer
//             bool report = radio_tx.writeFast(&payload, 32); // transmit & save the report
//                                                             // unsigned long end_timer = micros();							// end the timer

//             // if (report)
//             // {
//             // 	digitalWrite(2, led_state);
//             // 	led_state = (led_state == HIGH) ? LOW : HIGH;
//             // }
//         }

//         // read in sample from radio and put them on the audio out buffer

//         // This device is a RX node
//         uint8_t pipe;

//         if (radio_rx.available(&pipe))
//         { // is there a payload? get the pipe number that recieved it

//             led_state = (led_state == HIGH) ? LOW : HIGH;
//             uint8_t bytes = radio_rx.getPayloadSize(); // get the size of the payload
//                                                        // radio.read(&audio_buffer[write_idx], 32); // fetch payload from FIFO

//             digitalWrite(LED_BUILTIN, HIGH);
//             payload_state = (payload_state == HIGH) ? LOW : HIGH;
//             radio_rx.read(&payload, 32); // fetch payload from FIFO
//             // digitalWrite(1, LOW);
//             float amp = 0.7f;
//             for (int i = 0; i < SAMPS_PER_PAYLOAD; i++)
//             {
//                 switch (samp_type)
//                 {
//                 case float32:
//                     audio_buffer[write_idx] = amp * payload[i];
//                     break;
//                 case int32:
//                     audio_buffer[write_idx] = amp * float(payload[i]) / float(INT32_MAX);
//                     break;
//                 case int16:
//                     audio_buffer[write_idx] = amp * float(payload[i]) / float(INT16_MAX);
//                     break;
//                 case int8:
//                     audio_buffer[write_idx] = amp * float(payload[i]) / float(INT8_MAX);
//                     break;
//                 }
//                 write_idx++;
//                 if (write_idx >= (TOTAL_SAMPS))
//                     write_idx = 0;
//             }
//             available_samps += SAMPS_PER_PAYLOAD;
//         }
//         else
//         {
//             digitalWrite(LED_BUILTIN, LOW);
//         }
//     }

// } // loop
