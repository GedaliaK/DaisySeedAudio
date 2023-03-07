// #include <stdio.h>
// #include <iostream>
// #include "C:\\dev\\vcpkg\\vcpkg\\installed\\x64-windows\\include\\SDL2\\SDL2.h"
// // #include "SDL2/SDL.h"
// #include <vector>
// #include <fstream>
// #include <cassert>
// #include <algorithm>
// #include <mutex>

// std::mutex g_audio_mutex;
// const float PI = 3.14159f;
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
// const uint16_t csn_pin = 8 - 1; // chip select
// const uint16_t ce_pin = 7 - 1;

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
// RF24 radio(ce_pin, csn_pin, int(10e6)); // using pin 7 for the CE pin, and pin 8 for the CSN pin

// // Let these addresses be used for the pair
// uint8_t address[][6] = {"1Node", "2Node"};
// // It is very helpful to think of an address as a path instead of as
// // an identifying device destination

// // Used to control whether this node is sending or receiving
// bool role = false; // true = TX role, false = RX role

// // to use different addresses on a pair of radios, we need a variable to
// // uniquely identify which address this radio will use to transmit
// bool radioNumber = !role; // 0 uses address[0] to transmit, 1 uses address[1] to transmit

// // For this example, we'll be using a payload containing
// // a single float number that will be incremented
// // on every successful transmission
// // float payload[8] = {0};
// const float int16_factor = (1.f / 32767.f);

// #define SAMPS_PER_BUF 64
// #define N_BUFS int(10)
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

// void main_setup()
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
//     if (!radio.begin())
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

//     // print example's introductory prompt
//     Serial.println(F("RF24/examples/GettingStarted"));

//     // To set the radioNumber via the Serial monitor on startup
//     Serial.println(F("Which radio is this? Enter '0' or '1'. Defaults to '0'"));
//     // while (!Serial.available())
//     // {
//     // 	// wait for user input
//     // }
//     // char input = Serial.parseInt();
//     // radioNumber = input == 1;
//     Serial.print(F("radioNumber = "));
//     Serial.println((int)radioNumber);

//     // role variable is hardcoded to RX behavior, inform the user of this
//     Serial.println(F("*** PRESS 'T' to begin transmitting to the other node"));

//     // Set the PA Level low to try preventing power supply related problems
//     // because these examples are likely run with nodes in close proximity to
//     // each other.
//     radio.setPALevel(RF24_PA_MAX, 1); // RF24_PA_MAX is default.

//     // save on transmission time by setting the radio to only transmit the
//     // number of bytes we need to transmit a float
//     radio.setPayloadSize(32); // float datatype occupies 4 bytes
//     radio.setDataRate(RF24_1MBPS);

//     // set the TX address of the RX node into the TX pipe
//     radio.openWritingPipe(address[radioNumber]); // always uses pipe 0

//     // set the RX address of the TX node into a RX pipe
//     radio.openReadingPipe(1, address[!radioNumber]); // using pipe 1

//     if (use_auto_ack)
//     {
//         // to use ACK payloads, we need to enable dynamic payload lengths (for all nodes)
//         radio.enableDynamicPayloads(); // ACK payloads are dynamically sized

//         // Acknowledgement packets have no payloads by default. We need to enable
//         // this feature for all nodes (TX & RX) to use ACK payloads.
//         radio.enableAckPayload();
//         radio.setAutoAck(1);
//     }
//     else
//     {
//         radio.setAutoAck(0);
//     }

//     // additional setup specific to the node's role
//     if (role)
//     {
//         radio.stopListening(); // put radio in TX mode
//     }
//     else
//     {
//         radio.startListening(); // put radio in RX mode
//     }
//     // for (int i = 0; i < 3; i++)
//     // {
//     // 	digitalWrite(led, HIGH);
//     // 	delay(1000);
//     // 	digitalWrite(led, LOW);
//     // 	delay(1000);
//     // }

//     // DAISY.StartAudio(MyCallback);
//     // For debugging info
//     // printf_begin();             // needed only once for printing details
//     // radio.printDetails();       // (smaller) function that prints raw register values
//     // radio.printPrettyDetails(); // (larger) function that prints human readable data

// } // setup

// int led_state = HIGH;

// int last_read_us = micros();
// const int delay_time_us = float(32) / float(sizeof(SAMP_TYPE)) / SAMP_RATE_HZ * 1e6;
// int payload_state = LOW;

// void main_loop()
// {
//     // digitalWrite(2, callback_state);
//     // digitalWrite(1, payload_state);

//     int t0 = micros();
//     // digitalWrite(LED_BUILTIN, (overun) ? HIGH : LOW);
//     // digitalWrite(LED_BUILTIN, (available_samps <= SAMPS_PER_BUF) ? HIGH : LOW);
//     // digitalWrite(LED_BUILTIN, (error_weird) ? HIGH : LOW);
//     // digitalWrite(1, (overun) ? HIGH : LOW);
//     // digitalWrite(LED_BUILTIN, (available_samps >= TOTAL_SAMPS) ? HIGH : LOW);

//     if (role && (audio_in_available_samps >= SAMPS_PER_PAYLOAD))
//     {
//         // This device is a TX node
//         for (int i = 0; i < SAMPS_PER_PAYLOAD; i++)
//         {
//             switch (samp_type)
//             {
//             case float32:
//                 payload[i] = audio_in_buffer[audio_in_read_idx];
//                 break;
//             case int32:
//                 payload[i] = int32_t(audio_in_buffer[audio_in_read_idx] * float(INT32_MAX));
//                 break;
//             case int16:
//                 payload[i] = int16_t(audio_in_buffer[audio_in_read_idx] * float(INT16_MAX));
//                 break;
//             case int8:
//                 payload[i] = int8_t(audio_in_buffer[audio_in_read_idx] * float(INT8_MAX));
//                 break;
//             }
//             audio_in_read_idx++;
//             if (audio_in_read_idx >= (TOTAL_SAMPS))
//                 audio_in_read_idx = 0;
//         }
//         audio_in_available_samps -= SAMPS_PER_PAYLOAD;

//         // unsigned long start_timer = micros();						// start the timer
//         bool report = radio.writeFast(&payload, 32); // transmit & save the report
//                                                      // unsigned long end_timer = micros();							// end the timer
//         if (report)
//         {
//             digitalWrite(2, led_state);
//             led_state = (led_state == HIGH) ? LOW : HIGH;
//         }
//         // if (report)
//         // {
//         // 	Serial.print(F("Transmission successful! ")); // payload was delivered
//         // 	// Serial.print(F("Time to transmit = "));
//         // 	// Serial.print(end_timer - start_timer); // print the timer result
//         // 	Serial.print(F(" us. Sent: "));
//         // 	// for (int i = 0; i < 8; i++)
//         // 	// {
//         // 	// 	payload[i] += 0.01; // increment float payload
//         // 	// }
//         // 	Serial.println();
//         // }
//         // else
//         // {
//         // 	Serial.println(F("Transmission failed or timed out")); // payload was not delivered
//         // }

//         // to make this example readable in the serial monitor
//         // delay(1000); // slow transmissions down by 1 second
//     }
//     else
//     {
//         // This device is a RX node
//         uint8_t pipe;

//         if (radio.available(&pipe))
//         { // is there a payload? get the pipe number that recieved it
//             digitalWrite(2, led_state);
//             // if ((t0 - last_read_us) > delay_time_us)
//             // {
//             // 	digitalWrite(LED_BUILTIN, HIGH);
//             // }
//             // else
//             // {
//             // 	digitalWrite(LED_BUILTIN, LOW);
//             // }
//             last_read_us = t0;

//             led_state = (led_state == HIGH) ? LOW : HIGH;
//             uint8_t bytes = radio.getPayloadSize(); // get the size of the payload
//                                                     // radio.read(&audio_buffer[write_idx], 32); // fetch payload from FIFO

//             // digitalWrite(1, HIGH);
//             payload_state = (payload_state == HIGH) ? LOW : HIGH;
//             radio.read(&payload, 32); // fetch payload from FIFO
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

//             // memcpy(&audio_buffer[write_idx], payload, 32);
//             // for (int k = 0; k < 8; k++)
//             // {
//             // 	Serial.print(audio_buffer[write_idx + k]);
//             // 	Serial.print(" ");
//             // }
//             // Serial.println();

//             // write_idx += SAMPS_PER_PAYLOAD;
//             // available_samps += SAMPS_PER_PAYLOAD;

//             // if (available_samps >= TOTAL_SAMPS)
//             // {
//             // 	radio.stopListening();
//             // 	listening_paused = true;
//             // }
//             // else if (listening_paused)
//             // {
//             // 	listening_paused = false;
//             // 	radio.startListening();
//             // }
//             // if (write_idx >= (TOTAL_SAMPS))
//             // 	write_idx = 0;

//             // radio.read(&payload, bytes); // fetch payload from FIFO
//             // // write_idx += 8;
//             // // if (write_idx >= AUDIO_BUF)
//             // // 	write_idx = 0;
//             // Serial.println("rx!");
//         }
//     }
//     // if (Serial.available())
//     // {
//     // 	// change the role via the serial monitor

//     // 	char c = toupper(Serial.read());
//     // 	if (c == 'T' && !role)
//     // 	{
//     // 		// Become the TX node

//     // 		role = true;
//     // 		Serial.println(F("*** CHANGING TO TRANSMIT ROLE -- PRESS 'R' TO SWITCH BACK"));
//     // 		radio.stopListening();
//     // 	}
//     // 	else if (c == 'R' && role)
//     // 	{
//     // 		// Become the RX node

//     // 		role = false;
//     // 		Serial.println(F("*** CHANGING TO RECEIVE ROLE -- PRESS 'T' TO SWITCH BACK"));
//     // 		radio.startListening();
//     // 	}
//     // }

// } // loop

// void audio_out_callback(void *userdata, Uint8 *stream, int len);
// // void audio_in_callback(void* userdata, Uint8* stream, int len);

// class AudioPlayer
// {
// public:
//     // the representation of our audio device in SDL:
//     SDL_AudioDeviceID audio_device_input, audio_device_output;
//     // opening an audio device:
//     SDL_AudioSpec audio_spec_input, audio_spec_output;
//     std::vector<float> mic_data;

//     SDL_Window *window = NULL;
//     SDL_Renderer *renderer = NULL;
//     SDL_Surface *screenSurface = NULL;
//     float dt;
//     float t = 0.0;

//     const int screenWidth = 640, screenHeight = 480;
//     std::vector<SDL_Point> points;
//     std::vector<float> soundSig, tempSoundSig;

//     AudioPlayer(float freq = 44100.f, int samples_per_buffer = 1024)
//     {

//         mic_data.resize(samples_per_buffer, 0);
//         dt = 1.0f / freq;
//         SDL_Init(SDL_INIT_AUDIO);

//         int n = 100;
//         std::vector<float> x(n);

//         std::vector<float> y(n, 0);

//         /*    SDL_zero(audio_spec_input);

//             audio_spec_input.freq = freq;
//             audio_spec_input.format = AUDIO_F32LSB;
//             audio_spec_input.channels = 1;
//             audio_spec_input.samples = samples_per_buffer;
//             audio_spec_input.callback = audio_in_callback;
//             audio_spec_input.userdata = (void*)this;

//             audio_device_input = SDL_OpenAudioDevice(
//                 NULL, true, &audio_spec_input, NULL, 0);*/

//         SDL_zero(audio_spec_output);

//         audio_spec_output.freq = freq;
//         audio_spec_output.format = AUDIO_F32LSB;
//         audio_spec_output.channels = 1;
//         audio_spec_output.samples = samples_per_buffer;
//         audio_spec_output.callback = audio_out_callback;
//         audio_spec_output.userdata = (void *)this;

//         audio_device_output = SDL_OpenAudioDevice(
//             NULL, false, &audio_spec_output, NULL, 0);

//         if (SDL_Init(SDL_INIT_VIDEO) < 0)
//         {
//             fprintf(stderr, "could not initialize sdl2: %s\n", SDL_GetError());
//         }

//         if (SDL_CreateWindowAndRenderer(screenWidth, screenHeight, 0, &window, &renderer) < 0)
//         {
//             fprintf(stderr, "could not initialize window and renderer: %s\n", SDL_GetError());
//         }

//         // SDL_PauseAudioDevice(audio_device_input, 0);
//         SDL_PauseAudioDevice(audio_device_output, 0);
//     }

//     void loop()
//     {
//         bool quit = false;
//         unsigned int lastTime = SDL_GetTicks(), currentTime;
//         float FPS = 1000.0 / 120.;
//         while (!quit)
//         {
//             main_loop();
//             SDL_Event event;
//             while (SDL_PollEvent(&event))
//             {
//                 switch (event.type)
//                 {
//                 case SDL_QUIT:
//                     printf("user requested quit\n");
//                     quit = true;
//                     break;
//                 }
//             }
//             currentTime = SDL_GetTicks();
//             if (float((currentTime - lastTime)) > FPS)
//             {
//                 lastTime = currentTime;
//                 /*float dw = float(screenWidth) * 0.75 / float(springs.positions_x.size());
//                 for (int i = 0; i < springs.positions_x.size(); i++) {
//                     points[i].y = float(screenHeight / 4) - 20.0 * (springs.positions_x[i] - float(i) * eq_d);
//                     points[i].x = float(screenWidth) * .125 + float(i) * dw;

//                 }
//                 float scale = 2.0;
//                 SDL_SetRenderDrawColor(renderer, 0, 0, 0, SDL_ALPHA_OPAQUE);
//                 SDL_RenderClear(renderer);
//                 SDL_RenderSetScale(renderer, 1.0, scale);
//                 SDL_SetRenderDrawColor(renderer, 255, 255, 255, SDL_ALPHA_OPAQUE);
//                 SDL_RenderDrawLines(renderer, &points[0], springs.positions_x.size());
//                 SDL_RenderPresent(renderer);*/
//             }
//         }
//     }

//     ~AudioPlayer()
//     {
//         SDL_DestroyWindow(window);
//         SDL_CloseAudioDevice(audio_device_output);
//         // SDL_CloseAudioDevice(audio_device_input);

//         SDL_Quit();
//     }
// };

// void audio_out_callback(void *userdata, Uint8 *stream, int len)
// {
//     AudioPlayer *player = (AudioPlayer *)userdata;
//     float *buffer = reinterpret_cast<float *>(stream);
//     const int size = len / 4;
//     std::lock_guard<std::mutex> guard(g_audio_mutex);
//     // std::copy(player->mic_data.begin(), player->mic_data.end(), buffer);

//     if (role) // if tx node, copy to output buffer for nrf24
//     {
//         for (size_t i = 0; i < size; i++)
//         {
//             audio_in_buffer[audio_in_write_idx] = 0.5f * (in[0][i] + in[1][i]);

//             audio_in_write_idx++;

//             if (audio_in_write_idx >= (TOTAL_SAMPS))
//             {
//                 audio_in_write_idx = 0;
//             }
//             audio_in_available_samps++;
//         }
//     }
//     else
//     {

//         if (size != SAMPS_PER_BUF)
//         {
//             error_weird = true;
//         }
//         else
//         {
//             error_weird = false;
//         }
//         callback_state = (callback_state == HIGH) ? LOW : HIGH;
//         float amp = 0.9f;
//         // Serial.print("size: ");
//         // Serial.println(size);
//         if (available_samps >= (TOTAL_SAMPS / 2)) // we want 1 buffer available
//             enough_samples = true;

//         if (enough_samples && (available_samps >= SAMPS_PER_BUF)) // we want 1 buffer available
//         {
//             overun = false;

//             for (size_t i = 0; i < size; i++)
//             {
//                 buffer[i] = 0.9f * audio_buffer[read_idx];

//                 read_idx++;

//                 if (read_idx >= (TOTAL_SAMPS))
//                 {
//                     read_idx = 0;
//                 }
//                 available_samps--;
//             }
//         }
//         else
//         {
//             enough_samples = false;
//             // Serial.println("OVERUN :(");
//             overun = true;
//             for (size_t i = 0; i < SAMPS_PER_BUF; i++)
//             {
//                 for (size_t chn = 0; chn < num_channels; chn++)
//                 {
//                     out[chn][i] = .0f;
//                 }
//             }
//         }
//     }
//     for (int i = 0; i < len / 4; i++)
//     {
//         buffer[i] = sin(2.f * PI * 440.f * player->t);
//         player->t += player->dt;
//     }
// }

// int main(int argc, char **argv)
// {
//     AudioPlayer player(44100, 2048);
//     player.loop();
//     return 0;
// }