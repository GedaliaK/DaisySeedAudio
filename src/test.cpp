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
// RF24 radio(ce_tx_pin, csn_tx_pin, int(10e6)); // using pin 7 for the CE pin, and pin 8 for the CSN pin

// // Let these addresses be used for the pair
// uint8_t address[][6] = {"1Node", "2Node"};
// // It is very helpful to think of an address as a path instead of as
// // an identifying device destination

// // to use different addresses on a pair of radios, we need a variable to
// // uniquely identify which address this radio will use to transmit
// bool radioNumber = 1; // 0 uses address[0] to transmit, 1 uses address[1] to transmit

// // Used to control whether this node is sending or receiving
// bool role = false; // true = TX role, false = RX role

// // For this example, we'll be using a payload containing
// // a single float number that will be incremented
// // on every successful transmission
// float payload = 0.0;

// void setup()
// {

//     Serial.begin(115200);
//     // while (!Serial)
//     // {
//     //     // some boards need to wait to ensure access to serial over USB
//     // }

//     // initialize the transceiver on the SPI bus
//     pinMode(LED_BUILTIN, OUTPUT);
//     int led_state = LOW;
//     if (!radio.begin())
//     {
//         Serial.println(F("radio hardware is not responding!!"));
//         while (1)
//         {
//             digitalWrite(LED_BUILTIN, led_state);
//             led_state = (led_state == HIGH) ? LOW : HIGH;
//             delay(1000);

//         } // hold in infinite loop
//     }

//     // print example's introductory prompt
//     Serial.println(F("RF24/examples/GettingStarted"));

//     // To set the radioNumber via the Serial monitor on startup
//     Serial.println(F("Which radio is this? Enter '0' or '1'. Defaults to '0'"));
//     // while (!Serial.available())
//     // {
//     //     // wait for user input
//     // }
//     // char input = Serial.parseInt();
//     // radioNumber = input == 1;
//     // Serial.print(F("radioNumber = "));
//     // Serial.println((int)radioNumber);

//     // // role variable is hardcoded to RX behavior, inform the user of this
//     // Serial.println(F("*** PRESS 'T' to begin transmitting to the other node"));

//     // Set the PA Level low to try preventing power supply related problems
//     // because these examples are likely run with nodes in close proximity to
//     // each other.
//     radio.setPALevel(RF24_PA_LOW); // RF24_PA_MAX is default.
//     radio.setAutoAck(0);
//     // save on transmission time by setting the radio to only transmit the
//     // number of bytes we need to transmit a float
//     radio.setPayloadSize(sizeof(payload)); // float datatype occupies 4 bytes

//     // set the TX address of the RX node into the TX pipe
//     radio.openWritingPipe(address[radioNumber]); // always uses pipe 0

//     // set the RX address of the TX node into a RX pipe
//     radio.openReadingPipe(1, address[!radioNumber]); // using pipe 1

//     // additional setup specific to the node's role
//     if (role)
//     {
//         radio.stopListening(); // put radio in TX mode
//     }
//     else
//     {
//         radio.startListening(); // put radio in RX mode
//     }

//     digitalWrite(LED_BUILTIN, HIGH);
//     delay(1000);
//     digitalWrite(LED_BUILTIN, LOW);
//     // For debugging info
//     // printf_begin();             // needed only once for printing details
//     // radio.printDetails();       // (smaller) function that prints raw register values
//     // radio.printPrettyDetails(); // (larger) function that prints human readable data

// } // setup

// int led_state = HIGH;
// void loop()
// {

//     if (role)
//     {
//         // This device is a TX node

//         unsigned long start_timer = micros();               // start the timer
//         bool report = radio.write(&payload, sizeof(float)); // transmit & save the report
//         unsigned long end_timer = micros();                 // end the timer

//         if (report)
//         {
//             Serial.print(F("Transmission successful! ")); // payload was delivered
//             Serial.print(F("Time to transmit = "));
//             Serial.print(end_timer - start_timer); // print the timer result
//             Serial.print(F(" us. Sent: "));
//             Serial.println(payload); // print payload sent
//             payload += 0.01;         // increment float payload
//             digitalWrite(LEaD_BUILTIN, led_state);
//             led_state = (led_state == HIGH) ? LOW : HIGH;
//         }
//         else
//         {
//             Serial.println(F("Transmission failed or timed out")); // payload was not delivered
//         }

//         // to make this example readable in the serial monitor
//         delay(1000); // slow transmissions down by 1 second
//     }
//     else
//     {
//         // This device is a RX node

//         uint8_t pipe;
//         if (radio.available(&pipe))
//         {                                           // is there a payload? get the pipe number that recieved it
//             uint8_t bytes = radio.getPayloadSize(); // get the size of the payload
//             radio.read(&payload, bytes);            // fetch payload from FIFO
//             Serial.print(F("Received "));
//             Serial.print(bytes); // print the size of the payload
//             Serial.print(F(" bytes on pipe "));
//             Serial.print(pipe); // print the pipe number
//             Serial.print(F(": "));
//             Serial.println(payload); // print the payload's value
//             digitalWrite(LED_BUILTIN, led_state);
//             led_state = (led_state == HIGH) ? LOW : HIGH;
//         }
//         else
//         {
//             // digitalWrite(LED_BUILTIN, LOW);
//         }
//     } // role

//     // if (Serial.available())
//     // {
//     //     // change the role via the serial monitor

//     //     char c = toupper(Serial.read());
//     //     if (c == 'T' && !role)
//     //     {
//     //         // Become the TX node

//     //         role = true;
//     //         Serial.println(F("*** CHANGING TO TRANSMIT ROLE -- PRESS 'R' TO SWITCH BACK"));
//     //         radio.stopListening();
//     //     }
//     //     else if (c == 'R' && role)
//     //     {
//     //         // Become the RX node

//     //         role = false;
//     //         Serial.println(F("*** CHANGING TO RECEIVE ROLE -- PRESS 'T' TO SWITCH BACK"));
//     //         radio.startListening();
//     //     }
//     // }

// } // loop
