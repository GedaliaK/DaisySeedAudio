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

#define SAMP_RATE AUDIO_SR_16K
#define SAMP_RATE_HZ 16000.f
#define LUT_N 1024
float LUT[LUT_N];

enum sample_types
{
	float32 = 0,
	int32,
	int16,
	int8
};

// make sure these match
#define SAMP_TYPE int16_t
const sample_types samp_type = int16;
#define SAMPS_PER_PAYLOAD int(32 / sizeof(SAMP_TYPE))

// instantiate an object for the nRF24L01 transceiver
// RF24 radio(7, 8); // using pin 7 for the CE pin, and pin 8 for the CSN pin
RF24 radio(ce_pin, csn_pin, int(10e6)); // using pin 7 for the CE pin, and pin 8 for the CSN pin

// Let these addresses be used for the pair
uint8_t address[][6] = {"1Node", "2Node"};
// It is very helpful to think of an address as a path instead of as
// an identifying device destination

// to use different addresses on a pair of radios, we need a variable to
// uniquely identify which address this radio will use to transmit
bool radioNumber = 1; // 0 uses address[0] to transmit, 1 uses address[1] to transmit

// Used to control whether this node is sending or receiving
bool role = false; // true = TX role, false = RX role
// For this example, we'll be using a payload containing
// a single float number that will be incremented
// on every successful transmission
// float payload[8] = {0};
const float int16_factor = (1.f / 32767.f);

#define SAMPS_PER_BUF 64
#define N_BUFS int(10)
#define TOTAL_SAMPS SAMPS_PER_BUF *N_BUFS

float audio_buffer[TOTAL_SAMPS] = {0};
SAMP_TYPE payload[SAMPS_PER_PAYLOAD] = {0};

volatile int read_idx = 0;
volatile int write_idx = 0;
volatile int available_samps = 0;
const bool use_auto_ack = false;
DaisyHardware hw;

size_t num_channels;

float x, t = 0.f, omega = 2.f * 3.14159f * 440.f, dt = 1.f / 16e3;
int sine_dx = float(LUT_N) / 16e3 * 363.f;
int sine_idx = 0;
volatile bool overun = false;
bool listening_paused = false;

volatile int callback_state = LOW;
volatile bool enough_samples = false;
volatile bool error_weird = false;

void MyCallback(float **in, float **out, size_t size)
{
	if (size != SAMPS_PER_BUF)
	{
		error_weird = true;
	}
	else
	{
		error_weird = false;
	}
	callback_state = (callback_state == HIGH) ? LOW : HIGH;
	float amp = 0.9f;
	// Serial.print("size: ");
	// Serial.println(size);
	if (available_samps >= (TOTAL_SAMPS / 2)) // we want 1 buffer available
		enough_samples = true;

	if (enough_samples && (available_samps >= SAMPS_PER_BUF)) // we want 1 buffer available
	{
		overun = false;

		for (size_t i = 0; i < size; i++)
		{
			for (size_t chn = 0; chn < num_channels; chn++)
			{
				// out[chn][i] = amp * audio_buffer[read_idx];
				out[chn][i] = .1f * audio_buffer[read_idx] + 1.f * in[chn][i];
				// out[chn][i] = in[chn][i];
			}
			read_idx++;

			if (read_idx >= (TOTAL_SAMPS))
			{
				read_idx = 0;
			}
			available_samps--;
		}

		// switch (samp_type)
		// {
		// case float32:
		// 	for (size_t i = 0; i < SAMPS_PER_BUF; i++)
		// 	{
		// 		// x = 0.75f * LUT[sine_idx]; // increment float payload
		// 		// sine_idx += sine_dx;
		// 		// if (sine_idx >= LUT_N)
		// 		// 	sine_idx %= LUT_N;

		// 		for (size_t chn = 0; chn < num_channels; chn++)
		// 		{
		// 			// out[chn][i] = x;
		// 			// out[chn][i] = audio_buffer[read_idx];

		// 			// out[chn][i] = payload[idx];
		// 			out[chn][i] = amp * audio_buffer[read_idx];
		// 		}
		// 		read_idx++;

		// 		if (read_idx >= (TOTAL_SAMPS))
		// 			read_idx = 0;
		// 		available_samps--;
		// 	}
		// 	break;
		// case int32:
		// 	for (size_t i = 0; i < SAMPS_PER_BUF; i++)
		// 	{
		// 		// x = 0.75f * LUT[sine_idx]; // increment float payload
		// 		// sine_idx += sine_dx;
		// 		// if (sine_idx >= LUT_N)
		// 		// 	sine_idx %= LUT_N;

		// 		for (size_t chn = 0; chn < num_channels; chn++)
		// 		{
		// 			// out[chn][i] = x;
		// 			// out[chn][i] = audio_buffer[read_idx];

		// 			// out[chn][i] = payload[idx];
		// 			out[chn][i] = amp * float(audio_buffer[read_idx]) * (1.f / 2147483647.f);
		// 		}
		// 		read_idx++;

		// 		if (read_idx >= (TOTAL_SAMPS))
		// 			read_idx = 0;
		// 		available_samps--;
		// 	}
		// 	break;
		// case int16:
		// 	for (size_t i = 0; i < SAMPS_PER_BUF; i++)
		// 	{
		// 		// x = 0.75f * LUT[sine_idx]; // increment float payload
		// 		// sine_idx += sine_dx;
		// 		// if (sine_idx >= LUT_N)
		// 		// 	sine_idx %= LUT_N;

		// 		for (size_t chn = 0; chn < num_channels; chn++)
		// 		{
		// 			// out[chn][i] = x;
		// 			// out[chn][i] = audio_buffer[read_idx];

		// 			// out[chn][i] = payload[idx];
		// 			out[chn][i] = amp * float(audio_buffer[read_idx]) * int16_factor;
		// 			// out[chn][i] = amp * float(payload[i]) * int16_factor;
		// 		}
		// 		read_idx++;

		// 		if (read_idx >= (TOTAL_SAMPS))
		// 			read_idx = 0;
		// 		available_samps--;
		// 	}
		// 	break;

		// case int8:
		// 	for (size_t i = 0; i < SAMPS_PER_BUF; i++)
		// 	{
		// 		// x = 0.75f * LUT[sine_idx]; // increment float payload
		// 		// sine_idx += sine_dx;
		// 		// if (sine_idx >= LUT_N)
		// 		// 	sine_idx %= LUT_N;

		// 		for (size_t chn = 0; chn < num_channels; chn++)
		// 		{
		// 			// out[chn][i] = x;
		// 			// out[chn][i] = audio_buffer[read_idx];

		// 			// out[chn][i] = payload[idx];
		// 			out[chn][i] = amp * float(audio_buffer[read_idx]) * (1.f / 127.f);
		// 		}
		// 		read_idx++;

		// 		if (read_idx >= (TOTAL_SAMPS))
		// 			read_idx = 0;
		// 		available_samps--;
		// 	}
		// 	break;
		// }
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
	pinMode(1, OUTPUT);
	pinMode(2, OUTPUT);
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
	if (!radio.begin())
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

	// print example's introductory prompt
	Serial.println(F("RF24/examples/GettingStarted"));

	// To set the radioNumber via the Serial monitor on startup
	Serial.println(F("Which radio is this? Enter '0' or '1'. Defaults to '0'"));
	// while (!Serial.available())
	// {
	// 	// wait for user input
	// }
	// char input = Serial.parseInt();
	// radioNumber = input == 1;
	Serial.print(F("radioNumber = "));
	Serial.println((int)radioNumber);

	// role variable is hardcoded to RX behavior, inform the user of this
	Serial.println(F("*** PRESS 'T' to begin transmitting to the other node"));

	// Set the PA Level low to try preventing power supply related problems
	// because these examples are likely run with nodes in close proximity to
	// each other.
	radio.setPALevel(RF24_PA_MAX, 1); // RF24_PA_MAX is default.

	// save on transmission time by setting the radio to only transmit the
	// number of bytes we need to transmit a float
	radio.setPayloadSize(32); // float datatype occupies 4 bytes
	radio.setDataRate(RF24_1MBPS);

	// set the TX address of the RX node into the TX pipe
	radio.openWritingPipe(address[radioNumber]); // always uses pipe 0

	// set the RX address of the TX node into a RX pipe
	radio.openReadingPipe(1, address[!radioNumber]); // using pipe 1

	if (use_auto_ack)
	{
		// to use ACK payloads, we need to enable dynamic payload lengths (for all nodes)
		radio.enableDynamicPayloads(); // ACK payloads are dynamically sized

		// Acknowledgement packets have no payloads by default. We need to enable
		// this feature for all nodes (TX & RX) to use ACK payloads.
		radio.enableAckPayload();
		radio.setAutoAck(1);
	}
	else
	{
		radio.setAutoAck(0);
	}

	// additional setup specific to the node's role
	if (role)
	{
		radio.stopListening(); // put radio in TX mode
	}
	else
	{
		radio.startListening(); // put radio in RX mode
	}
	// for (int i = 0; i < 3; i++)
	// {
	// 	digitalWrite(led, HIGH);
	// 	delay(1000);
	// 	digitalWrite(led, LOW);
	// 	delay(1000);
	// }

	DAISY.StartAudio(MyCallback);
	// For debugging info
	// printf_begin();             // needed only once for printing details
	// radio.printDetails();       // (smaller) function that prints raw register values
	// radio.printPrettyDetails(); // (larger) function that prints human readable data

} // setup

int led_state = LOW;

int last_read_us = micros();
const int delay_time_us = float(32) / float(sizeof(SAMP_TYPE)) / SAMP_RATE_HZ * 1e6;
int payload_state = LOW;

void loop()
{
	digitalWrite(2, callback_state);
	digitalWrite(1, payload_state);

	int t0 = micros();
	// digitalWrite(LED_BUILTIN, (overun) ? HIGH : LOW);
	// digitalWrite(LED_BUILTIN, (available_samps <= SAMPS_PER_BUF) ? HIGH : LOW);
	digitalWrite(LED_BUILTIN, (error_weird) ? HIGH : LOW);
	// digitalWrite(1, (overun) ? HIGH : LOW);
	// digitalWrite(LED_BUILTIN, (available_samps >= TOTAL_SAMPS) ? HIGH : LOW);

	if (role)
	{
		// This device is a TX node

		// unsigned long start_timer = micros();						// start the timer
		bool report = radio.writeFast(&payload, 32); // transmit & save the report
		// unsigned long end_timer = micros();							// end the timer

		if (report)
		{
			Serial.print(F("Transmission successful! ")); // payload was delivered
			// Serial.print(F("Time to transmit = "));
			// Serial.print(end_timer - start_timer); // print the timer result
			Serial.print(F(" us. Sent: "));
			// for (int i = 0; i < 8; i++)
			// {
			// 	payload[i] += 0.01; // increment float payload
			// }
			Serial.println();
		}
		else
		{
			Serial.println(F("Transmission failed or timed out")); // payload was not delivered
		}

		// to make this example readable in the serial monitor
		// delay(1000); // slow transmissions down by 1 second
	}
	else
	{
		// This device is a RX node
		uint8_t pipe;

		if (radio.available(&pipe))
		{ // is there a payload? get the pipe number that recieved it
			// digitalWrite(1, led_state);
			// if ((t0 - last_read_us) > delay_time_us)
			// {
			// 	digitalWrite(LED_BUILTIN, HIGH);
			// }
			// else
			// {
			// 	digitalWrite(LED_BUILTIN, LOW);
			// }
			last_read_us = t0;

			led_state = (led_state == HIGH) ? LOW : HIGH;
			uint8_t bytes = radio.getPayloadSize(); // get the size of the payload
													// radio.read(&audio_buffer[write_idx], 32); // fetch payload from FIFO

			// digitalWrite(1, HIGH);
			payload_state = (payload_state == HIGH) ? LOW : HIGH;
			radio.read(&payload, 32); // fetch payload from FIFO
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
					audio_buffer[write_idx] = amp * float(payload[i]) / float(INT16_MAX);
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

			// memcpy(&audio_buffer[write_idx], payload, 32);
			// for (int k = 0; k < 8; k++)
			// {
			// 	Serial.print(audio_buffer[write_idx + k]);
			// 	Serial.print(" ");
			// }
			// Serial.println();

			// write_idx += SAMPS_PER_PAYLOAD;
			// available_samps += SAMPS_PER_PAYLOAD;

			// if (available_samps >= TOTAL_SAMPS)
			// {
			// 	radio.stopListening();
			// 	listening_paused = true;
			// }
			// else if (listening_paused)
			// {
			// 	listening_paused = false;
			// 	radio.startListening();
			// }
			// if (write_idx >= (TOTAL_SAMPS))
			// 	write_idx = 0;

			// radio.read(&payload, bytes); // fetch payload from FIFO
			// // write_idx += 8;
			// // if (write_idx >= AUDIO_BUF)
			// // 	write_idx = 0;
			// Serial.println("rx!");
		}
	}
	// if (Serial.available())
	// {
	// 	// change the role via the serial monitor

	// 	char c = toupper(Serial.read());
	// 	if (c == 'T' && !role)
	// 	{
	// 		// Become the TX node

	// 		role = true;
	// 		Serial.println(F("*** CHANGING TO TRANSMIT ROLE -- PRESS 'R' TO SWITCH BACK"));
	// 		radio.stopListening();
	// 	}
	// 	else if (c == 'R' && role)
	// 	{
	// 		// Become the RX node

	// 		role = false;
	// 		Serial.println(F("*** CHANGING TO RECEIVE ROLE -- PRESS 'T' TO SWITCH BACK"));
	// 		radio.startListening();
	// 	}
	// }

} // loop

// // instantiate an object for the nRF24L01 transceiver
// // RF24 radio(7, 8); // using pin 7 for the CE pin, and pin 8 for the CSN pin
// RF24 radio(ce_pin, csn_pin); // using pin 7 for the CE pin, and pin 8 for the CSN pin

// // Let these addresses be used for the pair
// uint8_t address[][6] = {"1Node", "2Node"};
// // It is very helpful to think of an address as a path instead of as
// // an identifying device destination

// // to use different addresses on a pair of radios, we need a variable to
// // uniquely identify which address this radio will use to transmit
// bool radioNumber = 0; // 0 uses address[0] to transmit, 1 uses address[1] to transmit

// // Used to control whether this node is sending or receiving
// bool role = false; // true = TX role, false = RX role

// // For this example, we'll be using a payload containing
// // a single float number that will be incremented
// // on every successful transmission
// byte payload = 0;

// void setup()
// {

// 	Serial.begin(9600);
// 	Serial.println("Waiting for Serial monitor to begin.");
// 	while (!Serial)
// 	{
// 		Serial.println("Waiting for Serial monitor to begin.");
// 		delay(1000);
// 		// some boards need to wait to ensure access to serial over USB
// 	}
// 	SPI.begin();
// 	SPI.setMOSI(mosi_pin);
// 	SPI.setMISO(miso_pin);
// 	SPI.setSCLK(sck_pin);
// 	// SPI.setCS(csn_pin);

// 	// initialize the transceiver on the SPI bus
// 	Serial.println("starting radio...");

// 	Serial.print("_init_pins: ");
// 	Serial.println(radio.isValid());

// 	Serial.print("_init_radio: ");
// 	SPI.begin();
// 	// Serial.println(radio.init_radio());

// 	if (!radio.begin(&SPI))
// 	{
// 		Serial.println(F("radio hardware is not responding!!"));
// 		while (1)
// 		{
// 			Serial.println(F("radio hardware is not responding!!"));
// 			delay(1000);

// 		} // hold in infinite loop
// 	}
// 	Serial.println("radio started!");

// 	// // print example's introductory prompt
// 	// Serial.println(F("RF24/examples/GettingStarted"));

// 	// // To set the radioNumber via the Serial monitor on startup
// 	// Serial.println(F("Which radio is this? Enter '0' or '1'. Defaults to '0'"));
// 	// while (!Serial.available())
// 	// {
// 	//  // wait for user input
// 	// }
// 	// char input = Serial.parseInt();
// 	// radioNumber = input == 1;
// 	// Serial.print(F("radioNumber = "));
// 	// Serial.println((int)radioNumber);

// 	// role variable is hardcoded to RX behavior, inform the user of this
// 	Serial.println(F("*** PRESS 'T' to begin transmitting to the other node"));

// 	// Set the PA Level low to try preventing power supply related problems
// 	// because these examples are likely run with nodes in close proximity to
// 	// each other.
// 	radio.setPALevel(RF24_PA_MAX); // RF24_PA_MAX is default.

// 	// save on transmission time by setting the radio to only transmit the
// 	// number of bytes we need to transmit a float
// 	radio.setPayloadSize(sizeof(payload)); // float datatype occupies 4 bytes

// 	radio.enableDynamicPayloads(); // ACK payloads are dynamically sized

// 	// Acknowledgement packets have no payloads by default. We need to enable
// 	// this feature for all nodes (TX & RX) to use ACK payloads.
// 	radio.enableAckPayload();

// 	// set the TX address of the RX node into the TX pipe
// 	radio.openWritingPipe(address[0]); // always uses pipe 0

// 	// set the RX address of the TX node into a RX pipe
// 	radio.openReadingPipe(1, address[1]); // using pipe 1
// 	// radio.setAutoAck(true);
// 	// additional setup specific to the node's role
// 	if (role)
// 	{
// 		radio.stopListening(); // put radio in TX mode
// 	}
// 	else
// 	{
// 		radio.startListening(); // put radio in RX mode
// 	}

// 	// For debugging info
// 	// printf_begin();             // needed only once for printing details
// 	// radio.printDetails();       // (smaller) function that prints raw register values
// 	radio.printPrettyDetails(); // (larger) function that prints human readable data

// } // setup

// void loop()
// {

// 	if (role)
// 	{
// 		// This device is a TX node

// 		unsigned long start_timer = micros();					  // start the timer
// 		bool report = radio.writeFast(&payload, sizeof(payload)); // transmit & save the report
// 		unsigned long end_timer = micros();						  // end the timer

// 		if (report)
// 		{
// 			Serial.print(F("Transmission successful! ")); // payload was delivered
// 			Serial.print(F("Time to transmit = "));
// 			Serial.print(end_timer - start_timer); // print the timer result
// 			Serial.print(F(" us. Sent: "));
// 			Serial.println(payload); // print payload sent
// 			payload++;				 // increment float payload
// 			if (payload >= 100)
// 				payload = 0;
// 		}
// 		else
// 		{
// 			Serial.println(F("Transmission failed or timed out")); // payload was not delivered
// 		}

// 		// to make this example readable in the serial monitor
// 		delay(100); // slow transmissions down by 1 second
// 	}
// 	else
// 	{
// 		// This device is a RX node

// 		uint8_t pipe;
// 		if (radio.available(&pipe))
// 		{											// is there a payload? get the pipe number that recieved it
// 			uint8_t bytes = radio.getPayloadSize(); // get the size of the payload
// 			radio.read(&payload, bytes);			// fetch payload from FIFO
// 			Serial.print(F("Received "));
// 			Serial.print(bytes); // print the size of the payload
// 			Serial.print(F(" bytes on pipe "));
// 			Serial.print(pipe); // print the pipe number
// 			Serial.print(F(": "));
// 			Serial.println(payload); // print the payload's value
// 		}
// 		else
// 		{
// 			Serial.println("no data");
// 		}
// 		radio.printPrettyDetails(); // (larger) function that prints human readable data
// 		delay(1000);				// slow transmissions down by 1 second

// 	} // role

// 	// if (Serial.available())
// 	// {
// 	// 	// change the role via the serial monitor

// 	// 	char c = toupper(Serial.read());
// 	// 	if (c == 'T' && !role)
// 	// 	{
// 	// 		// Become the TX node

// 	// 		role = true;
// 	// 		Serial.println(F("*** CHANGING TO TRANSMIT ROLE -- PRESS 'R' TO SWITCH BACK"));
// 	// 		radio.stopListening();
// 	// 	}
// 	// 	else if (c == 'R' && role)
// 	// 	{
// 	// 		// Become the RX node

// 	// 		role = false;
// 	// 		Serial.println(F("*** CHANGING TO RECEIVE ROLE -- PRESS 'T' TO SWITCH BACK"));
// 	// 		radio.startListening();
// 	// 	}
// 	// }

// } // loop