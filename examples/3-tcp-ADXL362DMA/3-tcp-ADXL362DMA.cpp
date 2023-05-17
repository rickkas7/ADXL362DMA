// Program to test sending accelerometer data off a Photon in real time
// Uses an Analog Devices ADXL362 SPI accelerometer (the one in the Electron Sensor Kit)

#include "Particle.h"

// Accelerometer sensor file. I only implemented the parts that I needed so the library is not complete.
#include "ADXL362DMA.h"

//
SYSTEM_THREAD(ENABLED);
SerialLogHandler logHandler;

// Number of 256 byte buffers to allocate. The more buffers, the longer network hiccup can be accommodated for.
const size_t NUM_BUFFERS = 128;

// Finite state machine states
enum State { STATE_CONNECT, STATE_CHECK_BUFFER, STATE_SEND, STATE_RETRY_WAIT };

// Various timeout values.
const unsigned long retryWaitTimeMs = 2000;
const unsigned long sendTimeoutMs = 60000;


// Connect the ADXL362 breakout:
// VIN: 3V3
// GND: GND
// SCL: A3 (SCK)
// SDA: A5 (MOSI)
// SDO: A4 (MISO)
// CS: A2 (SS)
// INT1: no connection
// INT1: no connection
ADXL362DMA accel(SPI, A2);

unsigned long lastCheck = 0;
size_t fillBuffer = 0;
size_t sendBuffer = 0;
ADXL362Data dataBuffers[NUM_BUFFERS];

// Set to the IP address of the server to connect to
IPAddress serverAddr(192,168,2,6);
const int serverPort = 7123;

// Global variables
State state = STATE_CONNECT;
TCPClient client;
unsigned long stateTime = 0;
size_t totalSent;

void setup() {

	accel.softReset();

	while(accel.readStatus() == 0) {
		Log.info("no status yet, waiting for device");
		delay(1000);
	}

	// Program the accelerometer to gather samples automatically and store them in its
	// internal FIFO.
	accel.writeFifoControlAndSamples(511, false, accel.FIFO_STREAM);
	accel.writeFilterControl(accel.RANGE_2G, false, false, accel.ODR_200);
	accel.setMeasureMode(true);

}


void loop() {
	// Handle emptying the FIFO
	if (!accel.getIsBusy()) {
		// Can't query the number of entries in the FIFO if we're currently reading entries from the FIFO

		// numEntries is the number of 16-byte values, not the number of bytes!
		uint16_t numEntries = accel.readNumFifoEntries();
		ADXL362Data *data = &dataBuffers[fillBuffer % NUM_BUFFERS];
		if (numEntries >= (data->bufSize / 2)) {
			Log.info("numEntries=%d fillBuffer=%d sendBuffer=%d state=%d", (int)numEntries, (int)fillBuffer, (int)sendBuffer, data->state);
			if ((fillBuffer - sendBuffer) == NUM_BUFFERS) {
				Log.info("send buffer full, discarding old samples sendBuffer=%d", sendBuffer);
				ADXL362Data *dataSend = &dataBuffers[sendBuffer % NUM_BUFFERS];
				dataSend->state = ADXL362Data::STATE_FREE;
				sendBuffer++;
			}
			accel.readFifoAsync(data);
			fillBuffer++;
		}
	}

	// Networking state machine
	switch(state) {
	case STATE_CONNECT:
		Log.info("** trying connection millis=%lu", millis());

		if (!client.connect(serverAddr, serverPort)) {
			// Connection failed
			stateTime = millis();
			state = STATE_RETRY_WAIT;
			break;
		}
		totalSent = 0;
		state = STATE_SEND;
		// Fall through

	case STATE_SEND:
		if (client.connected()) {
			ADXL362Data *data = &dataBuffers[sendBuffer % NUM_BUFFERS];
			if (data->state != ADXL362Data::STATE_READ_COMPLETE) {
				// No data to send yet
				break;
			}

			int count = client.write(data->buf, data->bytesRead);
			if (count == -16) {
				// Special case: Internal buffer is full, just retry at the same offset next time
				// I'm pretty sure the result code for this is different on the Core, and probably the Electron.
				//Serial.println("buffer full");
			}
			else
			if (count > 0) {
				// In theory, count could be less than buffer size. It wouldn't be a bad idea to support
				// that in real code, but for this test I ignore it. I've never seen it happen on the
				// Photon or Electron.
				if ((size_t)count < data->bytesRead) {
					Log.info("error: sent %d expected %d", count, data->bytesRead);
				}

				stateTime = millis();
				totalSent += count;

				data->state = ADXL362Data::STATE_FREE;
				sendBuffer++;
				state = STATE_SEND;
			}
			else {
				// Error
				Log.info("** error sending error=%d totalSent=%lu millis=%lu", count, totalSent, millis());
				client.stop();
				stateTime = millis();
				state = STATE_RETRY_WAIT;
			}
		}
		else {
			Log.info("** connection closed totalSent=%lu millis=%lu", totalSent, millis());
			client.stop();
			stateTime = millis();
			state = STATE_RETRY_WAIT;
		}
		break;

	case STATE_RETRY_WAIT:
		if (millis() - stateTime > retryWaitTimeMs) {
			// Wait a few seconds before retrying
			state = STATE_CONNECT;
			break;
		}
		break;
	}
}
