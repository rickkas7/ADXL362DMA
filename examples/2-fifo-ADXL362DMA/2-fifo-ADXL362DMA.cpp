// Program to test sending accelerometer data off a Photon in real time
// Uses an Analog Devices ADXL362 SPI accelerometer (the one in the Electron Sensor Kit)

#include "Particle.h"

// Accelerometer sensor file. I only implemented the parts that I needed so the library is not complete.
#include "ADXL362DMA.h"

//
SYSTEM_THREAD(ENABLED);
SerialLogHandler logHandler;

// Number of 256 byte buffers to allocate. The more buffers, the longer network hiccup can be accommodated for.
const size_t NUM_BUFFERS = 64;

// Finite state machine states
enum State { STATE_CONNECT, STATE_CHECK_BUFFER, STATE_SEND, STATE_RETRY_WAIT };



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

ADXL362DataEx<1024> dataBuffer;


// Global variables

void setup() {
    waitFor(Serial.isConnected, 10000);

	accel.softReset();
	while(accel.readStatus() == 0) {
		Log.info("no status yet, waiting for accelerometer");
		delay(1000);
	}

	// Program the accelerometer to gather samples automatically and store them in its
	// internal FIFO.
	accel.writeFifoControlAndSamples(511, false, accel.FIFO_STREAM);

    accel.setSampleRate(ADXL362DMA::SampleRate::RATE_3_125_HZ);
	accel.setMeasureMode(true);
}


void loop() {
	// Handle emptying the FIFO
    switch(dataBuffer.state) {
    case ADXL362DMA::STATE_FREE:
        // Read new samples
		accel.readFifoAsync(&dataBuffer);
        break;

    case ADXL362DMA::STATE_READING_FIFO:
        // Waiting for read to complete
        break;

    case ADXL362DMA::STATE_READ_COMPLETE:
        // Print output
        Log.info("bytesRead=%d numSamples=%d startOffset=%d", (int)dataBuffer.bytesRead, dataBuffer.numSamplesRead, dataBuffer.startOffset);
        for(size_t ii = 0; ii < dataBuffer.numSamplesRead; ii++) {
            Serial.printlnf("%5d %5d %5d", (int)dataBuffer.readX(ii), (int)dataBuffer.readY(ii), (int)dataBuffer.readZ(ii) );
        }

        // Fill buffer again
        dataBuffer.state = ADXL362DMA::STATE_FREE;
     
        break;
    }
}
