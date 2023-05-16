// Program to test sending accelerometer data off a Photon in real time
// Uses an Analog Devices ADXL362 SPI accelerometer (the one in the Electron Sensor Kit)

#include "Particle.h"

// Accelerometer sensor file. I only implemented the parts that I needed so the library is not complete.
#include "ADXL362DMA.h"

//
SYSTEM_THREAD(ENABLED);
SerialLogHandler logHandler;


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

// Global variables
unsigned long lastReport = 0;
const unsigned long lastReportPeriod = 100;

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
    if (millis() - lastReport >= lastReportPeriod) {
        lastReport = millis();

        int x, y, z, t;

        accel.readXYZT(x, y, z, t);

        Serial.printlnf("%5d %5d %5d %3d", x, y, z, t);
    }
}
