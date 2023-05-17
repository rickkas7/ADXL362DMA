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

unsigned long temperatureReport = 0;
const unsigned long temperatureReportPeriod = 30000;


void setup() {
    waitFor(Serial.isConnected, 10000);

	accel.softReset();
	while(accel.readStatus() == 0) {
		Log.info("no status yet, waiting for accelerometer");
		delay(1000);
	}

	accel.setMeasureMode(true);

}


void loop() {
    if (millis() - lastReport >= lastReportPeriod) {
        lastReport = millis();

        int16_t x, y, z;

        accel.readXYZ(x, y, z);

        Serial.printlnf("%5d %5d %5d", (int)x, (int)y, (int)z);
    }

    if (millis() - temperatureReport >= temperatureReportPeriod) {
        temperatureReport = millis();
        Log.info("temperature %.1f C, %.1f F", accel.readTemperatureC(), accel.readTemperatureF());
    }
}
