#include "Particle.h"

#include "ADXL362DMA.h"

#include <math.h>
#include <cmath>

// Library for the ADXL362 that uses SPI DMI for efficient data transfers
// https://github.com/rickkas7/ADXL362DMA


// Data Sheet:
// http://www.analog.com/media/en/technical-documentation/data-sheets/ADXL362.pdf

#if PLATFORM_THREADING
static Mutex syncCallbackMutex;
#else
// On the core, there is no mutex support
static volatile bool syncCallbackDone;
#endif

static ADXL362DataBase *readFifoData;
static ADXL362DMA *readFifoObject;

// These methods are described in greater detail in the .h file


ADXL362DMA::ADXL362DMA(SPIClass &spi, int cs, SPISettings settings) : spi(spi), cs(cs), settings(settings) {
	spi.begin(cs);
}

ADXL362DMA::~ADXL362DMA() {
}

void ADXL362DMA::softReset() {

	// Log.info("softReset");
	writeRegister8(REG_SOFT_RESET, 'R');
}

bool ADXL362DMA::chipDetect() {
	return readRegister8(REG_DEVID_AD) == 0xAD && readRegister8(REG_DEVID_MST) == 0x1D;
}

void ADXL362DMA::setSampleRate(SampleRate rate) {
	uint8_t filterCtl = readFilterControl();


	filterCtl &= ~ODR_MASK;
	filterCtl |= HALF_BW_MASK; // Actually means 1/4 bandwidth, the default

	switch(rate) {
		case SampleRate::RATE_3_125_HZ :
			filterCtl |= ODR_12_5;
			break;

		case SampleRate::RATE_6_25_HZ :
			filterCtl |= ODR_25;
			break;

		case SampleRate::RATE_12_5_HZ :
			filterCtl |= ODR_50;
			break;
			
		case SampleRate::RATE_25_HZ :
			filterCtl |= ODR_100;
			break;

		case SampleRate::RATE_50_HZ :
			filterCtl |= ODR_200;
			break;

		case SampleRate::RATE_200_HZ :
			filterCtl |= ODR_400;
			filterCtl &= ~HALF_BW_MASK; // Clearing the bit sets it to 1/2
			break;

		default:
		case SampleRate::RATE_100_HZ :
			filterCtl |= ODR_400;
			break;
	}
	Log.info("setSampleRate 0x%02x", filterCtl);

	writeFilterControl(filterCtl);
}

void ADXL362DMA::setMeasureMode(bool enabled) {

	uint8_t value = readRegister8(REG_POWER_CTL);

	value &= 0xfc; // remove low 2 bits
	if (enabled) {
		value |= 0x02;
	}

	writeRegister8(REG_POWER_CTL, value);

	//value = readRegister8(REG_POWER_CTL);
	//Log.info("setMeasureMode check=%02d", value);

}

void ADXL362DMA::readXYZT(int16_t &x, int16_t &y, int16_t &z, int16_t &t) {
	uint8_t req[10], resp[10];

	req[0] = CMD_READ_REGISTER;
	req[1] = REG_XDATA_L;
	for(size_t ii = 2; ii < sizeof(req); ii++) {
		req[ii] = 0;
	}

	syncTransaction(req, resp, sizeof(req));

	x = resp[2] | (((int16_t)resp[3]) << 8);
	y = resp[4] | (((int16_t)resp[5]) << 8);
	z = resp[6] | (((int16_t)resp[7]) << 8);
	t = resp[8] | (((int16_t)resp[9]) << 8);
}

void ADXL362DMA::readXYZ(int16_t &x, int16_t &y, int16_t &z) {
	uint8_t req[8], resp[8];

	req[0] = CMD_READ_REGISTER;
	req[1] = REG_XDATA_L;
	for(size_t ii = 2; ii < sizeof(req); ii++) {
		req[ii] = 0;
	}

	syncTransaction(req, resp, sizeof(req));

	x = resp[2] | (((int16_t)resp[3]) << 8);
	y = resp[4] | (((int16_t)resp[5]) << 8);
	z = resp[6] | (((int16_t)resp[7]) << 8);

}

float ADXL362DMA::readTemperatureC() {
	return ((float) ((int16_t)readRegister16(REG_TDATA_L))) / 16.0;
}

float ADXL362DMA::readTemperatureF() {
	return (readTemperatureC() * 9.0) / 5.0 + 32.0;
}

void ADXL362DMA::readRollPitchRadians(float &roll, float &pitch) {
	int16_t x, y, z;

	readXYZ(x, y, z);

	float xg, yg, zg;

	xg = (float)x * (float)rangeG / 2048.0;
	yg = (float)y * (float)rangeG / 2048.0;
	zg = (float)z * (float)rangeG / 2048.0;

	pitch = atan(xg / sqrt(pow(yg, 2) + pow(zg, 2)));
	roll = atan(yg / sqrt(pow(xg, 2) + pow(zg, 2)));
}

void ADXL362DMA::readRollPitchDegrees(float &roll, float &pitch) {
	readRollPitchRadians(roll, pitch);
	
	float conv = 180.0 / M_PI;

	pitch *= conv;
	roll *= conv;
}


uint8_t ADXL362DMA::readStatus() {
	return readRegister8(REG_STATUS);
}

uint16_t ADXL362DMA::readNumFifoEntries() {
	return readRegister16(REG_FIFO_ENTRIES_L);
}

void ADXL362DMA::readFifoAsync(ADXL362DataBase *data) {
	readFifoData = data;
	readFifoObject = this;

	data->sampleSizeInBytes = getSampleSizeInBytes();


	data->numSamplesRead = readNumFifoEntries() / (data->sampleSizeInBytes / 2);

	if (data->numSamplesRead < 1) {
		// Leave buffer in free state
		return;
	}

	size_t maxFullSamples = (data->bufSize - partialSampleBytesCount) / data->sampleSizeInBytes;
	if (data->numSamplesRead > maxFullSamples) {
		data->numSamplesRead = maxFullSamples;
	}

	data->bytesRead = data->numSamplesRead * data->sampleSizeInBytes;
	data->state = STATE_READING_FIFO;
	data->storeTemp = storeTemp;

	if (partialSampleBytesCount) {
		memcpy(data->buf, partialSampleBytes, partialSampleBytesCount);
	}

	beginTransaction();

	spi.transfer(CMD_READ_FIFO);

	spi.transfer(NULL, &data->buf[partialSampleBytesCount], data->bytesRead, readFifoCallbackInternal);
}

// [static]
void ADXL362DMA::readFifoCallbackInternal(void) {
	readFifoObject->endTransaction();
	readFifoObject->cleanBuffer(readFifoData);
	readFifoData->state = STATE_READ_COMPLETE;
}

void ADXL362DMA::cleanBuffer(ADXL362DataBase *data) {
	data->bytesRead += partialSampleBytesCount;
	partialSampleBytesCount = 0;

	for(data->startOffset = 0; data->startOffset < data->bytesRead; data->startOffset += 2) {
		uint8_t dataType = (data->buf[data->startOffset] >> 6) & 0x3;
		if (dataType == 0x0) { // x-axis
			break;
		}
	}

	data->numSamplesRead = (data->bytesRead - data->startOffset) / data->sampleSizeInBytes;

	partialSampleBytesCount = data->bytesRead - data->numSamplesRead * data->sampleSizeInBytes;
	if (partialSampleBytesCount > 0) {
		memcpy(partialSampleBytes, &data->buf[data->bytesRead - partialSampleBytesCount], partialSampleBytesCount);
	}

}


void ADXL362DMA::writeActivityThreshold(uint16_t value) { // value is an 11-bit integer
	writeRegister16(REG_THRESH_ACT_L, value);
}

void ADXL362DMA::writeActivityTime(uint8_t value) {
	writeRegister8(REG_TIME_ACT, value);
}
void ADXL362DMA::writeInactivityThreshold(uint16_t value) { // value is an 11-bit integer
	writeRegister16(REG_THRESH_INACT_L, value);
}
void ADXL362DMA::writeInactivityTime(uint16_t value) {
	writeRegister16(REG_TIME_INACT_L, value);
}

uint8_t ADXL362DMA::readActivityControl(uint8_t value) {
	return readRegister8(REG_ACT_INACT_CTL);
}

void ADXL362DMA::writeActivityControl(uint8_t value) {
	writeRegister8(REG_ACT_INACT_CTL, value);
}

void ADXL362DMA::writeActivityControl(uint8_t linkLoop, bool inactRef, bool inactEn, bool actRef, bool actEn) {
	uint8_t value = 0;

	value |= (linkLoop & 0x3) << 4;
	if (inactRef) {
		value |= ACTIVITY_INACT_REF;
	}
	if (inactEn) {
		value |= ACTIVITY_INACT_EN;
	}
	if (actRef) {
		value |= ACTIVITY_ACT_REF;
	}
	if (actEn) {
		value |= ACTIVITY_ACT_EN;
	}

	writeActivityControl(value);
}


uint8_t ADXL362DMA::readFifoControl() {
	return readRegister8(REG_FIFO_CONTROL);
}

void ADXL362DMA::writeFifoControl(uint8_t value) {
	writeRegister8(REG_FIFO_CONTROL, value);
}

void ADXL362DMA::writeFifoSamples(uint8_t value) {
	writeRegister8(REG_FIFO_SAMPLES, value);
}


void ADXL362DMA::writeFifoControlAndSamples(uint16_t samples, bool storeTemp, uint8_t fifoMode) {
	uint8_t value = 0;

	this->storeTemp = storeTemp;

	if (samples >= 0x100) {
		value |= 0x08; // AH bit
	}
	if (storeTemp) {
		value |= 0x04; // FIFO_TEMP bit
	}

	value |= (fifoMode & 0x3);

	writeRegister8(REG_FIFO_SAMPLES, samples & 0xff);
	writeRegister8(REG_FIFO_CONTROL, value);
}


uint8_t ADXL362DMA::readIntmap1() {
	return readRegister8(REG_FIFO_INTMAP1);
}

void ADXL362DMA::writeIntmap1(uint8_t value) {
	writeRegister8(REG_FIFO_INTMAP1, value);
}

uint8_t ADXL362DMA::readIntmap2() {
	return readRegister8(REG_FIFO_INTMAP2);
}

void ADXL362DMA::writeIntmap2(uint8_t value) {
	writeRegister8(REG_FIFO_INTMAP2, value);
}

uint8_t ADXL362DMA::readPowerCtl() {
	return readRegister8(REG_POWER_CTL);
}

void ADXL362DMA::writePowerCtl(uint8_t value) {
	writeRegister8(REG_POWER_CTL, value);
}

void ADXL362DMA::writePowerCtl(bool extClock, uint8_t lowNoise, bool wakeup, bool autosleep, uint8_t measureMode) {
	uint8_t temp = 0;

	if (extClock) {
		temp |= POWERCTL_EXT_CLK;
	}
	temp |= (lowNoise & 0x3) << 4;
	if (wakeup) {
		temp |= POWERCTL_WAKEUP;
	}
	if (autosleep) {
		temp |= POWERCTL_AUTOSLEEP;
	}
	temp |= measureMode & 0x3;

	writePowerCtl(temp);
}

void ADXL362DMA::writeLowNoise(uint8_t value) {
	uint8_t temp = readPowerCtl();

	temp &= 0xc0;
	temp |= (value & 0x3) << 4;

	writePowerCtl(temp);
}

void ADXL362DMA::writeMeasureMode(uint8_t value) {
	uint8_t temp = readPowerCtl();

	temp &= 0x3;
	temp |= value & 0x3;

	writePowerCtl(temp);
}



uint8_t ADXL362DMA::readFilterControl() {
	return readRegister8(REG_FILTER_CTL);
}

void ADXL362DMA::writeFilterControl(uint8_t value) {
	writeRegister8(REG_FILTER_CTL, value);
}


void ADXL362DMA::writeFilterControl(uint8_t range, bool halfBW, bool extSample, uint8_t odr) {
	uint8_t value = 0;

	value |= (range & 0x3) << 6;

	switch(range) {
		case RANGE_4G:
			rangeG = 4;
			break;

		case RANGE_8G:
			rangeG = 8;
			break;

		default:
		case RANGE_2G:
			rangeG = 2;
			break;
	}

	if (halfBW) {
		value |= 0x10;
	}
	if (extSample) {
		value |= 0x08;
	}
	value |= (odr & 0x7);

	writeRegister8(REG_FILTER_CTL, value);
}


uint8_t ADXL362DMA::readRegister8(uint8_t addr) {
	uint8_t req[3], resp[3];

	req[0] = CMD_READ_REGISTER;
	req[1] = addr;
	req[2] = 0;

	syncTransaction(req, resp, sizeof(req));

	return resp[2];
}

uint16_t ADXL362DMA::readRegister16(uint8_t addr) {
	uint8_t req[4], resp[4];

	req[0] = CMD_READ_REGISTER;
	req[1] = addr;
	req[2] = req[3] = 0;

	syncTransaction(req, resp, sizeof(req));

	return resp[2] | (((uint16_t)resp[3]) << 8);
}


void ADXL362DMA::writeRegister8(uint8_t addr, uint8_t value) {
	// Serial.printlnf("writeRegister addr=%02x value=%02x", addr, value);

	uint8_t req[3], resp[3];

	req[0] = CMD_WRITE_REGISTER;
	req[1] = addr;
	req[2] = value;

	syncTransaction(req, resp, sizeof(req));
}

void ADXL362DMA::writeRegister16(uint8_t addr, uint16_t value) {
	// Serial.printlnf("writeRegister addr=%02x value=%04x", addr, value);

	uint8_t req[4], resp[4];

	req[0] = CMD_WRITE_REGISTER;
	req[1] = addr;
	req[2] = value & 0xff;
	req[3] = value >> 8;

	syncTransaction(req, resp, sizeof(req));
}


void ADXL362DMA::beginTransaction() {
	spi.beginTransaction(settings);
	digitalWrite(cs, LOW);
}

void ADXL362DMA::endTransaction() {
	digitalWrite(cs, HIGH);
	spi.endTransaction();
}

void ADXL362DMA::syncTransaction(void *req, void *resp, size_t len) {
	beginTransaction();

	spi.transfer(req, resp, len, nullptr);

	endTransaction();
}



/*
It is recommended that an even number of bytes be read (using a multibyte transaction) because each sample consists of two bytes: 2 bits of axis information and 14 bits of data. If an odd number of bytes is read, it is assumed that the desired data was read; therefore, the second half of the last sample is discarded so a read from the FIFO always starts on a properly aligned even- byte boundary. Data is presented least significant byte first, followed by the most significant byte.
*/



int16_t ADXL362DataBase::readSigned14(const uint8_t *pValue) const {
	uint8_t msb = pValue[0] & 0x3f;
	if (msb & 0x20) {
		// Add in sign extension
		msb |= 0xc0;
	}

	return ((int16_t) pValue[1] | (msb << 8));
}

int16_t ADXL362DataBase::readX(size_t index) const {
	return readSigned14(&buf[startOffset + sampleSizeInBytes * index]);
}

int16_t ADXL362DataBase::readY(size_t index) const {
	return readSigned14(&buf[startOffset + sampleSizeInBytes * index + 2]);
}

int16_t ADXL362DataBase::readZ(size_t index) const {
	return readSigned14(&buf[startOffset + sampleSizeInBytes * index + 4]);
}

int16_t ADXL362DataBase::readT(size_t index) const {
	return readSigned14(&buf[startOffset + sampleSizeInBytes * index + 6]);
}

