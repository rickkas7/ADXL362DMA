#include "Particle.h"

#include "ADXL362DMA.h"

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

static ADXL362Data *readFifoData;
static ADXL362DMA *readFifoObject;

// These methods are described in greater detail in the .h file


ADXL362DMA::ADXL362DMA(SPIClass &spi, int cs, SPISettings settings) : spi(spi), cs(cs), settings(settings) {
	spi.begin(cs);
}

ADXL362DMA::~ADXL362DMA() {
}

void ADXL362DMA::softReset() {

	// Serial.println("softReset");
	writeRegister8(REG_SOFT_RESET, 'R');
}

bool ADXL362DMA::chipDetect() {
	return readRegister8(REG_DEVID_AD) == 0xAD && readRegister8(REG_DEVID_MST) == 0x1D;
}

void ADXL362DMA::setSampleRate(SampleRate rate) {
	uint8_t filterCtl = readFilterControl();

	filterCtl &= ~(HALF_BW_MASK | ODR_MASK);

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
			filterCtl |= ODR_400 | HALF_BW_MASK;
			break;

		default:
		case SampleRate::RATE_100_HZ :
			filterCtl |= ODR_400;
			break;
	}

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
	//Serial.printlnf("setMeasureMode check=%02d", value);

}

void ADXL362DMA::readXYZT(int &x, int &y, int &z, int &t) {
	uint8_t req[10], resp[10];

	req[0] = CMD_READ_REGISTER;
	req[1] = REG_XDATA_L;
	for(size_t ii = 2; ii < sizeof(req); ii++) {
		req[ii] = 0;
	}

	syncTransaction(req, resp, sizeof(req));

	x = resp[2] | (((int)resp[3]) << 8);
	y = resp[4] | (((int)resp[5]) << 8);
	z = resp[6] | (((int)resp[7]) << 8);
	t = resp[8] | (((int)resp[9]) << 8);
}

uint8_t ADXL362DMA::readStatus() {
	return readRegister8(REG_STATUS);
}

uint16_t ADXL362DMA::readNumFifoEntries() {
	return readRegister16(REG_FIFO_ENTRIES_L);
}

void ADXL362DMA::readFifoAsync(ADXL362Data *data) {
	if (busy) {
		return;
	}

	readFifoData = data;
	readFifoObject = this;

	size_t entrySetSize = getEntrySetSize();

	size_t maxFullEntries = data->bufSize / entrySetSize;

	size_t numEntries = ((size_t) readNumFifoEntries() * 2) / entrySetSize;
	if (numEntries > maxFullEntries) {
		numEntries = maxFullEntries;
	}
	data->bytesRead = numEntries * entrySetSize;
	data->state = ADXL362Data::STATE_READING_FIFO;
	data->storeTemp = storeTemp;


	beginTransaction();

	spi.transfer(CMD_READ_FIFO);

	spi.transfer(NULL, data->buf, data->bytesRead, readFifoCallbackInternal);
}

// [static]
void ADXL362DMA::readFifoCallbackInternal(void) {
	readFifoObject->endTransaction();
	readFifoData->state = ADXL362Data::STATE_READ_COMPLETE;
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
	busy = true;
	digitalWrite(cs, LOW);
}

void ADXL362DMA::endTransaction() {
	digitalWrite(cs, HIGH);
	busy = false;
	spi.endTransaction();
}

void ADXL362DMA::syncTransaction(void *req, void *resp, size_t len) {
#if PLATFORM_THREADING
	syncCallbackMutex.lock();

	beginTransaction();

	spi.transfer(req, resp, len, syncCallback);
	syncCallbackMutex.lock();

	endTransaction();

	syncCallbackMutex.unlock();
#else
	syncCallbackDone = false;
	beginTransaction();

	spi.transfer(req, resp, len, syncCallback);

	while(!syncCallbackDone) {
	}

	endTransaction();
#endif
}

// [static]
void ADXL362DMA::syncCallback(void) {
#if PLATFORM_THREADING
	syncCallbackMutex.unlock();
#else
	syncCallbackDone = true;
#endif
}




