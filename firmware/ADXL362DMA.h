#ifndef __ADXL362_H
#define __ADXL362_H

// Library for the ADXL362 that uses SPI DMI for efficient data transfers
// https://github.com/rickkas7/ADXL362DMA

// Connect the ADXL362 breakout (when using with SPI instead of SPI1)
// VIN: 3V3
// GND: GND
// SCL: A3 (SCK)
// SDA: A5 (MOSI)
// SDO: A4 (MISO)
// CS: A2 (SS)
// INT2: depends on usage
// INT1: depends on usage

class ADXL362Data; // forward declaration

class ADXL362DMA {
public:
	/**
	 * Initialize the ADXL362 handler object. Usually created as  global variable like this:
	 *
	 * ADXL362 accel(SPI, A2);
	 */
	ADXL362DMA(SPIClass &spi, int ss = A2);
	virtual ~ADXL362DMA();

	/**
	 * Issue a soft reset call. It may make a little while for the sensor to respond after
	 * a softReset. readStatus() will return non-zero when ready.
	 */
	void softReset();

	/**
	 * Enable or disable measure mode in the power control register
	 *
	 * This a convenience method that's easier than directly manipulating the POWER_CTL register.
	 */
	void setMeasureMode(bool enabled = true);

	/**
	 * Read a single XYZT sample from the current data register
	 *
	 * If you are continuously reading samples, using the FIFO is more efficient
	 */
	void readXYZT(int &x, int &y, int &z, int &t);

	/**
	 * Reads the status register STATUS
	 * Address: 0x0B, Reset: 0x40, Name: STATUS
	 *
	 * This is a good way to see if the chip is responding; it normally returns a
	 * non-zero value (0x40 = AWAKE).
	 */
	uint8_t readStatus();

	/**
	 * Reads the number of entries available to read from the FIFO (FIFO_ENTRIES_L and FIFO_ENTRIES_H)
	 *
	 * Use readFifoAsync to read the bytes. Note: Because this accesses the chip using SPI, you should
	 * check that the chips is not busy using getIsBusy() before calling this if you're mixing calls
	 * to this and readFifoAsync() in your loop().
	 */
	uint16_t readNumFifoEntries();

	/**
	 * Reads entries from the FIFO asynchronously using SPI DMA
	 */
	void readFifoAsync(ADXL362Data *data);

	/**
	 * Write the activity threshold register
	 * Address: 0x20, Reset: 0x00, Name: THRESH_ACT_L
	 * Address: 0x21, Reset: 0x00, Name: THRESH_ACT_H
	 *
	 * This doesn't enable the feature, you still need to set the appropriate bit with writeActivityControl.
	 *
	 * To detect activity, the ADXL362 compares the absolute value of
	 * the 12-bit (signed) acceleration data with the 11-bit (unsigned)
	 * THRESH_ACT value. See the Motion Detection section for
	 * more information on activity detection.
	 * The term, THRESH_ACT, refers to an 11-bit unsigned value comprising
	 * the THRESH_ACT_L register, which holds its eight LSBs;
	 * and the THRESH_ACT_H register, which holds its three MSBs.
	 * THRESH_ACT is set in codes; the value in g depends on the
	 * measurement range setting that is selected.
	 * THRESH_ACT [g] = THRESH_ACT [codes]/Sensitivity [codes per g]
	 *
	 * value is an 11-bit integer, 0 - 2047 inclusive
	 */
	void writeActivityThreshold(uint16_t value);

	/**
	 * Write the activity time register
	 * Address: 0x22, Reset: 0x00, Name: TIME_ACT
	 *
	 * The activity timer implements a robust activity detection that
	 * minimizes false positive motion triggers. When the timer is
	 * used, only sustained motion can trigger activity detection. Refer
	 * to the Fewer False Positives section for additional information.
	 * The value in this register sets the number of consecutive
	 * samples that must have at least one axis greater than the activity
	 * threshold (set by THRESH_ACT) for an activity event to be
	 * detected.
	 * The time (in seconds) is given by the following equation:
	 * Time = TIME_ACT/ODR
	 * where:
	 * TIME_ACT is the value set in this register.
	 * ODR is the output data rate set in the FILTER_CTL register
	 * (Address 0x2C).
	 * Setting the activity time to 0x00 has the same result as setting
	 * this time to 0x01: Activity is detected when a single acceleration
	 * sample has at least one axis greater than the activity threshold
	 * (THRESH_ACT).
	 * When the accelerometer is in wake-up mode, the TIME_ACT
	 * value is ignored and activity is detected based on a single
	 * acceleration sample.
	 */
	void writeActivityTime(uint8_t value);

	/**
	 * Write the inactivity threshold register
	 * Address: 0x23, Reset: 0x00, Name: THRESH_INACT_L
	 * Address: 0x24, Reset: 0x00, Name: THRESH_INACT_H
	 *
	 * To detect inactivity, the absolute value of the 12-bit acceleration
	 * data is compared with the 11-bit (unsigned) THRESH_INACT
	 * value. See the Motion Detection section for more information.
	 * The term, THRESH_INACT, refers to an 11-bit unsigned value
	 * comprised of the THRESH_INACT_L registers, which holds its
	 * eight LSBs and the THRESH_INACT_H register, which holds
	 * its three MSBs.
	 * This 11-bit unsigned value sets the threshold for inactivity
	 * detection. This value is set in codes; the value (in g) depends on
	 * the measurement range setting selected:
	 * THRESH_INACT [g] = THRESH_INACT [codes]/Sensitivity [codes per g]
	 *
	 * value is an 11-bit integer, 0 - 2047 inclusive
	 */
	void writeInactivityThreshold(uint16_t value);

	/**
	 * Writes the inactivity time register
	 * Address: 0x25, Reset: 0x00, Name: TIME_INACT_L
	 * Address: 0x26, Reset: 0x00, Name: TIME_INACT_H
	 *
	 * The 16-bit value in these registers sets the number of consecutive
	 * samples that must have all axes lower than the inactivity
	 * threshold (set by THRESH_INACT) for an inactivity event to
	 * be detected.
	 * The TIME_INACT_L register holds the eight LSBs and the
	 * TIME_INACT_H register holds the eight MSBs of the 16-bit
	 * TIME_INACT value.
	 * The time in seconds can be calculated as
	 * Time = TIME_INACT/ODR
	 * where:
	 * TIME_INACT is the 16-bit value set by the TIME_INACT_L register
	 * (eight LSBs) and the TIME_INACT_H register (eight MSBs).
	 * ODR is the output data rate set in the FILTER_CTL register
	 * (Address 0x2C)
	 *
	 * The 16-bit value allows for long inactivity detection times. The
	 * maximum value is 0xFFFF or 65,535 samples. At the lowest output
	 * data rate, 12.5 Hz, this equates to almost 90 minutes. In this configuration,
	 * the accelerometer must be stationary for 90 minutes
	 * before putting its system to sleep.
	 * Setting the inactivity time to 0x00 has the same result as setting
	 * this time to 0x01: Inactivity is detected when a single acceleration
	 * sample has all axes lower than the inactivity threshold
	 * (THRESH_INACT).
	 */
	void writeInactivityTime(uint16_t value);

	/**
	 * Read the activity/inactivity control register
	 * Address: 0x27, Reset: 0x00, Name: ACT_INACT_CTL
	 */
	uint8_t readActivityControl(uint8_t value);


	/**
	 * Write the activity/inactivity control register
	 * Address: 0x27, Reset: 0x00, Name: ACT_INACT_CTL
	 *
	 * There is also an overload that takes the parameters broken out into separate parameters for ease of use
	 */
	void writeActivityControl(uint8_t value);


	/**
	 * Write the activity/inactivity control register
	 * Address: 0x27, Reset: 0x00, Name: ACT_INACT_CTL
	 *
	 * linkLoop: one of the following constants: LINKLOOP_DEFAULT, LINKLOOP_LINKED, LINKLOOP_LOOP
	 * inactRef: inactivity detection uses reference mode (automatically compensates for gravity) if true
	 * inactEn: inactivity detection is enabled
	 * actRef: activity detection uses reference mode (automatically compensates for gravity) if true
	 * actEn: activity detection is enabled
	 */
	void writeActivityControl(uint8_t linkLoop, bool inactRef, bool inactEn, bool actRef, bool actEn);


	/**
	 * Read the FIFO control register
	 * Address: 0x28, Reset: 0x00, Name: FIFO_CONTROL
	 */
	uint8_t readFifoControl();

	/**
	 * Writes the FIFO control register
	 * Address: 0x28, Reset: 0x00, Name: FIFO_CONTROL
	 *
	 * It's usually easier to call writeFifoControlAndSamples which sets both registers at once.
	 */
	void writeFifoControl(uint8_t value);

	/**
	 * Writes the FIFO samples register
	 * Address: 0x29, Reset: 0x80, Name: FIFO_SAMPLES
	 *
	 * It's usually easier to call writeFifoControlAndSamples which sets both registers at once.
	 */
	void writeFifoSamples(uint8_t value);

	// samples 0-511, fifoMode = FIFO_DISABLED, etc.
	/**
	 * Writes the FIFO control and samples registers
	 *
	 * samples: Number of samples to store 0-511
	 * storeTemp: Whether to store XYZT (with temperature, if true) or just XYZ data in FIFO
	 * fifoMode: One of the constants: FIFO_DISABLED, FIFO_OLDEST_SAVED, FIFO_STREAM, FIFO_TRIGGERED
	 */
	void writeFifoControlAndSamples(uint16_t samples, bool storeTemp, uint8_t fifoMode);

	/**
	 * Reads the INTMAP1 register
	 * Address: 0x2A, Reset: 0x00, Name: INTMAP1
	 */
	uint8_t readIntmap1();

	/**
	 * Writes the INTMAP1 register
	 * Address: 0x2A, Reset: 0x00, Name: INTMAP1
	 *
	 * The INT1 and INT2 registers configure the INT1/INT2
	 * interrupt pins, respectively. Bits[B6:B0] select which function(s)
	 * generate an interrupt on the pin. If its corresponding bit is set to
	 * 1, the function generates an interrupt on the INT pin. Bit B7
	 * configures whether the pin operates in active high (B7 low) or
	 * active low (B7 high) mode.
	 * Any number of functions can be selected simultaneously for
	 * each pin. If multiple functions are selected, their conditions are
	 * OR'ed together to determine the INT pin state. The status of
	 * each individual function can be determined by reading the
	 * STATUS register. If no interrupts are mapped to an INT pin, the
	 * pin remains in a high impedance state, held to a valid logic state
	 * by a bus keeper.
	 */
	void writeIntmap1(uint8_t value);

	/**
	 * Reads the INTMAP2 register
	 * Address: 0x2B, Reset: 0x00, Name: INTMAP2
	 */
	uint8_t readIntmap2();

	/**
	 * Writes the INTMAP2 register
	 * Address: 0x2B, Reset: 0x00, Name: INTMAP2
	 */
	void writeIntmap2(uint8_t value);

	/**
	 * Reads the filter control register
	 * Address: 0x2C, Reset: 0x13, Name: FILTER_CTL
	 */
	uint8_t readFilterControl();

	/**
	 * Writes the filter control register
	 * Address: 0x2C, Reset: 0x13, Name: FILTER_CTL
	 *
	 * There is an overload of writeFilterControl that has the bits broken out into separate parameters
	 * that may be easier to use.
	 */
	void writeFilterControl(uint8_t value);

	/**
	 * Writes the filter control register
	 * Address: 0x2C, Reset: 0x13, Name: FILTER_CTL
	 *
	 * There is an overload of writeFilterControl that takes the single uint8_t value instead of the
	 * broken out values, as well.
	 */
	void writeFilterControl(uint8_t range, bool halfBW, bool extSample, uint8_t odr);

	/**
	 * Reads the power control register
	 * Address: 0x2D, Reset: 0x00, Name: POWER_CTL
	 */
	uint8_t readPowerCtl();


	/**
	 * Writes the power control register
	 * Address: 0x2D, Reset: 0x00, Name: POWER_CTL
	 *
	 * There are also separate calls writeLowNoise and writeMeasureMode to set just those values.
	 *
	 * lowNoise: one of LOWNOISE_NORMAL, LOWNOISE_LOW, LOWNOISE_ULTRALOW
	 * measureMode: one of MEASURE_STANDBY, MEASURE_MEASUREMENT
	 */
	void writePowerCtl(uint8_t value);
	void writePowerCtl(bool extClock, uint8_t lowNoise, bool wakeup, bool autosleep, uint8_t measureMode);

	/**
	 * Set the low noise mode in the power control register
	 *
	 * value: one of LOWNOISE_NORMAL, LOWNOISE_LOW, LOWNOISE_ULTRALOW
	 */
	void writeLowNoise(uint8_t value);

	/**
	 * Set the measure mode in the power control register
	 *
	 * value: one of MEASURE_STANDBY, MEASURE_MEASUREMENT
	 */
	void writeMeasureMode(uint8_t value);

	/**
	 * Reads an 8-bit register value
	 *
	 * Most of the calls have easier to use accessors like readStatus() that use this call internally.
	 *
	 * addr: One of the register addresses, such as REG_STATUS
	 */
	uint8_t readRegister8(uint8_t addr);

	/**
	 * Reads an 16-bit register value
	 *
	 * addr: One of the register addresses, such as REG_THRESH_ACT_L. It must be the first of a pair of _L and _H registers.
	 */
	uint16_t readRegister16(uint8_t addr);

	/**
	 * Write an 8-bit register value
	 *
	 * Most of the calls have easier to use accessors like writeIntmap1() that use this call internally.
	 *
	 * addr: One of the register addresses, such as REG_INTMAP1
	 */
	void writeRegister8(uint8_t addr, uint8_t value);

	/**
	 * Write an 16-bit register value
	 *
	 * Most of the calls have easier to use accessors like writeIntmap1() that use this call internally.
	 *
	 * addr: One of the register addresses, such as REG_THRESH_ACT_L. It must be the first of a pair of _L and _H registers.
	 */
	void writeRegister16(uint8_t addr, uint16_t value);

	/**
	 * Returns the number of bytes for a full XYZ or XYZT FIFO entry depending on the storeTemp flag
	 */
	size_t getEntrySetSize() { return storeTemp ? 8 : 6; };

	/**
	 * Returns true if a SPI command is currently being handled
	 */
	bool getIsBusy() { return busy; };

	/**
	 * Begin a synchronous SPI DMI transaction
	 */
	void syncTransaction(void *req, void *resp, size_t len);

	// Command bytes
	static const uint8_t CMD_WRITE_REGISTER = 0x0a;
	static const uint8_t CMD_READ_REGISTER = 0x0b;
	static const uint8_t CMD_READ_FIFO = 0x0d;

	// Registers
	static const uint8_t REG_DEVID_AD = 0x00;
	static const uint8_t REG_DEVID_MST = 0x01;
	static const uint8_t REG_STATUS = 0x0b;
	static const uint8_t REG_FIFO_ENTRIES_L = 0x0c;
	static const uint8_t REG_FIFO_ENTRIES_H = 0x0d;
	static const uint8_t REG_XDATA_L = 0x0e;
	static const uint8_t REG_XDATA_H = 0x0f;
	static const uint8_t REG_YDATA_L = 0x10;
	static const uint8_t REG_YDATA_H = 0x11;
	static const uint8_t REG_ZDATA_L = 0x12;
	static const uint8_t REG_ZDATA_H = 0x13;
	static const uint8_t REG_TDATA_L = 0x14;
	static const uint8_t REG_TDATA_H = 0x15;
	static const uint8_t REG_SOFT_RESET = 0x1f;
	static const uint8_t REG_THRESH_ACT_L = 0x20;
	static const uint8_t REG_THRESH_ACT_H = 0x21;
	static const uint8_t REG_TIME_ACT = 0x22;
	static const uint8_t REG_THRESH_INACT_L = 0x23;
	static const uint8_t REG_THRESH_INACT_H = 0x24;
	static const uint8_t REG_TIME_INACT_L = 0x25;
	static const uint8_t REG_TIME_INACT_H = 0x26;
	static const uint8_t REG_ACT_INACT_CTL = 0x27;
	static const uint8_t REG_FIFO_CONTROL = 0x28;
	static const uint8_t REG_FIFO_SAMPLES = 0x29;
	static const uint8_t REG_FIFO_INTMAP1 = 0x2a;
	static const uint8_t REG_FIFO_INTMAP2 = 0x2b;
	static const uint8_t REG_FILTER_CTL = 0x2c;
	static const uint8_t REG_POWER_CTL = 0x2d;
	static const uint8_t REG_SELF_TEST = 0x2e;


	// Status bits in status register
	static const uint8_t STATUS_ERR_USER_REGS = 0x80;
	static const uint8_t STATUS_AWAKE = 0x40;
	static const uint8_t STATUS_INACT = 0x20;
	static const uint8_t STATUS_ACT = 0x10;
	static const uint8_t STATUS_FIFO_OVERRUN = 0x08;
	static const uint8_t STATUS_FIFO_WATERMARK = 0x04;
	static const uint8_t STATUS_FIFO_READY = 0x02;
	static const uint8_t STATUS_DATA_READ = 0x01;

	// Activity/inactivity control register
	static const uint8_t LINKLOOP_DEFAULT = 0x0;
	static const uint8_t LINKLOOP_LINKED = 0x1;
	static const uint8_t LINKLOOP_LOOP = 0x3;

	static const uint8_t ACTIVITY_INACT_REF = 0x08;
	static const uint8_t ACTIVITY_INACT_EN = 0x04;
	static const uint8_t ACTIVITY_ACT_REF = 0x02;
	static const uint8_t ACTIVITY_ACT_EN = 0x01;

	// Range in Filter Control Register
	static const uint8_t RANGE_2G 	= 0x0;// default
	static const uint8_t RANGE_4G 	= 0x1;
	static const uint8_t RANGE_8G 	= 0x2;


	// Output Data Rate in Filter Control Register
	static const uint8_t ODR_12_5 	= 0x0;
	static const uint8_t ODR_25 	= 0x1;
	static const uint8_t ODR_50 	= 0x2;
	static const uint8_t ODR_100 	= 0x3; // default
	static const uint8_t ODR_200 	= 0x4;
	static const uint8_t ODR_400 	= 0x5;

	// FIFO mode
	static const uint8_t FIFO_DISABLED 	= 0x0;
	static const uint8_t FIFO_OLDEST_SAVED = 0x1;
	static const uint8_t FIFO_STREAM 		= 0x2;
	static const uint8_t FIFO_TRIGGERED 	= 0x3;

	// INTMAP1 and INTMAP2
	static const uint8_t INTMAP_INT_LOW = 0x80;
	static const uint8_t INTMAP_AWAKE = 0x40;
	static const uint8_t INTMAP_INACT = 0x20;
	static const uint8_t INTMAP_ACT = 0x10;
	static const uint8_t INTMAP_FIFO_OVERRUN = 0x08;
	static const uint8_t INTMAP_FIFO_WATERMARK = 0x04;
	static const uint8_t INTMAP_FIFO_READY = 0x02;
	static const uint8_t INTMAP_DATA_READY = 0x01;

	// Power Control
	static const uint8_t POWERCTL_EXT_CLK = 0x40;
	static const uint8_t POWERCTL_WAKEUP = 0x08;
	static const uint8_t POWERCTL_AUTOSLEEP = 0x04;

	static const uint8_t LOWNOISE_NORMAL = 0x0;
	static const uint8_t LOWNOISE_LOW = 0x1;
	static const uint8_t LOWNOISE_ULTRALOW = 0x2;

	static const uint8_t MEASURE_STANDBY = 0x0;
	static const uint8_t MEASURE_MEASUREMENT = 0x2;


private:
	void beginTransaction();
	void endTransaction();

	static void readFifoCallbackInternal(void);
	static void syncCallback(void);

	SPIClass &spi; // Typically SPI or SPI1
	int ss;		// SS or /CS chip select pin. Default: A2
	bool storeTemp = false;
	bool busy = false;
};

/**
 * Class used to store data from the FIFO
 */
class ADXL362Data {
public:
	static const size_t BUF_SIZE = 128;
	static const int STATE_FREE = 0;
	static const int STATE_READING_FIFO = 1;
	static const int STATE_READ_COMPLETE = 2;

	ADXL362Data();
	virtual ~ADXL362Data();

	uint8_t buf[BUF_SIZE];
	bool storeTemp;	// false=XYZ, true=XYZT (includes temperature)
	int state = STATE_FREE;
	size_t bytesRead = 0;

};

#endif /* __ADXL362_H */

