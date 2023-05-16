#ifndef __ADXL362_H
#define __ADXL362_H

// Library for the ADXL362 that uses SPI DMI for efficient data transfers
// Github: https://github.com/rickkas7/ADXL362DMA
// License: MIT 

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

/**
 * @brief Class for ADXL362 accelerometer, connected by SPI
 */
class ADXL362DMA {
public:
	enum class SampleRate {
		RATE_3_125_HZ,	//!< 3.125 samples per second (quarter oversampling)
		RATE_6_25_HZ,	//!< 6.25 samples per second (quarter oversampling)
		RATE_12_5_HZ,	//!< 12.5 samples per second (quarter oversampling)
		RATE_25_HZ,		//!< 25 samples per second (quarter oversampling)
		RATE_50_HZ,		//!< 50 samples per second (quarter oversampling)
		RATE_100_HZ,	//!< 100 samples per second (quarter oversampling)
		RATE_200_HZ,	//!< 200 samples per second (half oversampling)
	};

	/**
	 * @brief Initialize the ADXL362 handler object. 
	 * 
	 * @param spi The SPI interface, could be `SPI` or `SPI1`
	 * @param cs The chip select pin to use
	 * 
	 * Usually created as  global variable like this:
	 *
	 * ADXL362 accel(SPI, A2);
	 * 
	 * You can have generally have multiple devices on a single SPI bus, but each SPI device must
	 * have its own CS (chip select) pin. 
	 */
	ADXL362DMA(SPIClass &spi, int cs = A2, SPISettings settings = SPISettings(8*MHZ, MSBFIRST, SPI_MODE0));

	/**
	 * @brief Normally this object is created as a global variable and never deleted
	 */
	virtual ~ADXL362DMA();

	/**
	 * @brief Issue a soft reset call. 
	 * 
	 * It may make a little while for the sensor to respond after
	 * a softReset. readStatus() will return non-zero when ready.
	 */
	void softReset();

	/**
	 * @brief Returns true if the chip can be detected on the SPI bus
	 * 
	 * @return true 
	 * @return false 
	 */
	bool chipDetect();

	/**
	 * @brief Set the sample rate
	 * 
	 * @param rate See SampleRate. Ranges from 3.125 Hz to 200 Hz.
	 */
	void setSampleRate(SampleRate rate);

	/**
	 * @brief Enable or disable measure mode in the power control register
	 *
	 * @param enable true to enable measure mode or false to disable measure mode
	 * 
	 * This a convenience method that's easier than directly manipulating the POWER_CTL register.
	 */
	void setMeasureMode(bool enabled = true);

	/**
	 * @brief Read a single XYZT sample from the current data register
	 *
	 * @param x Filled in with the x acceleration value
	 * @param y Filled in with the y acceleration value
	 * @param z Filled in with the z acceleration value
	 * @param t Filled in with the temperature value
	 * 
	 * If you are continuously reading samples, using the FIFO is more efficient
	 */
	void readXYZT(int &x, int &y, int &z, int &t);

	/**
	 * @brief Reads the status register STATUS
	 * 
	 * @return register value (uint8_t)
	 * 
	 * Address: 0x0B, Reset: 0x40, Name: STATUS
	 *
	 * This is a good way to see if the chip is responding; it normally returns a
	 * non-zero value (0x40 = AWAKE).
	 * 
	 * - bit 7 (MSB, mask 0x80) ERR_USER_REGS
	 * - bit 6 AWAKE
	 * - bit 5 INACT
	 * - bit 4 ACT
	 * - bit 3 FIFO_OVERRUN
	 * - bit 2 FIFO_WATERMARK
	 * - bit 1 FIFO_READY
	 * - bit 0 DATA_READY
	 */
	uint8_t readStatus();

	/**
	 * @brief Reads the number of entries available to read from the FIFO (FIFO_ENTRIES_L and FIFO_ENTRIES_H)
	 * 
	 * @return Number of FIFO entries available (uint16_t)
	 *
	 * Use readFifoAsync to read the bytes. Note: Because this accesses the chip using SPI, you should
	 * check that the chips is not busy using getIsBusy() before calling this if you're mixing calls
	 * to this and readFifoAsync() in your loop().
	 */
	uint16_t readNumFifoEntries();

	/**
	 * @brief Reads entries from the FIFO asynchronously using SPI DMA
	 * 
	 */
	void readFifoAsync(ADXL362Data *data);

	/**
	 * @brief Write the activity threshold register
	 * 
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
	 * @brief Write the activity time register
	 * 
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
	 * @brief Write the inactivity threshold register
	 * 
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
	 * @brief Writes the inactivity time register
	 * 
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
	 * @brief Read the activity/inactivity control register
	 * 
	 * Address: 0x27, Reset: 0x00, Name: ACT_INACT_CTL
	 */
	uint8_t readActivityControl(uint8_t value);


	/**
	 * @brief rite the activity/inactivity control register
	 * 
	 * Address: 0x27, Reset: 0x00, Name: ACT_INACT_CTL
	 *
	 * There is also an overload that takes the parameters broken out into separate parameters for ease of use
	 */
	void writeActivityControl(uint8_t value);


	/**
	 * @brief Write the activity/inactivity control register
	 * 
	 * @param linkLoop one of the following constants: LINKLOOP_DEFAULT, LINKLOOP_LINKED, LINKLOOP_LOOP
	 * @param inactRef inactivity detection uses reference mode (automatically compensates for gravity) if true
	 * @param inactEn inactivity detection is enabled
	 * @param actRef activity detection uses reference mode (automatically compensates for gravity) if true
	 * @param actEn activity detection is enabled
	 * 
	 * Address: 0x27, Reset: 0x00, Name: ACT_INACT_CTL
	 *
	 */
	void writeActivityControl(uint8_t linkLoop, bool inactRef, bool inactEn, bool actRef, bool actEn);


	/**
	 * @brief Read the FIFO control register
	 * Address: 0x28, Reset: 0x00, Name: FIFO_CONTROL
	 */
	uint8_t readFifoControl();

	/**
	 * @brief Writes the FIFO control register
	 * Address: 0x28, Reset: 0x00, Name: FIFO_CONTROL
	 *
	 * It's usually easier to call writeFifoControlAndSamples which sets both registers at once.
	 */
	void writeFifoControl(uint8_t value);

	/**
	 * @brief Writes the FIFO samples register
	 * Address: 0x29, Reset: 0x80, Name: FIFO_SAMPLES
	 *
	 * It's usually easier to call writeFifoControlAndSamples which sets both registers at once.
	 */
	void writeFifoSamples(uint8_t value);

	// samples 0-511, fifoMode = FIFO_DISABLED, etc.
	/**
	 * @brief Writes the FIFO control and samples registers
	 *
	 * samples: Number of samples to store 0-511
	 * storeTemp: Whether to store XYZT (with temperature, if true) or just XYZ data in FIFO
	 * fifoMode: One of the constants: FIFO_DISABLED, FIFO_OLDEST_SAVED, FIFO_STREAM, FIFO_TRIGGERED
	 */
	void writeFifoControlAndSamples(uint16_t samples, bool storeTemp, uint8_t fifoMode);

	/**
	 * @brief Reads the INTMAP1 register
	 * Address: 0x2A, Reset: 0x00, Name: INTMAP1
	 */
	uint8_t readIntmap1();

	/**
	 * @brief Writes the INTMAP1 register
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
	 * @brief Reads the INTMAP2 register
	 * Address: 0x2B, Reset: 0x00, Name: INTMAP2
	 */
	uint8_t readIntmap2();

	/**
	 * @brief Writes the INTMAP2 register
	 * Address: 0x2B, Reset: 0x00, Name: INTMAP2
	 */
	void writeIntmap2(uint8_t value);

	/**
	 * @brief Reads the filter control register
	 * Address: 0x2C, Reset: 0x13, Name: FILTER_CTL
	 */
	uint8_t readFilterControl();

	/**
	 * @brief Writes the filter control register
	 * Address: 0x2C, Reset: 0x13, Name: FILTER_CTL
	 *
	 * There is an overload of writeFilterControl that has the bits broken out into separate parameters
	 * that may be easier to use.
	 */
	void writeFilterControl(uint8_t value);

	/**
	 * @brief Writes the filter control register
	 * Address: 0x2C, Reset: 0x13, Name: FILTER_CTL
	 *
	 * There is an overload of writeFilterControl that takes the single uint8_t value instead of the
	 * broken out values, as well.
	 */
	void writeFilterControl(uint8_t range, bool halfBW, bool extSample, uint8_t odr);

	/**
	 * @brief Reads the power control register
	 * Address: 0x2D, Reset: 0x00, Name: POWER_CTL
	 */
	uint8_t readPowerCtl();


	/**
	 * @brief Writes the power control register
	 * Address: 0x2D, Reset: 0x00, Name: POWER_CTL
	 *
	 * There are also separate calls writeLowNoise and writeMeasureMode to set just those values.
	 *
	 * lowNoise: one of LOWNOISE_NORMAL, LOWNOISE_LOW, LOWNOISE_ULTRALOW
	 * measureMode: one of MEASURE_STANDBY, MEASURE_MEASUREMENT
	 */
	void writePowerCtl(uint8_t value);


	/**
	 * @brief Writes the power control register
	 * 
	 * @param lowNoise one of LOWNOISE_NORMAL, LOWNOISE_LOW, LOWNOISE_ULTRALOW
	 * @param measureMode: one of MEASURE_STANDBY, MEASURE_MEASUREMENT
	 * 
	 * Address: 0x2D, Reset: 0x00, Name: POWER_CTL
	 *
	 * There are also separate calls writeLowNoise and writeMeasureMode to set just those values.
	 *
	 */
	void writePowerCtl(bool extClock, uint8_t lowNoise, bool wakeup, bool autosleep, uint8_t measureMode);

	/**
	 * @brief Set the low noise mode in the power control register
	 *
	 * @param value one of LOWNOISE_NORMAL, LOWNOISE_LOW, LOWNOISE_ULTRALOW
	 */
	void writeLowNoise(uint8_t value);

	/**
	 * @brief Set the measure mode in the power control register
	 *
	 * @param value one of MEASURE_STANDBY, MEASURE_MEASUREMENT
	 */
	void writeMeasureMode(uint8_t value);

	/**
	 * @brief Reads an 8-bit register value
	 *
	 * Most of the calls have easier to use accessors like readStatus() that use this call internally.
	 *
	 * addr: One of the register addresses, such as REG_STATUS
	 */
	uint8_t readRegister8(uint8_t addr);

	/**
	 * @brief Reads an 16-bit register value
	 *
	 * addr: One of the register addresses, such as REG_THRESH_ACT_L. It must be the first of a pair of _L and _H registers.
	 */
	uint16_t readRegister16(uint8_t addr);

	/**
	 * @brief Write an 8-bit register value
	 *
	 * Most of the calls have easier to use accessors like writeIntmap1() that use this call internally.
	 *
	 * @param addr One of the register addresses, such as REG_INTMAP1
	 */
	void writeRegister8(uint8_t addr, uint8_t value);

	/**
	 * @brief Write an 16-bit register value
	 *
	 * Most of the calls have easier to use accessors like writeIntmap1() that use this call internally.
	 *
	 * @param addr One of the register addresses, such as REG_THRESH_ACT_L. It must be the first of a pair of _L and _H registers.
	 */
	void writeRegister16(uint8_t addr, uint16_t value);

	/**
	 * @brief Returns the number of bytes for a full XYZ or XYZT FIFO entry depending on the storeTemp flag
	 */
	size_t getEntrySetSize() { return storeTemp ? 8 : 6; };

	/**
	 * @brief Returns true if a SPI command is currently being handled
	 */
	bool getIsBusy() { return busy; };

	/**
	 * @brief Begin a synchronous SPI DMI transaction
	 */
	void syncTransaction(void *req, void *resp, size_t len);


	// Command bytes
	static const uint8_t CMD_WRITE_REGISTER = 0x0a; 	//!< Write register command
	static const uint8_t CMD_READ_REGISTER = 0x0b; 		//!< Read register command
	static const uint8_t CMD_READ_FIFO = 0x0d;			//!< Read FIFO command

	// Registers
	static const uint8_t REG_DEVID_AD = 0x00;			//!< Device ID register (0xAD)
	static const uint8_t REG_DEVID_MST = 0x01;			//!< MEMS device ID (0x1D)
	static const uint8_t REG_PART_ID = 0x02;			//!< Part ID (0xF2)
	static const uint8_t REG_SILICON_ID = 0x03;			//!< Silicon Revision ID (0x01)
	static const uint8_t REG_XDATA_8 = 0x08;			//!< X axis data, 8 MSB only
	static const uint8_t REG_YDATA_8 = 0x09;			//!< Y axis data, 8 MSB only
	static const uint8_t REG_ZDATA_8 = 0x0a;			//!< Z axis data, 8 MSB only
	static const uint8_t REG_STATUS = 0x0b;				//!< Status register
	static const uint8_t REG_FIFO_ENTRIES_L = 0x0c;		//!< Number of FIFO entries (LSB)
	static const uint8_t REG_FIFO_ENTRIES_H = 0x0d;		//!< Number of FIFO entries (MSB)
	static const uint8_t REG_XDATA_L = 0x0e;			//!< X axis data (LSB)
	static const uint8_t REG_XDATA_H = 0x0f;			//!< X axis data (MSB)
	static const uint8_t REG_YDATA_L = 0x10;			//!< Y axis data (LSB)
	static const uint8_t REG_YDATA_H = 0x11;			//!< Y axis data (MSB)
	static const uint8_t REG_ZDATA_L = 0x12;			//!< Z axis data (LSB)
	static const uint8_t REG_ZDATA_H = 0x13;			//!< Z axis data (MSB)
	static const uint8_t REG_TDATA_L = 0x14;			//!< Temperature data (LSB)
	static const uint8_t REG_TDATA_H = 0x15;			//!< Temperature data (MSB)
	static const uint8_t REG_SOFT_RESET = 0x1f;			//!< Soft reset register
	static const uint8_t REG_THRESH_ACT_L = 0x20;		//!< Activity threshold (LSB)
	static const uint8_t REG_THRESH_ACT_H = 0x21;		//!< Activity threshold (MSB)
	static const uint8_t REG_TIME_ACT = 0x22;			///!< Activity time register
	static const uint8_t REG_THRESH_INACT_L = 0x23;		//!< Inactivity threshold (LSB)
	static const uint8_t REG_THRESH_INACT_H = 0x24;		//!< Inactivity threshold (MSB)
	static const uint8_t REG_TIME_INACT_L = 0x25;		//!< Time inactivity register (LSB)
	static const uint8_t REG_TIME_INACT_H = 0x26;		//!< Time inactivity register (MSB)
	static const uint8_t REG_ACT_INACT_CTL = 0x27;		//!< Activity/inactivity control register
	static const uint8_t REG_FIFO_CONTROL = 0x28;		//!< FIFO control register
	static const uint8_t REG_FIFO_SAMPLES = 0x29;		//!< Number of samples to store in FIFO 
	static const uint8_t REG_FIFO_INTMAP1 = 0x2a;		//!< Interrupt mapping register 1
	static const uint8_t REG_FIFO_INTMAP2 = 0x2b;		//!< Interrupt mapping register 2
	static const uint8_t REG_FILTER_CTL = 0x2c;			//!< Filter control register
	static const uint8_t REG_POWER_CTL = 0x2d;			//!< Power control register
	static const uint8_t REG_SELF_TEST = 0x2e;			//!< Self test register


	// Status bits in status register
	static const uint8_t STATUS_ERR_USER_REGS = 0x80;	//!< SEU error detect
	static const uint8_t STATUS_AWAKE = 0x40;			//!< AWAKE (1) or inactive (0) state
	static const uint8_t STATUS_INACT = 0x20;			//!< Inactivity or free fall condition
	static const uint8_t STATUS_ACT = 0x10;				//!< Activity detected
	static const uint8_t STATUS_FIFO_OVERRUN = 0x08;	//!< FIFO overflow
	static const uint8_t STATUS_FIFO_WATERMARK = 0x04;	//!< FIFO reached watermark
	static const uint8_t STATUS_FIFO_READY = 0x02;		//!< FIFO has at least one sample available
	static const uint8_t STATUS_DATA_READ = 0x01;		//!< New sample available to read

	// Activity/inactivity control register
	static const uint8_t LINKLOOP_DEFAULT = 0x0;		//!< Activity and inactivity detection enabled
	static const uint8_t LINKLOOP_LINKED = 0x1;			//!< Activity and inactivity sequentially linked
	static const uint8_t LINKLOOP_LOOP = 0x3;			//!< Sequentially linked, interrupts do not need be serviced 

	static const uint8_t ACTIVITY_INACT_REF = 0x08;		//!< Inactivity in referenced mode (1) or absolute mode (0)
	static const uint8_t ACTIVITY_INACT_EN = 0x04;		//!< Inactivity enable (1)
	static const uint8_t ACTIVITY_ACT_REF = 0x02;		//!< Activity in referenced mode (1) or absolute mode (0)
	static const uint8_t ACTIVITY_ACT_EN = 0x01;		//!< Activity enable (1)

	// Range in Filter Control Register
	static const uint8_t RANGE_2G 	= 0x0;				//!< Range +/- 2g (default)
	static const uint8_t RANGE_4G 	= 0x1;				//!< Range +/- 4g
	static const uint8_t RANGE_8G 	= 0x2;				//!< Range +/- 8g

	static const uint8_t HALF_BW_MASK = 0x80;			//!< Mask value for HALF_BW bit in FILTER_CTL register
	static const uint8_t ODR_MASK = 0x80;				//!< Mask value for ODR bits in FILTER_CTL register

	// Output Data Rate in Filter Control Register
	static const uint8_t ODR_12_5 	= 0x0;				//!< Output data rate 12.5 Hz
	static const uint8_t ODR_25 	= 0x1;				//!< Output data rate 25 Hz
	static const uint8_t ODR_50 	= 0x2;				//!< Output data rate 50 Hz
	static const uint8_t ODR_100 	= 0x3;				//!< Output data rate 100 Hz (default)
	static const uint8_t ODR_200 	= 0x4;				//!< Output data rate 200 Hz
	static const uint8_t ODR_400 	= 0x5;				//!< Output data rate 400 Hz

	// FIFO mode
	static const uint8_t FIFO_DISABLED 	= 0x0;			//!< FIFO disabled (default)
	static const uint8_t FIFO_OLDEST_SAVED = 0x1;		//!< FIFO oldest saved
	static const uint8_t FIFO_STREAM 		= 0x2;		//!< FIFO stream mode
	static const uint8_t FIFO_TRIGGERED 	= 0x3;		//!< FIFO triggered mode

	// INTMAP1 and INTMAP2
	static const uint8_t INTMAP_INT_LOW = 0x80;			//!< INT is active low
	static const uint8_t INTMAP_AWAKE = 0x40;			//!< Map awake status to INT
	static const uint8_t INTMAP_INACT = 0x20;			//!< Map inactivity status to INT
	static const uint8_t INTMAP_ACT = 0x10;				//!< Map activity status to INT
	static const uint8_t INTMAP_FIFO_OVERRUN = 0x08;	//!< Map FIFO overrun to INT
	static const uint8_t INTMAP_FIFO_WATERMARK = 0x04;	//!< Map FIFO watermark to INT
	static const uint8_t INTMAP_FIFO_READY = 0x02;		//!< Map FIFO ready to INT 
	static const uint8_t INTMAP_DATA_READY = 0x01;		//!< Map FIFI data ready to INT

	// Power Control
	static const uint8_t POWERCTL_EXT_CLK = 0x40;		//!< Use external clock
	static const uint8_t POWERCTL_WAKEUP = 0x08;		//!< Wakeup mode
	static const uint8_t POWERCTL_AUTOSLEEP = 0x04;		//!< Autosleep

	static const uint8_t LOWNOISE_NORMAL = 0x0;			//!< Normal operation (default)
	static const uint8_t LOWNOISE_LOW = 0x1;			//!< Low noise mode
	static const uint8_t LOWNOISE_ULTRALOW = 0x2;		//!< Ultra low noise mode

	static const uint8_t MEASURE_STANDBY = 0x0;			//!< Standby mode
	static const uint8_t MEASURE_MEASUREMENT = 0x2;		//!< Measurement mode


private:
	/**
	 * @brief Called before any SPI transaction. Calls spi.beginTransaction() and clears the CS pin.
	 */
	void beginTransaction();

	/**
	 * @brief Called after any SPI transaction. Calls spi.endTransaction() and sets the CS pin high.
	 */
	void endTransaction();

	static void readFifoCallbackInternal(void);
	static void syncCallback(void);

	SPIClass &spi; //!< SPI interface, typically SPI or SPI1
	int cs;		//!<  CS chip select pin. Default: A2
	SPISettings settings; //!<  SPI settings (mode, bit order, speed)
	bool storeTemp = false; //!< Whether to store temperature 
	bool busy = false; //!< Used for busy detection 
};

/**
 * @brief Class used to store data from the FIFO
 * 
 * Templated version that allows the buffer size to be specified. The ADXL362
 * supports up to 511 samples of either 6 bytes (no temperature) or 8 bytes
 * (with temperature), so the buffer could be up to 512 * 8 = 4096 bytes.
 * 
 * Since this class is used for asynchronous reads, 
 */
template <size_t BUF_SIZE> 
class ADXL362DataEx {
public:
	static const int STATE_FREE = 0;			//!< Not currently in use
	static const int STATE_READING_FIFO = 1;	//!< Reading FIFO by SPI DMA
	static const int STATE_READ_COMPLETE = 2;	//!< Reading complete

	/**
	 * @brief Constructor
	 */
	ADXL362DataEx() : bufSize(BUF_SIZE) {};

	/**
	 * @brief Destructor
	 */
	virtual ~ADXL362DataEx() {};

	/**
	 * @brief Buffer size. Should be a multiple of the entry size
	 * 
	 * The getEntrySize() method will return either 6 (without temperature) or 8 (with temperature)
	 */
	uint8_t buf[BUF_SIZE];

	/**
	 * @brief Whether to store temperature or not
	 * 
	 * false=XYZ (6 bytes per sample), true=XYZT (includes temperature, 8 bytes per sample)
	 */
	bool storeTemp;

	/**
	 * @brief State of this object (free, reading, or complete)
	 */
	int state = STATE_FREE;

	/**
	 * @brief Number of bytes (not samples!) read on completion
	 */
	size_t bytesRead = 0;

	/**
	 * @brief Size of buf
	 * 
	 */
	size_t bufSize;

};

/**
 * @brief Class used to store data from the FIFO
 * 
 * Backward compatible to old API, fixed at a maximum of 128 bytes. Use ADXL362DataEx to set a custom buffer size.
 */
class ADXL362Data : public ADXL362DataEx<128> {
};


#endif /* __ADXL362_H */

