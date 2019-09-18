#ifndef AS5147P_H
#define AS5147P_H
#define LIBRARY_VERSION 1.0.0

#include <SPI.h>

class AS5147P{

	public:

	/**
	 *	Constructor
	 */
	AS5147P(uint8_t arg_cs);

	/**
	 * Initialiser
	 * Sets up the SPI interface
	 */
	void init();

	/**
	 * Closes the SPI connection
	 */
	void close();

	/**
	 * Get the rotation of the sensor relative to the zero position.
	 *
	 * @return {int} between -2^13 and 2^13
	 */
	int getRotation();

	/**
	 * Get the angle in degrees of the sensor relative to the zero position.
	 *
	 * @return {int} between 0 and 360
	 */
	int getDegree();

	/**
	 * Returns the raw rotation directly from the sensor
	 */
	uint16_t getRawRotation();

	/**
	 * returns the value of the state register
	 * @return uint16_t containing flags
	 */
	uint16_t getState();

	/**
	 * Returns the value used for Automatic Gain Control (Part of diagnostic
	 * register)
	 */
	uint8_t getGain();

	/**
	 * Get the 13-bit CORDIC Magnitude
	 * @return uint16_t
	 */
	uint16_t getMagnitude();

	/*
	 * Get and clear the error register by reading it
	 * Error Register: 
	 * Bit 2: PARERR Parity Error
	 * Bit 1: INVCOMM Invalid Command Error
	 * Bit 0: FRERR Framing Error
	 */
	uint16_t getErrors();

	/*
	 * Set the zero position. If no argument is specified, sets zero to getRawRotation.
	 */
	void setZeroPosition();
	void setZeroPosition(uint16_t arg_position);

	
	/*
	 * Returns the current zero position
	 */
	uint16_t getZeroPosition();

	/*
	 * Check if an error has been encountered in the previous reading frame.
	 * @return 0 if no error, 1 if error
	 */
	bool error();

	/*
	 * Read a register from the sensor
	 * Takes the address of the register as a 16 bit uint16_t
	 * Returns the value of the register
	 */
	uint16_t read(uint16_t registerAddress);

	/*
	 * Write to a register
	 * Takes the 16-bit address of the target register and the 16 bit uint16_t of data
	 * to be written to that register
	 * Returns the value of the register after the write has been performed. This
	 * is read back from the sensor to ensure a sucessful write.
	 */
	uint16_t write(uint16_t registerAddress, uint16_t data);

	private:

	uint16_t _zero_position;
	bool _error_flag;
	uint8_t _cs;
	bool _spiCalcEvenParity(uint16_t);
	/*
 	* Takes a 16-bit value to be sent (default is NOP = 0x0000)
 	* Returns the value read during the SPI transfer.
 	* @return uint16_t
	*/
	uint16_t _readWriteSPI(uint16_t = 0x0000);
};
#endif
