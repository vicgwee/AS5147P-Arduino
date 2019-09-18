#include "Arduino.h"

#include <AS5147P.h>

//#define AS5147P_DEBUG

// 8MHz clock (AMS should be able to accept up to 10MHz)
#define SPI_SETTINGS SPISettings(8000000, MSBFIRST, SPI_MODE1)
#define READCOMMAND 0b0100000000000000 // PAR=0 R/W=R
#define WRITECOMMAND  0b0000000000000000 // PAR=0 R/W=W

namespace AS5147P_REGISTERS
{
//Volatile Registers
const int CLEAR_ERROR_FLAG              = 0x0001;
//const int PROGRAMMING_CONTROL           = 0x0003;
const int DIAG_AGC                      = 0x3FFC;
const int MAGNITUDE                     = 0x3FFD;
//const int ANGLE_UNCOM                   = 0x3FFE;
const int ANGLE                         = 0x3FFF;

/*Non-Volatile Registers
//not implemented
const int OTP_ZERO_POS_HIGH				= 0x0016;
const int OTP_ZERO_POS_LOW				= 0x0017;
const int OTP_SETTINGS1					= 0x0018;
const int OTP_SETTINGS2					= 0x0019;
const int OTP_REDUNDANCY				= 0x001A;
*/

}

/**
 * Constructor
 */
AS5147P::AS5147P(uint8_t arg_cs){
	_cs = arg_cs;
}


/**
 * Initialiser
 * Sets up the SPI interface
 */
void AS5147P::init(){
	//setup pins
	pinMode(_cs, OUTPUT);
	digitalWrite(_cs, HIGH);

	//SPI has an internal SPI-device counter, it is possible to call "begin()" from different devices
	SPI.begin();
}

/**
 * Closes the SPI connection
 * SPI has an internal SPI-device counter, for each init()-call the close() function must be called exactly 1 time
 */
void AS5147P::close(){
	SPI.end();
}

/**
 * Utility function used to calculate even parity of uint16_t
 */
bool AS5147P::_spiCalcEvenParity(uint16_t value){
	bool even = 0;
	uint8_t i;

	for (i = 0; i < 16; i++)
	{
		if (value & (0x0001 << i) )
		{
			even = !even;
		}
	}
	return even;
}



/**
 * Get the rotation of the sensor relative to the zero position.
 *
 * @return {int} between -2^13 and 2^13
 */
int AS5147P::getRotation(){
	int rotation;

	rotation = AS5147P::getRawRotation() - _zero_position;
	if(rotation > 8191) rotation = -((0x3FFF)-rotation); //more than -180


	return rotation;
}

/**
 * Get the angle in degrees (0-359) of the sensor relative to the zero position, but -1 if error flag is set.
 *
 * @return {int} between -1 and 359
 */
int AS5147P::getDegree(){
	uint16_t rotation;
	
	rotation = AS5147P::getRawRotation() - _zero_position;

	if(_error_flag){
		return -1;
	}
	else if (rotation >> 15){  //rotation is negative
		rotation += 0x3FFF;
	}
	return map(rotation, 0, 0x4000, 0, 360);
}

/**
 * Returns the raw rotation directly from the sensor
 */
uint16_t AS5147P::getRawRotation(){
	return AS5147P::read(AS5147P_REGISTERS::ANGLE);
}

/**
 * returns the value of the state register
 * @return 16 bit uint16_t containing flags
 */
uint16_t AS5147P::getState(){
	return AS5147P::read(AS5147P_REGISTERS::DIAG_AGC);
}


/**
 * Returns the value used for Automatic Gain Control (Part of diagnostic
 * register)
 */
uint8_t AS5147P::getGain(){
	uint16_t data = AS5147P::getState();
	return (uint8_t) data & 0xFF;
}

/**
 * Get the 13-bit CORDIC Magnitude
 * @return uint16_t
 */
uint16_t AS5147P::getMagnitude(){
	return AS5147P::read(AS5147P_REGISTERS::MAGNITUDE);
}

/*
 * Get and clear the error register by reading it
 */
uint16_t AS5147P::getErrors(){
	return AS5147P::read(AS5147P_REGISTERS::CLEAR_ERROR_FLAG);
}

/*
 * Set the zero position
 */
void AS5147P::setZeroPosition(){_zero_position = getRawRotation();}

void AS5147P::setZeroPosition(uint16_t arg_position){
	_zero_position = arg_position % 0x3FFF;
}

/*
 * Returns the current zero position
 */
uint16_t AS5147P::getZeroPosition(){
	return _zero_position;
}

/*
 * Check if an error has been encountered.
 */
bool AS5147P::error(){
	return _error_flag;
}

/*
 * Read a register from the sensor
 * Takes the address of the register as a 16 bit uint16_t
 * Returns the value of the register
 */
uint16_t AS5147P::read(uint16_t registerAddress){
	uint16_t command = registerAddress | READCOMMAND;

	//Add a parity bit on the the MSB
	command |= ((uint16_t)_spiCalcEvenParity(command)<<15);


#ifdef AS5147P_DEBUG
	Serial.print(F("Read (0x"));
	Serial.print(registerAddress, HEX);
	Serial.print(F(") with command: 0b"));
	Serial.println(command, HEX);
#endif

	//SPI - begin transaction
	SPI.beginTransaction(SPI_SETTINGS);

	//Send the command
	_readWriteSPI(command);

	//Now read the response
	uint16_t result = _readWriteSPI();

	//SPI - end transaction
	SPI.endTransaction();

#ifdef AS5147P_DEBUG
	Serial.print(F("Read returned: "));
	Serial.print(result, BIN);
#endif

	//Check if the error bit is set
	if (result & 0x4000) {
#ifdef AS5147P_DEBUG
		Serial.println(F("Setting error bit"));
#endif
		_error_flag = true;
	}
	else {
		_error_flag = false;
	}

	//Return the data, stripping the parity and error bits
	return result & ~0xC000;
}


/*
 * Write to a register
 * Takes the 16-bit  address of the target register and the 16 bit uint16_t of data
 * to be written to that register
 * Returns the value of the register after the write has been performed. This
 * is read back from the sensor to ensure a sucessful write.
 */
uint16_t AS5147P::write(uint16_t registerAddress, uint16_t data) {

	uint16_t command = registerAddress | WRITECOMMAND;
	uint16_t data_to_send = data | WRITECOMMAND;

	//Add a parity bit on the the MSB
	command |= ((uint16_t)_spiCalcEvenParity(command)<<15);
	data_to_send |= ((uint16_t)_spiCalcEvenParity(data_to_send)<<15);

#ifdef AS5147P_DEBUG
	Serial.print(F("Write (0x"));
	Serial.print(registerAddress, HEX);
	Serial.print(F(") with command: 0b"));
	Serial.println(command, BIN);
#endif

	//SPI - begin transaction
	SPI.beginTransaction(SPI_SETTINGS);

	//Start the write command with the target address
	_readWriteSPI(command);
	
#ifdef AS5147P_DEBUG
	Serial.print(F("Sending data to write: "));
	Serial.println(data_to_send, BIN);
#endif

	//Now send the data packet
	_readWriteSPI(data_to_send);
	
	//Send a NOP to read the new data in the register
	uint16_t result = _readWriteSPI();

	//SPI - end transaction
	SPI.endTransaction();

	//Return the data, stripping the parity and error bits
	return result & ~0xC000;
}

/*
 * Takes a 16-bit value to be sent (default is NOP = 0x0000)
 * Returns the value read during the SPI transfer.
 * @return uint16_t
 */
uint16_t AS5147P::_readWriteSPI(uint16_t data_out){
	digitalWrite(_cs,LOW);
	uint16_t data_in = SPI.transfer16(data_out);
	digitalWrite(_cs,HIGH);
	return data_in;
}
