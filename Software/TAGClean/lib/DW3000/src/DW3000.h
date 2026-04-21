/*
 * Copyright (c) 2023 by Philipp Hafkemeyer
 * Qorvo DW3000 library for Arduino
 * 
 * This project is licensed under the GNU GPL v3.0 License.
 * you may not use this file except in compliance with the License.
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
*/

#ifndef Morse_h
#define Morse_h

#include "Arduino.h"
#include "DW3000Constants.h"


class DW3000Class {
	public:
		// Constructor for per-instance pin assignment
		DW3000Class(uint8_t csPin, uint8_t rstPin, uint8_t irqPin);

		// Per-instance config
		int config[9]; 

		// Chip Setup
		void spiSelect();
		void begin();
		void init();

		void writeSysConfig();
		void configureAsTX();
		void setupGPIO();

		// Double-Sided Ranging
		void ds_sendFrame(int stage);
		void ds_sendRTInfo(int t_roundB, int t_replyB);
		int  ds_processRTInfo(int t_roundA, int t_replyA, int t_roundB, int t_replyB, int clock_offset);
		int  ds_getStage();
		bool ds_isErrorFrame();
		void ds_sendErrorFrame();

		// Radio Settings
		void setChannel(uint8_t data);
		void setPreambleLength(uint8_t data);
		void setPreambleCode(uint8_t data);
		void setPACSize(uint8_t data);
		void setDatarate(uint8_t data);
		void setPHRMode(uint8_t data);
		void setPHRRate(uint8_t data);

		// Protocol Settings
		void setMode(int mode);
		void setTXFrame(unsigned long long frame_data);
		void setFrameLength(int frame_len);
		void setTXAntennaDelay(int delay);
		void setSenderID(int senderID);
		void setDestinationID(int destID);

		// Status Checks
		int receivedFrameSucc();
		int sentFrameSucc();
		int getSenderID();
		int getDestinationID();
		bool checkForIDLE();
		bool checkSPI();

		// Radio Analytics
		double getSignalStrength();
		double getFirstPathSignalStrength();
		int getTXAntennaDelay();
		long double getClockOffset();
		long double getClockOffset(int32_t ext_clock_offset);
		int getRawClockOffset();
		float getTempInC();

		unsigned long long readRXTimestamp();
		unsigned long long readTXTimestamp();
		
		// Chip Interaction
		uint32_t write(int base, int sub, uint32_t data, int data_len);
		uint32_t write(int base, int sub, uint32_t data);

		uint32_t read(int base, int sub);
		uint8_t read8bit(int base, int sub);
		uint32_t readOTP(uint8_t addr);
		
		// Delayed Sending Settings
		void writeTXDelay(uint32_t delay);
		void prepareDelayedTX();

		// Radio Stage Settings / Transfer and Receive Modes
		void delayedTXThenRX();
		void delayedTX();
		void standardTX();
		void standardRX();
		void TXInstantRX();

		// DW3000 Firmware Interaction
		void softReset();
		void hardReset();
		void clearSystemStatus();

		// Hardware Status Information
		void pullLEDHigh(int led);
		void pullLEDLow(int led);

		// Calculation and Conversion
		double convertToCM(int dw3000_ps_units);
		void calculateTXRXdiff();

		// Printing
		void printRoundTripInformation();
		void printDouble(double val, unsigned int precision, bool linebreak);

	private:
		uint8_t _csPin, _rstPin, _irqPin;
		// Per-instance state (avoid globals so multiple instances don't clash)
		int _senderID = 0;
		int _destinationID = 0;
		int _ledStatus = 0;
		uint16_t _antennaDelay; // persisted per instance; set in ctor

		// Single Bit Settings
		void setBit(int reg_addr, int sub_addr, int shift, bool b);
		void setBitLow(int reg_addr, int sub_addr, int shift);
		void setBitHigh(int reg_addr, int sub_addr, int shift);

		// Fast Commands
		void writeFastCommand(int cmd);

		// SPI Interaction
		uint32_t readOrWriteFullAddress(uint32_t base, uint32_t sub, uint32_t data, uint32_t data_len, uint32_t readWriteBit);
		uint32_t sendBytes(int b[], int lenB, int recLen); 
		
		//Soft Reset Helper Method
		void clearAONConfig();

		// Other Helper Methods
		unsigned int countBits(unsigned int number);
		int checkForDevID();
};

#endif