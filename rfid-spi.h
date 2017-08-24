#ifndef _RFID_SPI_H_
#define _RFID_SPI_H_

#include <cpu.h>
#include <contiki.h>
#include <stdio.h> 
#include "spi-arch.h"
#include "dev/spi.h"
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <gpio.h>
#include <inttypes.h> //for printf int32_t

#define 	SPI_CS      			0x01        //PC0
#define 	SPI_MOSI    			0x10        //PA4
#define 	SPI_MISO    			0x02        //PB1
#define 	SPI_Ck      			0x04        //PA2

#define 	GPIO_PIN_6              0x40        //SDA (SLAVE SELECT)

#define 	MAX_LEN 				16   		// Maximum length of the array
#define 	MI_OK                 	0
#define 	MI_NOTAGERR           	1
#define 	MI_ERR                	2

#define 	PICC_REQIDL 	      	0x26		// Area of ​​the antenna is not trying to get into the idle state

#define     Reserved00            	0x00    
   
#define     CommIEnReg            	0x02    
#define     DivlEnReg             	0x03    
#define     CommIrqReg            	0x04    

#define 	PCD_IDLE             	0x00		// No action; And cancel the command
#define 	PCD_AUTHENT          	0x0E		// authentication key
#define 	PCD_RECEIVE          	0x08		// receiving data
#define 	PCD_TRANSMIT         	0x04		// Send data
#define 	PCD_TRANSCEIVE       	0x0C		// Send and receive data
#define 	PCD_RESETPHASE       	0x0F		// reset
#define 	PCD_CALCCRC          	0x03		// calculate CRC
#define     Reserved01            	0x0F
//Page 1:Command     
#define     Reserved10            	0x10	
#define     Reserved11            	0x1A
#define     Reserved12            	0x1B
#define     MifareReg             	0x1C
#define     Reserved13            	0x1D
#define     Reserved14            	0x1E
//Page 2:CFG    
#define     Reserved20          	0x20  
#define     Reserved21           	0x23
#define		CRCResultRegM			0x21		// shows the MSB and LSB values of the CRC calculation
// MFRC522 commands. Described in chapter 10 of the datasheet.
#define		PCD_Idle				0x00		// no action, cancels current command execution
#define		PCD_Mem					0x01		// stores 25 bytes into the internal buffer
#define		PCD_GenerateRandomID	0x02		// generates a 10-byte random ID number
#define		PCD_CalcCRC				0x03		// activates the CRC coprocessor or performs a self test
#define		PCD_Transmit			0x04		// transmits data from the FIFO buffer
#define		PCD_NoCmdChange			0x07		// no command change, can be used to modify the CommandReg register bits without affecting the command, for example, the PowerDown bit
#define		PCD_Receive				0x08		// activates the receiver circuits
#define		PCD_Transceive 			0x0C		// transmits data from FIFO buffer to antenna and automatically activates the receiver after transmission
#define		pcd_mf_authent 			0x0E		// performs the MIFARE standard authentication as a reader
#define		PCD_SoftReset			0x0F		// resets the MFRC522
#define     TxAutoReg				0x15
#define 	delay_ms(i) (ti_lib_cpu_delay(8000 * (i)))

typedef enum  {
		// Page 0: Command and status
		//						  0x00			// reserved for future use
		CommandReg				= 0x01 ,	// starts and stops command execution
		ComIEnReg				= 0x02 ,	// enable and disable interrupt request control bits
		DivIEnReg				= 0x03 ,	// enable and disable interrupt request control bits
		ComIrqReg				= 0x04 ,	// interrupt request bits
		DivIrqReg				= 0x05 ,	// interrupt request bits
		ErrorReg				= 0x06 ,	// error bits showing the error status of the last command executed 
		Status1Reg				= 0x07 ,	// communication status bits
		Status2Reg				= 0x08 ,	// receiver and transmitter status bits
		FIFODataReg				= 0x09 ,	// input and output of 64 byte FIFO buffer
		FIFOLevelReg			= 0x0A ,	// number of bytes stored in the FIFO buffer
		WaterLevelReg			= 0x0B ,	// level for FIFO underflow and overflow warning
		ControlReg				= 0x0C ,	// miscellaneous control registers
		BitFramingReg			= 0x0D ,	// adjustments for bit-oriented frames
		CollReg					= 0x0E ,	// bit position of the first bit-collision detected on the RF interface
		//						  0x0F			// reserved for future use
		
		// Page 1: Command
		// 						  0x10			// reserved for future use
		ModeReg					= 0x11 ,	// defines general modes for transmitting and receiving 
		TxModeReg				= 0x12 ,	// defines transmission data rate and framing
		RxModeReg				= 0x13 ,	// defines reception data rate and framing
		TxControlReg			= 0x14 ,	// controls the logical behavior of the antenna driver pins TX1 and TX2
		TxASKReg				= 0x15 ,	// controls the setting of the transmission modulation
		TxSelReg				= 0x16 ,	// selects the internal sources for the antenna driver
		RxSelReg				= 0x17 ,	// selects internal receiver settings
		RxThresholdReg			= 0x18 ,	// selects thresholds for the bit decoder
		DemodReg				= 0x19 ,	// defines demodulator settings
		// 						  0x1A			// reserved for future use
		// 						  0x1B			// reserved for future use
		MfTxReg					= 0x1C ,	// controls some MIFARE communication transmit parameters
		MfRxReg					= 0x1D ,	// controls some MIFARE communication receive parameters
		// 						  0x1E			// reserved for future use
		SerialSpeedReg			= 0x1F ,	// selects the speed of the serial UART interface
		
		// Page 2: Configuration
		// 						  0x20			// reserved for future use
		CRCResultRegH			= 0x21 ,	// shows the MSB and LSB values of the CRC calculation
		CRCResultRegL			= 0x22 ,
		// 						  0x23			// reserved for future use
		ModWidthReg				= 0x24 ,	// controls the ModWidth setting?
		// 						  0x25			// reserved for future use
		RFCfgReg				= 0x26 ,	// configures the receiver gain
		GsNReg					= 0x27 ,	// selects the conductance of the antenna driver pins TX1 and TX2 for modulation 
		CWGsPReg				= 0x28 ,	// defines the conductance of the p-driver output during periods of no modulation
		ModGsPReg				= 0x29 ,	// defines the conductance of the p-driver output during periods of modulation
		TModeReg				= 0x2A ,	// defines settings for the internal timer
		TPrescalerReg			= 0x2B ,	// the lower 8 bits of the TPrescaler value. The 4 high bits are in TModeReg.
		TReloadRegH				= 0x2C ,	// defines the 16-bit timer reload value
		TReloadRegL				= 0x2D ,
		TCounterValueRegH		= 0x2E ,	// shows the 16-bit timer value
		TCounterValueRegL		= 0x2F ,
		
		// Page 3: Test Registers
		// 						  0x30			// reserved for future use
		TestSel1Reg				= 0x31 ,	// general test signal configuration
		TestSel2Reg				= 0x32 ,	// general test signal configuration
		TestPinEnReg			= 0x33 ,	// enables pin output driver on pins D1 to D7
		TestPinValueReg			= 0x34 ,	// defines the values for D1 to D7 when it is used as an I/O bus
		TestBusReg				= 0x35 ,	// shows the status of the internal test bus
		AutoTestReg				= 0x36 ,	// controls the digital self-test
		VersionReg				= 0x37 ,	// shows the software version
		AnalogTestReg			= 0x38 ,	// controls the pins AUX1 and AUX2
		TestDAC1Reg				= 0x39 ,	// defines the test value for TestDAC1
		TestDAC2Reg				= 0x3A ,	// defines the test value for TestDAC2
		TestADCReg				= 0x3B 		// shows the value of ADC I and Q channels
		// 						  0x3C			// reserved for production tests
		// 						  0x3D			// reserved for production tests
		// 						  0x3E			// reserved for production tests
		// 						  0x3F			// reserved for production tests
	}pcd_register;
	
	// Commands sent to the PICC.
	typedef enum  {
		// The commands used by the PCD to manage communication with several PICCs (ISO 14443-3, Type A, section 6.4)
		PICC_CMD_REQA			= 0x26,		// REQuest command, Type A. Invites PICCs in state IDLE to go to READY and prepare for anticollision or selection. 7 bit frame.
		PICC_CMD_WUPA			= 0x52,		// Wake-UP command, Type A. Invites PICCs in state IDLE and HALT to go to READY(*) and prepare for anticollision or selection. 7 bit frame.
		PICC_CMD_CT				= 0x88,		// Cascade Tag. Not really a command, but used during anti collision.
		PICC_CMD_SEL_CL1		= 0x93,		// Anti collision/Select, Cascade Level 1
		PICC_CMD_SEL_CL2		= 0x95,		// Anti collision/Select, Cascade Level 2
		PICC_CMD_SEL_CL3		= 0x97,		// Anti collision/Select, Cascade Level 3
		PICC_CMD_HLTA			= 0x50,		// HaLT command, Type A. Instructs an ACTIVE PICC to go to state HALT.
		PICC_CMD_RATS           = 0xE0,     // Request command for Answer To Reset.
		// The commands used for MIFARE Classic (from http://www.mouser.com/ds/2/302/MF1S503x-89574.pdf, Section 9)
		// Use pcd_mf_authent to authenticate access to a sector, then use these commands to read/write/modify the blocks on the sector.
		// The read/write commands can also be used for MIFARE Ultralight.
		PICC_CMD_MF_AUTH_KEY_A	= 0x60,		// Perform authentication with Key A
		PICC_CMD_MF_AUTH_KEY_B	= 0x61,		// Perform authentication with Key B
		PICC_CMD_MF_READ		= 0x30,		// Reads one 16 byte block from the authenticated sector of the PICC. Also used for MIFARE Ultralight.
		PICC_CMD_MF_WRITE		= 0xA0,		// Writes one 16 byte block to the authenticated sector of the PICC. Called "COMPATIBILITY WRITE" for MIFARE Ultralight.
		PICC_CMD_MF_DECREMENT	= 0xC0,		// Decrements the contents of a block and stores the result in the internal data register.
		PICC_CMD_MF_INCREMENT	= 0xC1,		// Increments the contents of a block and stores the result in the internal data register.
		PICC_CMD_MF_RESTORE		= 0xC2,		// Reads the contents of a block into the internal data register.
		PICC_CMD_MF_TRANSFER	= 0xB0,		// Writes the contents of the internal data register to a block.
		// The commands used for MIFARE Ultralight (from http://www.nxp.com/documents/data_sheet/MF0ICU1.pdf, Section 8.6)
		// The PICC_CMD_MF_READ and PICC_CMD_MF_WRITE can also be used for MIFARE Ultralight.
		PICC_CMD_UL_WRITE		= 0xA2		// Writes one 4 byte page to the PICC.
	}PICC_Command;
	
	typedef enum  {
		PICC_TYPE_UNKNOWN		,
		PICC_TYPE_ISO_14443_4	,	// PICC compliant with ISO/IEC 14443-4 
		PICC_TYPE_ISO_18092		, 	// PICC compliant with ISO/IEC 18092 (NFC)
		PICC_TYPE_MIFARE_MINI	,	// MIFARE Classic protocol, 320 bytes
		PICC_TYPE_MIFARE_1K		,	// MIFARE Classic protocol, 1KB
		PICC_TYPE_MIFARE_4K		,	// MIFARE Classic protocol, 4KB
		PICC_TYPE_MIFARE_UL		,	// MIFARE Ultralight or Ultralight C
		PICC_TYPE_MIFARE_PLUS	,	// MIFARE Plus
		PICC_TYPE_MIFARE_DESFIRE,	// MIFARE DESFire
		PICC_TYPE_TNP3XXX		,	// Only mentioned in NXP AN 10833 MIFARE Type Identification Procedure
		PICC_TYPE_NOT_COMPLETE	= 0xff	// SAK indicates uid_struct is not complete.
	}picc_type;
	
	// Return codes from the functions in this class. Remember to update Getstatus_codeName() if you add more.
	// last value set to 0xff, then compiler uses less ram, it seems some optimisations are triggered
	typedef enum  {
		STATUS_OK				,	// Success
		STATUS_ERROR			,	// Error in communication
		STATUS_COLLISION		,	// Collission detected
		STATUS_TIMEOUT			,	// Timeout in communication.
		STATUS_NO_ROOM			,	// A buffer is not big enough.
		STATUS_INTERNAL_ERROR	,	// Internal error in the code. Should not happen ;-)
		STATUS_INVALID			,	// Invalid argument.
		STATUS_CRC_WRONG		,	// The CRC_A does not match
		STATUS_MIFARE_NACK		= 0xff	// A MIFARE PICC responded with NAK.
	}status_code;
	
	typedef enum  {
		MF_ACK					= 0xA,		// The MIFARE Classic uses a 4 bit ACK/NAK. Any other value than 0xA is NAK.
		MF_KEY_SIZE				= 6			// A Mifare Crypto1 key is 6 bytes.
	}MIFARE_Misc;

	// A struct used for passing a MIFARE Crypto1 key
	typedef struct {
		uint8_t		keyByte[MF_KEY_SIZE];
	} MIFARE_Key;

	// A struct used for passing the uid_struct of a PICC.
	typedef struct {
		uint8_t		size;			// Number of bytes in the UID. 4, 7 or 10.
		uint8_t		uidByte[10];
		uint8_t		sak;			// The SAK (Select acknowledge) byte returned from the PICC after successful selection.
	} uid_struct;

void write_bytes_mfrc522( pcd_register reg, uint8_t count, uint8_t *values);
void write_mfrc522(uint8_t adr, uint8_t val);
uint8_t read_mfrc522(uint8_t dev_cmd);
void read_fifo_mfrc522( pcd_register reg, uint8_t count, uint8_t *values, uint8_t rx_align);
picc_type picc_get_type(uint8_t sak);
void pcd_set_register_bit_mask(pcd_register reg, uint8_t mask);
status_code pcd_calculate_crc(uint8_t *data, uint8_t length, uint8_t *result);
bool pcd_reset();
status_code pcd_communicate_tith_picc(uint8_t command, uint8_t wait_irq, uint8_t *send_data, uint8_t send_len, uint8_t *back_data, uint8_t *back_len, uint8_t *valid_bits, uint8_t rx_align, bool check_crc);
status_code pcd_transceive_data(uint8_t *send_data, uint8_t send_len, uint8_t *back_data, uint8_t *back_len, uint8_t *valid_bits, uint8_t rx_align, bool check_crc);
status_code mifare_read( uint8_t blockAddr,uint8_t *buffer,uint8_t *bufferSize);
void pcd_clear_register_bit_mask(pcd_register reg, uint8_t mask);
void pcd_stop_cryptol();
status_code picc_select( uid_struct *uid, uint8_t valid_bits);
status_code picc_halt_a();
void picc_get_type_name(picc_type piccType);
status_code picc_reqa_or_wupa( uint8_t command, uint8_t *bufferATQA, uint8_t *bufferSize);
status_code picc_request_a( uint8_t *bufferATQA,uint8_t *bufferSize);
bool picc_is_new_card_present();
bool picc_read_card_serial();
uint32_t picc_dump_details_to_serial(uid_struct *uid);
status_code pcd_authenticate (uint8_t command, uint8_t blockAddr, MIFARE_Key *key, uid_struct *uid);
void get_status_code_name (status_code code);
void picc_dump_mifare_classic_sector_to_serial (uid_struct *uid, MIFARE_Key *key, uint8_t sector);
void picc_dump_mifare_ultralight_to_serial();
void picc_dump_mifare_classic_to_serial( uid_struct *uid, picc_type piccType, MIFARE_Key *key);
uint32_t picc_dump_to_serial(uid_struct *uid);
void antenna_on();
bool pcd_initialization();
#endif