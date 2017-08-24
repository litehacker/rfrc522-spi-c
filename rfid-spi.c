#include "rfid-spi.h"

uid_struct uid;  // Used by picc_read_card_serial().

/**
 * Writes a number of bytes to the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
void 
write_bytes_mfrc522( pcd_register reg, uint8_t count, uint8_t *values) 
{
  GPIO_CLR_PIN(GPIO_A_BASE, GPIO_PIN_6);
  SPI_WRITE((reg << 1) & 0x7e);
  uint8_t index;
  for (index = 0; index < count; index++) 
  {
    SPI_WRITE(values[index]);
  }  
  SPI_FLUSH();
  GPIO_SET_PIN(GPIO_A_BASE, GPIO_PIN_6);
} // End PCD_WriteRegister()

/**
 * Writes a byte to the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
void
write_mfrc522(uint8_t adr, uint8_t val)
{
  GPIO_CLR_PIN(GPIO_A_BASE, GPIO_PIN_6);
  SPI_WRITE((adr << 1) & 0x7e);
  SPI_WRITE(val);
  SPI_FLUSH();
  GPIO_SET_PIN(GPIO_A_BASE, GPIO_PIN_6);
}

/**
 * Reads a byte from the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
uint8_t
read_mfrc522(uint8_t dev_cmd)
{
  uint8_t ret = 0x0;
  GPIO_CLR_PIN(GPIO_A_BASE, GPIO_PIN_6);
  SPI_WRITE(((dev_cmd << 1) & 0x7e) | 0x80);
  SPI_FLUSH();
  SPI_READ(ret);
  GPIO_SET_PIN(GPIO_A_BASE, GPIO_PIN_6);
  return ret;
}

/**
 * Reads a number of bytes from the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
void read_fifo_mfrc522( pcd_register reg, uint8_t count, uint8_t *values, uint8_t rx_align) 
{
  if (count == 0) 
  {
    return;
  }
  //Serial.print(F("Reading "));  Serial.print(count); Serial.println(F(" bytes from register."));
  uint8_t index = 0;             // Index in values array.
  uint8_t ret = 0;
  GPIO_CLR_PIN(GPIO_A_BASE, GPIO_PIN_6);
  count--;                // One read is performed outside of the loop

  if (rx_align) {    // Only update bit positions rx_align..7 in values[0]
    // Create bit mask for bit positions rx_align..7
    uint8_t mask = (0xFF << rx_align) & 0xFF;
    // Read value and tell that we want to read the same address again.
    uint8_t value;
    ret = read_mfrc522(reg);
    value=ret;
    // Apply mask to both current value of values[0] and the new data in value.
    values[0] = (values[0] & ~mask) | (value & mask);
    index++;
  }

  while (index < count) {
    ret = read_mfrc522(reg);
    values[index] = ret;  // Read value and tell that we want to read the same address again.
    index++;
  }

    ret = read_mfrc522(reg);
    values[index] = ret;  // Read value and tell that we want to read the same address again.

  GPIO_SET_PIN(GPIO_A_BASE, GPIO_PIN_6);    // Release slave again
} // End PCD_ReadRegister()

/**
 * Translates the SAK (Select Acknowledge) to a PICC type.
 *
 * @return PICC_Type
 */
picc_type 
picc_get_type(uint8_t sak) 
{
  // http://www.nxp.com/documents/application_note/AN10833.pdf
  // 3.2 Coding of Select Acknowledge (SAK)
  // ignore 8-bit (iso14443 starts with LSBit = bit 1)
  // fixes wrong type for manufacturer Infineon (http://nfc-tools.org/index.php?title=ISO14443A)
  sak &= 0x7F;
  switch (sak) 
  {
    case 0x04:  return PICC_TYPE_NOT_COMPLETE;  // uid_struct not complete
    case 0x09:  return PICC_TYPE_MIFARE_MINI;
    case 0x08:  return PICC_TYPE_MIFARE_1K;
    case 0x18:  return PICC_TYPE_MIFARE_4K;
    case 0x00:  return PICC_TYPE_MIFARE_UL;
    case 0x10:
    case 0x11:  return PICC_TYPE_MIFARE_PLUS;
    case 0x01:  return PICC_TYPE_TNP3XXX;
    case 0x20:  return PICC_TYPE_ISO_14443_4;
    case 0x40:  return PICC_TYPE_ISO_18092;
    default:    return PICC_TYPE_UNKNOWN;
  }
} // End picc_get_type()


/**
 * Sets the bits given in mask in register reg.
 */
void 
pcd_set_register_bit_mask(pcd_register reg, uint8_t mask) 
{
  uint8_t tmp;
  tmp = read_mfrc522(reg);
  write_mfrc522(reg, tmp | mask);     // set bit mask
} // End pcd_set_register_bit_mask()


/**
 * Use the CRC coprocessor in the MFRC522 to calculate a CRC_A.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
status_code 
pcd_calculate_crc(uint8_t *data, uint8_t length, uint8_t *result) 
{
  write_mfrc522(CommandReg, PCD_Idle);    // Stop any active command.
  write_mfrc522(DivIrqReg, 0x04);         // Clear the CRCIRq interrupt request bit
  write_mfrc522(FIFOLevelReg, 0x80);      // FlushBuffer = 1, FIFO initialization 
  write_bytes_mfrc522(FIFODataReg, length, data); // Write data to the FIFO
  write_mfrc522(CommandReg, PCD_CalcCRC);   // Start the calculation
  // Wait for the CRC calculation to complete. Each iteration of the while-loop takes 17.73μs.
  // TODO check/modify for other architectures than Arduino Uno 16bit
  // Wait for the CRC calculation to complete. Each iteration of the while-loop takes 17.73us.
  uint16_t i;
  for ( i = 5000; i > 0; i--) 
  {
    // DivIrqReg[7..0] bits are: Set2 reserved reserved MfinActIRq reserved CRCIRq reserved reserved
    uint8_t n = read_mfrc522(DivIrqReg);
    if (n & 0x04) 
    {                 // CRCIRq bit set - calculation done
      write_mfrc522(CommandReg, PCD_Idle);  // Stop calculating CRC for new content in the FIFO.
      // Transfer the result from the registers to the result buffer
      result[0] = read_mfrc522(CRCResultRegL);
      result[1] = read_mfrc522(CRCResultRegH);
      return STATUS_OK;
    }
  }
  // 89ms passed and nothing happend. Communication with the MFRC522 might be down.
  printf("89ms passed and nothing happend.\n");
  return STATUS_TIMEOUT;
} // End pcd_calculate_crc()


/**
 * Performs a soft reset on the MFRC522 chip and waits for it to be ready again.
 */
bool
pcd_reset() 
{
  write_mfrc522(CommandReg, PCD_SoftReset); // Issue the SoftReset command.
  // The datasheet does not mention how long the SoftRest command takes to complete.
  // But the MFRC522 might have been in soft power-down mode (triggered by bit 4 of CommandReg)
  // Section 8.8.2 in the datasheet says the oscillator start-up time is the start up time of the crystal + 37,74μs. Let us be generous: 50ms.
  
  //BUSYWAIT_UNTIL_ABS(RTIMER_NOW(), US_TO_RTIMERTICKS(1000));
  // Wait for the PowerDown bit in CommandReg to be cleared
  int wait=0;
  while (read_mfrc522(CommandReg) & (1<<4)) 
  {
    // PCD still restarting - unlikely after waiting 50ms, but better safe than sorry.
    if(wait==1000)
    {
      return 0;
    }
    wait++;  
  }
  return 1;
} // End pcd_reset()

/**
 * Transfers data to the MFRC522 FIFO, executes a command, waits for completion and transfers data back from the FIFO.
 * CRC validation can only be done if back_data and back_len are specified.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */ 
status_code 
pcd_communicate_tith_picc(uint8_t command, uint8_t wait_irq, uint8_t *send_data, uint8_t send_len, uint8_t *back_data, uint8_t *back_len, uint8_t *valid_bits, uint8_t rx_align, bool check_crc) 
{
  // Prepare values for BitFramingReg
  uint8_t txLastBits = valid_bits ? *valid_bits : 0;
  uint8_t bitFraming = (rx_align << 4) + txLastBits;    // rx_align = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]

  write_mfrc522(CommandReg, PCD_Idle);                  // Stop any active command.
  write_mfrc522(ComIrqReg, 0x7F);                       // Clear all seven interrupt request bits
  write_mfrc522(FIFOLevelReg, 0x80);                    // FlushBuffer = 1, FIFO initialization
  write_bytes_mfrc522(FIFODataReg, send_len, send_data);  // Write send_data to the FIFO
  write_mfrc522(BitFramingReg, bitFraming);             // Bit adjustments
  write_mfrc522(CommandReg, command);                   // Execute the command

  if (command == PCD_Transceive) 
  {
    pcd_set_register_bit_mask(BitFramingReg, 0x80);        // StartSend=1, transmission of data starts
  }

  // Wait for the command to complete.
  // In pcd_initialization() we set the TAuto flag in TModeReg. This means the timer automatically starts when the PCD stops transmitting.
  // Each iteration of the do-while-loop takes 17.86μs.
  // TODO check/modify for other architectures than Arduino Uno 16bit
  
  uint16_t i;
  for (i = 2000; i > 0; i--) 
  {
    uint8_t n = read_mfrc522(ComIrqReg); // ComIrqReg[7..0] bits are: Set1 TxIRq RxIRq IdleIRq HiAlertIRq LoAlertIRq ErrIRq TimerIRq
    if (n & wait_irq) 
    {          // One of the  that signal success has been set.
        break;
    }
    if (n & 0x01) 
    {           // Timer interrupt - nothing received in 25ms
      return STATUS_TIMEOUT;
    }
  }

  // 35.7ms and nothing happend. Communication with the MFRC522 might be down.
  if (i == 0) 
  {
    return STATUS_TIMEOUT;
  }

  // Stop now if any errors except collisions were detected.
  uint8_t errorRegValue = read_mfrc522(ErrorReg); // ErrorReg[7..0] bits are: WrErr TempErr reserved BufferOvfl CollErr CRCErr ParityErr ProtocolErr
  if (errorRegValue & 0x13) 
  {  // BufferOvfl ParityErr ProtocolErr
    return STATUS_ERROR;
  }
  
  uint8_t _valid_bits = 0;

  // If the caller wants data back, get it from the MFRC522.
  if (back_data && back_len) 
  {
    uint8_t n = read_mfrc522(FIFOLevelReg);  // Number of bytes in the FIFO
    if (n > *back_len) 
    {
      return STATUS_NO_ROOM;
    }
    *back_len = n;                     // Number of bytes returned
    read_fifo_mfrc522(FIFODataReg, n, back_data, rx_align);  // Get received data from FIFO
    _valid_bits = read_mfrc522(ControlReg) & 0x07;   // RxLastBits[2:0] indicates the number of valid bits in the last received byte. If this value is 000b, the whole byte is valid.
    if (valid_bits) 
    {
      *valid_bits = _valid_bits;
      //printf(" *valid_bits = %x _valid_bits = %x\n",*valid_bits, _valid_bits);

    }
  }
  // Tell about collisions
  if (errorRegValue & 0x08) 
  {   // CollErr
    return STATUS_COLLISION;
  }

  // Perform CRC_A validation if requested.
  if (back_data && back_len && check_crc) 
  {
    // In this case a MIFARE Classic NAK is not OK.
    if (*back_len == 1 && _valid_bits == 4) 
    {
      return STATUS_MIFARE_NACK;
    }
    // We need at least the CRC_A value and all 8 bits of the last byte must be received.
    if (*back_len < 2 || _valid_bits != 0) 
    {
      return STATUS_CRC_WRONG;
    }
    // Verify CRC_A - do our own calculation and store the control in controlBuffer.
    uint8_t controlBuffer[2];
    status_code status = pcd_calculate_crc(&back_data[0], *back_len - 2, &controlBuffer[0]);
    if (status != STATUS_OK) 
    {
      return status;
    }
    if ((back_data[*back_len - 2] != controlBuffer[0]) || (back_data[*back_len - 1] != controlBuffer[1])) 
    {
      return STATUS_CRC_WRONG;
    }
  }

  return STATUS_OK;
} // End pcd_communicate_tith_picc()





/////////////////////////////////////////////////////////////////////////////////////
// Functions for communicating with PICCs
/////////////////////////////////////////////////////////////////////////////////////

/**
 * Executes the Transceive command.
 * CRC validation can only be done if back_data and back_len are specified.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
status_code 
pcd_transceive_data(uint8_t *send_data, uint8_t send_len, uint8_t *back_data, uint8_t *back_len, uint8_t *valid_bits, uint8_t rx_align, bool check_crc) 
{
  uint8_t wait_irq = 0x30;    // RxIRq and IdleIRq
  uint8_t result = pcd_communicate_tith_picc(PCD_Transceive, wait_irq, send_data, send_len, back_data, back_len, valid_bits, rx_align, check_crc);
  return result;
} // End pcd_transceive_data()




/**
 * Reads 16 bytes (+ 2 bytes CRC_A) from the active PICC.
 *
 * For MIFARE Classic the sector containing the block must be authenticated before calling this function.
 *
 * For MIFARE Ultralight only addresses 00h to 0Fh are decoded.
 * The MF0ICU1 returns a NAK for higher addresses.
 * The MF0ICU1 responds to the READ command by sending 16 bytes starting from the page address defined by the command argument.
 * For example; if blockAddr is 03h then pages 03h, 04h, 05h, 06h are returned.
 * A roll-back is implemented: If blockAddr is 0Eh, then the contents of pages 0Eh, 0Fh, 00h and 01h are returned.
 *
 * The buffer must be at least 18 bytes because a CRC_A is also returned.
 * Checks the CRC_A before returning STATUS_OK.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
status_code 
mifare_read( uint8_t blockAddr,uint8_t *buffer,uint8_t *bufferSize) 
{
  status_code result;

  // Sanity check
  if (buffer == NULL || *bufferSize < 18) 
  {
    return STATUS_NO_ROOM;
  }

  // Build command buffer
  buffer[0] = PICC_CMD_MF_READ;
  buffer[1] = blockAddr;
  // Calculate CRC_A
  result = pcd_calculate_crc(buffer, 2, &buffer[2]);
  if (result != STATUS_OK) 
  {
    return result;
  }

  // Transmit the buffer and receive the response, validate CRC_A.
  return pcd_transceive_data(buffer, 4, buffer, bufferSize, NULL, 0, true);
} // End mifare_read()




/**
 * Clears the bits given in mask from register reg.
 */
void 
pcd_clear_register_bit_mask(pcd_register reg, uint8_t mask) 
{
  uint8_t tmp;
  tmp = read_mfrc522(reg);
  write_mfrc522(reg, tmp & (~mask));    // clear bit mask
} // End pcd_clear_register_bit_mask()



/**
 * Used to exit the PCD from its authenticated state.
 * Remember to call this function after communicating with an authenticated PICC - otherwise no new communications can start.
 */
void 
pcd_stop_cryptol() 
{
  // Clear MFCrypto1On bit
  pcd_clear_register_bit_mask(Status2Reg, 0x08); // Status2Reg[7..0] bits are: TempSensClear I2CForceHS reserved reserved MFCrypto1On ModemState[2:0]
} // End pcd_stop_cryptol()





status_code 
picc_select( uid_struct *uid, uint8_t valid_bits) 
{
  bool uidComplete;
  bool selectDone;
  bool use_cascade_tag;
  uint8_t cascadeLevel = 1;
  uint8_t count;
  uint8_t index;
  uint8_t uid_index;          // The first index in uid->uidByte[] that is used in the current Cascade Level.
  uint8_t current_level_known_bits;   // The number of known uid_struct bits in the current Cascade Level.
  uint8_t buffer[9];         // The SELECT/ANTICOLLISION commands uses a 7 byte standard frame + 2 bytes CRC_A
  uint8_t bufferUsed;        // The number of bytes used in the buffer, ie the number of bytes to transfer to the FIFO.
  uint8_t rx_align;         // Used in BitFramingReg. Defines the bit position for the first bit received.
  uint8_t txLastBits;        // Used in BitFramingReg. The number of valid bits in the last transmitted byte.
  uint8_t *responseBuffer;
  uint8_t responseLength;
  status_code result;

  // Description of buffer structure:
  //    Byte 0: SEL         Indicates the Cascade Level: PICC_CMD_SEL_CL1, PICC_CMD_SEL_CL2 or PICC_CMD_SEL_CL3
  //    Byte 1: NVB         Number of Valid Bits (in complete command, not just the UID): High nibble: complete bytes, Low nibble: Extra bits.
  //    Byte 2: UID-data or CT    See explanation below. CT means Cascade Tag.
  //    Byte 3: UID-data
  //    Byte 4: UID-data
  //    Byte 5: UID-data
  //    Byte 6: BCC         Block Check Character - XOR of bytes 2-5
  //    Byte 7: CRC_A
  //    Byte 8: CRC_A
  // The BCC and CRC_A are only transmitted if we know all the uid_struct bits of the current Cascade Level.
  //
  // Description of bytes 2-5: (Section 6.5.4 of the ISO/IEC 14443-3 draft: uid_struct contents and cascade levels)
  //    uid_struct size  Cascade level Byte2 Byte3 Byte4 Byte5
  //    ========  ============= ===== ===== ===== =====
  //     4 bytes    1     uid0  uid1  uid2  uid3
  //     7 bytes    1     CT    uid0  uid1  uid2
  //                2     uid3  uid4  uid5  uid6
  //    10 bytes    1     CT    uid0  uid1  uid2
  //                2     CT    uid3  uid4  uid5
  //                3     uid6  uid7  uid8  uid9

  // Sanity checks
  if (valid_bits > 80) 
  {
    return STATUS_INVALID;
  }

  // Prepare MFRC522
  pcd_clear_register_bit_mask(CollReg, 0x80);    // ValuesAfterColl=1 => Bits received after collision are cleared.
  // Repeat Cascade Level loop until we have a complete UID.
  uidComplete = false;
  while (!uidComplete) 
  {
    // Set the Cascade Level in the SEL byte, find out if we need to use the Cascade Tag in byte 2.
    switch (cascadeLevel) 
    {
      case 1:
        buffer[0] = PICC_CMD_SEL_CL1;
        uid_index = 0;
        use_cascade_tag = valid_bits && uid->size > 4; // When we know that the uid_struct has more than 4 bytes
        break;
      case 2:
        buffer[0] = PICC_CMD_SEL_CL2;
        uid_index = 3;
        use_cascade_tag = valid_bits && uid->size > 7; // When we know that the uid_struct has more than 7 bytes
        break;
      case 3:
        buffer[0] = PICC_CMD_SEL_CL3;
        uid_index = 6;
        use_cascade_tag = false;            // Never used in CL3.
        break;
      default:
        printf("INTERNAL ERROR");
        return STATUS_INTERNAL_ERROR;
        break;
    }

    // How many uid_struct bits are known in this Cascade Level?
    current_level_known_bits = valid_bits - (8 * uid_index);
    if (current_level_known_bits < 0) 
    {
      current_level_known_bits = 0;
    }
    // Copy the known bits from uid->uidByte[] to buffer[]
    index = 2; // destination index in buffer[]
    if (use_cascade_tag) 
    {
      buffer[index++] = PICC_CMD_CT;
    }
    uint8_t bytes_to_copy = current_level_known_bits / 8 + (current_level_known_bits % 8 ? 1 : 0); // The number of bytes needed to represent the known bits for this level.
    if (bytes_to_copy) 
    {
      uint8_t maxBytes = use_cascade_tag ? 3 : 4; // Max 4 bytes in each Cascade Level. Only 3 left if we use the Cascade Tag
      if (bytes_to_copy > maxBytes) 
      {
        bytes_to_copy = maxBytes;
      }
      for (count = 0; count < bytes_to_copy; count++) {
        buffer[index++] = uid->uidByte[uid_index + count];
      }
    }
    // Now that the data has been copied we need to include the 8 bits in CT in current_level_known_bits
    if (use_cascade_tag) 
    {
      current_level_known_bits += 8;
    }

    // Repeat anti collision loop until we can transmit all uid_struct bits + BCC and receive a SAK - max 32 iterations.
    selectDone = false;
    while (!selectDone) 
    {
      // Find out how many bits and bytes to send and receive.
      if (current_level_known_bits >= 32) 
      { // All uid_struct bits in this Cascade Level are known. This is a SELECT.
        buffer[1] = 0x70; // NVB - Number of Valid Bits: Seven whole bytes
        // Calculate BCC - Block Check Character
        buffer[6] = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5];
        // Calculate CRC_A
        result = pcd_calculate_crc(buffer, 7, &buffer[7]);
        if (result != STATUS_OK) 
        {
          return result;
        }
        txLastBits    = 0; // 0 => All 8 bits are valid.
        bufferUsed    = 9;
        // Store response in the last 3 bytes of buffer (BCC and CRC_A - not needed after tx)
        responseBuffer  = &buffer[6];
        responseLength  = 3;
      }
      else 
      { // This is an ANTICOLLISION.
        txLastBits    = current_level_known_bits % 8;
        count     = current_level_known_bits / 8;  // Number of whole bytes in the uid_struct part.
        index     = 2 + count;          // Number of whole bytes: SEL + NVB + UIDs
        buffer[1]   = (index << 4) + txLastBits;  // NVB - Number of Valid Bits
        bufferUsed    = index + (txLastBits ? 1 : 0);
        // Store response in the unused part of buffer
        responseBuffer  = &buffer[index];
        responseLength  = sizeof(buffer) - index;
      }

      // Set bit adjustments
      rx_align = txLastBits;                     // Having a separate variable is overkill. But it makes the next line easier to read.
      write_mfrc522(BitFramingReg, (rx_align << 4) + txLastBits);  // rx_align = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]

      // Transmit the buffer and receive the response.
      result = pcd_transceive_data(buffer, bufferUsed, responseBuffer, &responseLength, &txLastBits, rx_align, false);
      if (result == STATUS_COLLISION) 
      { // More than one PICC in the field => collision.
        uint8_t valueOfCollReg = read_mfrc522(CollReg); // CollReg[7..0] bits are: ValuesAfterColl reserved CollPosNotValid CollPos[4:0]
        if (valueOfCollReg & 0x20) 
        { // CollPosNotValid
          return STATUS_COLLISION; // Without a valid collision position we cannot continue
        }
        uint8_t collisionPos = valueOfCollReg & 0x1F; // Values 0-31, 0 means bit 32.
        if (collisionPos == 0) 
        {
          collisionPos = 32;
        }
        if (collisionPos <= current_level_known_bits) 
        { // No progress - should not happen
          return STATUS_INTERNAL_ERROR;
        }
        // Choose the PICC with the bit set.
        current_level_known_bits = collisionPos;
        count     = (current_level_known_bits - 1) % 8; // The bit to modify
        index     = 1 + (current_level_known_bits / 8) + (count ? 1 : 0); // First byte is index 0.
        buffer[index] |= (1 << count);
      }
      else if (result != STATUS_OK) 
      {
        return result;
      }
      else 
      { // STATUS_OK
        if (current_level_known_bits >= 32) 
        { // This was a SELECT.
          selectDone = true; // No more anticollision
          // We continue below outside the while.
        }
        else 
        { // This was an ANTICOLLISION.
          // We now have all 32 bits of the uid_struct in this Cascade Level
          current_level_known_bits = 32;
          // Run loop again to do the SELECT.
        }
      }
    } // End of while (!selectDone)

    // We do not check the CBB - it was constructed by us above.

    // Copy the found uid_struct bytes from buffer[] to uid->uidByte[]
    index     = (buffer[2] == PICC_CMD_CT) ? 3 : 2; // source index in buffer[]
    bytes_to_copy   = (buffer[2] == PICC_CMD_CT) ? 3 : 4;
    for (count = 0; count < bytes_to_copy; count++) 
    {
      uid->uidByte[uid_index + count] = buffer[index++];
    }

    // Check response SAK (Select Acknowledge)
    if (responseLength != 3 || txLastBits != 0) 
    { // SAK must be exactly 24 bits (1 byte + CRC_A).Select Acknowledge
      return STATUS_ERROR;
    }
    // Verify CRC_A - do our own calculation and store the control in buffer[2..3] - those bytes are not needed anymore.
    result = pcd_calculate_crc(responseBuffer, 1, &buffer[2]);
    if (result != STATUS_OK) 
    {
      return result;
    }
    if ((buffer[2] != responseBuffer[1]) || (buffer[3] != responseBuffer[2])) 
    {
      return STATUS_CRC_WRONG;
    }
    if (responseBuffer[0] & 0x04) 
    { // Cascade bit set - uid_struct not complete yes
      cascadeLevel++;
    }
    else {
      uidComplete = true;
      uid->sak = responseBuffer[0];
    }
  } // End of while (!uidComplete)

  // Set correct uid->size
  uid->size = 3 * cascadeLevel + 1;

  return STATUS_OK;
} // End picc_select()

/**
 * Instructs a PICC in state ACTIVE(*) to go to state HALT.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
status_code 
picc_halt_a() 
{
  status_code result;
  uint8_t buffer[4];

  // Build command buffer
  buffer[0] = PICC_CMD_HLTA;
  buffer[1] = 0;
  // Calculate CRC_A
  result = pcd_calculate_crc(buffer, 2, &buffer[2]);
  if (result != STATUS_OK) 
  {
    return result;
  }
  // Send the command.
  // The standard says:
  //    If the PICC responds with any modulation during a period of 1 ms after the end of the frame containing the
  //    HLTA command, this response shall be interpreted as 'not acknowledge'.
  // We interpret that this way: Only STATUS_TIMEOUT is a success.
  result = pcd_transceive_data(buffer, sizeof(buffer), NULL, 0, NULL, 0, false);
  if (result == STATUS_TIMEOUT) 
  {
    return STATUS_OK;
  }
  if (result == STATUS_OK) 
  { // That is ironically NOT ok in this case ;-)
    return STATUS_ERROR;
  }
  return result;
} // End picc_halt_a()

/**
 * Returns a __FlashStringHelper pointer to the PICC type name.
 *
 * @return const __FlashStringHelper *
 */
void 
picc_get_type_name(picc_type piccType) 
{
  switch (piccType) 
  {
    case PICC_TYPE_ISO_14443_4:    printf("PICC compliant with ISO/IEC 14443-4\n"); break;
    case PICC_TYPE_ISO_18092:    printf("PICC compliant with ISO/IEC 18092 (NFC)\n");break;
    case PICC_TYPE_MIFARE_MINI:    printf("MIFARE Mini, 320 bytes\n");break;
    case PICC_TYPE_MIFARE_1K:    printf("MIFARE 1KB\n");break;
    case PICC_TYPE_MIFARE_4K:    printf("MIFARE 4KB\n");break;
    case PICC_TYPE_MIFARE_UL:    printf("MIFARE Ultralight or Ultralight C\n");break;
    case PICC_TYPE_MIFARE_PLUS:    printf("MIFARE Plus\n");break;
    case PICC_TYPE_MIFARE_DESFIRE:   printf("MIFARE DESFire\n");break;
    case PICC_TYPE_TNP3XXX:      printf("MIFARE TNP3XXX\n");break;
    case PICC_TYPE_NOT_COMPLETE:   printf("SAK indicates uid_struct is not complete.\n");break;
    case PICC_TYPE_UNKNOWN:
    default:             printf("Unknown type\n");
  }
} // End picc_get_type_name()


/**
 * Transmits REQA or WUPA commands.
 * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
status_code 
picc_reqa_or_wupa( uint8_t command, uint8_t *bufferATQA, uint8_t *bufferSize) 
{
  uint8_t valid_bits;
  status_code status;

  if (bufferATQA == NULL || *bufferSize < 2) 
  {  // The ATQA response is 2 bytes long.
    //printf("picc_reqa_or_wupa > STATUS_NO_ROOM: %x\n", STATUS_NO_ROOM);
    return STATUS_NO_ROOM;
  }

  pcd_clear_register_bit_mask(CollReg, 0x80);    // ValuesAfterColl=1 => Bits received after collision are cleared.
  valid_bits = 7;                  // For REQA and WUPA we need the short frame format - transmit only 7 bits of the last (and only) byte. TxLastBits = BitFramingReg[2..0]
  status = pcd_transceive_data(&command, 1, bufferATQA, bufferSize, &valid_bits, 0, false);
  if (status != STATUS_OK) 
  {
    //printf("picc_reqa_or_wupa > status: %x\n", status);
    return status;
  }
  if (*bufferSize != 2 || valid_bits != 0) 
  {   // ATQA must be exactly 16 bits.
     printf("STATUS_ERROR: %x\n", STATUS_ERROR);
    return STATUS_ERROR;
  }
  return STATUS_OK;
} // End picc_reqa_or_wupa()


/**
 * Transmits a REQuest command, Type A. Invites PICCs in state IDLE to go to READY and prepare for anticollision or selection. 7 bit frame.
 * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
status_code 
picc_request_a( uint8_t *bufferATQA,uint8_t *bufferSize) 
{
  return picc_reqa_or_wupa(PICC_CMD_REQA, bufferATQA, bufferSize);
} // End picc_request_a()

/**
 * Returns true if a PICC responds to PICC_CMD_REQA.
 * Only "new" cards in state IDLE are invited. Sleeping cards in state HALT are ignored.
 * 
 * @return bool
 */
bool 
picc_is_new_card_present() 
{
  uint8_t bufferATQA[2];
  uint8_t bufferSize = sizeof(bufferATQA);

  // Reset baud rates
  write_mfrc522(TxModeReg, 0x00);
  write_mfrc522(RxModeReg, 0x00);
  // Reset ModWidthReg
  write_mfrc522(ModWidthReg, 0x26);

 
  status_code result = picc_request_a(bufferATQA, &bufferSize);

  return (result == STATUS_OK || result == STATUS_COLLISION);
} // End picc_is_new_card_present()

/**
 * Simple wrapper around picc_select.
 * Returns true if a uid_struct could be read.
 * Remember to call picc_is_new_card_present(), picc_request_a() or PICC_WakeupA() first.
 * The read uid_struct is available in the class variable uid.
 * 
 * @return bool
 */
bool 
picc_read_card_serial() 
{
  status_code result = picc_select(&uid,0);
  return (result == STATUS_OK);
} // End 

uint32_t
picc_dump_details_to_serial(uid_struct *uid) 
{
  // UID!!!
  uint32_t card_uid=0;
  uint8_t i;
  for (i = 0; i < uid->size; i++) {
    card_uid=(uid->uidByte[i] | (card_uid << 8));
  } 
  
  // SAK -slave acknowledge
  printf("\nCard SAK: ");
  if(uid->sak < 0x10)
  printf("0\n");
  printf("%x\n",uid->sak);
  
  // (suggested) PICC type
  picc_type piccType = picc_get_type(uid->sak);
  printf("PICC type: ");
  picc_get_type_name(piccType);
  return card_uid;
} // End picc_dump_details_to_serial()

status_code 
pcd_authenticate (uint8_t command, uint8_t blockAddr, MIFARE_Key *key, uid_struct *uid) 
{
  uint8_t wait_irq = 0x10;    // IdleIRq
  uint8_t i;
  // Build command buffer
  uint8_t send_data[12];
  send_data[0] = command;
  send_data[1] = blockAddr;
  for ( i = 0; i < MF_KEY_SIZE; i++) 
  {  // 6 key bytes
    send_data[2+i] = key->keyByte[i];
  }
  // Use the last uid_struct bytes as specified in http://cache.nxp.com/documents/application_note/AN10927.pdf
  // section 3.2.5 "MIFARE Classic Authentication".
  // The only missed case is the MF1Sxxxx shortcut activation,
  // but it requires cascade tag (CT) byte, that is not part of uid.
  for ( i = 0; i < 4; i++) 
  {        // The last 4 bytes of the UID
    send_data[8+i] = uid->uidByte[i+uid->size-4];
  }

  // Start the authentication.
  return pcd_communicate_tith_picc(pcd_mf_authent, wait_irq, &send_data[0], sizeof(send_data), NULL, NULL, NULL, 0, false);
} // End pcd_authenticate()


/**
 * Returns a __FlashStringHelper pointer to a status code name.
 *
 * @return const __FlashStringHelper *
 */
void  
get_status_code_name (status_code code) 
{
  switch (code) 
  {
    case STATUS_OK:       printf("Success.\n"); break;
    case STATUS_ERROR:      printf("Error in communication.\n"); break;
    case STATUS_COLLISION:    printf("Collission detected.\n"); break;
    case STATUS_TIMEOUT:    printf("Timeout in communication.\n"); break;
    case STATUS_NO_ROOM:    printf("A buffer is not big enough."); break;
    case STATUS_INTERNAL_ERROR: printf("Internal error in the code. Should not happen.\n"); break;
    case STATUS_INVALID:    printf("Invalid argument.\n"); break;
    case STATUS_CRC_WRONG:    printf("The CRC_A does not match.\n"); break;
    case STATUS_MIFARE_NACK:  printf("A MIFARE PICC responded with NAK.\n"); break;
    default:          printf("Unknown error\n"); 
  }
} // End get_status_code_name()


/**
 * Dumps memory contents of a sector of a MIFARE Classic PICC.
 * Uses pcd_authenticate(), mifare_read() and pcd_stop_cryptol.
 * Always uses PICC_CMD_MF_AUTH_KEY_A because only Key A can always read the sector trailer access bits.
 */
void 
picc_dump_mifare_classic_sector_to_serial (uid_struct *uid, MIFARE_Key *key, uint8_t sector) 
{
  status_code status;
  uint8_t firstBlock;    // Address of lowest address to dump actually last block dumped)
  uint8_t no_of_blocks;    // Number of blocks in sector
  bool isSectorTrailer; // Set to true while handling the "last" (ie highest address) in the sector.

  // The access bits are stored in a peculiar fashion.
  // There are four groups:
  //    g[3]  Access bits for the sector trailer, block 3 (for sectors 0-31) or block 15 (for sectors 32-39)
  //    g[2]  Access bits for block 2 (for sectors 0-31) or blocks 10-14 (for sectors 32-39)
  //    g[1]  Access bits for block 1 (for sectors 0-31) or blocks 5-9 (for sectors 32-39)
  //    g[0]  Access bits for block 0 (for sectors 0-31) or blocks 0-4 (for sectors 32-39)
  // Each group has access bits [C1 C2 C3]. In this code C1 is MSB and C3 is LSB.
  // The four CX bits are stored together in a nible cx and an inverted nible cx_.
  uint8_t c1, c2, c3;    // Nibbles
  uint8_t c1_, c2_, c3_;   // Inverted nibbles
  bool invertedError;   // True if one of the inverted nibbles did not match
  uint8_t g[4];        // Access bits for each of the four groups.
  uint8_t group;       // 0-3 - active group for access bits
  bool firstInGroup;    // True for the first block dumped in the group

  // Determine position and size of sector.
  if (sector < 32) 
  { // Sectors 0..31 has 4 blocks each
    no_of_blocks = 4;
    firstBlock = sector * no_of_blocks;
  }
  else if (sector < 40) 
  { // Sectors 32-39 has 16 blocks each
    no_of_blocks = 16;
    firstBlock = 128 + (sector - 32) * no_of_blocks;
  }
  else 
  { // Illegal input, no MIFARE Classic PICC has more than 40 sectors.
    return;
  }

  // Dump blocks, highest address first.
  uint8_t byteCount,index;
  uint8_t buffer[18];
  uint8_t blockAddr;
  isSectorTrailer = true;
  invertedError = false;  // Avoid "unused variable" warning.
  int8_t blockOffset;
  for (blockOffset = no_of_blocks - 1; blockOffset >= 0; blockOffset--) 
  {
    blockAddr = firstBlock + blockOffset;
    // Sector number - only on first line
    if (isSectorTrailer) 
    {
      if(sector < 10)
        printf("   "); // Pad with spaces
      else
        printf("   "); // Pad with spaces
        printf("%x",sector);
        printf("   ");
    }
    else 
    {
      printf("       ");
    }
    // Block number
    if(blockAddr < 10)
      printf("   "); // Pad with spaces
    else 
    {
      if(blockAddr < 100)
        printf("  "); // Pad with spaces
      else
        printf(" "); // Pad with spaces
    }
    printf("%x",blockAddr);
    printf("  ");
    // Establish encrypted communications before reading the first block
    if (isSectorTrailer) 
    {
      status = pcd_authenticate(PICC_CMD_MF_AUTH_KEY_A, firstBlock, key, uid);
      if (status != STATUS_OK) 
      {
        printf("pcd_authenticate() failed: ");
        get_status_code_name(status);
        return;
      }
    }
    // Read block
    byteCount = sizeof(buffer);
    status = mifare_read(blockAddr, buffer, &byteCount);
    if (status != STATUS_OK) 
    {
      printf("mifare_read() failed: ");
      get_status_code_name(status);
      continue;
    }
    // Dump data

    for ( index = 0; index < 16; index++) 
    {
      if(buffer[index] < 0x10)
        printf(" 0");
      else
        printf(" ");
        printf("%x",buffer[index]);
      if ((index % 4) == 3) 
      {
        printf(" ");
      }
    }
    // Parse sector trailer data
    if (isSectorTrailer) 
    {
      c1  = buffer[7] >> 4;
      c2  = buffer[8] & 0xF;
      c3  = buffer[8] >> 4;
      c1_ = buffer[6] & 0xF;
      c2_ = buffer[6] >> 4;
      c3_ = buffer[7] & 0xF;
      invertedError = (c1 != (~c1_ & 0xF)) || (c2 != (~c2_ & 0xF)) || (c3 != (~c3_ & 0xF));
      g[0] = ((c1 & 1) << 2) | ((c2 & 1) << 1) | ((c3 & 1) << 0);
      g[1] = ((c1 & 2) << 1) | ((c2 & 2) << 0) | ((c3 & 2) >> 1);
      g[2] = ((c1 & 4) << 0) | ((c2 & 4) >> 1) | ((c3 & 4) >> 2);
      g[3] = ((c1 & 8) >> 1) | ((c2 & 8) >> 2) | ((c3 & 8) >> 3);
      isSectorTrailer = false;
    }

    // Which access group is this block in?
    if (no_of_blocks == 4) 
    {
      group = blockOffset;
      firstInGroup = true;
    }
    else 
    {
      group = blockOffset / 5;
      firstInGroup = (group == 3) || (group != (blockOffset + 1) / 5);
    }

    if (firstInGroup) 
    {
      // Print access bits
      printf(" [ ");
      printf("%d ",(g[group] >> 2) & 1);
      printf("%d ",(g[group] >> 1) & 1);
      printf("%d ",(g[group] >> 0) & 1);
      printf("] \n");
      if (invertedError) {
        printf(" Inverted access bits did not match!\n ");
      }
    }

    if (group != 3 && (g[group] == 1 || g[group] == 6)) 
    { // Not a sector trailer, a value block
      int32_t value;
      value = ((int32_t)(buffer[3]<<24) | (int32_t)(buffer[2]<<16) | (int32_t)(buffer[1]<<8) | (int32_t)buffer[0]);
      printf(" Value=0x"); printf("[%"PRIu32"]\n",value);
      printf(" Adr=0x"); printf("%x\n",buffer[12]);
    }
  }

  return;
} // End picc_dump_mifare_classic_sector_to_serial()


/**
 * Dumps memory contents of a MIFARE Ultralight PICC.
 */
void 
picc_dump_mifare_ultralight_to_serial() 
{
  status_code status;
  uint8_t byteCount;
  uint8_t buffer[18];
  uint8_t page,offset,index,i;

  printf("Page  0  1  2  3\n");
  // Try the mpages of the original Ultralight. Ultralight C has more pages.
  for ( page = 0; page < 16; page +=4) 
  { // Read returns data for 4 pages at a time.
    // Read pages
    byteCount = sizeof(buffer);
    status = mifare_read(page, buffer, &byteCount);
    if (status != STATUS_OK) 
    {
      printf("mifare_read() failed: ");
      get_status_code_name(status);
      break;
    }
    // Dump data
    for (offset = 0; offset < 4; offset++) 
    {
      i = page + offset;
      if(i < 10)
        printf("  "); // Pad with spaces
      else
        printf(" "); // Pad with spaces
      printf("%x",i);
      printf("  ");
      for (index = 0; index < 4; index++) 
      {
        i = 4 * offset + index;
        if(buffer[i] < 0x10)
          printf(" 0");
        else
          printf(" ");
        printf("%x",buffer[i]);
      }
      printf("\n");
    }
  }
} // End picc_dump_mifare_ultralight_to_serial()


/**
 * Dumps memory contents of a MIFARE Classic PICC.
 * On success the PICC is halted after dumping the data.
 */
void 
picc_dump_mifare_classic_to_serial( uid_struct *uid, picc_type piccType, MIFARE_Key *key) 
{
  uint8_t no_of_sectors = 0,i;
  switch (piccType) 
  {
    case PICC_TYPE_MIFARE_MINI:
      // Has 5 sectors * 4 blocks/sector * 16 bytes/block = 320 bytes.
      no_of_sectors = 5;
      break;

    case PICC_TYPE_MIFARE_1K:
      // Has 16 sectors * 4 blocks/sector * 16 bytes/block = 1024 bytes.
      no_of_sectors = 16;
      break;

    case PICC_TYPE_MIFARE_4K:
      // Has (32 sectors * 4 blocks/sector + 8 sectors * 16 blocks/sector) * 16 bytes/block = 4096 bytes.
      no_of_sectors = 40;
      break;

    default: // Should not happen. Ignore.
      break;
  }

  // Dump sectors, highest address first.
  if (no_of_sectors) 
  {
    printf("Sector Block   0  1  2  3   4  5  6  7   8  9 10 11  12 13 14 15  AccessBits\n");
    for (i = no_of_sectors - 1; i >= 0; i--) 
    {
      picc_dump_mifare_classic_sector_to_serial(uid, key, i);
    }
  }
  picc_halt_a(); // Halt the PICC before stopping the encrypted session.
  pcd_stop_cryptol();
} // End picc_dump_mifare_classic_to_serial()

uint32_t
picc_dump_to_serial(uid_struct *uid) 
{
  
  
  // Dump UID, SAK and Type
  return picc_dump_details_to_serial(uid);  
} // End picc_dump_to_serial()

/*--------------------------------------------------------------------------------------*/


void picc_dump_contents(uid_struct *uid)
{
  MIFARE_Key key;
  uint8_t i;

  picc_type piccType = picc_get_type(uid->sak);
  switch (piccType) {
    case PICC_TYPE_MIFARE_MINI:
    case PICC_TYPE_MIFARE_1K:
    case PICC_TYPE_MIFARE_4K:
      // All keys are set to FFFFFFFFFFFFh at chip delivery from the factory.

      for ( i = 0; i < 6; i++) {
        key.keyByte[i] = 0xFF;
      }
      picc_dump_mifare_classic_to_serial(uid, piccType, &key);
     break;
      
    case PICC_TYPE_MIFARE_UL:
      picc_dump_mifare_ultralight_to_serial();
      break;
      
    case PICC_TYPE_ISO_14443_4:
      printf("----------------------1\n");
      break;
    case PICC_TYPE_MIFARE_DESFIRE:
      printf("----------------------2\n");
      break;
    case PICC_TYPE_ISO_18092:
      printf("----------------------3\n");
      break;
    case PICC_TYPE_MIFARE_PLUS:
      printf("----------------------4\n");
      break;
    case PICC_TYPE_TNP3XXX:
      printf("Dumping memory contents not implemented for that PICC type.");
      break;
      
    case PICC_TYPE_UNKNOWN:
    case PICC_TYPE_NOT_COMPLETE:
    default:
      break; printf(" No memory dump here \n");
  }
  
  picc_halt_a(); // Already done if it was a MIFARE Classic PICC.
}


void 
antenna_on() 
{
  uint8_t value ;
  value = read_mfrc522(TxControlReg);
  if ((value & 0x03) != 0x03) {
    write_mfrc522(TxControlReg, value | 0x03);
  }
  value = read_mfrc522(TxControlReg);
}

/*
 * Initializes the MFRC522 chip.
 */
bool 
pcd_initialization() 
{
  printf("pcd_initialization()\n");

  if(pcd_reset()==0)
  {
    return 0;
  }
  // Reset baud rates
  write_mfrc522(TxModeReg, 0x00);  
  write_mfrc522(RxModeReg, 0xb0);  
  //Reset ModWidthReg
  write_mfrc522(ModWidthReg, 0x26);
  // When communicating with a PICC we need a timeout if something goes wrong.
  // f_timer = 13.56 MHz / (2*TPreScaler+1) where TPreScaler = [TPrescaler_Hi:TPrescaler_Lo].
  // TPrescaler_Hi are the four low bits in TModeReg. TPrescaler_Lo is TPrescalerReg.
  write_mfrc522(TModeReg, 0x80);      // TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds
  write_mfrc522(TPrescalerReg, 0xA9);   // TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25μs.
  write_mfrc522(TReloadRegH, 0x03);   // Reload timer with 0x3E8 = 1000, ie 25ms before timeout.
  write_mfrc522(TReloadRegL, 0xE8);
  write_mfrc522(TxASKReg, 0x40);    // Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting
  write_mfrc522(ModeReg, 0x3D);   // Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)
  write_mfrc522(RFCfgReg,0x70);
  write_mfrc522(ComIEnReg,0x80);
  write_mfrc522(FIFOLevelReg,0x00);
  antenna_on();            // Enable the antenna driver pins TX1 and TX2 (they were disabled by the reset)

  printf("initialization Done()\n");

  return 1;
} // End pcd_initialization()    */  




