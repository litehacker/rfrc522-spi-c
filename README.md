# rfrc522-spi-c
  Short Description: This is a lirary for RFID RC522 device in C. Originally the code was a C++ library MFRC522.cpp for Arduino projects (https://playground.arduino.cc/Learning/MFRC522). In order to run RF reader on a private MCU - Mahmote, which runs on Contiki OS, the code was implemented and "translated" to C.
## main.c :
  Starts Contiki process and does exactly the same initialization and looping as DumpInfo.ino file from Arduino Project. (https://github.com/miguelbalboa/rfid/blob/master/examples/DumpInfo/DumpInfo.ino)
## rfid-spi.c and rfid-spi.h :
  Are libraries similar to RFRC522.cpp and RFRC522.h 
