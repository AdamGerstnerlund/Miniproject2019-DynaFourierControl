/*********************************************************************
 * Code should accomplish the following:                             *
 * 1. Dynamixel servo motor                                          *
 * 2. Protocol 2                                                     *
 * 3. Arduino as master on RS485                                     *
 * 4. Easy change of motor ID in src                                 *
 * 5. Ability to identify ID of servo motors                         *
 * 6. Reset command (for all servos, if possible)                    *
 * 7. Enable/disable torques on Dynamixel motor                      *
 * 8. Set P-gain (once)                                              *
 * 9. With a fixed sampling rate/freq of 10 Hz:                      *
 *      get present temperature and reset or stop motor if too high  *
 *********************************************************************/

// Purple jumper wire must be in I/O pin 19 (for CC)
// Grey jumper wire must be in I/O pin 18 (for CC)

#include <Wire.h>                       // Necessary for some boards to communcate with I^2C devices

#define dynaBaud 57600                  // Fixed baud rate for Dynamixel MX-series is 57600
#define header1 0xff                    // Header 1 to herald attempted communication (dec 255)
#define header2 header1                 // Header 2 to herald attempted communication (same)
#define header3 0xfd                    // Header 3 to herald attempted communication (dec 253)
#define reserved 0x00                   // Dynamixel has set this byte as "reserved" and zero-initialized
#define instruction_read 0x02           // Byte for read-instruction fixed to dec 2
#define instruction_write 0x03          // Byte for write-instruction fixed to dec 3
#define torqueAddress 0x40              // Torque activation address is dec 64 (MX-series)
#define packetLengthHigh 0x00           // High-order byte describing array-length. Zero for most packets (unlike the Low-order byte which tends to vary).

unsigned char motorID = 1;              // Motor ID - Initialized with '1' for CrustCrawler base joint [SET]
bool torqueStatus;                      // Uninitialized - is torque enabled for the given motor?

unsigned char directPin = 2;            // Blue wire in the MEGA's digital 2 (not necessarily for my board)
unsigned char Status_Packet_Array[20];  // So-called "packet" (array) declared to later be filled with chars
unsigned int pGain = 6;                 // Gain-variable initialized with integer constant '6' (can be changed at will)
long actTime = 0;                       // Time-variable for sampling temperature later in the code
long hlpTime = 0;                       // Time-variable for sampling temperature later in the code

class serialFunctions                   // Class containing seven methods or functions providing each of the features listed in the top box
{
  public:
  bool type;
  void applyTorque(unsigned char motorID, bool torqueStatus);
  void setpGain(unsigned char motorID, unsigned int pGain);
  void rebootDynamixel(unsigned char motorID);
  void temperature(unsigned char motorID);
  void transmitInstructionPacket(unsigned char* var1, int var2); // var1 desc. purpose. var2 desc. packet index. char* is a pointer to the first char in the return "string" (char array)
  void readStatusPacket();
  void readStatusPacketTemp();
  unsigned short update_cRc(unsigned short cRc_accum, unsigned char* data_blk_ptr, unsigned short data_blk_size); // for the CRC check made to ensure that all bytes are properly received
};

void serialFunctions::applyTorque(unsigned char motorID, bool torqueStatus) // Method sending packet for applying torque
{
  unsigned char torqueInstruction[] =   // Command data sent to the device
  {
    header1,                            // 255 in dec
    header2,                            // Identical to Header 1
    header3,                            // 253 in dec
    reserved,                           // 4th byte is "reserved" beyond user-command
    motorID,                            // Packet ID: in this case motor ID for application
    0x06,                               // Low-order byte for packet length
    packetLengthHigh,                   // High-order byte for packet length
    instruction_write,                  // Tells motor to receive info rather than send info back
    torqueAddress,                      // 64 in dec (write-address for enabling torque)
    0x00,                               // Second Dynamixel "parameter" empty (write address Lo-byte)
    torqueStatus                        // ON/OFF ... FALSE/TRUE
  };
  type = false;                         // Default
  transmitInstructionPacket(torqueInstruction, 11); // 11 entries w/o the CRC-entries
}

void serialFunctions::setpGain(unsigned char motorID, unsigned int pGain) // Method sending packet for setting P_gain
{
  unsigned char p_gain[] =              // Command data sent to the device
  {
    header1,                            // 255 in dec
    header2,                            // Identical to Header 1
    header3,                            // 253 in dec
    reserved,                           // 4th byte is "reserved" beyond user-command
    motorID,                            // Packet ID: in this case motor ID for application
    0x07,                               // Low-order byte for packet length
    packetLengthHigh,                   // High-order byte for packet length
    instruction_write,                  // Tells motor to receive info rather than send info back
    0x54,                               // 84 in dec (Lo-byte for p_gain write-address)
    0x00,                               // Hi-byte for same address
    pGain&0x00ff,                       // Statement essentially splits int-value into two 1-byte values (chars) along with next line...
    (pGain&0xff00)>>8                   // ...using the bitwise AND operator, and here also the bitwise right-shift operator (even though gains above 255 probably won't be needed)
  };
  type = false;                         // Default
  transmitInstructionPacket(p_gain, 12);// 12 entries w/o the CRC-entries
}

void serialFunctions::rebootDynamixel(unsigned char motorID) // Method sending packet for rebooting Dynamixel
{
  unsigned char rebootInstruction[] =
  {
    header1,
    header2,
    header3,
    reserved,
    motorID,
    0x03,                               // Packet length: 3
    packetLengthHigh,
    0x08                                // Reboot instr.
  };
  type = false;
  transmitInstructionPacket(rebootInstruction, 8); // 8 entries w/o the CRC-entries
}

void serialFunctions::temperature(unsigned char motorID) // Method sending packet for recording motor heat
{
  unsigned char heat[] =
  {
    header1,
    header2,
    header3,
    reserved,
    motorID,
    0x07,                               // Packet length: 7
    packetLengthHigh,
    0x02,                               // Instr. to read temperature
    0x92,                               // Write address Lo-byte
    0x00,                               // Write address Hi-byte
    0x01,                               // ON-instr. (in accordance with "Dynamixel Wizard")
    0x00                                // Empty (in accordance with "Dynamixel Wizard")
  };
  type = true;
  transmitInstructionPacket(heat, 12);  // 12 entries w/o the CRC-entries
}

void serialFunctions::transmitInstructionPacket(unsigned char* var1, int var2) // Asterisk creates a pointer
{
  digitalWrite(directPin, HIGH);        // status set to HIGH (bcs it's an OUTPUT pin). Direction pin-var defined with global scope
  unsigned short cRc = update_cRc(0, var1, var2);
  
  for (int i = 0; i < var2; i++)        // For-loop writing to the 1st of the MEGA's default Serial Channels (Serial1: pins 18 & 19)
  {
    Serial1.write(*var1);               // variable one pointer
    var1++;                             // Postfix incrementation to count each iteration for both var1 and var2
  }
  unsigned char cRc_lo = (cRc & 0xff);    // Lo-byte, Dynamixel CRC-check
  unsigned char cRc_hi = (cRc>>8) & 0xff; // Hi-byte, Dynamixel CRC-check
  noInterrupts();                       // Temporarily disable interrupts. Passed no args.
  Serial1.write(cRc_lo);                // Send check back to Dyna
  Serial1.write(cRc_hi);                // Send check back to Dyna
  
  if ((UCSR1A & B01100000) != B01100000)
  {
    Serial1.flush();                    // Flush. Wait until TX data is sent
  }
  digitalWrite(directPin, LOW);         // TX buffer pin is set to LOW after data has been sent
  interrupts();                         // Re-enable interrupts. Passed no args.
  delay(50);                            // 5 centisecond delay

  if (type == false)
  {
    readStatusPacket();
  }
  else
  {
    readStatusPacketTemp();
  }
}

void serialFunctions::readStatusPacket() // Create an array whose i'th entry depends on the receiver
{
  if (Serial1.available() > 0)
  {
    for (int i = 0; i < 24; i++)
    {
      Status_Packet_Array[i] = Serial1.read();
    }
  }
}

void serialFunctions::readStatusPacketTemp()
{
  int i = 0;
  while ( Serial1.available() > 0 ) // While-loop defined for as long as Serial1 receives some nonzero integer
  {
    Status_Packet_Array[i] = Serial1.read();
    i++;
  }
  int temp = Status_Packet_Array[9];
  int h1 = Status_Packet_Array[0];
  int h2 = Status_Packet_Array[1];
  int h3 = Status_Packet_Array[2];
  if ( (h1 == 255) && (h2 == 255) && (h3 == 253) )
  {
    Serial.print("Temperature of motor # ");
    Serial.print(motorID);
    Serial.print(" is: ");
    Serial.print(temp);
    Serial.print(" °C \n");
    if ( temp > 75 )
    {
      rebootDynamixel(motorID);
      Serial.println("Too toasty. Pull the handbrake.");
      Serial.flush();             // Buffer flushed after reboot
    }
  }
}

unsigned short serialFunctions::update_cRc(unsigned short cRc_accum, unsigned char* data_blk_ptr, unsigned short data_blk_size)
{
  unsigned short i, j;
  unsigned short cRc_table[256] =
  {
    0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
    0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
    0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
    0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
    0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
    0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
    0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
    0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
    0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
    0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
    0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
    0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
    0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
    0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
    0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
    0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
    0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
    0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
    0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
    0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
    0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
    0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
    0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
    0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
    0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
    0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
    0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
    0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
    0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
    0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
    0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
    0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
  };

  for ( j = 0; j < data_blk_size; j++ )
  {
    i = ((unsigned short)(cRc_accum >> 8) ^ data_blk_ptr[j]) & 0xff;
    cRc_accum = (cRc_accum << 8) ^ cRc_table[i];
  }
  return cRc_accum;
}

serialFunctions dynamixel;                  // Makes it easier to call ftcs.

/***********************************************************************************************************
 * All the necessary functions have now been defined and can be call from the default setup- and loop ftc. *
 * Arranging these into classes could have been left out, but was done to make the code more portable.     *
 ***********************************************************************************************************/

void setup()
{
  Serial.flush();                           // Starts by clearing the serial buffer of garbage data before running the code.
  Serial1.begin(dynaBaud);                  // Initializes communication with Arduino at fixed baud rate for Dynamixel MX-series
  Serial.begin(dynaBaud);                   // Initializes communication with Serial device (monitor) at fixed baud rate for Dynamixel MX-series
  pinMode(directPin, OUTPUT);               // Set direction pin to output-mode
  Serial.println("It's on!");               // A hearty welcome
  Serial.println("Type an integer [1, 5] directly into the console and hit \"send\" (or enter) to trigger instructions to the servo.");
  Serial.println("\'1\' enables torque");
  Serial.println("\'2\' disables torque");
  Serial.println("\'3\' sets a proportional gain of \'6\'");
  Serial.println("\'4\' records temperature to monitor");
  Serial.println("\'5\' reboots Dynamixel");
}

void loop()                                 // Based on user-input, loop ftc calls five ftcs from the serialFunctions class defined earlier
{
  actTime = millis();
  actTime = actTime + 100;                  // 100 ms increment per iteration (10Hz)
  hlpTime = millis();
  String instruction;

  if (Serial.available() > 0)
  {
    instruction = Serial.readStringUntil('\n');
    delay(100);                             // Delay time fits with time-increment (10Hz)
  }

  if ( instruction.equals("1") )
  {
    dynamixel.applyTorque(motorID, true);
  }

  if ( instruction.equals("2") )
  {
    dynamixel.applyTorque(motorID, false);
  }

  if ( instruction.equals("3") )
  {
    dynamixel.setpGain(motorID, pGain);
  }

  if ( instruction.equals("4") )
  {
    while (1)
    {
      dynamixel.temperature(motorID);
      Serial.println(actTime-hlpTime);
      delay(actTime-hlpTime);
    };
  }

  if ( instruction.equals("5") )
  {
    dynamixel.rebootDynamixel(motorID);
  }
}

// "...and there was much rejoicing"
