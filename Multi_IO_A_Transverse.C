/*******************************************************************

	Multi_IO_A_Transverse.c
  	Mike Schoonover 2015

	This program is used with RCM4000 series controllers.

	Description
	===========

   This is the software for the Multi-IO Series Transverse boards.

   There are two different types of Transverse boards: Ring 1 & Ring 2.

   Set RING constant to 1 or 2 to compile for either version.

	Instructions
	============
	1.	Compile and run this program.

    I/O Assignments

      5 Volt Inputs (higher voltage requires inline resistor)

      	Encoder1 (Rotational)

			A  : Input 1, Port A0
			B  : Input 2, Port A1

      	Encoder2 (Linear)

			A  : Input 3, Port A2
			B	: Input 4, Port A3

			Unused			: Input 5, Port A4
			Unused			: Input 6, Port A5

		12 Volt inputs (higher voltage requires inline resistor)

			Inspection Control	: Input 7,	Port A6
      	Unused 					: Input 8,	Port A7
      	Top Dead Center		: Input 9, 	Port E0
     		Unused					: Input 10, Port E1

		Outputs (driven by host, so purpose is defined by host

			Out 1:	multi-purpose ground switching transistor control
			Out 2:	multi-purpose ground switching transistor control
			Out 3:	multi-purpose ground switching transistor control
			Out 4:	multi-purpose ground switching transistor control


 Inspection Control Signal from PLC

 	low voltage signal (read as 1) -- no pipe in system
 	low to high voltage transition (read as 1 to 0) --
	   leading vertical eye has detected end of tube
   short pulse high-low-high (read as 0-1-0) -- head 1 lowered or raised
   longer pulse high-low-high (read as 0-1-0) -- head 2 lowered or raised
	high to low for longer time than above (read as 0-1) --
   	trailing vertical eye has cleared end of tube

 Inspection Control Commands to Host

 	1 - no tube detected
 	2 - leading vertical eye is on tube
   3 - head 1 lowered
   4 - head 2 lowered
   5 - head 1 raised
   6 - head 2 raised

   Each time the command changes to reflect a new action (such as
   head 1 lowering), it stays at that value until a new action occurs
   (such as head 2 lowering).

	When inspecting in direction away from home, typical sequence is:
   	1, 2, 3, 4, 5, 6, 1

	For direction towards home:
   	1, 2, 4, 3, 6, 5, 1

      Note: These assume that carriage is not reversed off the tube
      		or any action taken that causes the heads to be
            raised/lowered out of sequence. In those cases, the
            command sequence will vary accordingly.


	UT Main Pulse Sync and Track Sync Control Lines

	Track Sync:

   	The Track Sync and Track Sync Reset lines allow the Control Board to send
	   signals to other boards in the chassis, such as UT Boards. This signal can
   	be used to transmit rotational position (TDC and clock) or linear position
	   information.

   	Tracking Modes:

		The host can select between different tracking modes by setting flags
	   in the controlFlags variable:

		RABBIT_SEND_CLOCK_MARKERS

   	Pulses the Track Sync Reset line (along with a Track Sync pulse to clock
      in the reset) each time the Top-Dead-Center signal is received. Pulses
      the Track Sync line once at each clock position (11) between the TDC
      signals. Thus the reset occurs once per revolution while the sync
      occurs 12 times per revolution.

		RABBIT_SEND_TDC

   	Pulses the Track Sync line once each time the Top-Dead-Center signal is
      received. Only performs a Track Sync Reset (along with a Track Sync pulse
      to clock in the reset) if requested by the Host computer.

   UT Main Pulse Sync:

   This program does not currently manipulate the Pulse Firing signals. One
   of the UT boards is usually set up as the master to control those lines.

*******************************************************************/

#class auto

#use RCM42xx.LIB

// Possible options for temporarily storing software update images.  Using
// temporary storage allows the image to be verified before it is used to
// replace the existing code.  The DIRECT_WRITE option does not store or verify
// the image first, so it is more dangerous to use.
//
//	#define BU_TEMP_USE_FAT				// use file on FAT filesystem
//	#define BU_TEMP_USE_SBF				// use unused portion of serial boot flash
//	#define BU_TEMP_USE_SFLASH			// use serial data flash (without FAT)
//	#define BU_TEMP_USE_DIRECT_WRITE	// write directly to boot firmware image

//store software update images in serial flash to allow for verification
#define BU_TEMP_USE_SFLASH			// use serial data flash (without FAT)

//page in Serial Flash to store temporary copies of software uploads
#define BU_TEMP_PAGE_OFFSET 0

//this "use" statement must be below the BU_TEMP_USE_xxx statement
#use "board_update.lib"

//this is the roll call query string expected from the host computer

#define ROLL_CALL_QUERY "Device Query"

//this is the response to the host computer's roll call -- it is used to
//identify the board type

#define ROLL_CALL_RESPONSE "Transverse Ring 1 Multi-IO Board A"

// Control Flags Bit Defines (controlFlags)
// The control register flags are set by the host to control functions of
// the Rabbit microprocessor.

// transmit tracking pulse to DSPs for every o'clock position and a reset
// pulse at every TDC
#define RABBIT_SEND_CLOCK_MARKERS 			0x0001
// transmit a single pulse at every TDC detection and a reset pulse for
// every linear advance of one step
#define RABBIT_SEND_TDC							0x0002
// enables the sending of track sync pulses (does not affect track reset pulses)
#define TRACK_PULSES_ENABLED					0x0004

//rotational
#define ENC1A 0  		// bit on Port A
#define ENC1B 1		// bit on Port A

//linear
#define ENC2A 2		// bit on Port A
#define ENC2B 3		// bit on Port A

#define UNUSED1 4		// bit on Port A
#define UNUSED2 5		// bit on Port A
#define INSPECT 6		// bit on Port A
#define UNUSED3 7		// bit on Port A
#define TDC 0			// bit on Port E
#define UNUSED4 1		// bit on Port E

#define ON_PIPE_CTRL 1
#define HEAD1_DOWN_CTRL 2
#define HEAD2_DOWN_CTRL 4

#define DS2 0
#define DS3 1

#define DS2_BIT 2
#define DS3_BIT 3

// Notes about the IP Address and Subnet Mask
// When a Windows computer is connected to the local network with only the
// Rabbit modules, it will assign itself an IP Address such as 169.254.56.136
// and a Mask Subnet of 255.255.0.0 because there is no DHCP server to assign
// these values to the hosts on the network.
//
// Each host (Windows computer and Rabbits) uses the Subnet Mask to determine
// if the computer it is connecting to is on the same subnet.  If it is on the
// same subnet, the data is sent directly.  If not, the computer sends it
// through the router (default gateway).  The part of the mask with ones is the
// part which specifies the local subnet - this part should match in all hosts
// on the subnet.  The Subnet Mask should also be the same in all hosts so they
// all understand which computers are on the same subnet.
//
// To use a Windows computer to talk to the Rabbits, you can either manually
// set the IP Address and Subnet Mask to match the Rabbits or set the Rabbits
// to match the Windows computer.  Since the Windows computer may also be used
// on other networks, it is inconvenient to switch back and forth; thus the
// Rabbits in this system use values which match the typical Windows computer.
//
// When the Windows computer is connected without manually setting the
// IP Address and Subnet Mask, a yellow warning sign will be displayed by the
// network icon and the warning "Limited or no connectivity" will be shown.
// This does not affect communication with the Rabbits and the warning may be
// ignored.  The warning sign being displayed varies between the various
// versions of Windows.
//


//TCPIP Defines
#define TCPCONFIG 1
#define _PRIMARY_STATIC_IP "169.254.56.11" //this gets changed later
#define _PRIMARY_NETMASK "255.255.0.0"
#define MY_GATEWAY "10.10.6.1"
#define MY_NAMESERVER "10.10.6.1"
#define PORT 23
#define TCP_BUF_SIZE 4096

//UDP Defines
#define USE_MULTICAST
#define MAX_UDP_SOCKET_BUFFERS 1

#use "dcrtcp.lib"  //tcpip library

//Change this number when the code is changed.
#define VERSION "1.2t"

//firmware code upload buffer size is 1024 data bytes plus one command byte
#define CODE_BUFFER_SIZE 1025

// definitions for Interrupt Vector(s)
#define	INT_TIMERA		0x0A
#define	INT_TIMERB		0x0B

// masks for Timer B Clock Source
#define TBCR_CLOCK_PCLK2	0x00
#define TBCR_CLOCK_PCLK16	0x08
#define TBCR_CLOCK_A1		0x04

// masks for Timer B Interrupt Priority
#define TBCR_NO_INT			0x00
#define TBCR_INT_PRI_1		0x01
#define TBCR_INT_PRI_2		0x02
#define TBCR_INT_PRI_3		0x03

// masks for Timer B Interrupt Enable
#define TBCSR_INT_MATCH1	0x02
#define TBCSR_INT_MATCH2	0x04

// masks for Timer B Main Clock
#define TBCSR_MAIN_CLK_DIS	0x00
#define TBCSR_MAIN_CLK_ENA	0x01

// values for Timer B Match Registers
// NOTE: this is not currently used - the simpler reload value of 0 is used
#define TIMERB_INCREMENT 900	// this is the delay in clocks
const int timerb_inc= TIMERB_INCREMENT&0xff | ((TIMERB_INCREMENT<<6) & 0xc000);

// I/O port bits

//port C

//C1, C3, C5, C7 are used for external outputs because on reset they default
//to inputs and will not improperly fire external devices

#define OUTPUT1 1		// bit on Port C
#define OUTPUT2 3		// bit on Port C
#define OUTPUT3 5		// bit on Port C
#define OUTPUT4 7		// bit on Port C

//C0, C2 default to outputs on reset - they are used for position tracking
//signals between boards in the system and their startup state is not important

#define TRACK_SYNC 0				// bit on Port C
#define TRACK_SYNC_RESET 2		// bit on Port C

//E3, E4 - used for UT pulse syncing between UT boards -- not normally driven
//by the Control board

#define PULSE_SYNC 3				// bit on Port E
#define PULSE_SYNC_RESET 4		// bit on Port E

#define HEAD1_SIGNAL_WIDTH 10	//width of signal pulse for head 1 up/down
#define HEAD2_SIGNAL_WIDTH 30	//width of signal pulse for head 2 up/down
#define OFF_PIPE_SIGNAL_WIDTH 40 //width of signal pulse for off pipe (or longer)

#define INSPECT_CONTROL_DEGLITCH_COUNT 20
#define TDC_DEGLITCH_COUNT 10

#define TRACK_SYNC_PULSE_WIDTH 20
#define TRACK_RESET_PULSE_WIDTH 20
#define TRACK_RESET_PULSE_HALF_WIDTH 10

//----------------------------------------------------------------------------
// Global Variables

unsigned int controlFlags = 0;	//flags set by host to control board functions

// byte for system status flags
char systemStatus = 0;

unsigned int globalDebug = 0x1234;

char pktID;

int reSynced, reSyncCount, reSyncPktID;

int pktError; //wrong size, checksum error, execution error

unsigned timerb_match; // match value
long timerBCount; //incremented with each timerb interrupt
long prevCountBTemp;
char countBAccessFlag;	// used to prevent ISR updating counter values while
						// they are being accessed by the non-ISR code
long tickCount; //incremented approximately every .01 seconds

int trackSyncPulseActive = FALSE;
long trackSyncPulseWidthCount = 0;
int trackSyncResetPulseActive = FALSE;
long trackSyncResetPulseWidthCount = 0;
long trackSyncPulseInactiveCount = -1;
long trackSyncPulseStartDelayCount = -1;

int resetPulseTrack = FALSE;

long encoderPosAtOnPipeSignal = 0;
long encoderPosAtOffPipeSignal = 0;
long encoderPosAtHead1DownSignal = 0;
long encoderPosAtHead1UpSignal = 0;
long encoderPosAtHead2DownSignal = 0;
long encoderPosAtHead2UpSignal = 0;

/* used along with TDC photo eye simulation code
long trackSyncResetPulseInactiveCount = 0; //debug mks
int tdcInput = 1; //debug mks
*/


short output1State, output1Timer;
short output2State, output2Timer;
short output3State, output3Timer;
short output4State, output4Timer;

long encoder1Count; 	// counter for tracking encoder 1 position
long encoder2Count;	// counter for tracking encoder 1 position
char encAccessFlag; // used to prevent ISR updating counter values while they
					// are being accessed by the non-ISR code

//number of encoder counts to trigger an update to the host
short encoder1Delta, encoder2Delta;

int forceSendInspectPacket = FALSE;

//set inspectionCtrl to a number other than 0 or 1 so the first time through
//it will not match the port bit read
int inspectionCtrl = 255;
long inspectionCtrlTimer = 0;
long inspectionCtrlTime = 0;
int inspectionStatus = 0;
int onPipe = FALSE;
int head1Down = FALSE;
int head2Down = FALSE;

int inspectMode = FALSE;

int monitorMode = FALSE;

int inspectPacketCount = 0;

byte status = 0;

int rpm = 0, prevRPM = 0, tdcOffReflector = 0;
long tdcPeriodCount=0;
long prevTDCPeriodCount=0;
long clockPeriodCount;
int maxClockPositions = 12;
long prevTDCCount = 0;
int rpmCorrectionCount = 0;
int rpmVariance;

// end of Global Variables
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// Global Variables for Monitor Function

int prevEnc1A, prevEnc1B, prevEnc2A, prevEnc2B;
int prevInspect, prevTDC;
int prevUnused1, prevUnused2, prevUnused3, prevUnused4;
int chassisAddr, slotAddr;
int prevChassisAddr, prevSlotAddr;

union {
	long lVal;
	char cVal[4];
	} enc1CntTemp;

union {
	long lVal;
	char cVal[4];
	} enc2CntTemp;

int sendMonitorPacket = FALSE;

// end of Global Variables for Monitor Function
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// Global Variables for processInspect Function

long prevEnc1Cnt, prevEnc2Cnt;

// end of Global Variables for processInspect Function
//----------------------------------------------------------------------------

// Command definitions from Host
//These should match the values in the host code.

#define NO_ACTION 0
#define GET_INSPECT_PACKET_CMD 1
#define ZERO_ENCODERS_CMD 2
#define GET_MONITOR_PACKET_CMD 3
#define PULSE_OUTPUT_CMD 4
#define TURN_ON_OUTPUT_CMD 5
#define TURN_OFF_OUTPUT_CMD 6
#define SET_ENCODERS_DELTA_TRIGGER_CMD 7
#define START_INSPECT_CMD 8
#define STOP_INSPECT_CMD 9
#define START_MONITOR_CMD 10
#define STOP_MONITOR_CMD 11
#define GET_STATUS_CMD 12
#define LOAD_FIRMWARE_CMD 13
#define SEND_DATA_CMD 14
#define DATA_CMD 15
#define GET_CHASSIS_SLOT_ADDRESS_CMD 16
#define SET_CONTROL_FLAGS_CMD 17
#define RESET_TRACK_COUNTERS_CMD 18
#define GET_ALL_ENCODER_VALUES_CMD 19

#define ERROR 125
#define DEBUG_CMD 126
#define EXIT_CMD 127

#define NO_STATUS 0


//----------------------------------------------------------------------------
// send2Bytes
//
// Sends two bytes via the specified socket.
//

void send2Bytes(tcp_Socket *socket, char byte1, char byte2)
{

	char buffer[2];
	buffer[0] = byte1; buffer[1] = byte2;
	sock_flushnext(socket);
	sock_write(socket, buffer, 2);

}//end of send2Bytes
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// send3Bytes
//
// Sends three bytes via the specified socket.
//

void send3Bytes(tcp_Socket *socket, char byte1, char byte2, char byte3)
{

	char buffer[3];
	buffer[0] = byte1; buffer[1] = byte2; buffer[2] = byte3;
	sock_flushnext(socket);
	sock_write(socket, buffer, 3);

}//end of send3Bytes
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// send2BytesUDP
//
// Sends two bytes via the specified UDP socket to host at IP address pIPAddr.
// Always sends to port 4445 on the remote.
//

void send2BytesUDP(
					udp_Socket *pSocket, longword pIPAddr, char byte1, char byte2)
{

	char buffer[2];
	buffer[0] = byte1; buffer[1] = byte2;
	udp_sendto(pSocket, buffer, 2, pIPAddr, 4445);

}//end of send2BytesUDP
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// readBytesAndVerify
//
// Attempts to read pNumBytes number of bytes from pSocket into pBuffer and
// verifies the data using the last byte as a checksum.
//
// Note: pNumBytes should include the data bytes AND the checksum byte.
//
// The packet ID should be provided via pPktID -- it is only used to verify the
// checksum as it is included in that calculation by the host.
//
// Returns the number of bytes read.
// On checksum error, returns -1.
// If pNumBytes cannot be read, returns -2.
//

int readBytesAndVerify(tcp_Socket *pSocket, char *pBuffer, int pNumBytes,
																						int pPktID)
{

	int i;
	int bytesRead;
   char checksum = 0;

   bytesRead = sock_fastread(pSocket, pBuffer, pNumBytes);

   if (bytesRead != pNumBytes) return(-2);

   //validate checksum by summing the packet id and all data along with the
   //checksum byte

	for(i = 0; i < pNumBytes; i++){
   	checksum += pBuffer[i];
   }

   if ( ((pPktID + checksum) & 0xff) != 0) {return(-1);}
   else
	   return(bytesRead);

}//end of readBytesAndVerify
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// cleanUpFirmwareInstall
//
// Prints status message and sends it to host, then cleans up any loose ends
// from the firmware install process and exits.
//

int cleanUpFirmwareInstall(tcp_Socket *socket, char *pMsg)
{

   printf(pMsg);
   sock_flushnext(socket);
   sock_write(socket, pMsg, strlen(pMsg));
   sock_close(socket);
   while(tcp_tick(socket)) {}

   while (buCloseFirmware() == -EBUSY);
   exit(0);

}//end of cleanUpFirmwareInstall
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// installNewFirmware
//
// Replaces the software running in the Rabbit micro-controller.  Verifies the
// new firmware image stored in the temporary location and then installs it
// over the old code.
//
// NOTE: The firmware update process was failing 1 out of 5 times until
//       time delay (by adding code sending message to host) was inserted
//			before the exit(0) call. Unsure why this solved the problem.
//
//	Return Codes:
//		Note that this function will either succeed and reboot to new firmware,
//		or it will fail with one of the following error codes:
//
//	      -EILSEQ: Not a valid firmware_info_t struct (bad marker bytes
//	               or unsupported version of structure).
//	      -EBADMSG: Bad CRC (structure has been corrupted).
//	      -ENODATA: Source not open, or firmware info not found in source.
//	      -EPERM: Firmware was compiled for a different target.
//	      -EBADDATA: CRC-32 mismatch, firmware image corrupted.
//	      -EBADMSG: CRC-32 mismatch after installing.
//	      -ENOMEM: Couldn't allocate buffer to copy firmware.
//
//	      Error codes when using a FAT file for temporary storage:
//	      -EINVAL: Couldn't parse BU_TEMP_FILE.
//	      -ENOENT: File BU_TEMP_FILE does not exist.
//	      -EMFILE: Too many open files.
//
//	      Error codes when using the serial flash for temporary storage:
//	      -ENODEV: Can't find/read the serial flash.
//
// On success, returns 1 regardless of the number of bytes read from the socket.
//

int installNewFirmware(tcp_Socket *socket)
{

   char buffer[200];

   firmware_info_t fi;
   int         i;
   int         result;
   int         progress;

   printf( "Verifying and installing new firmware...\n");

   result = buOpenFirmwareTemp(BU_FLAG_NONE);

   if (result){
      sprintf(buffer, "Error %d opening firmware image.\n", result);
      cleanUpFirmwareInstall(socket, buffer);
   }

   // buGetInfo is a non-blocking call, and may take multiple attempts
   // before the file is completely open.
   do {
      result = buGetInfo( &fi);
   } while ( (result == -EBUSY));

   if (result){
      sprintf(buffer, "Error %d getting firmware image info.\n", result);
      cleanUpFirmwareInstall(socket, buffer);
   }

   printf( "Found %s v%u.%02x...\n", fi.program_name,
         fi.version >> 8, fi.version & 0xFF);

   printf( "Attempting to install new version...\n");
   progress = 0;

   do{
      printf( "\r verify %u.%02u%%\r", progress / 100, progress % 100);

      //verify the firmware image
      result = buVerifyFirmware(&progress);

   } while (result == -EAGAIN);

   if (result){
      sprintf(buffer, "Error %d while verifying image.\n", result);
      cleanUpFirmwareInstall(socket, buffer);
	}

   printf( "Verify complete, installing new firmware...\n");

   //install the firmware image
   result = buInstallFirmware();

   if (result){
      //put the firmware running in ram back into flash on failure
      //buInstallFirmware supposedly does this, but have had issues with that
      buRestoreFirmware(0);

      sprintf(buffer, "Error %d while installing firmware.\n", result);
      cleanUpFirmwareInstall(socket, buffer);
      }

	//send message to host and exit
   sprintf(buffer, "Install successful: rebooting.\n");
   cleanUpFirmwareInstall(socket, buffer);

   return result;

}//end of installNewFirmware
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// receiveAndInstallNewFirmware
//
// Receives a Rabbit firmware image file from the host and overwrites the
// existing code.
//
// The image is received in 1024 byte packets, with the last packet being
// smaller as necessary.
//
// NOTE: The firmware update functions alter the Port C configuration.
// 		Currently, this function will halt the program whether or not
//			the update succeeds. This eliminates the need to reset Port C. If
//			That behavior is changed, then Port C will need to be reset before
//			program execution can continue.
//
//	Return Codes:
//
//	Return codes:
//		Most return codes come from install_firmware (see above).
//
//		Additional return codes for tftp_and_install:
//       -EPERM: buTempCreate not supported on this hardware.
//       -ENODEV: Couldn't read from serial flash.
//       -EBUSY: Timeout waiting for FAT filesystem.
//       -ENODATA: Download didn't contain a valid firmware image for this
//         		device.
//       <0: Error opening FAT file, see fat_Open for full list of
//             error codes and their meanings.
//

int receiveAndInstallNewFirmware(tcp_Socket *socket, int pPktID)
{

   int bytesRead, offset;
   char  codeBuffer[CODE_BUFFER_SIZE];
   char  bufferIn[5], bufferOut[5];
   int x,y, z, bufPtr, packetCount;
   char shiftReg;
   int result;

   printf("Loading new firmware...\n");

   //read in the remainder of the packet
   result = readBytesAndVerify(socket, bufferIn, 1, pPktID);
   if (result < 0) return(result);

   packetCount = 0;

   // set P2 back to active drive for use with the serial flash
   WrPortI(PCDCR, &PCDCRShadow, (PCDCRShadow & 0xfb));

   while ( (result = buTempCreate()) == -EBUSY);
   if (result){
      printf( "Error %d calling buTempCreate!\n", result);
      //send the error code back as MSB:LSB
      send3Bytes(socket, ERROR, (result >> 8) & 0xff, result & 0xff);
      return result;
   }

   //request first data packet from host
   send3Bytes(socket, SEND_DATA_CMD, NO_STATUS, NO_STATUS);

   printf("Receiving firmware packets...\n");

   //process packets until exit command packet received from host
   do{

      //wait for data or exit command packet from host
      do{

         codeBuffer[0] = NO_ACTION; //clean out the command byte

         //wait for data packet from host - last packet is not full sized so
         //just assume every packet is complete if a byte is received
         if (sock_bytesready(socket) > 0)
            bytesRead=sock_fastread(socket, codeBuffer, CODE_BUFFER_SIZE);

      } while(tcp_tick(socket) && codeBuffer[0] == NO_ACTION);

      // if the packet is data, store it in the temporary storage location
      if (codeBuffer[0] == DATA_CMD){

         packetCount++;
         printf( "Packet #: %d...\n", packetCount);

         //request next data packet from host while processing the previous one
         send3Bytes(socket, SEND_DATA_CMD, NO_STATUS, NO_STATUS);

         if (bytesRead > 0){

            // buTempWrite is non-blocking, so it may take multiple calls to
            // complete the write -- must call it repeatedly while updating the
            // offset to make sure it writes the entire buffer

            //the first byte is a command byte, the rest are data bytes, so skip
            //the command byte at index 0
            offset = 1;

            while (offset < bytesRead)
            {
               result = buTempWrite( &codeBuffer[offset], bytesRead - offset);

               if (result == -EBUSY){
                  // resources busy, try again without any changes
               }
               else if (result < 0){
                  printf(
                  	"Error %d writing firmware to temp location.\n", result);
                  send3Bytes(
                  	socket, ERROR, (result >> 8) & 0xff, result & 0xff);
                  return(result);
               } else {
                  offset += result;
               }
            }
         }//if (bytesRead > 0)
      }// if (codeBuffer[0] == DATA_CMD)

   } while(tcp_tick(socket) && codeBuffer[0] != EXIT_CMD);

   //install the newly downloaded firmware
   return installNewFirmware(socket);

   // if installNewFirmware returned without exiting the program, then the
   // install failed -- exit the program to force a restart from known state

	printf( "Install failed -- exiting program.\n");
   while (buCloseFirmware() == -EBUSY);
   exit(0);

   return(result);

}//end of receiveAndInstallNewFirmware
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// sendPacketHeader
//
// Sends via the socket: 0xaa, 0x55, 0xaa, 0x55, and the packet identifier.
//
// The socket is not flushed in anticipation of more bytes being sent to
// complete the packet.
//

void sendPacketHeader(tcp_Socket *socket, char pPacketID)
{

   char buffer[5];
   buffer[0] = 0xaa; buffer[1] = 0x55; buffer[2] = 0xbb; buffer[3] = 0x66;
   buffer[4] = pPacketID;
   sock_write(socket, buffer, 5);

}//end of sendPacketHeader
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// getStatus
//
// Sends the status byte via socket.  The status byte tells the state of the
// system.
//
// The value is sent back to the host via the socket in a 2 byte packet.  The
// first byte will be the value, the second byte is used for  debugging --
// the code is changed to return whatever value is of interest.
//

int getStatus(tcp_Socket *socket, int pPktID)
{

   char buffer[1];
   int result;
   int debugValue;

   //return any value of interest via debugValue
	// example: debugValue = (controlFlags >> 8) & 0xff;
   // example: debugValue = controlFlags & 0xff;
   //  or any other 8 bit value can be returned

	debugValue = globalDebug & 0xff;

   //read in the remainder of the packet
   result = readBytesAndVerify(socket, buffer, 1, pPktID);
   if (result < 0) return(result);

   sendPacketHeader(socket, GET_STATUS_CMD);
   send2Bytes(socket, systemStatus, debugValue);

   return(1); //return number of bytes read from socket

}//end of getStatus
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// getAllEncoderValues
//
// Returns all encoder values saved at different points in the inspection
// process.
//
// The data is sent back to the host via the socket.
//

int getAllEncoderValues(tcp_Socket *socket, int pPktID)
{

   char buffer[50];
   int x, result;

   //read in the remainder of the packet
   result = readBytesAndVerify(socket, buffer, 1, pPktID);
   if (result < 0) return(result);

   x = 0;

   //place encoders values into the buffer by byte, MSB first

   buffer[x++] = (char)((encoderPosAtOnPipeSignal >> 24) & 0xff);
   buffer[x++] = (char)((encoderPosAtOnPipeSignal >> 16) & 0xff);
   buffer[x++] = (char)((encoderPosAtOnPipeSignal >> 8) & 0xff);
   buffer[x++] = (char)(encoderPosAtOnPipeSignal & 0xff);

   buffer[x++] = (char)((encoderPosAtOffPipeSignal >> 24) & 0xff);
   buffer[x++] = (char)((encoderPosAtOffPipeSignal >> 16) & 0xff);
   buffer[x++] = (char)((encoderPosAtOffPipeSignal >> 8) & 0xff);
   buffer[x++] = (char)(encoderPosAtOffPipeSignal & 0xff);

   buffer[x++] = (char)((encoderPosAtHead1DownSignal >> 24) & 0xff);
   buffer[x++] = (char)((encoderPosAtHead1DownSignal >> 16) & 0xff);
   buffer[x++] = (char)((encoderPosAtHead1DownSignal >> 8) & 0xff);
   buffer[x++] = (char)(encoderPosAtHead1DownSignal & 0xff);

   buffer[x++] = (char)((encoderPosAtHead1UpSignal >> 24) & 0xff);
   buffer[x++] = (char)((encoderPosAtHead1UpSignal >> 16) & 0xff);
   buffer[x++] = (char)((encoderPosAtHead1UpSignal >> 8) & 0xff);
   buffer[x++] = (char)(encoderPosAtHead1UpSignal & 0xff);

   buffer[x++] = (char)((encoderPosAtHead2DownSignal >> 24) & 0xff);
   buffer[x++] = (char)((encoderPosAtHead2DownSignal >> 16) & 0xff);
   buffer[x++] = (char)((encoderPosAtHead2DownSignal >> 8) & 0xff);
   buffer[x++] = (char)(encoderPosAtHead2DownSignal & 0xff);

   buffer[x++] = (char)((encoderPosAtHead2UpSignal >> 24) & 0xff);
   buffer[x++] = (char)((encoderPosAtHead2UpSignal >> 16) & 0xff);
   buffer[x++] = (char)((encoderPosAtHead2UpSignal >> 8) & 0xff);
   buffer[x++] = (char)(encoderPosAtHead2UpSignal & 0xff);

   //send the buffer to host
   sendPacketHeader(socket, GET_ALL_ENCODER_VALUES_CMD);
   sock_flushnext(socket);
   //x already incremented properly for use as packet size
   sock_write(socket, buffer, x);

   return(1); //return number of bytes read from socket

}//end of getAllEncoderValues
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// zeroAllEncoderValues
//
// Sets counters for the encoders to zero.
//
// Zeroes all encoder count variables.
//

int zeroAllEncoderValues()
{

   encAccessFlag = 1; // block the ISR from modifying

   encoder1Count = 0;
   encoder2Count = 0;

   encAccessFlag = 0; // unblock the ISR

   //reset various encoder values

	encoderPosAtOnPipeSignal = 0;
	encoderPosAtOffPipeSignal = 0;
	encoderPosAtHead1DownSignal = 0;
	encoderPosAtHead1UpSignal = 0;
	encoderPosAtHead2DownSignal = 0;
	encoderPosAtHead2UpSignal = 0;

}//end of zeroAllEncoderValues
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// zeroEncoderCounts
//
// Sets counters for the encoders to zero.
//

int zeroEncoderCounts(tcp_Socket *socket, int pPktID)
{

   int x, result;
   char buffer[2];

   //read in the remainder of the packet
   result = readBytesAndVerify(socket, buffer, 2, pPktID);
   if (result < 0) return(result);

   zeroAllEncoderValues();

   return(0);

}//end of zeroEncoderCounts
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// pulseOutput
//
// Pulses the output lines specified by the byte in the packet from the host.
// To pulse, the ouput line is turned on here and turned off by the interrupt
// timer routine after a specified time.
//
// WIP MKS - this function currently pulses output1 without looking at the
//  data byte from the host - code needs to be changed so that each output
//  is specified by a different bit in the data byte.
// 1/10/15
//  changed to use the data byte value as a channel number...better if each
//  bit is a channel to enable pulsing multiple outputs with one call
//
// WIP MKS - in the future, the second and third bytes from the host will
// specify the time in .01 second increments to fire the pulse.
//

int pulseOutput(tcp_Socket *socket, int pPktID)
{

   int x, result;
   int output;
   char buffer[2];

   output = OUTPUT1;

   //read in the remainder of the packet
   result = readBytesAndVerify(socket, buffer, 2, pPktID);
   if (result < 0) return(result);

   //choose output bit, set flag that output is on, set count down timer

   if (buffer[0] == 0) {
   	output = OUTPUT1; output1State = TRUE; output1Timer = 15;}
   else
   if (buffer[0] == 1){
   	output = OUTPUT2; output2State = TRUE; output2Timer = 15;}
   else
   if (buffer[0] == 2){
   	output = OUTPUT3; output3State = TRUE; output3Timer = 15;}
   else
   if (buffer[0] == 3){
   	output = OUTPUT4; output4State = TRUE; output4Timer = 15;}

   //set output to 0 which turns on the optoisolator
   BitWrPortI(PCDR, &PCDRShadow, 0, output);
   //flag so other functions can know state

   return(0);

}//end of pulseOutput
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// turnOnOutput
//
// Turns on the output lines specified by the byte in the packet from the host.
//
// WIP MKS - this function currently turns on output1 without looking at the
//  data byte from the host - code needs to be changed so that each output
//  is specified by a different bit in the data byte.
//

int turnOnOutput(tcp_Socket *socket, int pPktID)
{

   int x, result;
   char buffer[2];

   //read in the remainder of the packet
   result = readBytesAndVerify(socket, buffer, 2, pPktID);
   if (result < 0) return(result);

   //set output to 0 which turns on the optoisolator
   BitWrPortI(PCDR, &PCDRShadow, 0, OUTPUT1);
   //flag so other functions can know state
   output1State = TRUE;

   return(0);

}//end of turnOnOutput
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// turnOffOutput
//
// Turns off the output lines specified by the byte in the packet from the host.
//
// WIP MKS - this function currently turns off output1 without looking at the
//  data byte from the host - code needs to be changed so that each output
//  is specified by a different bit in the data byte.
//

int turnOffOutput(tcp_Socket *socket, int pPktID)
{

   int x, result;
   char buffer[2];

   //read in the remainder of the packet
   result = readBytesAndVerify(socket, buffer, 2, pPktID);
   if (result < 0) return(result);

   //set output to 1 which turns off the optoisolator
   BitWrPortI(PCDR, &PCDRShadow, 1, OUTPUT1);
   //flag so other functions can know state
   output1State = FALSE;

   return(0);

}//end of turnOffOutput
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// setEncodersDeltaTrigger
//
// Tells the Control board how many encoder counts to wait before sending
// an encoder value update.  The trigger value for each encoder is sent.
//
// Normally, this value will be set to something reasonable like 1 inch of
// travel of the piece being inspected.
//

int setEncodersDeltaTrigger(tcp_Socket *socket, int pPktID)
{

   int x, result;
   char buffer[5];

   //read in the remainder of the packet
   result = readBytesAndVerify(socket, buffer, 5, pPktID);
   if (result < 0) return(result);

   x = 0;

   //get the encoder 1 delta trigger
   encoder1Delta = (unsigned)((buffer[x++]<<8) & 0xff00)
                                     + (unsigned)(buffer[x++] & 0xff);

   encoder2Delta = (unsigned)((buffer[x++]<<8) & 0xff00)
                                     + (unsigned)(buffer[x++] & 0xff);

   return(0);

}//end of setEncodersDeltaTrigger
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// startInspect
//
// Puts Control board in the inspect mode.  In this mode the Control board
// will monitor encoder and status signals and return position information to
// the host.
//

int startInspect(tcp_Socket *socket, int pPktID)
{

   int x, result;
   char buffer[2];

   //read in the remainder of the packet
   result = readBytesAndVerify(socket, buffer, 2, pPktID);
   if (result < 0) return(result);

   zeroAllEncoderValues();

   //start processing inputs and sending inspect data packets
   inspectMode = TRUE;

   return(0);

}//end of startInspect
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// stopInspect
//
// Exits the inspect mode
//

int stopInspect(tcp_Socket *socket, int pPktID)
{

   int x, result;
   char buffer[2];

   //read in the remainder of the packet
   result = readBytesAndVerify(socket, buffer, 2, pPktID);
   if (result < 0) return(result);

   inspectMode = FALSE;

   return(0);

}//end of stopInspect
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// sendInspectPacket
//
// Sends a packet with data necessary for inspection tracking such as inputs,
// encoder counts, status flags, etc...
//

int sendInspectPacket(tcp_Socket *socket)
{

   int pktSize = 12;
   char buffer[12];
   int x = 0;
   char controlFlags;

   sendPacketHeader(socket, GET_INSPECT_PACKET_CMD);

   //send the packet count back to the host, MSB followed by LSB
   buffer[x++] = (inspectPacketCount >> 8) & 0xff;
   buffer[x++] = inspectPacketCount++ & 0xff;

   //place the encoder 1 values into the buffer by byte, MSB first
   buffer[x++] = enc1CntTemp.cVal[3];
   buffer[x++] = enc1CntTemp.cVal[2];
   buffer[x++] = enc1CntTemp.cVal[1];
   buffer[x++] = enc1CntTemp.cVal[0];

   //place the encoder 2 values into the buffer by byte, MSB first
   buffer[x++] = enc2CntTemp.cVal[3];
   buffer[x++] = enc2CntTemp.cVal[2];
   buffer[x++] = enc2CntTemp.cVal[1];
   buffer[x++] = enc2CntTemp.cVal[0];

   controlFlags = 0;

   //transfer inspection control flags to the buffer
   if (onPipe) controlFlags = (controlFlags | ON_PIPE_CTRL);
   if (head1Down) controlFlags = (controlFlags | HEAD1_DOWN_CTRL);
   if (head2Down) controlFlags = (controlFlags | HEAD2_DOWN_CTRL);

   buffer[x++] = controlFlags;

   buffer[x++] = RdPortI(PEDR);

   //send the buffer to host
   sock_flushnext(socket);
   sock_write(socket, buffer, pktSize);

}//end of sendInspectPacket
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// processInspect
//
// Handle tasks for inspect mode.
//
// Monitor inputs and encoder counts -- send status packets to the host as
// necessary.
//

void processInspect(tcp_Socket *socket)
{

   // retrieve the encoder count values
   encAccessFlag = 1; // block the ISR from modifying
   enc1CntTemp.lVal = encoder1Count; //snag the values to local variables
   enc2CntTemp.lVal = encoder2Count;
   encAccessFlag = 0; // unblock the ISR

   //if the specified trigger encoder has moved the trigger distance, send a
   //packet to the host

   //wip mks - host needs to specify which encoder to use, add choice here

   if (forceSendInspectPacket ||
        (labs(enc2CntTemp.lVal - prevEnc2Cnt) >= encoder2Delta)){

      forceSendInspectPacket = FALSE;

      //save counts for future comparisons
      prevEnc1Cnt = enc1CntTemp.lVal; prevEnc2Cnt = enc2CntTemp.lVal;

      sendInspectPacket(socket);

   }
   else
   	return;

}//end of processInspect
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// getInspectPacket
//
// Forces an Inspect data packet to be sent even if input triggers have not
// occurred.
//

int getInspectPacket(tcp_Socket *socket, int pPktID)
{

   int x, result;
   char buffer[2];

   //read in the remainder of the packet
   result = readBytesAndVerify(socket, buffer, 2, pPktID);
   if (result < 0) return(result);

   // retrieve the encoder count values
   encAccessFlag = 1; // block the ISR from modifying
   enc1CntTemp.lVal = encoder1Count; //snag the values to local variables
   enc2CntTemp.lVal = encoder2Count;
   encAccessFlag = 0; // unblock the ISR

   //send the packet
   sendInspectPacket(socket);

}//end of getInspectPacket
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// getMonitorPacket
//
// Triggers a Monitor info packet to be sent to host even if no values have
// been changed.
//

int getMonitorPacket(tcp_Socket *socket, int pPktID)
{

   int x, result;
   char buffer[2];

   //read in the remainder of the packet
   result = readBytesAndVerify(socket, buffer, 2, pPktID);
   if (result < 0) return(result);

   //set flag to trigger send
   sendMonitorPacket = TRUE;

}//end of getMonitorPacket
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// parseInspectionCtrl
//
// Parses the meaning of the state of the Inspection Control signal based
// on its current state and the time it was in its last state.
//
// Only call this function when the signal has just changed state.
//
// The current deglitched value of the Inspection Control Signal should be
// passed in via pInspectionControlNow so no time is wasted in deglitching
// the input again in this function.
//
// NOTE: When PLC sends a high signal, it is read as low by the Rabbit.
//

void parseInspectionCtrl(int pInspectionCtrlNow)
{
	//this function is called only after the Inspection Control input has
   //changed

	//if the signal went low but onPipe is false, then must have just
   //come on to the pipe
	if(pInspectionCtrlNow == 0 && onPipe == FALSE){
   	onPipe = TRUE;
      inspectionStatus=2;
		encoderPosAtOnPipeSignal = enc2CntTemp.lVal;
      return;
	}

   //if the signal went low, already on pipe (off pipe caught above), and
   //time duration of previous state is near HEAD1_SIGNAL_WIDTH, then
   //head 1 was raised or lowered and a high pulse was received from PLC
	if((pInspectionCtrlNow == 0)
   		&& (inspectionCtrlTime > (HEAD1_SIGNAL_WIDTH-2))
         && (inspectionCtrlTime < (HEAD1_SIGNAL_WIDTH+2))){

		//switch state of head 1
		if (head1Down == FALSE) {
      	head1Down = TRUE;
      	inspectionStatus=3;
			encoderPosAtHead1DownSignal = enc2CntTemp.lVal;
      } else {
      	head1Down = FALSE;
      	inspectionStatus=5;
			encoderPosAtHead1UpSignal = enc2CntTemp.lVal;
      }
      return;
	}

   //if the signal went low, already on pipe (off pipe caught above), and
   //time duration of previous state is near HEAD2_SIGNAL_WIDTH, then
   //head 2 was raised or lowered and a high pulse was received from PLC
	if((pInspectionCtrlNow == 0)
   		&& (inspectionCtrlTime > (HEAD2_SIGNAL_WIDTH-2))
         && (inspectionCtrlTime < (HEAD2_SIGNAL_WIDTH+2))){

		//switch state of head 2
		if (head2Down == FALSE){
      	head2Down = TRUE;
         inspectionStatus=4;
			encoderPosAtHead2DownSignal = enc2CntTemp.lVal;
      } else {
      	head2Down = FALSE;
			inspectionStatus=6;
			encoderPosAtHead2UpSignal = enc2CntTemp.lVal;
         }
      return;
	}

}//end of parseInspectionCtrl
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// deglitchSignal
//
// Reads the input port pPort bit pBit in a manner which protects against false
// triggering due to noise or switch bounce.
//
// Checks the input pCheckCount number of times and counts how many times
// it is 1.  If it was 1 for more than 50% of the number of times it
// was checked, then function returns 1.  Returns 0 otherwise.
//
// This function would be better if it ran in parallel to the main code
// rather than stopping execution during the check!!
//

int deglitchSignal(int pPort, int pBit, int pCheckCount)
{

   int i, high = 0;

   for (i=0; i<pCheckCount; i++){
      if (BitRdPortI(pPort, pBit) == 1) high++;
   }

   if (high > pCheckCount / 2)
      return 1;
   else
      return 0;

}//end of deglitchSignal
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// setToOffPipeState
//
// Sets all variables to the "off pipe" state.  It is assumed that all heads
// are up when off the pipe.
//

int setToOffPipeState()
{

	onPipe = FALSE;
	head1Down = FALSE;
	head2Down = FALSE;

	inspectionStatus = 1;

   //store the current linear encoder position
   encoderPosAtOffPipeSignal = enc2CntTemp.lVal;

   //force send an inspection packet in case the encoder does not turn
   //enough afterwards to send the final packet to the host
   forceSendInspectPacket = TRUE;

}//end of setToOffPipeState
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// resetTrack
//
// Sends a track counter reset pulse to the DSPs.
//
// NOTE: If the host issues a reset command, this function is called. It is
// best if the TRACK_PULSES_ENABLED has been cleared before the call. The
// reset will still pulse the track lines even if the flag is cleared, but the
// function which pulses the sync line at each clock period will be disabled
// and won't interfere.
//

void resetTrack(long pCountBTemp)
{

   //deactivate the track sync pulse output in case it happens to
   //be currently in progress -- to reset the counter, the reset line
   //has to be low during a hi/low trigger transition, so the trigger
   //needs to be high before the reset pulse/trigger are fired together
   BitWrPortI(PCDR, &PCDRShadow, 1, TRACK_SYNC); //PC0
   trackSyncPulseActive = FALSE;

	//make sure period tracking code doesn't fire the sync pulse during reset
	trackSyncPulseInactiveCount = LONG_MAX;

   //start pulse of track sync reset signal to inspection boards
   BitWrPortI(PCDR, &PCDRShadow, 0, TRACK_SYNC_RESET); //PC2
   trackSyncResetPulseActive = TRUE;
   //set width of pulse
   trackSyncResetPulseWidthCount = pCountBTemp + TRACK_RESET_PULSE_WIDTH;

   //fire a trigger sync pulse halfway through the reset pulse
   //reset must be low during a hi/low sync pulse to be clocked in
   trackSyncPulseStartDelayCount = pCountBTemp + TRACK_RESET_PULSE_HALF_WIDTH;

}//end of resetTrack
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// resetTrackOnTDC
//
// Sends a track counter reset pulse to the DSPs each time the TDC sensor
// is triggered.
//
// Calculates the number of timer counts for each clock period based on the
// clock count recorded for the last TDC period.
//
// Counts calls between changes to the input.
// Computes RPM each time the TDC goes active.
//
// This function should be called approximately every third time the Timer B ISR
// increments timerBCount.
//

void resetTrackOnTDC(long pCountBTemp)
{

long allowedRPMVariance;
int tdcDeglitched;

/*
//debug mks -- simulates the photo eye
if (trackSyncResetPulseInactiveCount-- == 0){

	//syncPulseInactiveCount = 90; //value of 90 is 10 mS

	//syncPulseInactiveCount = 90; //value of 90 is 9 mS

   //trackSyncResetPulseInactiveCount = 130000; //10 seconds

   trackSyncResetPulseInactiveCount = 13000; //1 seconds

   tdcInput = 0;

	}
//debug mks end
*/
	//get the current, deglitched state of the TDC input
	tdcDeglitched = deglitchSignal(PEDR, TDC, TDC_DEGLITCH_COUNT);

   //check if TDC input went from "off reflector" to "on reflector"

   //if last read showed "off reflector" and now on it, process TDC trigger
   if (tdcOffReflector){

   	//TDC is off the reflector, check to see if it just came on
      //(input bit set to 0 when eye receives reflection)

    	if (!tdcDeglitched){

	      //now on reflector
   	   tdcOffReflector = FALSE;

         //store the current rpm so it can be compared with the next value
         prevRPM = rpm;
			//get number of timer B counts since last off/on transition
			tdcPeriodCount = pCountBTemp - prevTDCCount;
			//starting point for calculating the next period
			prevTDCCount = pCountBTemp;

         //handle missed reflector hits or extra noise hit:
         //compare the period time with the previous value -- if greater than
         //10% difference, then ignore the new value and use a value 5%
         //more or less than the old value depending on whether the new value
         //is smaller or larger than the old value -- thus a missed hit will
         //usually be 100% different and will be corrected, but if the rotation
         //is slowing down or speeding up intentionally, the period will be
         //able to track it at 5% change for each reflector hit
         //If a correction is made more than 10 times in a row, then no
         //correction will be made -- this allows the value to catch up with
         //the actual RPM in cases where it is ramping up or down quickly.

         allowedRPMVariance = (long)(prevTDCPeriodCount * 0.1);

         if (rpmCorrectionCount < 10 &&
         		tdcPeriodCount > (prevTDCPeriodCount + allowedRPMVariance)){
	         rpmCorrectionCount++; //count number of times corrected
            tdcPeriodCount =
            	prevTDCPeriodCount + (long)(prevTDCPeriodCount * 0.05);
         }
			else
         if (rpmCorrectionCount < 10 &&
         		tdcPeriodCount < (prevTDCPeriodCount - allowedRPMVariance)){
	         rpmCorrectionCount++; //count number of times corrected
         	tdcPeriodCount =
	            prevTDCPeriodCount - (long)(prevTDCPeriodCount * 0.05);
         }
         else{
         	rpmCorrectionCount = 0; //not corrected this time, so start over
         }

         prevTDCPeriodCount = tdcPeriodCount;

      	//calculate RPM
         if (tdcPeriodCount > 0)
      		rpm = (int)((1.0 / (tdcPeriodCount * 0.0002857)) * 60.0);
			else
         	rpm = 0;

         rpmVariance = rpm - prevRPM;

         //based on the count for one revolution, calculate the counts
         //expected for a single clock position and use this to drive
         //the track sync pulse during the next revolution
         clockPeriodCount = tdcPeriodCount / maxClockPositions;

			//send Track counter reset pulse to all DSP boards
         if((controlFlags & TRACK_PULSES_ENABLED) != 0){
	         resetTrack(pCountBTemp);
         }

      }//if (!tdcDeglitched)

   }//if (tdcOffReflector)
   else{
   	//TDC is on the reflector, check to see if it just went off
    	if (tdcDeglitched){
      	tdcOffReflector = TRUE;
      }//if (tdcDeglitched)

   }//else of if (tdcOffReflector)

//tdcInput = 1; //debug mks -- part of the TDC simulation (see top of function)

}//end of resetTrackOnTDC
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// incrementTrackOnTDC
//
// Processes input data from the TDC sensor. Used for wall mapping mode.
//
// Sends a track increment pulse to the DSPs each time the TDC sensor is
// triggered.
//
// Counts calls between changes to the input.
// Computes RPM each time the TDC goes active.
//
// This function should be called approximately every third time the Timer B ISR
// increments timerBCount.
//

void incrementTrackOnTDC(long pCountBTemp)
{

long allowedRPMVariance;
int tdcDeglitched;

	//get the current, deglitched state of the TDC input
	tdcDeglitched = deglitchSignal(PEDR, TDC, TDC_DEGLITCH_COUNT);

   //check if TDC input went from "off reflector" to "on reflector"

   //if last read showed "off reflector" and now on it, process TDC trigger
   if (tdcOffReflector){

   	//TDC is off the reflector, check to see if it just came on
      //(input bit set to 0 when eye receives reflection)

    	if (!tdcDeglitched){

	      //now on reflector
   	   tdcOffReflector = FALSE;

         //store the current rpm so it can be compared with the next value
         prevRPM = rpm;
			//get number of timer B counts since last off/on transition
			tdcPeriodCount = pCountBTemp - prevTDCCount;
			//starting point for calculating the next period
			prevTDCCount = pCountBTemp;

         //handle missed reflector hits or extra noise hit:
         //compare the period time with the previous value -- if greater than
         //10% difference, then ignore the new value and use a value 5%
         //more or less than the old value depending on whether the new value
         //is smaller or larger than the old value -- thus a missed hit will
         //usually be 100% different and will be corrected, but if the rotation
         //is slowing down or speeding up intentionally, the period will be
         //able to track it at 5% change for each reflector hit
         //If a correction is made more than 10 times in a row, then no
         //correction will be made -- this allows the value to catch up with
         //the actual RPM in cases where it is ramping up or down quickly.

         allowedRPMVariance = (long)(prevTDCPeriodCount * 0.1);

         if (rpmCorrectionCount < 10 &&
         		tdcPeriodCount > (prevTDCPeriodCount + allowedRPMVariance)){
	         rpmCorrectionCount++; //count number of times corrected
            tdcPeriodCount =
            	prevTDCPeriodCount + (long)(prevTDCPeriodCount * 0.05);
         }
			else
         if (rpmCorrectionCount < 10 &&
         		tdcPeriodCount < (prevTDCPeriodCount - allowedRPMVariance)){
	         rpmCorrectionCount++; //count number of times corrected
         	tdcPeriodCount =
	            prevTDCPeriodCount - (long)(prevTDCPeriodCount * 0.05);
         }
         else{
         	rpmCorrectionCount = 0; //not corrected this time, so start over
         }

         prevTDCPeriodCount = tdcPeriodCount;

      	//calculate RPM
         if (tdcPeriodCount > 0)
      		rpm = (int)((1.0 / (tdcPeriodCount * 0.0002857)) * 60.0);
			else
         	rpm = 0;

         rpmVariance = rpm - prevRPM;

			//active the track sync (increment pulse)
         if((controlFlags & TRACK_PULSES_ENABLED) != 0){
         	trackSyncPulseActive = TRUE;
	         trackSyncPulseWidthCount = pCountBTemp + TRACK_SYNC_PULSE_WIDTH;
   	      BitWrPortI(PCDR, &PCDRShadow, 0, TRACK_SYNC); //PC0 -- activate
         }

      }//if (!tdcDeglitched)

   }//if (tdcOffReflector)
   else{
   	//TDC is on the reflector, check to see if it just went off
    	if (tdcDeglitched){
      	tdcOffReflector = TRUE;
      }//if (tdcDeglitched)

   }//else of if (tdcOffReflector)

}//end of incrementTrackOnTDC
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// processTDC
//
// Processes input data from the TDC sensor.
// Counts calls between changes to the input.
//
// Calls different functions to handle the TDC as configured by the host
//
// This function should be called approximately every third time the Timer B ISR
// increments timerBCount.
//

void processTDC(long pCountBTemp)
{

	if((controlFlags & RABBIT_SEND_CLOCK_MARKERS) != 0){
		resetTrackOnTDC(pCountBTemp);
      }
	else
   if((controlFlags & RABBIT_SEND_TDC) != 0){
   	incrementTrackOnTDC(pCountBTemp);
      }

}//end of processTDC
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// handleEndingOfPulses
//
// Turns off the track increment and the track reset pulses after the
// appropriate time.
//
// Used for wall mapping.
//

void handleEndingOfPulses(long pCountBTemp)
{

   //if the sync reset pulse is active, count down until zero and deactivate
	if ((trackSyncResetPulseActive == TRUE) &&
	   							(pCountBTemp > trackSyncResetPulseWidthCount)){
	   trackSyncResetPulseActive = FALSE;
		BitWrPortI(PCDR, &PCDRShadow, 1, TRACK_SYNC_RESET); //PC2 -- deactivate

	}

   //if the track sync pulse is active, count down until zero and deactivate
	if ((trackSyncPulseActive == TRUE) &&
	   										(pCountBTemp > trackSyncPulseWidthCount)){
	   trackSyncPulseActive = FALSE;
   	BitWrPortI(PCDR, &PCDRShadow, 1, TRACK_SYNC); //PC0 -- deactivate

	}

}//end of handleEndingOfPulses
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// startTrackPulseEachPeriod
//
// Turns on the track increment pulse for each period.
//
// Other functions set up the period to trigger a pulse for each clock position
// or can force a pulse to occur at a specific future time by setting
// trackSyncPulseInactiveCount to the number of timer counts to delay. This
// method is used to fire a pulse a while after the Track Reset line has been
// driven low to clock the Reset pulse into the timers.
//

void startTrackPulseEachPeriod(long pCountBTemp)
{
   //the clock period count is the period between pulse starts
   //when it times out, activate the sync pulse

	if ((controlFlags & TRACK_PULSES_ENABLED) &&
   								(pCountBTemp > trackSyncPulseInactiveCount)){

      //set up to fire next period
		trackSyncPulseInactiveCount = pCountBTemp + clockPeriodCount;

      //set width of pulse and activate
		trackSyncPulseActive = TRUE;
   	trackSyncPulseWidthCount = pCountBTemp + TRACK_SYNC_PULSE_WIDTH;
   	BitWrPortI(PCDR, &PCDRShadow, 0, TRACK_SYNC); //PC0 -- activate

	}

}//end of startTrackPulseEachPeriod
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// startTrackPulseAfterDelay
//
// Turns on the track increment pulse after a delay specified by the value
// in trackSyncPulseInactiveCount. The pulse is not automatically repeated.
//
// Other functions can force a pulse to occur at a specific future time by
// setting trackSyncPulseInactiveCount to the number of timer counts to delay.
//
// NOTE: This method is mainly used to fire a pulse delayed after the Track
// Reset line has been driven low to clock the Reset pulse into the timers.
//

void startTrackPulseAfterDelay(long pCountBTemp)
{

   //when delay times out, activate the sync pulse
	if (trackSyncPulseStartDelayCount != -1
		   && pCountBTemp > trackSyncPulseStartDelayCount){

      //set up to fire next period if enabled
		trackSyncPulseInactiveCount = pCountBTemp + clockPeriodCount;

   	//prevent pulse from automatically firing again
		trackSyncPulseStartDelayCount = -1;
		trackSyncPulseActive = TRUE;
   	trackSyncPulseWidthCount = pCountBTemp + TRACK_SYNC_PULSE_WIDTH;
   	BitWrPortI(PCDR, &PCDRShadow, 0, TRACK_SYNC); //PC0 -- activate

	}

}//end of startTrackPulseAfterDelay
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// handleTrackSyncAndResetLines
//
// Initiates and turns off the DSP track counter increment and reset pulses.
//

void handleTrackSyncAndResetLines(long pCountBTemp)
{

	if((controlFlags & RABBIT_SEND_CLOCK_MARKERS) != 0){
		startTrackPulseEachPeriod(pCountBTemp);
      }
	else
   if((controlFlags & RABBIT_SEND_TDC) != 0){

      }

	//if Host has requested a Track Sync Reset, invoke one here

   if(resetPulseTrack){
   	resetPulseTrack = FALSE;
		//send Track counter reset pulse to all DSP boards
      resetTrack(pCountBTemp);
   }

   //handle sync pulses required for reset clocking -- sync pulses for
   //clock periods are handled elsewhere
	startTrackPulseAfterDelay(pCountBTemp);

   //end pulses at specified times
   handleEndingOfPulses(pCountBTemp);

}//end of handleTrackSyncAndResetLines
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// countTimerBInts
//
// Tracks the timer B interrupt counter and increments mSecCounter approximately
// every .01 seconds.
//
// Also handles turning off output pulses when they have timed out.
//
// Note that countBTemp is never reset, but at timer B rate of 3.5Khz, it will
// not roll over for 340 hours.
//

void countTimerBInts()
{

	long countBTemp;
   int inspectionCtrlNow;

	// retrieve the timer B counter
	countBAccessFlag = 1; // block the ISR from modifying
	countBTemp = timerBCount; //snag the interrupt counter to local variable
	countBAccessFlag = 0; // unblock the ISR

	processTDC(countBTemp);

   handleTrackSyncAndResetLines(countBTemp);

	//increment the tick counter approximately every .01 seconds
	if (countBTemp - prevCountBTemp >= 35){
   	prevCountBTemp = countBTemp;
      tickCount++;
   }
	else return; //do nothing if counter did not time out

	//this part executed if the tick count was updated - handles various timed
	//operations

	if (output1Timer != 0){
		//count down and turn off output when timed out
   	if (--output1Timer == 0){
			//set output to 1 which turns off the optoisolator
			BitWrPortI(PCDR, &PCDRShadow, 1, OUTPUT1);
			//flag so other functions can know state
			output1State = FALSE;
		}
	} //if (output1Timer != 0)

	if (output2Timer != 0){
		//count down and turn off output when timed out
   	if (--output2Timer == 0){
			//set output to 1 which turns off the optoisolator
			BitWrPortI(PCDR, &PCDRShadow, 1, OUTPUT2);
			//flag so other functions can know state
			output2State = FALSE;
		}
	} //if (output2Timer != 0)

	if (output3Timer != 0){
		//count down and turn off output when timed out
   	if (--output3Timer == 0){
			//set output to 1 which turns off the optoisolator
			BitWrPortI(PCDR, &PCDRShadow, 1, OUTPUT3);
			//flag so other functions can know state
			output3State = FALSE;
		}
	} //if (output3Timer != 0)

	if (output4Timer != 0){
		//count down and turn off output when timed out
   	if (--output4Timer == 0){
			//set output to 1 which turns off the optoisolator
			BitWrPortI(PCDR, &PCDRShadow, 1, OUTPUT4);
			//flag so other functions can know state
			output4State = FALSE;
		}
	} //if (output4Timer != 0)

   //get the deglitched value of the inspection control signal input
   inspectionCtrlNow =
		deglitchSignal(PADR, INSPECT, INSPECT_CONTROL_DEGLITCH_COUNT);

   //every time the Inspection Control input changes state, store the time
   //it was in the last state and start timing the new state
	if(inspectionCtrl != inspectionCtrlNow){
	   inspectionCtrl = inspectionCtrlNow;
   	inspectionCtrlTime = tickCount - inspectionCtrlTimer;
		inspectionCtrlTimer = tickCount;
		//call to parse the meaning of the state of the Inspection Control signal
		parseInspectionCtrl(inspectionCtrlNow);
	}else
   //if state hasn't changed, then watch for extended high state -- means
   //head is off pipe
   if( (inspectionCtrlNow == 1) &&
   		((tickCount - inspectionCtrlTimer) > OFF_PIPE_SIGNAL_WIDTH) ){

         setToOffPipeState(); //reset all variables to "off pipe" state
         inspectionCtrl = inspectionCtrlNow;
       	inspectionCtrlTimer = tickCount;
   }

}//end of countTimerBInts
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// getChassisSlotAddress
//
// Returns the board's chassis and slot address in a byte.  Returns the
// status byte as well.
//

int getChassisSlotAddress(tcp_Socket *socket, int pPktID)
{

   int result;
	char	buffer[2];

	//read in the remainder of the packet
	result = readBytesAndVerify(socket, buffer, 1, pPktID);
	if (result < 0) return(result);

	buffer[0] = (byte)(((byte)chassisAddr<<4 & 0xf0) + ((byte)slotAddr & 0xf));
	buffer[1] = status;

	sendPacketHeader(socket, GET_CHASSIS_SLOT_ADDRESS_CMD);
	sock_flushnext(socket);
	sock_write(socket,buffer,2);

}//end of getChassisSlotAddress
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// startMonitor
//
// Starts the monitor mode.
//

int startMonitor(tcp_Socket *socket, int pPktID)
{

   int x, result;
   char buffer[2];

   //read in the remainder of the packet
   result = readBytesAndVerify(socket, buffer, 2, pPktID);
   if (result < 0) return(result);

   // PLC sends a high to drive the opto isolator - inverted to low to Rabbit
   //
   // PLC sends a high during Inspection - Rabbit reads low.
   // PLC sends a high when Carriage on Pipe - Rabbit reads low.
   // PLC sends a high when TDC marker is at TDC - Rabbit reads low.

   //initialize previous state of inputs to the current states

   prevEnc1A = BitRdPortI(PADR, ENC1A);
   prevEnc1B = BitRdPortI(PADR, ENC1B);
   prevEnc2A = BitRdPortI(PADR, ENC2A);
   prevEnc2B = BitRdPortI(PADR, ENC2B);
   prevUnused1 = BitRdPortI(PADR, UNUSED1);
   prevUnused2 = BitRdPortI(PADR, UNUSED2);
   prevInspect = BitRdPortI(PADR, INSPECT);
   prevUnused3 = BitRdPortI(PADR, UNUSED3);
   prevTDC = BitRdPortI(PEDR, TDC);
   prevUnused4 = BitRdPortI(PEDR, UNUSED4);
   prevSlotAddr = ~(RdPortI(PBDR) | 0xfff0);
   prevChassisAddr = ~((RdPortI(PBDR) >> 4) | 0xfff0 );

   sendMonitorPacket = TRUE; //force update first time through

   printf("Monitor mode started...\n");

   monitorMode = TRUE;  //inititate calling of the monitor function

}//end of startMonitor
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// stopMonitor
//
// Stops the monitor mode.
//

int stopMonitor(tcp_Socket *socket, int pPktID)
{

   int x, result;
   char buffer[2];

   //read in the remainder of the packet
   result = readBytesAndVerify(socket, buffer, 2, pPktID);
   if (result < 0) return(result);

   printf("Monitor mode stopped...\n");

   monitorMode = FALSE;  //stop calling of the monitor function

}//end of stopMonitor
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// processMonitor
//
// Sends to host the status of all inputs when any input changes.
//

int processMonitor(tcp_Socket *socket)
{

   int bytes_read;
   int pktSize = 25;
   char  buffer[25];
   int x = 0;

   // PLC sends a high to drive the opto isolator - inverted to low to Rabbit
   //
   // PLC sends a high during Inspection - Rabbit reads low.
   // PLC sends a high when Carriage on Pipe - Rabbit reads low.
   // PLC sends a high when TDC marker is at TDC - Rabbit reads low.

   //check all inputs, send new status if changed from previous state

   //this part makes sure a quick transition gets transmitted to the host
   //even if the host is requesting periodic updates which might miss the
   //the event

   //don't send packet for every change in encoder inputs as this would
   //be too much data - if the encoders are turned slowly enough, the events
   //can be seen by the host when requesting periodic updates

   if (BitRdPortI(PADR, ENC1A) != prevEnc1A){
      prevEnc1A = BitRdPortI(PADR, ENC1A);
      //sendMonitorPacket = TRUE; don't transmit on change - see note above
   }

   if (BitRdPortI(PADR, ENC1B) != prevEnc1B){
      prevEnc1B = BitRdPortI(PADR, ENC1B);
      //sendMonitorPacket = TRUE; don't transmit on change - see note above
   }

   if (BitRdPortI(PADR, ENC2A) != prevEnc2A){
      prevEnc2A = BitRdPortI(PADR, ENC2A);
      //sendMonitorPacket = TRUE; don't transmit on change - see note above
   }

   if (BitRdPortI(PADR, ENC2B) != prevEnc2B){
      prevEnc2B = BitRdPortI(PADR, ENC2B);
      //sendMonitorPacket = TRUE; don't transmit on change - see note above
   }

   if (BitRdPortI(PADR, UNUSED1) != prevUnused1){
      prevUnused1 = BitRdPortI(PADR, UNUSED1);
      sendMonitorPacket = TRUE;
   }

   if (BitRdPortI(PADR, UNUSED2) != prevUnused2){
      prevUnused2 = BitRdPortI(PADR, UNUSED2);
      sendMonitorPacket = TRUE;
   }

   if (BitRdPortI(PADR, INSPECT) != prevInspect){
      prevInspect = BitRdPortI(PADR, INSPECT);
      sendMonitorPacket = TRUE;
   }

   if (BitRdPortI(PADR, UNUSED3) != prevUnused3){
      prevUnused3 = BitRdPortI(PADR, UNUSED3);
      sendMonitorPacket = TRUE;
   }

   if (BitRdPortI(PEDR, UNUSED4) != prevUnused4){
      prevUnused4 = BitRdPortI(PBDR, UNUSED4);
      sendMonitorPacket = TRUE;
   }

   //read the board's chassis and slot number from PortB which is connected to
   //the rotary switches on the motherboard

   // slot number is lower nibble of Port B inputs inverted
   slotAddr = ~(RdPortI(PBDR) | 0xfff0);

   if (slotAddr != prevSlotAddr){
      prevSlotAddr = slotAddr;
      sendMonitorPacket = TRUE;
   }

   // chassis number is upper nibble of Port B inputs inverted
   chassisAddr = ~((RdPortI(PBDR) >> 4) | 0xfff0 );

   if (chassisAddr != prevChassisAddr){
      chassisAddr = chassisAddr;
      sendMonitorPacket = TRUE;
   }

   if (sendMonitorPacket == TRUE){

      x = 0;

      if (BitRdPortI(PADR, ENC1A)) buffer[x++] = 0;
      else buffer[x++] = 1;

      if (BitRdPortI(PADR, ENC1B)) buffer[x++] = 0;
      else  buffer[x++] = 1;

      if (BitRdPortI(PADR, ENC2A)) buffer[x++] = 0;
      else  buffer[x++] = 1;

      if (BitRdPortI(PADR, ENC2B)) buffer[x++] = 0;
      else  buffer[x++] = 1;

      if (BitRdPortI(PADR, UNUSED1)) buffer[x++] = 0;
      else  buffer[x++] = 1;

      if (BitRdPortI(PADR, UNUSED2)) buffer[x++] = 0;
      else  buffer[x++] = 1;

      if (BitRdPortI(PADR, INSPECT)) buffer[x++] = 0;
      else  buffer[x++] = 1;

      if (BitRdPortI(PADR, UNUSED3)) buffer[x++] = 0;
      else  buffer[x++] = 1;

      if (BitRdPortI(PEDR, TDC)) buffer[x++] = 0;
      else  buffer[x++] = 1;

      if (BitRdPortI(PEDR, UNUSED4)) buffer[x++] = 0;
      else  buffer[x++] = 1;

      buffer[x++] = chassisAddr;

      buffer[x++] = slotAddr;

      buffer[x++] = inspectionStatus;

      buffer[x++] = (rpm >> 8) & 0xff;
      buffer[x++] = rpm & 0xff;

      buffer[x++] = (rpmVariance >> 8) & 0xff;
      buffer[x++] = rpmVariance & 0xff;

      // retrieve the encoder count values
      encAccessFlag = 1; // block the ISR from modifying
      enc1CntTemp.lVal = encoder1Count; //snag the values to local variables
      enc2CntTemp.lVal = encoder2Count;
      encAccessFlag = 0; // unblock the ISR

      //place the encoder 1 values into the buffer by byte, MSB first
      buffer[x++] = enc1CntTemp.cVal[3];
      buffer[x++] = enc1CntTemp.cVal[2];
      buffer[x++] = enc1CntTemp.cVal[1];
      buffer[x++] = enc1CntTemp.cVal[0];

      //place the encoder 2 values into the buffer by byte, MSB first
      buffer[x++] = enc2CntTemp.cVal[3];
      buffer[x++] = enc2CntTemp.cVal[2];
      buffer[x++] = enc2CntTemp.cVal[1];
      buffer[x++] = enc2CntTemp.cVal[0];

      //send the buffer to host
      sendPacketHeader(socket, GET_MONITOR_PACKET_CMD);
      sock_flushnext(socket);
      sock_write(socket,buffer,x);

   }// if (inputChanged == 1)

   sendMonitorPacket = FALSE; //don't update again until change detected

   return(0);

}//end of processMonitor
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// setControlFlags
//
// Sets the unsigned int controlFlags variable to the value of the next two
// bytes in pSocket, in MSB/LSB order.
//
// The controlFlags are set by the host to control the functionality of the
// Rabbit module.
//
// On success, returns the number of bytes read from the packet.
// On checksum error for received packet, returns -1.
// If received packet too small, returns -1.
//

int setControlFlags(tcp_Socket *pSocket, int pPktID)
{

   char buffer[3];
   int result;
   int x = 0;

   //read in the remainder of the packet
   result = readBytesAndVerify(pSocket, buffer, 3, pPktID);
   if (result < 0) return(result);

	controlFlags = (unsigned)((buffer[x++]<<8) & 0xff00)
                                  + (unsigned)(buffer[x++] & 0xff);

   return(3); //return number of bytes read from socket

}//end of setControlFlags
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// resetTrackCounters
//
// Resets all Track counters in the DSPs by pulsing the Track Sync Reset line.
//
// This will be ignored in certain modes such as those which already reset the
// counters in response to other inputs.
//

int resetTrackCounters(tcp_Socket *socket, int pPktID)
{

   int x, result;
   char buffer[2];

   //read in the remainder of the packet
   result = readBytesAndVerify(socket, buffer, 2, pPktID);
   if (result < 0) return(result);

   //trigger other code to pulse the reset line
	resetPulseTrack = TRUE;

   return(2); //return number of bytes read from socket

}//end of resetTrackCounters
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// timerBISR
//
// This is the Interrupt Service Routine for Timer B.  This routine tracks
// the encoder inputs, incrementing or decrementing a counter for each encoder.
//

	//easier to use static varibles for the asm code
	static char prevEnc1, prevEnc2;

#asm root nodebug

//change above to "debug" to set breakpoints in the assembler code
//this adds extra code, so jr (jump relative) opcodes may have to be replaced
//with jp (jump absolute) opcodes

timerBISR::

	// good practice to save only the registers that will be used here

	push	af					; save registers
	push	bcde
	push	jkhl
	ioi	ld	a, (TBCSR)			; clear the interrupt

	// end of housekeeping; body of ISR starts here

	// if the access flag is 1, skip modifying counters as the non-ISR code
	// is accessing the values

   ld		a, (countBAccessFlag)
	bit 	0,a
	jr		NZ, skip1			; skip if flag is 1 (Zero flag = 0)

	// increment the timer B counter

	ld   	jkhl, (timerBCount)
	ld   	bcde, 1
	add  	jkhl,bcde
	ld	 	(timerBCount), jkhl

skip1:

	// if the access flag is 1, skip modifying counters as the non-ISR code
	// is accessing the values -- should not miss a count as the ISR should
   // be called often enough to get several hits off of each encoder change

   ld    a, (encAccessFlag)
	bit 	0,a
	jp		NZ, done			; skip if flag is 1 (Zero flag = 0)

/*
;debug mks -- remove this
	ld   	jkhl, (encoder1Count)
	ld   	bcde, 1
	add  	jkhl,bcde
	ld	 	(encoder1Count), jkhl
;debug mks -- end
*/

	// check for Encoder 1 movement

	ld 	a,(prevEnc1)	; get the previous encoder A & B values
	sla	a					; shift up two bits, zeroing lower two bits
	sla	a
   ld		b, a				; move old value to register b

	ioi ld a, (0x0030)   ; load Port A
	and	0x03				; only keep lower two bits (encoder A & B lines)

   or		b					; combine old A & B with new A & B
	ld    (prevEnc1), a	; store for next time

   and	0x0f				; keep only lower nibble with old and new A & B

   cp		0x02				; check for all new A & B / old A & B combinations
	jp		z, inc1			;   which indicate forward rotation
   cp		0x0b
	jp		z, inc1
   cp		0x0d
	jp		z, inc1
   cp		0x04
	jp		z, inc1

   cp		0x01				; check for all new A & B / old A & B combinations
	jp		z, dec1			;   which indicate reverse rotation
   cp		0x07
	jp		z, dec1
   cp		0x0e
	jp		z, dec1
   cp		0x08
	jp		z, dec1

   jp    doEnc2 			; all other combinations are illegal or indicate
   							; no movement so do not adjust count

inc1:	; increment the encoder count

	ld   	jkhl, (encoder1Count)
	ld   	bcde, 1
	add  	jkhl,bcde
	ld	 	(encoder1Count), jkhl
   jp		doEnc2

dec1:	; decrement the encoder count

	ld   	jkhl, (encoder1Count)
	ld   	bcde, 1
	sub  	jkhl,bcde
	ld	 	(encoder1Count), jkhl

doEnc2:
	// check for Encoder 2 movement

	ld 	a,(prevEnc2)	; get the previous encoder A & B values
	sla	a					; shift up two bits, zeroing lower two bits
	sla	a
   ld		b, a				; move old value to register b

	ioi ld a, (0x0030)   ; load Port A
   sra	a					; shift I/O bits 3 & 2 down to 1 & 0
   sra	a					;  (encoder 2 A & B input on pins 3 & 2)
	and	0x03				; only keep lower two bits (encoder A & B lines)

   or		b					; combine old A & B with new A & B
	ld    (prevEnc2), a	; store for next time

   and	0x0f				; keep only lower nibble with old and new A & B

   cp		0x02				; check for all new A & B / old A & B combinations
	jp		z, inc2			;   which indicate forward rotation
   cp		0x0b
	jp		z, inc2
   cp		0x0d
	jp		z, inc2
   cp		0x04
	jp		z, inc2

   cp		0x01				; check for all new A & B / old A & B combinations
	jp		z, dec2			;   which indicate reverse rotation
   cp		0x07
	jp		z, dec2
   cp		0x0e
	jp		z, dec2
   cp		0x08
	jp		z, dec2

   jp    done 				; all other combinations are illegal or indicate
   							; no movement so do not adjust count

inc2:	; increment the encoder count

	ld   	jkhl, (encoder2Count)
	ld   	bcde, 1
	add  	jkhl,bcde
	ld	 	(encoder2Count), jkhl
   jp		done

dec2:	; decrement the encoder count

	ld   	jkhl, (encoder2Count)
	ld   	bcde, 1
	sub  	jkhl,bcde
	ld	 	(encoder2Count), jkhl

	// body of ISR ends here

	// do some more housekeeping before exiting the ISR

done:

	//setup to load the match register

	// this program uses a simple match value of zero each time to give
	// an interrupt period of 1024 counts (10 bit register is 1024 max)
	// TBL1R must be updated to enable the interrupt - if TBM1R is to be
	// updated it must be done first

	ld a, 0
	ioi	ld (TBL1R), a

	pop	jkhl			; restore registers
	pop	bcde
	pop	af

	ipres				; restore interrupts
	ret

#endasm

//end of timerBISR
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// setupTimerBInt
//
// Prepares the Timer B interrupt which is used to track the encoder inputs.
//
// Dynamic C 10.46 sets GCSR so that CPU = OSC and PCLK = OSC.  Thus PCLK
// runs at 58 Mhz on the RCM4200.  The code below sets Timer B to run at
// PCLK/16.  The ISR uses a match value of 0 each time, which results in
// the counter triggering at each 1024 counts.  The resulting interrupt
// frequency:
//
// PCLK = 58 Mhz
// Interrupt frequency = 58Mhz / 16 / 1024 = 3,540 Hz = 3.5 Khz
//

void setupTimerBInt()
{

#if __SEPARATE_INST_DATA__
		interrupt_vector timerb_intvec timerBISR;
#else
	   // initialize Timer B interrupt vector
      SetVectIntern(INT_TIMERB, timerBISR);
#endif

   // initialize Timer B
   // clock timer B to trigger on PCLK/16 and set interrupt level to 2
   WrPortI(TBCR, &TBCRShadow, TBCR_CLOCK_PCLK16 | TBCR_INT_PRI_2);

   // set initial match - use zero for 1024 counts
   WrPortI(TBM1R, NULL, 0);
   WrPortI(TBL1R, NULL, 0);

   // enable timer B and Match1 interrupts
   WrPortI(TBCSR, &TBCSRShadow, TBCSR_INT_MATCH1 | TBCSR_MAIN_CLK_ENA);

}//end of setupTimerBInt
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// initRegisters
//
// Sets up all ports and registers.
//

void initRegisters()
{

   // bit  7   = 1   - Ignore the SMODE pins program fetch function
   // bits 6:5 = 00  - Read only, states of SMODE pins, write with 0
   // bits 4:2 = 000 - Disable auxiliary I/O bus, Port A and Port B are I/O
   // bits 1:0 = 00  - Disable slave port interrupts
   // note that this also sets Port A to inputs

   WrPortI(SPCR, &SPCRShadow, 0x80);

   //------------ Setup Port A ------------

   //all necessary setup performed by writing to SPCR above

   //------------ Setup Port B ------------

   // set all B ports as inputs
   WrPortI(PBDDR, &PBDDRShadow, 0x00);

   //------------ Setup Port C ------------

   // C Port 0 - UT Bank Sync Output
   // C Port 1 - External Output 1
   // C Port 2 - UT Bank Sync Reset Output
   // C Port 3 - External Output 2
   // C Port 4 - Track Sync Output
   // C Port 5 - External Output 3
   // C Port 6 - output, Serial I/O by Dynamic C (do not modify)
   // C Port 7 - External Output 4 (also used by Dynamic C, see below)
   //             (should be changed to C Port 4 on board ver 1.2 - this should
   //              have been done on ver 1.1 but was overlooked)
   //            The idea is to swap Bank Sync/Reset with Track Sync/Reset
   //             as the Control board will rarely if ever drive the Bank signals.
   //             Thus, PortC bits 6,7 could be left as serial debugger
   //             communication pins.

   // NOTE - 6 & 7 are used by the Dynamic C debugger - if they are changed
   //    the debugger will be disabled
   // C Port 6 - on init ~ output, Serial I/O by Dynamic C
   // C Port 7 - on init ~ input, Serial I/O by Dynamic C

   // See RabbitCore RCM4200 User's Manual, page 29 for initialization states.

   // NOTE NOTE
   // For all board versions:
   // to use the debugger, use masks to leave PC6,PC7 unmodified as used by
   // Dynamic C link to host
   // For board version 1.0, this means cannot use Output 4 and one of the syncs
   // For board version 1.1, this means cannot use Output 4

   // set all usable C ports to driven (not open drain)
   // use mask to avoid modifying P6,P7 which are used by debugger
   WrPortI(PCDCR, &PCDCRShadow, (PCDCRShadow & 0xc0));

   // set all usable C ports function as I/O
   // use mask to avoid modifying P6,P7 which are used by debugger
   WrPortI(PCFR, &PCFRShadow, (PCFRShadow & 0xC0));

   // set bits P0-P5 to outputs
   // use mask to avoid modifying P6,P7 which are used by debugger
   WrPortI(PCDDR, &PCDDRShadow, (PCDDRShadow & 0xC0) | 0x3f);

   //set all Port C outputs to 1, which turns off the optoisolator - inactive

   BitWrPortI(PCDR, &PCDRShadow, 1, OUTPUT1); //PC1
   BitWrPortI(PCDR, &PCDRShadow, 1, OUTPUT2); //PC3
   BitWrPortI(PCDR, &PCDRShadow, 1, OUTPUT3); //PC5
   // don't set OUTPUT4 (PC7) or it will disrupt the debugger
   // put this back in when OUTPUT4 is changed to PC4 for board ver 1.2
   //BitWrPortI(PCDR, &PCDRShadow, 1, OUTPUT4); //PC7
   BitWrPortI(PCDR, &PCDRShadow, 1, TRACK_SYNC); //PC0
   BitWrPortI(PCDR, &PCDRShadow, 1, TRACK_SYNC_RESET); //PC2
   // don't set (PC6) or it will disrupt the debugger
   //BitWrPortI(PCDR, &PCDRShadow, 1, 6);

   //------------ Setup Port D ------------

   // Port D cannot be used as digital I/O as the default jumper settings
   // connect the board's I/O pins to be analog inputs rather than Port D I/O

   //------------ Setup Port E ------------

   // Note: The data sheet says that using a typical read-modify-write operation
   // for Ports D and E can lead to old data being written to the port due to
   // those ports being buffered.  Actually, all the ports are buffered -- the
   // data is transferred to the outputs on the next clock edge.  However, for
   // ports A, B, and C, this clock is always the peripheral clock which runs at
   // as slow as main clock / 8.
   //
   // On the other hand, Ports D & E may also be clocked by other timers which
   // might be much slower.  So if you read the port to modify a single bit and
   // then write it back, you could read old data if the data previously written
   // to the port buffer had not net been clocked to the output port -- the read
   // operation reads the actual port pins while the write operation writes to a
   // buffer which is not transferred until the next clock.
   //
   // For this reason, any single bit can be set for Ports D & E -- special one
   // bit ports have been provided for this purpose to avoid having to read the
   // old port data to modify the one bit.
   //
   // It would seem that this *could* also happen for ports A/B/C, but perhaps
   // it is more unlikely because the peripheral clock generally runs fairly
   // fast compared with the main clock and would be totally predictable so the
   // program could avoid another modify operation until the next clocking.
   // It would be a lot more difficult to determine this required delay for
   // ports D/E if they were being clocked by a much slower clock.
   //
   // The BitWrPortI command reads from a shadow register to get the current
   // value of the port buffer which will or already has been transferred to
   // the output pins, so this data is always fresh and no worries about
   // reading old data from the pins.
   //

   // Port E is buffered -- data is transferred out on the next clock edge
   // it is assumed that PECR has been set up by Dynamic C to transfer the
   // outputs by the peripheral clock

   // PEDDR -- set I/O direction for pin used by program -- others unchanged
   // PE0, PE1 : inputs, PE3, PE4 : outputs
   // PE2 is used for the Ethernet chip select -- DO NOT DISTURB --
   // use mask to avoid modifying ports not used by this program
   // DO NOT change PE7  config -- used to access serial Flash for updating
   // firmware.  If it is changed here, it must be changed back for code upload.

   WrPortI(PEDDR, &PEDDRShadow, (PEDDRShadow & 0xe4) | 0x18);

   // PEDCR -- set pins to PE3 & PE4 to driven as opposed to open drain
   // use mask to avoid modifying ports not used by this program

   WrPortI(PEDCR, &PEDCRShadow, (PEDCRShadow & 0xe4) | 0x00);

   // PEFR -- set pins to act as I/O and not alternate function
   WrPortI(PEFR, &PEFRShadow, (PEFRShadow & 0xe4) | 0x00);

   // inactivate the Pulse sync triggers

   BitWrPortI(PEDR, &PEDRShadow, 1, PULSE_SYNC);
   BitWrPortI(PEDR, &PEDRShadow, 1, PULSE_SYNC_RESET);

}//end of initRegisters
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// setupEthernet
//
// Prepares an ethernet socket for use as a UDP and a TCP/IP connection.
//

setupEthernet()
{

   char  buffer[100];
   char MACbuffer[7];
   longword IPaddr;

   //obtain a socket for use by TCPIP and UDP
   // Start network and wait for interface to come up (or error exit).
   sock_init_or_exit(1);

  	// To avoid having to manually register each device's IP in the host computer,
   // they use the MAC address to derive an IP address. The host then collects
   // these IP addresses using a UDP broadcast/response to all listening
   // devices.  Each board is supposed to have a unique MAC stored in it by the
   // manufacturer - use the bottom two bytes of the MAC as the bottom two bytes
   // of the IP which will make it very unlikely that any two boards in a system
   // have the same IP.

   pd_getaddress(0, MACbuffer);
   printf("Link Address: %02x%02x:%02x%02x:%02x%02x\n", MACbuffer[0],
         MACbuffer[1], MACbuffer[2], MACbuffer[3], MACbuffer[4], MACbuffer[5]);


   //set the IP address for this board
   //Use 169.254 for upper part to match the default for a Windows host which
   //cannot retrieve a dynamic IP (as is usually the case when connected to this
   //system) - this allows the Windows host to communicate with the system.
   //Use the two lower bytes of the board's MAC as the two lower bytes of the IP
   //to produce a (mostly) unique IP.

   IPaddr = ((longword)169<<24) + ((longword)254<<16)
                   + ((longword)MACbuffer[4]<<8) + ((longword)MACbuffer[5]);

   //set the IP address
   ifconfig(IF_DEFAULT,
            IFS_DOWN,
            IFS_IPADDR, IPaddr,
            IFS_UP,
            IFS_END);

   //retrieve the host ID into a string
   //because the netmask is set to 255.255.0.0, this function always returns
   //255.255 for the top two bytes
   inet_ntoa(buffer, gethostid());

   //display the IP address
   printf("Changing local IP address to: %s\n", buffer);

}//end of setupEthernet
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// waitForHostViaUDP
//
// Opens a UDP socket and waits for the broadcast message from the host.  The
// board's IP address is then transmitted back to the host.
//
// The board uses the lower two bytes of the MAC number as the lower two bytes
// of the IP address.  This should have a very low chance of resulting in
// duplicate IP addresses in a system.  The first version of the Control board
// had issues reading the chassis and slot address switches and the UT boards
// cannot read those switches until the FPGA is loaded.  Thus, the boards
// cannot use those switches to derive a unique IP address.
//
// To solve both these problems, the host PC broadcasts a roll call message to
// which the boards respond by sending their IP addresses (derived from the
// MAC).  The host can then make a connection to send the FPGA code to UT
// boards, after which the chassis and board number switches can be read.
// The host can then request these numbers in order to know which board is
// in which slot and can then tie each board to its assigned channels.
//
// For version 1.0 Control boards, which must be modified extensively to
// read the address switches and is therefore not done, the host PC can assume
// that there is only one Control board in the system and the one which responds
// to the roll call broadcast is that board.  Future systems which use multiple
// Control boards will have to use version 1.1 boards which can read their
// chassis and board addresses.  Even in that case, the boards will still be
// contacted first via UPD roll call broadcast to maintain compatibility.
// (the comment section above is mirrored in Capulin UT Board.c)
//

void waitForHostViaUDP()
{

#define SIZE_OF_UDP_BUFFER 128

   _udp_datagram_info datagramInfo;

   int status;
   int length;
   char buffer[SIZE_OF_UDP_BUFFER];
   int timeOut;

   udp_Socket socket;

   printf("Waiting for roll call from host...\n");

   //use local port 4446, use 230.0.0.1 for remote IP - this is a reserved IP
   //for multicasting, host broadcasts to port 4446 from port 4445
   //using a multicast IP automatically tells udp_open to join that IP group

   status = udp_open(&socket, 4446, resolve("230.0.0.1"), 4445, NULL);

   printf("Socket status : %d\n", status);

   //listen for roll call packets

   status = 0;

   while(status != 1){

      tcp_tick(&socket); //process TCP packets

      //peek at the packet instead of reading it so we can get IP info
      status = udp_peek(&socket, &datagramInfo );

      if ((status == 1)
         &&
         ((length = udp_recv(&socket, buffer, SIZE_OF_UDP_BUFFER)) != -1)){

           //display the message received via UDP
           buffer[length] = '\0'; //terminate the string
         printf("Received: %s\n", buffer);

           //if the roll call is not meant for this board, keep waiting
           if(_n_strcmp(ROLL_CALL_QUERY, buffer) != 0) status = 0;

         //display the sender's IP address in decimal format
         inet_ntoa(buffer, datagramInfo.remip);
         printf("Roll call received from %s\n", buffer);

      }
      else status = 0; //if packet not received, keep waiting

   }//while(status != 1)

   //send a return packet so the host can know the board's IP
   printf("Sending response...\n");
   sprintf(buffer, "%s, Ver %s...", ROLL_CALL_RESPONSE, VERSION);


   //When the socket is first opened, it is not tied to the host IP because it
   //is unknown at that time.  The documentation seems to imply that after
   // receiving a packet from an IP, that IP will be resolved to a hardware
   // address.  Sending back to the host using udp_sendto and with the host IP
   //(learned from the packet broadcast from the host) usually seemed to work,
   //but it could not be ascertained for certain that the IP address was
   //resolved, which would result in an error from udp_sendto if it was not
   //resolved.  Even if the udp_rcv function was used to collect the broadcast
   //packet, sock_resolved always failed - probably because the socket was still
   //not tied to the host IP.  The solution used here is to close and reopen
   //the socket forcing the remote IP to that of the host (now known).  This
   // allowed sock_resolved to be used to ensure that the sendto would not fail.

   udp_close(&socket);
   status = udp_open(&socket, 4446, datagramInfo.remip, 4445, NULL);

   // give tcp handler time to resolve the newly received IP address to the
   // hardware address or the udp_sendto will fail
   timeOut = 0; status = 0;
   while(timeOut++ < 100 && status == 0){
      tcp_tick(NULL);
      status = sock_resolved(&socket);
   }

   status =
   		udp_sendto(&socket, buffer, strlen(buffer), datagramInfo.remip, 4445);
   printf("Send status  : %d\n", status);

   udp_close(&socket);

}//end of waitForHostViaUDP
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// waitForHostTCPIPConnection
//
// Waits for the host computer to make a TCP/IP connection. Sends a greeting
// back to the host after the connection is established.
//

void waitForHostTCPIPConnection(tcp_Socket *socket)
{

	const char * const greeting = "Hello from the device!\n";
	const int greetingLength = 26;

   tcp_listen(socket,PORT,0,0,NULL,0);

   printf("Waiting for connection...\n");
   while(!sock_established(socket) && sock_bytesready(socket)==-1)
      tcp_tick(socket);

   printf("Connection received...\n");

   sock_flushnext(socket);
   sock_write(socket, greeting, greetingLength);

}//end of waitForHostTCPIPConnection
//----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// reSync
//
// Clears bytes from the socket buffer until 0xaa byte reached which signals
// the *possible* start of a new valid packet header or until the buffer is
// empty.
//
// If an 0xaa byte is found, the flag reSynced is set true to that other
// functions will know that an 0xaa byte has already been removed from the
// stream, signalling the possible start of a new packet header.
//

void reSync(tcp_Socket *socket)
{

   char pktBuffer[1];

   reSynced = FALSE;

   //track the number of time this function is called, even if a resync is not
   //successful - this will track the number of sync errors
   reSyncCount++;

   //store info pertaining to what caused the reSync - these values will be
   //overwritten by the next reSync, so they only reflect the last error
   //NOTE: when a reSync occurs, these values are left over from the PREVIOUS
   // packet, so they indicate what PRECEDED the sync error.

   reSyncPktID = pktID;

   while (sock_bytesready(socket) > 0) {
      sock_fastread(socket,pktBuffer,1);
       if (pktBuffer[0] == (byte)0xaa) {reSynced = TRUE; break;}
   }

}//end of reSync
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// processDataPackets
//
// This function should be called often to allow processing of data packets
// received from the remotes and stored in the socket buffer.
//
// All packets received from the remote devices should begin with
// 0xaa, 0x55, 0xbb, 0x66, followed by the packet identifier/command.  This is
// followed by the packet data and a checksum which includes all packet data
// and the packet identifier/command.  When the packet data and packet
// identifier are added to the checksum, the result should be zero.
//
// If pWaitForPkt is true, the function will wait until data is available.
//
// Returns number of bytes retrieved from the socket, not including the
// 4 header bytes and the packet ID. Thus, if a non-zero value is returned, a
// packet was processed.  If zero is returned, some bytes may have been read
// but a packet was not successfully processed due to missing bytes or header
// corruption.  Some functions simply return 1 if the number of bytes read
// is inconsequential.
//
// If the function returns -1, then some bytes may have been read but a
// complete packet and command were not read and executed fully.  It is
// expected that the next call will result in a reSync to clean up any leftover
// bytes.
//
// It is up to the target function to perform an optional verification
// of the checksum.
//

int processDataPackets(tcp_Socket *socket, int pWaitForPkt)
{

   int bytes_read;
   char pktBuffer[10];

   //wait for a packet if parameter is true
   if (pWaitForPkt){while(sock_bytesready(socket) < 5){}}

   //wait until 5 bytes are available - this should be the 4 header bytes, and
   //the packet identifier/command
   if ((bytes_read = sock_bytesready(socket)) < 5) return 0;

   //read the bytes in one at a time so that if an invalid byte is encountered
   //it won't corrupt the next valid sequence in the case where it occurs
   //within 3 bytes of the invalid byte

   //check each byte to see if the first four create a valid header
   //if not, jump to resync which deletes bytes until a valid first header
   //byte is reached

   //if the reSynced flag is true, the buffer has been resynced and an 0xaa
   //byte has already been read from the buffer so it shouldn't be read again

   //after a resync, the function exits without processing any packets

   if (!reSynced){
       //look for the 0xaa byte unless buffer just resynced
         sock_fastread(socket,pktBuffer,1);
       if (pktBuffer[0] != (byte)0xaa) {reSync(socket); return 0;}
   }
   else reSynced = FALSE;

   sock_fastread(socket,pktBuffer,1);
   if (pktBuffer[0] != (byte)0x55) {reSync(socket); return 0;}
   sock_fastread(socket,pktBuffer,1);
   if (pktBuffer[0] != (byte)0xbb) {reSync(socket); return 0;}
   sock_fastread(socket,pktBuffer,1);
   if (pktBuffer[0] != (byte)0x66) {reSync(socket); return 0;}

   //read in the packet identifier/command
   sock_fastread(socket,pktBuffer,1);

   //store the ID of the packet (the packet type)
   pktID = pktBuffer[0];

   if (pktID == GET_CHASSIS_SLOT_ADDRESS_CMD)
      return getChassisSlotAddress(socket, pktID);
   else
   // return the status byte which tells the state of the system
   if (pktID == GET_STATUS_CMD) return getStatus(socket, pktID);
   else
   if (pktID == GET_INSPECT_PACKET_CMD) return getInspectPacket(socket, pktID);
   else
   if (pktID == START_MONITOR_CMD) return startMonitor(socket, pktID);
   else
   if (pktID == STOP_MONITOR_CMD) return stopMonitor(socket, pktID);
   else
   // set encoder counts to 0
   if (pktID == ZERO_ENCODERS_CMD) return zeroEncoderCounts(socket, pktID);
   else
   // pulse the specified output(s)
   if (pktID == PULSE_OUTPUT_CMD) return pulseOutput(socket, pktID);
   else
   // turn on the specified output(s)
   if (pktID == TURN_ON_OUTPUT_CMD) return turnOnOutput(socket, pktID);
   else
   // turn off the specified output(s)
   if (pktID == TURN_OFF_OUTPUT_CMD) return turnOffOutput(socket, pktID);
   else
   // set the number of encoder count change to trigger update to host
   if (pktID == SET_ENCODERS_DELTA_TRIGGER_CMD)
      return setEncodersDeltaTrigger(socket, pktID);
   else
   // enter inspection mode
   if (pktID == START_INSPECT_CMD)  return startInspect(socket, pktID);
   else
   // exit inspection mode
   if (pktID == STOP_INSPECT_CMD) return stopInspect(socket, pktID);
   else
   // force a monitor packet to be sent to the host
   if (pktID == GET_MONITOR_PACKET_CMD) return getMonitorPacket(socket, pktID);
   else
   if (pktID == LOAD_FIRMWARE_CMD)
      return receiveAndInstallNewFirmware(socket, pktID);
	else
   if (pktID == SET_CONTROL_FLAGS_CMD) return setControlFlags(socket, pktID);
   else
   if (pktID == RESET_TRACK_COUNTERS_CMD)
   	return resetTrackCounters(socket, pktID);
	else
   if (pktID == GET_ALL_ENCODER_VALUES_CMD)
      return getAllEncoderValues(socket, pktID);

   return 0;

}//end of processDataPackets
//-----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// main
//
//

main()
{

   //TCPIP variables
   tcp_Socket socket;

	controlFlags = 0;
   reSyncCount = 0; reSyncPktID = 0; reSynced = FALSE;
   pktError = 0;

   output1State = FALSE, output1Timer = 0;
   output2State = FALSE, output2Timer = 0;
	output3State = FALSE, output3Timer = 0;
   output4State = FALSE, output4Timer = 0;

   //init the timer interrupt variables
   timerBCount = 0;
   countBAccessFlag = 0;

   //init encoder variables
   encoder1Count = 0;
   encoder2Count = 0;
   encAccessFlag = 0;

   // initialize I/O pins
   //brdInit();  -- DO NOT CALL brdInit - not a good config for Capulin --

   setupEthernet();

   // setup all registers and I/O ports
//debug mks   initRegisters();

   // setup the Timber B interrupt and install the Interrupt Service Routine
   // to track the encoder inputs
   setupTimerBInt();

   while(1) {

      //wait for the host computer to broadcast the roll call via UDP
      waitForHostViaUDP();

      waitForHostTCPIPConnection(&socket);

      printf("Waiting for command...\n");

      //this is the main processing loop

      do{

         //process any data packets received from the host
         // return value of -1 means packet size, checksum, or execution error
         // return value of 0 means no packet found
         // value greater than 0 means that a packet was found and processed
         // and the value usually equals the number of bytes read to
         // complete the packet

         if (processDataPackets(&socket, FALSE) == -1) pktError++;

         //process events timed with timer B
         countTimerBInts();

         //process the inspect function if enabled
         if (inspectMode) processInspect(&socket);

         //process the monitor function if enabled
         if (monitorMode) processMonitor(&socket);

      } while(tcp_tick(&socket));

         printf("Connection closed...\n");

   }//while (1)

   //printf("Command received: %d\n", buffer[0]); //debug mks

}//end of main
//----------------------------------------------------------------------------

