#include "socket.hpp"
#include "DriverComms.hpp"
#include "include/MauDriveData.h"
#include "MauTime.h"
#include <wpi/priority_mutex.h>
#include "ErrorOrPrintMessage.h"
#include "buddy.h"
#include <unistd.h>
#include <sys/statvfs.h>
#include <sys/types.h>
#include <signal.h>

// UDP Driver Station Thread constants
#define WPI_DRIVESTATION_UDP_ROBOTLISTEN_PORT   1110
#define WPI_DRIVESTATION_UDP_DSLISTEN_PORT		1150

// TCP Driver Station Thread constants
#define WPI_DRIVESTATION_TCP_PORT   			1740
#define MAX_NUM_DS_CONNECTIONS      			1
#define MAX_WPI_DRIVESTATION_TCP_READSIZE		8192
#define MAX_WPI_DRIVESTATION_UDP_READSIZE		4096

// TCP Log Server Thread constants
#define MAX_NUM_CLIENT_CONNECTIONS				5
#define WPI_LOGGING_PORT            			1741
#define NEW_DS_TCP_MESSAGE_WAIT_TIMEOUT_MS 		5

// STDOUT Capture constants
#define STDOUT_CAPTURE_TIMEOUT_MS				50
#define MAX_STDOUT_LINE_LEN						32768

// Buddy-system (constant-time memory allocator) for message memory allocation
#define LOG_BUFFER_SIZE_POWER  					16	// 16:  64K
#define LOG_BUFFER_SIZE 						(1 << LOG_BUFFER_SIZE_POWER) // NOTE:  Must be power of 2-sized.

// Logging Message constants
#define MAX_MESSAGE_LEN							4096	// Maximum allowable individual message length

static uint64_t ntoh64(uint64_t *input) {
	uint64_t rval;
	uint8_t *data = (uint8_t *)&rval;

	data[0] = *input >> 56;
	data[1] = *input >> 48;
	data[2] = *input >> 40;
	data[3] = *input >> 32;
	data[4] = *input >> 24;
	data[5] = *input >> 16;
	data[6] = *input >> 8;
	data[7] = *input >> 0;

	return rval;
}

static uint64_t hton64(uint64_t *input) {
	return (ntoh64(input));
}

#include <thread>
namespace mau {
    namespace comms {

    	// DriveStation Protocol working buffers
    	char ds_udp_send_buffer[44];			// UDP Send Buffer
    	char ds_udp_rcv_buffer[MAX_WPI_DRIVESTATION_UDP_READSIZE];		// UDP Decode Buffer
    	char ds_tcp_rcv_buffer[MAX_WPI_DRIVESTATION_TCP_READSIZE];		// TCP Receive Buffer

    	bool version_data_requested = false;

    	// Buddy-system memory allocator, for allocation of log messages
    	BuddyPool *logmsg_mem_allocator = 0;
    	std::mutex allocLock;

    	// Multi-producer, single-consumer message queue
    	// Newer messages are placed later in the linked list
    	MessageHeader *s_p_head = 0;		// oldest message in queue
    	MessageHeader *s_p_tail = 0;		// newset message in queue
    	std::mutex queueLock;
    	std::mutex queueNewDataLock;
    	std::condition_variable queueNewDataCondition;

    	// Log Message Sequence numbers
    	int sequence_number = 0;

    	// DS UDP Protocol Working Variables
    	uint8_t sq_1 = 0;				// Sequence Number
    	uint8_t sq_2 = 0;				// Sequence Number
    	uint8_t udp_ds_control = 0;		// Control Word

    	void (*shutdown_handler)(int) = 0;

    	_TempJoyData joys[HAL_kMaxJoysticks];			// Last-received Joystick Data
    	long long lastDsUDPUpdateReceivedTimestamp;

    	// Cached System Status
    	volatile double voltage;

    	// Cached CAN Bus Status;
    	volatile float		canStatusPercentBusUtilization = 0.0f;
    	volatile uint32_t	canStatusBusOffCount = 0;
    	volatile uint32_t	canStatusTxFifoFullCount = 0;
    	volatile uint8_t	canStatusRxErrorCount = 0;
    	volatile uint8_t    canStatusTxErrorCount = 0;

    	std::thread udpThread;
    	std::thread tcpThread;
    	std::thread logServerThread;
    	std::thread stdoutCaptureThread;
    	std::mutex runLock;
    	volatile bool isRunning;

    	// Internal Threads
    	void tcpProcess();
    	void udpProcess();
    	void logServerProcess();
        void stdoutCaptureProcess();

    	// Message memory allocation routines
    	void *AllocMessageMemory(size_t len);
    	void FreeMessageMemory(void *p_mem);

    	// Message Queue (linked list) routines
    	void AddTail(MessageHeader *p_new);
    	MessageHeader *RemoveHead();
    	// Integrated Memory Allocation & Queue Routines
    	void *AllocMessageMemoryWithReclamation(size_t len);

    	// Internal helpers
        void enqueuePrintMessage(char *msg);

        uint32_t getTotalSystemMemory();
        uint64_t getAvailableDiskSpace();
    }
}

uint32_t mau::comms::getTotalSystemMemory() {
	long pages = sysconf(_SC_PHYS_PAGES);
	long page_size = sysconf(_SC_PAGE_SIZE);
	uint32_t total_system_memory = static_cast<uint32_t>(pages) * static_cast<uint32_t>(page_size);
	return total_system_memory;
}

uint64_t mau::comms::getAvailableDiskSpace() {
	struct statvfs stat;
	if (statvfs("/", &stat) != 0) {
		return 0;
	}
	return static_cast<uint64_t>(stat.f_bsize) * static_cast<uint64_t>(stat.f_bavail);
}

void mau::comms::setShutdownHandler(void (*shutdown_handler_func)(int)) {
	mau::comms::shutdown_handler = shutdown_handler_func;
}

void mau::comms::start() {
    if (!isRunning) {
    	logmsg_mem_allocator = new BuddyPool(LOG_BUFFER_SIZE);
        Toast::Net::Socket::socket_init();

        runLock.lock();
        isRunning = true;
        printf("Server Running...\n");
        runLock.unlock();

        udpThread = std::thread(udpProcess);
        udpThread.detach();
        tcpThread = std::thread(tcpProcess);
        tcpThread.detach();
        stdoutCaptureThread = std::thread(stdoutCaptureProcess);
        stdoutCaptureThread.detach();
    }
}

void mau::comms::stop() {
    runLock.lock();
    isRunning = false;
    runLock.unlock();
}

//// ----- DriverStation Comms: Encode ----- ////

#define UDP_DS_PROTOCOL_VERSION				0x01

// All protocol tags are sent in ROBOT->DS direction, except where indicated below
#define DS_UDP_PROTOCOL_HID_RUMBLE					 1 // u32 HIDOutputs, u16 leftRumble, u16 rightRumble
#define DS_UDP_PROTOCOL_DISK_USAGE					 4 // u64, in bytes
#define DS_UDP_PROTOCOL_CPU_USAGE					 5 // u8 count, f32:  percent for each (Normal, Timed Structs, Time Critical, ISR)
#define DS_UDP_PROTOCOL_MEMORY_USAGE				 6 // u32:  free memory, u32:  largest free block
#define DS_UDP_PROTOCOL_MATCH_TIME_COUNTDOWN		 7 // DS->ROBOT:  f32 with a sentinel of -1 when not available
#define DS_UDP_PROTOCOL_PDP_STATUS					 8 // u8 PDP Device ID, followed by 3 concatenated status frames (24 bytes)
#define DS_UDP_PROTOCOL_PDP_ENERGY					 9 // u8 PDP Device ID, followed by the energy frame (8 bytes)
#define DS_UDP_PROTOCOL_TAG_JOYSTICK_STATE			12 // DS->ROBOT:  axes (u8 count, i8 value) per axis, buttons (u8 count, u8 bit array [lsb], and pov (u8 count, i16 value per pov)
#define DS_UDP_PROTOCOL_TAG_CAN_STATS				14 // 1Hz; u32 pctBusUtil, u32 busOffCount, u32 txFifoFullCnt, u8 rxErrCnt, u8 txErrCnt
#define DS_UDP_PROTOCOL_TAG_DATE_TIME				15 // DS->ROBOT:  // Selected to match the linux API UTC Timezone (requested by robot)
#define DS_UDP_PROTOCOL_TAG_TIME_ZONE				16 // DS->ROBOT:  // ASCII string matching the linux std (requested by robot)

#define DS_UDP_PROTOCOL_HEADER_LENGTH						6
#define DS_UDP_PROTOCOL_HEADER_INDEX_VERSION		 		2
#define DS_UDP_PROTOCOL_HEADER_INDEX_CONTROLBYTE	 		3
#define DS_UDP_PROTOCOL_HEADER_INDEX_REQUESTBYTE	 		4
#define DS_UDP_PROTOCOL_HEADER_INDEX_ALLIANCE_STATION_ID	5



#define UDP_DS_STATUSBYTE_ESTOP				0x80
#define UDP_DS_STATUSBYTE_BROWNOUTPREVENT	0x10
#define UDP_DS_STATUSBYTE_PROGSTART			0x08
#define UDP_DS_STATUSBYTE_ENABLED			0x04
#define UDP_DS_STATUSBYTE_AUTO				0x02
#define UDP_DS_STATUSBYTE_TEST				0x01

#define UDP_DS_USERPROGRAM_TRACE_USERCODE	0x20
#define UDP_DS_USERPROGRAM_TRACE_ROBORIO	0x10
#define UDP_DS_USERPROGRAM_TRACE_TEST		0x08
#define UDP_DS_USERPROGRAM_TRACE_AUTO		0x04
#define UDP_DS_USERPROGRAM_TRACE_TELEOP		0x02
#define UDP_DS_USERPROGRAM_TRACE_DISABLED	0x01

#define UDP_DS_REQUESTBYTE_DISABLE_BIT		0x02
#define UDP_DS_REQUESTBYTE_DATETIME_BIT		0x01

void mau::comms::encodePacket(char* data) {

	ErrorOrPrintMessage m;

    data[0] = sq_1;
    data[1] = sq_2;
    data[2] = UDP_DS_PROTOCOL_VERSION;							// Comm Version
    data[3] = udp_ds_control;									// STATUSBYTE (TODO:  "ProgStart" (instead of FMS) and "BrownoutPrevent" bits)
    data[4] = UDP_DS_USERPROGRAM_TRACE_ROBORIO |
    		  UDP_DS_USERPROGRAM_TRACE_USERCODE;				// USERPROGRAM_TRACE (TODO:  Add Test/Auto/Tele/Disabled bits)

    double voltage_intpart =
    		static_cast<double>(static_cast<int>(voltage));
    data[5] = (uint8_t) voltage_intpart;
    double voltage_decpart = voltage - voltage_intpart;
    data[6] = (uint8_t) (voltage_decpart * 100);
    data[7] = 0;												// REQUESTBYTE: Used to make requests of the DS

    // Tagged Data goes here....
    data[8] = 9;
    data[9] = DS_UDP_PROTOCOL_MEMORY_USAGE;
    uint32_t free_memory = getTotalSystemMemory();
    *(uint32_t *)&data[10] = htonl(free_memory);
    *(uint32_t *)&data[14] = htonl(free_memory);

    data[18] = 9;
    data[19] = DS_UDP_PROTOCOL_DISK_USAGE;
    uint64_t free_disk = getAvailableDiskSpace();
    *(uint64_t *)&data[20] = hton64(&free_disk);

    data[28] = 15;
    data[29] = DS_UDP_PROTOCOL_TAG_CAN_STATS; // 1 Hz.  TODO:  only send this periodically....
    *(uint32_t *)&data[30] = m.FloatToNetworkOrderedU32(canStatusPercentBusUtilization);
    *(uint32_t *)&data[34] = htonl(canStatusBusOffCount);
    *(uint32_t *)&data[38] = htonl(canStatusTxFifoFullCount);
    data[42] = canStatusRxErrorCount;
    data[43] = canStatusTxErrorCount;
}

void mau::comms::setInputVoltage(double new_voltage) {
	voltage = new_voltage;
}

void mau::comms::setCANStatus(float percentBusUtilization, uint32_t busOffCount, uint32_t txFifoFullCount, uint8_t rxErrorCount, uint8_t txErrorCount)
{
	mau::comms::canStatusPercentBusUtilization = percentBusUtilization;
	mau::comms::canStatusBusOffCount = busOffCount;
	mau::comms::canStatusTxFifoFullCount = txFifoFullCount;
	mau::comms::canStatusRxErrorCount = rxErrorCount;
	mau::comms::canStatusTxErrorCount = txErrorCount;
}

//// ----- DriverStation Comms: Decode ----- ////

int32_t mau::comms::enqueueErrorMessage(uint16_t num_occur, int32_t errorCode, uint8_t flags, const char *details, const char *location, const char *callStack) {
	ErrorOrPrintMessage m;
	// Calculate required length
	int len = m.FormatErrorMessage(NULL, MAX_MESSAGE_LEN, sequence_number, num_occur, errorCode, flags, details, location, callStack);
	if (len < 0) return -1;
	// Allocate storage for message
	unsigned char *p_mem = static_cast<unsigned char *>(AllocMessageMemoryWithReclamation(len));
	if (p_mem) {
		len = m.FormatErrorMessage(p_mem, len, sequence_number, num_occur, errorCode, flags, details, location, callStack);
		if (len > 0) {
			sequence_number++;
			AddTail(static_cast<MessageHeader *>(static_cast<void *>(p_mem)));
			return 0;
		} else {
			FreeMessageMemory(p_mem);
		}
	}
	return -1;
}

// All protocol tags are sent in ROBOT->DS direction, except where indicated below
#define DS_TCP_PROTOCOL_TAG_ERROR_WARN_MSG			0	// Same format as currently used, <markup>, etc.
#define DS_TCP_PROTOCOL_USAGE_REPORT				1	// Same as currently used
#define DS_TCP_PROTOCOL_TAG_JOYSTICK_DESCRIPTIONS	2	// DS->ROBOT:
#define DS_TCP_PROTOCOL_TAG_MATCH_INFO				3	// DS->ROBOT: u8 eventNameLen, chars[x] ASCII Encoded name, u8 Match tupe ('P','Q','E' or '00'), u8 matchNumber
#define DS_TCP_PROTOCOL_TAG_DISABLE_COUNT			4	// u16 System Comm time-out count, u16 Input Power fault count
#define DS_TCP_PROTOCOL_TAG_USER_RAIL_FAULT_COUNT	5	// 6V, 5V, 3.3V rail fault count (u16s)
#define DS_TCP_PROTOCOL_TAG_PDPLOGRECORD			6	// NO LONGER USED
#define DS_TCP_PROTOCOL_TAG_NEWER_MATCH_INFO		7	// DS->ROBOT; newer match info (u8 event len + chars[var]), u8 MatchType (enum None/Practice/Qualification/Elimination/Test), u16 matchNum, u8 replayNUmber
#define DS_TCP_PROTOCOL_TAG_ROBOT_IPADDR_FOR_DB		8	// u32
#define DS_TCP_PROTOCOL_TAG_DB_WINDOW_MODE			9	// u8
#define DS_TCP_PROTOCOL_TAG_VERSION_DATA		   10	// one tag for each version (u32 id if model, otherwise 0), u8 elementNameLen, char[var], u8 versionStringLen, char[var]
#define DS_TCP_PROTOCOL_TAG_ERROR_DATA			   11	// See ErrorOrPrintMessage.h
#define DS_TCP_PROTOCOL_TAG_CONSOLE_PRINT_DATA	   12	// See ErrorOrPrintMessage.h
#define DS_TCP_PROTOCOL_TAG_USER_RAIL_FAULT_DATA   13	// u16 5VBrownoutCOunt, u8 6VRailStatusEnum, u8 5VRailStatusEnum, u8 3.3VRailStatusEnum
#define DS_TCP_PROTOCOL_TAG_GAME_SPECIFIC_MSG	   14	// DS->ROBOT; (the message is the entire size of the tag)

void mau::comms::decodeTcpPacket(char* data, int length) {
	int currTagDataFirstIndex = 0;
	while (currTagDataFirstIndex < length) {
		uint16_t currTagDataLen = ntohs(*(uint16_t *)&data[currTagDataFirstIndex]);
		if (currTagDataLen < 1) {
			// Invalid Data Length received; this is a sign of an error or truncated packet.
			return;
		}
		currTagDataFirstIndex += sizeof(currTagDataLen);		// Seek beyond the length variable
		currTagDataLen -= sizeof(uint8_t);						// Exclude size of the tag variable
		char currTag = data[currTagDataFirstIndex++];			// Read tag value and seek beyond the tag ID
		switch (currTag) {
			case DS_TCP_PROTOCOL_TAG_JOYSTICK_DESCRIPTIONS:
			{
				int i = currTagDataFirstIndex;
				while ((i - currTagDataFirstIndex) < currTagDataLen) {
					uint8_t joyid = data[i++];
					bool xbox = data[i++] == 1;
					uint8_t type = data[i++];
					uint8_t name_length = data[i++];
					int nb_i = i;
					i += name_length;
					uint8_t axis_count = data[i++];
					//uint8_t axis_types[16];
					int at_i = i;
					i += axis_count;
					uint8_t button_count = data[i++];
					uint8_t pov_count = data[i++];

					if (type != 255 && axis_count <= HAL_kMaxJoystickAxes) {
						HAL_JoystickDescriptor desc;
						desc.buttonCount = button_count;
						desc.povCount = pov_count;
						desc.isXbox = xbox;
						desc.type = type;
						// Range-protected received joystick name length
						if (name_length >= static_cast<int>(sizeof(desc.name))) {
							name_length = sizeof(desc.name) - 1;
						}
						memcpy(desc.name, &data[nb_i], name_length);
						desc.name[name_length] = '\0';
						// Range-protected received joystick axis count
						if (axis_count > HAL_kMaxJoystickAxes) {
							desc.axisCount = HAL_kMaxJoystickAxes;
						} else {
							desc.axisCount = axis_count;
						}
						for (int x = 0; x < desc.axisCount; x++) {
							desc.axisTypes[x] = data[at_i + x];
						}
						Mau_DriveData::updateJoyDescriptor(joyid, &desc);
					}

				}
				break;
			}
			case DS_TCP_PROTOCOL_TAG_MATCH_INFO:
				break;
			case DS_TCP_PROTOCOL_TAG_NEWER_MATCH_INFO:
			{
				char event_name[256];
				int i = currTagDataFirstIndex;
				uint8_t event_name_length = data[i++];
				memcpy(event_name, &data[i], event_name_length);
				event_name[event_name_length] = '\0';
				i += event_name_length;
				uint8_t match_type = data[i]; // enum {None, Practice, Qualification, Elimination, Test}
				i += sizeof(match_type);
				uint16_t match_number = ntohs(*(uint16_t *)&data[i]);
				i += sizeof(match_number);
				uint8_t replay_number = data[i];
				Mau_DriveData::updateMatchIdentifyInfo(event_name, match_type, match_number, replay_number);
				break;
			}
			case DS_TCP_PROTOCOL_TAG_GAME_SPECIFIC_MSG:
			{
				uint16_t game_specific_msg_size = currTagDataLen;
				Mau_DriveData::updateMatchGameSpecificMessage(game_specific_msg_size, static_cast<uint8_t *>(static_cast<void *>(&data[currTagDataFirstIndex])));
				break;
			}
			default:
				break;
		}
		currTagDataFirstIndex += currTagDataLen;
	}
}

void mau::comms::decodeUdpPacket(char* data, int length) {

	if (length < DS_UDP_PROTOCOL_HEADER_LENGTH) return;

	// Decode Packet Number
	sq_1 = data[0];
    sq_2 = data[1];
    uint16_t sequence_number = (static_cast<uint16_t>(sq_1) << 8) + sq_2;

    char version = data[DS_UDP_PROTOCOL_HEADER_INDEX_VERSION];

    if (version == 1) {

        udp_ds_control = data[DS_UDP_PROTOCOL_HEADER_INDEX_CONTROLBYTE];
        bool test = IS_BIT_SET(udp_ds_control, 0);
        bool auton = IS_BIT_SET(udp_ds_control, 1);
        bool enabled = IS_BIT_SET(udp_ds_control, 2);
        bool fms = IS_BIT_SET(udp_ds_control, 3);
        bool eStop = IS_BIT_SET(udp_ds_control, 7);

        uint8_t udp_ds_request = data[DS_UDP_PROTOCOL_HEADER_INDEX_REQUESTBYTE];
        version_data_requested = IS_BIT_SET(udp_ds_request, 0);	// Requests that version info be sent
        bool usage_request = IS_BIT_SET(udp_ds_request, 1);
        bool restart = IS_BIT_SET(udp_ds_request, 2);			// Soft Restart
        bool reboot = IS_BIT_SET(udp_ds_request, 3);			// Hard Restart
        bool progStartRequest = IS_BIT_SET(udp_ds_request, 4);	// How is this different than Soft Restart?

        int shutdown_code;
        if (reboot || restart) {
        	if (reboot) {
        		shutdown_code = MAU_COMMS_SHUTDOWN_REBOOT;
        		printf("NOTICE: Driver Station Requested Reboot.\n");
        		printf("#DS UDP Seq#:  %d.  Bytes:  %d.  Header Data:  %02X %02X %02X %02X %02X %02X\n", sequence_number, length, data[0], data[1], data[2], data[3], data[4], data[5]);
        	} else {
        		shutdown_code = MAU_COMMS_SHUTDOWN_RESTART;
        		printf("NOTICE: Driver Station Requested Code Restart.\n");
        		printf("#DS UDP Seq#:  %d.  Bytes:  %d.  Header Data:  %02X %02X %02X %02X %02X %02X\n", sequence_number, length, data[0], data[1], data[2], data[3], data[4], data[5]);
        	}

            stop();
            if (shutdown_handler) {
            	shutdown_handler(shutdown_code);
            }
            return;

        } else if (eStop) {
            printf("NOTICE: Driver Station Estop \n");
    		printf("#DS UDP Seq#:  %d.  Bytes:  %d.  Header Data:  %02X %02X %02X %02X %02X %02X\n", sequence_number, length, data[0], data[1], data[2], data[3], data[4], data[5]);

    		stop();
            if (shutdown_handler) {
            	shutdown_handler(MAU_COMMS_SHUTDOWN_ESTOP);
            }
            return;

        }
        HAL_AllianceStationID alliance = (HAL_AllianceStationID)data[DS_UDP_PROTOCOL_HEADER_INDEX_ALLIANCE_STATION_ID];

        bool ds_attached;
        if (mau::vmxGetTime() - lastDsUDPUpdateReceivedTimestamp > 1000000) {
            // DS Disconnected
            ds_attached = false;
        } else {
            // DS Connected
            ds_attached = true;
        }

        Mau_DriveData::updateControlWordAndAllianceID(enabled, auton, test, eStop, fms, ds_attached, alliance);

        int i = DS_UDP_PROTOCOL_HEADER_LENGTH;
        bool search = true;

        while (i < length && search) {
            int struct_size = data[i];
            char tag = data[i + 1];
            switch (tag) {
				case DS_UDP_PROTOCOL_TAG_JOYSTICK_STATE:
				{
					bool remaining_joystick_data = true;
					int joy_id = 0;

			        for (int joyNum = 0; joyNum < HAL_kMaxJoysticks; joyNum++) {
			            _TempJoyData* tempJoy = &joys[joyNum];
			            tempJoy->has_update = false;
			        }

					while (remaining_joystick_data) {
						_TempJoyData* joy = &joys[joy_id];
						joy->has_update = true;
						char axisCount = data[i + 2];
						if (axisCount > HAL_kMaxJoystickAxes) {
							axisCount = HAL_kMaxJoystickAxes;
							printf("DriverComms::decodeUdpPacket got too-large axis count during Joystick Update; this event was discarded.  Value limited to miaximum.\n");
						}
						joy->axis_count = axisCount;
						for (int ax = 0; ax < joy->axis_count; ax++) {
							joy->axis[ax] = data[i + 2 + ax + 1];
						}
						int b = i + 2 + axisCount + 1;
						joy->button_count = data[b];
						int button_delta = (joy->button_count / 8 + ((joy->button_count % 8 == 0) ? 0 : 1));
						uint32_t total_mask = 0;
						for (int bm = 0; bm < button_delta; bm++) {
							uint8_t m = data[b + bm + 1];
							total_mask = (total_mask << (bm * 8)) | m;
						}
						joy->button_mask = total_mask;
						b += button_delta + 1;
						char povCount = data[b];
						if (povCount > HAL_kMaxJoystickPOVs) {
							povCount = HAL_kMaxJoystickPOVs;
							printf("DriverComms::decodeUdpPacket got too-large POV count during Joystick Update; this event was discarded.  Value limited to miaximum.\n");
						}
						joy->pov_count = povCount;
						int bytes_for_pov = 0;
						for (int pv = 0; pv < joy->pov_count; pv++) {
							uint8_t a1 = data[b + 1 + (pv * 2)];
							uint8_t a2 = data[b + 1 + (pv * 2) + 1];
							joy->pov[pv] = (uint16_t) (a1 << 8 | a2);
							bytes_for_pov += 2;
						}

						int next_index = b + bytes_for_pov;
						if (next_index >= (i + struct_size)) {
							remaining_joystick_data = false;

					        for (int joyNum = 0; joyNum < HAL_kMaxJoysticks; joyNum++) {
					            _TempJoyData* tempJoy = &joys[joyNum];
					            if (tempJoy->has_update) {
					            	Mau_DriveData::updateJoyAxis(joyNum, tempJoy->axis_count, tempJoy->axis);
					            	Mau_DriveData::updateJoyPOV(joyNum, tempJoy->pov_count, tempJoy->pov);
					            	Mau_DriveData::updateJoyButtons(joyNum, tempJoy->button_count, tempJoy->button_mask);
					            }
					        }
						} else {
							joy_id++;
							if (joy_id >= HAL_kMaxJoysticks) {
								// This is a sign of an internal error.
								printf("DriverComms::decodeUdpPacket got extraneous data during Joystick Update; this event was discarded.\n");
								break;
							}
						}
					}

					break;
				}
				case DS_UDP_PROTOCOL_MATCH_TIME_COUNTDOWN:
				{
					float matchTime = ErrorOrPrintMessage::NetworkOrderedU32ToFloat(*(uint32_t *)(&data[i + 2]));
					Mau_DriveData::updateMatchTime(matchTime);
					break;
				}
				case DS_UDP_PROTOCOL_TAG_DATE_TIME:
				{
					// TODO:  Implement once purpose is understood.
					break;
				}
				case DS_UDP_PROTOCOL_TAG_TIME_ZONE:
				{
					// TODO:  Implement once purpose is understood.
					break;
				}
				default:
					break;
            }
            i += struct_size + 1;
        }
        lastDsUDPUpdateReceivedTimestamp = mau::vmxGetTime();
    }
}

namespace mau {
    namespace comms {

    	// callback invoked whenever new data is available on the DS TCP Socket
        void driverStationTCPDataReceivedCallback(int client_id, Toast::Net::Socket::ClientSocket sock) {
			int ret = sock.read(ds_tcp_rcv_buffer, MAX_WPI_DRIVESTATION_TCP_READSIZE);
			if (ret > 0) {
				mau::comms::decodeTcpPacket(ds_tcp_rcv_buffer, ret);
			}
        }

		void tcpProcess() {

        	// The driver station server socket accepts a single TCP client connections
        	// This is configured as non-blocking.
			Toast::Net::Socket::ServerSocket dsServerSocket(WPI_DRIVESTATION_TCP_PORT);
            Toast::Net::Socket::socket_nonblock(dsServerSocket.get_socket());
            Toast::Net::Socket::SelectiveServerSocket ds_sock(dsServerSocket.get_socket(), MAX_NUM_DS_CONNECTIONS);
            ds_sock.prepare();
            ds_sock.on_data(mau::comms::driverStationTCPDataReceivedCallback);
            dsServerSocket.open();

        	// The secondary logging server socket accepts multiple TCP client connections
        	// This is configured as non-blocking.
        	Toast::Net::Socket::ServerSocket logServerSocket(WPI_LOGGING_PORT);
            Toast::Net::Socket::socket_nonblock(logServerSocket.get_socket());
            Toast::Net::Socket::SelectiveServerSocket log_sock(logServerSocket.get_socket(), MAX_NUM_CLIENT_CONNECTIONS);
            log_sock.prepare();
            logServerSocket.open();

			while (isRunning) {

    			try {
					unsigned char version_data_buffer[1024];

					// Check for new DS client (non-blocking)
					// New data from connected DS client will invoke driverStationTCPDataReceivedCallback().
					ds_sock.accept();
					// Prune dead DS client, if they have become disconnected.
					ds_sock.prune_disconnected_clients();

					// Check for new non-DS logging clients (non-blocking)
					log_sock.accept();
					// Prune dead non-DS logging clients
					log_sock.prune_disconnected_clients();

					if (version_data_requested) {

						ErrorOrPrintMessage m;
						uint32_t model_id = 0;
						const char *element_names[4];
						element_names[0] = "roboRIO Image";
						//element_names[1] = "FRC_Lib_Version";
						//element_names[2] = 0;
						const char *version_strings[4];
						version_strings[0] = "0.0.000";
						//version_strings[1] = "Scott's Test Version";
						//version_strings[2] = 0;
						if (m.FormatVersionDataMessage(version_data_buffer, sizeof(version_data_buffer),
								model_id,
								DS_TCP_PROTOCOL_TAG_VERSION_DATA,
								1,
								element_names,
								version_strings) != -1) {
							RobotVersionDataHeader *header = (RobotVersionDataHeader *)version_data_buffer;
							ds_sock.send_to_all_connected_clients(static_cast<char *>(static_cast<void *>(&(header->len))), ntohs(header->len) + sizeof(uint16_t));
						}

						element_names[0] = "FRC_Lib_Version";
						version_strings[0] = "C++ 2019.4.1";
						if (m.FormatVersionDataMessage(version_data_buffer, sizeof(version_data_buffer),
								model_id,
								DS_TCP_PROTOCOL_TAG_VERSION_DATA,
								1,
								element_names,
								version_strings) != -1) {
							RobotVersionDataHeader *header = (RobotVersionDataHeader *)version_data_buffer;
							ds_sock.send_to_all_connected_clients(static_cast<char *>(static_cast<void *>(&(header->len))), ntohs(header->len) + sizeof(uint16_t));
						}

						element_names[0] = "VMXPiFirmware";
						version_strings[0] = "3.1.222";
						if (m.FormatVersionDataMessage(version_data_buffer, sizeof(version_data_buffer),
								model_id,
								DS_TCP_PROTOCOL_TAG_VERSION_DATA,
								1,
								element_names,
								version_strings) != -1) {
							RobotVersionDataHeader *header = (RobotVersionDataHeader *)version_data_buffer;
							ds_sock.send_to_all_connected_clients(static_cast<char *>(static_cast<void *>(&(header->len))), ntohs(header->len) + sizeof(uint16_t));
						}

						// Send "end of version info" indicator.
						element_names[0] = 0;
						version_strings[0] = 0;

						if (m.FormatVersionDataMessage(version_data_buffer, sizeof(version_data_buffer),
								model_id,
								DS_TCP_PROTOCOL_TAG_VERSION_DATA,
								1,
								element_names,
								version_strings) != -1) {
							RobotVersionDataHeader *header = (RobotVersionDataHeader *)version_data_buffer;
							ds_sock.send_to_all_connected_clients(static_cast<char *>(static_cast<void *>(&(header->len))), ntohs(header->len) + sizeof(uint16_t));
						}

						version_data_requested = false;
					}

					// Send pending logging messages to DS client and all non-DS logging clients.
					// Always send the oldest messages first.
					// Note that all client sockets are also non-blocking.
					MessageHeader *p_head = RemoveHead();
					while (p_head) {
						ds_sock.send_to_all_connected_clients(static_cast<char *>(static_cast<void *>(&(p_head->len))), ntohs(p_head->len) + sizeof(uint16_t));
						log_sock.send_to_all_connected_clients(static_cast<char *>(static_cast<void *>(&(p_head->len))), ntohs(p_head->len) + sizeof(uint16_t));
						FreeMessageMemory(p_head);
						p_head = RemoveHead();
					}

					// Block, waiting either for new messages, or a brief timeout
					std::unique_lock<std::mutex> lck(queueNewDataLock);
					queueNewDataCondition.wait_for(lck, std::chrono::milliseconds(NEW_DS_TCP_MESSAGE_WAIT_TIMEOUT_MS));
        		} catch(const std::exception& ex){
        			printf("mau::comms::tcpProcess - Caught exception:  %s\n", ex.what());
        		}
			}

			ds_sock.close();
		}

		void udpProcess() {
			Toast::Net::Socket::DatagramSocket sock(WPI_DRIVESTATION_UDP_ROBOTLISTEN_PORT);
			sock.bind();

			Toast::Net::Socket::SocketAddress addr;
			while (true) {
				runLock.lock();
				if (!isRunning)
					break;
				runLock.unlock();

				try {
					memset(ds_udp_rcv_buffer, 0, MAX_WPI_DRIVESTATION_UDP_READSIZE);
					int len = sock.read(ds_udp_rcv_buffer, MAX_WPI_DRIVESTATION_UDP_READSIZE, &addr);
					if (len > 0) {
						mau::comms::decodeUdpPacket(ds_udp_rcv_buffer, len);
						mau::comms::encodePacket(ds_udp_send_buffer);
						addr.set_port(WPI_DRIVESTATION_UDP_DSLISTEN_PORT);
						sock.send(ds_udp_send_buffer, 44, &addr);
					}
				} catch(const std::exception& ex){
					printf("mau::comms::udpProcess - Caught exception:  %s\n", ex.what());
        		}
			}
			sock.close();
		}

    	void *AllocMessageMemory(size_t len) {
    		void *p_mem;
    		allocLock.lock();
    		p_mem = logmsg_mem_allocator->Acquire(len);
    		allocLock.unlock();
    		return p_mem;

    	}

    	void FreeMessageMemory(void *p_mem) {
    		allocLock.lock();
    		logmsg_mem_allocator->Release(p_mem);
    		allocLock.unlock();
    	}

    	void AddTail(MessageHeader *p_new) {

    		if (!p_new) return;

    		p_new->next = 0;

    	    queueLock.lock();

    		if (s_p_tail) {
    			s_p_tail->next = p_new;
    		}

    		s_p_tail = p_new;

    		if (!s_p_head) {
    			// List is empty
    			s_p_head = p_new;
    		}

    		queueLock.unlock();

    	    queueNewDataCondition.notify_all();
    	}

    	MessageHeader *RemoveHead() {

    		queueLock.lock();

    		MessageHeader *p_head = s_p_head;

    		if (p_head) {

    			s_p_head = p_head->next;

    			if (!s_p_head) {
    				// List is now empty
    				s_p_tail = 0;
    			}

    			p_head->next = 0;
    		}

    		queueLock.unlock();

    		return p_head;
    	}

    	// Allocates memory for a Message.  If insufficient memory remains,
    	// the oldest allocated messages are deleted, until there is sufficient
    	// memory available for this message.
    	void *AllocMessageMemoryWithReclamation(size_t len) {
    		unsigned char *p_mem = static_cast<unsigned char *>(AllocMessageMemory(len));
    		if (!p_mem) {
    			// No memory available.  Keep removing oldest messages until sufficient memory is available
    			MessageHeader *p_head = RemoveHead();
    			while (!p_mem && p_head) {
    				FreeMessageMemory(p_head);
    				p_mem = static_cast<unsigned char *>(AllocMessageMemory(len));
    				if (!p_mem) {
    					p_head = RemoveHead();
    				}
    			}
    			// FATAL ERROR:  The Message Memory Pool is not big enough to handle this message.  (TODO:  Add logging here????).
    			if (!p_mem) return 0;
    		}
    		return p_mem;
    	}

        void enqueuePrintMessage(char *msg) {
        	ErrorOrPrintMessage m;
        	// Calculate required length
        	int len = m.FormatPrintMessage(NULL, MAX_MESSAGE_LEN, sequence_number, msg);
        	if (len < 0) return;

        	// Allocate storage for message
        	unsigned char *p_mem = static_cast<unsigned char *>(AllocMessageMemoryWithReclamation(len));
        	if (p_mem) {
        		len = m.FormatPrintMessage(p_mem, len, sequence_number, msg);
        		if (len > 0) {
        			sequence_number++;
        			AddTail(static_cast<MessageHeader *>(static_cast<void *>(p_mem)));
        		} else {
        			FreeMessageMemory(p_mem);
        		}
        	}
        }

        void stdoutCaptureProcess() {

        	fd_set set;
        	struct timeval timeout;

        	int out_pipe[2];
        	int stdout_prev_fd;

        	// Save current stdout file descriptor
        	stdout_prev_fd = dup(STDOUT_FILENO);
        	if (stdout_prev_fd == -1) return;

        	if (pipe(out_pipe) != 0) return;

        	// Redirect stdout to the pipe
        	if (dup2(out_pipe[1], STDOUT_FILENO) == -1) return;
        	close(out_pipe[1]);

        	FILE *pipe_file = fdopen(out_pipe[0], "r");

        	char *line = (char *)malloc(MAX_STDOUT_LINE_LEN);

        	while (isRunning) {

        		try {
					FD_ZERO(&set);
					FD_SET(out_pipe[0], &set);
					timeout.tv_sec = 0;
					timeout.tv_usec = STDOUT_CAPTURE_TIMEOUT_MS * 1000;
					int ret = select((out_pipe[0] + 1), &set, NULL, NULL, &timeout);
					if (ret == 0) {
						// Timeout
					} else if (ret > 0) {
						if (FD_ISSET(out_pipe[0], &set)) {
							// Input Available; occurs at end of every line, since STDOUT is configured
							// for line buffering in Mau HAL Initialization.
							ssize_t num_read;
							size_t len = MAX_STDOUT_LINE_LEN;
							while ((num_read = getline(&line, &len, pipe_file)) != -1) {
								// Output to console FD
								write(stdout_prev_fd, line, num_read);
								// Enqueue Message for transmission to WPI Logging Client
								if (num_read > 0) {
									line[num_read - 1] = 0; // replace final line feed with null
								}
								enqueuePrintMessage(line);
							}
						}
					} else {
						// Error
					}
        		} catch(const std::exception& ex){
        			printf("mau::comms::stdoutCaptureProcess - Caught exception:  %s\n", ex.what());
        		}
        	}

        	free(line);

        	// Restore flags and reconnect stdout after done
        	dup2(stdout_prev_fd, STDOUT_FILENO);
        	fclose(pipe_file);
        }
    }
}
