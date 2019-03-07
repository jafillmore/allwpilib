#include "socket.hpp"
#include "DriverComms.hpp"
#include "include/MauDriveData.h"
#include "MauTime.h"
#include <wpi/priority_mutex.h>
#include "ErrorOrPrintMessage.h"
#include "buddy.h"

// UDP Driver Station Thread constants
#define WPI_DRIVESTATION_UDP_ROBOTLISTEN_PORT   1110
#define WPI_DRIVESTATION_UDP_DSLISTEN_PORT		1150

// TCP Driver Station Thread constants
#define WPI_DRIVESTATION_TCP_PORT   			1740
#define MAX_NUM_DS_CONNECTIONS      			1
#define MAX_WPI_DRIVESTATION_TCP_READSIZE		8192

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

#include <thread>
namespace mau {
    namespace comms {

    	// DriveStation Protocol working buffers
    	char ds_udp_send_buffer[8];			// UDP Send Buffer
    	char ds_udp_rcv_buffer[1024];		// UDP Decode Buffer
    	char ds_tcp_rcv_buffer[1024];		// TCP Receive Buffer

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
    	uint8_t udp_ds_request = 0;		// Request

    	_TempJoyData joys[6];			// Last-received Joystick Data
    	long long lastDsUDPUpdateReceivedTimestamp;
    	double voltage;

    	std::thread udpThread;
    	std::thread tcpThread;
    	std::thread logServerThread;
    	std::thread stdoutCaptureThread;
    	std::mutex runLock;
    	bool isRunning;

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
    }
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
        //logServerThread = std::thread(logServerProcess);
        //logServerThread.detach();
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

void mau::comms::encodePacket(char* data) {
    data[0] = sq_1;
    data[1] = sq_2;
    data[2] = 0x01;
    data[3] = udp_ds_control;
    data[4] = 0x10 | 0x20;

    double voltage = voltage;

    data[5] = (uint8_t) (voltage);
    data[6] = (uint8_t) ((voltage * 100 - ((uint8_t) voltage) * 100) * 2.5);
    data[7] = 0;
}

//// ----- DriverStation Comms: Decode ----- ////

bool last = false;
void mau::comms::periodicUpdate() {
    if (mau::vmxGetTime() - lastDsUDPUpdateReceivedTimestamp > 1000000) {
        // DS Disconnected
        Mau_DriveData::updateIsDsAttached(false);
    } else {
        // DS Connected
        Mau_DriveData::updateIsDsAttached(true);
    }
    for (int joyNum = 0; joyNum < 6; joyNum++) {
        _TempJoyData* tempJoy = &joys[joyNum];
        Mau_DriveData::updateJoyAxis(joyNum, tempJoy->axis_count, tempJoy->axis);
        Mau_DriveData::updateJoyPOV(joyNum, tempJoy->pov_count, tempJoy->pov);
        Mau_DriveData::updateJoyButtons(joyNum, tempJoy->button_count, tempJoy->button_mask);

        tempJoy->has_update = false;
    }
}

void mau::comms::setInputVoltage(double new_voltage) {
	voltage = new_voltage;
}

void mau::comms::enqueueErrorMessage(uint16_t num_occur, int32_t errorCode, uint8_t flags, const char *details, const char *location, const char *callStack) {
	ErrorOrPrintMessage m;
	// Calculate required length
	int len = m.FormatErrorMessage(NULL, MAX_MESSAGE_LEN, sequence_number, num_occur, errorCode, flags, details, location, callStack);
	if (len < 0) return;
	// Allocate storage for message
	unsigned char *p_mem = static_cast<unsigned char *>(AllocMessageMemoryWithReclamation(len));
	if (p_mem) {
		len = m.FormatErrorMessage(p_mem, len, sequence_number, num_occur, errorCode, flags, details, location, callStack);
		if (len > 0) {
			sequence_number++;
			AddTail(static_cast<MessageHeader *>(static_cast<void *>(p_mem)));
		} else {
			FreeMessageMemory(p_mem);
		}
	}
}

void mau::comms::decodeTcpPacket(char* data, int length) {
    if (data[2] == 0x02) {
        // Joystick Descriptor
        int i = 3;
        while (i < length) {
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

            if (type != 255 && axis_count != 255) {
                HAL_JoystickDescriptor desc;
                desc.buttonCount = button_count;
                desc.povCount = pov_count;
                desc.isXbox = xbox;
                desc.type = type;
                if (name_length > Mau_kJoystickNameLength) {
                    name_length = Mau_kJoystickNameLength;
                }
                memcpy(desc.name, &data[nb_i], name_length);
                desc.axisCount = axis_count;
                for (int x = 0; x < axis_count; x++) {
                	if ( x < HAL_kMaxJoystickAxes) {
                        desc.axisTypes[x] = data[at_i + x];
                	} else {
                		break;
                	}
                }
                Mau_DriveData::updateJoyDescriptor(joyid, &desc);
            }
        }
    }
}

void mau::comms::decodeUdpPacket(char* data, int length) {
    sq_1 = data[0];
    sq_2 = data[1];
    if (data[2] != 0) {
        udp_ds_control = data[3];
        bool test = IS_BIT_SET(udp_ds_control, 0);
        bool auton = IS_BIT_SET(udp_ds_control, 1);
        bool enabled = IS_BIT_SET(udp_ds_control, 2);
        bool fms = IS_BIT_SET(udp_ds_control, 3);
        bool eStop = IS_BIT_SET(udp_ds_control, 7);

        udp_ds_request = data[4];
        bool reboot = IS_BIT_SET(udp_ds_request, 3);
        bool restart = IS_BIT_SET(udp_ds_request, 2);

        if (reboot || restart) {
//            printf("NOTICE: Driver Station Requested Code Restart \n");
            stop();
            exit(0);
        } else if (eStop) {
//            printf("NOTICE: Driver Station Estop \n");
            stop();
            exit(0);
        }
        HAL_AllianceStationID alliance = (HAL_AllianceStationID)data[5];
        int i = 6;
        bool search = true;
        int joy_id = 0;

        while (i < length && search) {
            int struct_size = data[i];
            search = data[i + 1] == 0x0c;
            if (!search) continue;
            _TempJoyData* joy = &joys[joy_id];
            joy->axis_count = data[i + 2];
            for (int ax = 0; ax < joy->axis_count; ax++) {
                joy->axis[ax] = data[i + 2 + ax + 1];
            }
            int b = i + 2 + joy->axis_count + 1;
            joy->button_count = data[b];
            int button_delta = (joy->button_count / 8 + ((joy->button_count % 8 == 0) ? 0 : 1));
            uint32_t total_mask = 0;
            for (int bm = 0; bm < button_delta; bm++) {
                uint8_t m = data[b + bm + 1];
                total_mask = (total_mask << (bm * 8)) | m;
            }
            joy->button_mask = total_mask;
            b += button_delta + 1;
            joy->pov_count = data[b];
            for (int pv = 0; pv < joy->pov_count; pv++) {
                uint8_t a1 = data[b + 1 + (pv * 2)];
                uint8_t a2 = data[b + 1 + (pv * 2) + 1];
                /*if (a2 < 0) a2 = 256 + a2;*/  /* ??? a2 is unsigned so can never be negative! */
                joy->pov[pv] = (uint16_t) (a1 << 8 | a2);
            }
            joy->has_update = true;
            joy_id++;
            i += struct_size + 1;
        }
        Mau_DriveData::updateAllianceID(alliance);
        Mau_DriveData::updateIsEnabled(enabled);
        Mau_DriveData::updateIsAutonomous(auton);
        Mau_DriveData::updateIsTest(test);
        Mau_DriveData::updateEStop(eStop);
        Mau_DriveData::updateIsFmsAttached(fms);
        periodicUpdate();

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

				// Check for new DS client (non-blocking)
				// New data from connected DS client will invoke driverStationTCPDataReceivedCallback().
                ds_sock.accept();
                // Prune dead DS client, if they have become disconnected.
        		ds_sock.prune_disconnected_clients();

            	// Check for new non-DS logging clients (non-blocking)
                log_sock.accept();
                // Prune dead non-DS logging clients
        		log_sock.prune_disconnected_clients();

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

				int len = sock.read(ds_udp_rcv_buffer, 1024, &addr);
				mau::comms::decodeUdpPacket(ds_udp_rcv_buffer, len);

				mau::comms::encodePacket(ds_udp_send_buffer);
				addr.set_port(WPI_DRIVESTATION_UDP_DSLISTEN_PORT);
				sock.send(ds_udp_send_buffer, 8, &addr);
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

        void logServerProcess() {

        	// The logging server socket accepts multiple TCP client connections
        	// This is configured as non-blocking.
        	Toast::Net::Socket::ServerSocket serverSocket(WPI_LOGGING_PORT);
            Toast::Net::Socket::socket_nonblock(serverSocket.get_socket());

            Toast::Net::Socket::SelectiveServerSocket sock(serverSocket.get_socket(), MAX_NUM_CLIENT_CONNECTIONS);
            sock.prepare();

            serverSocket.open();

            while (isRunning) {
            	// Accept (non-blocking) new clients (the client list is managed by the SelectiveServerSocket)
                sock.accept();

                // Prune dead clients
        		sock.prune_disconnected_clients();

                // Send pending logging messages to all clients, from oldest to newest.
        		// Note that all client sockets are also non-blocking.

        		MessageHeader *p_head = RemoveHead();
        		while (p_head) {
            		sock.send_to_all_connected_clients(static_cast<char *>(static_cast<void *>(&(p_head->len))), ntohs(p_head->len) + sizeof(uint16_t));
            		FreeMessageMemory(p_head);
            		p_head = RemoveHead();
                }

        		// Block, waiting either for new messages, or a timeout.
        		std::unique_lock<std::mutex> lck(queueNewDataLock);
        		queueNewDataCondition.wait_for(lck, std::chrono::milliseconds(NEW_DS_TCP_MESSAGE_WAIT_TIMEOUT_MS));
            }

            sock.close();  // Internally, this closes the serverSocket
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
        	}

        	free(line);

        	// Restore flags and reconnect stdout after done
        	dup2(stdout_prev_fd, STDOUT_FILENO);
        	fclose(pipe_file);
        }
    }
}
