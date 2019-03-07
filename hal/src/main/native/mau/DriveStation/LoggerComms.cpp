#include "socket.hpp"

#include "LoggerComms.hpp"
#include "MauTime.h"
#include <wpi/priority_mutex.h>
#include "buddy.h"
#include "ErrorOrPrintMessage.h"

#include <thread>
#include <condition_variable>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/select.h>
#include <fcntl.h>

static std::mutex runLock;
static bool isRunning = false;
static std::thread tcpThread;
static std::thread stdoutCaptureThread;

// Buddy-system (constant-time memory allocator) for message memory allocation

#define LOG_BUFFER_SIZE_POWER  16						// 16:  64K
#define LOG_BUFFER_SIZE (1 << LOG_BUFFER_SIZE_POWER)	// NOTE:  Must be power of 2-sized.
#define MAX_MESSAGE_LEN 4096							// Maximum allowable individual message length

static BuddyPool *mem_allocator = 0;
static std::mutex allocLock;

// Multi-producer, single-consumer message queue
// Newer messages are placed later in the linked list

static int sequence_number = 0;

static MessageHeader *s_p_head = 0;		// oldest message in queue
static MessageHeader *s_p_tail = 0;		// newset message in queue

static std::mutex queueLock;
static std::mutex queueNewDataLock;
static std::condition_variable queueNewDataCondition;

// STDOUT Capture
// Any messages output to the console (STDOUT) are also added
// to the message queue as "Print Messages".

// Message memory allocation routines

static void *AllocMessageMemory(size_t len) {
	void *p_mem;
	allocLock.lock();
	p_mem = mem_allocator->Acquire(len);
	allocLock.unlock();
	return p_mem;

}

static void FreeMessageMemory(void *p_mem) {
	allocLock.lock();
	mem_allocator->Release(p_mem);
	allocLock.unlock();
}

// Message Queue (linked list) routines

static void AddTail(MessageHeader *p_new) {

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

#if 0 // Unused
static void AddHead(MessageHeader *p_new) {

	if (!p_new) return;

    queueLock.lock();

	p_new->next = s_p_head;

	s_p_head = p_new;

	if (!s_p_tail) {
		// List is empty
		s_p_tail = p_new;
	}

    queueLock.unlock();

    queueNewDataCondition.notify_all();
}
#endif

#if 0 // Unused
static size_t GetCount() {
	size_t count = 0;

	queueLock.lock();

	MessageHeader *p_msg = s_p_head;
	while (p_msg) {
		count++;
		p_msg = p_msg->next;
	}

	queueLock.unlock();

	return count;
}
#endif

static MessageHeader *RemoveHead() {

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

// Integrated Memory Allocation & Queue Routines

static void *AllocMessageMemoryWithReclamation(size_t len) {
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

// TCP Log Server Thread

#define MAX_NUM_CLIENT_CONNECTIONS	5
#define WPI_LOGGING_PORT            1741
#define NEW_MESSAGE_WAIT_TIMEOUT_MS 50

static void logServerProcess() {

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
		queueNewDataCondition.wait_for(lck, std::chrono::milliseconds(NEW_MESSAGE_WAIT_TIMEOUT_MS));
    }

    sock.close();  // Internally, this closes the serverSocket
}

static void enqueuePrintMessage(char *msg) {
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

#define STDOUT_CAPTURE_TIMEOUT_MS 50
#define MAX_STDOUT_LINE_LEN 32768

static void stdoutCaptureProcess() {

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

void mau::LoggerComms::start() {

	mem_allocator = new BuddyPool(LOG_BUFFER_SIZE);

    if (!isRunning) {
        Toast::Net::Socket::socket_init();

        runLock.lock();
        isRunning = true;
        printf("Log Server Running...\n");
        runLock.unlock();

        tcpThread = std::thread(logServerProcess);
        tcpThread.detach();

        stdoutCaptureThread = std::thread(stdoutCaptureProcess);
        stdoutCaptureThread.detach();
    }
}

void mau::LoggerComms::stop() {
    runLock.lock();
    isRunning = false;
    runLock.unlock();
}

void mau::LoggerComms::enqueueErrorMessage(uint16_t num_occur, int32_t errorCode, uint8_t flags, const char *details, const char *location, const char *callStack) {
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

