#include "socket.hpp"
#include "LogComms.h"
#include "LogThread.h"
#include "Message.h"
#include "MessageQueue.h"
#include <thread>
#include <wpi/priority_mutex.h>
#include <wpi/priority_condition_variable.h>

mau::log::Message mau::log::mesBuffer;
std::mutex mau::log::runLock;
bool mau::log::isRunning;
std::thread logThread;
wpi::priority_mutex* mau::log::queueMutex;
wpi::priority_condition_variable* mau::log::queueSignal;

void mau::log::start() {
    if (!isRunning) {
        wpi::priority_mutex* newMutex = new wpi::priority_mutex();
        queueMutex = newMutex;
        queueSignal = MessageQueue::getPushSignal();

        Toast::Net::Socket::socket_init();
        runLock.lock();
        isRunning = true;
        printf("RioLog Server Running...\n");
        runLock.unlock();
        logThread = std::thread(tcpProcess);
        logThread.detach();
    }
}

void mau::log::stop() {
    runLock.lock();
    isRunning = false;
    runLock.unlock();
}

bool mau::log::periodicUpdate() {
	std::unique_lock<wpi::priority_mutex> queueLock(*queueMutex);

    std::atomic<bool> expired{false};
    auto waitTime = std::chrono::milliseconds((int)Mau_kRioLogMessageTimeout);

    expired = (bool)queueSignal->wait_for(queueMutex, waitTime);
    if(!expired) {
        MessageQueue::pop(&mesBuffer);
    }
    return expired;
}