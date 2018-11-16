//
// Created by dylanlamarca on 8/7/18.
//

#ifndef NATIVE_RIOLOGTHREAD_H

#include "socket.hpp"

#include "Message.h"
#include "MessageQueue.h"
#include "MauTime.h"
#include "LogComms.h"

#define Mau_kRioLogMessageTimeout 100

namespace mau {
    namespace log {
        extern Message mesBuffer;

        extern std::mutex runLock;
        extern bool isRunning;
		extern std::thread logThread;

        void tcpProcess();

        void tcpProcess() {
            Toast::Net::Socket::ServerSocket sock(1741);
            sock.open();

            Toast::Net::Socket::SocketAddress addr;
            while (true) {
                runLock.lock();
                if (!isRunning)
                    break;
                runLock.unlock();

                bool mesUpdated = mau::log::periodicUpdate();
                if(mesUpdated) {
                    unsigned short len = mesBuffer.getLength();
                    char* bytes = mesBuffer.getBytes();
                    // TODO: figure out what to set port to, IF I need to set it at all
                    // addr.set_port(1150);
                    // sock.send(bytes, len, &addr);
                    sock.send(bytes, len);
                }
            }
            sock.close();
        }
    }
}