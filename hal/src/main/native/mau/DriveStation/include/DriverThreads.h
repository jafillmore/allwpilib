//
// Created by dylanlamarca on 8/7/18.
//

#ifndef NATIVE_DRIVERSTATIONTHREAD_H

#include "socket.hpp"

#include "DriverComms.hpp"
#include "MauTime.h"
#include "MauDriveData.h"
#include <wpi/priority_mutex.h>

#include <thread>

#define NATIVE_DRIVERSTATIONTHREAD_H

namespace mau {
    namespace comms {
        extern char encode_buffer[8];
        extern char decode_buffer[1024];
        extern char tcp_buffer[1024];
        
        extern std::mutex runLock;
        extern bool isRunning;
        extern std::thread udpThread;
		extern std::thread tcpThread;

        void tcpProcess();
        void udpProcess();

        void tcpProcess() {
            Toast::Net::Socket::ServerSocket sock(1740);
            sock.open();

            while (true) {
                Toast::Net::Socket::ClientSocket client = sock.accept();
                int ret = 1;

                while (ret > 0) {
                    ret = client.read(tcp_buffer, 8192);

                    if (ret > 0) {
                        mau::comms::decodeTcpPacket(tcp_buffer, ret);
                    }
                }
                client.close();
                sleep(100);
            }

            sock.close();
        }

        void udpProcess() {
            Toast::Net::Socket::DatagramSocket sock(1110);
            sock.bind();

            Toast::Net::Socket::SocketAddress addr;
            while (true) {
                runLock.lock();
                if (!isRunning)
                    break;
                runLock.unlock();

                int len = sock.read(decode_buffer, 1024, &addr);
                mau::comms::decodeUdpPacket(decode_buffer, len);

                mau::comms::encodePacket(encode_buffer);
                addr.set_port(1150);
                sock.send(encode_buffer, 8, &addr);
            }
            sock.close();
        }
    }
}
#endif //NATIVE_DRIVERSTATIONTHREAD_H
