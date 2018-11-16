#include "MessageQueue.h"
#include <cstring>

wpi::priority_mutex mau::log::MessageQueue::memLock;
wpi::priority_condition_variable mau::log::MessageQueue::memSignal;
std::queue<Message> mau::log::MessageQueue::mesQueue;

void mau::log::MessageQueue::unlockAndSignal() {
    memSignal.notify_all();
    memLock.unlock();
}

void mau::log::MessageQueue::empty(bool* isEmpty) {
	memLock.lock();
	*isEmpty = mesQueue.empty();
	memLock.unlock();
}

void mau::log::MessageQueue::pop(Message* message) {
	memLock.lock();
	Message out = mesQueue.pop();
	message->encode(&out);
	delete out;
	memLock.unlock();
}

void mau::log::MessageQueue::push(Message* message) {
	memLock.lock();
	mesQueue.push(*message);
	unlockAndSignal();
}

std::priority_condition_variable* mau::log::MessageQueue::getPushSignal() {
	return &memSignal;
}
