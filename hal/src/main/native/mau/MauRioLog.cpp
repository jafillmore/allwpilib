#include <string>
#include <VMXErrors.h>
#include "MauRioLog.h"
#include "MauClockInternal.h"
#include "MessageQueue.h"
#include "Message.h"

namespace hal {
	namespace init {
		void InitializeMauRioLog() {

		}
	}
}

using mau::log;

void Mau_log(std::string str) {
	PrintInfo info;
	info.line.assign(str);
	float time = Mau_getTimestamp();
	short seqNum = 0; // <- I don't really know what the hell that does yet :/
	Message print = new Message(time, 0, &info);
	MessageQueue::push(&print);
}

void Mau_logError(VMXErrorCode er) {
	ErrorInfo info;
	info.code = (int)er;
	info.det.assign(GetVMXErrorString(er));
	float time = Mau_getTimestamp();
	short seqNum = 0; // <- I don't really know what the hell that does yet :/
	Message error = new Message(time, 0, &info);
	MessageQueue::push(&error);
}