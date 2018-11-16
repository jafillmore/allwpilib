#include "Message.h"

#include <queue>
#include <wpi/priority_mutex.h>
#include <wpi/priority_condition_variable.h>

namespace mau {
	namespace log {
		class MessageQueue {
		    static wpi::priority_mutex memLock;
		    static wpi::priority_condition_variable memSignal;
			static std::queue<Message> mesQueue;

			static void unlockAndSignal();
		public:
			static void empty(bool* isEmpty);
			static void pop(Message* message);
			static void push(Message* message);
			static std::priority_condition_variable* getPushSignal();
		};
	}
}