
#include <wpi/priority_mutex.h>
#include <wpi/priority_condition_variable.h>

namespace mau {
	namespace log {
        extern wpi::priority_mutex queueMutex;
        extern wpi::priority_condition_variable* queueSignal;

		void start();
		void stop();
		bool periodicUpdate();
	}
}