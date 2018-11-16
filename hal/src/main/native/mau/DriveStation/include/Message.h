#include <string>

#define Mau_kStaticPrintMessageLength 9
#define Mau_kStaticErrorMessageLength 22

namespace mau {
	namespace log {
		struct PrintInfo {
			std::string line;
		}

		struct ErrorInfo {
			short numOccur;
			int code;
			char flag;
			std::string det;
			std::string loc;
			std::string stack;
		}

		class Message {
			char* bytes;
			unsigned short length;

			template<typename T>
			void encode(T var, int byteIndex);
			void encodeString(std::string line, int byteIndex);
			void encodeUniversal(float timestamp, short seqNum);
		public:
			Message(float timestamp, short seqNum, PrintInfo* pInfo);
			Message(float timestamp, short seqNum, ErrorInfo* eInfo);
			~Message();

			char getByte(int index);
			char* getBytes();
			unsigned short getLength();
			void encode(Message* message);
		};
	}
}