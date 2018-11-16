#include "Message.h"
#include <string>

using mau::log;

// ----- private ----- //
template<typename T>
void Message::encode(T var, int byteIndex) {
	char* bVar = (char *)var;
	for (std::size_t count = 0; count != sizeof(T); count++) {
		bytes[count + byteIndex] = bVar[count];
	}
}

void Message::encode(std::string line, int byteIndex) {
	for (size_t count = 0; count != line.size(); count++) {
		bytes[count + byteIndex] = line.at(count);
	}
}

Message::encodeUniversal(float timestamp, short seqNum) {
	// 0-1: Length
	encode<unsigned short>(length, 0);
	// 3-6: timestamp
	encode<float>(timestamp, 3);
	// 7-8: seqNumber
	encode<short>(seqNum, 7);
}

// ----- Constructor/Destructor ----- //
Message::Message(float timestamp, short seqNum, PrintInfo* pInfo) {
	length = Mau_kStaticPrintMessageLength + pInfo->line.size();
	bytes = new char[length];
	encodeUniversal(timestamp, seqNum);
	// 2: tag (PrintMessage == 12)
	encode<char>(12, 2);
	// 9-End: String
	encode(pInfo->line, 9);
}

Message::Message(float timestamp, short seqNum, ErrorInfo* eInfo) {
	short detLen = eInfo->det.size();
	short locLen = eInfo->loc.size();
	short stackLen = eInfo->stack.size();
	length = Mau_kStaticErrorMessageLength + detLen + locLen + stackLen;
	bytes = new char[length];
	encodeUniversal(timestamp, seqNum);
	// 2: tag (ErrorMessage == 11)
	encode<unsigned char>(11, 2);
	// 9-10: numOccur
	encode<short>(eInfo->numOccur, 9);
	// 11-14: Error Code
	encode<int>(eInfo->code, 11);
	// 15: flag
	encode<unsigned char>(eInfo->flag, 15);
	// 16-17: Detail
	encode<short>(detLen, 16);
	encode(eInfo->det, 18);
	// std::string loc;
	encode<short>(locLen, 19 + detLen);
	encode(eInfo->loc, 21 + detLen);
	// std::string stack;
	encode<short>(stackLen, 19 + detLen + locLen);
	encode(eInfo->stack, 21 + detLen + locLen);
}

Message::~Message() {
	delete[] bytes;
}

// ----- Get ----- //
unsigned char Message::getByte(int index) {
	return bytes[index];
}

unsigned char* Message::getBytes() {
	return bytes;
}

unsigned short Message::getLength() {
	return length;
}

void Message::encode(Message* message) {
	delete[] bytes;
	length = message->getLength();
	bytes = new char[length];
	for(unsigned short count = 0; count < length; count++) {
		bytes[count] = message->getByte(count);
	}
}