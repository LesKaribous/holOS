#include "consoleStream.h"
#include "os/console.h"

// FLASHMEM: ConsoleStream is pure logging infrastructure — never time-critical.
// Keeping it out of ITCM (RAM1) frees precious 512KB for ISR and hotpath code.

FLASHMEM ConsoleStream::ConsoleStream(ConsoleLevel lvl, const String& origin, bool forceIgnore) {
	_ignored = forceIgnore || (lvl < Console::getLevel());

	if (!_ignored) {
		if (origin.length() == 0 || origin == "NOT_A_SERVICE")
			Console::write(header(lvl).c_str());
		else
			Console::write((header(lvl) + "(" + origin + "): ").c_str());
	}
}

FLASHMEM String ConsoleStream::header(ConsoleLevel lvl) {
	String str;

	switch (lvl) {
	case ConsoleLevel::VERBOSE:
		str = "[Trace]";
		break;
	case ConsoleLevel::INFO:
		str = "[Info]";
		break;
	case ConsoleLevel::WARNING:
		str = "[Warning]";
		break;
	case ConsoleLevel::CRITICAL:
		str = "[Error]";
		break;
	case ConsoleLevel::SUCCESS:
		str = "[OK]";
		break;
	default:
		str = "";
		break;
	}
	return(str);
}


FLASHMEM ConsoleStream& ConsoleStream::operator<<(short i) {
	if (_ignored) return *this;
	Console::print(i);
	return *this;
}

FLASHMEM ConsoleStream& ConsoleStream::operator<<(int i) {
	if (_ignored) return *this;
	Console::print(i);
	return *this;
}

FLASHMEM ConsoleStream& ConsoleStream::operator<<(long i) {
	if (_ignored) return *this;
	Console::print(i);
	return *this;
}

FLASHMEM ConsoleStream& ConsoleStream::operator<<(size_t i) {
	if (_ignored) return *this;
	Console::print(i);
	return *this;
}

FLASHMEM ConsoleStream& ConsoleStream::operator<<(float i) {
	if (_ignored) return *this;
	Console::print(i);
	return *this;
}

FLASHMEM ConsoleStream& ConsoleStream::operator<<(Vec2 i) {
	if (_ignored) return *this;
	Console::print(String(i));
	return *this;
}

FLASHMEM ConsoleStream& ConsoleStream::operator<<(Vec3 i) {
	if (_ignored) return *this;
	Console::print(String(i));
	return *this;
}

FLASHMEM ConsoleStream& ConsoleStream::operator<<(double i) {
	if (_ignored) return *this;
	Console::print(i);
	return *this;
}

FLASHMEM ConsoleStream& ConsoleStream::operator<<(char i) {
	if (_ignored) return *this;
	Console::print(i);
	return *this;
}

FLASHMEM ConsoleStream& ConsoleStream::operator<<(const String& i) {
	if (_ignored) return *this;
	Console::print(i);
	return *this;
}

FLASHMEM ConsoleStream& ConsoleStream::operator<<(const char* i) {
	if (_ignored) return *this;
	Console::print(i);
	return *this;
}
