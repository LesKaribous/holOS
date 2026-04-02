#include "console.h"
#include <Arduino.h>

const String Console::endl = "\n";
ConsoleLevel Console::m_level      = ConsoleLevel::INFO;
uint32_t     Console::m_sourceMask = 0xFFFFFFFFu;  // all enabled until init() applies settings

// ── ServiceID stream factories — check source mask, use readable name ─────────
// FLASHMEM: logging is never time-critical; keeping these out of ITCM saves RAM1.

FLASHMEM ConsoleStream Console::trace(const ServiceID& origin) {
	bool filtered = (origin != ID_NOT_A_SERVICE) && !isSourceEnabled(origin);
	return ConsoleStream(ConsoleLevel::VERBOSE, Service::toString(origin), filtered);
}

FLASHMEM ConsoleStream Console::info(const ServiceID& origin) {
	bool filtered = (origin != ID_NOT_A_SERVICE) && !isSourceEnabled(origin);
	return ConsoleStream(ConsoleLevel::INFO, Service::toString(origin), filtered);
}

FLASHMEM ConsoleStream Console::warn(const ServiceID& origin) {
	bool filtered = (origin != ID_NOT_A_SERVICE) && !isSourceEnabled(origin);
	return ConsoleStream(ConsoleLevel::WARNING, Service::toString(origin), filtered);
}

FLASHMEM ConsoleStream Console::error(const ServiceID& origin) {
	bool filtered = (origin != ID_NOT_A_SERVICE) && !isSourceEnabled(origin);
	return ConsoleStream(ConsoleLevel::CRITICAL, Service::toString(origin), filtered);
}

FLASHMEM ConsoleStream Console::success(const ServiceID& origin) {
	bool filtered = (origin != ID_NOT_A_SERVICE) && !isSourceEnabled(origin);
	return ConsoleStream(ConsoleLevel::SUCCESS, Service::toString(origin), filtered);
}

// ── String-origin stream factories ────────────────────────────────────────────

FLASHMEM ConsoleStream Console::info(const String &origin)
{
    return ConsoleStream(ConsoleLevel::INFO, origin);
}

FLASHMEM ConsoleStream Console::warn(const String& origin) {
	return ConsoleStream(ConsoleLevel::WARNING, origin);
}

FLASHMEM ConsoleStream Console::error(const String& origin) {
	return ConsoleStream(ConsoleLevel::CRITICAL, origin);
}

FLASHMEM ConsoleStream Console::trace(const String& origin) {
	return ConsoleStream(ConsoleLevel::VERBOSE, origin);
}

FLASHMEM ConsoleStream Console::success(const String& origin) {
	return ConsoleStream(ConsoleLevel::SUCCESS, origin);
}

// ── Timestamps ────────────────────────────────────────────────────────────────

FLASHMEM String Console::timeStamp(){
	String time = String(millis());
	return String("[t=" + time + "ms]");
}

FLASHMEM String Console::microTimeStamp(){
	String time = String(micros());
	return String("[t=" + time + "us]");
}

// ── Boot header ───────────────────────────────────────────────────────────────

static FLASHMEM void printHeader(){
	CONSOLE_SERIAL.println("");
	CONSOLE_SERIAL.println("  _______       _                     _"                        );
	CONSOLE_SERIAL.println(" |__   __|     (_)                   | |"                       );
	CONSOLE_SERIAL.println("    | |_      ___ _ __  ___ _   _ ___| |_ ___ _ __ ___  "       );
	CONSOLE_SERIAL.println("    | \\ \\ /\\ / / | '_ \\/ __| | | / __| __/ _ \\ '_ ` _ \\"  );
	CONSOLE_SERIAL.println("    | |\\ V  V /| | | | \\__ \\ |_| \\__ \\ ||  __/ | | | | |"  );
	CONSOLE_SERIAL.println("    |_| \\_/\\_/ |_|_| |_|___/\\__, |___/\\__\\___|_| |_| |_|"  );
	CONSOLE_SERIAL.println("                             __/ |"                             );
	CONSOLE_SERIAL.println("                            |___/"                              );
	CONSOLE_SERIAL.println();
	CONSOLE_SERIAL.println("Author  : Nadarbreicq, JulesTopart ");
	CONSOLE_SERIAL.println();
	CONSOLE_SERIAL.print("Twinsystem... compiled  ");
	CONSOLE_SERIAL.print(__DATE__);
	CONSOLE_SERIAL.print(" at ");
	CONSOLE_SERIAL.println(__TIME__);
}

// ── Lifecycle ─────────────────────────────────────────────────────────────────

FLASHMEM void Console::init(){
	CONSOLE_SERIAL.begin(CONSOLE_BAUDRATE);

	// Apply boot defaults from Settings::Log
	m_level = Settings::Log::BOOT_LEVEL;

	// Start with all sources enabled, then clear disabled ones
	m_sourceMask = 0xFFFFFFFFu;
	if (!Settings::Log::SRC_LIDAR)        disableSource(ID_LIDAR);
	if (!Settings::Log::SRC_CHRONO)       disableSource(ID_CHRONO);
	if (!Settings::Log::SRC_IHM)          disableSource(ID_IHM);
	if (!Settings::Log::SRC_SAFETY)       disableSource(ID_SAFETY);
	if (!Settings::Log::SRC_MOTION)       disableSource(ID_MOTION);
	if (!Settings::Log::SRC_NAVIGATION)   disableSource(ID_NAVIGATION);
	if (!Settings::Log::SRC_NEOPIXEL)     disableSource(ID_NEOPIXEL);
	if (!Settings::Log::SRC_INTERCOM)     disableSource(ID_INTERCOM);
	if (!Settings::Log::SRC_TERMINAL)     disableSource(ID_TERMINAL);
	if (!Settings::Log::SRC_ACTUATORS)    disableSource(ID_ACTUATORS);
	if (!Settings::Log::SRC_LOCALISATION) disableSource(ID_LOCALISATION);
	if (!Settings::Log::SRC_VISION)       disableSource(ID_VISION);
	if (!Settings::Log::SRC_JETSON)       disableSource(ID_JETSON);

	printHeader();
}

// ── Raw output ────────────────────────────────────────────────────────────────

void Console::write(const char* str) {
	CONSOLE_SERIAL.write(str);
}

FLASHMEM void Console::plot(const String& n, String s){
	print(">" + n + ":"); println(s);
}

FLASHMEM void Console::plotXY(const String& n, String x, String y){
	println(">" + n + ":" + x + ":" + y + "|xy");
}

void Console::print(const String& s){
	CONSOLE_SERIAL.print(s);
}

void Console::println(const String& s){
	CONSOLE_SERIAL.println(s);
}

FLASHMEM void Console::prettyPrint(const String& s){
	int l = 0;
	line();
	CONSOLE_SERIAL.print(l);
	CONSOLE_SERIAL.print(":\t");
	for (size_t i = 0; i <= s.length(); i++){
		if(s[i] == '\n'){
			CONSOLE_SERIAL.println();
			CONSOLE_SERIAL.print(++l);
			CONSOLE_SERIAL.print(":\t");
		}else{
			CONSOLE_SERIAL.print(s[i]);
		}
	}
	CONSOLE_SERIAL.println(s[s.length() - 1]);
	line();
}

FLASHMEM void Console::line(){
	println("_________________________________________");
}
