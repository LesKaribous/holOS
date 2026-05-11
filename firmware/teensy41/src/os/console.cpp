#include "console.h"
#include <Arduino.h>

const String Console::endl = "\n";
ConsoleLevel Console::m_level      = ConsoleLevel::INFO;
uint32_t     Console::m_sourceMask = 0xFFFFFFFFu;  // all enabled until init() applies settings

// Tracks whether the next write starts a new line — if so we inject
// CONSOLE_DEBUG_PREFIX so holOS's transport filter drops the line.
static bool s_atLineStart = true;

static inline void _writeDebugPrefixIfNeeded() {
    if (s_atLineStart) {
        CONSOLE_SERIAL.write(CONSOLE_DEBUG_PREFIX);
        s_atLineStart = false;
    }
}

static inline void _trackLineEnd(const char* s, size_t n) {
    if (n == 0 || s == nullptr) return;
    s_atLineStart = (s[n - 1] == '\n');
}

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
	Console::println("");
	Console::println("  _______       _                     _"                        );
	Console::println(" |__   __|     (_)                   | |"                       );
	Console::println("    | |_      ___ _ __  ___ _   _ ___| |_ ___ _ __ ___  "       );
	Console::println("    | \\ \\ /\\ / / | '_ \\/ __| | | / __| __/ _ \\ '_ ` _ \\"  );
	Console::println("    | |\\ V  V /| | | | \\__ \\ |_| \\__ \\ ||  __/ | | | | |"  );
	Console::println("    |_| \\_/\\_/ |_|_| |_|___/\\__, |___/\\__\\___|_| |_| |_|"  );
	Console::println("                             __/ |"                             );
	Console::println("                            |___/"                              );
	Console::println();
	Console::println("Author  : Nadarbreicq, JulesTopart ");
	Console::println();
	Console::print("Twinsystem... compiled  ");
	Console::print(__DATE__);
	Console::print(" at ");
	Console::println(__TIME__);
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
//
// Every write goes through these wrappers, which inject CONSOLE_DEBUG_PREFIX
// at each new line. holOS's transport drops `#`-prefixed lines before
// parse_frame() runs.

void Console::write(const char* str) {
	if (!str || !*str) return;
	_writeDebugPrefixIfNeeded();
	CONSOLE_SERIAL.write(str);
	size_t n = strlen(str);
	_trackLineEnd(str, n);
}

FLASHMEM void Console::plot(const String& n, String s){
	print(">" + n + ":"); println(s);
}

FLASHMEM void Console::plotXY(const String& n, String x, String y){
	println(">" + n + ":" + x + ":" + y + "|xy");
}

void Console::print(const String& s){
	if (s.length() == 0) return;
	_writeDebugPrefixIfNeeded();
	CONSOLE_SERIAL.print(s);
	_trackLineEnd(s.c_str(), s.length());
}

void Console::println(const String& s){
	_writeDebugPrefixIfNeeded();
	CONSOLE_SERIAL.println(s);
	s_atLineStart = true;
}

FLASHMEM void Console::prettyPrint(const String& s){
	int l = 0;
	line();
	print(String(l) + ":\t");
	for (size_t i = 0; i <= s.length(); i++){
		if(s[i] == '\n'){
			println();
			print(String(++l) + ":\t");
		}else{
			print(String(s[i]));
		}
	}
	println(String(s[s.length() - 1]));
	line();
}

FLASHMEM void Console::line(){
	println("_________________________________________");
}
