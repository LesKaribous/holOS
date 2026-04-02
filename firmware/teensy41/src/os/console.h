#pragma once
#include "config/settings.h"
#include "services/service.h"
#include "debug/consoleStream.h"

#define HERE " [" + String(__FILE__) + " at line " + String(__LINE__) + "]"
#define THROW(x) Console::println( "Throw in " + String(__FILE__) + " at line " + String(__LINE__) + " : " + x);
#define BEEP Console::println( "Beep in " + String(__FILE__) + " at line " + String(__LINE__));

class ConsoleStream;

class Console{
public:
	friend class ConsoleStream;
	static const String endl;

	// ── Log level ─────────────────────────────────────────────────────────────
	static inline ConsoleLevel getLevel() { return m_level; };
	static inline void setLevel(ConsoleLevel l) { m_level = l; };

	// ── Per-source filter (bitmask, one bit per ServiceID) ────────────────────
	// Sources enabled by default are set from Settings::Log at init().
	// Control at runtime:  Console::enableSource(ID_MOTION)
	//                      Console::disableSource(ID_MOTION)
	//                      Console::enableAllSources()
	//                      Console::setSourceMask(0xFFFF)
	static inline bool     isSourceEnabled(ServiceID id) { return (m_sourceMask >> static_cast<uint8_t>(id)) & 1u; }
	static inline void     enableSource(ServiceID id)    { m_sourceMask |=  (1u << static_cast<uint8_t>(id)); }
	static inline void     disableSource(ServiceID id)   { m_sourceMask &= ~(1u << static_cast<uint8_t>(id)); }
	static inline void     enableAllSources()            { m_sourceMask = 0xFFFFFFFFu; }
	static inline uint32_t getSourceMask()               { return m_sourceMask; }
	static inline void     setSourceMask(uint32_t mask)  { m_sourceMask = mask; }

	// ── Lifecycle ─────────────────────────────────────────────────────────────
	static void init();  // applies Settings::Log defaults

	// ── Timestamps ───────────────────────────────────────────────────────────
	static String timeStamp();
	static String microTimeStamp();

	// ── Stream factories — String origin (strategy/routine code) ─────────────
	// String-origin messages are NOT filtered by the source mask (only by level).
	// These are typically one-off informational messages, not service spam.
	static ConsoleStream info(const String& origin);
	static ConsoleStream warn(const String& origin);
	static ConsoleStream error(const String& origin);
	static ConsoleStream trace(const String& origin);
	static ConsoleStream success(const String& origin);

	// ── Stream factories — ServiceID origin (service classes) ─────────────────
	// ServiceID-origin messages ARE filtered by the source mask.
	// Produces human-readable tag: [Info](MOTION): instead of [Info](4):
	static ConsoleStream info(const ServiceID& origin = ID_NOT_A_SERVICE);
	static ConsoleStream warn(const ServiceID& origin = ID_NOT_A_SERVICE);
	static ConsoleStream error(const ServiceID& origin = ID_NOT_A_SERVICE);
	static ConsoleStream trace(const ServiceID& origin = ID_NOT_A_SERVICE);
	static ConsoleStream success(const ServiceID& origin = ID_NOT_A_SERVICE);

	// ── Plotting (Serial Studio / Teleplot) ───────────────────────────────────
	static void plot(const String& name, String s);
	static void plotXY(const String& n, String x, String y);

	// ── Raw output ────────────────────────────────────────────────────────────
	static void print(const String& s = "");
	static void println(const String& s = "");
	static void prettyPrint(const String& s);
	static void line();

private:
	static void write(const char* str);
 	static ConsoleLevel m_level;
	static uint32_t     m_sourceMask;  ///< bit N set = ServiceID(N) output enabled
};
