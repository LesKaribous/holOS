#include "terminal.h"
#include "os/console.h"

SINGLETON_INSTANTIATE(Terminal, terminal)

Terminal::Terminal() : Service(ID_TERMINAL){}

FLASHMEM void Terminal::attach(){
    Console::info() << "Terminal activated" << Console::endl;
}   

FLASHMEM void Terminal::run(){
    if(!enabled()) return;

    // JetsonBridge owns the bridge serial port (auto-detected: USB-CDC or XBee).
    // When bridge is on USB-CDC, it shares Serial with CONSOLE_SERIAL.
    // Terminal must never read from Serial — stealing bytes would corrupt the
    // bridge handshake and trigger a firmware crash.
    // Terminal input is handled by JetsonBridge::handleRequest() via the 'cmd'
    // command over the bridge channel.
    (void)this;  // suppress unused-warning if optimised out
    return;
}   

FLASHMEM String Terminal::dequeCommand(){
    String command = _pendingCommands.front();
    _pendingCommands.pop_front();
    return command;
}

FLASHMEM int Terminal::commandAvailable(){
    return _pendingCommands.size();
}
