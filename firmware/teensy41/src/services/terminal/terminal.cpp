#include "terminal.h"
#include "os/console.h"

SINGLETON_INSTANTIATE(Terminal, terminal)

Terminal::Terminal() : Service(ID_TERMINAL){}

void Terminal::attach(){
    Console::info() << "Terminal activated" << Console::endl;
}   

void Terminal::run(){
    if(!enabled()) return;

    // CONSOLE_SERIAL and BRIDGE_SERIAL are both Serial (USB-CDC).
    // JetsonBridge owns the port exclusively for ping/pong + framed commands.
    // Terminal must never read from it — stealing bytes would corrupt the bridge
    // handshake and trigger a firmware crash.
    // Terminal input is handled by JetsonBridge::handleRequest() via the 'cmd'
    // command over the same USB channel.
    (void)this;  // suppress unused-warning if optimised out
    return;
}   

String Terminal::dequeCommand(){
    String command = _pendingCommands.front();
    _pendingCommands.pop_front();
    return command;
}

int Terminal::commandAvailable(){
    return _pendingCommands.size();
}
