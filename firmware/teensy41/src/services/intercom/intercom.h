#pragma once
#include "services/service.h"
#include "request.h"

class Request;

using messageCallback_ptr = void (*)(const String&);
using requestCallback_ptr = void (*)(Request&);

class Intercom : public Service {
public:
    Intercom();

    void attach()  override;
    void run()     override;

    void enable()  override;
    void disable() override;

    void sendMessage(const char* message);
    void sendMessage(const String& message);

    // IC-2: returns slot index (not UID) — use to check status
    int  sendRequest(const char* payload, long timeout = 300,
                     requestCallback_ptr cbfunc = nullptr,
                     callback_ptr func = nullptr);
    bool closeRequest(int uid);
    const char* getRequestResponse(int uid);

    void setConnectLostCallback(callback_ptr callback);
    void setRequestCallback(requestCallback_ptr callback);
    void setConnectionSuccessCallback(callback_ptr callback);

    bool isConnected();

private:

    void pingReceived();
    void connectionLost();
    void connectionSuccess();

    bool debug = false;

    Stream& _stream;

    // IC-2: Fixed-size request slots — zero heap allocation
    static constexpr uint8_t MAX_PENDING = 8;
    Request  _pool   [MAX_PENDING];   // pre-allocated storage
    bool     _poolActive[MAX_PENDING];

    requestCallback_ptr onRequestCallback    = nullptr;
    callback_ptr        onConnectionLost     = nullptr;
    callback_ptr        onConnectionSuccess  = nullptr;

    unsigned long _lastStream = 0;
    unsigned long _lastPing   = 0;
    bool          _connected  = false;

    void _processIncomingData();
    void _processPendingRequests();

    // IC-1: Non-blocking char accumulator
    char          _inBuf[512];
    uint16_t      _inBufLen     = 0;
    unsigned long _inBufStartMs = 0;

    SINGLETON(Intercom)
};
SINGLETON_EXTERN(Intercom, intercom)
