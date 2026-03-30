
#pragma once
#include "intercom.h"

class Request;

using messageCallback_ptr = void (*)(const String&);
using requestCallback_ptr = void (*)(Request&);

class Intercom;

// ── BridgeSource ──────────────────────────────────────────────────────────────
// Identifies which physical channel a Request arrived on.
// Set at construction time so Request::reply() can route back correctly
// without any compile-time #ifdef USB_DIRECT.
enum class BridgeSource : uint8_t {
    INTERCOM = 0,   ///< XBee / Jetson path — reply via Intercom (Serial1)
    USB      = 1,   ///< USB-CDC direct path — reply via BRIDGE_SERIAL
};

class Request {
public:
    enum class Status{
        IDLE,
        SENT,
        TIMEOUT,
        OK,
        CLOSED,
        TO_ANSWER,
        ERROR
    };

    /// Receive-side constructor (used by Intercom parser & JetsonBridge)
    Request(int id, const String& content,
            BridgeSource src = BridgeSource::INTERCOM);

    /// Send-side constructor (used internally to send requests outward)
    Request(const String& payload, long timeout = 0,
            requestCallback_ptr callback = nullptr,
            callback_ptr timeout_callback = nullptr);

    void setTimeoutCallback(callback_ptr func);
    void setCallback(requestCallback_ptr func);

    void send();
    void reply(const String& answer);
    void close();
    void setStatus(Status status);
    
    void onTimeout();
    void onResponse(const String& response);
    
    bool isTimedOut() const;

    int          ID()         const;
    Status       getStatus() const;
    BridgeSource source()    const { return m_source; }
    String       getPayload() const;

    const String& getContent()  const;
    const String& getResponse() const;
    
    unsigned long getTimeout() const;
    unsigned long getResponseTime() const;
    unsigned long getLastSent() const;
    
private:
    int          _uid;
    BridgeSource m_source;
    String       _prefix;
    String       _crc;
    String       _content;
    String       _response;
    unsigned long _firstSent = 0;
    unsigned long _lastSent;
    unsigned long _responseTime;
    unsigned long _timeout;
    static int _uidCounter;
    Status _status;
    requestCallback_ptr _callback;
    callback_ptr _timeoutCallback;
};