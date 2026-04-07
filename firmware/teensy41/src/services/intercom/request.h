#pragma once
#include "services/service.h"

class Request;

using messageCallback_ptr = void (*)(const String&);
using requestCallback_ptr = void (*)(Request&);

class Intercom;

// ── BridgeSource ──────────────────────────────────────────────────────────────
enum class BridgeSource : uint8_t {
    INTERCOM = 0,
    USB      = 1,
};

class Request {
public:
    // Increased to 384 to accommodate pathfinding commands with multiple
    // via() waypoints (e.g. "via(1234.5,1234.5);via(...)...;go(1234.5,1234.5)").
    // Previous value (128) caused silent truncation and garbled commands.
    static constexpr uint16_t CONTENT_MAX = 384;
    static constexpr uint16_t PAYLOAD_MAX = 400;  // CONTENT_MAX + 16 overhead for "uid:content|crc"

    enum class Status {
        IDLE,
        SENT,
        TIMEOUT,
        OK,
        CLOSED,
        TO_ANSWER,
        ERROR
    };

    /// Default constructor (for pool pre-allocation in Intercom)
    Request();

    /// Receive-side constructor (used by Intercom parser & JetsonBridge)
    Request(int id, const char* content,
            BridgeSource src = BridgeSource::INTERCOM);

    /// Send-side constructor (used internally to send requests outward)
    Request(const char* payload, long timeout = 0,
            requestCallback_ptr callback = nullptr,
            callback_ptr timeout_callback = nullptr);

    void setTimeoutCallback(callback_ptr func);
    void setCallback(requestCallback_ptr func);

    void send();
    void reply(const char* answer);
    void close();
    void setStatus(Status status);

    /// PR-1: Flush any queued reply when TX buffer was full. Called by JetsonBridge::run().
    static bool flushPendingReply();

    void onTimeout();
    void onResponse(const char* response);

    bool isTimedOut() const;

    int          ID()      const;
    Status       getStatus() const;
    BridgeSource source()  const { return m_source; }

    const char* getContent()  const { return _content; }
    const char* getResponse() const { return _response; }
    const char* getPayload()  const { return _payload; }

    unsigned long getTimeout()      const;
    unsigned long getResponseTime() const;
    unsigned long getLastSent()     const;

private:
    void _buildPayload();  ///< Rebuild _payload from current _uid + _content

    int           _uid;
    BridgeSource  m_source;
    char          _content [CONTENT_MAX];
    char          _response[CONTENT_MAX];
    char          _payload [PAYLOAD_MAX];  ///< Pre-built "uid:content|crc" ready to send
    uint8_t       _crcVal;
    unsigned long _firstSent    = 0;
    unsigned long _lastSent     = 0;
    unsigned long _responseTime = 0;
    unsigned long _timeout;
    static int    _uidCounter;
    Status        _status;
    requestCallback_ptr _callback;
    callback_ptr        _timeoutCallback;
};