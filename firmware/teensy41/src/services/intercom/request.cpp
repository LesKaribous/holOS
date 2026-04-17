#include "request.h"
#include "intercom.h"
#include "comUtilities.h"
#include "config/settings.h"
#include <string.h>
#include <stdio.h>

int Request::_uidCounter = 0;

// PR-1: Single-slot pending reply buffer — prevents silent reply drop on full TX buffer
static char s_pendingReply[Request::PAYLOAD_MAX + 4] = {};
static bool s_hasPendingReply = false;

// ─────────────────────────────────────────────────────────────────────────────
//  Constructors
// ─────────────────────────────────────────────────────────────────────────────

/// Default constructor (for pool pre-allocation in Intercom::_pool[MAX_PENDING])
Request::Request()
    : _uid(-1),
      m_source(BridgeSource::INTERCOM),
      _crcVal(0),
      _firstSent(0),
      _lastSent(0),
      _responseTime(0),
      _timeout(0),
      _status(Status::IDLE),
      _callback(nullptr),
      _timeoutCallback(nullptr)
{
    _content[0]  = '\0';
    _response[0] = '\0';
    _payload[0]  = '\0';
}

/// Receive-side: firmware received this request, needs to reply
Request::Request(int id, const char* content, BridgeSource src)
    : _uid(id),
      m_source(src),
      _crcVal(0),
      _firstSent(0),
      _lastSent(0),
      _responseTime(0),
      _timeout(0),
      _status(Status::IDLE),
      _callback(nullptr),
      _timeoutCallback(nullptr)
{
    strncpy(_content,  content, CONTENT_MAX - 1);  _content[CONTENT_MAX - 1]  = '\0';
    _response[0] = '\0';
    // Reply payload built in reply() — not needed at construction
    _payload[0] = '\0';
}

/// Send-side: firmware wants to send a request outward (to T4.0 via Intercom)
Request::Request(const char* content, long timeout,
                 requestCallback_ptr func_call, callback_ptr timeout_call)
    : _uid(0),
      m_source(BridgeSource::INTERCOM),
      _crcVal(0),
      _firstSent(0),
      _lastSent(0),
      _responseTime(0),
      _timeout((unsigned long)timeout),
      _status(Status::IDLE),
      _callback(func_call),
      _timeoutCallback(timeout_call)
{
    _uid = _uidCounter++;
    strncpy(_content,  content, CONTENT_MAX - 1);  _content[CONTENT_MAX - 1]  = '\0';
    _response[0] = '\0';
    _buildPayload();
}

// ─────────────────────────────────────────────────────────────────────────────
//  _buildPayload — build "uid:content|crc" into _payload
// ─────────────────────────────────────────────────────────────────────────────
void Request::_buildPayload() {
    // Temporary buffer to compute CRC over "uid:content"
    char tmp[PAYLOAD_MAX];
    int n = snprintf(tmp, sizeof(tmp), "%d:%s", _uid, _content);
    _crcVal = CRC8.smbus((uint8_t*)tmp, (size_t)n);
    snprintf(_payload, sizeof(_payload), "%s|%d", tmp, (int)_crcVal);
}

// ─────────────────────────────────────────────────────────────────────────────
//  PR-1 — flush pending reply
// ─────────────────────────────────────────────────────────────────────────────
bool Request::flushPendingReply() {
    if (!s_hasPendingReply) return false;
    int len = (int)strlen(s_pendingReply);
    if (BRIDGE_SERIAL.availableForWrite() >= len + 1) {
        BRIDGE_SERIAL.print(s_pendingReply);
        BRIDGE_SERIAL.write('\n');
        s_hasPendingReply = false;
        return true;
    }
    // Frame too large for the TX buffer — write blocking to avoid
    // the reply being stuck forever (same issue as ring buffer drain).
    if (len + 1 > 64) {
        BRIDGE_SERIAL.print(s_pendingReply);
        BRIDGE_SERIAL.write('\n');
        s_hasPendingReply = false;
        return true;
    }
    return false;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Callbacks
// ─────────────────────────────────────────────────────────────────────────────
void Request::setTimeoutCallback(callback_ptr func)      { _timeoutCallback = func; }
void Request::setCallback(requestCallback_ptr func)       { _callback = func; }

// ─────────────────────────────────────────────────────────────────────────────
//  send — transmit this request via Intercom
// ─────────────────────────────────────────────────────────────────────────────
void Request::send() {
    _status   = Status::SENT;
    _lastSent = millis();
    if (_firstSent == 0) _firstSent = millis();
    Intercom::instance().sendMessage(_payload);
}

// ─────────────────────────────────────────────────────────────────────────────
//  reply — send reply back to the originator of this request
// ─────────────────────────────────────────────────────────────────────────────
void Request::reply(const char* answer) {
    _status   = Status::CLOSED;
    _lastSent = millis();
    strncpy(_content, answer, CONTENT_MAX - 1);  _content[CONTENT_MAX - 1] = '\0';

    // Build reply payload: "rUID:answer|crc"
    char tmp[PAYLOAD_MAX];
    int n = snprintf(tmp, sizeof(tmp), "r%d:%s", _uid, _content);
    uint8_t crc = CRC8.smbus((uint8_t*)tmp, (size_t)n);
    snprintf(_payload, sizeof(_payload), "%s|%d", tmp, (int)crc);

    if (m_source == BridgeSource::USB) {
        int needed = (int)strlen(_payload) + 1;
        if (BRIDGE_SERIAL.availableForWrite() >= needed) {
            BRIDGE_SERIAL.print(_payload);
            BRIDGE_SERIAL.write('\n');
        } else if (needed > 64) {
            // Frame exceeds HardwareSerial TX buffer capacity (e.g. calib
            // reports ~80+ bytes on Serial2/XBee).  Write blocking —
            // Serial.write() handles chunked TX internally.  Without this,
            // the reply would be queued in s_pendingReply whose flush has
            // the same availableForWrite() gate and would never drain.
            BRIDGE_SERIAL.print(_payload);
            BRIDGE_SERIAL.write('\n');
        } else {
            // PR-1: queue for retry — never silently drop a reply
            strncpy(s_pendingReply, _payload, sizeof(s_pendingReply) - 1);
            s_pendingReply[sizeof(s_pendingReply) - 1] = '\0';
            s_hasPendingReply = true;
        }
    } else {
        Intercom::instance().sendMessage(_payload);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  close / status
// ─────────────────────────────────────────────────────────────────────────────
void Request::close()                  { _status = Status::CLOSED; }
void Request::setStatus(Status status) { _status = status; }

void Request::onResponse(const char* response) {
    _status       = Status::OK;
    _responseTime = millis() - _lastSent;
    strncpy(_response, response, CONTENT_MAX - 1);  _response[CONTENT_MAX - 1] = '\0';
    if (_callback) _callback(*this);
}

void Request::onTimeout() {
    _status = Status::TIMEOUT;
    if (_timeoutCallback) _timeoutCallback();
}

// ─────────────────────────────────────────────────────────────────────────────
//  Accessors
// ─────────────────────────────────────────────────────────────────────────────
int           Request::ID()              const { return _uid; }
Request::Status Request::getStatus()     const { return _status; }
unsigned long Request::getTimeout()      const { return _timeout; }
unsigned long Request::getResponseTime() const { return _responseTime; }
unsigned long Request::getLastSent()     const { return _lastSent; }

bool Request::isTimedOut() const {
    return (_timeout > 0) && (millis() - _firstSent >= _timeout);
}
