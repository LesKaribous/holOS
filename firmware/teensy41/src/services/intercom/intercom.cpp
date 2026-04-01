#include "intercom.h"
#include "os/console.h"
#include "config/settings.h"
#include "comUtilities.h"
#include <string.h>

SINGLETON_INSTANTIATE(Intercom, intercom)

Intercom::Intercom() : Service(ID_INTERCOM), _stream(INTERCOM_SERIAL) {
    memset(_poolActive, 0, sizeof(_poolActive));
    // Placement-initialise pool entries so they are valid objects
    // (they are default-constructed by the compiler — no extra action needed)
}

// ─────────────────────────────────────────────────────────────────────────────
//  Lifecycle
// ─────────────────────────────────────────────────────────────────────────────

FLASHMEM void Intercom::attach() {
    Console::info() << "Intercom activated" << Console::endl;
}

FLASHMEM void Intercom::run() {
    if (!enabled()) return;

    if (_stream.available()) {
        _lastStream = millis();
        _processIncomingData();
    }
    _processPendingRequests();

    if (millis() - _lastPing > 500 &&
        (!_connected || (_connected && millis() - _lastStream > 1000))) {
        sendMessage("ping");
        _lastPing = millis();
    }
    if (_connected && millis() - _lastStream > 2000) {
        connectionLost();
    }
}

FLASHMEM void Intercom::enable() {
    Service::enable();
    INTERCOM_SERIAL.begin(INTERCOM_BAUDRATE);
    delay(10);
    while (_stream.available()) _stream.read();
    _inBufLen = 0;
    sendMessage("ping");
}

FLASHMEM void Intercom::disable() {
    INTERCOM_SERIAL.end();
    Service::disable();
}

// ─────────────────────────────────────────────────────────────────────────────
//  sendMessage — non-blocking, drop if TX full
// ─────────────────────────────────────────────────────────────────────────────

FLASHMEM void Intercom::sendMessage(const char* message) {
    if (!_stream.availableForWrite()) return;
    _stream.print(message);
    _stream.write('\n');
    if (debug) Console::trace("Intercom") << ">" << message << Console::endl;
}

FLASHMEM void Intercom::sendMessage(const String& message) {
    sendMessage(message.c_str());
}

// ─────────────────────────────────────────────────────────────────────────────
//  sendRequest — IC-2: uses fixed-size pool, no heap alloc
// ─────────────────────────────────────────────────────────────────────────────

FLASHMEM int Intercom::sendRequest(const char* payload, long timeout,
                          requestCallback_ptr cbfunc, callback_ptr func) {
    // Find a free slot
    for (uint8_t i = 0; i < MAX_PENDING; ++i) {
        if (!_poolActive[i]) {
            // Construct Request in-place in the pool
            _pool[i] = Request(payload, timeout, cbfunc, func);
            _pool[i].send();
            _poolActive[i] = true;
            return _pool[i].ID();
        }
    }
    // No free slot — evict oldest (first active slot)
    Console::warn("Intercom") << "Request pool full, evicting oldest" << Console::endl;
    for (uint8_t i = 0; i < MAX_PENDING; ++i) {
        if (_poolActive[i]) {
            _poolActive[i] = false;
            _pool[i] = Request(payload, timeout, cbfunc, func);
            _pool[i].send();
            _poolActive[i] = true;
            return _pool[i].ID();
        }
    }
    return -1;
}

FLASHMEM bool Intercom::closeRequest(int uid) {
    for (uint8_t i = 0; i < MAX_PENDING; ++i) {
        if (_poolActive[i] && _pool[i].ID() == uid) {
            _pool[i].close();
            _poolActive[i] = false;
            return true;
        }
    }
    Console::warn("Intercom") << "closeRequest: UID " << uid << " not found" << Console::endl;
    return false;
}

const char* Intercom::getRequestResponse(int uid) {
    for (uint8_t i = 0; i < MAX_PENDING; ++i) {
        if (_poolActive[i] && _pool[i].ID() == uid) {
            return _pool[i].getResponse();
        }
    }
    Console::warn("Intercom") << "getRequestResponse: UID " << uid << " not found" << Console::endl;
    return "";
}

// ─────────────────────────────────────────────────────────────────────────────
//  Connection management
// ─────────────────────────────────────────────────────────────────────────────

FLASHMEM void Intercom::pingReceived()  { sendMessage("pong"); }
FLASHMEM bool Intercom::isConnected()   { return _connected; }

FLASHMEM void Intercom::connectionLost() {
    Console::warn("Intercom") << "Connection lost." << Console::endl;
    _connected = false;
    if (onConnectionLost) onConnectionLost();
}

FLASHMEM void Intercom::connectionSuccess() {
    Console::info("Intercom") << "Connection successful." << Console::endl;
    _connected = true;
    if (onConnectionSuccess) onConnectionSuccess();
}

FLASHMEM void Intercom::setConnectLostCallback(callback_ptr cb)       { onConnectionLost    = cb; }
FLASHMEM void Intercom::setRequestCallback(requestCallback_ptr cb)    { onRequestCallback   = cb; }
FLASHMEM void Intercom::setConnectionSuccessCallback(callback_ptr cb) { onConnectionSuccess = cb; }

// ─────────────────────────────────────────────────────────────────────────────
//  _processIncomingData — IC-1/IC-4: char array, non-blocking, partial timeout
// ─────────────────────────────────────────────────────────────────────────────
FLASHMEM void Intercom::_processIncomingData() {
    // IC-4: discard partial frames older than 200ms
    if (_inBufLen > 0 && millis() - _inBufStartMs > 200) {
        Console::warn("Intercom") << "Partial frame discarded (" << (int)_inBufLen << " bytes)" << Console::endl;
        _inBufLen = 0;
    }

    while (_stream.available()) {
        char c = (char)_stream.read();

        if (c == '\n' || c == '\r') {
            if (_inBufLen == 0) continue;

            _inBuf[_inBufLen] = '\0';
            // Trim trailing whitespace
            while (_inBufLen > 0 && (_inBuf[_inBufLen - 1] == ' ' || _inBuf[_inBufLen - 1] == '\r')) {
                _inBuf[--_inBufLen] = '\0';
            }
            if (_inBufLen == 0) { _inBufLen = 0; continue; }

            if (strncmp(_inBuf, "ping", 4) == 0) {
                pingReceived();
                _inBufLen = 0;
                continue;
            }
            if (strncmp(_inBuf, "pong", 4) == 0) {
                if (!_connected) { _connected = true; connectionSuccess(); }
                _inBufLen = 0;
                continue;
            }

            char* sep    = strchr (_inBuf, ':');
            char* crcSep = strrchr(_inBuf, '|');

            if (!sep || !crcSep || crcSep <= sep) {
                if (debug) Console::trace("Intercom") << "Unparseable frame" << Console::endl;
                _inBufLen = 0;
                continue;
            }

            // Validate CRC
            int     crcVal   = atoi(crcSep + 1);
            uint8_t computed = CRC8.smbus((uint8_t*)_inBuf, (size_t)(crcSep - _inBuf));
            if ((uint8_t)crcVal != computed) {
                Console::warn("Intercom") << "Bad CRC" << Console::endl;
                _inBufLen = 0;
                continue;
            }

            // Split in-place
            *sep    = '\0';
            *crcSep = '\0';

            if (_inBuf[0] == 'r') {
                // Reply: r{uid}:{content}
                int  responseId = atoi(_inBuf + 1);
                const char* responseData = sep + 1;

                for (uint8_t i = 0; i < MAX_PENDING; ++i) {
                    if (_poolActive[i] && _pool[i].ID() == responseId) {
                        _pool[i].onResponse(responseData);
                        if (debug) Console::trace("Intercom") << "<r" << responseId << Console::endl;
                        break;
                    }
                }
            } else {
                // Incoming request: {uid}:{content}
                int        uid     = atoi(_inBuf);
                const char* content = sep + 1;
                Request request(uid, content);
                if (onRequestCallback) onRequestCallback(request);
            }

            // Restore (defensive)
            *sep    = ':';
            *crcSep = '|';
            _inBufLen = 0;

        } else if (c != '\r') {
            if (_inBufLen < 511) {
                if (_inBufLen == 0) _inBufStartMs = millis();
                _inBuf[_inBufLen++] = c;
            }
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  _processPendingRequests — IC-2: iterate fixed array, IC-6: correct cleanup
// ─────────────────────────────────────────────────────────────────────────────
FLASHMEM void Intercom::_processPendingRequests() {
    for (uint8_t i = 0; i < MAX_PENDING; ++i) {
        if (!_poolActive[i]) continue;

        Request& req    = _pool[i];
        Request::Status status = req.getStatus();

        // Hard timeout: clear requests that have been in flight > 300ms
        if (status != Request::Status::CLOSED &&
            status != Request::Status::IDLE   &&
            millis() - req.getLastSent() > 300) {
            req.close();
            _poolActive[i] = false;
            continue;
        }

        switch (status) {
            case Request::Status::IDLE:
                req.send();
                break;

            case Request::Status::SENT:
                if (req.isTimedOut()) {
                    req.onTimeout();
                    _poolActive[i] = false;  // clean up timed-out request
                } else if (millis() - req.getLastSent() > 100) {
                    req.send();  // retry
                }
                break;

            case Request::Status::OK:
                req.close();
                _poolActive[i] = false;
                break;

            case Request::Status::TIMEOUT:
            case Request::Status::CLOSED:
            case Request::Status::ERROR:
                _poolActive[i] = false;
                break;

            default:
                break;
        }
    }
}
