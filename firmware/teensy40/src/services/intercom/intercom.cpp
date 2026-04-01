#include "intercom.h"
#include "os/console.h"
#include "settings.h"
#include "comUtilities.h"

INSTANTIATE_SERVICE(Intercom)

Intercom::Intercom() : Service(ID_INTERCOM),  _stream(INTERCOM_SERIAL) {}

void Intercom::onAttach() {
    Console::info() << "Intercom activated" << Console::endl;
}

void Intercom::onUpdate() {
    if(!enabled()) return;

    if (_stream.available()) {
        _lastStream = millis();
        _processIncomingData();         
    }
    _processPendingRequests();

    if(millis() - _lastPing > 500 && (!_connected || (_connected && millis() - _lastStream > 1000))){
        sendMessage("ping");
        _lastPing = millis();
    }
    if(_connected && millis() - _lastStream > 2000){
        connectionLost();
    }
}

void Intercom::enable(){
    Service::enable();
    INTERCOM_SERIAL.begin(INTERCOM_BAUDRATE);
    delay(10);
    while(_stream.available()) _stream.read();  // drain stale bytes
    _inBufLen = 0;
    sendMessage("ping");
}

void Intercom::disable(){
    INTERCOM_SERIAL.end();
    Service::disable();
}

void Intercom::sendMessage(const char* message) {
    if (!_stream.availableForWrite()) return;  // non-blocking: drop if full
    _stream.print(message);
    _stream.write('\n');
    Console::trace("Intercom") << ">" << message << Console::endl;
}

void Intercom::sendMessage(const String& message) {
    if (!_stream.availableForWrite()) return;
    _stream.print(message);
    _stream.write('\n');
    Console::trace("Intercom") << ">" << message.c_str() << Console::endl;
}

int Intercom::sendRequest(const String& payload, long timeout,  requestCallback_ptr cbfunc,  callback_ptr func){
    Request req(payload, timeout, cbfunc, func);
    req.send();
    _sentRequests.insert({req.ID(), req});
    return req.ID();
}

void Intercom::pingReceived() {
    sendMessage("pong");
}

bool Intercom::closeRequest(int uid) {
    if(_sentRequests.count(uid)){
        _sentRequests.find(uid)->second.close();
        return true;
    }else{
        Console::warn("Intercom") << __FILE__ << " at line " << __LINE__ << " request " << int(uid) << " does not exist" << Console::endl;
        return false;
    };
}

void Intercom::setConnectLostCallback(callback_ptr callback){
    onConnectionLost = callback;
}

void Intercom::setRequestCallback(requestCallback_ptr callback){
     onRequestCallback = callback;
}

void Intercom::setConnectionSuccessCallback(callback_ptr callback){
    onConnectionSuccess = callback;
}


String Intercom::getRequestResponse(int uid) {
    if(_sentRequests.count(uid) > 0){
        return _sentRequests.find(uid)->second.getResponse();
    }else{
        Console::warn("Intercom") << __FILE__ << " at line " << __LINE__ << " request " << int(uid)   << " does not exist" << Console::endl;
        return "ERROR";
    };
}

void Intercom::connectionLost() {
    Console::warn("Intercom") << "Connection lost." << Console::endl;
    _connected = false;
    if(onConnectionLost!=nullptr){
        onConnectionLost();
    }
}

void Intercom::connectionSuccess(){
    Console::info("Intercom") << "Connection successful." << Console::endl;
    _connected = true;
    if(onConnectionSuccess!=nullptr){
        onConnectionSuccess();
    }
}

void Intercom::_processIncomingData() {
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

            // ── Connection handshake ─────────────────────────────────────────
            if (strncmp(_inBuf, "ping", 4) == 0) {
                pingReceived();
                _inBufLen = 0;
                continue;
            }
            if (strncmp(_inBuf, "pong", 4) == 0) {
                if (!isConnected()) connectionSuccess();
                _inBufLen = 0;
                continue;
            }

            // ── Find delimiters ──────────────────────────────────────────────
            char* sep    = strchr (_inBuf, ':');
            char* crcSep = strrchr(_inBuf, '|');

            if (!sep || !crcSep || crcSep <= sep) {
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

            if (_inBuf[0] == 'r' && !_sentRequests.empty()) {
                // ── Reply: r{uid}:{content}|{crc} ───────────────────────────
                int responseId = atoi(_inBuf + 1);
                String responseData = String(sep + 1);  // one unavoidable alloc

                auto requestIt = _sentRequests.find(responseId);
                if (requestIt != _sentRequests.end()) {
                    requestIt->second.onResponse(responseData);
                } else {
                    Console::trace("Intercom") << "Reply UID " << responseId << " not found" << Console::endl;
                }
            } else {
                // ── Incoming request: {uid}:{content}|{crc} ─────────────────
                int    uid     = atoi(_inBuf);
                String content = String(sep + 1);  // one unavoidable alloc

                Request request(uid, content);
                if (onRequestCallback != nullptr) onRequestCallback(request);
            }

            // Restore delimiters
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
    // NOTE: Removed INTERCOM_SERIAL.clear() — it was discarding bytes
    // that arrived during processing, causing silent data loss.
}

void Intercom::_processPendingRequests() {
    for (auto it = _sentRequests.begin(); it != _sentRequests.end();) {

        Request& request = it->second;
        Request::Status status = request.getStatus();
    
        if(status != Request::Status::CLOSED && status != Request::Status::IDLE && millis() - request.getLastSent() > 1000){
            Console::trace("Intercom") << ": request " << request.getContent() << "too old, cleared." << Console::endl;
            request.close();
            ++it;
            continue;
        }

        if (status == Request::Status::IDLE) {
            request.send();
            ++it;
        } else if (status == Request::Status::SENT) {
            if (request.isTimedOut()) {
                request.onTimeout();
            } else {
                if(millis() - request.getLastSent() > 5)  request.send();
                ++it;
            }
        } else if (status == Request::Status::OK) {
            ++it;
            request.close();
        } else if (status == Request::Status::CLOSED) {
            Console::trace("Intercom") << int(_sentRequests.size()) << "currently in the buffer" << Console::endl;
            it = _sentRequests.erase(it); // Remove the request from the map

        } else if (status == Request::Status::TIMEOUT) {
            request.close();
            it = _sentRequests.erase(it);  // IC-6: was missing erase+advance
            continue;
        } else if (status == Request::Status::ERROR) {
            request.close();
            Console::error("Intercom") << "request " << request.getContent() << " unknown error." << Console::endl;
        } else {
            ++it;
        }
    }
}
