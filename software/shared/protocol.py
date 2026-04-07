"""
shared/protocol.py — Wire protocol between Jetson and Teensy 4.1

Protocol (mirrors src/services/intercom/):

  Request (Jetson → Teensy):
      <id>:<content>|<crc8>\n
      Example: 42:go(500,300)|123\n

  Reply (Teensy → Jetson):
      r<id>:<response>|<crc8>\n
      Example: r42:ok|45\n

  Message fire-and-forget:
      ping\n
      pong:<bridge>\n              (bridge = 'usb' or 'xbee')
      TEL:<type>:<data>|<crc8>\n   (telemetry push, no reply needed)

Telemetry types (Teensy → Jetson):
  TEL:pos:<x>,<y>,<theta>         — robot position update
  TEL:motion:<state>              — IDLE | RUNNING | PAUSED | DONE:ok | DONE:fail
  TEL:safety:<0|1>                — obstacle detected
  TEL:chrono:<elapsed_ms>         — match timer
  TEL:occ:<hex_encoded_map>       — occupancy map bytes

Commands (Jetson → Teensy, content field of requests):
  hb                              — heartbeat (reply: "ok")
  tel                             — request full telemetry snapshot
  occ                             — request occupancy map
  go(<x>,<y>)                     — move to position (reply when done)
  goPolar(<angle>,<dist>)         — polar move
  turn(<angle>)                   — rotate to absolute heading
  align(<side>,<angle>)           — align robot side to orientation
  setAbsPosition(<x>,<y>,<angle>) — force position
  cancel                          — cancel current motion
  pause                           — pause motion
  resume                          — resume motion
  elevator(<side>,<pose>)         — move actuator
  grab(<side>)                    — grab object
  drop(<side>)                    — release object
  start                           — start match
  stop                            — stop robot
  fb(<id>)                        — trigger fallback sequence by ID
  feed(<feedrate>)                — set feedrate [0.05..1.0]
"""

import crcmod

# CRC8-SMBUS (polynomial 0x07, init=0x00) — matches Teensy CRC8.smbus()
_crc8_fn = crcmod.predefined.mkCrcFun('crc-8')


def crc8(data: str) -> int:
    return _crc8_fn(data.encode('ascii'))


def encode_request(uid: int, content: str) -> str:
    """Build a request frame: '<uid>:<content>|<crc>\n'"""
    payload = f"{uid}:{content}"
    return f"{payload}|{crc8(payload)}\n"


def encode_reply(uid: int, response: str) -> str:
    """Build a reply frame: 'r<uid>:<response>|<crc>\n'"""
    payload = f"r{uid}:{response}"
    return f"{payload}|{crc8(payload)}\n"


def encode_telemetry(ttype: str, data: str) -> str:
    """Build a telemetry push frame: 'TEL:<type>:<data>|<crc>\n'"""
    payload = f"TEL:{ttype}:{data}"
    return f"{payload}|{crc8(payload)}\n"


def parse_frame(line: str):
    """
    Parse an incoming line from Teensy.

    Returns one of:
      ('ping',    None,    None)
      ('pong',    None,    str_bridge)     — bridge type: 'usb', 'xbee', or None
      ('reply',   int_id,  str_response)   — response to a sent request
      ('request', int_id,  str_content)    — incoming request from Teensy
      ('tel',     str_type, str_data)      — telemetry push
      (None,      None,    None)           — parse error
    """
    line = line.strip()
    if not line:
        return (None, None, None)

    if line == 'ping':
        return ('ping', None, None)
    if line.startswith('pong'):
        # "pong:usb", "pong:xbee", or bare "pong" (legacy)
        bridge = line.split(':', 1)[1] if ':' in line else None
        return ('pong', None, bridge)

    # Strip CRC
    if '|' not in line:
        return (None, None, None)

    data_part, crc_part = line.rsplit('|', 1)
    try:
        expected_crc = int(crc_part)
    except ValueError:
        return (None, None, None)
    if crc8(data_part) != expected_crc:
        return (None, None, None)

    # Telemetry push — compact format: "T:<type> <data...>"
    #                  legacy format:  "TEL:<type>:<data...>"
    if data_part.startswith('T:'):
        rest = data_part[2:]
        # Compact: "T:p 1234 678 1571" → type="p", data="1234 678 1571"
        # Legacy:  "TEL:pos:x=1.0,y=2.0" → strip extra "EL:", type="pos", data="x=1.0,y=2.0"
        if rest.startswith('EL:'):
            # Legacy "TEL:" prefix — strip "EL:" and split on ':'
            rest = rest[3:]
            colon = rest.index(':')
            ttype = rest[:colon]
            tdata = rest[colon + 1:]
        else:
            # Compact "T:" prefix — split on first space
            sp = rest.find(' ')
            if sp >= 0:
                ttype = rest[:sp]
                tdata = rest[sp + 1:]
            else:
                ttype = rest
                tdata = ''
        return ('tel', ttype, tdata)

    # Reply from Teensy to one of our requests
    if data_part.startswith('r'):
        colon = data_part.index(':')
        uid   = int(data_part[1:colon])
        resp  = data_part[colon + 1:]
        return ('reply', uid, resp)

    # Request from Teensy to us
    if ':' in data_part:
        colon   = data_part.index(':')
        uid     = int(data_part[:colon])
        content = data_part[colon + 1:]
        return ('request', uid, content)

    return (None, None, None)
