"""
transport/wired.py — USB-CDC wired transport for Teensy 4.1 (USB_DIRECT firmware).

Connects to the Teensy via USB-CDC (COM port / /dev/ttyACM0) when the firmware
is compiled with #define USB_DIRECT in settings.h.

Protocol is identical to XBeeTransport — framed CRC8 requests and telemetry.
The only difference is the default baudrate (115200) and transport identity.

Usage:
    transport = WiredTransport(port="COM6")
    transport.connect()
    ok, resp = transport.execute("go(500,300)", timeout_ms=30000)
"""

from .xbee import XBeeTransport
from shared.config import USB_DIRECT_BAUDRATE


class WiredTransport(XBeeTransport):
    """
    USB-CDC wired transport for Teensy 4.1 running USB_DIRECT firmware.

    Inherits the full XBeeTransport protocol (ping/pong handshake, framed
    CRC8 requests, telemetry subscriptions, heartbeat).

    Use this class when the robot is physically connected via USB cable.
    Use XBeeTransport when the robot communicates via XBee radio through a Jetson.
    """

    TRANSPORT_TYPE = 'usb'

    def __init__(self, port: str, baudrate: int = USB_DIRECT_BAUDRATE):
        super().__init__(port=port, baudrate=baudrate)

    @property
    def transport_type(self) -> str:
        return self.TRANSPORT_TYPE
