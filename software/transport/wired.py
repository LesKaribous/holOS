"""
transport/wired.py — USB-CDC wired transport (thin alias for XBeeTransport).

Since firmware auto-detects the active bridge (USB-CDC vs XBee) and both use
the same baudrate (57600), this is a convenience alias that sets TRANSPORT_TYPE
to 'usb' for logging / display purposes.  Protocol is identical.

Usage:
    transport = WiredTransport(port="COM6")
    transport.connect()
"""

from .xbee import XBeeTransport
from shared.config import BRIDGE_BAUDRATE


class WiredTransport(XBeeTransport):
    """USB-CDC wired transport — alias for XBeeTransport with 'usb' label."""

    TRANSPORT_TYPE = 'usb'

    def __init__(self, port: str, baudrate: int = BRIDGE_BAUDRATE):
        super().__init__(port=port, baudrate=baudrate)

    @property
    def transport_type(self) -> str:
        return self.TRANSPORT_TYPE
