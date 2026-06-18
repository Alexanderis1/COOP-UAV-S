"""FCU <-> Mission Computer link (P3-7): framing + deterministic channel."""

from coopuavs.coopfc.link.coop_link import (
    BATT_CODES, BATT_NAMES, Channel, DEGRADED_CODES, DEGRADED_NAMES,
    FAILSAFE_CODES, FAILSAFE_NAMES, FrameDecoder, MAX_PAYLOAD, MODE_CODES,
    MODE_NAMES, MSG, STATE_CODES, STATE_NAMES, decode_msg, encode_frame,
    encode_msg,
)

__all__ = ["BATT_CODES", "BATT_NAMES", "Channel", "DEGRADED_CODES",
           "DEGRADED_NAMES", "FAILSAFE_CODES", "FAILSAFE_NAMES",
           "FrameDecoder", "MAX_PAYLOAD", "MODE_CODES", "MODE_NAMES", "MSG",
           "STATE_CODES", "STATE_NAMES", "decode_msg", "encode_frame",
           "encode_msg"]
