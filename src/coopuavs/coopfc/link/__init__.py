"""FCU <-> Mission Computer link (P3-7): framing + deterministic channel."""

from coopuavs.coopfc.link.coop_link import (
    Channel, FrameDecoder, MSG, decode_msg, encode_frame, encode_msg,
)

__all__ = ["Channel", "FrameDecoder", "MSG", "decode_msg", "encode_frame",
           "encode_msg"]
