# comms/packet.py — Transfer header builder
#
# Builds the JSON header dict that prefixes every TCP transfer.
# Protocol: JSON header (newline-terminated) → raw payload bytes → ACK/NACK byte.

import hashlib
import json
import os


def build_image_header(filepath, meta_dict):
    """Build the JSON transfer header for one image file.

    Args:
        filepath:   Full path to the JPEG on disk.
        meta_dict:  The image's metadata dict (from MetadataBuilder.build()).

    Returns:
        dict — JSON-serialisable header ready to send.
    """
    return {
        "type": "image",
        "filename": os.path.basename(filepath),
        "file_size": os.path.getsize(filepath),
        "md5": md5_file(filepath),
        "metadata": meta_dict,
    }


def build_telemetry_header(telem_dict):
    """Build the JSON transfer header for a telemetry packet.

    Returns:
        (header_dict, payload_bytes) — header to send first, then payload bytes.
        The payload is the telemetry dict serialised to UTF-8 JSON.
    """
    payload = json.dumps(telem_dict, default=str).encode("utf-8")
    header = {
        "type": "telemetry",
        "filename": "telemetry.json",
        "file_size": len(payload),
        "md5": hashlib.md5(payload).hexdigest(),
        "metadata": {},
    }
    return header, payload


def md5_file(filepath):
    """Compute MD5 hex digest of a file. Reads in 64 KB chunks."""
    h = hashlib.md5()
    with open(filepath, "rb") as f:
        for chunk in iter(lambda: f.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()
