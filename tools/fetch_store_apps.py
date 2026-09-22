#!/usr/bin/env python3
"""Download the pinned App Bundles that seed the on-device MicroPixel store.

Every entry in apps/store-seed.json is a published store release for the Xtensa
(ESP32-S3) AOT target. Downloading the prebuilt bundles means the board port
never has to build Guests, so no WASI SDK or wamrc is involved anywhere in the
cloud pipeline.

usage: python3 fetch_store_apps.py --manifest apps/store-seed.json --output apps/downloaded
"""

from __future__ import annotations

import argparse
import hashlib
import http.client
import json
import sys
import urllib.error
import urllib.request
from pathlib import Path

TIMEOUT_SECONDS = 120
ATTEMPTS = 4
BUNDLE_MAGIC = b"MPXBNDL\0"
BLOCK_SIZE = 64 * 1024


def fail(message: str) -> None:
    print(f"[store] ERROR: {message}", file=sys.stderr)
    raise SystemExit(1)


def fetch(url: str) -> bytes:
    request = urllib.request.Request(url, headers={"User-Agent": "micropixel-h1-store-fetch"})
    last: Exception | None = None
    for attempt in range(1, ATTEMPTS + 1):
        try:
            with urllib.request.urlopen(request, timeout=TIMEOUT_SECONDS) as response:
                if response.status != 200:
                    raise urllib.error.HTTPError(url, response.status, "unexpected status", response.headers, None)
                return response.read()
        except (urllib.error.URLError, urllib.error.HTTPError, TimeoutError,
                http.client.IncompleteRead, ConnectionError, OSError) as error:
            last = error
            print(f"[store] attempt {attempt}/{ATTEMPTS} failed for {url}: {error}", file=sys.stderr)
    fail(f"could not download {url}: {last}")


def verify(entry: dict, data: bytes) -> None:
    expected_size = entry["sizeBytes"]
    if len(data) != expected_size:
        fail(f"{entry['appId']}: size {len(data)} != recorded {expected_size}")
    if len(data) % BLOCK_SIZE != 0:
        fail(f"{entry['appId']}: size {len(data)} is not 64 KiB aligned; BundleFS would reject it")
    if not data.startswith(BUNDLE_MAGIC):
        fail(f"{entry['appId']}: missing MPXBNDL magic")
    digest = hashlib.sha256(data).hexdigest()
    if digest != entry["sha256"]:
        fail(f"{entry['appId']}: sha256 {digest} != recorded {entry['sha256']}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    arguments = parser.parse_args()

    manifest = json.loads(arguments.manifest.read_text(encoding="utf-8"))
    template = manifest["downloadUrlTemplate"]
    output = arguments.output.resolve()
    output.mkdir(parents=True, exist_ok=True)

    total = 0
    blocks = 0
    for entry in manifest["apps"]:
        app_id = entry["appId"]
        target = output / f"{app_id}.bundle.bin"
        if target.is_file():
            cached = target.read_bytes()
            try:
                verify(entry, cached)
            except SystemExit:
                print(f"[store] cached copy of {app_id} is invalid; downloading again", file=sys.stderr)
                target.unlink()
            else:
                print(f"[store] cached  {app_id:<26} v{entry['version']:<7} {len(cached):>8} B")
                total += len(cached)
                blocks += len(cached) // BLOCK_SIZE
                continue
        data = fetch(template.format(releaseId=entry["releaseId"]))
        verify(entry, data)
        # A partially written bundle would poison the BundleFS image.
        temporary = target.with_suffix(".part")
        temporary.write_bytes(data)
        temporary.replace(target)
        print(f"[store] fetched {app_id:<26} v{entry['version']:<7} {len(data):>8} B  {entry['title']}")
        total += len(data)
        blocks += len(data) // BLOCK_SIZE

    print()
    print(f"[store] {len(manifest['apps'])} bundle(s), {total} bytes, {blocks} BundleFS blocks")
    print(f"[store] staged in {output}")


if __name__ == "__main__":
    main()
