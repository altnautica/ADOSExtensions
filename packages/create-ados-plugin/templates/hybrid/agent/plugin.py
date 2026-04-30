"""Hello agent plugin.

Runs as a subprocess under ados-supervisor. Talks to the host over the
UDS msgpack bridge that the plugin host opens at startup. The Python
SDK at `ados.sdk` (re-exported from the agent package) hides the IPC
shape; this template uses the bare loop until the SDK API is final.
"""

from __future__ import annotations

import asyncio
import logging
import sys


log = logging.getLogger("plugin.hello")


async def main() -> None:
    log.info("hello plugin online")
    # Plugins talk to the host on stdin/stdout (msgpack) or via the
    # UDS socket the supervisor passes in PLUGIN_SOCKET. See the
    # `ados.sdk` reference at `docs.altnautica.com/developers/sdk-python`.
    while True:
        await asyncio.sleep(60)


if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s %(message)s",
        stream=sys.stderr,
    )
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
