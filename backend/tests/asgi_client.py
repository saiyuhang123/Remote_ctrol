"""In-memory HTTP client without a nested TestClient portal thread."""
import asyncio
from contextlib import suppress

import httpx


class ASGIClient:
    def __init__(self, app):
        self.app = app

    def request(self, method, url, **kwargs):
        async def tick():
            # A bounded selector tick also supports restricted test runtimes
            # whose cross-thread asyncio wakeups are not delivered reliably.
            while True:
                await asyncio.sleep(.005)

        async def perform():
            heartbeat = asyncio.create_task(tick())
            try:
                async with httpx.AsyncClient(transport=httpx.ASGITransport(app=self.app), base_url='http://test') as client:
                    return await client.request(method, url, **kwargs)
            finally:
                heartbeat.cancel()
                with suppress(asyncio.CancelledError):
                    await heartbeat
        return asyncio.run(perform())

    def get(self, url, **kwargs):
        return self.request('GET', url, **kwargs)

    def post(self, url, **kwargs):
        return self.request('POST', url, **kwargs)
