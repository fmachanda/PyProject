"""Run UAV

Simulate powering on components of UAV. Initilaizes onboard computer,
clock, sensor hub, motor hub, and GPS unit.
"""

import asyncio

import uav
import xpio

async def main() -> None:
    tasks = [
        asyncio.create_task(uav.main()),
        asyncio.create_task(xpio.main())
    ]

    await asyncio.gather(*tasks)

if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
