"""Run UAV

Simulate powering on components of UAV. Initilaizes onboard computer,
clock, sensor hub, motor hub, and GPS unit.
"""

import argparse
import asyncio

import uav
import xpio

async def main(graph: str | bool = False, print_: str | bool = False) -> None:
    tasks = [
        asyncio.create_task(uav.main(graph=graph, print_=print_)),
        asyncio.create_task(xpio.main())
    ]

    await asyncio.gather(*tasks)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-g", "--graph", nargs='?', default=False, const='rxdata.att.rollspeed', help="Attribute to graph")
    parser.add_argument("-p", "--print", nargs='?', default=False, const='afcs._throttles', help="Attribute to print")
    parser.add_argument("-s", "--skip", nargs='?', default='-1', const='0', help="Skip number of modes on startup")
    args = parser.parse_args()

    whitelisted = frozenset("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789._-")
    if args.graph:
        assert all(char in whitelisted for char in args.graph) and '__' not in args.graph
    if args.print:
        assert all(char in whitelisted for char in args.print) and '__' not in args.print
    if args.skip:
        assert all(char in whitelisted for char in args.skip) and '__' not in args.skip

    uav.DEBUG_SKIP = int(args.skip)

    try:
        asyncio.run(main(args.graph, args.print))
    except KeyboardInterrupt:
        pass
