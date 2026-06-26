#!/usr/bin/env python3
"""WebSocket client for testing nav2_status_socket_bridge output."""

import argparse
import asyncio
import json
import sys

import websockets


async def run(args: argparse.Namespace) -> int:
    url = f'ws://{args.host}:{args.port}{args.path}'
    received = 0

    try:
        async with websockets.connect(url) as websocket:
            print(f'Connected to {url}', flush=True)
            async for message in websocket:
                try:
                    payload = json.loads(message)
                    print(json.dumps(payload, ensure_ascii=False, indent=2))
                except json.JSONDecodeError:
                    print(message)
                received += 1
                if args.count and received >= args.count:
                    return 0
    except OSError as exc:
        print(f'Connect failed: {exc}', file=sys.stderr)
        return 1
    except websockets.exceptions.ConnectionClosed:
        print('Connection closed by server', file=sys.stderr)
        return 2

    return 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description='Connect to nav2_status_socket_bridge WebSocket and print JSON messages.'
    )
    parser.add_argument('--host', default='127.0.0.1', help='Bridge host (default: 127.0.0.1)')
    parser.add_argument('--port', type=int, default=9090, help='Bridge port (default: 9090)')
    parser.add_argument(
        '--path',
        default='/nav2/status',
        help='WebSocket path (default: /nav2/status)',
    )
    parser.add_argument('--count', type=int, default=0, help='Stop after N messages (0 = run forever)')
    args = parser.parse_args()
    if not args.path.startswith('/'):
        args.path = f'/{args.path}'

    try:
        return asyncio.run(run(args))
    except KeyboardInterrupt:
        return 0


if __name__ == '__main__':
    raise SystemExit(main())
