#!/usr/bin/env python3
# coding=utf-8

"""Small example OSC server

This program listens to several addresses, and prints some information about
received packets.
https://pypi.python.org/pypi/python-osc
"""

import argparse
import math

from pythonosc import dispatcher
from pythonosc import osc_server


def print_dmx_handler(addr, value):
    print("[{0}] ~ {1}".format(addr, value))

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--ip",
        default="127.0.0.1",
        help="The ip to listen on"
    )
    parser.add_argument(
        "--port",
        type=int,
        default=9006,
        help="The port to listen on"
    )
    args = parser.parse_args()

    dispatcher = dispatcher.Dispatcher()
    dispatcher.map("/6/dmx/*", print_dmx_handler)

    server = osc_server.ThreadingOSCUDPServer(
      (args.ip, args.port), dispatcher)
    print("Serving on {}".format(server.server_address))
    server.serve_forever()
