#!/usr/bin/env python3
# coding=utf-8

"""
bridge between serial and osc.

     (with format conversion)

    history:
        see git commits

    todo:
        ~ all fine :-)
"""

# https://docs.python.org/2.7/howto/pyporting.html#division
from __future__ import division

import sys
import os
import time
import array
import struct
import signal
import argparse
import re
import readline
import json
import io
import threading
import queue
# import multiprocessing

import serial
from serial.tools import list_ports

# python-osc (python3)
from pythonosc import osc_message_builder
from pythonosc import udp_client
from pythonosc import dispatcher
from pythonosc import osc_server

# pyOSC (python2)
# from OSC import OSCClient
# from OSC import OSCMessage

# from configdict import ConfigDict


version = """04.12.2016 22:53 stefan"""


##########################################
# globals

# config


##########################################
# functions

def helper_join(instance):
    try:
        instance.join()
    except RuntimeError as e:
        message = "{}".format(e)
    else:
        message = "joined"
    finally:
        # print("* {}: {}.".format(instance, message))
        print("* {}: {}.".format(
            instance.__class__.__name__,
            message
        ))

##########################################
# classes


class SerialHandler(threading.Thread):
    """Wrapper for SerialPort."""

    def __init__(self, device, rx_queue, tx_queue):
        super(SerialHandler, self).__init__()

        self._shutdown = threading.Event()

        self.device = device
        self.baudrate = 115200

        self.rx_queue = rx_queue
        self.tx_queue = tx_queue

        # https://pythonhosted.org/pyserial/shortintro.html#eol
        ser = serial.serial_for_url(
            self.device,
            baudrate=self.baudrate,
            timeout=0,
            do_not_open=True
        )
        sio = io.TextIOWrapper(io.BufferedRWPair(ser, ser))

        self.ser = ser
        self.sio = sio

    def open(self):
        """Open the Serial port."""
        if not self.ser.is_open:
            try:
                self.ser.open()
            except serial.serialutil.SerialException as e:
                error_message = (
                    "Port {} is already in use by other program: {}"
                    .format(
                        self.ser.port,
                        e
                    )
                )
                print(error_message)
                # if thread is running stop it.
                self._shutdown.set()
        # print(" is_open:", ser.is_open)

    def close(self):
        """Close the Serial port."""
        if self.ser.is_open:
            print("close port:")
            self.ser.close()

    def read_nonblocking(self):
        """Read and put in queue."""
        # last_line = None
        line = None
        if self.ser.is_open:
            ser = self.ser
            sio = self.sio
            try:
                # lines = sio.readlines()
                line = sio.readline()
            except serial.serialutil.SerialException as e:
                error_message = (
                    "readlines failed at port {}: {}"
                    .format(
                        ser.port,
                        e
                    )
                )
                print(error_message)
                self.close()
                self._shutdown.set()
            # else:
            #     # print(lines)
            #     if len(lines) > 0:
            #         last_line = lines[len(lines)-1]
            #         # print(last_line)
        # return last_line
        return line

    def send(self, message):
        if self.ser.is_open:
            sio.write(unicode(message))
            # this it is buffering. required to get the data out *now*
            sio.flush()

    def run(self):
        while not self._shutdown.is_set:
            if self.ser.is_open:
                # check for new content
                line = self.read_nonblocking()
                if line:
                    self.rx_queue.put(line)

                # check if there is data to send.
                try:
                    message = self.tx_queue.get_nowait()
                except queue.Empty:
                    # thats fine :-)
                    pass
                else:
                    self.send(message)
            else:
                # try to open
                self.open()

        self.close()

    def shutdown(self):
        self._shutdown.set()


class Serial2OSC(threading.Thread):
    """threaded serial to osc conversion"""

    def __init__(self, target_ip, target_port, rx_queue):
        super(Serial2OSC, self).__init__()

        self._shutdown = threading.Event()

        self.rx_queue = rx_queue
        self.target_ip = target_ip
        self.target_port = target_port

        self.osc_client = udp_client.SimpleUDPClient(
            self.target_ip,
            self.target_port
        )

    def handle_serial_message(message):
        print(message)
        # self.osc_client.send_message("/filter", random.random())

    def run(self):
        while not self._shutdown.is_set:
            # check if there is data to send.
            try:
                message = self.rx_queue.get_nowait()
            except queue.Empty:
                # thats fine :-)
                pass
            else:
                self.handle_serial_message(message)

    def shutdown(self):
        self._shutdown.set()


class OSC2Serial(threading.Thread):
    """threaded osc to serial conversion"""

    def __init__(self, listen_ip, listen_port, tx_queue):
        super(OSC2Serial, self).__init__()

        self._shutdown = threading.Event()

        self.listen_ip = listen_ip
        self.listen_port = listen_port
        self.tx_queue = tx_queue

        self.channel_count = 512

        # prepare temp array
        data_output = array.array('B')
        data_output *= self.channel_count

        self.dispatcher = dispatcher.Dispatcher()

        # /QLC+ universe - 1/dmx/DMX channel - 1
        self.dispatcher.map("/6/dmx", self.handle_osc_message)

        # self.osc_server = osc_server.ForkingOSCUDPServer(
        self.osc_server = osc_server.ThreadingOSCUDPServer(
            (self.listen_ip, self.listen_port),
            self.dispatcher
        )

    def handle_osc_message(self, message):
        # check for new content

        if line:
            self.tx_queue.put(line)

    def run(self):
        self.osc_server.serve_forever()

    def shutdown(self):
        # active_children is possible by ForkingOSCUDPServer
        # if self.osc_server.active_children:
        #     print("osc_server.shutdown()")
        #     self.osc_server.shutdown()
        self.osc_server.shutdown()


##########################################
def handle_userinput(user_input):
    """Handle userinput in interactive mode."""
    global flag_run
    if user_input == "q":
        flag_run = False
        print("stop script.")
    elif user_input.startswith("x"):
        # do something?!
        pass
    else:
        flag_run = False
        print("stop script.")


##########################################
if __name__ == '__main__':

    print(42*'*')
    print('Python Version: ' + sys.version)
    print(42*'*')

    ##########################################
    # commandline arguments
    # filename_default = "./pattern.json"
    osc_target_ip_default = "127.0.0.1"
    # osc_target_port_default = 7700
    osc_target_port_default = 7706
    # osc_listen_port_default = 9000
    osc_listen_port_default = 9006

    serial_device_default = "/dev/ttyACM0"

    parser = argparse.ArgumentParser(
        description="OSC-Serial bridge"
    )
    # parser.add_argument(
    #     "-c",
    #     "--config",
    #     help="specify a location for the config file (defaults to {})".
    #     format(
    #         filename_default
    #     ),
    #     metavar='FILENAME',
    #     default=filename_default
    # )
    parser.add_argument(
        "--osc-target-ip",
        help="osc target ip",
        metavar='osc_target_ip',
        default=osc_target_ip_default
    )
    parser.add_argument(
        "--osc-target-port",
        help="osc target port",
        metavar='osc_target_port',
        default=osc_target_port_default
    )
    parser.add_argument(
        "--osc-listen-port",
        help="osc listen port",
        metavar='osc_listen_port',
        default=osc_listen_port_default
    )
    parser.add_argument(
        "-d",
        "--serial-device",
        help="serial device",
        metavar='serial_device',
        default=serial_device_default
    )
    parser.add_argument(
        "-i",
        "--interactive",
        help="run in interactive mode",
        action="store_true"
    )
    parser.add_argument(
        "-v",
        "--verbose",
        help="show advanced log information",
        action="store_true"
    )
    args = parser.parse_args()

    ##########################################
    # prepare
    if args.interactive:
        print(42*'*')
        print(__doc__)
        print(42*'*')

    # init flag_run
    flag_run = False

    # helper
    def _exit_helper(signal, frame):
        """Stop loop."""
        global flag_run
        flag_run = False

    # setup termination and interrupt handling:
    signal.signal(signal.SIGINT, _exit_helper)
    signal.signal(signal.SIGTERM, _exit_helper)

    ##########################################
    # main logic

    serial_tx_queue = queue.Queue()
    serial_rx_queue = queue.Queue()

    mySerialHandler = SerialHandler(
        args.serial_device,
        serial_rx_queue,
        serial_tx_queue
    )

    mySerial2OSC = Serial2OSC(
        args.osc_target_ip,
        args.osc_target_port,
        serial_rx_queue
    )

    myOSC2Serial = OSC2Serial(
        "127.0.0.1",
        args.osc_listen_port,
        serial_tx_queue
    )

    ##########################################
    # start things

    print("start")
    print("* mySerialHandler")
    mySerialHandler.start()
    print("* mySerial2OSC")
    mySerial2OSC.start()
    print("* myOSC2Serial")
    myOSC2Serial.start()

    print("running...\n")

    ##########################################

    if args.interactive:
        # wait for user to hit key.
        flag_run = True
        while flag_run:

            message = (
                "\n" +
                42*'*' + "\n"
                " -"
                "Ctrl+C or 'q' to stop script\n" +
                42*'*' + "\n"
                "\n"
            ).format(
                ""
            )
            try:
                if sys.version_info.major >= 3:
                    # python3
                    user_input = input(message)
                elif sys.version_info.major == 2:
                    # python2
                    user_input = raw_input(message)
                else:
                    # no input methode found.
                    value = "q"
            except KeyboardInterrupt:
                print("\nstop script.")
                flag_run = False
            except EOFError:
                print("\nstop script.")
                flag_run = False
            except Exception as e:
                print("unknown error: {}".format(e))
                flag_run = False
                print("stop script.")
            else:
                try:
                    if len(user_input) > 0:
                        handle_userinput(user_input)
                except Exception as e:
                    print("unknown error: {}".format(e))
                    flag_run = False
                    print("stop script.")
    # if not interactive
    else:
        # just wait
        flag_run = True
        print("(terminate with Ctrl+C)")
        try:
            while flag_run:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\nstop script.")
            flag_run = False

    # newline
    print()

    # shtudown all other threads:
    print("send shutdown to")
    print("* mySerialHandler")
    mySerialHandler.shutdown()
    print("* mySerial2OSC")
    mySerial2OSC.shutdown()
    print("* myOSC2Serial")
    myOSC2Serial.shutdown()

    # block untill threads have joined.
    print("wait for join...")
    helper_join(mySerialHandler)
    helper_join(mySerial2OSC)
    helper_join(myOSC2Serial)

    ##########################################
    print("script end.")
    ##########################################
