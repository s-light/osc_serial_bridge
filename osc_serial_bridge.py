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
        """Init Serial Handler."""
        super(SerialHandler, self).__init__()

        self._shutdown = threading.Event()

        self.device = device
        self.baudrate = 115200

        self.rx_queue = rx_queue
        self.tx_queue = tx_queue

        # https://pythonhosted.org/pyserial/shortintro.html#eol
        self.ser = serial.serial_for_url(
            self.device,
            baudrate=self.baudrate,
            timeout=0,
            do_not_open=True
        )
        self.sio = io.TextIOWrapper(
            io.BufferedRWPair(self.ser, self.ser)
        )
        # http://stackoverflow.com/a/16961872/574981
        # self.sio = io.TextIOWrapper(
        #     io.BufferedRWPair(self.ser, self.ser, 1),
        #     newline='\n',
        #     line_buffering=True
        # )

    def open(self):
        """Open the Serial port."""
        # print("open():")
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
            else:
                print("opened serial device '{}'".format(self.device))
        # print(" is_open:", self.ser.is_open)

    def close(self):
        """Close the Serial port."""
        if self.ser.is_open:
            print("close port:")
            self.ser.close()

    def read_nonblocking(self):
        """Read serial message and put in queue."""
        # last_line = None
        line = None
        if self.ser.is_open:
            ser = self.ser
            sio = self.sio
            try:
                # lines = sio.readlines()
                line = sio.readline()
                # line = ser.readline()
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
            else:
                # if line:
                #     print("line: '{}'".format(line))
                # if len(lines) > 0:
                #     print(lines)
                #     line = lines[len(lines)-1]
                #     print(line)
                pass
        # return last_line
        return line

    def send(self, message):
        """Send serial message."""
        if self.ser.is_open:
            # print("send: " + message)
            self.sio.write(message)
            # this it is buffering. required to get the data out *now*
            self.sio.flush()

    def run(self):
        """handle incomming and outgoing serial treffic."""
        while not self._shutdown.is_set():
            if self.ser.is_open:
                # check for new content
                line = self.read_nonblocking()
                if line:
                    # print("line: '{}'".format(line))
                    self.rx_queue.put(line)
                    # print("rx_queue.qsize: '{}'".format(
                    #     self.rx_queue.qsize()
                    # ))

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
        # loop has exited.
        # make sure we close the port on exit..
        self.close()

    def shutdown(self):
        """Shutdown Serial Port."""
        self._shutdown.set()


class Serial2OSC(threading.Thread):
    """threaded serial to osc conversion"""

    def __init__(self, target_ip, target_port, rx_queue):
        """Init."""
        super(Serial2OSC, self).__init__()

        self._shutdown = threading.Event()

        self.rx_queue = rx_queue
        self.target_ip = target_ip
        self.target_port = target_port

        self.osc_client = udp_client.SimpleUDPClient(
            self.target_ip,
            self.target_port
        )

    def handle_serial_message(self, message):
        """Process an incomming serial message."""
        # print(message)
        print("message: '{}'".format(message))
        # if message.startswith("f"):
        #     pass
        # self.osc_client.send_message("/filter", random.random())

    def run(self):
        """Run - waits for incomming serial messages."""
        while not self._shutdown.is_set():
            # check if there is data to send.
            try:
                message = self.rx_queue.get_nowait()
            except queue.Empty:
                # thats fine :-)
                pass
            else:
                self.handle_serial_message(message)

    def shutdown(self):
        """Shutdown Serial handling."""
        self._shutdown.set()


class OSC2Serial(threading.Thread):
    """threaded osc to serial conversion"""

    def __init__(self, listen_ip, listen_port, tx_queue):
        """Init Class."""
        super(OSC2Serial, self).__init__()

        self._shutdown = threading.Event()

        self.listen_ip = listen_ip
        self.listen_port = listen_port
        self.tx_queue = tx_queue

        self.channel_count = 512
        # self.channel_count_bytes = struct.pack("H", self.channel_count)

        # prepare temp array
        # self.data = array.array('B')
        # self.data *= self.channel_count

        self.dispatcher = dispatcher.Dispatcher()

        self.setup_dispatcher()

        # self.osc_server = osc_server.ForkingOSCUDPServer(
        self.osc_server = osc_server.ThreadingOSCUDPServer(
            (self.listen_ip, self.listen_port),
            self.dispatcher
        )

    def setup_dispatcher(self):
        """setup osc listening addresses."""
        # /QLC+ universe - 1/dmx/DMX channel - 1
        self.dispatcher.map("/0/dmx/*", self.handle_osc_message)

    def handle_osc_message(self, osc_addr, osc_message):
        """Handle osc messages."""
        # check for new content
        if osc_message:
            # print("{}: {}".format(osc_addr, osc_message))
            # extract data
            channel_id = int(osc_addr[osc_addr.rfind('/')+1:])
            # convert from 0..1 to 0..255
            channel_value = int(255 * osc_message)

            # print(
            #     "channel_id: {}; "
            #     "channel_value: {};"
            #     "".format(
            #         channel_id,
            #         channel_value
            #     )
            # )

            # self.data[channel_id] = channel_value

            serial_message = "dmx/{:d}:{:d}\n".format(channel_id, channel_value)
            # print(serial_message)
            self.tx_queue.put(
                serial_message
            )

            # self.tx_queue.put(
            #     self.create_serial_dmx_message()
            # )

    # def create_serial_dmx_message(self):
    #     """Format dmx data as serial message."""
    #     # format: dmxSS/_____
    #     # SS = 2byte size
    #     # ____ channels each 1Byte
    #     return (
    #         "dmx" +
    #         struct.pack("H", len(self.data)) +
    #         "/" +
    #         self.data.tostring()
    #     )

    def run(self):
        """Run osc server."""
        self.osc_server.serve_forever()

    def shutdown(self):
        """Shutdown osc server."""
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
    osc_target_port_default = 7700
    # osc_target_port_default = 7706
    osc_listen_port_default = 9000
    # osc_listen_port_default = 9006

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
