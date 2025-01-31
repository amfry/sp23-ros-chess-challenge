#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import serial

class SerialComms():
    """ General User Serial wrapper"""

    def __init__(self,port_name, baudrate=9600, timeout=1, bytesize=8, parity_bit=serial.PARITY_NONE, stopbits=1):
        self.port_name = port_name
        self.baudrate = baudrate
        self.timeout = timeout
        self.stopbits = stopbits
        self.parity_bit = parity_bit
        self.bytesize = bytesize
        self.ser = None
       
    def __enter__(self):
        self.open_port()
        return self
         
    def __exit__(self, exc_type, exc_value, traceback):
        self.close_port()
        if exc_type:
            print(exc_type, exc_value, traceback)
        return

    def open_port(self):
        self.ser = serial.Serial()
        self.ser.port = self.port_name
        self.ser.baudrate = self.baudrate
        self.ser.bytesize = self.bytesize
        self.ser.parity = self.parity_bit
        self.ser.stopbits = self.stopbits
        self.ser.timeout = self.timeout
        self.ser.open()
        self.ser.reset_output_buffer()
        self.ser.reset_input_buffer()
        return

    def close_port(self):
        try:
            self.ser.flush()
        except:
            self.ser.close()
        return

    def write(self,cmd):
        """Write to serial port, carriage return appeneded to str message
            Parameters:
                    cmd(str): message to be sent to serial device
        """
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self.ser.write(str.encode(cmd + '\r')) #carriage return
        return

    def read(self, chunk_size=100, sleep=0.2):
        """Returns string of data read from the output buffer

            Parameters:
                    chunk_size(int): default 200, the number of characters read from buffer
                    at one time. Larger size means faster reads

            Returns:
                    results (str): cleaned data read from output buffer"""
        time.sleep(sleep)
        read_buffer = b''
        while self.ser.isOpen() == True:
            byte_chunk = self.ser.read(chunk_size)
            read_buffer += byte_chunk
            size = len(byte_chunk)
            if not size == chunk_size:
                break
        results = read_buffer.decode().strip()
        return results
        
        