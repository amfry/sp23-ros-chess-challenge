import serial
import time
import sys
from serial_comms import SerialComms
from helper import substr_search

port = "/tmp/stockfish"
with SerialComms(port) as conn:
    conn.ser.reset_input_buffer()
    conn.ser.reset_output_buffer()
    time.sleep(0.1)
    print(conn.port_name)

    try:
        conn.write("uci")
        flag = True
    #read_data = self.conn.read()
    #flag = substr_search("uciok",read_data)
    except:
        flag = False
    print(flag)

    try:
        conn.write("ucinewgame")
        flag = True
    #read_data = self.conn.read()
    #flag = substr_search("uciok",read_data)
    except:
        flag = False
    print(flag)

    try:
        conn.write("isready")
        read_data = conn.read()
        print(read_data)
        flag = substr_search("readyok",read_data)
        print("new:" + str(flag))
    except:
        flag = False
    print(flag)


    # conn.write("uci")
    # result_str = conn.read() 
    # time.sleep(1)
    # print(result_str)
    # if "uciok" in result_str:
    #     print("Found!")
    # else:
    #     print("Not found!")
    # conn.write("ucinewgame")
    # time.sleep(1)
    # conn.write("isready")
    # time.sleep(1)
    # result_str = conn.read() 
    # time.sleep(1)
    # print(result_str)
    # if "readyok" in result_str:
    #     print("Found!")
    # else:
    #     print("Not found!")
    # conn.write("go movetime " + str(time))
    # time.sleep(1)
    # read_data = conn.read()
    # print(read_data)



# ser = serial.Serial()
# ser.port = port
# ser.baudrate = baud
# ser.bytesize = serial.EIGHTBITS
# ser.parity = serial.PARITY_NONE
# ser.xonxoff = 0
# ser.rtscts = 0
# ser.dsrdtr = 0
# ser.stopbits = 1
# ser.timeout = 0.3

# ser.close()
# ser.open()
# if ser.isOpen():
#     print(ser.name + ' is open...')
#     ser.flushOutput()
#     ser.flushInput()
#     while True:
#         cmd = "uci"
#         ser.write(str.encode(cmd + '\r')) #carriage return
#         #val1 = ser.read_until(str.encode("uciok\r"),64).decode()
#         val1 = ser.read(256).decode()
#         time.sleep(1)
#         print(ser.inWaiting())
#         out1 = val1.strip()
#         print('Receiving... ')
#         print(out1)
#         print(val1)
#         break
# ser.close()

    # def read(self,chunk_size=200):
    #     #self.ser.reset_output_buffer()
    #     read_buffer = b''
    #     round = 0
    #     empty_reads = 0
    #     while True:
    #         round = round + 1
    #         print("round" + str(round))
    #         # Read in chunks. Each chunk will wait as long as specified by
    #         # timeout. Increase chunk_size to fail quicker
    #         print("out_buffer: " + str(self.ser.out_waiting))
    #         print("in_buffer: " + str(self.ser.in_waiting))
    #         byte_chunk = self.ser.read(size=chunk_size)
    #         read_buffer += byte_chunk
    #         size = len(byte_chunk)
    #         print(size)
    #         if size == 0:
    #             empty_reads = empty_reads + 1
    #             print("empty:" + str(empty_reads))
    #             if empty_reads >= 3:
    #                 break
    #                 # if not size >= chunk_size:
    #                 #     break
    #     read_buffer_str = read_buffer.decode()
    #     return read_buffer_str
