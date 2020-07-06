import PQ9Client
import random
import struct
import csv
import sys

# this function communicates over the spacecraft bus and requests the detumble service.
# input: sensor readings (two sensors)
# output: desired dipole moment, tumble parameter
def detumblealgo(magx1, magy1, magz1, magx2, magy2, magz2):
    # important: set the correct address and port in PQ9Client.py
    # also important: comment out all print() statements in PQ9Client.py when performing HIL tests

    # used to connect to the satellite
    client = PQ9Client.PQ9Client() 
    client.connect()

    # The spacecraft bus communicates using bytes. hence the magnetic sensor readings should be converted to a strig of bytes

    # convert input to array of bytes to send over the spacecraft bus
    bmagx1 = bytearray(struct.pack("d",magx1)) 
    bmagy1 = bytearray(struct.pack("d",magy1))
    bmagz1 = bytearray(struct.pack("d",magz1)) 
    bmagx2 = bytearray(struct.pack("d",magx2)) 
    bmagy2 = bytearray(struct.pack("d",magy2)) 
    bmagz2 = bytearray(struct.pack("d",magz2)) 

    # initialise strings 
    strbmagx1 = ""
    strbmagy1 = ""
    strbmagz1 = ""
    strbmagx2 = ""
    strbmagy2 = ""
    strbmagz2 = ""

    # convert bytes to strings and append with spaces in between
    for i in range(0,len(bmagx1)):
        strbmagx1 = strbmagx1 + " " + str(bmagx1[i])
    for i in range(0,len(bmagx2)):
        strbmagx2 = strbmagx2 + " " + str(bmagx2[i])

    for i in range(0,len(bmagy1)):
        strbmagy1 = strbmagy1 + " " + str(bmagy1[i])
    for i in range(0,len(bmagy2)):
        strbmagy2 = strbmagy2 + " " + str(bmagy2[i])

    for i in range(0,len(bmagz1)):
        strbmagz1 = strbmagz1 + " " + str(bmagz1[i])
    for i in range(0,len(bmagz2)):
        strbmagz2 = strbmagz2 + " " + str(bmagz2[i])

    # generate detumble request
    # ADCS destination = 5
    # source (does not really matter for no) = 8
    # detumbleservicenumber = 25
    # request = 1
    # detumblepayload command = 0
    # data is the string of bytes in order: x1 y1 z1 x2 y2 z2    
    command = {}
    command["_send_"] = "SendRaw"
    command["dest"] = "5" # EPS
    command["src"] = "8"
    command["data"] = "25 1 0" + strbmagx1 + strbmagy1 + strbmagz1 + strbmagx2 + strbmagy2 + strbmagz2
    client.sendFrame(command)
    succes, msg = client.getFrame()

    # spacecraft bus returns string of bytes, need to be converted again to doubles
    
    # split string of bytes into individual integers
    PLbytes = [int(s) for s in msg["_raw_"][1:-1].split(',')]

    # obtain only the payload integers
    PLbytes = PLbytes[5:-2]

    #output are doubles, so 8 bytes. convert string to bytes and assign to variables
    PLbytesx = bytes(PLbytes[0:8])
    PLbytesy = bytes(PLbytes[8:16])
    PLbytesz = bytes(PLbytes[16:24])
    tumblebytesx = bytes(PLbytes[24:32])
    tumblebytesy = bytes(PLbytes[32:40])
    tumblebytesz = bytes(PLbytes[40:48])

    #convert bytes to doubles
    M_DESx = struct.unpack('d', PLbytesx)[0]
    M_DESy = struct.unpack('d', PLbytesy)[0]
    M_DESz = struct.unpack('d', PLbytesz)[0]
    tumblex = struct.unpack('d', tumblebytesx)[0]
    tumbley = struct.unpack('d', tumblebytesy)[0]
    tumblez = struct.unpack('d', tumblebytesz)[0]

    #return output
    return [M_DESx, M_DESy, M_DESz, tumblex, tumbley, tumblez]

#code below is generated to run the above function from a matlab file            
if __name__ == "__main__":
    x1 = float(sys.argv[1])
    y1 = float(sys.argv[2])
    z1 = float(sys.argv[3])
    x2 = float(sys.argv[4])
    y2 = float(sys.argv[5])
    z2 = float(sys.argv[6])
    sys.stdout.write(str(detumblealgo(x1,y1,z2,x2,y2,z2)))