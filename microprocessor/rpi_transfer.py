import datetime
import string
import scipy.io as sio
import struct

TRANSFER = 1 # 0 means do not enable TCP transport, 1 means enable
SAVE = 0 # 0 means no storage is enabled, 1 means it is enabled. Using mat file storage method

chip_num = 1 # chip num of ads1299

pkg_length = 160

if(TRANSFER==1):
    import socket
    import json
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # client.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65536)
    # send_buff = client.getsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF)
    # print(send_buff)
    client.connect( ('169.254.3.90', 8080) )
    # client.connect( ('127.0.0.1', 8080) )


if(SAVE==1):
    save_pkg_num = 25 
    save_data = []
    # save_status = []

fifo_name = "adc.fifo"
fifo = open(fifo_name, "r")


class Datapool:
    def __init__(self):
        self.pkgnum = 1 
        self.dec_data = [] 
        self.time_stamp = [] 
        self.status = [] 
    def next_pkg(self):
        self.pkgnum += 1
        self.dec_data = []
        self.time_stamp = []
        self.status = []

datapool = Datapool()
while 1:
    datapool.time_stamp.append(str(datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')))
    for i in range(0,pkg_length): # 0~pkg_length-1, 0~159
        line = fifo.readline() 
        line = line.strip('\n') 
        array = line.split(',') 
        
        data = []
        status = []
        for chip_no in range(0,chip_num):
            data_temp = list(map(float, array[1+9*chip_no:9+9*chip_no])) 
            data.extend(data_temp)

            try:
                status_temp = int(array[0+9*chip_no])
                status.append(status_temp)
            except Exception:
                # status_temp = null
                status = []
                # pass
            
            
            # print('channel:', type(data), len(data),data)
            # print('status:', stuatus)
            # print('channel:', data)

        # print('channel:', type(data), len(data),data)

        datapool.dec_data.append(data)
        datapool.status.append(status)
    
    if(datapool.status[-1]==[]):
        break


    # print('pkg_num', datapool.pkgnum)
    # print('time_stamp:', datapool.time_stamp)
    # print('status:', datapool.status)
    # print('dec_data:', len(datapool.dec_data), 'X' , len(datapool.dec_data[0]), '\n', datapool.dec_data)
    
    if(TRANSFER==1):
        
        send_data = (json.dumps(datapool.__dict__)).encode('utf-8')

        print('length', len(send_data))
        x = struct.pack('i', len(send_data))
        client.send(x)
        # client.send( str(len(send_data)).encode('utf-8') )
        # send_buff = client.getsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF)
        # print('buff', send_buff)
        client.send( send_data )
        print('sent:', datapool.pkgnum)
    
    if(SAVE==1):
        save_data.extend(datapool.dec_data)
        # print(datapool.pkgnum%save_pkg_num)
        if(datapool.pkgnum%save_pkg_num==0):
            sio.savemat('data'+str(datapool.pkgnum/save_pkg_num)+'.mat', mdict={'data': save_data})
            print(len(save_data), 'X', len(save_data[0]))
            save_data = []
    
    datapool.next_pkg()


fifo.close()
