import datetime
import string
import scipy.io as sio
import struct

# MATLAB可以兼容之前的MATLAB

TRANSFER = 1 # 0表示不启用TCP传输，1表示启用
SAVE = 0 # 0表示不启用存储，1表示启用。采用mat文件存储方式

chip_num = 1 # ads1299的片数

pkg_length = 160

if(TRANSFER==1):
    import socket
    import json
    # 先连接服务器，再打开文件
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # client.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65536)
    # send_buff = client.getsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF)
    # print(send_buff)
    client.connect( ('169.254.3.90', 8080) )
    # client.connect( ('127.0.0.1', 8080) )


if(SAVE==1):
    save_pkg_num = 25 # 可调参数,收到这么多包之后存为mat文件
    save_data = []
    # save_status = []

fifo_name = "adc.fifo"
fifo = open(fifo_name, "r")

# 用来存储数据的类
class Datapool:
    def __init__(self):
        self.pkgnum = 1 # 每个数据包编号
        self.dec_data = [] # 转化后的十进制信号值
        self.time_stamp = [] # 时间戳，一个数据包一个
        self.status = [] # 芯片返回的状态位
    def next_pkg(self):
        self.pkgnum += 1
        self.dec_data = []
        self.time_stamp = []
        self.status = []


# TODO : 将数据每160个点发送一次，可以采用队列，两个进程。
datapool = Datapool()
while 1:
    # HACK(LeBoi): 时间戳功能需要由采集数据端来完成，此处只是为了暂时能和MATLAB端接上
    datapool.time_stamp.append(str(datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')))
    for i in range(0,pkg_length): # 0~pkg_length-1, 0~159
        line = fifo.readline() # line是一个list，每个元素为string
        line = line.strip('\n') # 去掉换行符\n
        array = line.split(',') # array是一个list，每个元素为string,有chip_num*9个元素
        
        data = []
        status = []
        for chip_no in range(0,chip_num):
            # 索引范围是[a,b),包含a不含b
            data_temp = list(map(float, array[1+9*chip_no:9+9*chip_no])) # 把[1:9)的元素转化为float
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

        datapool.dec_data.append(data) # 索引范围是[a,b),包含a不含b
        datapool.status.append(status)
    
    # # 测试用语句,退出。之后可以改为对FIFO文件状态的判断
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


# 养成良好习惯，最后关闭文件
fifo.close()
