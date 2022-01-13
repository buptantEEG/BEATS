import os
import scipy.io as sio
import numpy as np

path = './test01/'
files = os.listdir(path)
files.sort(key=lambda x: int(x[4:-4]))
print(files)
for file in files:
    if file == files[0]:
        dataFile = sio.loadmat(path + file)
        print(dataFile['data'])
        data = dataFile['data']
    else:
        tempFile = sio.loadmat(path + file)
        tempData = tempFile['data']
        data = np.vstack((data, tempData))
sio.savemat('./test_01_segment_stacked.mat', mdict={'data': data})
