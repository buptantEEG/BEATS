# _*_ coding:utf-8 _*_
import json
import os
import queue
import socket
import struct
import sys
import threading
import time
import numpy as np
import scipy.io as sio
from PyQt5 import QtWidgets
from PyQt5.QtCore import QTimer, pyqtSignal
from PyQt5.QtWidgets import QApplication, QMessageBox
from pyqtgraph import mkPen
from scipy import signal
from scipy.signal import detrend
import pyqtgraph as pg
import stop_thread
from Detection_system_Lite import Ui_MainWindow
import socket

DATA_LIST_TEMP_CH = [[] for i in range(8)]  # data pool
DATA_QUEUE_TEMP_CH = queue.Queue()  # Queues for data storage


class detection_window_1(QtWidgets.QMainWindow, Ui_MainWindow):
    """
    窗口类
    """
    global DATA_LIST_TEMP_CH, DATA_QUEUE_TEMP_CH
    tcp_recv_signal = pyqtSignal(bytes)

    def __init__(self):
        super(detection_window_1, self).__init__()
        self.channel_num = 8  # quantity of channels
        if self.channel_num > 8:
            DATA_LIST_TEMP_CH.extend([] for i in range(self.channel_num - 8))  #update data pool

        self.time_temp = 0
        self.fresh_interval = 10  # clock refresh time (ms)
        self.pdata = 0
        self.temp_len_data = 0
        self.temp_ll = 0
        self.sample_rate = 250  # sampling rates
        self.package_length = 50
        self.plot_time = 5
        self.plot_length = self.sample_rate * self.plot_time  # data length to show
        self.save_pkg_length = self.sample_rate / self.package_length * 5
        self.offset = -0.001  # curve distance
        # self.offset_list=[[j*self.offset for i in range(self.package_length)]for j in range(self.channel_num)]
        # print(len(self.offset_list[0]))
        self.offset_list = [self.offset * i for i in range(self.channel_num)]
        self.is_filt = True
        self.is_detrend = True
        self.is_save_data = True

        # set coordinate
        self.x_list = [0.5 * self.sample_rate * i for i in range(2 * self.plot_time + 1)]
        self.x_labels = [str(0.5 * i) for i in range(2 * self.plot_time + 1)]
        self.y_list = [self.offset * i for i in range(self.channel_num)]
        self.y_labels = ['ch' + str(i + 1) for i in range(self.channel_num)]

        self.xticks = [(i, j) for i, j in zip(self.x_list, self.x_labels)]
        self.yticks = [(i, j) for i, j in zip(self.y_list, self.y_labels)]

        self.xstrAxes = pg.AxisItem(orientation='bottom')
        self.xstrAxes.setTicks([self.xticks])

        self.ystrAxes = pg.AxisItem(orientation='left')
        self.ystrAxes.setTicks([self.yticks])

        # initial stimulation parameters
        self.start_time = 0
        self.sti_times = []
        self.sti_event = []
        self.sti_point = []
        # initial UI
        # pg.setConfigOption('background', 'w')
        # pg.setConfigOption('foreground', 'r')
        self.setupUi(self)

        # timers for EEG signal updating
        self.timer_EEG = QTimer()
        self.timer_EEG.timeout.connect(self.update)

        self.pushButton.setEnabled(True)
        self.pushButton_2.setEnabled(False)

        # filters
        # self.b, self.a = signal.butter(5, [2 * 1 / self.sample_rate, 2 * 50 / self.sample_rate], 'bandpass')
        self.b, self.a = signal.butter(5, 2 * 30 / self.sample_rate, 'lowpass')

        # front overlap 
        self.overlap_begin = [[] for i in range(self.channel_num)]
        self.overlap_end = [[] for i in range(self.channel_num)]
        self.before_data = [[] for i in range(self.channel_num)]
        self.savecount = 0
        self.save_data_queue = queue.Queue()
        
        self.data = [[] for i in range(self.channel_num)]
        self.curves = []
        for i in range(self.channel_num):
            self.data[i] = [i * self.offset] * self.plot_length

        # initial drawing parameters
        self.graphicsView.setDownsampling(mode='subsample')
        self.graphicsView.setRange(
            yRange=[(self.offset * (self.channel_num - 1)) + 0.2 * self.offset, -0.2 * self.offset])
        # self.graphicsView.setRange(
        #     xRange=[0.0, self.plot_length])
        self.graphicsView.setLabel('left', 'channels')
        self.graphicsView.setAxisItems(axisItems={'bottom': self.xstrAxes, 'left': self.ystrAxes})
        self.graphicsView.setLabel('bottom', 'points')

        self.pens = [(255, 0, 0), (255, 130, 71), (255, 215, 0), (0, 255, 0), (0, 0, 255), (0, 255, 255),
                     (160, 32, 240),(255,255,255)]

        # initial curves for each channel
        for i in range(self.channel_num):
            self.curves.append(self.graphicsView.plot(
                pen=mkPen(width=2,color=self.pens[i]),  # 画笔颜色
            ))

        self.ptr1 = 0
        for i in range(self.channel_num):
            self.curves[i].setData(self.data[i])

        # TCP/IP parameters

        hostname = socket.gethostname()
        self.client_ip = socket.gethostbyname(hostname)
        self.serverIP = '0.0.0.0'
        self.serverPort = 8080
        self.server_addr = (self.serverIP, self.serverPort)
        self.sockServer = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.BUFFER_SIZE = 8 * 1024
        # initial TCP/IP
        self.tcp_server_init()

        # bind click events
        self.click_events()

        # initial thread parameter
        self.EEG_is_running = True

        # thread for suspend
        self.stopEvent = threading.Event()
        self.stopEvent.clear()

        # thread for ending
        self.stopEvent_EEG = threading.Event()
        self.stopEvent_EEG.clear()

    def click_eeg_start_button(self):
        """
        Turn on signal acceptance and draw after receiving the signal
        """
        print(self.client_ip)
        self.pushButton.setEnabled(False)
        self.pushButton_2.setEnabled(True)
        print("EEG signal starting...")
        self.statusBar.showMessage('EEG signal starting...')

        self.tcp_recv_thread = threading.Thread(target=self.thread_tcp_server, args=())
        self.tcp_recv_thread.start()
        # print('Showing EEG signal...')

        self.timer_EEG = QTimer()
        self.timer_EEG.timeout.connect(self.update_curves)
        self.timer_EEG.start(self.fresh_interval)  

        if self.is_save_data:
            self.save_thread = threading.Thread(target=self.save_data, args=())
            self.save_thread.start()

    def click_eeg_stop_button(self):
        """
        Stop data reception and display
        """
        self.pushButton.setEnabled(True)
        self.pushButton_2.setEnabled(False)
        self.stopEEGEventHandle()
        self.statusBar.showMessage('Paused')

    def click_eeg_STI_button(self):
        self.sti_times.append(time.time())
        self.sti_event.append(self.comboBox.currentText())
        # if self.sti_times:
        #     print((self.sti_times[-1] - self.start_time) * self.sample_rate)
        # print(self.sti_times)
        # print(self.sti_event)
        print(time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(self.sti_times[-1])))

    def click_eeg_undo_button(self):
        if self.sti_times:
            self.sti_times.pop()
            self.sti_event.pop()
        else:
            pass
        print(self.sti_times)
        print(self.sti_event)

    def click_eeg_localtest_button(self):
        path = ''
        files = os.listdir(path)
        files.sort(key=lambda x: int(x[4:-4]))
        print(files)
        for file in files:
            if file == files[0]:
                dataFile = sio.loadmat(path + file)
                data = dataFile['data']
                self.start_time = dataFile['time_stamp'][0][0]
                print("start time: ", self.start_time)
            else:
                tempFile = sio.loadmat(path + file)
                tempData = tempFile['data']
                data = np.vstack((data, tempData))

        for i in range(len(self.sti_times)):
            self.sti_point.append(int(round(self.sti_times[i] - self.start_time) * self.sample_rate))

        sio.savemat('./local_save_test.mat',
                    mdict={'data': data, 'events': self.sti_event, 'event_time': self.sti_point,
                           'exp_start_time': time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time()))})
        self.statusBar.showMessage('saved')
        print("saved!")

    def click_events(self):
        """
        bind click events with buttons
        :return:
        """
        self.pushButton.clicked.connect(self.click_eeg_start_button)
        self.pushButton_2.clicked.connect(self.click_eeg_stop_button)
        self.pushButton_3.clicked.connect(self.click_eeg_STI_button)
        self.pushButton_4.clicked.connect(self.click_eeg_undo_button)
        # self.pushButton_5.clicked.connect(self.click_eeg_localtest_button)

    def stopEEGEventHandle(self):
        """
        suspend 
        :return:
        """
        if not self.EEG_is_running:
            return
        self.EEG_is_running = False
        # self.stopEvent_EEG.set()
        # print("stopEvent set")
        self.timer_EEG.stop()
        if self.is_save_data == 1:
            self.save_data_queue.put('end')

    def update_curves(self):
        # if not DATA_LIST_TEMP_CH[1] == []:
        # print('data length:',len(DATA_LIST_TEMP_CH1))
        if not DATA_LIST_TEMP_CH[1] == []:
            # data = DATA_QUEUE_TEMP_CH.get()
            ll = len(DATA_LIST_TEMP_CH[0])
            # print("ll",ll,"temp_len",self.temp_len_data)
            if ll > self.temp_ll:  # check if new data or not
                if ll <= self.plot_length: 
                    if self.is_detrend:
                        for i in range(self.channel_num):
                            self.data[i] = list(np.array(detrend(DATA_LIST_TEMP_CH[i])) + self.offset_list[i])
                    elif not self.is_detrend:
                        for i in range(self.channel_num):
                            # self.data[i] = list(map(lambda x: x + self.offset * i, DATA_LIST_TEMP_CH[i]))
                            self.data[i] = list(np.array(DATA_LIST_TEMP_CH[i]) + self.offset_list[i])
                    # print(len(self.data[0]))
                elif ll > self.plot_length:
                    if self.is_detrend:
                        for i in range(self.channel_num):
                            self.data[i] = list(
                                np.array(detrend(DATA_LIST_TEMP_CH[i][-self.plot_length:-1])) + self.offset_list[i])
                            # self.data[i] = list(np.array(detrend(DATA_LIST_TEMP_CH[i][self.pdata:self.pdata +
                            # self.plot_length])) + self.offset_list[i])
                    elif not self.is_detrend:
                        for i in range(self.channel_num):
                            self.data[i] = list(
                                np.array(DATA_LIST_TEMP_CH[i][-self.plot_length:-1]) + self.offset_list[i])
                            # self.data[i] = list(np.array(DATA_LIST_TEMP_CH[i][self.pdata:self.pdata +
                            # self.plot_length]) + self.offset_list[i])
                    # print(len(self.data[0]))
                    self.pdata += self.package_length
                self.temp_ll = ll
                # print(self.pdata)

                # clear data
                if self.pdata >= 2 * self.plot_length:
                    # print("p_data",self.pdata,"plot_length",self.plot_length)
                    # print("clear")
                    self.clear_timer(self.pdata)
                    self.pdata = 0
                    self.temp_ll = 0
                    # print("time:", time.time() - self.time_temp)
                    # self.time_temp = time.time()

                # update curve
                for i in range(self.channel_num):
                    self.curves[i].setData(self.data[i])
                    self.data[i] = []

                # reset x origin 
                # for i in range(self.channel_num):
                #     self.curves[i].setPos(self.pdata, 0)
            self.temp_len_data = ll

    def clear_timer(self, pt):
        """
        For data saving and memory cleaning
        :param pt: Pointer to the next data of the data that needs to be saved
        """
        for i in range(self.channel_num):
            DATA_LIST_TEMP_CH[i] = DATA_LIST_TEMP_CH[i][pt:]

    def save_data(self):
        counter = 1
        all_data = []
        all_status = []
        while 1:
            if not self.save_data_queue.empty():
                datapool = self.save_data_queue.get()
                if (datapool == 'end'):
                    # print('save_pkg_length ', save_pkg_length)
                    # if not longer than save_pkg_length，the data is not saved automatically and save it here
                    sio.savemat('./Data/data' + str(int(counter / self.save_pkg_length) + 1) + '.mat',
                                mdict={'data': all_data, 'status': all_status, 'time_stamp': start_time_stamp})
                    all_data = []
                    all_status = []
                    break
                else:
                    # print('p_save ', type(datapool), len(datapool))
                    if (counter == 1):
                        start_time_stamp = datapool['time_stamp']
                    all_data.extend(datapool['dec_data'])
                    all_status.extend(datapool['status'])
                    if (counter % self.save_pkg_length == 0):
                        sio.savemat('./Data/data' + str(int(counter / self.save_pkg_length)) + '.mat',
                                    mdict={'data': all_data, 'status': all_status, 'time_stamp': start_time_stamp})
                        all_data = []
                        all_status = []
                    counter += 1

        self.statusBar.showMessage('saving')
        path = './Data/'
        files = os.listdir(path)
        files.sort(key=lambda x: int(x[4:-4]))
        print(files)
        for file in files:
            if file == files[0]:
                dataFile = sio.loadmat(path + file)
                data = dataFile['data']
                self.start_time = dataFile['time_stamp'][0][0]
                print("start time: ", self.start_time)
            else:
                tempFile = sio.loadmat(path + file)
                tempData = tempFile['data']
                data = np.vstack((data, tempData))
        print(self.sti_times)
        for i in range(len(self.sti_times)):
            self.sti_point.append(int(round((self.sti_times[i] - self.start_time) * self.sample_rate)))

        sio.savemat('../Data/data_stacked/stacked.mat',
                    mdict={'data': data, 'events': self.sti_event, 'event_time': self.sti_point,
                           'exp_start_time': time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time()))})
        self.statusBar.showMessage('saved')
        print("saved!")

    def tcp_server_init(self):
        """
        初始化
        """
        self.sockServer.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sockServer.bind(self.server_addr)
        print('ready for receiving!')

    def thread_tcp_server(self):
        """
        thread for tcp server
        :return:
        """
        try:
            self.statusBar.showMessage('connecting...')
            self.sockServer.listen(10)
            conn, client_addr = self.sockServer.accept()
            self.statusBar.showMessage('connection success...')

        except Exception as e:
            print(e)
            pass

        while self.EEG_is_running:
            receive_length = conn.recv(4)
            if receive_length == b'':
                break
            data_length = struct.unpack('i', receive_length)[0]
            # print(data_length)
            # dataRecv = (clientSock.recv(data_length)).decode('utf-8')

            # Python socket can only read all the data of the buffer at most at one time.
            # If the size of the specified data packet is larger than the size of the buffer, 
            # the valid data read is only the data of the buffer.
            # If it can be determined that the data sent is larger than the size of the buffer, multiple times are required:
            # socket.recv(receiverBufsize)
            # and then the received data is spliced into a complete data packet and then parsed.
            receive_data = []
            while data_length > 0:
                if data_length > self.BUFFER_SIZE:
                    temp = conn.recv(self.BUFFER_SIZE)
                else:
                    temp = conn.recv(data_length)
                receive_data.append(temp)
                data_length = data_length - len(temp)
            receive_data = b''.join(receive_data).decode('utf-8')
            # reference: https://www.cnblogs.com/luckgopher/p/4816919.html
            temp_data = json.loads(receive_data)
            # display_queue.put(data)

            data = list(map(list, zip(*temp_data["dec_data"])))
            data = data[0:self.channel_num]
            # if len(DATA_LIST_TEMP_CH[0])==0:
            #     timetemp1=time.time()
            # elif len(DATA_LIST_TEMP_CH[0])%32000==0:
            #     print(time.time()-timetemp1)

            # detrend_data=detrend(data)
            # self.overlap_end = [i[::-1] for i in data]

            if self.is_filt:
                stackdata = np.hstack(
                    (np.array(self.overlap_begin), np.array(self.before_data), np.array(self.overlap_end)))
                # print(stackdata.shape )
                listdata = stackdata.tolist()
                # print("data:",len(listdata[1]))
                # print(len(listdata[1]))
                if listdata[1][self.package_length:-self.package_length]:
                    filtedData = signal.filtfilt(self.b, self.a, listdata)  
                    # print("filtdata:",len(filtedData[1]))
                    filtedData = [i[self.package_length:-self.package_length] for i in filtedData]
                    # print("no_overlap_data:",len(filtedData[1]))
                    for i in range(self.channel_num):
                        # DATA_LIST_TEMP_CH[i].extend(detrend(filtedData[i]))
                        DATA_LIST_TEMP_CH[i].extend(filtedData[i])
                    # DATA_QUEUE_TEMP_CH.put(filtedData)
                    # print(1)
                    # temp_data["dec_data"] = np.array(filtedData).T
                    if self.is_save_data:
                        self.save_data_queue.put(temp_data)

                self.overlap_begin = self.before_data
                # self.before_data = [i[-50:] for i in data]
                self.before_data = self.overlap_end
                self.overlap_end = data
                # print(data)


            elif not self.is_filt:
                for i in range(self.channel_num):
                    DATA_LIST_TEMP_CH[i].extend(data[i])
                # DATA_QUEUE_TEMP_CH.put(data)
                # print(1)
                if self.is_save_data:
                    self.save_data_queue.put(temp_data)

    def tcp_close(self):
        """
        Function to close the network connection
        """
        if self.sockServer:
            self.sockServer.close()

    def closeEvent(self, event):
        # Create a message box (prompt box)
        quitMsgBox = QMessageBox()
        quitMsgBox.setWindowTitle('warn')
        quitMsgBox.setText('Sure to quit?')
        quitMsgBox.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
        buttonY = quitMsgBox.button(QMessageBox.Yes)
        buttonY.setText('yes')
        buttonN = quitMsgBox.button(QMessageBox.No)
        buttonN.setText('no')
        quitMsgBox.exec_()
        if quitMsgBox.clickedButton() == buttonY:
            self.stopEEGEventHandle()
            try:
                stop_thread.stop_thread(self.tcp_recv_thread)
            except Exception as e:
                print(e)
                pass
            self.tcp_close()
            event.accept()
        else:
            event.ignore()


if __name__ == '__main__':
    # SAVE=1

    app = QApplication(sys.argv)
    MainWindow = detection_window_1()
    MainWindow.setStyleSheet("#MainWindow{background-color: black}")
    MainWindow.show()
    sys.exit(app.exec_())
