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
# from PyQt5 import QtWidgets
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import QTimer, pyqtSignal
from PyQt5.QtWidgets import QApplication, QMessageBox
from pyqtgraph import mkPen
from PyQt5.QtCore import Qt
from scipy import signal
from scipy.signal import detrend
import pyqtgraph as pg
import stop_thread
from Detection_system_Lite import Ui_MainWindow
from WaveformWindow import  Ui_Form
import socket
from scipy import fftpack

from pyqtgraph import PlotWidget

DATA_LIST_TEMP_CH = [[]]  # data pool
DATA_QUEUE_TEMP_CH = queue.Queue()  # Queues for data storage


class Main_window(QtWidgets.QMainWindow, Ui_MainWindow):
    """
    窗口类
    """
    global DATA_LIST_TEMP_CH, DATA_QUEUE_TEMP_CH

    def __init__(self):
        super(Main_window, self).__init__()
        desktop = QApplication.desktop()
        # 通过桌面的宽和高来比例位置显示
        # print(desktop.width(),desktop.height())
        self.move(int(desktop.width() *0.01), int(desktop.height() * 0.01))

        self.setupUi(self)
        self.channelComboBox.setCurrentIndex(7)  # 设置默认值
        self.channel_num = int(self.channelComboBox.currentText())  # quantity of channels
        if self.channel_num > 1:
            DATA_LIST_TEMP_CH.extend([] for i in range(self.channel_num - 1))  #update data pool
        self.time_temp = 0
        self.fresh_interval = 1  # clock refresh time (ms)
        self.pdata = 0
        self.temp_len_data = 0
        self.temp_ll = 0
        self.sample_rate = 4000  # sampling rates
        self.package_length = 800
        self.plot_time = 4

        self.plot_length = self.sample_rate * self.plot_time  # data length to show


        self.offset = 0.01  # curve distance
        self.is_connected=False
        # self.offset_list=[[j*self.offset for i in range(self.package_length)]for j in range(self.channel_num)]
        # print(len(self.offset_list[0]))

        self.is_filt = False
        self.is_detrend = False
        self.is_plotting = True
        self.is_save_data = False

        #initial check_box
        self.showWaveBox.setChecked(True)
        # self.showFFTBox.setChecked(False)

        # initial stimulation parameters
        self.start_time = 0
        self.sti_times = []
        self.sti_event = []
        self.sti_point = []
        # initial UI
        # pg.setConfigOption('background', 'w')
        # pg.setConfigOption('foreground', 'r')

        self.wave_window = Wave_window(self)
        self.wave_window.show()


        # timers for EEG signal updating
        self.timer_EEG = QTimer()
        self.timer_EEG.timeout.connect(self.wave_window.update_curves)

        self.pushButton.setEnabled(True)
        self.pushButton_2.setEnabled(False)
        self.pushButton_3.setEnabled(False)
        self.pushButton_4.setEnabled(False)


        # filters
        # self.b, self.a = signal.butter(5, [2 * 1 / self.sample_rate, 2 * 30 / self.sample_rate], 'bandpass')
        self.b, self.a = signal.butter(5, 2 * 30 / self.sample_rate, 'lowpass')


        # front overlap
        self.overlap_begin = [[] for i in range(self.channel_num)]
        self.overlap_end = [[] for i in range(self.channel_num)]
        self.before_data = [[] for i in range(self.channel_num)]

        # TCP/IP parameters

        hostname = socket.gethostname()
        self.client_ip = socket.gethostbyname(hostname)
        self.serverIP = '0.0.0.0'
        self.serverPort = 8080
        self.server_addr = (self.serverIP, self.serverPort)
        self.is_socket_seted=False
        self.BUFFER_SIZE = 8 * 1024

        # initial save_data
        self.save_data_queue = queue.Queue()
        self.save_counter = 1
        self.save_all_data = []
        self.save_all_status = []
        self.save_start_time_stamp = 0
        self.save_time = 5
        self.save_pkg_length = self.sample_rate / self.package_length * self.save_time
        self.saving_data=False

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

    def click_events(self):
        """
        bind click events with buttons
        :return:
        """
        self.pushButton.clicked.connect(self.click_eeg_start_button)
        self.pushButton_2.clicked.connect(self.click_eeg_stop_button)
        self.pushButton_3.clicked.connect(self.click_eeg_STI_button)
        self.pushButton_4.clicked.connect(self.click_eeg_undo_button)
        self.showWaveBox.stateChanged.connect(self.open_wave_window)
        self.verticalSlider.valueChanged.connect(self.wave_window.change_scale_y)
        self.horizontalSlider.valueChanged.connect(self.wave_window.change_scale_x)
        self.saveDataBox.stateChanged.connect(self.set_is_savedata)

        self.channelComboBox.currentIndexChanged.connect(self.channel_num_change)
        # self.pushButton_5.clicked.connect(self.click_eeg_localtest_button)

    def channel_num_change(self):
        self.channel_num=int(self.channelComboBox.currentText())
        if self.is_plotting==True:
            # self.wave_window.hide()
            # self.wave_window=[]
            self.wave_window = Wave_window(self)
            self.verticalSlider.valueChanged.connect(self.wave_window.change_scale_y)
            self.horizontalSlider.valueChanged.connect(self.wave_window.change_scale_x)
            self.wave_window.show()

        if self.is_plotting==False:
            self.wave_window = Wave_window(self)
            self.verticalSlider.valueChanged.connect(self.wave_window.change_scale_y)
            self.horizontalSlider.valueChanged.connect(self.wave_window.change_scale_x)

    def set_is_savedata(self):
        if self.saveDataBox.checkState() == Qt.Checked:
            self.is_save_data=True
            if not os.path.exists('./Data'):
                os.mkdir('./Data')

        elif self.saveDataBox.checkState() == Qt.Unchecked:
            self.is_save_data=False


    def open_wave_window(self):
        if self.showWaveBox.checkState() == Qt.Checked:
            self.is_plotting=True
            self.statusBar.showMessage('Creating the Wave Window...')
            self.wave_window.show()

        elif self.showWaveBox.checkState() == Qt.Unchecked:
            self.is_plotting=False
            self.statusBar.showMessage('The Wave Window is closed.')
            self.wave_window.hide()




    def click_eeg_start_button(self):
        """
        Turn on signal acceptance and draw after receiving the signal
        """
        print(self.client_ip)
        self.tcp_server_init()

        for i in range(self.channel_num):
            self.wave_window.graphicsViews[i].setMenuEnabled(False)
        self.EEG_is_running=True
        self.pushButton.setEnabled(False)
        self.pushButton_2.setEnabled(True)
        self.pushButton_3.setEnabled(True)
        self.pushButton_4.setEnabled(True)
        self.saveDataBox.setEnabled(False)
        self.channelComboBox.setEnabled(False)

        # print("EEG signal starting...")
        self.statusBar.showMessage('EEG signal starting...')
        self.statusBar.show()

        self.tcp_recv_thread = threading.Thread(target=self.thread_tcp_server, args=())
        self.tcp_recv_thread.start()
        # print('Showing EEG signal...')

        self.timer_EEG = QTimer()
        self.timer_EEG.timeout.connect(self.wave_window.update_curves)
        self.timer_EEG.start(self.fresh_interval)

        self.timer_save_data = QTimer()
        self.timer_save_data.timeout.connect(self.save_data)

        if self.is_save_data:
            self.timer_save_data.start(self.save_time)
            self.saving_data=True
        # self.timer_save_data.start(self.save_time)

        # self.save_thread = threading.Thread(target=self.save_data, args=())
        # print("is_save_data:",self.is_save_data)


    def click_eeg_stop_button(self):
        """
        Stop data reception and display
        """
        self.pushButton.setEnabled(True)
        self.pushButton_2.setEnabled(False)
        self.EEG_is_running = False
        self.saveDataBox.setEnabled(True)
        self.channelComboBox.setEnabled(True)
        # self.tcp_recv_thread = []
        # print('Showing EEG signal...')
        stop_thread.stop_thread(self.tcp_recv_thread)

        self.stopEEGEventHandle()

        self.statusBar.showMessage('Receiving Stoped')

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
        self.statusBar.showMessage('Saved!')
        # print("saved!")

    def stopEEGEventHandle(self):
        """
        suspend
        :return:
        """
        # if self.EEG_is_running==True:
        #     self.EEG_is_running = False
        # self.stopEvent_EEG.set()
        # print("stopEvent set")
        self.timer_EEG.stop()

        if self.is_connected==True:
            stop_thread.stop_thread(self.tcp_recv_thread)

            self.tcp_close()
            if self.saving_data== True:
                self.saving_data = False
                self.save_data_queue.put('end')
                self.save_data()

        elif self.is_socket_seted==True:
            self.sockServer.close()




    def save_data(self):

        while self.save_data_queue.empty() == False:
        #
        # print(self.save_data_queue.empty())
        # self.statusBar.showMessage('saving data...')
            datapool = self.save_data_queue.get()
            if (datapool == 'end'):
                # print('save_pkg_length ', save_pkg_length)
                # if not longer than save_pkg_length，the data is not saved automatically and save it here
                if len(self.save_all_data)!=0:
                    sio.savemat('./Data/data' + str(int(self.save_counter / self.save_pkg_length) + 1) + '.mat',
                                mdict={'data': self.save_all_data, 'status': self.save_all_status, 'time_stamp':self.save_start_time_stamp})
                else:
                    # print('No more data~')
                    pass
                self.save_all_data = []
                self.save_all_status = []
                self.timer_save_data.stop()
                self.stack_data()
            else:
                # print('p_save ', type(datapool), len(datapool))
                if (self.save_counter == 1):
                    self.save_start_time_stamp = datapool['time_stamp']
                self.save_all_data.extend(datapool['dec_data'])
                self.save_all_status.extend(datapool['status'])
                if (self.save_counter % self.save_pkg_length == 0):
                    sio.savemat('./Data/data' + str(int(self.save_counter / self.save_pkg_length)) + '.mat',
                                mdict={'data': self.save_all_data, 'status': self.save_all_status, 'time_stamp': self.save_start_time_stamp})
                    self.save_all_data = []
                    self.save_all_status = []
                self.save_counter += 1
                # time.sleep(0.1)

    def stack_data(self):
        self.statusBar.showMessage('Stacking and saving data...')
        path = './Data/'
        files = os.listdir(path)
        if len(files)!=0:
            files.sort(key=lambda x: int(x[4:-4]))
            # print(files[0])

            for file in files:
                if file == files[0]:
                    dataFile = sio.loadmat(path + file)
                    data = dataFile['data']
                    self.start_time = dataFile['time_stamp'][0][0]
                    # print("start time: ", self.start_time)
                else:
                    tempFile = sio.loadmat(path + file)
                    tempData = tempFile['data']
                    data = np.vstack((data, tempData))
            # print(self.sti_times)
            for i in range(len(self.sti_times)):
                self.sti_point.append(int(round((self.sti_times[i] - self.start_time) * self.sample_rate)))

            sio.savemat('./data_stacked.mat',
                        mdict={'data': data, 'events': self.sti_event, 'event_time': self.sti_point,
                               'exp_start_time': time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time()))})
            self.statusBar.showMessage('All data saved!')
        # print("saved!")

    def tcp_server_init(self):
        """
        初始化
        """
        self.sockServer = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sockServer.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sockServer.bind(self.server_addr)
        self.is_socket_seted=True
        print('ready for receiving!')

    def thread_tcp_server(self):
        """
        thread for tcp server
        :return:
        """
        try:
            self.statusBar.showMessage('Connecting...')
            self.sockServer.listen(10)
            self.conn, client_addr = self.sockServer.accept()
            self.statusBar.showMessage('Connection success!')
            self.is_connected=True

        except Exception as e:
            print(e)
            pass

        while (self.EEG_is_running==True and self.is_connected==True):
            # print(self.is_connected)
            receive_length = self.conn.recv(4)
            # print('eee')
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
                    temp = self.conn.recv(self.BUFFER_SIZE)
                else:
                    temp = self.conn.recv(data_length)
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
                    # if self.is_save_data:
                    #     self.save_data_queue.put(temp_data)

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
        # if self.sockServer:
        self.is_connected = False
        self.conn.close()


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
            # try:
            #     stop_thread.stop_thread(self.tcp_recv_thread)
            # except Exception as e:
            #     print(e)
            #     pass
            # print(self.is_connected)
            # if self.is_connected==True:
            #     self.tcp_close()
            event.accept()
            os._exit(0)
        else:
            event.ignore()

class Wave_window(QtWidgets.QWidget,Ui_Form):
    global DATA_LIST_TEMP_CH, DATA_QUEUE_TEMP_CH

    def __init__(self,Father):
        super(Wave_window, self).__init__()
        # set coordinate
        self.setupUi(self)
        self.father = Father
        self.sample_rate = self.father.sample_rate
        self.temp_ll=Father.temp_ll
        self.plot_time = self.father.plot_time
        self.plot_length = self.father.plot_length
        self.channel_num = self.father.channel_num
        self.yoffset = self.father.offset
        self.freqs = fftpack.fftfreq(self.plot_length) * self.sample_rate
        # print(self.freqs.shape)
        self.time_temp = self.father.time_temp
        self.fresh_interval = self.father.fresh_interval  # clock refresh time (ms)
        self.pdata = self.father.pdata
        self.temp_len_data = self.father.temp_len_data
        self.package_length = self.father.package_length
        self.is_detrend = 0

        self.x_list = [0.5 * self.sample_rate * i for i in range(2 * self.plot_time + 1)]
        self.x_labels = [str(0.5 * i) for i in range(2 * self.plot_time + 1)]
        self.xticks = [(i, j) for i, j in zip(self.x_list, self.x_labels)]
        self.xstrAxes = pg.AxisItem(orientation='bottom')
        self.xstrAxes.setTicks([self.xticks])
        # -----------------------------------------------------------------------------------------
        self.isFilterBox.stateChanged.connect(self.filtData)
        self.isDetrendBox.stateChanged.connect(self.detrendData)
        # -----------------------------------------------------------------------------------------
        self.widgets = [self.widget]
        self.horizontalLayouts = [self.horizontalLayout]
        self.channel_checkboxs = [self.checkBox]
        self.FFTViews=[self.FFTView]
        # -----------------------------------------------------------------------------------------
        self.graphicsViews=[self.signalView]
        # initial drawing parameters
        for i in range(self.channel_num-1):
            self.widgets.append(QtWidgets.QWidget(self))

            self.horizontalLayouts.append(QtWidgets.QHBoxLayout(self.widgets[i+1]))
            self.horizontalLayouts[i+1].setContentsMargins(0, 0, 0, 0)

            self.graphicsViews.append(PlotWidget(self.widgets[i+1]))
            self.FFTViews.append(PlotWidget(self.widgets[i+1]))

            self.channel_checkboxs.append(QtWidgets.QCheckBox(self))



        # print(len(self.graphicsViews))
        # print(len(self.FFTViews))
        # print(len(self.widgets))
        # print(len(self.horizontalLayouts))

        self.signal_curves=[]
        self.fft_curves = []
        self.pens = [(255, 0, 0), (255, 130, 71), (255, 215, 0), (0, 255, 0), (0, 0, 255), (0, 255, 255),
                     (160, 32, 240), (0, 0, 0),(255, 0, 0), (255, 130, 71), (255, 215, 0), (0, 255, 0), (0, 0, 255), (0, 255, 255),
                     (160, 32, 240), (0, 0, 0),(255, 0, 0), (255, 130, 71), (255, 215, 0), (0, 255, 0), (0, 0, 255), (0, 255, 255),
                     (160, 32, 240), (0, 0, 0)]

        self.selected_channels_name=[]
        self.data = [[] for i in range(self.channel_num)]
        self.fftdata = [[] for i in range(self.channel_num)]

        for i in range(self.channel_num):
            self.selected_channels_name.append(i)
            # self.horizontalLayouts[i].setContentsMargins(0, 0, 0, 0)
            font = QtGui.QFont()
            font.setFamily("Times New Roman")
            font.setPointSize(20)
            brush = QtGui.QBrush(QtGui.QColor(255, 255, 255))
            brush.setStyle(QtCore.Qt.NoBrush)

            self.graphicsViews[i].setFont(font)
            self.graphicsViews[i].setLineWidth(1)
            self.graphicsViews[i].setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
            self.graphicsViews[i].setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
            self.graphicsViews[i].setBackgroundBrush(brush)
            self.graphicsViews[i].setDragMode(QtWidgets.QGraphicsView.NoDrag)
            self.graphicsViews[i].setMenuEnabled(False)
            self.graphicsViews[i].setMouseEnabled(False,False)
            self.graphicsViews[i].setObjectName("graphicsView")

            self.horizontalLayouts[i].addWidget(self.graphicsViews[i])


            self.FFTViews[i].setFont(font)
            self.FFTViews[i].setLineWidth(1)
            self.FFTViews[i].setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
            self.FFTViews[i].setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
            self.FFTViews[i].setBackgroundBrush(brush)
            self.FFTViews[i].setDragMode(QtWidgets.QGraphicsView.NoDrag)
            self.FFTViews[i].setObjectName("FFTView")
            self.FFTViews[i].setMenuEnabled(False)
            self.FFTViews[i].setMouseEnabled(False, False)
            self.horizontalLayouts[i].addWidget(self.FFTViews[i])


            # self.horizontalLayouts[i].setObjectName("horizontalLayout")
            self.horizontalLayouts[i].setStretch(0, 4)
            self.horizontalLayouts[i].setStretch(1, 1)
            # self.verticalLayout.addLayout(self.oneChannelLayouts[i])

            # print(self.widgets[i])
            self.ChannelsLayout.addWidget(self.widgets[i])

            self.channel_checkboxs[i].setObjectName('checkBox_channel'+str(i+1))
            self.channel_checkboxs[i].setText("Channel"+str(i+1))
            self.channelSelectLayout.addWidget(self.channel_checkboxs[i])


            self.channel_checkboxs[i].stateChanged.connect(self.plot_channel_change)
            # self.channel_checkboxs[i].stateChanged.connect(self.chaself.plot_channel_change(i))
            self.channel_checkboxs[i].setChecked(True)

            self.graphicsViews[i].setDownsampling(mode='subsample')
            self.graphicsViews[i].setRange(
                yRange=[-self.yoffset,self.yoffset,])
            # self.graphicsView.setRange(
            #     xRange=[0.0, self.plot_length])
            self.graphicsViews[i].setLabel('left', f'channel{i+1}')
            # self.graphicsViews[i].setAxisItems(axisItems={'bottom': self.xstrAxes, 'left': self.ystrAxes})

            self.data[i] = [0] * self.plot_length
            self.fftdata[i]=np.abs(fftpack.fft(self.data[i])*2*np.pi/self.plot_length)


            # initial curves for each channel
            self.signal_curves.append(self.graphicsViews[i].plot(
                pen=mkPen(width=1, color=self.pens[i]),  # 画笔颜色
            ))
            self.fft_curves.append(self.FFTViews[i].plot(
                pen=mkPen(width=1, color=self.pens[i]),  # 画笔颜色
            ))

            self.ptr1 = 0
            self.signal_curves[i].setData(self.data[i])
            self.fft_curves[i].setData(self.freqs, self.fftdata[i])


        # self.b, self.a = signal.ellip(6, 0.1 , 60, [5, 45],
        #                               btype='bandpass',fs=self.sample_rate)
        # for i in range(len(self.channel_checkboxs)):
        #     self.channel_checkboxs[i].setChecked(False)

        self.graphicsViews[-1].setLabel('bottom', 'Time')
        # print(len(self.widgets))
        # -----------------------------------------------------------------------------------------

    def plot_channel_change(self):

        for i in range(len(self.channel_checkboxs)):
            if self.channel_checkboxs[i].checkState()==Qt.Unchecked:
                # print(str(i)+'not checked')
                self.widgets[i].hide()
                # pass
            elif self.channel_checkboxs[i].checkState()==Qt.Checked:
                self.widgets[i].show()
                # print(str(i)+'checked')
                # pass

        # print(self.selected_channels_name)

    def filtData(self):
        if self.isFilterBox.checkState() == Qt.Checked:
            self.father.is_filt=1
        elif self.isFilterBox.checkState() == Qt.Unchecked:
            self.father.is_filt=0

    def detrendData(self):
        if self.isDetrendBox.checkState() == Qt.Checked:
            self.is_detrend=1
        elif self.isDetrendBox.checkState() == Qt.Unchecked:
            self.is_detrend=0

    def clear_timer(self, pt):
        """
        For data saving and memory cleaning
        :param pt: Pointer to the next data of the data that needs to be saved
        """
        for i in range(self.channel_num):
            DATA_LIST_TEMP_CH[i] = DATA_LIST_TEMP_CH[i][pt:]

    def update_curves(self):
        if self.father.showWaveBox.checkState()== Qt.Checked:

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
                                self.data[i] = list(np.array(detrend(DATA_LIST_TEMP_CH[i])))
                        elif not self.is_detrend:
                            for i in range(self.channel_num):
                                # self.data[i] = list(map(lambda x: x + self.offset * i, DATA_LIST_TEMP_CH[i]))
                                self.data[i] = list(np.array(DATA_LIST_TEMP_CH[i]))
                        # print(len(self.data[0]))
                    elif ll > self.plot_length:
                        if self.is_detrend:
                            for i in range(self.channel_num):
                                self.data[i] = list(
                                    np.array(detrend(DATA_LIST_TEMP_CH[i][-self.plot_length:-1])))
                                # self.data[i] = list(np.array(detrend(DATA_LIST_TEMP_CH[i][self.pdata:self.pdata +
                                # self.plot_length])) + self.offset_list[i])
                        elif not self.is_detrend:
                            for i in range(self.channel_num):
                                self.data[i] = list(
                                    np.array(DATA_LIST_TEMP_CH[i][-self.plot_length:-1]))
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

                    # tempData = signal.filtfilt(self.b, self.a , self.data)
                    # update curve

                    for i in range(len(self.signal_curves)):
                        self.signal_curves[i].setData(self.data[i])

                        fft_len=len(self.data[i])
                        self.freqs=fftpack.fftfreq(fft_len) * self.sample_rate
                        self.fft_curves[i].setData(self.freqs, np.abs(fftpack.fft(self.data[i]) * 2 * np.pi / fft_len))

                        self.data[i] = []

                    # reset x origin
                    # for i in range(self.channel_num):
                    #     self.curves[i].setPos(self.pdata, 0)
                self.temp_len_data = ll

    def change_scale_y(self):
        # print(int(self.verticalSlider.value()))
        self.yoffset = 0.01/int(self.father.verticalSlider.value())
        # print(self.offset)

        for i in range(len(self.graphicsViews)):
            # print(i)
            self.graphicsViews[i].setRange(
                yRange=[-self.yoffset, self.yoffset])

            # self.graphicsViews[i].setAxisItems(axisItems={'bottom': self.xstrAxes, 'left': self.ystrAxes})

        self.update_curves()

    def change_scale_x(self):
        self.plot_length = int(self.sample_rate * self.plot_time / self.father.horizontalSlider.value())

        for i in range(len(self.graphicsViews)):
            # print(i)
            self.graphicsViews[i].setRange(
                xRange=[0, self.plot_length])

            # self.graphicsViews[i].setAxisItems(axisItems={'bottom': self.xstrAxes, 'left': self.ystrAxes})

        self.update_curves()

    def closeEvent(self, a0: QtGui.QCloseEvent) -> None:
        self.father.showWaveBox.setChecked(False)




if __name__ == '__main__':
    # SAVE=1

    app = QApplication(sys.argv)
    MainWindow = Main_window()
    # MainWindow.setStyleSheet("#MainWindow{background-color: black}")
    MainWindow.show()
    sys.exit(app.exec_())
