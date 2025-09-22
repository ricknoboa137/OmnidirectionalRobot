import argparse
import logging

import paho.mqtt.client as paho
from paho import mqtt
from paho.mqtt import client as mqtt_client
#import pygame
import json
import time
import random
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtCore
import numpy as np
import pandas as pd
import tensorflow as tf
from tensorflow.keras.models import Sequential, Model
from tensorflow.keras.layers import LSTM, Dense, Dropout, Input
from tensorflow.keras.layers import Conv1D, Conv2D,Conv1D, MaxPooling1D, MaxPooling2D, Flatten, Dense
from tensorflow.keras.layers import ConvLSTM2D, ConvLSTM1D, Dropout, TimeDistributed
from tensorflow.keras import metrics
from tensorflow.keras import layers
from tensorflow import keras
from tensorflow.keras import layers
from tensorflow.keras import regularizers
from scipy.signal import butter, lfilter, sosfilt

from mindrove.board_shim import BoardShim, MindRoveInputParams, BoardIds
from mindrove.data_filter import DataFilter, FilterTypes, DetrendOperations





class Graph:
    def __init__(self, board_shim, loaded_model):
        self.board_id = board_shim.get_board_id()
        self.board_shim = board_shim
        self.exg_channels = BoardShim.get_exg_channels(self.board_id)
        self.sampling_rate = BoardShim.get_sampling_rate(self.board_id)
        self.update_speed_ms = 50
        self.window_size = 4
        self.num_points = self.window_size * self.sampling_rate
        self.MAX_DATA_POINTS = self.num_points//2
        self.loaded_model=loaded_model
        self.app = QtGui.QApplication([])
        self.win = pg.GraphicsWindow(title='Mindrove Plot',size=(800, 600))
        
        self.broker = '192.168.0.174'
        self.port = 1883
        self.topic = "gamepad/joystick"
        self.client_id = f'python-mqtt-{random.randint(0, 1000)}'
        # username = 'emqx'
        # password = 'public'
        self.client = self.connect_mqtt()
        self.client.loop_start()
        
        

        self._init_timeseries()

        timer = QtCore.QTimer()
        timer.timeout.connect(self.update)
        timer.start(self.update_speed_ms)
        QtGui.QApplication.instance().exec_()
        
        

    def _init_timeseries(self):
        self.plots = list()
        self.curves = list()
        self.signals = [[], []]
        for i in range(len(self.exg_channels)):
            p = self.win.addPlot(row=i,col=0)
            p.showAxis('left', False)
            p.setMenuEnabled('left', False)
            p.showAxis('bottom', False)
            p.setMenuEnabled('bottom', False)
            if i == 0:
                p.setTitle('TimeSeries Plot')
            self.plots.append(p)
            curve = p.plot()
            self.curves.append(curve)

    def update(self):
        message = ""
        result=[]
        data = self.board_shim.get_current_board_data(1000)
        sampling_rate = 500  # Replace with your actual sampling rate in Hz
        cutoff_frequency = 10   # cutoff frequency in Hz (10)
        low_cutoff_frequency = 10  # Lower cutoff frequency in Hz (20)
        high_cutoff_frequency = 240 # Upper cutoff frequency in Hz (240)
        filter_order = 4
        filter_type = 'bandpass'  # Choose 'highpass', 'lowpass', 'bandpass', or 'bandstop'
        nyquist_freq = 0.5 * sampling_rate
        snip_points = 150
        final_data = [[],[],[],[],[],[],[],[]]
        for count, channel in enumerate(self.exg_channels):
            # plot timeseries
            DataFilter.detrend(data[channel], DetrendOperations.LINEAR.value)
            DataFilter.perform_bandpass(data[channel], self.sampling_rate, 10.0, 250.0, 2, FilterTypes.BUTTERWORTH.value, 0)
            DataFilter.perform_bandstop(data[channel], self.sampling_rate, 48.0, 52.0, 2,
                                        FilterTypes.BUTTERWORTH.value, 0)
            DataFilter.perform_bandstop(data[channel], self.sampling_rate, 58.0,62.0, 2,
                                        FilterTypes.BUTTERWORTH.value, 0)
            self.curves[count].setData(data[channel].tolist())
            # Normalize both cutoff frequencies
            normalized_low_cutoff = low_cutoff_frequency / nyquist_freq
            normalized_high_cutoff = high_cutoff_frequency / nyquist_freq
            normalized_cutoff = [normalized_low_cutoff, normalized_high_cutoff]
            sos = butter(filter_order, normalized_cutoff, btype=filter_type, analog=False, output= 'sos')
            filtered_data = np.zeros_like(data[channel], dtype=float)
              # Iterate through channels
            filtered_data[:] = sosfilt(sos, data[channel][:])
                
            cropped_filtered_data = filtered_data[snip_points:]
            dc_filtered_data = []            
            dc_filtered_data=cropped_filtered_data - np.mean(cropped_filtered_data)
            dc_filtered_data= np.array(dc_filtered_data)
            #print(dc_filtered_data.shape)
            local_max = np.max(dc_filtered_data)
            local_min = np.min(dc_filtered_data)
            normalized_data = np.zeros_like(cropped_filtered_data, dtype=float)
            if local_max == local_min: #prevent division by zero.
                normalized_data[:] = 0.0 #set all values to zero if max and min are equal.
                print("min =max")
            else:
                normalized_data[:] = 2 * (dc_filtered_data - local_min) / (local_max - local_min) - 1
            final_data[channel]= normalized_data
        final_np = np.array(final_data).reshape(1,8,850)
        print(f"Final shape: {final_np.shape}")
        for i in range(len(self.signals)):
            if len(self.signals[i]) > self.MAX_DATA_POINTS:
                self.signals[i].pop(0)
        predictions = self.loaded_model.predict(final_np)
        horizontal_predictions = -predictions[0][0][0]
        vertical_predictions = predictions[1][0][0]
        fist_predictions = predictions[2][0][0]

        print(f"\nVertical Rotation Predictions (tanh probabilities):\n{vertical_predictions}")
        print(f"Horizontal Rotation Predictions (tanh probabilities):\n{horizontal_predictions}")
        print(f"Fist Activity Predictions (sigmoid probabilities):\n{fist_predictions}")
        #message = " ".join(str(v) for v in data[3])
        a= np.array([vertical_predictions, horizontal_predictions, fist_predictions])
        a = a.tolist()
        message = json.dumps({"axes":a})
        #print(message)
        #result.extend(data[channel][-1000:])
        self.app.processEvents()
        self.publish(self.client, message)
        
    
    def connect_mqtt(self):
        def on_connect(self, client, userdata, flags, rc):
        # For paho-mqtt 2.0.0, you need to add the properties parameter.
        # def on_connect(client, userdata, flags, rc, properties):
            if rc == 0:
                print("Connected to MQTT Broker!")
            else:
                print("Failed to connect, return code %d\n", rc)
        # Set Connecting Client ID
        #self.client = mqtt_client.Client(self.client_id)

        # For paho-mqtt 2.0.0, you need to set callback_api_version.
        client = mqtt_client.Client(client_id=self.client_id, callback_api_version=mqtt_client.CallbackAPIVersion.VERSION2)

        # client.username_pw_set(username, password)
        client.on_connect = on_connect
        client.connect(self.broker, self.port)
        return client
        
    def publish(self, client, message):
        msg_count = 1        
        #time.sleep(1)
        #msg = f"messages: {msg_count}"
        result = client.publish(self.topic, message)
        # result: [0, 1]
        status = result[0]
        if status == 0:
            print(f"Send `{len(message)}` to topic `{self.topic}`")
        else:
            print(f"Failed to send message to topic {self.topic}")
        msg_count += 1
        if msg_count > 5:
            msg_count = 0


def main():
    sampling_rate = 500  # Replace with your actual sampling rate in Hz
    cutoff_frequency = 10   # cutoff frequency in Hz (10)
    low_cutoff_frequency = 10  # Lower cutoff frequency in Hz (20)
    high_cutoff_frequency = 240 # Upper cutoff frequency in Hz (240)
    filter_order = 4
    filter_type = 'bandpass'  # Choose 'highpass', 'lowpass', 'bandpass', or 'bandstop'
    nyquist_freq = 0.5 * sampling_rate
    snip_points = 150
    

    BoardShim.enable_dev_board_logger()
    logging.basicConfig(level=logging.DEBUG)


    params = MindRoveInputParams()
    
    
    model_path = 'D:\DoctoralSchool\Telepresence Toolkit\ArmBandConnection\Models\LSTM_model_04.h5' 
    loaded_model = tf.keras.models.load_model(model_path) #, custom_objects=custom_objects
    print(f"Model loaded successfully from {model_path}")
    loaded_model.summary()

    try:
        
        board_shim = BoardShim(BoardIds.MINDROVE_WIFI_BOARD, params)
        board_shim.prepare_session()
        board_shim.start_stream()
        time.sleep(4)
        #pygame.init()
        #clock = pygame.time.Clock()
        #fps = 500
        #clock.tick(fps)
        #if not pygame.joystick.get_count():
       #     print("Error: No joystick detected.")
        #    quit()
        #joystick = pygame.joystick.Joystick(0)
        #joystick_name = joystick.get_name()
        #print(f"Connected joystick: {joystick_name}")
        #print("-----------------------------------------------------------------")
        
        Graph(board_shim, loaded_model)
        
    except BaseException:
        logging.warning('Exception', exc_info=True)
    finally:
        logging.info('End')
        if board_shim.is_prepared():
            logging.info('Releasing session')
            board_shim.release_session()
        client.loop_stop()
        #pygame.quit()


if __name__ == '__main__':
    main()