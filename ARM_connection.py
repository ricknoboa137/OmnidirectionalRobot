import argparse
import logging
import pygame
import time
import numpy as np
import pandas as pd
import pyqtgraph as pg
import threading
import queue
import os
from pyqtgraph.Qt import QtGui, QtCore
from mindrove.board_shim import BoardShim, MindRoveInputParams, BoardIds
from mindrove.data_filter import DataFilter, FilterTypes, DetrendOperations, WindowOperations




class Graph:
    def __init__(self, board_shim, joystick):        
        self.time_start= time.time()
        self.time_sample= time.time()
        self.joystick = joystick
        self.board_id = board_shim.get_board_id()
        self.board_shim = board_shim
        self.exg_channels = BoardShim.get_exg_channels(self.board_id)
        self.sampling_rate = BoardShim.get_sampling_rate(self.board_id)
        self.update_speed_ms = 50
        self.window_size = 2
        self.num_points = self.window_size * self.sampling_rate
        self.nfft = DataFilter.get_nearest_power_of_two (self.sampling_rate * self.window_size)
        self.MAX_DATA_POINTS = DataFilter.get_nearest_power_of_two(self.window_size * self.sampling_rate//2)
        self.app = QtGui.QApplication([])
        self.win = pg.GraphicsWindow(title='Mindrove ARM Plot',size=(800, 600))
        #self.feature_names = ["mean_power", "max_power", "std_dev", "smr_power","alpha_power","beta_power"]
        self.all_names = []
        # Data buffer for the saving thread
        self.save_buffer = []
        self.save_buffer_size = 80 # Save every 100 rows (5 seconds of data)
        self.data_queue = queue.Queue()
        self.stop_saving = threading.Event()
        self.save_thread = threading.Thread(target=self._save_data_periodically)
        self.save_thread.daemon = True # Allows the main program to exit even if the thread is running
        self.save_thread.start()
        
        for ch in range(8):
            for i in range(1000):
                self.all_names.append(f"{i}-ch:{ch+1}")
                #self.df.Columns = {self.all_names}
                
        self.all_names.append("label_x")
        self.all_names.append("label_y")
        self.all_names.append("label_g")
        print(self.all_names)
        self.df=pd.DataFrame(columns=self.all_names)
        self._init_timeseries()
        timer = QtCore.QTimer()
        timer.timeout.connect(self.update)
        timer.start(self.update_speed_ms)
        QtGui.QApplication.instance().exec_()
        
        
        
    def _init_timeseries(self):
        #print(self.time_start)
        self.count=0
        self.plots = list()
        self.curves = list()
        self.signals=[[], [],[], [],[],[],[],[],[]]
        self.last_features=[]
        for i in range(len(self.signals)):
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
            
    def _save_data_periodically(self):
        csv_filepath = 'D:\\DoctoralSchool\\Telepresence Toolkit\\ArmBandConnection\\Datasets\\ARM_2D1T\\Arm_Mateo_00.csv'
        header_written = False # Flag to write header only once
        os.makedirs(os.path.dirname(csv_filepath), exist_ok=True)
        # Main loop for the saving thread
        while not self.stop_saving.is_set() or not self.data_queue.empty():
            try:
                # Get data from the queue with a timeout
                # Using a timeout allows the thread to check the stop_saving flag periodically
                data_row = self.data_queue.get(timeout=0.1) # Get data, wait max 0.1  seconds
                self.save_buffer.append(data_row) 
                # If buffer is full or if stopping and there's data left, save the buffer
                if len(self.save_buffer) >= self.save_buffer_size or (self.stop_saving.is_set() and self.save_buffer):
                    df_chunk = pd.DataFrame(self.save_buffer, columns=self.all_names) # Convert buffer to DataFrame

                    # Write the chunk to CSV
                    df_chunk.to_csv(csv_filepath, mode='a', header=not header_written, index=False)
                    header_written = True # Header is now written

                    self.save_buffer = [] # Clear the buffer
                    logging.debug(f"Saved a chunk of {len(df_chunk)} rows to CSV.")

            except queue.Empty:
                # No data in the queue, continue loop and check stop_saving flag
                pass
            except Exception as e:
                logging.error("Error in saving thread:", exc_info=True)
                # Decide if you want to break the loop on error

        # Final save of any remaining data in the buffer when stopping
        if self.save_buffer:
            df_chunk = pd.DataFrame(self.save_buffer, columns=self.all_names)
            df_chunk.to_csv(csv_filepath, mode='a', header=not header_written, index=False)
            logging.debug("Saved final buffer chunk to CSV.")
    
    #=====================================================================================
    def _stop_saving_thread(self):
        logging.info("Signaling saving thread to stop.")
        self.stop_saving.set()
        self.save_thread.join(timeout=5.0) # Wait for the thread to finish (with a timeout)
        if self.save_thread.is_alive():
            logging.warning("Saving thread did not join within timeout.")
    
    
    #=====================================================================================
    def update(self):
        trigger = 0.2
        features =()
        result=[]
        current_emg_window = []
        data = self.board_shim.get_current_board_data(self.nfft+500)
        for count, channel in enumerate(self.exg_channels):
            # plot timeseries
            DataFilter.detrend(data[channel], DetrendOperations.LINEAR.value)
            DataFilter.perform_bandpass(data[channel], self.sampling_rate, 10.0, 250.0, 2, FilterTypes.BUTTERWORTH.value, 0)
            #DataFilter.perform_bandpass(data[channel], self.sampling_rate, 10.0, 250.0, 2, FilterTypes.BUTTERWORTH.value, 0)
            DataFilter.perform_bandstop(data[channel], self.sampling_rate, 48.0, 52.0, 2,
                                        FilterTypes.BUTTERWORTH.value, 0)
            DataFilter.perform_bandstop(data[channel], self.sampling_rate, 58.0, 62.0, 2, 
                                        FilterTypes.BUTTERWORTH.value, 0)
            #self.curves[count].setData(data[channel].tolist())
            #psd = DataFilter.get_psd_welch(data[channel], self.nfft, self.nfft // 4, self.sampling_rate, WindowOperations.BLACKMAN_HARRIS.value)
            data_mean = np.mean(data[channel])#Mean power
            data_std = np.std(data[channel])#Standard deviation of power
            
            if channel < 6:
                #self.signals[channel].append(band_power_smr)
                #self.curves[channel].setData(self.signals[channel])
                #for i in range(len(data[channel])):
                #data[channel][i]= data[channel][i] #(data[channel][i]-data_mean)/data_std #Normalization
                self.curves[channel].setData(data[channel])
                #features = features + (data[channel][-25:],) # collect the last 25 data points from each channel and concat before labeling
                #print ("FeatureN: ", features, "FeatureSh:") #, features.shape
            #for lst in features:
            result.extend(data[channel][-1000:])
            current_emg_window.extend(data[channel][-1000:]) #Get the last 1000 points
            print(len(current_emg_window))
        self.last_features.append(current_emg_window)
        if len(self.last_features) > 5:# store a list of the last 5 iterations
            self.last_features.pop(0)
        #print(len(self.last_features),len(self.last_features[0]))
        print("--------------------------------")
        pygame.event.get()
        #print(len(features))
        self.x_axis = self.joystick.get_axis(0)# Left thumbstick horizontal (-1 to 1)
        self.y_axis = - self.joystick.get_axis(1)# Left thumbstick vertical (-1 to 1)
        self.g_axis = self.joystick.get_axis(4)
        if abs(self.x_axis) < 0.2: self.x_axis = 0
        if abs(self.y_axis) < 0.2: self.y_axis = 0
        combined_row = current_emg_window + [self.x_axis, self.y_axis, self.g_axis]    
        if len(combined_row) != 8003:
            logging.warning(f"Unexpected row length: {len(combined_row)}. Expected 8003.")
        try:
            self.data_queue.put_nowait(combined_row) # Use put_nowait to avoid blocking update loop
        except queue.Full:
            logging.warning("Data queue is full. Skipping saving this data point.")

        print(self.df.shape)
        self.signals[6].append(self.x_axis)
        self.signals[7].append(self.y_axis)
        self.signals[8].append(self.g_axis)
        for i in range(len(self.signals)):
            if len(self.signals[i]) > 30 : #self.MAX_DATA_POINTS
                self.signals[i].pop(0)

        self.curves[6].setData(self.signals[6])
        self.curves[7].setData(self.signals[7])
        self.curves[8].setData(self.signals[8])
        self.app.processEvents()

def main():
    BoardShim.enable_dev_board_logger()
    logging.basicConfig(level=logging.DEBUG)
    params = MindRoveInputParams()
    try:
        board_shim = BoardShim(BoardIds.MINDROVE_WIFI_BOARD, params)
        board_shim.prepare_session()
        board_shim.start_stream()
        time.sleep(4)
        pygame.init()
        clock = pygame.time.Clock()
        #fps = 500
        #clock.tick(fps)
        if not pygame.joystick.get_count():
            print("Error: No joystick detected.")
            quit()
        joystick = pygame.joystick.Joystick(0)
        joystick_name = joystick.get_name()
        print(f"Connected joystick: {joystick_name}")
        print("-----------------------------------------------------------------")
        graph_instance = Graph(board_shim, joystick)
    except BaseException:
        logging.warning('Exception', exc_info=True)
    finally:
        logging.info('End')
        if board_shim.is_prepared():
            logging.info('Releasing session')
            board_shim.release_session()
        if 'graph_instance' in locals() and hasattr(graph_instance, '_stop_saving_thread'): 
            graph_instance._stop_saving_thread() # Call the stop method
        pygame.quit()

if __name__ == '__main__':

    main()