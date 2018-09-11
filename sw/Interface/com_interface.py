#!/usr/bin/python

__author__ = 'gdiaz'

# COM INTERFACE

from bt_receiver import btReceiver
from file_manager import fileManager

class ComInterface:
    def __init__(self, test_name):
        # Only argument stuff
        self.running = False
        self.bt_receiver = btReceiver(debug = False)
        self.file_manager = fileManager(test_name, debug = False)

    def initialize(self):
        #Buetooth
        self.bt_receiver.initialize()

    def stop(self):
        self.running = False
        self.bt_receiver.stop()
        self.file_manager.stop()

    def update_state(self):
        while True:
            # Receive data
            self.bt_receiver.read()
            packet = self.bt_receiver.packet
            self.bt_receiver.reset()
            #write data to file
            self.file_manager.save_data(packet)

if __name__ == '__main__':
    test_name = raw_input("test_name: ")
    comm = ComInterface(test_name)
    comm.initialize()
    comm.update_state()
    comm.stop()