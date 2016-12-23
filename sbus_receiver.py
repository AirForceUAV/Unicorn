#!/usr/bin/evn python
# coding:utf-8

import serial
import array
import time
import sys
from library import CancelWatcher
from library import CutFrame


class SBUSReceiver:

    def __init__(self):

        self.sbus = self.open_serial('/dev/ttyUSB0')

        # constants
        self.START_BYTE = b'0f'
        self.END_BYTE = b'00'
        self.SBUS_FRAME_LEN = 25
        self.SBUS_NUM_CHAN = 18
        self.OUT_OF_SYNC_THD = 10
        self.SBUS_NUM_CHANNELS = 18
        self.SBUS_SIGNAL_OK = 0
        self.SBUS_SIGNAL_LOST = 1
        self.SBUS_SIGNAL_FAILSAFE = 2

        # Stack Variables initialization
        self.validSbusFrame = 0
        self.lostSbusFrame = 0
        self.frameIndex = 0
        self.resyncEvent = 0
        self.outOfSyncCounter = 0
        self.sbusBuff = bytearray(1)  # single byte used for sync
        self.rawFrame = ''
        self.sbusFrame = bytearray(25)  # single SBUS Frame

        self.sbusChannels = array.array(
            'H', [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])  # RC Channels
        self.isSync = False
        self.startByteFound = False
        self.failSafeStatus = self.SBUS_SIGNAL_FAILSAFE

    def open_serial(self, portname):
        while True:
            try:
                com = serial.Serial(port=portname, baudrate=100000,
                                    parity=serial.PARITY_EVEN, stopbits=serial.STOPBITS_TWO, bytesize=serial.EIGHTBITS, timeout=1)
                return com
            except serial.SerialException:
                info = sys.exc_info()
                print "{0}:{1}".format(*info)
                time.sleep(0.5)

    def RawFrame(self):
        self.sbus.flushInput()
        msg = self.sbus.read(self.SBUS_FRAME_LEN * 2).encode('hex')
        begin = msg.find(self.START_BYTE)
        end = begin + self.SBUS_FRAME_LEN * 2
        if end >= self.SBUS_FRAME_LEN * 4:
            print 'Error:Frame is unvalid', msg
            return self.RawFrame()
        argv = msg[begin:end]
        if not msg[end - 2:end] in ['04', '14', '24', '34']:
            print 'Error:Frame is unvalid', argv
            return self.RawFrame()
        return argv

    def sbusFrame(self):
        package = self.RawFrame()
        self.sbusFrame = CutFrame(package, 4)

    def sendFrame(self, frame):
        self.sbus.write(frame)

    def radio(self):
        self.sbus.flushInput()
        watcher = CancelWatcher()
        while not watcher.IsCancel():
            frame = self.sbus.read(self.SBUS_FRAME_LEN + 10)
            # print frame.encode('hex')
            self.sendFrame(frame)

    def GCS(self):
        CancelWatcher.Cancel = True

    def get_rx_channels(self):
        """
        Used to retrieve the last SBUS channels values reading
        :return:  an array of 18 unsigned short elements containing 16 standard channel values + 2 digitals (ch 17 and 18)
        """
        return self.sbusChannels

    def get_rx_channel(self, num_ch):
        """
        Used to retrieve the last SBUS channel value reading for a specific channel
        :param: num_ch: the channel which to retrieve the value for
        :return:  a short value containing
        """
        return self.sbusChannels[num_ch]

    def get_failsafe_status(self):
        """
        Used to retrieve the last FAILSAFE status
        :return:  a short value containing
        """
        return self.failSafeStatus

    def get_rx_report(self):
        """
        Used to retrieve some stats about the frames decoding
        :return:  a dictionary containg three information ('Valid Frames','Lost Frames', 'Resync Events')
        """

        rep = {}
        rep['Valid Frames'] = self.validSbusFrame
        rep['Lost Frames'] = self.lostSbusFrame
        rep['Resync Events'] = self.resyncEvent

        return rep

    def decode_frame(self):
        # TODO: DoubleCheck if it has to be removed
        for i in range(0, self.SBUS_NUM_CHANNELS - 2):
            self.sbusChannels[i] = 0

        # counters initialization
        byte_in_sbus = 1
        bit_in_sbus = 0
        ch = 0
        bit_in_channel = 0

        for i in range(0, 175):  # TODO Generalization
            if self.sbusFrame[byte_in_sbus] & (1 << bit_in_sbus):
                self.sbusChannels[ch] |= (1 << bit_in_channel)

            bit_in_sbus += 1
            bit_in_channel += 1

            if bit_in_sbus == 8:
                bit_in_sbus = 0
                byte_in_sbus += 1

            if bit_in_channel == 11:
                bit_in_channel = 0
                ch += 1

        # Decode Digitals Channels

        # Digital Channel 1
        if self.sbusFrame[self.SBUS_FRAME_LEN - 2] & (1 << 0):
            self.sbusChannels[self.SBUS_NUM_CHAN - 2] = 1
        else:
            self.sbusChannels[self.SBUS_NUM_CHAN - 2] = 0

        # Digital Channel 2
        if self.sbusFrame[self.SBUS_FRAME_LEN - 2] & (1 << 1):
            self.sbusChannels[self.SBUS_NUM_CHAN - 1] = 1
        else:
            self.sbusChannels[self.SBUS_NUM_CHAN - 1] = 0

        # Failsafe
        self.failSafeStatus = self.SBUS_SIGNAL_OK
        if self.sbusFrame[self.SBUS_FRAME_LEN - 2] & (1 << 2):
            self.failSafeStatus = self.SBUS_SIGNAL_LOST
        if self.sbusFrame[self.SBUS_FRAME_LEN - 2] & (1 << 3):
            self.failSafeStatus = self.SBUS_SIGNAL_FAILSAFE

if __name__ == '__main__':

    sbus = SBUSReceiver()
    while True:
        self.get_sbusFrame()
        self.decode_frame()
        print 'channels', sbus.get_rx_channels()
        raw_input('next')
