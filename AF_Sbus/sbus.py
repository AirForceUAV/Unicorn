#!/usr/bin/evn python
# coding:utf-8

import sys
sys.path.append('..')
import array
import time
from lib.tools import CancelWatcher
from lib.science import CutFrame


class SBUS(object):

    def __init__(self):
        # constants
        self.START_BYTE = b'0f'
        self.FLAGS_BYTE = b'00'
        self.END_BYTE = [b'04', b'14', b'24', b'34']
        self.SBUS_FRAME_LEN = 25
        self.SBUS_NUM_CHAN = 18
        self.OUT_OF_SYNC_THD = 10
        self.SBUS_NUM_CHANNELS = 18
        self.SBUS_SIGNAL_OK = 0
        self.SBUS_SIGNAL_LOST = 1
        self.SBUS_SIGNAL_FAILSAFE = 2
        self.number = 0

        # Stack Variables initialization
        self.validSbusFrame = 0
        self.lostSbusFrame = 0
        self.frameIndex = 0
        self.resyncEvent = 0
        self.outOfSyncCounter = 0
        self.sbusBuff = bytearray(1)  # single byte used for sync
        self.sbusFrame = bytearray(25)  # single SBUS Frame
        # RC Channels
        # self.sbusChannels = array.array(
        #     'H', [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        self.sbusChannels = [0] * 18
        self.isSync = False
        self.startByteFound = False
        self.failSafeStatus = self.SBUS_SIGNAL_FAILSAFE

    def get_rx_channels(self):
        """
        Used to retrieve the last SBUS channels values reading
        :return:  an array of 18 unsigned short elements containing 16
        standard channel values + 2 digitals (ch 17 and 18)
        """
        return self.sbusChannels

    def get_rx_channel(self, num_ch):
        """
        Used to retrieve the last SBUS channel value reading for
        a specific channel
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
        :return:  a dictionary containg three information
        ('Valid Frames','Lost Frames', 'Resync Events')
        """

        rep = {}
        rep['Valid Frames'] = self.validSbusFrame
        rep['Lost Frames'] = self.lostSbusFrame
        rep['Resync Events'] = self.resyncEvent

        return rep

    def decode_frame(self):
        # TODO: DoubleCheck if it has to be removed
        for i in xrange(0, self.SBUS_NUM_CHANNELS - 2):
            self.sbusChannels[i] = 0

        # counters initialization
        byte_in_sbus = 1
        bit_in_sbus = 0
        ch = 0
        bit_in_channel = 0

        for i in xrange(0, 175):  # TODO Generalization
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

    def filter(self, package):
        size = self.SBUS_FRAME_LEN * 2
        begin = package.find(self.START_BYTE)
        if begin == -1:
            return None
        end = begin + size
        argv = package[begin:end]
        if argv[-2:] in self.END_BYTE and len(argv) == size:
            return argv
        else:
            return None

    def decode(self, package):
        if package is None:
            return None
        self.sbusFrame = CutFrame(package)
        self.decode_frame()
        result = self.get_rx_channels()
        return result[:8]

    def encode(self, sbusChannels):
        if sbusChannels is None:
            return None
        ch9_16 = '0004200001080738001080'
        # dec2bin len(ch8_bin)=8
        ch8_bin = map(lambda x: format(x, '0>11b'), sbusChannels[:8])

        # init some parameter
        flag = False
        i = 8
        j = 3
        # len(cut_ch8)=2*8+2
        cut_ch8 = []

        for ch in ch8_bin:
            cut_ch8.append(ch[-i:])
            if flag:
                cut_ch8.append(ch[j: j + 8])
                flag = False
            cut_ch8.append(ch[: j])

            i = (16 - j) % 8
            j = 11 - i
            if j > 8:
                j -= 8
                flag = True

        # Reverse ch8
        for i in [1, 3, 6, 8, 10, 13, 15]:
            tmp = cut_ch8[i + 1]
            cut_ch8[i + 1] = cut_ch8[i]
            cut_ch8[i] = tmp

        # Merge cut_ch8(binary)
        package = reduce(lambda x, y: x + y, cut_ch8)
        # bin2hex
        ch1_8 = reduce(lambda x, y: x + y, [format(int(package[x:x + 8], 2), '0>2x')
                                            for x in xrange(len(package)) if x % 8 == 0])
        # Pack Frame
        return self.START_BYTE + ch1_8 + ch9_16 + self.FLAGS_BYTE


def build_sbus():
    import serial
    from lib.config import config
    while True:
        try:
            com = serial.Serial(**config.sbus_serial)
            return com
        except serial.SerialException:
            info = sys.exc_info()
            print("{0}:{1}".format(*info))
            time.sleep(.5)

if __name__ == '__main__':
    # package = '0f00a420a809086a504b182c00042000010807380010800014'
    package = '0f600135A8410D165003102C00042000010807380010800014'
    sbus = SBUS()
    package = sbus.filter(package)
    if package is None:
        print 'package is error'
    else:
        sbusChannels = sbus.decode(package)
        print sbusChannels
    print package
    print sbus.encode(sbusChannels) + '14'
    # print sbus.encode([352, 1696, 1696, 1696, 1024, 352, 1024, 1024]) + '14'
