#!/usr/bin/evn python
# coding:utf-8

import sys
sys.path.append('..')
import serial
import time
import os
import signal
from lib.config import config
from lib.logger import logger


def open_serial(portname, baudrate, timeout=None):
    while True:
        try:
            # print("port:{0},baudrate:{1}".format(portname, baudrate))
            com = serial.Serial(portname, baudrate, timeout=timeout)
            return com
        except serial.SerialException:
            info = sys.exc_info()
            logger.error("{0}:{1}".format(*info))
            time.sleep(1.0)


class Singleton(type):

    def __init__(cls, name, bases, dict):
        super(Singleton, cls).__init__(name, bases, dict)
        cls._instance = None

    def __call__(cls, *args, **kw):
        if cls._instance is None:
            cls._instance = super(Singleton, cls).__call__(*args, **kw)
        return cls._instance


class CancelWatcher(object):
    Cancel = False
    count = 0

    def __init__(self):
        if self.__class__.count == 0 and self.__class__.Cancel:
            self.__class__.Cancel = False
        self.__class__.count += 1

    def IsCancel(self):
        return self.__class__.Cancel

    def __del__(self):
        self.__class__.count -= 1
        if self.__class__.count == 0:
            self.__class__.Cancel = False


class Watcher(object):
    """this class solves two problems with multithreaded
    programs in Python, (1) a signal might be delivered
    to any thread (which is just a malfeature) and (2) if
    the thread that gets the signal is waiting, the signal
    is ignored (which is a bug).

    The watcher is a concurrent process (not thread) that
    waits for a signal and the process that contains the
    threads.  See Appendix A of The Little Book of Semaphores.
    http://greenteapress.com/semaphores/

    I have only tested this on Linux.  I would expect it to
    work on the Macintosh and not work on Windows.
    """

    def __init__(self):
        """ Creates a child thread, which returns.  The parent
            thread waits for a KeyboardInterrupt and then kills
            the child thread.
        """
        self.child = os.fork()
        if self.child == 0:
            return
        else:
            self.watch()

    def watch(self):
        try:
            os.wait()
        except KeyboardInterrupt:
            # I put the capital B in KeyBoardInterrupt so I can
            # tell when the Watcher gets the SIGINT
            self.kill()
        sys.exit()

    def kill(self):
        try:
            os.kill(self.child, signal.SIGKILL)
        except OSError:
            pass
