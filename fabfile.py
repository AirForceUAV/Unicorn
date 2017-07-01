from __future__ import with_statement
import time,os
from fabric.api import *
from fabric.contrib.console import confirm


def start(architecture="arm",isRunMC=True,isRunFC=True):
    # if isRunMC:
    #     startMC(architecture)
    # time.sleep(3)
    if isRunFC:
        startFC()
    

def startMC(architecture):
    arc2cmd = { "arm":"DataProxy_linux_arm",
                "amd":"DataProxy_linux_amd64",
                "damd":"DataProxy_darwin_amd64"}
    DataProxyBinPath="~/DataProxyBin"
    DataProxyCommand = "nohup ./"+arc2cmd.get(architecture)+" debug 2>&1 &"

    # logger("cd "+DataProxyBinPath)
    with lcd(DataProxyBinPath):
        # logger("excute:" + DataProxyCommand)
        # local("tmux")
        local(DataProxyCommand)

def startFC():
    MainControllerPath="~/Unicorn/examples"   
    MainControllerCommand="nohup python test.py"+" debug 2>&1 &" 

    # logger("cd "+MainControllerPath)
    with lcd(MainControllerPath):
        # logger("excute:"+MainControllerCommand)
        local(MainControllerCommand)


def logger(message):
    print("[DEBUG]"+message)


def test():
    import os
    