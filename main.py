#!/usr/bin/evn python
#coding:utf-8

import time
from config import config
from vehicle import vehicle

if __name__=='__main__':
	if config.get_cloud()[0] is 1:
		from cloud_module import cloud
	while True:
		time.sleep(1)
