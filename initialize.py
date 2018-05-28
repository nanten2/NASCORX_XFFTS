#!/usr/bin/env python

"""
Initialize XFFTS @ NANTEN2
--------------------------

BE : 1
Bandwidth : 2000 [MHz]
"""

import xffts_udp_client

client = xffts_udp_client.udp_client()
client.open()
client.stop()

client.set_synctime(100000)              # synctime : 100 ms
client.set_usedsections([1]*16)          # using board : 16
for i in range(1, 17):
    client.set_board_bandwidth(i, 2000)  # bandwidth : 2000 MHz
client.configure()                       # apply settings
client.caladc()                          # re-calibration for all ADCs
client.start()                           # start measurement


# History
# -------
# 2017/12/21 : written by T.Inaba
