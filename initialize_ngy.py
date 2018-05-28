#!/usr/bin/env python

"""
Initialize XFFTS @ nagoya environment
-------------------------------------

BE : 1
Bandwidth : 2000 [MHz]
"""

import xffts_udp_client

client = xffts_udp_client.udp_client()
client.open()
client.stop()

client.set_synctime(100000)          # synctime : 100 ms
client.set_usedsections([1])         # using board : 1
client.set_board_bandwidth(1, 2000)  # bandwidth : 2000 MHz
client.configure()                   # apply settings
client.board_caladc(1)               # re-calibration
client.start()                       # start measurement


# History
# -------
# 2017/12/21 : written by T.Inaba
