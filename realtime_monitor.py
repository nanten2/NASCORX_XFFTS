#!/usr/bin/env python
import rospy
import matplotlib.pyplot as plt
import time
import tkinter
import threading
import numpy
from datetime import datetime as dt
from nascorx_xffts.msg import XFFTS_msg
from nascorx_xffts.msg import XFFTS_pm_msg
from nascorx_xffts.msg import XFFTS_temp_msg

class XFFTS_monitor():
    xffts_data = 0
    if_num = 1
    flag = True
    count_ave = 0
    
    def __init__(self):
        rospy.init_node("monitor")
        sub = rospy.Subscriber('XFFTS_SPEC', XFFTS_msg, self.update_data)
        th = threading.Thread(target=self.button)
        th.setDaemon(True)
        th.start()
        th2 = threading.Thread(target=self.stdout_status)
        th2.setDaemon(True)
        th2.start()
        self.realtime_plot()
        pass

    def update_data(self, req):
        self.xffts_data = req


    def stdout_status(self):
        while not rospy.is_shutdown():
            print("\rupdate : {}\naverage count : {:.5e}".format(self.flag,self.count_ave))
            print("IF : {}".format(self.if_num))
            time.sleep(0.5)

    def button(self):
        root = tkinter.Tk()
        root.title("xfftsmonitor")
        root.geometry("150x500")
        buttons = []
        buttons.append(tkinter.Button(root, text="IF1", command=lambda:self.change_if(1)))
        buttons.append(tkinter.Button(root, text="IF2", command=lambda:self.change_if(2)))
        buttons.append(tkinter.Button(root, text="IF3", command=lambda:self.change_if(3)))
        buttons.append(tkinter.Button(root, text="IF4", command=lambda:self.change_if(4)))
        buttons.append(tkinter.Button(root, text="IF5", command=lambda:self.change_if(5)))
        buttons.append(tkinter.Button(root, text="IF6", command=lambda:self.change_if(6)))
        buttons.append(tkinter.Button(root, text="IF7", command=lambda:self.change_if(7)))
        buttons.append(tkinter.Button(root, text="IF8", command=lambda:self.change_if(8)))
        buttons.append(tkinter.Button(root, text="IF9", command=lambda:self.change_if(9)))
        buttons.append(tkinter.Button(root, text="IF10", command=lambda:self.change_if(10)))
        buttons.append(tkinter.Button(root, text="IF11", command=lambda:self.change_if(11)))
        buttons.append(tkinter.Button(root, text="IF12", command=lambda:self.change_if(12)))
        buttons.append(tkinter.Button(root, text="IF13", command=lambda:self.change_if(13)))
        buttons.append(tkinter.Button(root, text="IF14", command=lambda:self.change_if(14)))
        buttons.append(tkinter.Button(root, text="IF15", command=lambda:self.change_if(15)))
        buttons.append(tkinter.Button(root, text="IF16", command=lambda:self.change_if(16)))
        buttons.append(tkinter.Button(root, text="stop/start updating", command=lambda:self.update_flag()))
        for i in range(17):
            buttons[i].pack(fill="x")
        root.mainloop()        

    def change_if(self, i):
        self.if_num = i

    def update_flag(self):
        if self.flag:
            self.flag = False
        else:
            self.flag = True
        
    def realtime_plot(self):
        plt.style.use("dark_background")
        fig = plt.figure(figsize=(9,4))
        ax = fig.add_subplot(111)
        while not rospy.is_shutdown():
            if self.xffts_data == 0:
                time.sleep(1)
                continue
            if not self.flag:
                time.sleep(1)
                continue
            tstamp = dt.utcfromtimestamp(float(self.xffts_data.timestamp))
            ax.plot(eval("self.xffts_data.SPEC_BE{}".format(self.if_num)))
            ax.set_title(tstamp.strftime("%Y/%m/%d %H:%M:%S IF:{}".format(self.if_num)))
            ax.set_xlabel("ch")
            ax.set_ylabel("count")
            ax.set_yscale("log")
            self.count_ave = numpy.mean(eval("self.xffts_data.SPEC_BE{}".format(self.if_num)))
            ax.text(0.1, 0.95, "Count Average : {:.3e}".format(self.count_ave), transform=ax.transAxes)
            ax.grid(True)
            plt.pause(0.5)
            plt.cla()
        pass
        
if __name__ == "__main__":
    x = XFFTS_monitor()
    rospy.spin()
