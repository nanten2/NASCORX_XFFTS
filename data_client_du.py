#!/usr/bin/env python

import time
import rospy
import numpy
import calendar
import datetime
import threading
import astropy.io.fits as fits

from NASCORX_XFFTS.msg import XFFTS_msg
from NASCORX_XFFTS.msg import XFFTS_pm_msg
from NASCORX_XFFTS.msg import XFFTS_temp_msg
from NASCORX_XFFTS.msg import XFFTS_para_msg

dir = '/home/amigos/ros/src/NASCORX_XFFTS/data/'

class data_client(object):
    synctime = 0.1
    spec_data = None
    conti_data = None

    def __init__(self, synctime=0.1):
        self.synctime = synctime
        pass

    def index_search(self, start, mode):
        """
        DESCRIPTION
        ===========
        This function returns the index of the first data to use.
        Used to take into account of start time(waiting time).

        ARGUMENTS
        =========
        1. start : Same as that of oneshot.

        RETURNS
        =======
        1. index  : The index of the first data to use.
             Type : int
        """
        if start is None: index = 0
        else:
            if mode == 'spec':
                unixlist = self.unixlist
                start_arg = round(start,1)
            elif mode == 'conti':
                unixlist = self.conti_unixlist
                start_arg = round(start,1)
            elif mode == 'temp':
                unixlist = self.btemp_unixlist
                start_arg = round(start,1)
            index = unixlist.index(start_arg)

        return index

    """
            try:
                index = unixlist.index(start_arg)
            except ValueError:
                try:
                    start_arg = round(start_arg + 0.1, 1)
                    index = unixlist.index(start_arg)
                except ValueError:
                    try:
                        start_arg = round(start_arg + 0.2, 1)
                        index = unixlist.index(start_arg)
                    except ValueError:
                        start_arg = round(start_arg + 0.3, 1)
                        index = unixlist.index(start_arg)
    """

    def timestamp_to_unixtime(self, timestamp):
        """
        DESCRIPTION
        ===========
        This function converts "XFFTS-timestamp(UTC, string)" to "UNIX-time(UTC, float)".

        ARGUMENTS
        =========
        1. timestamp : The timestamp of XFFTS(UTC, str).
             Type    : Str
             format  : '2017-10-20T09:34:13.9193PC  '

        RETURNS
        =======
        1. unixtime : The unix-timestamp(UTC, float) of input XFFTS-timestamp(UTC, str).
             Type   : float
        """
        t = datetime.datetime.strptime(timestamp, '%Y-%m-%dT%H:%M:%S.%fPC  ')
        unixtime = calendar.timegm(t.timetuple()) + t.microsecond / 1e6
        return unixtime

    # Spectrum Func
    # -------------

    def oneshot(self, integtime, repeat, start):
        """
        DESCRIPTION
        ===========
        This is the master function of this class.
        This function returns integrated XFFTS spectrum.
        You can choose integration time, Number of repetition and start time.

        ARGUMENTS
        =========
        1. integtime [sec] : integration time of each observation.
             Type          : float
             Default       : Nothing
        2. repeat    : How many times to repeat observation.
             Type    : int
             Default : Nothing
        3. start [Unix time] : When to start observation in Unix-time.
             Type            : float (time.time like value)
             Default         : None
             Example         : time.time()+5
                             : 1508495163.9506361

        RETURNS
        =======
        1. timelist : The timestamp of the first data of each integrations.
             Type   : Str list
             Length : Same as "repeat"
        2. unixlist : The Unix-time version of timelist.
             Type   : float list
             Length : Same as "repeat"
        3. spectrum : The integrated spectrum.
             Type   : float array
             dim    : 3
             shape  : [repeat, BE_num, ch_num]

        USAGE
        =====
        ret = data_client.oneshot(integtime=0.5, repeat=5)
        ts, uts, spec = data_client.oneshot(0.5, 5, start=time.time()+5)
        """

        # define data list
        # ----------------
        self.result = []
        self.timestamp = []
        self.unixlist = []
        self.data = []

        if start is None or start == 0: waittime = 0
        else: waittime = start - time.time()

        # subscribe data
        # --------------
        self.data_subscriber(integtime=integtime, repeat=repeat, waittime=waittime)

        # data integration
        # ----------------
        spectrum = []
        timelist = []
        unixlist = []
        init_index = self.index_search(start=start, mode='spec')
        for i in range(repeat):
            start = init_index + int(integtime / self.synctime * i)
            fin = init_index + int(integtime / self.synctime * (i+1))
            spectrum.append(numpy.average(self.data[start:fin], axis=0))
            timelist.append(self.timestamp[start:fin])
            unixlist.append(self.unixlist[start:fin])
        self.spec_data = [timelist, unixlist, spectrum]

        return

    def data_subscriber(self, integtime, repeat, waittime):
        """
        DESCRIPTION
        ===========
        This function receives values from data-server(GIGABYTE PC) and append it to list.

        ARGUMENTS
        =========
        1. integtime [sec] : Same as that of oneshot.
        2. repeat          : Same as that of oneshot.
        3. waiting [sec]   : How long to wait until the first observation starts.
             Type            : float
             Default         : Nothing.
        """
        sub = rospy.Subscriber('XFFTS_SPEC', XFFTS_msg, self.append)
        time.sleep(waittime + integtime * repeat + 0.5)
        sub.unregister()

        return

    def append(self, req):
        """
        DESCRIPTION
        ===========
        This function appends received-data to self list.

        ARGUMENTS
        =========
        1. req : ROS format argument.

        RETURNS
        =======
        Nothing but append to self-list.
        1. timestamp : The timestamp of XFFTS(UTC, str).
             where   : self.timestamp
        2. unix_ret  : The unix-timestamp(UTC, float) of format "xxxxxxxxxx.x" .
             where   : self.unixlist
        3. data_temp : The spectrum of each sync-time. Only for available Back-End.
             where   : self.data
        """
        data_temp = []

        # Calculate UNIX-time
        # -------------------
        unixtime = self.timestamp_to_unixtime(req.timestamp)
        unix_ret = round(unixtime, 1)                                               # using xx.x [sec] format

        # append data to temporary list
        # -----------------------------
        reqlist = [req.SPEC_BE1, req.SPEC_BE2, req.SPEC_BE3, req.SPEC_BE4,
                   req.SPEC_BE5, req.SPEC_BE6, req.SPEC_BE7, req.SPEC_BE8,
                   req.SPEC_BE9, req.SPEC_BE10, req.SPEC_BE11, req.SPEC_BE12,
                   req.SPEC_BE13, req.SPEC_BE14, req.SPEC_BE15, req.SPEC_BE16,
                   req.SPEC_BE17, req.SPEC_BE18, req.SPEC_BE19, req.SPEC_BE20]
        for i in range(req.BE_num):
            data_temp.append(reqlist[i])

        # append return value
        # -------------------
        self.timestamp.append(req.timestamp)
        self.unixlist.append(unix_ret)
        self.data.append(data_temp)
        return

    # Continuum Func
    # --------------

    def conti_oneshot(self, integtime, repeat, start):
        """
        DESCRIPTION
        ===========
        This is the master function of this class.
        This function returns integrated XFFTS power (continuum).
        You can choose integration time, Number of repetition and start time.

        ARGUMENTS
        =========
        1. integtime [sec] : integration time of each observation.
        2. repeat    : How many times to repeat observation.
        3. start [Unix time] : When to start observation in Unix-time.

        RETURNS
        =======
        1. timelist : The timestamp of the first data of each integrations.
        2. unixlist : The Unix-time version of timelist.
        3. power : The integrated power.
             Type   : float array
             dim    : 3
             shape  : [repeat, BE_num]

        USAGE
        =====
        -- COMING SOON --
        """
        # define data list
        # ----------------
        self.conti_result = []
        self.conti_timestamp = []
        self.conti_unixlist = []
        self.conti_data = []

        if start is None or start == 0: waittime = 0
        else: waittime = start - time.time()

        # subscribe data
        # --------------
        self.conti_data_subscriber(integtime=integtime, repeat=repeat, waittime=waittime)

        # data integration
        # ----------------
        spectrum = []
        timelist = []
        unixlist = []
        init_index = self.index_search(start=start, mode='conti')
        for i in range(repeat):
            start = init_index + int(integtime / self.synctime * i)
            fin = init_index + int(integtime / self.synctime * (i+1))
            spectrum.append((numpy.average(self.conti_data[start:fin], axis=0)))
            timelist.append(self.conti_timestamp[start:fin])
            unixlist.append(self.conti_unixlist[start:fin])
        self.conti_data = [timelist, unixlist, spectrum]

        return

    def conti_data_subscriber(self, integtime, repeat, waittime):
        sub2 = rospy.Subscriber('XFFTS_PM', XFFTS_pm_msg, self.conti_append)
        time.sleep(waittime + integtime * repeat + 0.5)
        sub2.unregister()
        return

    def conti_append(self, req):
        """
        DESCRIPTION
        ===========
        This function appends received-data to self list.

        ARGUMENTS
        =========
        1. req : ROS format argument.

        RETURNS
        =======
        Nothing but append to self-list.
        1. timestamp : The timestamp of XFFTS(UTC, str).
             where   : self.timestamp
        2. unix_ret  : The unix-timestamp(UTC, float) of format "xxxxxxxxxx.x" .
             where   : self.unixlist
        3. data_temp : The spectrum of each sync-time. Only for available Back-End.
             where   : self.data
        """
        data_temp = []

        # Calculate UNIX-time
        # -------------------
        unixtime = self.timestamp_to_unixtime(req.timestamp)
        unix_ret = round(unixtime, 1)                                               # using xx.x [sec] format

        # append data to temporary list
        # -----------------------------
        reqlist = [req.POWER_BE1, req.POWER_BE2, req.POWER_BE3, req.POWER_BE4,
                   req.POWER_BE5, req.POWER_BE6, req.POWER_BE7, req.POWER_BE8,
                   req.POWER_BE9, req.POWER_BE10, req.POWER_BE11, req.POWER_BE12,
                   req.POWER_BE13, req.POWER_BE14, req.POWER_BE15, req.POWER_BE16,
                   req.POWER_BE17, req.POWER_BE18, req.POWER_BE19, req.POWER_BE20]
        for i in range(req.BE_num):
            data_temp.append(reqlist[i])

        # append return value
        # -------------------
        self.conti_timestamp.append(req.timestamp)
        self.conti_unixlist.append(unix_ret)
        self.conti_data.append(data_temp)
        return

    # Board Temperature Func
    # ----------------------

    def btemp_oneshot(self, sec, start):
        """
        DESCRIPTION
        ===========
        This is the master function of this class.
        This function returns XFFTS Board Temperature.
        You can choose observation time and start time.

        ARGUMENTS
        =========
        1. sec [sec] : observation time.
             Type          : int
             Default       : Nothing
        3. start [Unix time] : When to start observation in Unix-time.
             Type            : float (time.time like value)
             Default         : None
             Example         : time.time()+5
                             : 1508495163.9506361

        RETURNS
        =======
        1. unixlist : The Unix-time version of timelist.
             Type   : float list
             Length : Same as "sec"
        2. temperature : The Board Temperature.
             Type      : float array
             dim       : 3
             shape     : [sec(int), BE_num=16, 3]
        """

        # define data list
        # ----------------
        self.btemp_result = []
        self.btemp_unixlist = []
        self.btemp_data = []

        if start is None or start == 0: waittime = 0
        else: waittime = start - time.time()

        # subscribe data
        # --------------
        self.btemp_data_subscriber(sec=sec, waittime=waittime)

        # data sum
        # --------
        init_index = self.index_search(start=start, mode='temp')
        fin_index = init_index + sec
        if fin_index > len(self.btemp_unixlist):
            fin_index = len(self.btemp_unixlist)+1
        else:
            pass
        unixlist = self.btemp_unixlist[init_index:fin_index]
        templist = self.btemp_data[init_index:fin_index]
        print("unixlist's length: ",len(self.btemp_unixlist))
        return [unixlist, templist]

    def btemp_data_subscriber(self, sec, waittime):
        sub3 = rospy.Subscriber('XFFTS_TEMP', XFFTS_temp_msg, self.btemp_append)
        time.sleep(waittime+sec+1)
        sub3.unregister()

    def btemp_append(self, req):
        # Reform TimeStamp
        # ----------------
        unix_ret = round(req.timestamp)

        # append data to temporary list
        # -----------------------------
        data_temp = [req.TEMP_BE1, req.TEMP_BE2, req.TEMP_BE3, req.TEMP_BE4,
                     req.TEMP_BE5, req.TEMP_BE6, req.TEMP_BE7, req.TEMP_BE8,
                     req.TEMP_BE9, req.TEMP_BE10, req.TEMP_BE11, req.TEMP_BE12,
                     req.TEMP_BE13, req.TEMP_BE14, req.TEMP_BE15, req.TEMP_BE16,
                     req.TEMP_BE17, req.TEMP_BE18, req.TEMP_BE19, req.TEMP_BE20]

        # append return value
        # -------------------
        self.btemp_unixlist.append(unix_ret)
        self.btemp_data.append(data_temp)
        return


    def measure(self, integtime, repeat, start):
        th1 = threading.Thread(target=self.oneshot, args=(integtime, repeat, start))
        th2 = threading.Thread(target=self.conti_oneshot, args=(integtime, repeat, start))
        th1.setDaemon(True)
        th2.setDaemon(True)
        th1.start()
        th2.start()

        th1.join()
        th2.join()

        return self.spec_data, self.conti_data




def run(req):
    data = data_client()

    integtime = req.integtime
    repeat = req.repeat
    synctime = req.synctime
    start = req.timestamp + float(0.1)
    spec, conti = data.measure(integtime, repeat, start)

    timelist = spec[0]
    unixtime = spec[1]
    spectrum = numpy.array(spec[2])
    continuum = conti[2]
    
    print("\nunixtime\n",unixtime,"\n")
    print("spectrum\n",spectrum,"\n")
    print("continuum\n",continuum,"\n")
    
    for i in range(numpy.shape(spectrum)[1]):
        hdu1 = fits.PrimaryHDU(unixtime)
        hdu2 = fits.ImageHDU(spectrum[:, i, :])
        hdulist = fits.HDUList([hdu1, hdu2])
        hdulist.writeto(dir+'spec_{}-{}_BE{}_{}.fits'.format(integtime, repeat, i+1, round(unixtime[0][0])))
        pass
    
    return


if __name__ == '__main__':
    
    rospy.init_node('XFFTS')
    sub = rospy.Subscriber('XFFTS_parameter', XFFTS_para_msg, run)
    rospy.spin()

