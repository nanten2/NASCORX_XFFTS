#!/usr/bin/env python
import sys
import time
import rospy
import numpy
import socket
import struct
import signal
import threading
import psutil
import queue

sys.path.append('/home/amigos/ros/src/NASCORX_XFFTS')
import udp_client
from necst.msg import xffts_flag_msg
from nascorx_xffts.msg import XFFTS_spec_msg
from nascorx_xffts.msg import XFFTS_totalp_msg
from nascorx_xffts.msg import XFFTS_temp_msg


class data_server(object):
    header_size = 64
    BE_num_Max = 20
    _sock = None
    _stop_loop = False

    def __init__(self, host='localhost', port=25144):
        #self.connect(host, port)
        rospy.init_node('XFFTS_data_server')
        sub = rospy.Subscriber("XFFTS_DB_flag", xffts_flag_msg, self.callback, queue_size = 1)
        self.on = numpy.load("on_array.npy")
        self.off = numpy.load("off_array.npy")
        self.hot = numpy.load("hot_array.npy")
        self.spec = self.on
        self.queue = queue.Queue()
        pass

    def callback(self, req):
        print("check")
        if req.obs_mode == "ON":
            self.spec = self.on
        elif req.obs_mode == "OFF" or req.obs_mode == "SKY":
            self.spec = self.off
        else:
            self.spec = self.hot


    def connect(self, host, port):
        print('Create New Socket: host={}, port={}'.format(host, port))
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((host, port))
        self._sock = sock
        return

    def recv_header(self):
        """
        DESCRIPTION
        ===========
        This function receives and returns XFFTS data header.

        ARGUMENTS
        =========
        Nothing.

        RETURNS
        =======
        See "data_header" class.
        """
        header = self._sock.recv(self.header_size)
        return data_header(header)

    def recv_data(self, data_size):
        """
        DESCRIPTION
        ===========
        This function receives and returns XFFTS data.

        ARGUMENTS
        =========
        1. data_size  : Data size of XFFTS output. Maybe changing as BE_num changes.
             Type     : int
             Default  : Nothing.
             example  : header.data_size

        RETURNS
        =======
        1. data : The XFFTS spectrum in binary format.
            Type : binary
        """
        data = self._sock.recv(data_size, socket.MSG_WAITALL)
        return data

    def stop_loop(self):
        """
        DESCRIPTION
        ===========
        This function stops data relaying loop function.
        This function is also called when Ctrl-C is pressed.

        ARGUMENTS
        =========
        Nothing.

        RETURNS
        =======
        Nothing.
        """
        self._stop_loop = True
        return

    # Master function
    # ---------------

    def data_relaying_loop(self):
        """
        DESCRIPTION
        ===========
        This is the master function of this class.
        This function relays XFFTS data from XFFTS to FAC by ROS method.
        When you stop the loop, call stop_loop function.

        ARGUMENTS
        =========
        Nothing.

        RETURNS
        =======
        Nothing, but send values listed below to ROS subscriber.
        1. XFFTS_SPEC : Send spectrum data
            1. timestamp : XFFTS-format timestamp.
                Type     : str
                fmt      : '2017-10-20T09:34:13.9193PC  '
            2. BE_num    : Number of Back End Board.
                Number   : 1 - 16
                Type     : int
            3. SPEC_BE1-16 : The spectrum of each BE(1-16).
                Type       : float list
        2. XFFTS_PM : Send total power data(=continuum data).
            1. timestamp : Same as above.
            2. BE_num    : Same as above.
            3. POWER_BE1-16 : The total counts of each BE(1-16).
        """
        # Print Welcome Massage
        # ---------------------
        print('\n\n'
              '  =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n'
              '   Start : XFFTS Data Relaying Loop \n'
              '  =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-'
              '\n\n')

        # ROS setting
        # -----------
        d = {}
        import csv
        with open("beam_board.txt") as f:
            reader = csv.reader(f)
            for i in reader:
                d[i[0]] = i[1]
        self.pub = [rospy.Publisher("xffts_spec_{}".format(d[i]), XFFTS_spec_msg, queue_size=10) for i in d.keys()]
        self.pub2 = [rospy.Publisher("xffts_power_{}".format(d[i]), XFFTS_totalp_msg, queue_size=10) for i in d.keys()]

        xffts_spec = [XFFTS_spec_msg() for i in range(20)]
        xffts_totalp = [XFFTS_totalp_msg() for i in range(20)]
        #
        # ----------
        #proc = psutil.Process()
        #proc.nice(-20)

        # data relaying loop
        # ------------------
        c = 0
        while True:

            if self._stop_loop: break

            # get data
            # --------
            #header = self.recv_header()
            #rawdata = self.recv_data(header.data_size)
            #timestamp = header.timestamp.decode('utf-8')
            timestamp = time.time()
            timestamp = c
            c = c+1
            timestamp2 = str(time.time())
            #BE_num = header.BE_num
            # BE_num = BE_num_temp
            #print(header.timestamp, header.BE_num)

            # binary to float conversion
            # --------------------------
            spec = []
            pow = []
            counter = 0
            """
            for i in range(self.BE_num_Max):
                # For Available BE
                if i+1 <= header.BE_num:
                    #BE_num_temp, ch_num = struct.unpack('2I', rawdata[counter:counter+8])
                    #data_temp = list(struct.unpack('{}f'.format(ch_num), rawdata[counter+8:counter+8+ch_num*4]))
                    BE_num_temp, ch_num = numpy.frombuffer(rawdata[counter:counter+8], ("<i", 2)).tolist()[0]
                    data_temp = numpy.frombuffer(rawdata[counter+8:counter+8+ch_num*4], ("<f",32768)).tolist()[0]
                    pow_temp = sum(data_temp)
                    spec.append(data_temp)
                    pow.append(pow_temp)
                    counter += 8 + ch_num * 4
                # For Unavailable BE
                elif header.BE_num <= i+1:
                    spec.append([0])
                    pow.append(0)
            """
            for i in range(20):
                spec.append([2.34567]*32768)
                pow.append(8)
            # ROS Data Trans
            # --------------
            # Spectrum
            for i in range(20):
                xffts_spec[i].timestamp = timestamp
                xffts_spec[i].spec = spec[i]

            for i in range(20):
                xffts_totalp[i].timestamp = timestamp
                xffts_totalp[i].total_power = pow[i]
                
            self.queue.put([xffts_spec, xffts_totalp])
            time.sleep(0.07)
            

        # Print Shut Down Massage
        # -----------------------
        print('\n\n'
              '  =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n'
              '   Shut Down : XFFTS Data Relaying Loop \n'
              '  =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-'
              '\n\n')
        return

    def publish(self):
        while not rospy.is_shutdown():
            #print(self.queue.qsize())
            tmp = self.queue.get()
            for i in range(20):
                self.pub[i].publish(tmp[0][i])
                self.pub2[i].publish(tmp[1][i])
            #self.pub.publish(tmp[0])
            #self.pub2.publish(tmp[1])
            time.sleep(0.001)

    def temp_relaying_loop(self):
        """
        DESCRIPTION
        ===========
        This is the master function of this class.
        This function relays XFFTS Board temperature from XFFTS to FAC by ROS method.
        When you stop the loop, call stop_loop function.

        ARGUMENTS
        =========
        Nothing.

        RETURNS
        =======
        Nothing, but send values listed below to ROS subscriber.
        1. XFFTS_SPEC : Send spectrum data
            1. timestamp : XFFTS-format timestamp.
                Type     : str
                fmt      : '2017-10-20T09:34:13.9193PC  '
            2. BE_num    : Number of Back End Board.
                Number   : 1 - 16
                Type     : int
            3. SPEC_BE1-16 : The spectrum of each BE(1-16).
                Type       : float list
        2. XFFTS_PM : Send total power data(=continuum data).
            1. timestamp : Same as above.
            2. BE_num    : Same as above.
            3. POWER_BE1-16 : The total counts of each BE(1-16).
        """

        # ROS setting
        # -----------
        pub3 = rospy.Publisher('XFFTS_TEMP', XFFTS_temp_msg, queue_size=10)
        XFFTS_TEMP = XFFTS_temp_msg()

        # define UDP connection
        # ---------------------
        udp = udp_client.udp_client()
        udp.open()

        while True:

            if self._stop_loop: break

            # get data
            # --------
            timestamp = time.time()
            temps = udp.query_all_temperature()
            used = udp.query_usedsections()

            # data reform
            # -----------
            for i, temp in enumerate(temps):
                if temp is None: temps[i] = [0]
                else: pass

            # ROS Data Trans
            # --------------
            XFFTS_TEMP.timestamp = timestamp
            #XFFTS_TEMP.USED_BE = used
            XFFTS_TEMP.TEMP_BE1 = temps[0]
            XFFTS_TEMP.TEMP_BE2 = temps[1]
            XFFTS_TEMP.TEMP_BE3 = temps[2]
            XFFTS_TEMP.TEMP_BE4 = temps[3]
            XFFTS_TEMP.TEMP_BE5 = temps[4]
            XFFTS_TEMP.TEMP_BE6 = temps[5]
            XFFTS_TEMP.TEMP_BE7 = temps[6]
            XFFTS_TEMP.TEMP_BE8 = temps[7]
            XFFTS_TEMP.TEMP_BE9 = temps[8]
            XFFTS_TEMP.TEMP_BE10 = temps[9]
            XFFTS_TEMP.TEMP_BE11 = temps[10]
            XFFTS_TEMP.TEMP_BE12 = temps[11]
            XFFTS_TEMP.TEMP_BE13 = temps[12]
            XFFTS_TEMP.TEMP_BE14 = temps[13]
            XFFTS_TEMP.TEMP_BE15 = temps[14]
            XFFTS_TEMP.TEMP_BE16 = temps[15]
            pub3.publish(XFFTS_TEMP)

            time.sleep(1)
        return

    def start_thread(self):
        th = threading.Thread(target=self.data_relaying_loop)
        th.setDaemon(True)
        th.start()

        th2 = threading.Thread(target=self.temp_relaying_loop)
        th2.setDaemon(True)
        th2.start()
        
        th3 = threading.Thread(target=self.publish)
        th3.setDaemon(True)
        th3.start()
        return


class data_header(object):
    header_size = 64

    def __init__(self, header):
        self.ieee = struct.unpack('<4s', header[0:4])[0]
        self.data_format = struct.unpack('4s', header[4:8])[0]
        self.package_length = struct.unpack('I', header[8:12])[0]
        self.BE_name = struct.unpack('8s', header[12:20])[0]
        self.timestamp = struct.unpack('28s', header[20:48])[0]
        self.integration_time = struct.unpack('I', header[48:52])[0]
        self.phase_number = struct.unpack('I', header[52:56])[0]
        self.BE_num = struct.unpack('I', header[56:60])[0]
        self.blocking = struct.unpack('I', header[60:64])[0]
        self.data_size = self.package_length - self.header_size
        return

if __name__ == '__main__':
    serv = data_server()

    # Signal handler
    # --------------
    def signal_handler(num, flame):
        serv.stop_loop()
        sys.exit()
    signal.signal(signal.SIGINT, signal_handler)

    serv.start_thread()
    rospy.spin()


# History
# -------
# written by T.Inaba
# 2017/10/25 T.Inaba : add board temperature function & main thread.
# 2017/11/01 T.Inaba : add counter method to index of rawdata instead of data-size method.
