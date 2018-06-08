#!/usr/bin/env python

import rospy
import astropy.io.fits as fits

from NASCORX_XFFTS.msg import XFFTS_msg

dir = '/home/amigos/ros/src/NASCORX_XFFTS/data_all/'



class data_saver(object):

    def __init__(self):
        pass

    def data_subscriber(self):
        rospy.init_node('XFFTS_SAVER')
        sub = rospy.Subscriber('XFFTS_SPEC', XFFTS_msg, self.save)
        rospy.spin()
        
        return

    def save(self, req):

        timestamp = req.timestamp
        datalist = [req.SPEC_BE1, req.SPEC_BE2, req.SPEC_BE3, req.SPEC_BE4,
                req.SPEC_BE5, req.SPEC_BE6, req.SPEC_BE7, req.SPEC_BE8,
                req.SPEC_BE9, req.SPEC_BE10, req.SPEC_BE11, req.SPEC_BE12,
                req.SPEC_BE13, req.SPEC_BE14, req.SPEC_BE15, req.SPEC_BE16,
                req.SPEC_BE17, req.SPEC_BE18, req.SPEC_BE19, req.SPEC_BE20]
        
        hdr = fits.Header()
        hdr['TIME'] = timestamp
        hdu = fits.PrimaryHDU(datalist, header=hdr)
        hdu.writeto(dir+'data_{}.fits'.format(timestamp))
        
        return

    def saver(self):
        print("\n"
                " Start : XFFTS Data Saving System ""\n")

        self.data_subscriber()
        
        print("\n"
                " Shut Down : XFFTS Data Saving System ""\n")

        return
