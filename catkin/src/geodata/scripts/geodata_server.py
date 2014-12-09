#!/usr/bin/env python

from geodata.srv import GeodataQuery
import rospy
import math
import numpy as np
import os


class GeodataLayer():
    def __init__(self, path):
        with open(path) as f:
            d = np.load(f)
            params = dict(d['params'])
            self.data = d['d']

        self.ncols = int(params['ncols'])
        self.nrows = int(params['nrows'])
        self.xllcorner = float(params['xllcorner'])
        self.yllcorner = float(params['yllcorner'])
        self.cellsize = float(params['cellsize'])
        self.nodata_value = float(params['nodata_value'])

    def query(self, req):
        lat = req.point.latitude
        lng = req.point.longitude
        rad = max(0, int(round(req.radius * 4 * self.cellsize)))

        # translate lat/lng to floating row/col
        x = int(round((lng - self.xllcorner) / self.cellsize))
        y = int(round((lat - self.yllcorner) / self.cellsize))

        # check latitude bounds
        if y >= self.nrows - rad or y <= rad:
            return self.nodata_value

        # invert latitude
        y = self.nrows - y

        # make a subarray centered around the point
        if rad > 0:
            sub = self.data[y-rad:y+rad, x-rad:x+rad]
            val = sub.sum()
        else:
            val = self.data[y, x]

        return val


def geodata_server():
    rospy.init_node('geodata_server')

    # load a population count layer
    # http://sedac.ciesin.columbia.edu/data/set/gpw-v3-population-count-future-estimates/data-download
    # Population Count Grid Future Estimates, v3 (2015)
    src_path = rospy.get_param(
        '~src',
        '/opt/ros/{}/share/geodata/geodata_population.npz'.format(
            os.environ['ROS_DISTRO']
        )
    )
    layer = GeodataLayer(src_path)

    s = rospy.Service('geodata/population', GeodataQuery, layer.query)

    print 'ready.'

    rospy.spin()

if __name__ == '__main__':
    geodata_server()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
