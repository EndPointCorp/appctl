#!/usr/bin/env python

import math
import numpy as np
import rospy

DATA_HEADER_SIZE = 6


class GeodataLayer():
    def __init__(self, path):
        self.path = path
        with open(path) as f:
            lines = f.readlines()

        # parse the header
        def sanitize_line(line):
            return line.strip().split()

        def read_param(key, params):
            return filter(lambda p: p[0] == key, params)[0][1]

        params = map(sanitize_line, lines[0:DATA_HEADER_SIZE])

        self.ncols = int(read_param('ncols', params))
        self.nrows = int(read_param('nrows', params))
        self.xllcorner = float(read_param('xllcorner', params))
        self.yllcorner = float(read_param('yllcorner', params))
        self.cellsize = float(read_param('cellsize', params))
        self.nodata_value = float(read_param('NODATA_value', params))

        # load data points into 2d array of floats
        self.data = np.array(
            map(lambda l: l.split(), lines[DATA_HEADER_SIZE:]),
            dtype=np.float32
        )

        self.params = np.array(
            [
                ('ncols', self.ncols),
                ('nrows', self.nrows),
                ('xllcorner', self.xllcorner),
                ('yllcorner', self.yllcorner),
                ('cellsize', self.cellsize),
                ('nodata_value', self.nodata_value)
            ],
            dtype=np.string_
        )


def geodata_export():
    src_path = rospy.get_param('~src', '/tmp/glp15ag.asc')
    dst_path = rospy.get_param('~dst', '/tmp/geodata_population.npz')

    layer = GeodataLayer(src_path)

    np.savez_compressed(dst_path, params=layer.params, d=layer.data)

    print 'done.'

if __name__ == '__main__':
    geodata_export()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
