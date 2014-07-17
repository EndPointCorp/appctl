#!/usr/bin/env python

from geodata.srv import GeodataQuery
import rospy
import math

DATA_HEADER_SIZE = 6

class GeodataLayer():
  def __init__(self, path):
    self.path = path
    with open(path) as f:
      lines = f.readlines()

    # parse the header
    params = map(lambda line: line.strip().split(), lines[0:DATA_HEADER_SIZE])

    self.ncols = int(GeodataLayer.read_param('ncols', params))
    self.nrows = int(GeodataLayer.read_param('nrows', params))
    self.xllcorner = float(GeodataLayer.read_param('xllcorner', params))
    self.yllcorner = float(GeodataLayer.read_param('yllcorner', params))
    self.cellsize = float(GeodataLayer.read_param('cellsize', params))
    self.nodata_value = float(GeodataLayer.read_param('NODATA_value', params))

    # load data points into 2d array of floats
    self.data = map(lambda l: map(float, l.split()), lines[DATA_HEADER_SIZE:])

  @staticmethod
  def read_param(key, params):
    return filter(lambda p: p[0] == key, params)[0][1]

  def query(self, req):
    lat = req.point.latitude
    lng = req.point.longitude

    # translate lat/lng to floating row/col
    x = (lng - self.xllcorner) / self.cellsize
    y = (lat - self.yllcorner) / self.cellsize

    # wrap longitude with the assumption that the data is 360
    x = x % self.ncols

    # linear average of all surrounding data points
    x_ratio = math.fmod(x, 1.0)
    x_low = int(math.floor(x) % self.ncols)
    x_high = int(math.ceil(x) % self.ncols)
    y_ratio = math.fmod(y, 1.0)
    y_low = int(math.floor(y))
    y_high = int(math.ceil(y))

    if y_low >= 0 and y_high < self.nrows:
      val = (
              self.data[y_low][x_low] * y_ratio * x_ratio + \
              self.data[y_high][x_low] / y_ratio * x_ratio + \
              self.data[y_low][x_high] * y_ratio / x_ratio + \
              self.data[y_high][x_high] / y_ratio / x_ratio
            ) / 4
    else:
      val = self.nodata_value

    return val

def geodata_server():
  rospy.init_node('geodata_server')

  # load a population count layer
  # http://sedac.ciesin.columbia.edu/data/set/gpw-v3-population-count-future-estimates/data-download
  # Population Count Grid Future Estimates, v3 (2015)
  layer = GeodataLayer('/tmp/glp15ag.asc')
  s = rospy.Service('geodata/population', GeodataQuery, layer.query)

  print 'ready.'

  rospy.spin()

if __name__ == '__main__':
  geodata_server()
