#!/usr/bin/env python

from geodata.srv import GeodataQuery
import rospy
import math
import numpy as np

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
    self.data = np.array(
      map(lambda l: l.split(), lines[DATA_HEADER_SIZE:]),
      dtype=np.float32
    )

    #self.rad = 4
    #self.weights = np.zeros((self.rad*2,self.rad*2))
    #yog,xog = np.ogrid[
    #  -self.rad:self.rad,
    #  -self.rad:self.rad
    #]
    #self.mask = xog*xog + yog*yog <= self.rad * self.rad
    #self.weights[self.mask] = 1

  @staticmethod
  def read_param(key, params):
    return filter(lambda p: p[0] == key, params)[0][1]

  def query(self, req):
    lat = req.point.latitude
    lng = req.point.longitude
    rad = int(round(req.radius * self.cellsize))

    # translate lat/lng to floating row/col
    x = int(round((lng - self.xllcorner) / self.cellsize))
    y = int(round((lat - self.yllcorner) / self.cellsize))

    # check latitude bounds
    if y >= self.nrows - rad or y <= rad:
      return self.nodata_value

    # invert latitude
    y = self.nrows - y

    # make a subarray centered around the point
    sub = self.data[y-rad:y+rad, x-rad:x+rad]
    val = sub.sum()

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
