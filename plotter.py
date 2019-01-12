import geopy
from geopy.distance import distance
from geopy.geocoders import GoogleV3
from geopy.point import Point
import math
import numpy as np
from random import randint, choice, random
from datetime import datetime
import time
import requests
import json
import re
from math import radians, degrees, cos, sin, sqrt, atan2, asin, fabs, pi
from collections import namedtuple
from mpl_toolkits.basemap import Basemap
import matplotlib.pyplot as plt
import re
from threading import Thread

class Plotter():

    def __init__(self):
        print("initialising plotter")

    def plot_route(self, coords):
        min_lat, min_long, max_lat, max_long = 0, 0, 0, 0
        for c in coords:
            min_lat = min(c[0], min_lat)
            max_lat = max(c[0], max_lat)
            min_long = min(c[1], min_long)
            max_long = max(c[1], max_long)
        min_lat -= 15
        min_long -= 15
        max_lat += 15
        max_long += 15
        mid_lat, mid_long = max_lat / 2, max_long / 2
        map = Basemap(projection='merc', lat_0=mid_lat, lon_0=mid_long,
        resolution = 'l', area_thresh = 0.1,
        llcrnrlon = min_long, llcrnrlat = min_lat,
        urcrnrlon = max_long, urcrnrlat = max_lat)
        map.drawcoastlines()
        map.drawcountries()
        map.fillcontinents(color='c')
        map.drawmapboundary()
        for lat, lon in coords:
            x, y = map(lon, lat)
            map.plot(x, y, 'bo', markersize=2)
        plt.draw()
        plt.show()

class Point():

    def __init__(self, x, y):
        self.x = x
        self.y = y

class Navigator(object):

    def __init__(self, pos=(0.1, 52.2)):
        self.pos = Point(pos[0], pos[1])
        print("navigator initialised")
        print("current position: ({}, {})".format(self.pos.y, self.pos.x))

    def bearing(self, destination):
        A, B = self.pos, destination
        dLon = radians(B.x - A.x)
        lat1 = radians(A.y)
        lat2 = radians(B.y)
        y = sin(dLon) * cos(lat2)
        x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon)
        return atan2(y, x)

    def bearing_degrees(self, destination):
        return degrees(self.bearing(destination))

    def midpoint(self, destination, start=None):
        if start:
            A = Point(start[0], start[1])
        else:
            A = self.pos
        B = Point(destination[0], destination[1])
        if A.x == B.x: return Point(A.x, (A.y + B.y) / 2)
        if A.y == B.y: return Point((A.x + B.x) / 2, A.y)
        lon1, lat1 = radians(A.x), radians(A.y)
        lon2, lat2 = radians(B.x), radians(B.y)
        dLon = lon2 - lon1
        Bx = cos(lat2) * cos(dLon)
        By = cos(lat2) * sin(dLon)
        lat3 = atan2(sin(lat1) + sin(lat2), sqrt((cos(lat1) + Bx) * (cos(lat1) + Bx) + By * By))
        lon3 = lon1 + atan2(By, cos(lat1) + Bx)
        return Point(degrees(lon3), degrees(lat3))

    def generate_waypoints(self, destination, sectors, plot=False):
        """
        :param destination: destination as point object (x = long, y = lat)
        :param interval: number of waypoints as divisor of route distance e.g. 10
        :param plot: whether to plot the waypoints
        :return: list of waypoints e.g. (52, -5)
        """
        fractions = [round((1 / sectors) * x, 2) for x in range(sectors)][1:]
        waypoints = [self.get_waypoint(destination, f) for f in fractions]
        if plot:
            p = Plotter()
            p.plot_route(waypoints)

    def get_waypoint(self, destination, fraction):
        """
        :param destination: destination as point object (x = long, y = lat)
        :param fraction: fraction of the trajectory covered
        :return: coordinate of waypoint at fraction
        """
        f = fraction
        lat1, long1 = radians(self.pos.y), radians(self.pos.x)
        lat2, long2 = radians(destination.y), radians(destination.x)
        d = distance((self.pos.y, self.pos.x), (destination.y, destination.x)).kilometers
        d /= 6371
        a = sin((1 - f) * d) / sin(d)
        b = sin(f * d) / sin(d)
        x = a * cos(lat1) * cos(long1) + b * cos(lat2) * cos(long2)
        y = a * cos(lat1) * sin(long1) + b * cos(lat2) * sin(long2)
        z = a * sin(lat1) + b * sin(lat2)
        wp_lat = degrees(atan2(z, sqrt((x * x) + (y * y))))
        wp_long = degrees(atan2(y, x))
        return ((wp_lat, wp_long))
