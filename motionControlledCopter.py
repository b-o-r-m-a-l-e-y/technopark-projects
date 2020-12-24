# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2014 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA  02110-1301, USA.
"""
Simple example that connects to the first Crazyflie found, logs the Stabilizer
and prints it to the console. After 10s the application disconnects and exits.
"""
import logging
import time
from threading import Timer

import cflib.crtp  # noqa
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig

from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

class LoggingExample:
    """
    Simple logging example class that logs the Stabilizer from a supplied
    link uri and disconnects after 5s.
    """

    def __init__(self, link_uri):
        """ Initialize and run the example with the specified link_uri """

        self._cf = Crazyflie(rw_cache='./cache')

        # Connect some callbacks from the Crazyflie API
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)
        self._dataPrevAx = 0
        self._dataPrevAy = 0
        self._dataPrevAz = 0

        self.flyXLeft = 0
        self.flyXRight = 0
        self.flyYForward = 0
        self.flyYBack = 0
        self.flyZUp = 0
        self.flyZDown = 0

        print('Connecting to %s' % link_uri)

        # Try to connect to the Crazyflie
        self._cf.open_link(link_uri)

        # Variable used to keep main loop occupied until disconnect
        self.is_connected = True

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        print('Connected to %s' % link_uri)

        # The definition of the logconfig can be made before connecting
        self._lg_stab = LogConfig(name='Accel', period_in_ms=400)
        self._lg_stab.add_variable('stateEstimate.ax', 'float')
        self._lg_stab.add_variable('stateEstimate.ay', 'float')
        self._lg_stab.add_variable('stateEstimate.az', 'float')

        # Adding the configuration cannot be done until a Crazyflie is
        # connected, since we need to check that the variables we
        # would like to log are in the TOC.
        try:
            self._cf.log.add_config(self._lg_stab)
            # This callback will receive the data
            self._lg_stab.data_received_cb.add_callback(self._stab_log_data)
            # This callback will be called on errors
            self._lg_stab.error_cb.add_callback(self._stab_log_error)
            # Start the logging
            self._lg_stab.start()
        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add Stabilizer log config, bad configuration.')

        # Start a timer to disconnect in 10s
        t = Timer(20, self._cf.close_link)
        t.start()

    def _stab_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print('Error when logging %s: %s' % (logconf.name, msg))

    def _stab_log_data(self, timestamp, data, logconf):
        """Callback from a the log API when data arrives"""
        #print('[%d][%s]: %s' % (timestamp, logconf.name, data))
       # print(data)
        ax = data['stateEstimate.ax']
        ay = data['stateEstimate.ay']
        az = data['stateEstimate.az']
        threshold = 1.0
        if (abs(ax-self._dataPrevAx)>threshold and ax>0 and self._dataPrevAx):
            print("Зафиксировано движение Вправо по оси Х")
            self.flyXRight = 1
            return
        if (abs(ax-self._dataPrevAx)<threshold and ax<0 and self._dataPrevAx):
            print("Зафиксировано движение Влево по оси Х")
            self.flyXLeft = 1
            return
        self._dataPrevAx = ax
        if (abs(ay-self._dataPrevAy)>threshold and ay>0 and self._dataPrevAy):
            print("Зафиксировано движение Вперёд по оси y")
            self.flyYForward = 1
            return
        if (abs(ay-self._dataPrevAy)<threshold and ay<0 and self._dataPrevAy):
            print("Зафиксировано движение Назад по оси y")
            self.flyYBack = 1
            return
        self._dataPrevAy = ay
        if (abs(az-self._dataPrevAz)>threshold and az>0 and self._dataPrevAz):
            print("Зафиксировано движение Вверх по оси z")
            self.flyZUp = 1
            return
        if (abs(az-self._dataPrevAz)<threshold and az<0 and self._dataPrevAz):
            print("Зафиксировано движение Вниз по оси z")
            self.flyZDown = 1
            return
        self._dataPrevAz = ax

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))
        self.is_connected = False

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)
        self.is_connected = False


if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)
    # Scan for Crazyflies and use the first one found
    uri = 'radio://0/80/2M/E7E7E7E7F2'
    URIFlying = 'radio://0/80/2M/E7E7E7E7E6'
    
    le = LoggingExample(uri)
    # The Crazyflie lib doesn't contain anything to keep the application alive,
    # so this is where your application should do something. In our case we
    # are just waiting until we are disconnected.
    with SyncCrazyflie(URIFlying, cf=Crazyflie(rw_cache='./cache')) as scf:
        # We take off when the commander is created
        with MotionCommander(scf) as mc:
            time.sleep(1)
            if (le.flyXLeft == 1):
                mc.left(0.2)
                le.flyXLeft = 0
            if (le.flyXRight == 1):
                mc.right(0.2)
                le.flyXRight = 0
            if (le.flyYForward == 1):
                mc.forward(0.2)
                le.flyYForward = 0
            if (le.flyYBack == 1):
                mc.back(0.2)
                le.flyYBack= 0
            if (le.flyZUp == 1):
                mc.up(0.2)
                le.flyZUp= 0
            if (le.flyZDown == 1):
                mc.down(0.2)
                le.flyZDown= 0
    #while le.is_connected:
        #time.sleep(1)
