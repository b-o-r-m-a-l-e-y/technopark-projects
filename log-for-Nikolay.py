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
import numpy as np
import matplotlib.pyplot as plt


import cflib.crtp  # noqa
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig

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

        self.dataX = []
        self.dataY = []
        self.dataZ = []

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
        self._lg_stab = LogConfig(name='PosData', period_in_ms=100)
        self._lg_stab.add_variable('stateEstimate.x', 'float')
        self._lg_stab.add_variable('stateEstimate.y', 'float')
        self._lg_stab.add_variable('stateEstimate.z', 'float')

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
        t = Timer(10, self._cf.close_link)
        t.start()

    def _stab_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print('Error when logging %s: %s' % (logconf.name, msg))

    def _stab_log_data(self, timestamp, data, logconf):
        """Callback from a the log API when data arrives"""
        #print('[%d][%s]: %s' % (timestamp, logconf.name, data))
        #print(data)
        self.dataX.append(data['stateEstimate.x'])
        self.dataY.append(data['stateEstimate.y'])
        self.dataZ.append(data['stateEstimate.z'])

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
    '''
    cflib.crtp.init_drivers(enable_debug_driver=False)
    # Scan for Crazyflies and use the first one found
    uri = 'radio://0/80/2M/E7E7E7E7E5'
    
    le = LoggingExample(uri)
    # The Crazyflie lib doesn't contain anything to keep the application alive,
    # so this is where your application should do something. In our case we
    # are just waiting until we are disconnected.
    while le.is_connected:
        time.sleep(1)

    print(le.dataX)
    print(le.dataY)
    '''
    a = [-4.18819522857666, -4.188503265380859, -4.18867301940918, -4.18831205368042, -4.188274383544922, -4.190276145935059, -4.190556049346924, -4.19222354888916, -4.193119049072266, -4.192699909210205, -4.1937384605407715, -4.195711135864258, -4.1934075355529785, -4.197160720825195, -4.200630187988281, -4.205377101898193, -4.204904079437256, -4.205684661865234, -4.210937976837158, -4.208333492279053, -4.207971096038818, -4.210206031799316, -4.208774566650391, -4.209221839904785, -4.209522724151611, -4.205291748046875, -4.211694717407227, -4.211846828460693, -4.209316730499268, -4.216280460357666, -4.217368125915527, -4.218880653381348, -4.214470386505127, -4.215321063995361, -4.223378658294678, -4.228857517242432, -4.2234110832214355, -4.229701519012451, -4.233022689819336, -4.23349142074585, -4.235903739929199, -4.236464023590088, -4.239033222198486, -4.2393598556518555, -4.242741584777832, 
-4.245118141174316, -4.24627161026001, -4.248228073120117, -4.2429890632629395, -4.248841285705566, -4.25048303604126, -4.250890731811523, -4.252286911010742, -4.255359649658203, -4.252754211425781, -4.256353855133057, -4.252493381500244, -4.251247406005859, -4.25228214263916, -4.250643730163574, -4.254970073699951, -4.255213260650635, -4.254988193511963, -4.2521162033081055, -4.253142833709717, -4.2554731369018555, -4.25645637512207, -4.258654594421387, -4.25919771194458, -4.25652551651001, -4.260899543762207, -4.26065731048584, -4.26123571395874, -4.271770477294922, -4.262786388397217, -4.255111217498779, -4.253579139709473, -4.252880573272705, -4.2521653175354, -4.254368305206299, -4.255382061004639, -4.257903099060059, -4.256897926330566, -4.2533721923828125, -4.251129627227783, -4.25206995010376, -4.252162933349609, -4.254453659057617, -4.257375717163086, -4.248505115509033, -4.256804943084717, -4.266366958618164, -4.2694091796875, -4.273106575012207, -4.276010036468506, -4.2797322273254395, -4.278544902801514, -4.284336566925049, -4.288623332977295]

    print(np.sqrt(np.var(a)))
    
    plt.plot([1, 2, 3, 4])
    plt.ylabel('some numbers')
    plt.show()
