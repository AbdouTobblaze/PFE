'''
	@author : Tobbal Abdelhafid
	@brief	: A simple script to connect the PYNQ board to wifi throught a wifi dongle 
'''
import argparse 		# to parse commands
from pynq.lib import Wifi 	# python's pynq library to connect to a wifi

ap = argparse.ArgumentParser()
ap.add_argument("-s", "--ssid", required=True, help="Wifi access point's name")
ap.add_argument("-p", "--password", required=True, help="Wifi password")


print(args["ssid"])
print(args["password"])