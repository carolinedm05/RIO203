'''
**********************************************************************
* Filename    : dht11.py
* Description : test for SunFoudner DHT11 humiture & temperature module
* Author      : Dream
* Brand       : SunFounder
* E-mail      : service@sunfounder.com
* Website     : www.sunfounder.com
* Update      : Dream    2016-09-30    New release
**********************************************************************
'''
#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time
import requests
import os
import random

import serial, string
import numpy as np
import struct

ser = serial.Serial('/dev/ttyUSB0',115200,timeout=1)


"""while True : 
    output = ser.readline()
    #print(output)
    output = output.decode()
    data = output.split()
    #print(data)
    if len(data)==3:
        Rssi1, Rssi2, Rssi3 = map(int,data)
        print("Rssi1:", Rssi1,"Rssi2:", Rssi2,"Rssi3:", Rssi3)
    vecteur = [Rssi1,Rssi2,Rssi3]
    print (vecteur)
    time.sleep(1)"""
    
    

DHTPIN = 22

GPIO.setmode(GPIO.BCM)

MAX_UNCHANGE_COUNT = 100

STATE_INIT_PULL_DOWN = 1
STATE_INIT_PULL_UP = 2
STATE_DATA_FIRST_PULL_DOWN = 3
STATE_DATA_PULL_UP = 4
STATE_DATA_PULL_DOWN = 5

def read_dht11_dat():
	GPIO.setup(DHTPIN, GPIO.OUT)
	GPIO.output(DHTPIN, GPIO.HIGH)
	time.sleep(0.05)
	GPIO.output(DHTPIN, GPIO.LOW)
	time.sleep(0.02)
	GPIO.setup(DHTPIN, GPIO.IN, GPIO.PUD_UP)

	unchanged_count = 0
	last = -1
	data = []
	while True:
		current = GPIO.input(DHTPIN)
		data.append(current)
		if last != current:
			unchanged_count = 0
			last = current
		else:
			unchanged_count += 1
			if unchanged_count > MAX_UNCHANGE_COUNT:
				break

	state = STATE_INIT_PULL_DOWN

	lengths = []
	current_length = 0

	for current in data:
		current_length += 1

		if state == STATE_INIT_PULL_DOWN:
			if current == GPIO.LOW:
				state = STATE_INIT_PULL_UP
			else:
				continue
		if state == STATE_INIT_PULL_UP:
			if current == GPIO.HIGH:
				state = STATE_DATA_FIRST_PULL_DOWN
			else:
				continue
		if state == STATE_DATA_FIRST_PULL_DOWN:
			if current == GPIO.LOW:
				state = STATE_DATA_PULL_UP
			else:
				continue
		if state == STATE_DATA_PULL_UP:
			if current == GPIO.HIGH:
				current_length = 0
				state = STATE_DATA_PULL_DOWN
			else:
				continue
		if state == STATE_DATA_PULL_DOWN:
			if current == GPIO.LOW:
				lengths.append(current_length)
				state = STATE_DATA_PULL_UP
			else:
				continue
	if len(lengths) != 40:
		#print ("Data not good, skip")
		return False

	shortest_pull_up = min(lengths)
	longest_pull_up = max(lengths)
	halfway = (longest_pull_up + shortest_pull_up) / 2
	bits = []
	the_bytes = []
	byte = 0

	for length in lengths:
		bit = 0
		if length > halfway:
			bit = 1
		bits.append(bit)
	#print ("bits: %s, length: %d" % (bits, len(bits)))
	for i in range(0, len(bits)):
		byte = byte << 1
		if (bits[i]):
			byte = byte | 1
		else:
			byte = byte | 0
		if ((i + 1) % 8 == 0):
			the_bytes.append(byte)
			byte = 0
	#print (the_bytes)
	checksum = (the_bytes[0] + the_bytes[1] + the_bytes[2] + the_bytes[3]) & 0xFF
	if the_bytes[4] != checksum:
		#print ("Data not good, skip")
		return False

	return the_bytes[0], the_bytes[2]


fingerprints_dic = {}
fingerprints_dic[1] = ([42,30,28], 0.1, 0.1)
fingerprints_dic[2] = ([35,35,20], 0.3, 0.1)
fingerprints_dic[3] = ([36,50,23], 0.5, 0.1)
fingerprints_dic[4] = ([28,40,20], 0.7, 0.1)
fingerprints_dic[5] = ([30,50,30], 0.9, 0.1)
fingerprints_dic[6] = ([31,42,30], 0.1, 0.3)
fingerprints_dic[7] = ([27,43,28], 0.3, 0.3)
fingerprints_dic[8] = ([41,44,30], 0.5, 0.3)
fingerprints_dic[9] = ([35,44,28], 0.7, 0.3)
fingerprints_dic[10] = ([40,35,25], 0.9, 0.3)
fingerprints_dic[11] = ([50,35,22], 0.1, 0.5)
fingerprints_dic[12] = ([48,35,24], 0.3, 0.5)
fingerprints_dic[13] = ([37,30,24], 0.5, 0.5)
fingerprints_dic[14] = ([35,30,36], 0.7, 0.5)
fingerprints_dic[15] = ([27,36,36], 0.9, 0.5)
fingerprints_dic[16] = ([31,26,40], 0.1, 0.7)
fingerprints_dic[17] = ([30,24,30], 0.3, 0.7)
fingerprints_dic[18] = ([40,25,30], 0.5, 0.7)
fingerprints_dic[19] = ([45,35,27], 0.7, 0.7)
fingerprints_dic[20] = ([45,23,25], 0.9, 0.7)
fingerprints_dic[21] = ([30,25,22], 0.1, 0.9)
fingerprints_dic[22] = ([30,21,31], 0.3, 0.9)
fingerprints_dic[23] = ([33,24,36], 0.5, 0.9)
fingerprints_dic[24] = ([32,20,43], 0.7, 0.9)
fingerprints_dic[25] = ([24,28,46], 0.9, 0.9)


def closest_RSSI(vec,vec_dict):
    min_distance = float('inf')
    closest_id = None
    vec = np.array(vec)
    for id, v in vec_dict.items():
        v = np.array(v[0])
        distance = np.linalg.norm(vec - v)
        if distance < min_distance:
            min_distance = distance
            closest_id = id
    return vec_dict[closest_id][1], vec_dict[closest_id][2]



def main():
    print ("Raspberry Pi wiringPi DHT11 Temperature test program\n")
    while True:
        result = read_dht11_dat()
        if result:
            output = ser.readline()
            output = output.decode()
            data = output.split()
            if len(data)==3:
                Rssi1, Rssi2, Rssi3 = map(int,data)
                #print("Rssi1:", Rssi1,"Rssi2:", Rssi2,"Rssi3:", Rssi3)
            vecteur = [Rssi1,Rssi2,Rssi3]
            #print (vecteur)
            print(closest_RSSI(vecteur,fingerprints_dic))
            humidity, temperature = result
            print ("humidity: %s %%,  Temperature: %s C`" % (humidity, temperature))
            longitude, latitude = closest_RSSI(vecteur,fingerprints_dic)
            #toto=(random.randrange(1,10,1))/10
            #tata=(random.randrange(1,10,1))/10
            #os.system('curl -X POST -d "{"salle01_humidite":'+str(humidity)+',"salle01_temperature":'+str(temperature)+',"salle02_humidite":'+str(humidity+2)+',"salle02_temperature":'+str(temperature+2)+',"salle03_humidite":'+str(humidity+3)+',"salle03_temperature":'+str(temperature+3)+',"salle04_humidite":'+str(humidity-1)+',"salle04_temperature":'+str(temperature-1)+',"salle05_humidite":'+str(humidity-3)+',"salle05_temperature":'+str(temperature-3)+',"salle06_humidite":'+str(humidity-5)+',"salle06_temperature":'+str(temperature-5)+'}" https://demo.thingsboard.io/api/v1/RIO203GRPCARO/telemetry --header "Content-Type:application/json"')
            os.system('curl -X POST -d "{"salle01_humidite":'+str(humidity)+',"salle01_temperature":'+str(temperature)+'}" https://demo.thingsboard.io/api/v1/RIO203T01/telemetry --header "Content-Type:application/json"')
            os.system('curl -X POST -d "{"latitude":'+str(latitude)+',"longitude":'+str(longitude)+'}" https://demo.thingsboard.io/api/v1/RIO203POS01/telemetry --header "Content-Type:application/json"')
    


"""while True : 
    output = ser.readline()
    #print(output)
    output = output.decode()
    data = output.split()
    #print(data)
    if len(data)==3:
        Rssi1, Rssi2, Rssi3 = map(int,data)
        print("Rssi1:", Rssi1,"Rssi2:", Rssi2,"Rssi3:", Rssi3)
    vecteur = [Rssi1,Rssi2,Rssi3]
    print (vecteur)
    time.sleep(1)
    main()"""


    
#PAr salle : gaz temp humidity flamme
#par zone : buzz_incendie


#headers = {'Content-Type': 'application/json',}
#payload = {'temperature':'15','temperature01':'11'}
#url = 'http://192.168.56.10:8080/api/v1/MYTOKEN/telemetry'
#print ("eee %s %%",headers)
#print ("rrr %s %%",payload)
#print ("ttt %s %%",url)
#print ("ttt %s %%",humidity)
#r=requests.post('http://192.168.56.10:8080/api/v1/MYTOKEN/telemetry', data={'temperature':'15','temperature01':'11'}, headers={'Content-Type': 'application/json',})
#os.system('curl -X POST -d "{"temperature":'+humidity+',"temperature01":'+temperature+'}" http://192.168.56.10:8080/api/v1/MYTOKEN/telemetry --header "Content-Type:application/json"')
time.sleep(1)

def destroy():
	GPIO.cleanup()

if __name__ == '__main__':
	try:
		main()
	except KeyboardInterrupt:
		destroy() 
