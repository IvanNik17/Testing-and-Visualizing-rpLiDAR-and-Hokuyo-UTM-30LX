import serial
import numpy as np
import timeit

def _getData(ser):
	data = []
	while True:
		rep = ser.readline()
		rep = rep.decode()
		if rep == '\n':
			break
		else:
			data.append(rep)

	return data

def _sumCheck(string, sumChar):
	lineSum = 0
	for x in list(string):
		lineSum = lineSum + ord(x)

	mask = (1 << 6) - 1
	little = lineSum & mask
	little = little + 0x30

	if chr(little) == sumChar:
		return True
	
	return False

def _checkAndFormatData(data, command):
	error = 'noError'

	if data[0] != command:
		error = 'commandEccoError'
		return None, error

	if command == 'VV\n' or command == 'PP\n' or command == 'II\n':
		if not _sumCheck(data[1][:-2],data[1][-2]):
			error = 'sumCheckError'
			return None, error

		for x in data[2:]:
			if not _sumCheck(x[:-3],x[-2]):
				error = 'sumCheckError'
				return None, error

		# Format data for return
		data = [x[:-3] for x in data[1:]]

	else:
		for x in data[1:]:
			if not _sumCheck(x[:-2],x[-2]):
				error = 'sumCheckError'
				return None, error
		# Format data for return
		data = [x[:-2] for x in data[1:]]


	return data, error

def _sendCommand(ser, command):
	data = []
	error = ''

	ser.write(command.encode())
	if command[0] == 'M':
		data = _getData(ser)
		# Not Implemented!
		pass
	else:
		data = _getData(ser)
		data, error = _checkAndFormatData(data,command)
	return data, error

def _dataToRanges(data):
	# Concat all data in to one array
	data = ''.join(data)

	# Calculate the ranges
	rangeData = []
	for i in range(0,len(data),3):
		a = ord(data[i]) - 0x30;
		b = ord(data[i+1]) - 0x30;
		c = ord(data[i+2]) - 0x30;

		rangeData.append((((a << 6) + b) << 6) + c)

	return rangeData

def _dataToTimestamp(data):
	# Concat all data in to one array
	data = ''.join(data)

	# Calculate the ranges
	a = ord(data[0]) - 0x30;
	b = ord(data[1]) - 0x30;
	c = ord(data[2]) - 0x30;
	d = ord(data[3]) - 0x30;

	timeStamp = (((((a << 6) + b) << 6) + c) << 6) + d

	return timeStamp


def _calcAngels(start, end, clusterCount):
	anglePrStep = 0.25

	# Calculate angels
	angleData = []
	for i in range(end+1, start, -clusterCount):
		angleData.append(((i * anglePrStep + 180 * anglePrStep)+180)%360)

	return angleData

def _scanToNpArray(data, start, end, clusterCount):
	dataRanges = _dataToRanges(data)
	dataAngles = _calcAngels(start, end, clusterCount)

	# Fill npArray
#	npMeasurements = np.zeros((len(dataRanges),2))
	 
	npMeasurements = np.array([dataAngles,dataRanges]).T  
#	for idx, x in enumerate(dataAngles):
#		npMeasurements[idx,0] = x 
#	for idx, x in enumerate(dataRanges):
#		npMeasurements[idx,1] = x 

	# return npSamples
	return npMeasurements




def connect(port):
	ser = serial.Serial(port, 115200, timeout=1)
	return ser

def getLatestScanSteps(lidar, start, end, clusterCount):
	command = 'GD' + str(start).zfill(4) + str(end).zfill(4) + str(clusterCount).zfill(2) + '\n'
	status = None

	data, error = _sendCommand(lidar, command)

	if error != 'noError':
		return error

	# Check status
	if data[0] == '00':
		status = 'gotLastScan'
	elif data[0] == '01':
		status = 'startStepNonNumericValue'
	elif data[0] == '02':
		status = 'endStepNonNumericValue'
	elif data[0] == '03':
		status = 'clusterCountNonNumericValue'
	elif data[0] == '04':
		status = 'endStepOutOfRange'
	elif data[0] == '05':
		status = 'endStepSmallThanStartStep'
	elif data[0] == '10':
		status = 'laserIsOff'
	else:
		status = 'HardwareError'

	# Calculate TimeStamp in ms
#	timeStamp = _dataToTimestamp(data[1])

	# Convert Data
	npData = _scanToNpArray(data[2:], start, end, clusterCount)


	return npData, status


# Not implemented, Need?
# def getMultipleScanSteps(lidar, start, end, clusterCount, scanInterval, nrScans):
	# command = 'MD' + str(start).zfill(4) + str(end).zfill(4) + str(clusterCount).zfill(2) + str(scanInterval) + str(nrScans).zfill(2) + '\n'
	# status = None

	# data, error = _sendCommand(lidar, command)

	# if error != 'noError':
	# 	return error

	# return data, status
	# pass

def startLaser(lidar):
	command = 'BM\n'
	status = None

	data, error = _sendCommand(lidar, command)

	if error != 'noError':
		return error

	if data[0] == '00':
		status = 'laserOn'
	elif data[0] == '01':
		status = 'noControl_LaserMalfunction'
	elif data[0] == '02':
		status = 'laserOn'
	
	return status

def stopLaser(lidar):
	command = 'QT\n'
	status = None

	data, error = _sendCommand(lidar, command)

	if error != 'noError':
		return error
	
	if data[0] == '00':
		status = 'laserOff'

	return status

def resetLidar(lidar):
	command = 'RS\n'
	status = None

	data, error = _sendCommand(lidar, command)

	if error != 'noError':
		return error

	if data[1] == '00':
		status = 'lidarReset'
	
	return status

# Not Implemented (no need)
# def adjustTime():
	# pass

# Not Implemented (no need)
# def changeBitRate():
	# pass

def getVersion(lidar):
	command = 'VV\n'
	status = None

	data, error = _sendCommand(lidar, command)

	if error != 'noError':
		return error

	ven = data[1][5:]
	prod = data[2][5:]
	firmVer = data[3][5:]
	protVer = data[4][5:]
	serial = data[5][5:]

	return status, ven, prod, firmVer, protVer, serial

def getSpecifications(lidar):
	command = 'PP\n'
	status = None

	data, error = _sendCommand(lidar, command)

	if error != 'noError':
		return error

	model = data[1][5:]
	minDist = data[2][5:]
	maxDist = data[3][5:]
	totalSteps = data[4][5:]
	firstStep = data[5][5:]
	lastStep = data[6][5:]
	stepOnFrontAxis = data[7][5:]
	motSpeed = data[8][5:]

	return status, model, minDist, maxDist, totalSteps, firstStep, lastStep, stepOnFrontAxis, motSpeed

def getRunningState(lidar):
	command = 'II\n'
	status = None

	data, error = _sendCommand(lidar, command)

	if error != 'noError':
		return error

	model = data[1][5:]
	lasState = data[2][5:]
	motSpeed = data[3][5:]
	measMode = data[4][5:]
	bitRate = data[5][5:]
	tStamp = data[6][5:]
	sensDiag = data[7][5:]

	return status, model, lasState, motSpeed, measMode, bitRate, tStamp, sensDiag




if __name__ == "__main__":
    startStep = 0
    endStep = 1080
    clusterCount = 1

    lidar = connect('COM10')
 

#	# Test output to show the sensor is connected and working
#    status, vendorInfo, productInfo, firmwareVersion, protocalVersion, sensorSerialNumber = getVersion(lidar)
#    print('{:>50}'.format('Sensor Version Informations'))
#    print('{:>40}{}'.format('Status\t',status))
#    print('{:>40}{}'.format('Vendor Info\t',vendorInfo))
#    print('{:>40}{}'.format('Product Info\t',productInfo))
#    print('{:>40}{}'.format('Firmware Version\t',firmwareVersion))
#    print('{:>40}{}'.format('Protocal Version\t',protocalVersion))
#    print('{:>40}{}'.format('Sensor Serial Number\t',sensorSerialNumber))
##
#    status, model, minDist, maxDist, totalSteps, firstStep, lastStep, stepOnFrontAxis, motSpeed = getSpecifications(lidar)
#    print('\n{:>50}'.format('Sensor Specifications'))
#    print('{:>40}{}'.format('Status\t',status))
#    print('{:>40}{}'.format('Sensor Model\t',model))
#    print('{:>40}{}'.format('Minimum Measurement [mm]\t',minDist))
#    print('{:>40}{}'.format('Maximum Measurement [mm]\t',maxDist))
#    print('{:>40}{}'.format('Total number of steps for 360 deg\t',totalSteps))
#    print('{:>40}{}'.format('First Step of the Measurement Range\t',firstStep))
#    print('{:>40}{}'.format('Last Step of the Measurement Range\t',lastStep))
#    print('{:>40}{}'.format('Step number on the sensors front axis\t',stepOnFrontAxis))
#    print('{:>40}{}'.format('Standard motor speed [rpm]\t',motSpeed))
#
#    status, model, laserIllumState, motSpeed, measuerMode, bitRate, timeStamp, sensorDiagnostic = getRunningState(lidar)
#    print('\n{:>50}'.format('Sensor Running State'))
#    print('{:>40}{}'.format('Status\t',status))
#    print('{:>40}{}'.format('Sensor Model\t',model))
#    print('{:>40}{}'.format('Laser illumination state\t',laserIllumState))
#    print('{:>40}{}'.format('Motor Speed\t',motSpeed))
#    print('{:>40}{}'.format('Measurement Mode\t',measuerMode))
#    print('{:>40}{}'.format('Bit Rate for RS232C\t',bitRate))
#    print('{:>40}{}'.format('Time Stamp\t',timeStamp))
#    print('{:>40}{}'.format('Sensor Diagnostic\t',sensorDiagnostic))
    
    startLaser(lidar)
    npdata, status = getLatestScanSteps(lidar, 0, 1080,1)
#    new_npdata = npdata[npdata[:,1]<5000,:]
    new_npdata = npdata[np.logical_and(npdata[:,1]>2000, npdata[:,1]<5000),:]
#    print(angleDistance)
    
#    print("-------")
#    
#    print(status)
    lidar.close()