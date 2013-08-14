(* Mathematica Package *)

(* Created by Phil Ngo, Wolfram Reasearch Inc., Jun 24, 2013 *)

BeginPackage["SpheroConnector`"]

(* Exported symbols *)
SpheroDeviceConnect::usage = "SpheroDeviceConnect[spheroPort,numRetries:7] connects to Sphero and returns a sphero connection object. Ex) spheroConnection = ConnectSphero[\"/dev/tty.Sphero-BWR-RN-SPP\"]. (find this address on unix by typying 'ls /dev/tty.*' at a terminal";
SpheroDeviceDisconnect::usage = "SpheroDeviceDisconnect[spheroConnection] disconnects Sphero at the given port";
SpheroDeviceData::usage = "SpheroDeviceData[spheroConnection] returns a current list of named data from Sphero.";
WritePacket::usage = "WritePacket[spheroconnection, packet] sends a packet to Sphero manually";
Checksum::usage = "Checksum[header, data] computes a checksum for a packet";
PacketByteString::usage = "PacketByteString[header, data] creates a fully formed bytestring packet that can be sent to Sphero]";
DisplayControls::usage = "DisplayControls[] displays a simple GUI for moving Sphero";
GetData::usage = "GetData[] returns a computable array with all of the data currently streaming from Sphero";
GetDataBuffer::usage = "GetDataBuffer[] returns a computable matrix with all of the data currently streaming from Sphero up the length of the buffer specified by SetDataBuffer[length]";
SetDataBufferLength::usage = "SetDataBuffer[length] sets the length of the data buffer";

(* Raw Commands *)
Ping::usage = "Ping[spheroConnection] pings the sphero to make sure the connection works. Success generates no errors.";
SetStabilization::usage = "SetStabilization[spheroConnection, True/False] turns Sphero's internal stabilization on and off";
Sleep::usage = "Sleep[spheroConnection, n] puts Sphero to sleep for n seconds";
SetHeading::usage = "SetHeading[spheroConnection,newHeading] sets sphero's heading to newHeading, always relative to current heading (degrees clockwise viewed from above). Use this for calibration.";
SetRotationRate::usage = "SetRotationRate[spheroConnection] sets sphero's rotation rate. This is not a command to change heading, but when Sphero changes heading, he does so at this rotation rate.";
SetDataStreaming::usage = "SetDataStreaming[spheroConnection_,maxSampleRateDivisor_, numFramesPerPacket_, mask1_, packetCount_, mask2] sets data streaming from sphero. The default maximum sample rate is 400Hz. packetCount of 0 allows infinite streaming. Mask1 and mask2 tell Sphero which data to stream, this is documented in Sphero API 1.46. If unsure, setting both masks to FFFFFFFFh turns on everything. ";
SetRGB::usage = "SetRGB[spheroConnection,RGBcolor] sets Sphero's color to the given RGB color.";
SetRoll::usage = "SetRoll[spheroConnection,speed,heading] sets Sphero's speed and heading. Speed must be a real number between 0 and 1 inclusive. Heading must be an Integer between 0 and 359 inclusive (clockwise degrees, with 90 = right, 270 = left). ";
Recalibrate::usage = "Recalibrate[spheroConnection] recalibrates such that the current heading becomes the zero heading";

(* Data Getters *)
GetAccelerometerAxisXRaw::usage = "GetAccelerometerAxisXRaw[] returns raw accelerometer data along the x-axis. Range: -2048 to 2047, units/LSB: 4mG, mask (Mask 1): 0x80000000";
GetAccelerometerAxisYRaw::usage = "GetAccelerometerAxisYRaw[] returns raw accelerometer data along the y-axis. Range: -2048 to 2047, units/LSB: 4mG, mask (Mask 1): 0x40000000";
GetAccelerometerAxisZRaw::usage = "GetAccelerometerAxisZRaw[] returns raw accelerometer data along the z-axis. Range: -2048 to 2047, units/LSB: 4mG, mask (Mask 1): 0x20000000";
GetGyroAxisXRaw::usage = "GetGyroAxisXRaw[] returns raw gyrometer data around the x-axis. Range: -32768 to 32767, units/LSB: 0.068deg, mask (Mask 1): 0x10000000";
GetGyroAxisYRaw::usage = "GetGyroAxisYRaw[] returns raw gyrometer data around the y-axis. Range: -32768 to 32767, units/LSB: 0.068deg, mask (Mask 1): 0x08000000";
GetGyroAxisZRaw::usage = "GetGyroAxisZRaw[] returns raw gyrometer data around the z-axis. Range: -32768 to 32767, units/LSB: 0.068deg, mask (Mask 1): 0x04000000";
GetRightMotorBackEMFRaw::usage = "GetRightMotorBackEMFRaw[] returns raw electromotive force at the right back motor. Range: -32768 to 32767, units/LSB: 22.5cm, mask (Mask 1): 0x00400000";
GetLeftMotorBackEMFRaw::usage = "GetLeftMotorBackEMFRaw[] returns raw electromotive force at the left back motor. Range: -32768 to 32767, units/LSB: 22.5cm, mask (Mask 1): 0x00200000";
GetLeftMotorPWMRaw::usage = "GetLeftMotorPWMRaw[] returns raw pulse width modulation at the left back motor. Range: -2048 to 2047, units/LSB: duty cycle, mask (Mask 1): 0x00100000";
GetRightMotorPWMRaw::usage = "GetRightMotorPWMRaw[] returns raw pulse width modulation at the right back motor.Range: -2048 to 2047, units/LSB: duty cycle, mask (Mask 1): 0x00080000";
GetIMUPitchAngleFiltered::usage = "GetIMUPitchAngleFiltered[] returns filtered IMU pitch angle (about x-axis). Range: -180 to 179, units/LSB: deg, mask (Mask 1): 0x00040000";
GetIMURollAngleFiltered::usage = "GetIMURollAngleFiltered[] returns filtered IMU roll angle (about y-axis). Range: -180 to 179, units/LSB: deg, mask (Mask 1): 0x00020000";
GetIMUYawAngleFiltered::usage = "GetIMUYawAngleFiltered[] returns filtered IMU yaw angle (about z-axis). Range: -180 to 179, units/LSB: deg, mask (Mask 1): 0x00010000";
GetAccelerometerAxisXFiltered::usage = "GetAccelerometerAxisXFiltered[] returns filtered accelerometer data along the x-axis. Range: -32768 to 32767, units/LSB: 1/4096 G, mask (Mask 1): 0x00008000";
GetAccelerometerAxisYFiltered::usage = "GetAccelerometerAxisYFiltered[] returns filtered accelerometer data along the y-axis. Range: -32768 to 32767, units/LSB: 1/4096 G, mask (Mask 1): 0x00004000";
GetAccelerometerAxisZFiltered::usage = "GetAccelerometerAxisZFiltered[] returns filtered accelerometer data along the z-axis. Range: -32768 to 32767, units/LSB: 1/4096 G, mask (Mask 1): 0x00002000";
GetGyroAxisXFiltered::usage = "GetGyroAxisXFiltered[] returns filtered gyrometer data around the x-axis. Range: -20000 to 20000, units/LSB: 0.1 dps, mask (Mask 1): 0x00001000";
GetGyroAxisYFiltered::usage = "GetGyroAxisYFiltered[] returns filtered gyrometer data around the y-axis. Range: -20000 to 20000, units/LSB: 0.1 dps, mask (Mask 1): 0x00000800";
GetGyroAxisZFiltered::usage = "GetGyroAxisZFiltered[] returns filtered gyrometer data around the z-axis. Range: -20000 to 20000, units/LSB: 0.1 dps, mask (Mask 1): 0x00000400";
GetRightMotorBackEMFFiltered::usage = "GetRightMotorBackEMFRaw[] returns filtered electromotive force at the right back motor. Range: -32768 to 32767, units/LSB: 22.5cm, mask (Mask 1): 0x00000040";
GetLeftMotorBackEMFFiltered::usage = "GetRightMotorBackEMFRaw[] returns filtered electromotive force at the right back motor. Range: -32768 to 32767, units/LSB: 22.5cm, mask (Mask 1): 0x00000020";

(* Mask 2*)
GetQuaternionQ0::usage = "GetQuaternionQ0[] returns Q0 data. Range: -10000 to 10000, units/LSB: 1/10000 Q, mask (Mask 2): 0x80000000";
GetQuaternionQ1::usage = "GetQuaternionQ1[] returns Q1 data. Range: -10000 to 10000, units/LSB: 1/10000 Q, mask (Mask 2): 0x40000000";
GetQuaternionQ2::usage = "GetQuaternionQ2[] returns Q2 data. Range: -10000 to 10000, units/LSB: 1/10000 Q, mask (Mask 2): 0x20000000";
GetQuaternionQ3::usage = "GetQuaternionQ3[] returns Q3 data. Range: -10000 to 10000, units/LSB: 1/10000 Q, mask (Mask 2): 0x10000000";
GetOdometerX::usage = "GetOdometerX[] returns odometer data along x-axis if supported by the device. Range: -32768 to 32767, units/LSB: cm, mask (Mask 2): 0x08000000";
GetOdometerY::usage = "GetOdometerY[] returns odometer data along y-axis if supported by the device. Range: -32768 to 32767, units/LSB: cm, mask (Mask 2): 0x04000000";
GetAccelOne::usage = "GetAccelOne[] returns accel one data if supported by the device. Range: 0 to 8000, units/LSB: 1 mG, mask (Mask 2): 0x02000020";
GetVelocityX::usage = "GetVelocityX[] returns velocity along x-axis if supported by the device. Range: -32768 to 32767, units/LSB: mm/s, mask (Mask 2): 0x01000000";
GetVelocityY::usage = "GetVelocityY[] returns velocity along y-axis if supported by the device. Range: -32768 to 32767, units/LSB: mm/s, mask (Mask 2): 0x00800000";

(* Data Buffer Getters *)
GetAccelerometerAxisXRawBuffer::usage = "GetAccelerometerAxisXRaw[] returns buffered raw accelerometer data along the x-axis. Range: -2048 to 2047, units/LSB: 4mG";
GetAccelerometerAxisYRawBuffer::usage = "GetAccelerometerAxisYRaw[] returns buffered raw accelerometer data along the y-axis. Range: -2048 to 2047, units/LSB: 4mG";
GetAccelerometerAxisZRawBuffer::usage = "GetAccelerometerAxisZRaw[] returns buffered raw accelerometer data along the z-axis. Range: -2048 to 2047, units/LSB: 4mG";
GetGyroAxisXRawBuffer::usage = "GetGyroAxisXRaw[] returns buffered raw gyrometer data around the x-axis. Range: -32768 to 32767, units/LSB: 0.068deg";
GetGyroAxisYRawBuffer::usage = "GetGyroAxisYRaw[] returns buffered raw gyrometer data around the y-axis. Range: -32768 to 32767, units/LSB: 0.068deg";
GetGyroAxisZRawBuffer::usage = "GetGyroAxisZRaw[] returns buffered raw gyrometer data around the z-axis. Range: -32768 to 32767, units/LSB: 0.068deg";
GetRightMotorBackEMFRawBuffer::usage = "GetRightMotorBackEMFRaw[] returns buffered raw electromotive force at the right back motor. Range: -32768 to 32767, units/LSB: 22.5cm";
GetLeftMotorBackEMFRawBuffer::usage = "GetLefttMotorBackEMFRaw[] returns buffered raw electromotive force at the left back motor. Range: -32768 to 32767, units/LSB: 22.5cm";
GetLeftMotorPWMRawBuffer::usage = "GetLeftMotorPWMRaw[] returns buffered raw pulse width modulation at the left back motor. Range: -2048 to 2047, units/LSB: duty cycle";
GetRightMotorPWMRawBuffer::usage = "GetLeftMotorPWMRaw[] returns buffered raw pulse width modulation at the right back motor. Range: -2048 to 2047, units/LSB: duty cycle";
GetIMUPitchAngleFilteredBuffer::usage = "GetIMUPitchAngleFiltered[] returns filtered IMU pitch angle (about x-axis). Range: -180 to 179, units/LSB: deg";
GetIMURollAngleFilteredBuffer::usage = "GetIMURollAngleFiltered[] returns filtered IMU roll angle (about y-axis). Range: -180 to 179, units/LSB: deg";
GetIMUYawAngleFilteredBuffer::usage = "GetIMUYawAngleFiltered[] returns filtered IMU yaw angle (about z-axis). Range: -180 to 179, units/LSB: deg";
GetAccelerometerAxisXFilteredBuffer::usage = "GetAccelerometerAxisXFiltered[] returns buffered and filtered accelerometer data along the x-axis. Range: -32768 to 32767, units/LSB: 1/4096 G";
GetAccelerometerAxisYFilteredBuffer::usage = "GetAccelerometerAxisYFiltered[] returns buffered and filtered accelerometer data along the y-axis. Range: -32768 to 32767, units/LSB: 1/4096 G";
GetAccelerometerAxisZFilteredBuffer::usage = "GetAccelerometerAxisZFiltered[] returns buffered and filtered accelerometer data along the z-axis. Range: -32768 to 32767, units/LSB: 1/4096 G";
GetGyroAxisXFilteredBuffer::usage = "GetGyroAxisXFiltered[] returns buffered and filtered gyrometer data around the x-axis. Range: -20000 to 20000, units/LSB: 0.1 dps";
GetGyroAxisYFilteredBuffer::usage = "GetGyroAxisYFiltered[] returns buffered and filtered gyrometer data around the y-axis. Range: -20000 to 20000, units/LSB: 0.1 dps";
GetGyroAxisZFilteredBuffer::usage = "GetGyroAxisZFiltered[] returns buffered and filtered gyrometer data around the z-axis. Range: -20000 to 20000, units/LSB: 0.1 dps";
GetRightMotorBackEMFFilteredBuffer::usage = "GetRightMotorBackEMFRaw[] returns buffered and filtered electromotive force at the right back motor. Range: -32768 to 32767, units/LSB: 22.5cm";
GetLeftMotorBackEMFFilteredBuffer::usage = "GetRightMotorBackEMFRaw[] returns buffered and filtered electromotive force at the leftt back motor. Range: -32768 to 32767, units/LSB: 22.5cm";

(* Mask 2*)
GetQuaternionQ0Buffer::usage = "GetQuaternionQ0[] returns buffered Q0 data. Range: -10000 to 10000, units/LSB: 1/10000 Q";
GetQuaternionQ1Buffer::usage = "GetQuaternionQ1[] returns buffered Q1 data. Range: -10000 to 10000, units/LSB: 1/10000 Q";
GetQuaternionQ2Buffer::usage = "GetQuaternionQ2[] returns buffered Q2 data. Range: -10000 to 10000, units/LSB: 1/10000 Q";
GetQuaternionQ3Buffer::usage = "GetQuaternionQ3[] returns buffered Q3 data. Range: -10000 to 10000, units/LSB: 1/10000 Q";
GetOdometerXBuffer::usage = "GetOdometerX[] returns buffered odometer data along x-axis if supported by the device. Range: -32768 to 32767, units/LSB: cm";
GetOdometerYBuffer::usage = "GetOdometerY[] returns buffered odometer data along y-axis if supported by the device. Range: -32768 to 32767, units/LSB: cm";
GetAccelOneBuffer::usage = "GetAccelOne[] returns buffered accel one data if supported by the device. Range: 0 to 8000, units/LSB: 1 mG";
GetVelocityXBuffer::usage = "GetVelocityX[] returns buffered velocity along x-axis if supported by the device. Range: -32768 to 32767, units/LSB: mm/s";
GetVelocityYBuffer::usage = "GetVelocityY[] returns buffered velocity along y-axis if supported by the device. Range: -32768 to 32767, units/LSB: mm/s";
(* Messages *)
WritePacket::badresp = "Could not write packet or read packet response";

Begin["`Private`"]

(* Implementation of the package *)


AppendTo[$Path,FileNameTake[$InputFileName,{1,-3}]];
<<SerialIO`

(* globals for buffering SerialIO stream *)
rawBuffer = {};
packetBuffer = {};

(* Data buffer variables *)
dataStreaming = False;
allData = Table[0,{64}];
bufferLength = 50;
dataBuffer = Table[allData,{bufferLength}];

(* Robot control variables *)
currentHeading = 0;

(* Constants *)
CODEOK = 0;
SOP1 = FromDigits["FF", 16]; (*Start of Packet byte 1*)
SOP2Sync = FromDigits["FF", 16]; (*Start of Packet byte 2 signifying Synchronous*)
SOP2Async = FromDigits["FE", 16]; (*Start of Packet byte 2 signifying Asynchronous*)
DIDCore = FromDigits["00", 16];  (*(Virtual) Device ID*)
DIDBootloader = FromDigits["01", 16];  (*(Virtual) Device ID*)
DIDSphero = FromDigits["02", 16];  (*(Virtual) Device ID*)
seq = FromDigits["00", 16];  (*Sequence number*)

pingcid = FromDigits["01", 16];
getVersioncid = FromDigits["02", 16];
setDeviceNamecid = FromDigits["10", 16];
getBluetoothInfocid = FromDigits["11", 16];
getAutoReconnectcid = FromDigits["12", 16];
setAutoReconnectcid = FromDigits["13", 16];
getPowerStatecid = FromDigits["20", 16];
setPowerNotificationcid = FromDigits["21", 16];
sleepcid = FromDigits["22", 16];
getVoltageTripPointscid = FromDigits["23", 16];
setVoltageTripPointscid = FromDigits["24", 16];
setInactivityTimeoutcid = FromDigits["25", 16];
jumpToBootloadercid = FromDigits["30", 16];
performLevel1Diagnosticscid = FromDigits["40", 16];
performLevel2Diagnosticscid = FromDigits["41", 16];
clearCounterscid = FromDigits["42", 16];
setTimeValuecid = FromDigits["50", 16];
pollPacketTimescid = FromDigits["51", 16];

setHeadingcid = FromDigits["01", 16];
setStabilizationcid = FromDigits["02", 16];
setRotationRatecid = FromDigits["03", 16];
setApplicationConfigurationBlockcid = FromDigits["04", 16];
getApplicationConfigurationBlockcid = FromDigits["05", 16];
reenableDemoModecid = FromDigits["06", 16];
getChassisIdcid = FromDigits["07", 16];
setChassisIdcid = FromDigits["08", 16];
selfLevelcid = FromDigits["09", 16];
setVDLcid = FromDigits["0A", 16];
setDataStreamingcid = FromDigits["11", 16];
configureCollisionDetectioncid = FromDigits["12", 16];
locatorcid = FromDigits["13", 16];
setAccelerometercid = FromDigits["14", 16];
readLocatorcid = FromDigits["15", 16];
setRGBcid = FromDigits["20", 16];
setBackLEDOutputcid = FromDigits["21", 16];
getRGBcid = FromDigits["22", 16];
rollcid = FromDigits["30", 16];
setBoostWithTimecid = FromDigits["31", 16];
setRawMotorValuescid = FromDigits["33", 16];
setMotionTimeoutcid = FromDigits["34", 16];
setOptionFlagscid = FromDigits["35", 16];
getOptionFlagscid = FromDigits["36", 16];
getConfigurationBlockcid = FromDigits["40", 16];
getDeviceModecid = FromDigits["42", 16];
runMacrocid = FromDigits["50", 16];
saveTemporaryMacrocid = FromDigits["51", 16];
reinitMacrocid = FromDigits["54", 16];
abortMacrocid = FromDigits["55", 16];
getMacroStatuscid = FromDigits["56", 16];
setMacroParametercid = FromDigits["57", 16];
appendMacroChunkcid = FromDigits["58", 16];
eraseOrbbasicStoragecid =FromDigits["60", 16];
appendOrbbasicFragmentcid = FromDigits["61", 16];
runOrbbasicProgramcid = FromDigits["62", 16];
abortOrbbasicProgramcid = FromDigits["63", 16];
answerInputcid = FromDigits["64", 16];

(* Streaming Data Masks - use these to select the types of data you want*)
dataStreamingOffMask = FromDigits["00000000", 16];

(* Mask 1 *)
currentMask1 = 0;
accelerometerAxisXRawMask = FromDigits["80000000", 16];
accelerometerAxisYRawMask = FromDigits["40000000", 16];
accelerometerAxisZRawMask = FromDigits["20000000", 16];
gyroAxisXRawMask = FromDigits["10000000", 16];
gyroAxisYRawMask = FromDigits["08000000", 16];
gyroAxisZRawMask = FromDigits["04000000", 16];
rightMotorBackEMFRawMask = FromDigits["00400000", 16];
leftMotorBackEMFRawMask = FromDigits["00200000", 16];
leftMotorPWMRawMask = FromDigits["00100000", 16];
rightMotorPWMRawMask = FromDigits["00080000", 16];
IMUPitchAngleFilteredMask = FromDigits["00040000", 16];
IMURollAngleFilteredMask = FromDigits["00020000", 16];
IMUYawAngleFilteredMask = FromDigits["00010000", 16];
accelerometerAxisXFilteredMask = FromDigits["00008000", 16];
accelerometerAxisYFilteredMask = FromDigits["00004000", 16];
accelerometerAxisZFilteredMask = FromDigits["00002000", 16];
gyroAxisXFilteredMask = FromDigits["00001000", 16];
gyroAxisYFilteredMask = FromDigits["00000800", 16];
gyroAxisZFilteredMask = FromDigits["00000400", 16];
rightMotorBackEMFFilteredMask = FromDigits["00000040", 16];
leftMotorBackEMFFilteredMask = FromDigits["00000020", 16];
allDataRawMask = FromDigits["FC780000", 16];
allDataFilteredMask = FromDigits["0007FC60", 16];
allMask1Mask = FromDigits["FC7FFC60", 16];

(* Mask 2*)
currentMask2 = 0;
quaternionQ0Mask = FromDigits["80000000", 16];
quaternionQ1Mask = FromDigits["40000000", 16];
quaternionQ2Mask = FromDigits["20000000", 16];
quaternionQ3Mask = FromDigits["10000000", 16];
odometerXMask = FromDigits["08000000", 16];
odometerYMask = FromDigits["04000000", 16];
accelOneMask = FromDigits["02000000", 16];
velocityXMask = FromDigits["01000000", 16];
velocityYMask = FromDigits["00800000", 16];
allMask2Mask = FromDigits["FF800000", 16];


SpheroDeviceConnect[spheroPort_,numRetries_:7] :=
	Module[{spheroConnection,i},
		For[i = 0, i < numRetries, i++,
			spheroConnection = SerialOpen[spheroPort];
			If[Head[spheroConnection] == SerialPort, 
				SerialSetOptions[spheroConnection, "BaudRate" -> 115200];
				currentHeading = 0;
				rawBuffer = {};
				packetBuffer = {};
				bufferLength = 50;
				allData = Table[0,{64}];
				dataBuffer = Table[allData,{bufferLength}];
				RunScheduledTask[
					UpdateRawBuffer[spheroConnection];
					UpdatePacketBuffer[];
					ProcessPackets[];,
					.01
				];
				Return@spheroConnection;
			];
			PrintTemporary["Retrying..."];
		];
		Return@$Failed;
	]
	
SpheroDeviceDisconnect[spheroConnection_] :=
	Module[{}, 
		SerialClose[spheroConnection];
	]

SpheroDeviceData[spheroConnection_] :=
	Module[{},
		If[!dataStreaming,
			SetDataStreaming[spheroConnection, 10, 1, FromDigits["FFFFFFFF", 16], 0, 
 			FromDigits["FFFFFFFF", 16]]
		];
		Return[{
			"AccelerometerAxisXRaw"->GetAccelerometerAxisXRaw[],
			"AccelerometerAxisXRaw"->GetAccelerometerAxisYRaw[],
			"AccelerometerAxisXRaw"->GetAccelerometerAxisZRaw[],
			"GyroAxisXRaw"->GetGyroAxisXRaw[],
			"GyroAxisYRaw"->GetGyroAxisYRaw[],
			"GyroAxisZRaw"->GetGyroAxisZRaw[],
			"RightMotorBackEMFRaw"->GetRightMotorBackEMFRaw[],
			"LeftMotorBackEMFRaw"->GetLeftMotorBackEMFRaw[],
			"RightMotorPWMRaw"->GetRightMotorPWMRaw[],
			"LeftMotorPWMRaw"->GetLeftMotorPWMRaw[],
			"IMUPitchAngleFiltered"->GetIMUPitchAngleFiltered[],
			"IMURollAngleFiltered"->GetIMURollAngleFiltered[],
			"IMUYawAngleFiltered"->GetIMUYawAngleFiltered[],
			"AccelerometerAxisXFiltered"->GetAccelerometerAxisXFiltered[],
			"AccelerometerAxisYFiltered"->GetAccelerometerAxisYFiltered[],
			"AccelerometerAxisZFiltered"->GetAccelerometerAxisZFiltered[],
			"GyroAxisXFiltered"->GetGyroAxisXFiltered[],
			"GyroAxisYFiltered"->GetGyroAxisYFiltered[],
			"GyroAxisZFiltered"->GetGyroAxisZFiltered[],
			"GetRightMotorBackEMFFiltered"->GetRightMotorBackEMFFiltered[],
			"GetLeftMotorBackEMFFiltered"->GetLeftMotorBackEMFFiltered[],
			"QuaternionQ0"->GetQuaternionQ0[],
			"QuaternionQ1"->GetQuaternionQ1[],
			"QuaternionQ2"->GetQuaternionQ2[],
			"QuaternionQ3"->GetQuaternionQ3[],
			"OdometerX"->GetOdometerX[],
			"OdometerY"->GetOdometerY[],
			"AccelOne"->GetAccelOne[],
			"VelocityX"->GetVelocityX[],
			"VelocityY"->GetVelocityY[]
		}];
	]

(*The modulo 256 sum of all the bytes from the DID through the end of the data payload, bit inverted (1's complement)*)
Checksum[header_, data_] :=
 	Module[{}, 
  		Return@BitXor[Mod[Total[Join[Drop[header,2], data]], 256],255];
 	]
 
CorrectChecksumQ[header_, data_, checksum_] :=
 	Module[{}, 
  		Return@( checksum == Checksum[header, data]);
 	]

PacketByteString[header_, data_] := 
	Module[{}, 
		Return@FromCharacterCode[Join[header, data, {Checksum[header, data]}]];
	]

WritePacket[spheroConnection_, requestPacket_] := 
	Module[{}, 
		SerialWrite[spheroConnection, requestPacket];
		seq = Mod[seq+1, 256];
	]

UpdateRawBuffer[spheroConnection_] :=
	Module[{},
		rawBuffer = Join[rawBuffer,ToCharacterCode[SerialRead[spheroConnection,.01]]];
	]

UpdatePacketBuffer[] :=
	Module[{length,checksum,data,header,packetCorrupt},
		packetCorrupt = False;
		While[Length[rawBuffer] > 5 && !packetCorrupt,
			packetCorrupt = True;
			For[i = 1, i <= Length[rawBuffer]-5, i++,
				header = rawBuffer[[i;;i+4]];
				length = rawBuffer[[i+4]];
				If[Length[rawBuffer] >= i + 4 + length,
					If[length > 1, data = rawBuffer[[i + 5;;i + 3 + length]], data = {}];
					checksum = rawBuffer[[i + 4 + length]];
					If[CorrectChecksumQ[header, data, checksum] && header[[1]] == 255,
						packetBuffer = Append[packetBuffer,PacketByteString[header, data]];
						rawBuffer = Drop[rawBuffer, i + 4 + length];
						packetCorrupt = False;
					]
				]
			];
		];
		(* Having extracted all that we could, avoid infinite loops by scrapping any leftover shards *)
		rawBuffer = {};
	]

ProcessPackets[] :=
	Module[{},
		While[Length[packetBuffer]>0,
			packet = Flatten[ToCharacterCode[Take[packetBuffer,1]]];
			packetBuffer = Drop[packetBuffer,1];
			Switch[Take[packet,3],
				{255,254,3},
					ReceiveAsynchronousDataPacket[packet];,
				{255,255,CODEOK},
					ReceiveSynchronousResponsePacket[packet];
			]
			
		]
	]

ReceiveAsynchronousDataPacket[packet_] :=
	Module[{length,data,processedData,mask1Bits,mask2Bits,individualMasks,rules},
		length = packet[[5]];
		data = packet[[6;; 4 + length]];
		processedData = Map[Uint16toint16[FromDigits[#, 2^8]] &, Partition[data, 2]];
		mask1Bits = IntegerDigits[currentMask1, 2, 32];
		mask2Bits = IntegerDigits[currentMask2, 2, 32];
		(*Print[processedData];*)
		If[(length - 1)/2 == Total[Join[mask1Bits,mask2Bits]],
			individualMasks = Join[Position[mask1Bits,1],Position[mask2Bits,1]];
			rules = MapIndexed[#1 -> processedData[[#2]][[1]] &, individualMasks];
			allData = ReplacePart[allData,rules];
			(*Print[allData];*)
			UpdateDataBuffer[];,
			(*else*)
			Return@$Failed;
		]
	]

ReceiveSynchronousResponsePacket[packet_] :=
	Module[{},
		(* TODO: not yet implemented *)
		Return@Null;
	]
	

InspectResponse[responsePacket_] :=
	Module[{packet,header,body,data,checksum},
		packet = ToCharacterCode[responsePacket];
		header = Take[packet,5];
		body = Drop[packet,5];
		checksum = Take[body,-1][[1]];
		data = Drop[body,-1];
		Print["Response:"];
		Print[StringJoin["SOP1 =     ", ToString[header[[1]]]]];
		Print[StringJoin["SOP2 =     ", ToString[header[[2]]], Switch[header[[2]],FromDigits["FF",16]," (Synchronous)",FromDigits["FE",16]," (Asynchronous)", _, " (??)"]]];
		Switch[header[[2]],
			FromDigits["FF",16],
				Print[StringJoin["MRSP =     ", ToString[header[[3]] ], Switch[header[[3]],FromDigits["00",16]," (CODE_OK)", _, " (??)"]]];
				Print[StringJoin["SEQ =      ", ToString[header[[4]] ]," (Echo)"]];
				Print[StringJoin["DLEN =     ", ToString[header[[5]] ]]];,
			FromDigits["FE",16],
				Print[StringJoin["ID CODE =  ", ToString[header[[3]] ]]];
				Print[StringJoin["DLEN = ", ToString[header[[4]] * 256 + header[[5]] ]]]; (*MSB * 256 + LSB*)];
		
		Print[StringJoin["Data = ", ToString[data]]];
	]

InspectRequest[requestPacket_] :=
	Module[{packet,header,body,data,checksum},
		packet = ToCharacterCode[requestPacket];
		header = Take[packet,6];
		body = Drop[packet,6];
		checksum = Take[body,-1][[1]];
		data = Drop[body,-1];
		Print["Request:"];
		Print[StringJoin["SOP1 = ", ToString[header[[1]] ]]];
		Print[StringJoin["SOP2 = ", ToString[header[[2]] ]]];
		Print[StringJoin["DID =  ", ToString[header[[3]] ]]];
		Print[StringJoin["CID =  ", ToString[header[[4]] ]]];
		Print[StringJoin["SEQ =  ", ToString[header[[5]] ]]];
		Print[StringJoin["DLEN = ", ToString[header[[6]] ]]];
		Print[StringJoin["Data = ", ToString[data]]];
	]

Ping[spheroConnection_] :=
	Module[{packet},
		packet = PacketByteString[{SOP1, SOP2Sync, DIDCore, pingcid, seq, 01}, {}];
		WritePacket[spheroConnection, packet];
	]

GetVersion[spheroConnection_] :=
	Module[{packet},
		packet = PacketByteString[{SOP1, SOP2Sync, DIDCore, getVersioncid, seq, 01}, {}];
		WritePacket[spheroConnection, packet];
	]

GetBluetoothInfo[spheroConnection_] :=
	Module[{packet},
		packet = PacketByteString[{SOP1, SOP2Sync, DIDCore, getBluetoothInfocid, seq, 01}, {}];
		WritePacket[spheroConnection, packet];
	]

GetPowerState[spheroConnection_] :=
	Module[{packet},
		packet = PacketByteString[{SOP1, SOP2Sync, DIDCore, getPowerStatecid, seq, 01}, {}];
		WritePacket[spheroConnection, packet];
	]

Sleep[spheroConnection_] :=
	Module[{packet},
		packet = PacketByteString[{SOP1, SOP2Sync, DIDCore, sleepcid, seq, 06}, {0,0,0,0,0}];
		WritePacket[spheroConnection, packet];
	]

SetInactivityTimeout[spheroConnection_,seconds_] :=
	Module[{packet,secondsMSB,secondsLSB},
		secondsMSB = Mod[Floor[seconds/256],256];
		secondsLSB = Mod[Floor[seconds],256];
		packet = PacketByteString[{SOP1, SOP2Sync, DIDCore, setInactivityTimeoutcid, seq, 03}, {secondsMSB,secondsLSB}];
		WritePacket[spheroConnection, packet];
	]

PerformLevel1Diagnostics[spheroConnection_] :=
	Module[{packet},
		packet = PacketByteString[{SOP1, SOP2Sync, DIDCore, performLevel1Diagnosticscid, seq, 01}, {}];
		WritePacket[spheroConnection, packet];
	]
	
PerformLevel2Diagnostics[spheroConnection_] :=
	Module[{packet},
		packet = PacketByteString[{SOP1, SOP2Sync, DIDCore, performLevel2Diagnosticscid, seq, 01}, {}];
		WritePacket[spheroConnection, packet];
	]

SetHeading[spheroConnection_, heading_] :=
	Module[{packet,hmsb,hlsb},
		hmsb = Mod[Floor[heading/256],256];
		hlsb = Mod[Floor[heading],256];
		currentHeading = Floor[heading];
		packet = PacketByteString[{SOP1, SOP2Sync, DIDSphero, setHeadingcid, seq, 03}, {hmsb,hlsb}];
		WritePacket[spheroConnection, packet];
	]

SetStabilization[spheroConnection_, state_/;(state || !state)] :=
	Module[{packet},
		packet = PacketByteString[{SOP1, SOP2Sync, DIDSphero, setStabilizationcid, seq, 02}, {Boole[state]}];
		WritePacket[spheroConnection, packet];
	] 

SetRotationRate[spheroConnection_, rotationRate_] :=
	Module[{packet},
		packet = PacketByteString[{SOP1, SOP2Sync, DIDSphero, setRotationRatecid, seq, 02}, {Floor[rotationRate]}];
		WritePacket[spheroConnection, packet];
	]

SetDataStreaming[spheroConnection_,maxSampleRateDivisor_, numFramesPerPacket_, mask1_, packetCount_, mask2_] :=
	Module[{divBytes,frameBytes,processedMask1,processedMask2,mask1Bytes,mask2Bytes,packet},
		divBytes = GetBytesBigEndian[maxSampleRateDivisor,2];
		frameBytes = GetBytesBigEndian[numFramesPerPacket,2];
		processedMask1 = BitAnd[mask1,allMask1Mask];
		processedMask2 = BitAnd[mask2,allMask2Mask];
		mask1Bytes = GetBytesBigEndian[processedMask1,4];
		mask2Bytes = GetBytesBigEndian[processedMask2,4];
		currentMask1 = processedMask1;
		currentMask2 = processedMask2;
		packet = PacketByteString[{SOP1, SOP2Sync, DIDSphero, setDataStreamingcid, seq, 14},
			   		{divBytes[[1]],divBytes[[2]],frameBytes[[1]],frameBytes[[2]],
			 		mask1Bytes[[1]],mask1Bytes[[2]],mask1Bytes[[3]],mask1Bytes[[4]],packetCount,
			 		mask2Bytes[[1]],mask2Bytes[[2]],mask2Bytes[[3]],mask2Bytes[[4]]}];
		WritePacket[spheroConnection, packet];
		dataStreaming = True;
	]

SetRGB[spheroConnection_,color_] :=
	Module[{packet},
		packet = PacketByteString[{SOP1, SOP2Async, DIDSphero, setRGBcid, seq, 04}, {Floor[color[[1]]*255],Floor[color[[2]]*255],Floor[color[[3]]*255]}];
		WritePacket[spheroConnection, packet];
	]

SetRoll[spheroConnection_,speed_,heading_] :=
	Module[{packet,hmsb,hlsb,state},
		hmsb = Mod[Floor[heading/256],256];
		hlsb = Mod[Floor[heading],256];
		state = 1;
		currentHeading = Floor[heading];
		packet = PacketByteString[{SOP1, SOP2Async, DIDSphero, rollcid, seq, 05}, {Floor[255 * speed], hmsb, hlsb, state}];
		WritePacket[spheroConnection, packet];
	]

Recalibrate[spheroConnection_] :=
	Module[{},
		SetHeading[spheroConnection,currentHeading];
	]

ResponseOKQ[responsePacket_] := 
	Module[{packet,header,body,data,checksum}, 
		packet = ToCharacterCode[responsePacket];
		header = Take[packet,5];
		checksum = Take[body,-1][[1]];
		data = Drop[body,-1];
		Return[Checksum[header,data] == checksum && header[[3]] == CODEOK];
	]

DisplayControls[spheroConnection_] := 
	Module[{color,pos}, 
	  	Manipulate[
	   		Block[{	heading = If[VectorAngle[{-1, 0}, pos] < Pi/2, 
	       						360 - VectorAngle[{0, 1}, pos]*180/Pi, 
	       						180/Pi*VectorAngle[{0, 1}, pos]],
	       			speed = Clip[Norm[pos]]},
	       			SetRGB[spheroConnection, color];
	       			SetRoll[spheroConnection, speed, Floor[heading]];
	       			Graphics[
                        {color, Arrow[{{0, 0}, pos}]},
	       				PlotRange -> {{-1.1, 1.1}, {-1.1, 1.1}}
	  				] 
	       	], 
	       	Row[{Button["Recalibrate",Recalibrate[spheroConnection];pos = {0.0,0.1}]}],
	  		{{pos, {0,0.1}}, {-1, -1}, {1, 1}, 
                ControlType -> Locator}, {{color,Green,"Color"}, Green}
		]
	]

(* Helper Methods *)
GetAnswerBit[packet_] :=
	Module[{packetCharCode},
		packetCharCode = ToCharacterCode[packet];
		Return@(packetCharCode[[2]] - 254);
	]

GetBytesBigEndian[digits_, n_] := 
	Module[{},
		Map[#[[1]]*16 + #[[2]] &, 
		Partition[IntegerDigits[digits, 16, 2*n], 2]]
	]

Uint16toint16[uint_] := 
	Module[{},
		Return@(BitAnd[2^15 - 1, uint] - BitAnd[2^15, uint]);
	]

(* Data Getters *)

GetData[] :=
	Module[{},
		Return@allData;
	]
	
GetAccelerometerAxisXRaw[] := 
	Module[{},
		Return@allData[[1]];
	]

GetAccelerometerAxisYRaw[] := 
	Module[{},
		Return@allData[[2]];
	]

GetAccelerometerAxisZRaw[] := 
	Module[{},
		Return@allData[[3]];
	]

GetGyroAxisXRaw[] :=
	Module[{},
		Return@allData[[4]];
	]

GetGyroAxisYRaw[] :=
	Module[{},
		Return@allData[[5]];
	]

GetGyroAxisZRaw[] :=
	Module[{},
		Return@allData[[6]];
	]

GetRightMotorBackEMFRaw[] :=
	Module[{},
		Return@allData[[10]];
	]
	
GetLeftMotorBackEMFRaw[] :=
	Module[{},
		Return@allData[[11]];
	]
	
GetLeftMotorPWMRaw[] :=
	Module[{},
		Return@allData[[12]];
	]
	
GetRightMotorPWMRaw[] :=
	Module[{},
		Return@allData[[13]];
	]
	
GetIMUPitchAngleFiltered[] :=
	Module[{},
		Return@allData[[14]];
	]
	
GetIMURollAngleFiltered[] :=
	Module[{},
		Return@allData[[15]];
	]
	
GetIMUYawAngleFiltered[] :=
	Module[{},
		Return@allData[[16]];
	]
	
GetAccelerometerAxisXFiltered[] :=
	Module[{},
		Return@allData[[17]];
	]
	
GetAccelerometerAxisYFiltered[] :=
	Module[{},
		Return@allData[[18]];
	]
	
GetAccelerometerAxisZFiltered[] :=
	Module[{},
		Return@allData[[19]];
	]
	
GetGyroAxisXFiltered[] :=
	Module[{},
		Return@allData[[20]];
	]
	
GetGyroAxisYFiltered[] :=
	Module[{},
		Return@allData[[21]];
	]
	
GetGyroAxisZFiltered[] :=
	Module[{},
		Return@allData[[22]];
	]
GetRightMotorBackEMFFiltered[] :=
	Module[{},
		Return@allData[[26]];
	]
	
GetLeftMotorBackEMFFiltered[] :=
	Module[{},
		Return@allData[[27]];
	]

(* Mask 2*)
GetQuaternionQ0[] :=
	Module[{},
		Return@allData[[33]];
	]
	
GetQuaternionQ1[] :=
	Module[{},
		Return@allData[[34]];
	]
	
GetQuaternionQ2[] :=
	Module[{},
		Return@allData[[35]];
	]
	
GetQuaternionQ3[] :=
	Module[{},
		Return@allData[[36]];
	]
	
GetOdometerX[] :=
	Module[{},
		Return@allData[[37]];
	]
	
GetOdometerY[] :=
	Module[{},
		Return@allData[[38]];
	]
	
GetAccelOne[] :=
	Module[{},
		Return@allData[[39]];
	]
	
GetVelocityX[] :=
	Module[{},
		Return@allData[[40]];
	]
	
GetVelocityY[] :=
	Module[{},
		Return@allData[[41]];
	]

GetDataBuffer[] :=
	Module[{},
		Return@dataBuffer;
	]

GetAccelerometerAxisXRawBuffer[] := 
	Module[{},
		Return@dataBuffer[[;;, 1]];
	]

GetAccelerometerAxisYRawBuffer[] := 
	Module[{},
		Return@dataBuffer[[;;, 2]];
	]

GetAccelerometerAxisZRawBuffer[] := 
	Module[{},
		Return@dataBuffer[[;;, 3]];
	]

GetGyroAxisXRawBuffer[] :=
	Module[{},
		Return@dataBuffer[[;;, 4]];
	]

GetGyroAxisYRawBuffer[] :=
	Module[{},
		Return@dataBuffer[[;;, 5]];
	]

GetGyroAxisZRawBuffer[] :=
	Module[{},
		Return@dataBuffer[[;;, 6]];
	]

GetRightMotorBackEMFRawBuffer[] :=
	Module[{},
		Return@dataBuffer[[;;, 10]];
	]
	
GetLeftMotorBackEMFRawBuffer[] :=
	Module[{},
		Return@dataBuffer[[;;, 11]];
	]
	
GetLeftMotorPWMRawBuffer[] :=
	Module[{},
		Return@dataBuffer[[;;, 12]];
	]
	
GetRightMotorPWMRawBuffer[] :=
	Module[{},
		Return@dataBuffer[[;;, 13]];
	]
	
GetIMUPitchAngleFilteredBuffer[] :=
	Module[{},
		Return@dataBuffer[[;;, 14]];
	]
	
GetIMURollAngleFilteredBuffer[] :=
	Module[{},
		Return@dataBuffer[[;;, 15]];
	]
	
GetIMUYawAngleFilteredBuffer[] :=
	Module[{},
		Return@dataBuffer[[;;, 16]];
	]
	
GetAccelerometerAxisXFilteredBuffer[] :=
	Module[{},
		Return@dataBuffer[[;;, 17]];
	]
	
GetAccelerometerAxisYFilteredBuffer[] :=
	Module[{},
		Return@dataBuffer[[;;, 18]];
	]
	
GetAccelerometerAxisZFilteredBuffer[] :=
	Module[{},
		Return@dataBuffer[[;;, 19]];
	]
	
GetGyroAxisXFilteredBuffer[] :=
	Module[{},
		Return@dataBuffer[[;;, 20]];
	]
	
GetGyroAxisYFilteredBuffer[] :=
	Module[{},
		Return@dataBuffer[[;;, 21]];
	]
	
GetGyroAxisZFilteredBuffer[] :=
	Module[{},
		Return@dataBuffer[[;;, 22]];
	]
GetRightMotorBackEMFFilteredBuffer[] :=
	Module[{},
		Return@dataBuffer[[;;, 26]];
	]
	
GetLeftMotorBackEMFFilteredBuffer[] :=
	Module[{},
		Return@dataBuffer[[;;, 27]];
	]

(* Mask 2*)
GetQuaternionQ0Buffer[] :=
	Module[{},
		Return@dataBuffer[[;;, 33]];
	]
	
GetQuaternionQ1Buffer[] :=
	Module[{},
		Return@dataBuffer[[;;, 34]];
	]
	
GetQuaternionQ2Buffer[] :=
	Module[{},
		Return@dataBuffer[[;;, 35]];
	]
	
GetQuaternionQ3Buffer[] :=
	Module[{},
		Return@dataBuffer[[;;, 36]];
	]
	
GetOdometerXBuffer[] :=
	Module[{},
		Return@dataBuffer[[;;, 37]];
	]
	
GetOdometerYBuffer[] :=
	Module[{},
		Return@dataBuffer[[;;, 38]];
	]
	
GetAccelOneBuffer[] :=
	Module[{},
		Return@dataBuffer[[;;, 39]];
	]
	
GetVelocityXBuffer[] :=
	Module[{},
		Return@dataBuffer[[;;, 40]];
	]
	
GetVelocityYBuffer[] :=
	Module[{},
		Return@dataBuffer[[;;, 41]];
	]
	
SetDataBufferLength[length_:bufferLength] :=
	Module[{},
		If[length <= 0,
			Message[SetDataBufferLength::badlength];
			Return@$Failed;,
			If[length > bufferLength,
				dataBuffer = Join[Table[Table[0,{64}], {(length - bufferLength)}], dataBuffer];,
				dataBuffer = dataBuffer[[bufferLength + 1 - length;;bufferLength]]
			];
			bufferLength = length;
		]
	]

UpdateDataBuffer[] :=
	Module[{},
		If[bufferLength > 1,
			dataBuffer[[1;;bufferLength - 1]] = dataBuffer[[2;;bufferLength]];
			dataBuffer[[bufferLength]] = allData;,
			If[ bufferLength == 1,
				dataBuffer = {allData};,
				Message[UpdateDataBuffer::badlength];
				Return@$Failed;
			]
		]
	]

End[]

EndPackage[]

