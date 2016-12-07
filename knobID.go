// knobID sensor platform to study the knob use microaction
// hist sakfaoin fñalskdnfñands a
// derived from d2r2, hathan-osman and mrmorphic

package main

import (
	"./gpio"
	"./i2c"
	"flag"
	"fmt"
	"log"
	"os"
	"path/filepath"
	"time"
)

const (
	// based on mrmorphic/hwio/gy520.go
	// This is the default address. Some devices may also respond to 0x69
	DEVICE_ADDRESS = 0x68

	REG_CONFIG         = 0x1a
	REG_GYRO_CONFIG    = 0x1b
	REG_ACCEL_CONFIG   = 0x1c
	REG_ACCEL_CONFIG_2 = 0x1d

	// accelerometer sensor registers, read-only
	REG_ACCEL_XOUT_H = 0x3b
	REG_ACCEL_XOUT_L = 0x3c
	REG_ACCEL_YOUT_H = 0x3d
	REG_ACCEL_YOUT_L = 0x3e
	REG_ACCEL_ZOUT_H = 0x3f
	REG_ACCEL_ZOUT_L = 0x40

	// temperature sensor registers, read-only
	REG_TEMP_OUT_H = 0x41
	REG_TEMP_OUT_L = 0x42

	// gyroscope sensor registers, read-only
	REG_GYRO_XOUT_H = 0x43
	REG_GYRO_XOUT_L = 0x44
	REG_GYRO_YOUT_H = 0x45
	REG_GYRO_YOUT_L = 0x46
	REG_GYRO_ZOUT_H = 0x47
	REG_GYRO_ZOUT_L = 0x48

	REG_PWR_MGMT_1 = 0x6b

	PARAM_SLEEP = 0x40

	PARAM_ACCEL_FS_2G     = 0x00
	PARAM_ACCEL_FS_4G     = 0x08
	PARAM_ACCEL_FS_8G     = 0x10
	PARAM_ACCEL_FS_16G    = 0x18
	PARAM_ACCEL_FCHOICE_b = 0x08

	SENSITIVITY_ACCEL_SF_FS_2G  = 16384.0
	SENSITIVITY_ACCEL_SF_FS_4G  = 8192.0
	SENSITIVITY_ACCEL_SF_FS_8G  = 4096.0
	SENSITIVITY_ACCEL_SF_FS_16G = 2048.0

	PARAM_GYRO_FS_250  = 0x00
	PARAM_GYRO_FS_500  = 0x08
	PARAM_GYRO_FS_1000 = 0x10
	PARAM_GYRO_FS_2000 = 0x18

	SENSITIVITY_GYRO_SF_FS_250  = 131.0
	SENSITIVITY_GYRO_SF_FS_500  = 65.5
	SENSITIVITY_GYRO_SF_FS_1000 = 32.8
	SENSITIVITY_GYRO_SF_FS_2000 = 16.4

	REG_CLOCK_PLL_XGYRO   = 0x01
	REG_SMPLRT_DIV_CONFIG = 0x19
	REG_SMPLRT_DIV        = 0x07
)

type ThreeDData struct {
	X int16
	Y int16
	Z int16
}

type preTimAccGyr struct {
	Tim time.Time
	Acc ThreeDData //X, Y, Z  int16
	Gyr ThreeDData //X, Y, Z  int16
}

type TimAccGyr struct {
	Tim time.Duration
	Acc ThreeDData //X, Y, Z  int16
	Gyr ThreeDData //X, Y, Z  int16
}

const (
	DATA_CAP           int    = 1000 //DATA_CAP initial capacity of slice Data
	PRE_DATA_CAP       int    = 1000 //PRE_DATA_CAP initial capacity of circular array of prefechted Data
	DATAFILE_EXTENSION string = ".csv"
)

// GPIO section =================
// GPIO section =================
// GPIO section =================

func readIR(ir *gpio.Pin, led *gpio.Pin, presence chan<- bool) {

	var valueOld = ir.GetStatus() //at beginning
	var valueNow gpio.Value
	//var err error
	//make the led match the ir
	led.Write(ir.GetStatus())
	for {
		valueNow, _ = ir.Read()
		presence <- (valueNow == gpio.HIGH)
		if valueNow != valueOld {
			valueOld = valueNow
			_, _ = led.Toggle()
		}
	}
}

// MPU section ==================
// MPU section ==================
// MPU section ==================

type MPU9250 struct {
	i2c      *i2c.I2C
	accel_fs int
	gyro_fs  int
}

func NewMPU9250(devAddress int, a_fs int, g_fs int) (*MPU9250, error) {
	thei2c, err := i2c.NewI2C(DEVICE_ADDRESS, devAddress)
	if err != nil {
		return nil, err
	}
	theMPU := MPU9250{i2c: thei2c, accel_fs: a_fs, gyro_fs: g_fs}
	return &theMPU, nil
}

// ConfigAcc Config the accelerometer Full Scale
func (mpu *MPU9250) Config() error {
	log.Printf("Configuration parameters:")
	// config accel
	v, e := mpu.i2c.ReadRegU8(REG_ACCEL_CONFIG)
	if e != nil {
		return e
	}

	v |= byte(PARAM_ACCEL_FS_16G) //has bits 4:3 set to 1
	switch mpu.accel_fs {
	case 2:
		v &= byte(PARAM_ACCEL_FS_2G)
	case 4:
		v &= byte(PARAM_ACCEL_FS_4G)
	case 8:
		v &= byte(PARAM_ACCEL_FS_8G)
	case 16:
		v &= byte(PARAM_ACCEL_FS_16G)
	default:
		v &= byte(PARAM_ACCEL_FS_2G)
	}
	log.Printf("\tACCEL_FS: %#x", byte(v))

	e = mpu.i2c.WriteRegU8(REG_ACCEL_CONFIG, v)
	if e != nil {
		return e
	}

	time.Sleep(10 * time.Millisecond)

	//Setup acc data rates to maximun avoiding DLPF
	v, e = mpu.i2c.ReadRegU8(REG_ACCEL_CONFIG_2)
	if e != nil {
		return e
	}

	//log.Printf("v before: %b", v)
	v |= byte(PARAM_ACCEL_FCHOICE_b) //has bits 4 set to 1
	log.Printf("\tACCEL_FCHOICE_B: %#x", byte(v))
	//log.Printf("FCHOICE_B %b", byte(0x00))
	//v = byte(0x00) //has bits 4 set to 1
	//log.Printf("v after: %b", v)

	e = mpu.i2c.WriteRegU8(REG_ACCEL_CONFIG_2, v)
	if e != nil {
		return e
	}

	time.Sleep(10 * time.Millisecond)

	// config gyro

	v, e = mpu.i2c.ReadRegU8(REG_GYRO_CONFIG)
	if e != nil {
		return e
	}

	v |= byte(PARAM_GYRO_FS_2000) //has bits 4:3 set to 1
	switch mpu.gyro_fs {
	case 250:
		v &= byte(PARAM_GYRO_FS_250)
	case 500:
		v &= byte(PARAM_GYRO_FS_500)
	case 1000:
		v &= byte(PARAM_GYRO_FS_1000)
	case 2000:
		v &= byte(PARAM_GYRO_FS_2000)
	default:
		v &= byte(PARAM_GYRO_FS_250)
	}

	log.Printf("\tGYRO_FS: %#x", byte(v))
	e = mpu.i2c.WriteRegU8(REG_GYRO_CONFIG, v)
	if e != nil {
		return e
	}

	time.Sleep(10 * time.Millisecond)

	return nil
}

// Wake the device. By default on power on, the device is asleep.
// based on mrmorphic/hwio/gy520.go
func (mpu *MPU9250) Wake() error {
	v, e := mpu.i2c.ReadRegU8(REG_PWR_MGMT_1)
	if e != nil {
		return e
	}

	v &= ^byte(PARAM_SLEEP)

	e = mpu.i2c.WriteRegU8(REG_PWR_MGMT_1, v)
	if e != nil {
		return e
	}

	return nil
}

// Put the device back to sleep.
// based on mrmorphic/hwio/gy520.go
func (mpu *MPU9250) Sleep() error {
	v, e := mpu.i2c.ReadRegU8(REG_PWR_MGMT_1)
	if e != nil {
		return e
	}

	v |= ^byte(PARAM_SLEEP)

	e = mpu.i2c.WriteRegU8(REG_PWR_MGMT_1, v)
	if e != nil {
		return e
	}

	return nil
}

func (mpu *MPU9250) GetData() (accelX int, accelY int, accelZ int, gyroX int, gyroY int, gyroZ int) {
	// based on mrmorphic/hwio/gy520.go

	theAccelX, _ := mpu.i2c.ReadRegU16BE(REG_ACCEL_XOUT_H)
	theAccelY, _ := mpu.i2c.ReadRegU16BE(REG_ACCEL_YOUT_H)
	theAccelZ, _ := mpu.i2c.ReadRegU16BE(REG_ACCEL_ZOUT_H)
	theGyroX, _ := mpu.i2c.ReadRegU16BE(REG_GYRO_XOUT_H)
	theGyroY, _ := mpu.i2c.ReadRegU16BE(REG_GYRO_YOUT_H)
	theGyroZ, _ := mpu.i2c.ReadRegU16BE(REG_GYRO_ZOUT_H)

	return int(theAccelX), int(theAccelY), int(theAccelZ), int(theGyroX), int(theGyroY), int(theGyroZ)
}

func (mpu *MPU9250) GetAllData() (accelX int, accelY int, accelZ int, gyroX int, gyroY int, gyroZ int) {
	// based on mrmorphic/hwio/gy520.go
	//read the acc and gyro data in one step

	_, err := mpu.i2c.Write([]byte{REG_ACCEL_XOUT_H})
	if err != nil {
		return 0, 0, 0, 0, 0, 0 //error!
	}
	buf := make([]byte, 14) //to store 14 bytes
	_, err = mpu.i2c.Read(buf)
	if err != nil {
		return 0, 0, 0, 0, 0, 0 //error!
	}
	ax := int(uint16(buf[0]<<8) + uint16(buf[1]))
	ay := int(uint16(buf[2]<<8) + uint16(buf[3]))
	az := int(uint16(buf[4]<<8) + uint16(buf[5]))
	//temp  := int(uint16(buf[6]<<8) + uint16(buf[7]))
	gx := int(uint16(buf[8]<<8) + uint16(buf[9]))
	gy := int(uint16(buf[10]<<8) + uint16(buf[11]))
	gz := int(uint16(buf[12]<<8) + uint16(buf[13]))

	return int(ax), int(ay), int(az), int(gx), int(gy), int(gz)
}

func (mpu *MPU9250) GetAccel() (accelX int, accelY int, accelZ int, e error) {
	// based on mrmorphic/hwio/gy520.go

	theAccelX, err := mpu.i2c.ReadRegU16BE(REG_ACCEL_XOUT_H)
	if err != nil {
		return 0, 0, 0, err
	}
	theAccelY, err := mpu.i2c.ReadRegU16BE(REG_ACCEL_YOUT_H)
	if err != nil {
		return 0, 0, 0, err
	}
	theAccelZ, err := mpu.i2c.ReadRegU16BE(REG_ACCEL_ZOUT_H)
	if err != nil {
		return 0, 0, 0, err
	}

	return int(theAccelX), int(theAccelY), int(theAccelZ), nil
}

func (mpu *MPU9250) GetGyro() (gyroX int, gyroY int, gyroZ int, e error) {
	// based on mrmorphic/hwio/gy520.go

	theGyroX, err := mpu.i2c.ReadRegU16BE(REG_GYRO_XOUT_H)
	if err != nil {
		return 0, 0, 0, err
	}
	theGyroY, err := mpu.i2c.ReadRegU16BE(REG_GYRO_YOUT_H)
	if err != nil {
		return 0, 0, 0, err
	}
	theGyroZ, err := mpu.i2c.ReadRegU16BE(REG_GYRO_ZOUT_H)
	if err != nil {
		return 0, 0, 0, err
	}

	return int(theGyroX), int(theGyroY), int(theGyroZ), nil
}

func checkError(err error) {
	if err != nil {
		log.Fatal(err)
	}
}

func main() {

	//environment data
	var (
		//Data slices with raw data readed from mpu sensor
		dataStore       = make([]TimAccGyr, 0, DATA_CAP)
		dataFile        *os.File
		dataFilePath    string
		dataFileName    string
		acquisitionName string
		acquisitionNum  int
		dataDirectory   string
		preThisData     preTimAccGyr
		thisData        TimAccGyr
		presenceBefore  bool
		presence        = make(chan bool)
		firstValue      int
		lastValue       int
		preValue        preTimAccGyr
	)

	acquisitionNum = 0 //increased each infrared sensor (ir) activation

	//args processing

	var nameArg string
	var dirArg string
	var accFS int
	var accFSMAX float64
	var gyrFS int
	var gyrFSMAX float64
	var margin int

	flag.StringVar(&nameArg, "name", "event", "Name of the acquisition")
	flag.StringVar(&dirArg, "dir", "data", "Directory where store acquisitions")
	flag.IntVar(&accFS, "acc", 2, "Accelerometer full scale g (2, 4, 8, 16)")
	flag.IntVar(&gyrFS, "gyro", 250, "Gyroscope full scale dps (250, 500, 1000, 20000)")
	flag.IntVar(&margin, "marg", 250, fmt.Sprintf("Margin of data to acquire (< %d)", PRE_DATA_CAP))

	flag.Parse()

	log.Printf("Arguments:")
	log.Printf("\t Name: %s", nameArg)
	log.Printf("\t Dir: %s", dirArg)
	log.Printf("\t Acc: %d", accFS)
	log.Printf("\t Gyro: %d", gyrFS)
	log.Printf("\t Marg: %d", margin)

	if margin < 0 {
		margin = 0
		log.Printf("Margin negative!, set to 0: %d", margin)
	}

	if margin > PRE_DATA_CAP {
		margin = PRE_DATA_CAP
		log.Printf("Margin too big, set to the maximun available: %d", margin)
	}

	preDataStore := make([]preTimAccGyr, margin)
	//set the vars regarding the args
	acquisitionName = nameArg
	dataDirectory = dirArg

	//set the full scale dependinf of acc and gyr configuration
	switch accFS {
	case 2:
		accFSMAX = SENSITIVITY_ACCEL_SF_FS_2G
	case 4:
		accFSMAX = SENSITIVITY_ACCEL_SF_FS_4G
	case 8:
		accFSMAX = SENSITIVITY_ACCEL_SF_FS_8G
	case 16:
		accFSMAX = SENSITIVITY_ACCEL_SF_FS_16G
	default:
		accFSMAX = SENSITIVITY_ACCEL_SF_FS_2G
	}

	switch gyrFS {
	case 250:
		gyrFSMAX = SENSITIVITY_GYRO_SF_FS_250
	case 500:
		gyrFSMAX = SENSITIVITY_GYRO_SF_FS_500
	case 1000:
		gyrFSMAX = SENSITIVITY_GYRO_SF_FS_1000
	case 2000:
		gyrFSMAX = SENSITIVITY_GYRO_SF_FS_2000
	default:
		gyrFSMAX = SENSITIVITY_GYRO_SF_FS_250
	}

	//create data dir if not exists
	dataFilePath = filepath.Join(".", dataDirectory)
	if _, err := os.Stat(dataFilePath); os.IsNotExist(err) {
		os.Mkdir(dataFilePath, 0666)
		log.Printf("Data dir created.")
	}

	const (
		PIN_LED int = 4
		PIN_IR  int = 22 //17
	)

	//led
	led, err := gpio.OpenPin(PIN_LED, gpio.OUT)
	if err != nil {
		log.Fatal(err)
	}
	defer led.Close()
	defer led.Write(gpio.LOW)

	//ir
	//open the pin in the GPIO
	ir, err := gpio.OpenPin(PIN_IR, gpio.IN)
	if err != nil {
		log.Fatal(err)
	}
	defer ir.Close()
	presenceBefore = false

	//create the MPU, open the i2c comm and set the accel and gyro full scale value
	//var mpu MPU9250

	mpu, err := NewMPU9250(1, accFS, gyrFS)
	checkError(err)

	defer mpu.i2c.Close()

	mpu.Wake()
	mpu.Config()
	mpu.Wake()

	//test the sensor
	_, _, _, err = mpu.GetAccel()
	checkError(err)
	_, _, _, err = mpu.GetGyro()
	checkError(err)
	log.Println("Sensor Ready!")

	//var creation out of the loop
	var (
		ax uint16
		ay uint16
		az uint16
		gx uint16
		gy uint16
		gz uint16
	)
	buf := make([]byte, 14) //to store 14 bytes
	time0 := time.Now()
	//i := 0

	//gorutine detec presence at IR and set led
	go readIR(ir, led, presence)

	for presenceNow := range presence {

		if presenceNow {
			if !presenceBefore { //begin a capture
				log.Println("Presence detected, begin acquisition")
				time0 = time.Now() //reset time of measures
				//acquisitionNum++   //increase the num of acquisitions
				//i = 0              //log purposes
			}
			//read the acc and gyro data in one step without err consideration
			_, _ = mpu.i2c.Write([]byte{REG_ACCEL_XOUT_H})
			_, _ = mpu.i2c.Read(buf)
			//time of readdings
			thisData.Tim = time.Now().Sub(time0)
			//data of readdings
			ax = uint16(buf[0])<<8 | uint16(buf[1])
			if ax > 32767 {
				ax = ^ax + 0x01
				thisData.Acc.X = -1 * int16(ax)
			} else {
				thisData.Acc.X = int16(ax)
			}

			ay = uint16(buf[2])<<8 | uint16(buf[3])
			if ay > 32767 {
				ay = ^ay + 0x01
				thisData.Acc.Y = -1 * int16(ay)
			} else {
				thisData.Acc.Y = int16(ay)
			}

			az = uint16(buf[4])<<8 | uint16(buf[5])
			if az > 32767 {
				az = ^az + 0x01
				thisData.Acc.Z = -1 * int16(az)
			} else {
				thisData.Acc.Z = int16(az)
			}

			gx = uint16(buf[8])<<8 | uint16(buf[9])
			if gx > 32767 {
				gx = ^gx + 0x01
				thisData.Gyr.X = -1 * int16(gx)
			} else {
				thisData.Gyr.X = int16(gx)
			}

			gy = uint16(buf[10])<<8 | uint16(buf[11])
			if gy > 32767 {
				gy = ^gy + 0x01
				thisData.Gyr.Y = -1 * int16(gy)
			} else {
				thisData.Gyr.Y = int16(gy)
			}

			gz = uint16(buf[12])<<8 | uint16(buf[13])
			if gz > 32767 {
				gz = ^gz + 0x01
				thisData.Gyr.Z = -1 * int16(gz)
			} else {
				thisData.Gyr.Z = int16(gz)
			}

			//log.Printf("[%d]; %d; %f; %f; %f; %f; %f; %f\n",
			//	i,
			//	time.Now().Sub(time0)/time.Millisecond,
			//	float64(thisData.Acc.X)/16384.0,
			//	float64(thisData.Acc.Y)/16384.0,
			//	float64(thisData.Acc.Z)/16384.0,
			//	float64(thisData.Gyr.X)/131.0,
			//	float64(thisData.Gyr.Y)/131.0,
			//	float64(thisData.Gyr.Z)/131.0)
			//	i += 1
			dataStore = append(dataStore, thisData)

		} else {
			if presenceBefore { //end of a capture, dump data
				//post margin acquisition
				for i := 0; i < margin; i++ {
					//here the acquistion margin post
					//read the acc and gyro data in one step without err consideration
					_, _ = mpu.i2c.Write([]byte{REG_ACCEL_XOUT_H})
					_, _ = mpu.i2c.Read(buf)
					//time of readdings
					thisData.Tim = time.Now().Sub(time0)
					log.Printf("post margin %d ", thisData.Tim/time.Microsecond)
					//data of readdings
					ax = uint16(buf[0])<<8 | uint16(buf[1])
					if ax > 32767 {
						ax = ^ax + 0x01
						thisData.Acc.X = -1 * int16(ax)
					} else {
						thisData.Acc.X = int16(ax)
					}

					ay = uint16(buf[2])<<8 | uint16(buf[3])
					if ay > 32767 {
						ay = ^ay + 0x01
						thisData.Acc.Y = -1 * int16(ay)
					} else {
						thisData.Acc.Y = int16(ay)
					}

					az = uint16(buf[4])<<8 | uint16(buf[5])
					if az > 32767 {
						az = ^az + 0x01
						thisData.Acc.Z = -1 * int16(az)
					} else {
						thisData.Acc.Z = int16(az)
					}

					gx = uint16(buf[8])<<8 | uint16(buf[9])
					if gx > 32767 {
						gx = ^gx + 0x01
						thisData.Gyr.X = -1 * int16(gx)
					} else {
						thisData.Gyr.X = int16(gx)
					}

					gy = uint16(buf[10])<<8 | uint16(buf[11])
					if gy > 32767 {
						gy = ^gy + 0x01
						thisData.Gyr.Y = -1 * int16(gy)
					} else {
						thisData.Gyr.Y = int16(gy)
					}

					gz = uint16(buf[12])<<8 | uint16(buf[13])
					if gz > 32767 {
						gz = ^gz + 0x01
						thisData.Gyr.Z = -1 * int16(gz)
					} else {
						thisData.Gyr.Z = int16(gz)
					}

					//log.Printf("[%d]; %d; %f; %f; %f; %f; %f; %f\n",
					//	i,
					//	time.Now().Sub(time0)/time.Millisecond,
					//	float64(thisData.Acc.X)/16384.0,
					//	float64(thisData.Acc.Y)/16384.0,
					//	float64(thisData.Acc.Z)/16384.0,
					//	float64(thisData.Gyr.X)/131.0,
					//	float64(thisData.Gyr.Y)/131.0,
					//	float64(thisData.Gyr.Z)/131.0)
					//	i += 1
					dataStore = append(dataStore, thisData)

				}
				log.Println("Absence detected, stop acquisition")
				log.Printf("Data store size: %d (of %d)", len(dataStore), cap(dataStore))
				//log.Printf("Tim[0]: %d", int(dataStore[0].Tim/time.Microsecond))
				//log.Printf("Tim[%d]: %d", len(dataStore)-1, int(dataStore[len(dataStore)-1].Tim/time.Microsecond))
				log.Printf("Data acquisition rate: %d Hz", int(1000000.0*float32(len(dataStore))/float32(dataStore[len(dataStore)-1].Tim/time.Microsecond)))
				//dump the dataStore on the slice into a file
				//Create and open file
				dataFileName = fmt.Sprintf("%s%d%s", filepath.Join(dataFilePath, acquisitionName), acquisitionNum, DATAFILE_EXTENSION)
				//Create and Close if not exists
				if _, err := os.Stat(dataFileName); os.IsNotExist(err) {
					log.Printf("Creating %s\n", dataFileName)
					dataFile, _ = os.Create(dataFileName)
					dataFile.Close()
				}
				//open file
				log.Printf("Opennign %s\n", dataFileName)
				dataFile, err := os.OpenFile(dataFileName, os.O_RDWR|os.O_APPEND, 0666)
				if err != nil {
					log.Println(err.Error())
				}

				//headding line
				headLine := "##########\n"
				headLine = headLine + fmt.Sprintf("# %v Data Acquisition\n", time.Now())
				headLine = headLine + fmt.Sprintf("# Acquisition name: %s\n", acquisitionName)
				headLine = headLine + fmt.Sprintf("# Acquisition num: %d\n", acquisitionNum)
				headLine = headLine + fmt.Sprintf("# Accelerometer full scale: %d (%d)\n", accFS, int(accFSMAX))
				headLine = headLine + fmt.Sprintf("# Gyroscope full scale: %d (%d)\n", gyrFS, int(gyrFSMAX))
				headLine = headLine + "##########\n"
				headLine = headLine + fmt.Sprintf("num; time(us); accX(g); accY(g); accZ(g); gyrX(o/s); gyrY(o/s); gyrZ(o/s)\n")
				dataFile.WriteString(headLine) //write headding line in the file
				if margin > 0 {
					firstValue = (lastValue + 1) % margin
					log.Printf("margin: %d, firstValue: %d, lastValue: %d", margin, firstValue, lastValue)
					for i := 1; i <= margin; i++ {
						led.Toggle() //indicate transferring state with led
						preValue = preDataStore[firstValue]
						firstValue = (firstValue + 1) % margin
						dataString := fmt.Sprintf("%d;%d;%f;%f;%f;%f;%f;%f\n",
							i,
							int64(preValue.Tim.Sub(time0)/time.Microsecond),
							float64(preValue.Acc.X)/accFSMAX,
							float64(preValue.Acc.Y)/accFSMAX,
							float64(preValue.Acc.Z)/accFSMAX,
							float64(preValue.Gyr.X)/gyrFSMAX,
							float64(preValue.Gyr.Y)/gyrFSMAX,
							float64(preValue.Gyr.Z)/gyrFSMAX)
						dataFile.WriteString(dataString) //write data in the file

					}
					dataFile.WriteString("======================================================\n")
				}
				//write dataStore to the file
				for i, value := range dataStore {
					led.Toggle() //indicate transferring state with led
					dataString := fmt.Sprintf("%d;%d;%f;%f;%f;%f;%f;%f\n",
						i,
						int64(value.Tim/time.Microsecond),
						float64(value.Acc.X)/accFSMAX,
						float64(value.Acc.Y)/accFSMAX,
						float64(value.Acc.Z)/accFSMAX,
						float64(value.Gyr.X)/gyrFSMAX,
						float64(value.Gyr.Y)/gyrFSMAX,
						float64(value.Gyr.Z)/gyrFSMAX)
					dataFile.WriteString(dataString) //write data in the file
				}
				led.Write(gpio.LOW)
				dataFile.Close()
				log.Printf("Closed %s\n", dataFileName)

				acquisitionNum++ //increase num of acquisitions for the next time
				//initialize the slices to prepare it for new data
				preDataStore = make([]preTimAccGyr, PRE_DATA_CAP)
				lastValue = -1
				dataStore = make([]TimAccGyr, 0, DATA_CAP)
				//log.Printf("New data store size: %d (of %d)", len(dataStore), cap(dataStore))
				log.Printf("Ready for new acquisition....")

			} else { //while no presence detected prefech data and store in prebuf[]

				if margin > 0 {
					//read the acc and gyro data in one step without err consideration
					_, _ = mpu.i2c.Write([]byte{REG_ACCEL_XOUT_H})
					_, _ = mpu.i2c.Read(buf)
					//time of readdings
					preThisData.Tim = time.Now() //TODO .Sub(time0) substract time0 when dump to the file
					//data of readdings
					ax = uint16(buf[0])<<8 | uint16(buf[1])
					if ax > 32767 {
						ax = ^ax + 0x01
						preThisData.Acc.X = -1 * int16(ax)
					} else {
						preThisData.Acc.X = int16(ax)
					}

					ay = uint16(buf[2])<<8 | uint16(buf[3])
					if ay > 32767 {
						ay = ^ay + 0x01
						preThisData.Acc.Y = -1 * int16(ay)
					} else {
						preThisData.Acc.Y = int16(ay)
					}

					az = uint16(buf[4])<<8 | uint16(buf[5])
					if az > 32767 {
						az = ^az + 0x01
						preThisData.Acc.Z = -1 * int16(az)
					} else {
						preThisData.Acc.Z = int16(az)
					}

					gx = uint16(buf[8])<<8 | uint16(buf[9])
					if gx > 32767 {
						gx = ^gx + 0x01
						preThisData.Gyr.X = -1 * int16(gx)
					} else {
						preThisData.Gyr.X = int16(gx)
					}

					gy = uint16(buf[10])<<8 | uint16(buf[11])
					if gy > 32767 {
						gy = ^gy + 0x01
						preThisData.Gyr.Y = -1 * int16(gy)
					} else {
						preThisData.Gyr.Y = int16(gy)
					}

					gz = uint16(buf[12])<<8 | uint16(buf[13])
					if gz > 32767 {
						gz = ^gz + 0x01
						preThisData.Gyr.Z = -1 * int16(gz)
					} else {
						preThisData.Gyr.Z = int16(gz)
					}

					lastValue = (lastValue + 1) % margin
					log.Printf("lastValue: %d", lastValue)
					preDataStore[lastValue] = preThisData
				}
			}
		}
		presenceBefore = presenceNow
	}
	//END
}
