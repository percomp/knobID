// derived from d2r2, hathan-osman and mrmorphic

package main

import (
	"./gpio"
	"./i2c"
	//"bufio"
	"flag"
	"fmt"
	"log"
	"math"
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

type TimAccGyr struct {
	Tim time.Duration
	Acc ThreeDData //X, Y, Z  int16
	Gyr ThreeDData //X, Y, Z  int16
}

const (
	DATA_CAP                  int    = 1000 //DATA_CAP initial capacity of slice Data
	DATAFILE_EXTENSION        string = ".csv"
	CALIBRATION_SAMPLE_PERIOD int    = 200 //milliseconds
	BLINK_PERIOD              int    = 500 //milliseconds
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

func blink(led *gpio.Pin, percent <-chan float64) {
	//makes led to blink at frec frequency
	//freq is in [0..1], where 0 means no blink, exit
	var frequency int
	for p := range percent {
		if p == 0.0 {
			led.Write(gpio.LOW)
			return
		}
		frequenc12
		int(p * float64(BLINK_PERIOD/2))
		log.Printf("frequency %d\n", frequency)
		led.Write(gpio.HIGH)
		time.Sleep(time.Duration(frequency) * time.Millisecond)
		led.Write(gpio.LOW)
		time.Sleep(time.Duration(frequency) * time.Millisecond)
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

	e = mpu.i2c.WriteRegU8(REG_ACCEL_CONFIG, v)
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
	case 2:
		v &= byte(PARAM_GYRO_FS_250)
	case 4:
		v &= byte(PARAM_GYRO_FS_500)
	case 8:
		v &= byte(PARAM_GYRO_FS_1000)
	case 16:
		v &= byte(PARAM_GYRO_FS_2000)
	default:
		v &= byte(PARAM_GYRO_FS_250)
	}

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

func (mpu *MPU9250) GetAccGyroData() (accelX int, accelY int, accelZ int, gyroX int, gyroY int, gyroZ int, err error) {
	// based on mrmorphic/hwio/gy520.go
	//read the acc and gyro data in one step
	var (
		ax uint16
		ay uint16
		az uint16
		gx uint16
		gy uint16
		gz uint16
	)

	_, err = mpu.i2c.Write([]byte{REG_ACCEL_XOUT_H})
	if err != nil {
		return 0, 0, 0, 0, 0, 0, err //error!
	}
	buf := make([]byte, 14) //to store 14 bytes
	_, err = mpu.i2c.Read(buf)
	if err != nil {
		return 0, 0, 0, 0, 0, 0, err //error!
	}
	ax = uint16(buf[0])<<8 | uint16(buf[1])
	if ax > 32767 {
		ax = ^ax + 0x01
		accelX = -1 * int(ax)
	} else {
		accelX = int(ax)
	}

	ay = uint16(buf[2])<<8 | uint16(buf[3])
	if ay > 32767 {
		ay = ^ay + 0x01
		accelY = -1 * int(ay)
	} else {
		accelY = int(ay)
	}

	az = uint16(buf[4])<<8 | uint16(buf[5])
	if az > 32767 {
		az = ^az + 0x01
		accelZ = -1 * int(az)
	} else {
		accelZ = int(az)
	}

	gx = uint16(buf[8])<<8 | uint16(buf[9])
	if gx > 32767 {
		gx = ^gx + 0x01
		gyroX = -1 * int(gx)
	} else {
		gyroX = int(gx)
	}

	gy = uint16(buf[10])<<8 | uint16(buf[11])
	if gy > 32767 {
		gy = ^gy + 0x01
		gyroY = -1 * int(gy)
	} else {
		gyroY = int(gy)
	}

	gz = uint16(buf[12])<<8 | uint16(buf[13])
	if gz > 32767 {
		gz = ^gz + 0x01
		gyroZ = -1 * int(gz)
	} else {
		gyroZ = int(gz)
	}

	return accelX, accelY, accelZ, gyroX, gyroY, gyroZ, nil
}

/*
// These functions are wrong!!!
// These functions are wrong!!!
// These functions are wrong!!!

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

*/
func checkError(err error) {
	if err != nil {
		log.Fatal(err)
	}
}

func calibrateAcc(mpu *MPU9250, xled *gpio.Pin, yled *gpio.Pin, zled *gpio.Pin, precision float64) {

	var (
		maxAcc    int
		lapseOnAx = make(chan float64, 4) // [0..1] 0 means no blink and exit
		lapseOnAy = make(chan float64, 4) // [0..1] 0 means no blink and exit
		lapseOnAz = make(chan float64, 4) // [0..1] 0 means no blink and exit
		maxOfDev  int
	)

	switch mpu.accel_fs {
	case 2:
		maxAcc = int(SENSITIVITY_ACCEL_SF_FS_2G)
		maxOfDev = int(precision * SENSITIVITY_ACCEL_SF_FS_2G)
	case 4:
		maxAcc = int(SENSITIVITY_ACCEL_SF_FS_4G)
		maxOfDev = int(precision * SENSITIVITY_ACCEL_SF_FS_4G)
	case 8:
		maxAcc = int(SENSITIVITY_ACCEL_SF_FS_8G)
		maxOfDev = int(precision * SENSITIVITY_ACCEL_SF_FS_8G)
	case 16:
		maxAcc = int(SENSITIVITY_ACCEL_SF_FS_16G)
		maxOfDev = int(precision * SENSITIVITY_ACCEL_SF_FS_16G)
	}

	go blink(xled, lapseOnAx)
	go blink(yled, lapseOnAy)
	go blink(zled, lapseOnAz)

	devAx := maxOfDev
	devAy := maxOfDev
	devAz := maxOfDev
	for devAx >= maxOfDev || devAy >= maxOfDev || devAz >= maxOfDev {
		ax, ay, az, _, _, _, _ := mpu.GetAccGyroData()
		log.Printf("%d, %d, %d\n", ax, ay, az)
		//calculate desviation
		//devAx
		if int(math.Abs(float64(ax))) <= maxAcc { //TODO sometimes ax is > maxAx !!!
			if int(math.Abs(float64(ax))) < maxAcc/2 {
				devAx = int(math.Abs(float64(ax)))
			} else {
				devAx = maxAcc - int(math.Abs(float64(ax)))
			}
			lapseOnAx <- float64(devAx) / float64(maxAcc/2)
		}
		//devAy
		if int(math.Abs(float64(ay))) <= maxAcc { //TODO sometimes ax is > maxAx !!!
			if int(math.Abs(float64(ay))) < maxAcc/2 {
				devAy = int(math.Abs(float64(ay)))
			} else {
				devAy = maxAcc - int(math.Abs(float64(ay)))
			}
			lapseOnAy <- float64(devAy) / float64(maxAcc/2)
		}
		//devAz
		if int(math.Abs(float64(az))) <= maxAcc { //TODO sometimes ax is > maxAx !!!
			if int(math.Abs(float64(az))) < maxAcc/2 {
				devAz = int(math.Abs(float64(az)))
			} else {
				devAz = maxAcc - int(math.Abs(float64(az)))
			}
			lapseOnAz <- float64(devAz) / float64(maxAcc/2)
		}
		log.Printf("devAx %d [of %d, max %d] \n", devAx, maxOfDev, maxAcc/2)
		log.Printf("devAy %d [of %d, max %d] \n", devAy, maxOfDev, maxAcc/2)
		log.Printf("devAz %d [of %d, max %d] \n", devAz, maxOfDev, maxAcc/2)

		time.Sleep(time.Duration(CALIBRATION_SAMPLE_PERIOD) * time.Millisecond)
	}
	lapseOnAx <- 0.0 //stop blink
	lapseOnAy <- 0.0 //stop blink
	lapseOnAz <- 0.0 //stop blink

}

func main() {

	//environment data
	var (
		//Data slice with raw data readed from mpu sensor
		dataStore       = make([]TimAccGyr, 0, DATA_CAP)
		dataFile        *os.File
		dataFilePath    string
		dataFileName    string
		acquisitionName string
		acquisitionNum  int
		dataDirectory   string
		thisData        TimAccGyr
		presenceBefore  bool
		presence        = make(chan bool)
	)

	acquisitionNum = 0 //increased each infrared sensor (ir) activation

	//args processing

	var nameArg string
	var dirArg string
	var accFS int
	var accFSMAX float64
	var gyrFS int
	var gyrFSMAX float64

	flag.StringVar(&nameArg, "name", "event", "Name of the acquisition")
	flag.StringVar(&dirArg, "dir", "data", "Directory where store acquisitions")
	flag.IntVar(&accFS, "acc", 2, "Accelerometer full scale g (2, 4, 8, 16)")
	flag.IntVar(&gyrFS, "gyro", 250, "Gyroscope full scale dps (250, 500, 1000, 20000)")

	flag.Parse()

	log.Printf("Arguments:")
	log.Printf("\t Name: %s", nameArg)
	log.Printf("\t Dir: %s", dirArg)
	log.Printf("\t Acc: %d", accFS)
	log.Printf("\t Gyro: %d", gyrFS)

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
	}

	const (
		PIN_RED_LED    int = 4
		PIN_YELLOW_LED int = 27
		PIN_GREEN_LED  int = 22
		PIN_IR         int = 17
	)

	//Red led
	redLed, err := gpio.OpenPin(PIN_RED_LED, gpio.OUT)
	if err != nil {
		log.Fatal(err)
	}
	defer redLed.Close()
	defer redLed.Write(gpio.LOW)

	//Yellow led
	yellowLed, err := gpio.OpenPin(PIN_YELLOW_LED, gpio.OUT)
	if err != nil {
		log.Fatal(err)
	}
	defer yellowLed.Close()
	defer yellowLed.Write(gpio.LOW)

	//Green led
	greenLed, err := gpio.OpenPin(PIN_GREEN_LED, gpio.OUT)
	if err != nil {
		log.Fatal(err)
	}
	defer greenLed.Close()
	defer greenLed.Write(gpio.LOW)

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
	_, _, _, _, _, _, err = mpu.GetAccGyroData()
	checkError(err)
	log.Println("Sensor Ready!")

	//CALIBRATE
	log.Println("Calibrating the sensor")
	precision := 0.001
	calibrateAcc(mpu, redLed, yellowLed, greenLed, precision)

	//RUN
	log.Println("Ready to read the sensor")
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
	go readIR(ir, redLed, presence)

	for presenceNow := range presence {

		if presenceNow {
			if !presenceBefore { //begin a capture
				log.Println("Presence detected, begin acquisition")
				time0 = time.Now() //reset time of measures
				acquisitionNum++   //increase the num of acquisitions
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
				log.Println("Absence detected, stop acquisition")
				log.Printf("Data store size: %d (of %d)", len(dataStore), cap(dataStore))
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
				headLine = headLine + fmt.Sprintf("# Accelerometer full scale: %d (%f)\n", accFS, accFSMAX)
				headLine = headLine + fmt.Sprintf("# Gyroscope full scale: %d (%f)\n", gyrFS, gyrFSMAX)
				headLine = headLine + "##########\n"
				headLine = headLine + fmt.Sprintf("num; time(us); accX(g); accY(g); accZ(g); gyrX(o/s); gyrY(o/s); gyrZ(o/s)\n")
				dataFile.WriteString(headLine) //write headding line in the file
				//write data to the file
				for i, value := range dataStore {
					redLed.Toggle() //indicate transferring state with red led
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
				redLed.Write(gpio.LOW)
				dataFile.Close()
				log.Printf("Closed %s\n", dataFileName)

				//initialize the slice to prepare it for new data
				dataStore = make([]TimAccGyr, 0, DATA_CAP)
				//log.Printf("New data store size: %d (of %d)", len(dataStore), cap(dataStore))
				log.Printf("Ready for new acquisition....")

			}
		}
		presenceBefore = presenceNow

	}
	//END
}
