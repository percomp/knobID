// derived from d2r2 and mrmorphic

package main

import (
	"fmt"
	"github.com/d2r2/go-i2c"
	"log"
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

	SENSITIVITY_ACCEL_SF_FS_2G  = 16384
	SENSITIVITY_ACCEL_SF_FS_4G  = 8192
	SENSITIVITY_ACCEL_SF_FS_8G  = 4096
	SENSITIVITY_ACCEL_SF_FS_16G = 2048

	PARAM_GYRO_FS_250  = 0x00
	PARAM_GYRO_FS_500  = 0x08
	PARAM_GYRO_FS_1000 = 0x10
	PARAM_GYRO_FS_2000 = 0x18

	SENSITIVITY_GYRO_SF_FS_250  = 131
	SENSITIVITY_GYRO_SF_FS_500  = 65.5
	SENSITIVITY_GYRO_SF_FS_1000 = 32.8
	SENSITIVITY_GYRO_SF_FS_2000 = 16.4

	REG_CLOCK_PLL_XGYRO   = 0x01
	REG_SMPLRT_DIV_CONFIG = 0x19
	REG_SMPLRT_DIV        = 0x07
)

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

	v |= ^byte(PARAM_ACCEL_FS_16G) //set 1 bits 4:3
	switch mpu.accel_fs {
	case 2:
		v &= ^byte(PARAM_ACCEL_FS_2G)
	case 4:
		v &= ^byte(PARAM_ACCEL_FS_4G)
	case 8:
		v &= ^byte(PARAM_ACCEL_FS_8G)
	case 16:
		v &= ^byte(PARAM_ACCEL_FS_16G)
	default:
		v &= ^byte(PARAM_ACCEL_FS_2G)
	}

	e = mpu.i2c.WriteRegU8(REG_ACCEL_CONFIG, v)
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

	//create the MPU, open the i2c comm and set the accel and gyro full scale value
	//var mpu MPU9250

	mpu, err := NewMPU9250(1, 2, 250)
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
	//for i := 0; i < 100; i++ {
	for {
		//ax, ay, az, gx, gy, gz := mpu.GetData()
		//ax, ay, az, gx, gy, gz := mpu.GetAllData()
		//read the acc and gyro data in one step without err consideration
		_, _ = mpu.i2c.Write([]byte{REG_ACCEL_XOUT_H})
		_, _ = mpu.i2c.Read(buf)
		ax = uint16(buf[0])<<8 | uint16(buf[1])
		ay = uint16(buf[2])<<8 | uint16(buf[3])
		az = uint16(buf[4])<<8 | uint16(buf[5])
		//temp  = int(uint16(buf[6]<<8) + uint16(buf[7]))
		gx = uint16(buf[8])<<8 | uint16(buf[9])
		gy = uint16(buf[10])<<8 | uint16(buf[11])
		gz = uint16(buf[12])<<8 | uint16(buf[13])

		//TODO hacer la conversion a con signo de unsigned
		//TODO hacer la conversion a con signo de unsigned
		//TODO hacer la conversion a con signo de unsigned
		//TODO hacer la conversion a con signo de unsigned
		//TODO hacer la conversion a con signo de unsigned

		fmt.Printf("[%d]; %d; %d; %d; %d; %d; %d\n",
			time.Now().Sub(time0)/time.Millisecond,
			ax, ay, az, gx, gy, gz)
		/*
			fmt.Printf("[%d]; %f; %f; %f; %f; %f; %f\n",
				time.Now().Sub(time0)/time.Millisecond,
				float64(ax)/16384.0,
				float64(ay)/16384.0,
				float64(az)/16384.0,
				float64(gx)/131.0,
				float64(gy)/131.0,
				float64(gz)/131.0)
		*/

	}
}
