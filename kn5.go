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

	REG_CONFIG       = 0x1a
	REG_GYRO_CONFIG  = 0x1b
	REG_ACCEL_CONFIG = 0x1c

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

	PARAM_SLEEP       = 0x40
	PARAM_ACCEL_FS_2  = 0x00
	PARAM_GYRO_FS_250 = 0x00

	REG_CLOCK_PLL_XGYRO   = 0x01
	REG_SMPLRT_DIV_CONFIG = 0x19
	REG_SMPLRT_DIV        = 0x07
)

type MPU9250 struct {
	i2c *i2c.I2C
}

func NewMPU9250(i2c *i2c.I2C) (*MPU9250, error) {
	this := &MPU9250{i2c}
	return this, nil
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

	//TODO hacer toda la lectura de una tacada

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

func checkError(err error) {
	if err != nil {
		log.Fatal(err)
	}
}

func main() {
	thei2c, err := i2c.NewI2C(DEVICE_ADDRESS, 1)
	checkError(err)
	defer thei2c.Close()

	mpu, err := NewMPU9250(thei2c)
	checkError(err)
	time0 := time.Now()
	mpu.Wake()
	//test the sensor
	_, _, _, err = mpu.GetAccel()
	checkError(err)
	_, _, _, err = mpu.GetGyro()
	checkError(err)
	log.Println("Sensor Ready!")

	for i := 0; i < 100; i++ {
		ax, ay, az, gx, gy, gz := mpu.GetData()
		fmt.Printf("[%d]; %f; %f; %f; %f; %f; %f\n",
			time.Now().Sub(time0)/time.Millisecond,
			float64(ax)/16384.0,
			float64(ay)/16384.0,
			float64(az)/16384.0,
			float64(gx)/131.0,
			float64(gy)/131.0,
			float64(gz)/131.0)

	}
}
