package main

import (
	"flag"
	"fmt"
	"log"
	"os"
	"path/filepath"
	"time"

	"github.com/hybridgroup/gobot"
	"github.com/hybridgroup/gobot/platforms/gpio"
	"github.com/hybridgroup/gobot/platforms/i2c"
	"github.com/hybridgroup/gobot/platforms/raspi"
)

type TimAccGyr struct {
	Tim time.Duration
	Acc i2c.ThreeDData //X, Y, Z  int16
	Gyr i2c.ThreeDData //X, Y, Z  int16
}

const (
	//DATA_CAP initial capacity of slice Data
	DATA_CAP           int    = 100
	DATAFILE_EXTENSION string = ".csv"
)

var (
	//Data slice with raw data readed from mpu sensor
	dataStore       = make([]TimAccGyr, 0, DATA_CAP)
	dataFile        *os.File
	dataFilePath    string
	dataFileName    string
	acquisitionName string
	acquisitionNum  int
	dataDirectory   string
)

func main() {

	var thisData TimAccGyr
	acquisitionNum = 0 //increased each buttonPush

	//dataDirectory = "data"
	//acquisitionName = "datos" //read this from args

	//sensor MPU and gobot read frequency
	var sensorFreq time.Duration //= 5 //read this from args
	var readFreq time.Duration   //= 8   //read this from args

	//args processing

	var nameArg string
	var dirArg string
	var sensorFreqArg int
	var readFreqArg int

	flag.StringVar(&nameArg, "name", "event", "Name of the acquisition.")
	flag.StringVar(&dirArg, "dir", "data", "Directory for store acquisitios.")
	flag.IntVar(&sensorFreqArg, "sensor", 5, "Sensor frecuency (ms)")
	flag.IntVar(&readFreqArg, "read", 8, "Read frecuency (ms)")

	flag.Parse()

	log.Printf("Arguments:")
	log.Printf("\t Name: %s", nameArg)
	log.Printf("\t Dir: %s", dirArg)
	log.Printf("\t Sensor: %d", sensorFreqArg)
	log.Printf("\t Read: %d", readFreqArg)

	//set the vars regarding the args
	acquisitionName = nameArg
	dataDirectory = dirArg
	sensorFreq = time.Duration(sensorFreqArg)
	readFreq = time.Duration(readFreqArg)

	//create data dir if not exists
	dataFilePath = filepath.Join(".", dataDirectory)
	if _, err := os.Stat(dataFilePath); os.IsNotExist(err) {
		os.Mkdir(dataFilePath, 0666)
	}
	mainBot := gobot.NewGobot()

	r := raspi.NewRaspiAdaptor("raspi")
	button := gpio.NewButtonDriver(r, "button", "11")
	led := gpio.NewLedDriver(r, "led", "7")
	mpu6050 := i2c.NewMPU6050Driver(r, "mpu6050", sensorFreq*time.Millisecond)
	time0 := time.Now()
	readNow := false

	workSensor := func() {
		gobot.Every(readFreq*time.Millisecond, func() {
			if readNow {
				//append the slice with the data in mpu6050
				thisData.Tim = time.Now().Sub(time0)
				thisData.Acc.X = mpu6050.Accelerometer.X
				thisData.Acc.Y = mpu6050.Accelerometer.Y
				thisData.Acc.Z = mpu6050.Accelerometer.Z
				thisData.Gyr.X = mpu6050.Gyroscope.X
				thisData.Gyr.Y = mpu6050.Gyroscope.Y
				thisData.Gyr.Z = mpu6050.Gyroscope.Z
				//log.Printf("thisData: %v;%d;%d;%d;%d;%d;%d\n",
				//		thisData.Tim,
				//		thisData.Acc.X, thisData.Acc.Y, thisData.Acc.Z,
				//		thisData.Gyr.X, thisData.Gyr.Y, thisData.Gyr.Z)
				dataStore = append(dataStore, thisData)
			}
		})
	}

	robotSensor := gobot.NewRobot("mpu6050Bot",
		[]gobot.Connection{r},
		[]gobot.Device{mpu6050},
		workSensor,
	)

	workButton := func() {

		button.On(gpio.ButtonPush, func(data interface{}) {
			log.Println("Presence detected, begin acquisition")
			led.On()
			time0 = time.Now() //reset time of measures
			acquisitionNum++   //increase the num of acquisitions
			readNow = true

		})

		button.On(gpio.ButtonRelease, func(data interface{}) {
			log.Println("Absence detected, stop acquisition")
			readNow = false
			led.Off()
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
			headLine = headLine + fmt.Sprintf("# Sensor Frequency: %d\n", sensorFreq)
			headLine = headLine + fmt.Sprintf("# Read Frequency: %d\n", readFreq)
			headLine = headLine + "##########\n"
			headLine = headLine + fmt.Sprintf("num; time(us); accX(g); accY(g); accZ(g); gyrX(o/s); gyrY(o/s); gyrZ(o/s)\n")
			dataFile.WriteString(headLine) //write headding line in the file
			//write data to the file
			for i, value := range dataStore {
				led.Toggle() //indicate transferring state with led
				dataString := fmt.Sprintf("%d;%d;%f;%f;%f;%f;%f;%f\n",
					i,
					int64(value.Tim/time.Microsecond),
					float64(value.Acc.X)/16384.0,
					float64(value.Acc.Y)/16384.0,
					float64(value.Acc.Z)/16384.0,
					float64(value.Gyr.X)/131.0,
					float64(value.Gyr.Y)/131.0,
					float64(value.Gyr.Z)/131.0)
				dataFile.WriteString(dataString) //write data in the file
			}
			dataFile.Close()
			log.Printf("Closed %s\n", dataFileName)

			//initialize the slice to prepare it for new data
			led.Off() //just in case led is On
			dataStore = make([]TimAccGyr, 0, DATA_CAP)
			//log.Printf("New data store size: %d (of %d)", len(dataStore), cap(dataStore))
			log.Printf("Ready for new acquisition....")
		})
	}

	robotButton := gobot.NewRobot("buttonBot",
		[]gobot.Connection{r},
		[]gobot.Device{button, led},
		workButton,
	)
	mainBot.AddRobot(robotButton)
	mainBot.AddRobot(robotSensor)
	mainBot.Start()
}
