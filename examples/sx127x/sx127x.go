package main

// This example code demonstrates Lora RX/TX With SX127x driver
// You need to connect SPI, RST, CS, DIO0 (aka IRQ) and DIO1 to use.

import (
	"errors"
	"fmt"
	"io"
	"machine"
	"time"

	"github.com/soypat/lora"
	"github.com/soypat/lora/sx127x"
)

const (
	LORA_DEFAULT_RXTIMEOUT_MS = 1000
	LORA_DEFAULT_TXTIMEOUT_MS = 5000
)

var (
	loraRadio *sx127x.Device
	txmsg     = []byte("Hello TinyGO")

	// We assume LoRa Featherwing module is connected to Raspberry Pi Pico:
	SX127X_PIN_RST  = machine.GP16
	SX127X_PIN_DIO0 = machine.GP6
	SX127X_PIN_DIO1 = machine.GP7
	// SPI definition for SX127x
	SX127X_SPI     = machine.SPI0
	SX127X_PIN_SCK = machine.GP2
	SX127X_PIN_TX  = machine.GP3
	SX127X_PIN_RX  = machine.GP4
	SX127X_PIN_CS  = machine.GP5
	LED            = machine.LED
	loraConf, _, _ = lora.CountryConfig("ar", false)
)

func main() {
	// LED.Configure(machine.PinConfig{Mode: machine.PinOutput})
	time.Sleep(3 * time.Second)
	println("\n# TinyGo Lora RX/TX test")
	println("# ----------------------")
	// machine.LED.Configure(machine.PinConfig{Mode: machine.PinOutput})
	SX127X_PIN_RST.Configure(machine.PinConfig{Mode: machine.PinOutput})
	SX127X_PIN_CS.Configure(machine.PinConfig{Mode: machine.PinOutput})
	err := SX127X_SPI.Configure(machine.SPIConfig{
		Frequency: 100000,
		Mode:      0,
		SCK:       SX127X_PIN_SCK,
		SDO:       SX127X_PIN_TX,
		SDI:       SX127X_PIN_RX,
	})
	if err != nil {
		panic(err)
	}

	println("main: create and start SX127x driver")
	dev := sx127x.NewDev(SX127X_SPI, SX127X_PIN_CS, SX127X_PIN_RST)
	dev.Reset()
	err = dev.CheckConnection()
	if err != nil {
		panic(err.Error())
	}
	println("main: sx127x found")
	loraConf.Freq = lora.MHz433_0
	cfg, cont, err := dev.ReadConfig()

	// Prepare for Lora Operation

	fmt.Println("time on air 10:", loraConf.TimeOnAir(10), " To 255:", loraConf.TimeOnAir(255))
	fmt.Printf("%v\n%+v err=%v\n\nwant=%+v\n", cont, cfg, err, loraConf)

	err = dev.Init(loraConf)
	if err != nil {
		panic(err.Error())
	}
	println("loRa init success")
	cfg, cont, err = dev.ReadConfig()
	fmt.Printf("%v\n%+v err=%v\n\nwant=%+v\n", cont, cfg, err, loraConf)
	myName, err := dev.RandomU32()
	if err != nil {
		println("random get fail:" + err.Error())
	}
	me := byte('A' + uint8(myName)%26)
	txBuf := append([]byte("Hello from "), me)
	println("I am ", string(me))
	delay := loraConf.TimeOnAir(len(txBuf))
	println("tx delay:", delay.String())
	cycle := true
	for {
		// LED.Set(cycle)
		cycle = !cycle
		for dev.CheckConnection() != nil {
			println("check SPI connection!")
			time.Sleep(time.Second)
		}
		print(" listening")
		err := dev.ListenAndDo(doRx5Seconds)
		if err != nil {
			println("listen error:", err.Error())
		}
		print(" transmitting")
		err = dev.TxPacket(txBuf)
		if err != nil {
			println("transmit error:", err.Error())
		}
		time.Sleep(delay)
		err = dev.SetOpMode(sx127x.OpStandby)
		if err != nil {
			println("transmit error:", err.Error())
		}
		time.Sleep(time.Second)
	}
}

func doRx5Seconds(d *sx127x.Dev) {
	var buf [255]byte
	for i := 0; i < 5; i++ {
		time.Sleep(time.Second)
		n, err := d.ParsePacket(buf[:])
		if n > 0 {
			println("got packet ", string(buf[:n]))
		} else {
			if errors.Is(err, io.EOF) {
				print(".")
			} else {
				println("Rx error:", err.Error())
			}
		}
	}
}
