package main

// This example code demonstrates Lora RX/TX With SX127x driver
// You need to connect SPI, RST, CS, DIO0 (aka IRQ) and DIO1 to use.

import (
	"fmt"
	"machine"
	"time"

	"github.com/soypat/lora"
	"github.com/soypat/lora/sx127x"
)

var (
	loraRadio *sx127x.Device
	txmsg     = []byte("Hello TinyGO")

	// We assume LoRa module is connected to Raspberry Pi Pico:
	SX127X_PIN_RST = machine.GP16
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
	println("\n# TinyGo Lora RX/TX test\n# ----------------------")
	// machine.LED.Configure(machine.PinConfig{Mode: machine.PinOutput})
	SX127X_PIN_RST.Configure(machine.PinConfig{Mode: machine.PinOutput})
	SX127X_PIN_CS.Configure(machine.PinConfig{Mode: machine.PinOutput})
	err := SX127X_SPI.Configure(machine.SPIConfig{
		Frequency: 100000,
		SCK:       SX127X_PIN_SCK,
		SDO:       SX127X_PIN_TX,
		SDI:       SX127X_PIN_RX,
	})
	if err != nil {
		panic(err.Error())
	}
	dev := sx127x.NewDev(SX127X_SPI, SX127X_PIN_CS, SX127X_PIN_RST)
	loraConf.Freq = lora.MHz433_0
	err = dev.Init(loraConf)
	if err != nil {
		panic(err.Error())
	}
	println("loRa init success")
	cfg, cont, err := dev.ReadConfig()
	fmt.Printf("%v\n%+v err=%v\n\nwant=%+v\n", cont, cfg, err, loraConf)

	var buf [256]byte
	err = dev.SetOpMode(sx127x.OpRxSingle)
	if err != nil {
		panic(err.Error())
	}
	for {
		n, err := dev.ParsePacket(buf[:])
		if err != nil {
			println("no packet rx opmode:", dev.OpMode().String(), err.Error())
			time.Sleep(time.Second)
			continue
		}
		println("RXPACKET: ", string(buf[:n]))
	}
}
