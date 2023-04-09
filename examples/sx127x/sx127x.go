package main

// This example code demonstrates Lora RX/TX With SX127x driver
// You need to connect SPI, RST, CS, DIO0 (aka IRQ) and DIO1 to use.

import (
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
)

func dioIrqHandler(machine.Pin) {
	loraRadio.HandleInterrupt()
}

func main() {
	time.Sleep(5 * time.Second)
	println("\n# TinyGo Lora RX/TX test")
	println("# ----------------------")
	machine.LED.Configure(machine.PinConfig{Mode: machine.PinOutput})
	SX127X_PIN_RST.Configure(machine.PinConfig{Mode: machine.PinOutput})
	err := SX127X_SPI.Configure(machine.SPIConfig{
		Frequency: 500000,
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
	loraRadio.SetRadioController(sx127x.NewRadioControl(SX127X_PIN_CS, SX127X_PIN_DIO0, SX127X_PIN_DIO1))

	err = dev.CheckConnection()
	if err != nil {
		panic(err.Error())
	}
	println("main: sx127x found")

	// Prepare for Lora Operation
	loraConf := lora.Config{
		Freq:           lora.MHz868_1,
		Bandwidth:      lora.Bandwidth125_0,
		Spread:         lora.SpreadingFactor9,
		CodingRate:     lora.CodingRate4_7,
		HeaderType:     lora.HeaderExplicit,
		PreambleLength: 12,
		IQ:             lora.IQStandard,
		CRC:            lora.CRCOn,
		SyncWord:       lora.SyncPrivate,
		LoraTxPowerDBm: 20,
	}

	loraRadio.LoraConfig(loraConf)

	var count uint
	for {
		tStart := time.Now()

		println("main: Receiving Lora for 10 seconds")
		for time.Since(tStart) < 10*time.Second {
			buf, err := loraRadio.Rx(LORA_DEFAULT_RXTIMEOUT_MS)
			if err != nil {
				println("RX Error: ", err)
			} else if buf != nil {
				println("Packet Received: len=", len(buf), string(buf))
			}
		}
		println("main: End Lora RX")
		println("LORA TX size=", len(txmsg), " -> ", string(txmsg))
		err := loraRadio.Tx(txmsg, LORA_DEFAULT_TXTIMEOUT_MS)
		if err != nil {
			println("TX Error:", err)
		}
		count++
	}
}
