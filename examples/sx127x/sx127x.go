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

func main() {
	time.Sleep(3 * time.Second)
	println("\n# TinyGo Lora RX/TX test")
	println("# ----------------------")
	// machine.LED.Configure(machine.PinConfig{Mode: machine.PinOutput})
	SX127X_PIN_RST.Configure(machine.PinConfig{Mode: machine.PinOutput})
	SX127X_PIN_CS.Configure(machine.PinConfig{Mode: machine.PinOutput})
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
	err = dev.CheckConnection()
	if err != nil {
		panic(err.Error())
	}
	println("main: sx127x found")
	cfg, cont, err := dev.ReadConfig()

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
	fmt.Println("time on air 10:", loraConf.TimeOnAir(10), " ToA255:", loraConf.TimeOnAir(255))
	fmt.Printf("%v\n%+v err=%v\n\nwant=%+v\n", cont, cfg, err, loraConf)

	err = dev.Init(loraConf)
	if err != nil {
		panic(err.Error())
	}
	println("loRa init success")
	cfg, cont, err = dev.ReadConfig()
	fmt.Printf("%v\n%+v err=%v\n\nwant=%+v\n", cont, cfg, err, loraConf)
	txBuf := []byte("Hello LoRa!")
	delay := loraConf.TimeOnAir(len(txBuf))
	delayStr := delay.String()
	for {
		println("listening...")
		err := dev.ListenAndDo(doRx5Seconds)
		if err != nil {
			println("listen error:", err.Error())
		}
		println("transmitting packet")
		err = dev.TxPacket(txBuf)
		if err != nil {
			println("transmit error:", err.Error())
		}

		println("waiting for transmission end for ", delayStr)
		time.Sleep(delay)
		err = dev.SetOpMode(sx127x.OpStandby)
		if err != nil {
			println("transmit error:", err.Error())
		}
		println("finished Tx, now wait before loop")
		time.Sleep(time.Second)
	}
}

func doRx5Seconds(d *sx127x.Dev) {
	var buf [64]byte
	for i := 0; i < 5; i++ {
		time.Sleep(time.Second)
		n, err := d.RxPacket(buf[:])
		if n > 0 {
			println("got packet ", string(buf[:n]))
		} else {
			println("Rx error:", err.Error())
		}
	}
}
