package sx127x

import (
	"encoding/binary"
	"errors"
	"fmt"
	"io"
	"machine"
	"runtime"
	"time"

	"github.com/soypat/lora"
	"tinygo.org/x/drivers"
)

type Dev struct {
	spi      drivers.SPI
	nss      machine.Pin
	rst      machine.Pin
	dio0     machine.Pin
	prevFreq uint32
	spreadF  uint8
}

func NewDev(spi drivers.SPI, nssOrCS, reset machine.Pin) *Dev {
	d := &Dev{
		nss:  nssOrCS,
		spi:  spi,
		rst:  reset,
		dio0: machine.NoPin,
	}
	nssOrCS.Configure(machine.PinConfig{Mode: machine.PinOutput})
	nssOrCS.High() // Make sure CS is set to high or first transaction will fail.
	return d
}

func (d *Dev) Init(cfg lora.Config) (err error) {
	d.Reset()
	err = d.CheckConnection()
	if err != nil {
		return err
	}
	err = d.sleep()
	if err != nil {
		return err
	}
	err = d.setFrequency(cfg.Freq)
	if err != nil {
		return err
	}

	// Set Base addresses.
	d.Write8(SX127X_REG_FIFO_TX_BASE_ADDR, 0)
	d.Write8(SX127X_REG_FIFO_ADDR_PTR, 0)
	// Set Low noise amplifier to max gain setting.
	err = d.Write8(SX127X_REG_LNA, SX127X_LNA_MAX_GAIN)
	if err != nil {
		return err
	}
	err = d.setHeaderType(cfg.HeaderType)
	if err != nil {
		return err
	}
	err = d.setAutoAGC(SX127X_AGC_AUTO_ON)
	if err != nil {
		return err
	}
	err = d.setTxPower(17)
	if err != nil {
		return err
	}
	// Set to standby
	return d.idle()
}

func (d *Dev) sleep() error { return d.SetOpMode(opLoRaBit | OpSleep) }
func (d *Dev) idle() error  { return d.SetOpMode(opLoRaBit | OpStandby) }

func (d *Dev) TxPacket(packet []byte) (err error) {
	if len(packet) > 255 {
		return errors.New("packet too large to broadcast")
	}
	err = d.idle()
	if err != nil {
		return err
	}
	d.Write8(SX127X_REG_FIFO_ADDR_PTR, 0)
	d.Write8(SX127X_REG_PAYLOAD_LENGTH, 0)

	// Begin writing data.
	currentLength, err := d.Read8(SX127X_REG_PAYLOAD_LENGTH)
	if err != nil {
		return err
	}
	if currentLength != 0 {
		return errors.New("payload length not set to zero")
	}
	for i := 0; i < len(packet); i++ {
		err := d.Write8(SX127X_REG_FIFO, packet[i])
		if err != nil {
			return err
		}
	}
	err = d.Write8(SX127X_REG_PAYLOAD_LENGTH, uint8(len(packet)))
	if err != nil {
		return err
	}

	err = d.SetOpMode(opLoRaBit | OpTx)
	startTx := time.Now()
	if err != nil {
		return err
	}
	counts := 0
	var reg uint8
	for {
		counts++
		reg, err = d.Read8(SX127X_REG_IRQ_FLAGS)
		if reg&SX127X_IRQ_LORA_TXDONE_MASK != 0 || err != nil {
			if err != nil {
				return err
			}
			break
		}
		runtime.Gosched() // Yield to scheduler.
	}
	println("tx done in:", time.Since(startTx).String(), counts)
	d.idle()
	return d.Write8(SX127X_REG_IRQ_FLAGS, SX127X_IRQ_LORA_TXDONE_MASK)
}

// Init calls device initialization functions without resetting it.
func (d *Dev) _init_MUSTFIX(cfg lora.Config) (err error) {
	cfg.LoraTxPowerDBm = min(cfg.LoraTxPowerDBm, 20) // Set Max to 20dBm.
	err = d.SetOpMode(OpSleep)
	if err != nil {
		return err
	}

	err = d.entryLoRa()
	if err != nil {
		return err
	}
	// d.SetHopPeriod(0)
	// d.SetLowFrequencyModeOn(false) // High freq mode.
	err = d.setFrequency(cfg.Freq)
	if err != nil {
		return err
	}
	err = d.setSyncWord(uint8(cfg.SyncWord))
	if err != nil {
		return err
	}
	err = d.setBandwidth(cfg.Bandwidth)
	if err != nil {
		return err
	}
	err = d.setSpreadingFactor(cfg.Spread)
	if err != nil {
		return err
	}
	err = d.setIqMode(cfg.IQ)
	if err != nil {
		return err
	}
	err = d.setCodingRate(cfg.CodingRate)
	if err != nil {
		return err
	}
	err = d.enableCRC(cfg.CRC != 0)
	if err != nil {
		return err
	}
	err = d.setTxPower(cfg.LoraTxPowerDBm)
	if err != nil {
		return err
	}
	err = d.setHeaderType(cfg.HeaderType)
	if err != nil {
		return err
	}
	err = d.setAutoAGC(SX127X_AGC_AUTO_ON)
	if err != nil {
		return err
	}
	err = d.setRxTimeout(8)
	if err != nil {
		return err
	}
	err = d.setPreambleLength(cfg.PreambleLength)
	if err != nil {
		return err
	}
	// set the IRQ mapping DIO0=TxDone DIO1=NOP DIO2=NOP
	d.Write8(SX127X_REG_DIO_MAPPING_1, SX127X_MAP_DIO0_LORA_TXDONE|SX127X_MAP_DIO1_LORA_NOP|SX127X_MAP_DIO2_LORA_NOP)
	// Clear all radio IRQ Flags.
	d.Write8(SX127X_REG_IRQ_FLAGS, 0xFF)
	// Mask all but TxDone.
	err = d.Write8(SX127X_REG_IRQ_FLAGS_MASK, ^SX127X_IRQ_LORA_TXDONE_MASK)
	if err != nil {
		return err
	}
	d.setHopPeriod(0)
	d.setLowFrequencyModeOn(false)
	// set PA ramp-up time 50 uSec
	reg, err := d.Read8(SX127X_REG_PA_RAMP)
	if err != nil {
		return err
	}
	d.Write8(SX127X_REG_PA_RAMP, (reg&0xF0)|0x08)
	// Set Low Noise Amplifier to MAX
	d.Write8(SX127X_REG_LNA, SX127X_LNA_MAX_GAIN)
	// Wake back up or Tx and Rx will not work.
	err = d.SetOpMode(OpStandby)
	return err
}

func (d *Dev) ParsePacket(buf []byte) (uint8, error) {
	if len(buf) < 255 {
		return 0, errors.New("please pass in a 255 or larger sized buffer")
	}
	startOpmode := d.OpMode()

	cfg, _, _ := d.ReadConfig()
	if cfg.HeaderType == lora.HeaderImplicit {
		return 0, errors.New("implicit header not implemented")
		// d.Write8(SX127X_REG_PAYLOAD_LENGTH)
	}
	irqflags, err := d.Read8(SX127X_REG_IRQ_FLAGS)
	if err != nil {
		return 0, err
	}
	err = d.clrLoRaIrq()
	if err != nil {
		return 0, err
	}
	if irqflags&SX127X_IRQ_LORA_RXDONE_MASK != 0 {
		return 0, errors.New("no packet in fifo")
	}
	if irqflags&SX127X_IRQ_LORA_CRCERR_MASK == 0 {
		return 0, errors.New("crc error")
	}
	// Succesfully received packet.
	plen, err := d.Read8(SX127X_REG_RX_NB_BYTES) // For EXPLICIT header mode.
	if err != nil {
		return 0, err
	}
	curraddr, err := d.Read8(SX127X_REG_FIFO_RX_CURRENT_ADDR)
	if err != nil {
		return 0, err
	}
	err = d.Write8(SX127X_REG_FIFO_ADDR_PTR, curraddr)
	if err != nil {
		return 0, err
	}
	d.idle() // Ready to read. TODO(soypat): Move this before Reading lengths to prevent packet read during initial processing.
	time.Sleep(20 * time.Millisecond)
	for i := uint8(0); i < plen; i++ {
		buf[i], err = d.Read8(SX127X_REG_FIFO)
		if err != nil {
			return i, err
		}
	}
	if startOpmode == OpRx || startOpmode == OpRxSingle {
		d.SetOpMode(opLoRaBit | startOpmode) // Revert to previous opmode.
	}
	return plen, nil
}

func (d *Dev) RxPacket(buf []byte) (int, error) {
	switch {
	case d.prevFreq == 0:
		return 0, errors.New("frequency not set- was device initialized?")
	// case d.dio0 == machine.NoPin:
	// 	return 0, errors.New("no DIO0 pin set")
	case d.OpMode() == OpTx:
		return 0, errors.New("currently transmitting")
		// case d.dio0.Get() == false:
		// 	return 0, io.EOF// Not ok to receive?
	}
	pktSize, err := d.Read8(SX127X_REG_RX_NB_BYTES) // Number for received bytes.
	if err != nil {
		return 0, err
	}
	if pktSize == 0 {
		return 0, io.EOF // No packets.
	}
	pktAddr, err := d.Read8(SX127X_REG_FIFO_RX_CURRENT_ADDR)
	if err != nil {
		return 0, err
	}
	err = d.Write8(SX127X_REG_FIFO_ADDR_PTR, pktAddr) // Set BaseAddr -> FIFO_ADDR_PTR
	if err != nil {
		return 0, err
	}

	n := min(int(pktSize), len(buf))
	err = d.read(SX127X_REG_FIFO, buf[:n])
	return n, err
	// Single RX mode don't properly handle Timeouts on sx127x, so we use Continuous RX
	// Go routine is a workaround to stop the Continuous RX and fire a timeout Event
}

// Listen listens for Rx activity on the LoRa network. Do not call other functions
// that change the opmode during the duration of the callback to prevent device
// from malfunctioning or failing to listen to a packet. After the callback
// finishes execution the device's state is returned to standby.
func (d *Dev) ListenAndDo(fn func(*Dev)) error {
	if fn == nil {
		return errors.New("nil callback")
	}
	err := d.SetOpMode(OpRx)
	if err != nil {
		return err
	}
	fn(d)
	return d.SetOpMode(OpStandby)
}

// SetCrc Enable CRC generation and check on payload.
func (d *Dev) enableCRC(enable bool) error {
	reg, err := d.Read8(SX127X_REG_MODEM_CONFIG_2)
	if err != nil {
		return err
	}
	if enable {
		err = d.Write8(SX127X_REG_MODEM_CONFIG_2, reg|0x04)
	} else {
		err = d.Write8(SX127X_REG_MODEM_CONFIG_2, reg&0xfb)
	}
	return err
}

// setTxPower sets the transmitter output (with paBoost ON)
func (d *Dev) setTxPower(txPower int8) error {
	return d.setTxPowerWithPaBoost(txPower, true)
}

// SetIQMode Sets I/Q polarity configuration
func (d *Dev) setIqMode(val uint8) (err error) {
	//Default IQ is Standard normal values.
	var (
		iq1 uint8 = 0x27
		iq2 uint8 = 0x1d
	)
	if val != lora.IQStandard {
		// Invert IQ Back.
		iq1 = 0x66
		iq2 = 0x19
	}
	err = d.Write8(SX127X_REG_INVERTIQ, iq1)
	if err != nil {
		return err
	}
	return d.Write8(SX127X_REG_INVERTIQ2, iq2)
}

// SetPublicNetwork changes Sync Word to match network type.
func (d *Dev) SetPublicNetwork(enabled bool) (err error) {
	if enabled {
		err = d.setSyncWord(SX127X_LORA_MAC_PUBLIC_SYNCWORD)
	} else {
		err = d.setSyncWord(SX127X_LORA_MAC_PRIVATE_SYNCWORD)
	}
	return err
}

// setSyncWord defines the 8bit sync word.
func (d *Dev) setSyncWord(syncWord uint8) error { return d.Write8(SX127X_REG_SYNC_WORD, syncWord) }

// setFrequency sets the LoRa frequency parameter.
func (d *Dev) setFrequency(freq uint32) error {
	d.prevFreq = freq
	var freqReg [3]byte
	f64 := (uint64(freq) << 8) / 15625
	freqReg[0] = byte(f64 >> 16)
	freqReg[1] = byte(f64 >> 8)
	freqReg[2] = byte(f64 >> 0)
	d.Write8(SX127X_REG_FRF_MSB, freqReg[0])
	d.Write8(SX127X_REG_FRF_MID, freqReg[1])
	return d.Write8(SX127X_REG_FRF_LSB, freqReg[2])
}

// setBandwidth updates the bandwidth the LoRa module is using
func (d *Dev) setBandwidth(bw uint8) error {
	val, err := d.Read8(SX127X_REG_MODEM_CONFIG_1)
	if err != nil {
		return err
	}
	return d.Write8(SX127X_REG_MODEM_CONFIG_1, (val&0x0f)|(bw<<4))
}

// setCodingRate updates the coding rate the LoRa module is using.
func (d *Dev) setCodingRate(cr uint8) error {
	val, err := d.Read8(SX127X_REG_MODEM_CONFIG_1)
	if err != nil {
		return err
	}
	return d.Write8(SX127X_REG_MODEM_CONFIG_1, (0xf1&val)|(cr<<1))
}

// setAutoAGC enables Automatic Gain Control.
func (d *Dev) setAutoAGC(val uint8) error {
	reg, err := d.Read8(SX127X_REG_MODEM_CONFIG_3)
	if err != nil {
		return err
	}
	if val == SX127X_AGC_AUTO_ON {
		err = d.Write8(SX127X_REG_MODEM_CONFIG_3, reg|0x04)
	} else {
		err = d.Write8(SX127X_REG_MODEM_CONFIG_3, reg&0xfb)
	}
	return err
}

// setLowFrequencyModeOn enables Low Data Rate Optimization
func (d *Dev) setLowFrequencyModeOn(val bool) (err error) {
	reg, err := d.Read8(SX127X_REG_OP_MODE)
	if err != nil {
		return err
	}
	if val {
		err = d.Write8(SX127X_REG_OP_MODE, reg|0x04)
	} else {
		err = d.Write8(SX127X_REG_OP_MODE, reg&0xfb)
	}
	return err
}

// ReadConfig reads the configuration parameters from the device and returns
// the corresponding lora.Config for the current device configuration.
// Some lora.Config parameters are not set such as IQ, LDR, and Tx power.
func (d *Dev) ReadConfig() (cfg lora.Config, continuousMode bool, err error) {
	var buf [5]byte
	err = d.read(SX127X_REG_MODEM_CONFIG_1, buf[:5])
	if err != nil {
		return cfg, continuousMode, err
	}
	cfg1 := buf[0]
	cfg.Bandwidth = cfg1 >> 4
	cfg.CodingRate = (cfg1 >> 1) & 0b111
	cfg.HeaderType = cfg1 & 1
	cfg2 := buf[1]
	cfg.Spread = cfg2 >> 4
	cfg.CRC = (cfg2 & 0x4) >> 2
	continuousMode = cfg2&0x8 != 0
	cfg.PreambleLength = binary.BigEndian.Uint16(buf[3:])
	// Read sync word.
	sync, err := d.Read8(SX127X_REG_SYNC_WORD)
	if err != nil {
		return cfg, continuousMode, err
	}
	cfg.SyncWord = uint16(sync)
	// Read Frequency.
	err = d.read(SX127X_REG_FRF_MSB, buf[:3])
	freq := uint64(buf[0])<<16 | uint64(buf[1])<<8 | uint64(buf[2])
	cfg.Freq = uint32((freq * 15625) >> 8)
	return cfg, continuousMode, nil
}

// setHeaderType set implicit or explicit mode.
func (d *Dev) setHeaderType(headerType uint8) error {
	val, err := d.Read8(SX127X_REG_MODEM_CONFIG_1)
	if err != nil {
		return err
	} else if headerType == lora.HeaderImplicit {
		val |= 0x01
	} else {
		val &= 0xfe
	}
	return d.Write8(SX127X_REG_MODEM_CONFIG_1, val)
}

// setHopPeriod sets number of symbol periods between frequency hops. (0 = disabled).
func (d *Dev) setHopPeriod(val uint8) error { return d.Write8(SX127X_REG_HOP_PERIOD, val) }

// setSpreadingFactor changes spreading factor.
func (d *Dev) setSpreadingFactor(sf uint8) error {
	d.spreadF = sf
	// Declare values for spread factor of 6.
	var (
		opt        uint8 = 0xc5
		threshhold uint8 = 0x0c
	)
	if sf != lora.SpreadingFactor6 {
		opt = 0xc3
		threshhold = 0x0a
	}
	err := d.Write8(SX127X_REG_DETECTION_OPTIMIZE, opt)
	if err != nil {
		return err
	}
	err = d.Write8(SX127X_REG_DETECTION_THRESHOLD, threshhold)
	if err != nil {
		return err
	}
	cfg, err := d.Read8(SX127X_REG_MODEM_CONFIG_2)
	if err != nil {
		return err
	}
	return d.Write8(SX127X_REG_MODEM_CONFIG_2, cfg&0x0f|((sf<<4)&0xf0))
}

// SetRxTimeout defines RX Timeout expressed as number of symbols
// Default timeout is 64 * Ts
func (d *Dev) setRxTimeout(tmoutSymb uint8) error {
	return d.Write8(SX127X_REG_SYMB_TIMEOUT_LSB, tmoutSymb)
}

// SetTxPowerWithPaBoost sets the transmitter output power and may activate paBoost.
func (d *Dev) setTxPowerWithPaBoost(txPower int8, paBoost bool) (err error) {
	switch paBoost {
	case true:
		var (
			dac uint8 = 0x87
			ocp uint8 = 140
		)
		if txPower > 17 {
			// High Power +20 dBm Operation (Semtech SX1276/77/78/79 5.4.3.).
			if txPower > 20 {
				txPower = 20
			}
			txPower -= 3
		} else if txPower < 2 {
			ocp = 100
			dac = 0x84
			txPower = 2
		}

		err = d.Write8(SX127X_REG_PA_DAC, dac)
		if err != nil {
			break
		}
		err = d.setOCP(ocp)
		if err != nil {
			break
		}
		err = d.Write8(SX127X_REG_PA_CONFIG, SX127X_PA_BOOST|uint8(txPower-2))

	case false:
		// RFO
		if txPower < 0 {
			txPower = 0
		} else if txPower > 14 {
			txPower = 14
		}
		err = d.Write8(SX127X_REG_PA_CONFIG, 0x70|uint8(txPower))
	}
	return err
}

// setOCP defines Overload Current Protection configuration.
func (d *Dev) setOCP(mA uint8) error {
	ocpTrim := uint8(27)
	if mA < 45 {
		mA = 45
	}
	if mA <= 120 {
		ocpTrim = (mA - 45) / 5
	} else if mA <= 240 {
		ocpTrim = (mA + 30) / 10
	}
	return d.Write8(SX127X_REG_OCP, 0x20|(0x1F&ocpTrim))
}

// setPreambleLength defines number of preamble
func (d *Dev) setPreambleLength(pLen uint16) error {
	var buf [2]byte
	binary.BigEndian.PutUint16(buf[:], pLen)
	d.Write8(SX127X_REG_PREAMBLE_MSB, buf[0])
	return d.Write8(SX127X_REG_PREAMBLE_LSB, buf[1])
}

func (d *Dev) OpMode() OpMode {
	val, err := d.Read8(SX127X_REG_OP_MODE)
	if err != nil {
		return 255 // Out of band error.
	}
	return OpMode(val & SX127X_OPMODE_MASK)
}

// LastPacketRSSI gives the RSSI of the last packet received. If SPI transaction fails this returns 0.
func (d *Dev) LastPacketRSSI() uint8 {
	// section 5.5.5
	var adjustValue uint8 = 157
	if d.prevFreq < 868000000 {
		adjustValue = 164
	}
	val, err := d.Read8(SX127X_REG_PKT_RSSI_VALUE)
	if err != nil {
		return 0
	}
	return val - adjustValue
}

// PrintRegisters outputs the sx127x transceiver registers.
func (d *Dev) RandomU32() (rnd uint32, err error) {
	// Disable ALL irqs
	err = d.Write8(SX127X_REG_IRQ_FLAGS, 0xff)
	if err != nil {
		return 0, err
	}
	err = d.Write8(SX127X_REG_OP_MODE, SX127X_OPMODE_LORA) // LoRa Opmode.
	if err != nil {
		return 0, err
	}
	err = d.SetOpMode(OpSleep)
	if err != nil {
		return 0, err
	}
	err = d.setFrequency(d.prevFreq)
	if err != nil {
		return 0, err
	}
	err = d.SetOpMode(OpRx)
	if err != nil {
		return 0, err
	}
	for i := 0; i < 32; i++ {
		time.Sleep(time.Millisecond * 10)
		val, err := d.Read8(SX127X_REG_RSSI_WIDEBAND)
		if err != nil {
			return rnd, err
		}
		// Unfiltered RSSI value reading. Only takes the LSB value
		rnd |= (uint32(val) & 0x01) << i
	}
	return rnd, nil
}

func (d *Dev) clrLoRaIrq() error {
	flags, _ := d.Read8(SX127X_REG_IRQ_FLAGS)
	return d.Write8(SX127X_REG_IRQ_FLAGS, flags)
}

// func (d *Dev) sleep() error      { return d.Write8(SX127X_REG_OP_MODE, 0x08) }
func (d *Dev) standby() error   { return d.Write8(SX127X_REG_OP_MODE, 0x09) }
func (d *Dev) entryLoRa() error { return d.Write8(SX127X_REG_OP_MODE, 0x88) }

// SetOpMode changes the sx1276 mode. This function can halt a receive or transmit operation.
func (d *Dev) SetOpMode(mode OpMode) error {
	cur, err := d.Read8(SX127X_REG_OP_MODE)
	if err != nil {
		return err
	}
	new := (cur & (^SX127X_OPMODE_MASK)) | uint8(mode) | uint8(opLoRaBit)
	err = d.Write8(SX127X_REG_OP_MODE, new)
	if mode == OpSleep && err == nil {
		time.Sleep(15 * time.Millisecond) // Wait for wake.
	}
	return err
}

// Reset re-initialize the sx127x device
func (d *Dev) Reset() {
	d.rst.Configure(machine.PinConfig{Mode: machine.PinOutput})
	d.rst.High()
	time.Sleep(200 * time.Millisecond)
	d.rst.Low()
	time.Sleep(200 * time.Millisecond)
	d.rst.High()
	time.Sleep(50 * time.Millisecond)
}

var ErrInvalidID = errors.New("sx1278: invalid device id or faulty SPI connection")

// CheckConnection validates the device is connected and SPI is working.
func (d *Dev) CheckConnection() error {
	id, err := d.Read8(SX127X_REG_VERSION)
	if err != nil {
		return fmt.Errorf("failed to read from SPI: %w", err)
	}
	if id == 0 {
		return errors.New("sx1278: faulty SPI connection. is device hooked up?")
	}
	if id != 0x12 {
		return ErrInvalidID
	}
	return nil
}

// SetRxDoneOnDIO0 sets the RxDone interrupt on the DIO0 pin of the device.
// If withTimeoutOnDIO1 is true then a timeout IRQ is also set on DIO1.
func (d *Dev) SetIRQRxDoneOnDIO0(withTimeoutOnDIO1 bool) (err error) {
	irq := SX127X_MAP_DIO0_LORA_RXDONE
	if withTimeoutOnDIO1 {
		// set the IRQ mapping DIO0=RxDone DIO1=RxTimeout DIO2=NOP
		irq |= SX127X_MAP_DIO1_LORA_RXTOUT
	} else {
		irq |= SX127X_MAP_DIO1_LORA_NOP
	}
	err = d.Write8(SX127X_REG_DIO_MAPPING_1, irq|SX127X_MAP_DIO2_LORA_NOP)
	if err != nil {
		return err
	}
	err = d.clrLoRaIrq() // Clear all radio IRQ Flags to avoid instant interrupt raise.
	if err != nil {
		return err
	}
	// Mask all but RxDone
	return d.Write8(SX127X_REG_IRQ_FLAGS_MASK, ^irq)
}

// SetRxDoneOnDIO0 sets the TxDone interrupt on the DIO0 pin of the device.
func (d *Dev) SetIRQTxDoneOnDIO0() (err error) {
	// set the IRQ mapping DIO0=TxDone DIO1=NOP DIO2=NOP.
	err = d.Write8(SX127X_REG_DIO_MAPPING_1, SX127X_MAP_DIO0_LORA_TXDONE|SX127X_MAP_DIO1_LORA_NOP|SX127X_MAP_DIO2_LORA_NOP)
	if err != nil {
		return err
	}
	err = d.clrLoRaIrq() // Clear all radio IRQ Flags to avoid instant interrupt raise.
	if err != nil {
		return err
	}
	// Mask all but TxDone.
	return d.Write8(SX127X_REG_IRQ_FLAGS_MASK, ^SX127X_IRQ_LORA_TXDONE_MASK)
}

// IO

func (d *Dev) read(addr uint8, buf []byte) error {
	if len(buf) <= 1 {
		return io.ErrShortBuffer
	}
	d.enable(true)
	_, err := d.spi.Transfer(addr)
	if err != nil {
		d.enable(false)
		return err
	}
	err = d.spi.Tx(nil, buf)
	d.enable(false)
	return err
}

// This function does not work for some reason??
func (d *Dev) _DONOTUSE_write(addr uint8, buf []byte) error {
	if len(buf) <= 1 {
		return io.ErrShortBuffer
	}
	d.enable(true)
	_, err := d.spi.Transfer(addr)
	if err != nil {
		d.enable(false)
		return err
	}
	err = d.spi.Tx(buf, nil)
	d.enable(false)
	return err
}

func (d *Dev) Write8(addr, value uint8) error {
	d.enable(true)
	_, err := d.w8(addr, value)
	d.enable(false)
	return err
}

// WriteRegister writes value to register
func (d *Dev) w8(addr uint8, value uint8) (uint8, error) {
	d.enable(true)
	defer d.enable(false)
	var buf [4]byte
	buf[0] = addr | 0x80
	buf[1] = value
	err := d.spi.Tx(buf[:2], buf[2:])
	return buf[3], err
}

// ReadRegister reads value from register.
func (d *Dev) Read8(addr uint8) (uint8, error) {
	d.enable(true)
	val, err := d.r8(addr)
	d.enable(false)
	return val, err
}

func (d *Dev) r8(addr uint8) (uint8, error) {
	var buf [4]byte
	buf[0] = addr & 0x7f
	err := d.spi.Tx(buf[:2], buf[2:])
	return buf[3], err
}

//go:inline
func (d *Dev) enable(b bool) {
	d.nss.Set(!b)
}

func min[T ~int | ~int64 | ~uint8 | ~int8](a, b T) T {
	if a < b {
		return a
	}
	return b
}

func max[T ~int | ~int64 | ~uint8](a, b T) T {
	if a > b {
		return a
	}
	return b
}
