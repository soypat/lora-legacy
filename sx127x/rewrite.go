package sx127x

import (
	"encoding/binary"
	"errors"
	"fmt"
	"io"
	"machine"
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
	err = d.sleep()
	if err != nil {
		return err
	}
	time.Sleep(15 * time.Millisecond)

	err = d.entryLoRa()
	if err != nil {
		return err
	}
	// d.SetHopPeriod(0)
	// d.SetLowFrequencyModeOn(false) // High freq mode.
	err = d.SetFrequency(cfg.Freq)
	if err != nil {
		return err
	}
	err = d.SetSyncWord(uint8(cfg.SyncWord))
	if err != nil {
		return err
	}
	err = d.SetBandwidth(cfg.Bandwidth)
	if err != nil {
		return err
	}
	err = d.SetSpreadingFactor(cfg.Spread)
	if err != nil {
		return err
	}
	err = d.SetIqMode(cfg.IQ)
	if err != nil {
		return err
	}
	err = d.SetCodingRate(cfg.CodingRate)
	if err != nil {
		return err
	}
	err = d.EnableCRC(cfg.CRC != 0)
	if err != nil {
		return err
	}
	err = d.SetTxPower(cfg.LoraTxPowerDBm)
	if err != nil {
		return err
	}
	err = d.SetHeaderType(cfg.HeaderType)
	if err != nil {
		return err
	}
	err = d.SetAutoAGC(SX127X_AGC_AUTO_ON)
	if err != nil {
		return err
	}
	err = d.setRxTimeout(8)
	if err != nil {
		return err
	}
	err = d.SetPreambleLength(cfg.PreambleLength)
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
	d.SetHopPeriod(0)
	d.SetLowFrequencyModeOn(false)
	// set PA ramp-up time 50 uSec
	reg, err := d.Read8(SX127X_REG_PA_RAMP)
	if err != nil {
		return err
	}
	d.Write8(SX127X_REG_PA_RAMP, (reg&0xF0)|0x08)
	// Set Low Noise Amplifier to MAX
	d.Write8(SX127X_REG_LNA, SX127X_LNA_MAX_GAIN)
	// Wake back up or Tx and Rx will not work.
	err = d.SetOpMode(SX127X_OPMODE_STANDBY)
	return err
}

func (d *Dev) TxPacket(pkt []byte) (err error) {
	if len(pkt) > 255 {
		return errors.New("packet too large to transmit. limit is 255")
	}
	if d.IsTransmitting() {
		return errors.New("is currently transmitting packet")
	}
	err = d.clrLoRaIrq()
	if err != nil {
		return err
	}
	// initialize the payload size and address pointers
	d.Write8(SX127X_REG_PAYLOAD_LENGTH, uint8(len(pkt)))
	d.Write8(SX127X_REG_FIFO_TX_BASE_ADDR, 0)
	d.Write8(SX127X_REG_FIFO_ADDR_PTR, 0)
	err = d.write(SX127X_REG_FIFO, pkt)
	if err != nil {
		return err
	}
	err = d.SetOpMode(SX127X_OPMODE_TX)
	if err != nil {
		return err
	}
	return nil
}

func (d *Dev) RxPacket(buf []byte) (int, error) {
	switch {
	case d.prevFreq == 0:
		return 0, errors.New("frequency not set- was device initialized?")
	case d.dio0 == machine.NoPin:
		return 0, errors.New("no DIO0 pin set")
	case d.IsTransmitting():
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

	return 0, nil
}

// Listen listens for Rx activity on the LoRa network. Do not call other functions
// that change the opmode during the duration of the callback to prevent device
// from malfunctioning or failing to listen to a packet. After the callback
// finishes execution the device's state is returned to standby.
func (d *Dev) ListenAndDo(fn func()) error {
	if fn == nil {
		return errors.New("nil callback")
	}
	err := d.SetOpMode(SX127X_OPMODE_RX)
	if err != nil {
		return err
	}
	fn()
	return d.SetOpMode(SX127X_OPMODE_STANDBY)
}

// SetCrc Enable CRC generation and check on payload.
func (d *Dev) EnableCRC(enable bool) error {
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

// SetTxPower sets the transmitter output (with paBoost ON)
func (d *Dev) SetTxPower(txPower int8) error {
	return d.setTxPowerWithPaBoost(txPower, true)
}

// SetIQMode Sets I/Q polarity configuration
func (d *Dev) SetIqMode(val uint8) (err error) {
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
		err = d.SetSyncWord(SX127X_LORA_MAC_PUBLIC_SYNCWORD)
	} else {
		err = d.SetSyncWord(SX127X_LORA_MAC_PRIVATE_SYNCWORD)
	}
	return err
}

// SetSyncWord defines the 8bit sync word.
func (d *Dev) SetSyncWord(syncWord uint8) error { return d.Write8(SX127X_REG_SYNC_WORD, syncWord) }

// SetFrequency sets the LoRa frequency parameter.
func (d *Dev) SetFrequency(freq uint32) error {
	d.prevFreq = freq
	var freqReg [3]byte
	f64 := uint64(freq<<19) / 32000000
	freqReg[0] = byte(f64 >> 16)
	freqReg[1] = byte(f64 >> 8)
	freqReg[2] = byte(f64 >> 0)
	return d.write(SX127X_REG_FRF_MSB, freqReg[:])
}

// SetBandwidth updates the bandwidth the LoRa module is using
func (d *Dev) SetBandwidth(bw uint8) error {
	val, err := d.Read8(SX127X_REG_MODEM_CONFIG_1)
	if err != nil {
		return err
	}
	return d.Write8(SX127X_REG_MODEM_CONFIG_1, (val&0x0f)|(bw<<4))
}

// SetCodingRate updates the coding rate the LoRa module is using.
func (d *Dev) SetCodingRate(cr uint8) error {
	val, err := d.Read8(SX127X_REG_MODEM_CONFIG_1)
	if err != nil {
		return err
	}
	return d.Write8(SX127X_REG_MODEM_CONFIG_1, (0xf1&val)|(cr<<1))
}

// SetAutoAGC enables Automatic Gain Control.
func (d *Dev) SetAutoAGC(val uint8) error {
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

// SetLowFrequencyModeOn enables Low Data Rate Optimization
func (d *Dev) SetLowFrequencyModeOn(val bool) (err error) {
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
	cfg.CRC = (cfg2 | 0x4) >> 2
	continuousMode = cfg2|0x8 != 0
	cfg.PreambleLength = binary.BigEndian.Uint16(buf[3:])

	err = d.read(SX127X_REG_FRF_MSB, buf[:3])
	if err != nil {
		return cfg, continuousMode, err
	}
	freq := uint64(buf[0]<<16) | uint64(buf[1]<<8) | uint64(buf[2])
	freq = (freq >> 19) * 320000
	cfg.Freq = uint32(freq)
	// freq := uint64(fHz<<19) / 32000000
	// freqReg[0] = byte(freq >> 16)
	// freqReg[1] = byte(freq >> 8)
	// freqReg[2] = byte(freq >> 0)
	return cfg, continuousMode, nil
}

// SetHeaderType set implicit or explicit mode.
func (d *Dev) SetHeaderType(headerType uint8) error {
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

// SetHopPeriod sets number of symbol periods between frequency hops. (0 = disabled).
func (d *Dev) SetHopPeriod(val uint8) error { return d.Write8(SX127X_REG_HOP_PERIOD, val) }

// SetSpreadingFactor changes spreading factor.
func (d *Dev) SetSpreadingFactor(sf uint8) error {
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
		err = d.SetOCP(ocp)
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

// SetOCP defines Overload Current Protection configuration
func (d *Dev) SetOCP(mA uint8) error {
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

// SetPreambleLength defines number of preamble
func (d *Dev) SetPreambleLength(pLen uint16) error {
	var buf [2]byte
	binary.BigEndian.PutUint16(buf[:], pLen)
	return d.write(SX127X_REG_PREAMBLE_MSB, buf[:])
}

// IsTransmitting tests if a packet transmission is in progress.
// If the SPI transaction fails then it returns false.
func (d *Dev) IsTransmitting() bool {
	val, err := d.Read8(SX127X_REG_OP_MODE)
	if err != nil {
		return false
	}
	return (val & SX127X_OPMODE_TX) == SX127X_OPMODE_TX
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
	err = d.SetOpMode(SX127X_OPMODE_SLEEP)
	if err != nil {
		return 0, err
	}
	err = d.SetFrequency(d.prevFreq)
	if err != nil {
		return 0, err
	}
	err = d.SetOpMode(SX127X_OPMODE_RX)
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
func (d *Dev) clrLoRaIrq() error { return d.Write8(SX127X_REG_IRQ_FLAGS, 0xff) }
func (d *Dev) sleep() error      { return d.Write8(SX127X_REG_OP_MODE, 0x08) }
func (d *Dev) standby() error    { return d.Write8(SX127X_REG_OP_MODE, 0x09) }
func (d *Dev) entryLoRa() error  { return d.Write8(SX127X_REG_OP_MODE, 0x88) }

// SetOpMode changes the sx1276 mode
func (d *Dev) SetOpMode(mode uint8) error {
	cur, err := d.Read8(SX127X_REG_OP_MODE)
	if err != nil {
		return err
	}
	new := (cur & (^SX127X_OPMODE_MASK)) | mode
	return d.Write8(SX127X_REG_OP_MODE, new)
}

// Reset re-initialize the sx127x device
func (d *Dev) Reset() {
	d.rst.Configure(machine.PinConfig{Mode: machine.PinOutput})
	d.rst.Low()
	time.Sleep(100 * time.Millisecond)
	d.rst.High()
	time.Sleep(100 * time.Millisecond)
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

func (d *Dev) write(addr uint8, buf []byte) error {
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
	var buf [3]byte
	buf[0] = addr & 0x7f
	err := d.spi.Tx(buf[:2], buf[2:])
	return buf[2], err
}

//go:inline
func (d *Dev) enable(b bool) {
	d.nss.Set(!b)
}

func min[T ~int | ~uint8](a, b T) T {
	if a < b {
		return a
	}
	return b
}
