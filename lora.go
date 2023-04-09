package lora

import "errors"

// Config holds the LoRa configuration parameters.
type Config struct {
	Freq           uint32 // Frequency [Hz]
	CodingRate     uint8  // Coding Rate
	Spread         uint8  // Spread Factor
	Bandwidth      uint8  // Bandwidth
	LDR            uint8  // Low Data Rate
	PreambleLength uint16 // PreambleLength
	SyncWord       uint16 // Sync Word
	HeaderType     uint8  // Header : Implicit/explicit
	CRC            uint8  // CRC : Yes/No
	IQ             uint8  // iq : Standard/inverted
	LoraTxPowerDBm int8   // Tx power in Dbm
}

var (
	ErrUndefinedLoraConf = errors.New("Undefined Lora configuration")
)

const (
	SpreadingFactor5  = 5
	SpreadingFactor6  = 6
	SpreadingFactor7  = 7
	SpreadingFactor8  = 8
	SpreadingFactor9  = 9
	SpreadingFactor10 = 10
	SpreadingFactor11 = 11
	SpreadingFactor12 = 12
)

const (
	CodingRate4_5 = 0x01 //  7     0     LoRa coding rate: 4/5
	CodingRate4_6 = 0x02 //  7     0                       4/6
	CodingRate4_7 = 0x03 //  7     0                       4/7
	CodingRate4_8 = 0x04 //  7     0                       4/8
)

const (
	HeaderExplicit = 0x00 //  7     0     LoRa header mode: explicit
	HeaderImplicit = 0x01 //  7     0                       implicit
)

const (
	LowDataRateOptimizeOff = 0x00 //  7     0     LoRa low data rate optimization: disabled
	LowDataRateOptimizeOn  = 0x01 //  7     0                                      enabled
)

const (
	CRCOff = 0x00 //  7     0     LoRa CRC mode: disabled
	CRCOn  = 0x01 //  7     0                    enabled
)

const (
	IQStandard = 0x00 //  7     0     LoRa IQ setup: standard
	IQInverted = 0x01 //  7     0                    inverted
)

const (
	Bandwidth7_8   = iota // 7.8 kHz
	Bandwidth10_4         // 10.4 kHz
	Bandwidth15_6         // 15.6 kHz
	Bandwidth20_8         // 20.8 kHz
	Bandwidth31_25        // 31.25 kHz
	Bandwidth41_7         // 41.7 kHz
	Bandwidth62_5         // 62.5 kHz
	Bandwidth125_0        // 125.0 kHz
	Bandwidth250_0        // 250.0 kHz
	Bandwidth500_0        // 500.0 kHz
)

const (
	SyncPublic = iota
	SyncPrivate
)

const (
	MHz868_1 = 868100000
	MHz868_5 = 868500000
	MHz916_8 = 916800000
	MHz923_3 = 923300000
)

const (
	RadioEventRxDone = iota
	RadioEventTxDone
	RadioEventTimeout
	RadioEventWatchdog
	RadioEventCrcError
	RadioEventUnhandled
)

// RadioEvent are used for communicating in the radio Event Channel
type RadioEvent struct {
	EventType int
	IRQStatus uint16
	EventData []byte
}

// NewRadioEvent() returns a new RadioEvent that can be used in the RadioChannel
func NewRadioEvent(eType int, irqStatus uint16, eData []byte) RadioEvent {
	r := RadioEvent{EventType: eType, IRQStatus: irqStatus, EventData: eData}
	return r
}

type Radio interface {
	Reset()
	Tx(pkt []uint8, timeoutMs uint32) error
	Rx(timeoutMs uint32) ([]uint8, error)
	SetFrequency(freq uint32)
	SetIqMode(mode uint8)
	SetCodingRate(cr uint8)
	SetBandwidth(bw uint8)
	SetCrc(enable bool)
	SetSpreadingFactor(sf uint8)
	SetPreambleLength(plen uint16)
	SetTxPower(txpow int8)
	SetSyncWord(syncWord uint16)
	SetPublicNetwork(enable bool)
	SetHeaderType(headerType uint8)
	LoraConfig(cnf Config)
}
