package lora

import (
	"errors"
	"time"
)

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

func (cfg *Config) TimeOnAir(payloadLength int) time.Duration {
	crc := int64(max(cfg.CRC, 1))
	ih := int64(max(cfg.HeaderType, 1))
	ldr := int64(max(cfg.LDR, 1))
	cr := int64(cfg.CodingRate)
	spread := int64(cfg.Spread)
	// Page 31 SX1276IMLTRT SEMTECH | Alldatasheet.
	Npayload := 8*int64(payloadLength) - 4*spread + 28 + 16*crc - 20*ih
	div := 4 * (spread - 2*ldr)
	// Apply Ceil and max with minimal branching.
	if Npayload < 0 || div <= 0 {
		Npayload = 0
	} else if Npayload%div == 0 {
		Npayload /= div
		Npayload *= (cr + 4)
	} else {
		Npayload /= div
		Npayload++
		Npayload *= (cr + 4)
	}
	Npayload += 8 + int64(cfg.PreambleLength) + 5 // Says 4.25 in manual but we round up.
	// Calculate LoRa Transmission Parameter Relationship page 28.
	Ts_us := 1000_000 * (int64(1) << cfg.Spread) / int64(BandwidthToHertz(cfg.Bandwidth))
	return time.Microsecond * (time.Duration(Npayload * Ts_us))
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

func BandwidthToHertz(bw uint8) (BW int) {
	switch bw {
	case Bandwidth7_8:
		BW = 7.8e3
	case Bandwidth10_4:
		BW = 10.4e3
	case Bandwidth15_6:
		BW = 15.6e3
	case Bandwidth20_8:
		BW = 20.8e3
	case Bandwidth31_25:
		BW = 31.25e3
	case Bandwidth41_7:
		BW = 41.7e3
	case Bandwidth62_5:
		BW = 62.5e3
	case Bandwidth125_0:
		BW = 125e3
	case Bandwidth250_0:
		BW = 250e3
	case Bandwidth500_0:
		BW = 500e3
	default:
		BW = -1
	}
	return BW
}

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
	MHz433_0 = 433050000 // Medical, scientific and industrial band.
	MHz434_8 = 434790000 // Medical, scientific and industrial band.
	MHz868_1 = 868100000
	MHz868_5 = 868500000
	MHz916_8 = 916800000
	MHz923_3 = 923300000
)

func max[T ~int | ~int64 | ~uint8](a, b T) T {
	if a > b {
		return a
	}
	return b
}

// CountryConfig see https://www.thethingsnetwork.org/country/
func CountryConfig(code string, isPublic bool) (cfg Config, maxPayload uint16, err error) {
	var (
		freq     uint32
		spread   uint8
		maxTxPow int8
	)
	// Set Default values.
	cfg.CodingRate = CodingRate4_7
	cfg.HeaderType = HeaderExplicit
	cfg.PreambleLength = 12
	cfg.IQ = IQStandard
	cfg.CRC = CRCOn
	cfg.SyncWord = SyncPrivate
	if isPublic {
		cfg.SyncWord = SyncPublic
	}
	cfg.Bandwidth = Bandwidth125_0
	// https://www.iban.com/country-codes
	switch string(code[:]) {
	case "ar", "us":
		freq = MHz916_8 + (MHz923_3-MHz916_8)/4 // Range 916.8Mhz to 923.3MHz.
		spread = SpreadingFactor9
		maxTxPow = 30
		maxPayload = 242
	case "de", "es", "fr", "be", "eu":
		freq = MHz868_1 + (MHz868_5-MHz868_1)/4 // Range 868.1Mhz to 868.5MHz.
		spread = SpreadingFactor9
		maxTxPow = 20
		maxPayload = 242
	default:
		return cfg, 0, errors.New("country config not added yet")
	}
	cfg.LoraTxPowerDBm = maxTxPow
	cfg.Freq = freq
	cfg.Spread = spread
	return cfg, maxPayload, nil
}

/*
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
*/
