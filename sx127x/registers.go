package sx127x

const (
	// registers
	SX127X_REG_FIFO                 = 0x00
	SX127X_REG_OP_MODE              = 0x01
	SX127X_REG_FRF_MSB              = 0x06
	SX127X_REG_FRF_MID              = 0x07
	SX127X_REG_FRF_LSB              = 0x08
	SX127X_REG_PA_CONFIG            = 0x09
	SX127X_REG_PA_RAMP              = 0x0a
	SX127X_REG_OCP                  = 0x0b
	SX127X_REG_LNA                  = 0x0c
	SX127X_REG_FIFO_ADDR_PTR        = 0x0d
	SX127X_REG_FIFO_TX_BASE_ADDR    = 0x0e
	SX127X_REG_FIFO_RX_BASE_ADDR    = 0x0f
	SX127X_REG_FIFO_RX_CURRENT_ADDR = 0x10
	SX127X_REG_IRQ_FLAGS_MASK       = 0x11
	SX127X_REG_IRQ_FLAGS            = 0x12
	SX127X_REG_RX_NB_BYTES          = 0x13
	SX127X_REG_PKT_SNR_VALUE        = 0x19
	SX127X_REG_PKT_RSSI_VALUE       = 0x1a
	SX127X_REG_RSSI_VALUE           = 0x1b
	SX127X_REG_MODEM_CONFIG_1       = 0x1d
	SX127X_REG_MODEM_CONFIG_2       = 0x1e
	SX127X_REG_SYMB_TIMEOUT_LSB     = 0x1f
	SX127X_REG_PREAMBLE_MSB         = 0x20
	SX127X_REG_PREAMBLE_LSB         = 0x21
	SX127X_REG_PAYLOAD_LENGTH       = 0x22
	SX127X_REG_MAX_PAYLOAD_LENGTH   = 0x23
	SX127X_REG_HOP_PERIOD           = 0x24
	SX127X_REG_MODEM_CONFIG_3       = 0x26
	SX127X_REG_FREQ_ERROR_MSB       = 0x28
	SX127X_REG_FREQ_ERROR_MID       = 0x29
	SX127X_REG_FREQ_ERROR_LSB       = 0x2a
	SX127X_REG_RSSI_WIDEBAND        = 0x2c
	SX127X_REG_DETECTION_OPTIMIZE   = 0x31
	SX127X_REG_INVERTIQ             = 0x33
	SX127X_REG_DETECTION_THRESHOLD  = 0x37
	SX127X_REG_SYNC_WORD            = 0x39
	SX127X_REG_INVERTIQ2            = 0x3b
	SX127X_REG_DIO_MAPPING_1        = 0x40
	SX127X_REG_DIO_MAPPING_2        = 0x41
	SX127X_REG_VERSION              = 0x42
	SX127X_REG_PA_DAC               = 0x4d
	// PA config
	SX127X_PA_BOOST = 0x80

	// Bits masking the corresponding IRQs from the radio
	SX127X_IRQ_LORA_RXTOUT_MASK uint8 = 0x80
	SX127X_IRQ_LORA_RXDONE_MASK uint8 = 0x40
	SX127X_IRQ_LORA_CRCERR_MASK uint8 = 0x20
	SX127X_IRQ_LORA_HEADER_MASK uint8 = 0x10
	SX127X_IRQ_LORA_TXDONE_MASK uint8 = 0x08
	SX127X_IRQ_LORA_CDDONE_MASK uint8 = 0x04
	SX127X_IRQ_LORA_FHSSCH_MASK uint8 = 0x02
	SX127X_IRQ_LORA_CDDETD_MASK uint8 = 0x01

	// DIO function mappings                D0D1D2D3
	SX127X_MAP_DIO0_LORA_RXDONE uint8 = 0x00 // 00------
	SX127X_MAP_DIO0_LORA_TXDONE uint8 = 0x40 // 01------
	SX127X_MAP_DIO1_LORA_RXTOUT uint8 = 0x00 // --00----
	SX127X_MAP_DIO1_LORA_NOP    uint8 = 0x30 // --11----
	SX127X_MAP_DIO2_LORA_NOP    uint8 = 0xC0 // ----11--

	SX127X_PAYLOAD_LENGTH uint8 = 0x40

	// Low Noise Amp
	SX127X_LNA_MAX_GAIN uint8 = 0x23
	SX127X_LNA_OFF_GAIN uint8 = 0x00
	SX127X_LNA_LOW_GAIN uint8 = 0x20

	// Bandwidth
	SX127X_LORA_BW_7_8   uint8 = 0x00
	SX127X_LORA_BW_10_4  uint8 = 0x01
	SX127X_LORA_BW_15_6  uint8 = 0x02
	SX127X_LORA_BW_20_8  uint8 = 0x03
	SX127X_LORA_BW_31_25 uint8 = 0x04
	SX127X_LORA_BW_41_7  uint8 = 0x05
	SX127X_LORA_BW_62_5  uint8 = 0x06
	SX127X_LORA_BW_125_0 uint8 = 0x07
	SX127X_LORA_BW_250_0 uint8 = 0x08
	SX127X_LORA_BW_500_0 uint8 = 0x09
	// Automatic gain control
	SX127X_AGC_AUTO_OFF uint8 = 0x00
	SX127X_AGC_AUTO_ON  uint8 = 0x01

	SX127X_LORA_MAC_PUBLIC_SYNCWORD  = 0x34
	SX127X_LORA_MAC_PRIVATE_SYNCWORD = 0x14
)

// Operation modes.
const (
	SX127X_OPMODE_MASK      uint8 = 0x07
	SX127X_OPMODE_SLEEP     uint8 = 0x00
	SX127X_OPMODE_STANDBY   uint8 = 0x01
	SX127X_OPMODE_FSTX      uint8 = 0x02
	SX127X_OPMODE_TX        uint8 = 0x03
	SX127X_OPMODE_FSRX      uint8 = 0x04
	SX127X_OPMODE_RX        uint8 = 0x05
	SX127X_OPMODE_RX_SINGLE uint8 = 0x06
	SX127X_OPMODE_CAD       uint8 = 0x07
	SX127X_OPMODE_LORA      uint8 = 0x80
)

// OpMode represents the available operation modes of the SX127x devices.
type OpMode uint8

// Operation modes.
const (
	// Sleep mode will sleep goroutine for 15ms on SetOpmode call.
	OpSleep OpMode = iota
	// Standby mode is the default mode when waiting to operate.
	OpStandby
	OpFSTx
	OpTx
	OpFSRx
	OpRx
	OpRxSingle
	OpCAD
	opLoRaBit = OpMode(SX127X_OPMODE_LORA)
)

func (op OpMode) String() (s string) {
	op = op &^ opLoRaBit
	switch op {
	case OpSleep:
		s = "sleep"
	case OpStandby:
		s = "standby"
	case OpFSTx:
		s = "fstx"
	case OpTx:
		s = "tx"
	case OpFSRx:
		s = "fsrx"
	case OpRx:
		s = "rx"
	case OpRxSingle:
		s = "rx-single"
	case OpCAD:
		s = "cad"
	default:
		s = "unknown"
	}
	return s
}
