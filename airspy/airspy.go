package airspy

// #cgo pkg-config: libairspy
// #include <stdlib.h>
// #include <libairspy/airspy.h>
// extern int go_airspy_callback(airspy_transfer* transfer);
import "C"

import (
	"errors"
	"fmt"
	"reflect"
	"sync"
	"unsafe"
)

// Errors

var (
	ErrInvalidParam     = errors.New("AIRSPY_ERROR_INVALID_PARAM (-2)")
	ErrNotFound         = errors.New("AIRSPY_ERROR_NOT_FOUND (-5)")
	ErrBusy             = errors.New("AIRSPY_ERROR_BUSY (-6)")
	ErrNoMem            = errors.New("AIRSPY_ERROR_NO_MEM (-11)")
	ErrLibusb           = errors.New("AIRSPY_ERROR_LIBUSB (-1000)")
	ErrThread           = errors.New("AIRSPY_ERROR_THREAD (-1001)")
	ErrStreamingThread  = errors.New("AIRSPY_ERROR_STREAMING_THREAD_ERR (-1002)")
	ErrStreamingStopped = errors.New("AIRSPY_ERROR_STREAMING_STOPPED (-1003)")
	ErrOther            = errors.New("AIRSPY_ERROR_OTHER (-9999)")
)

func makeError(code C.int) error {
	switch code {
	case 0:
		return nil
	case -2:
		return ErrInvalidParam
	case -5:
		return ErrNotFound
	case -6:
		return ErrBusy
	case -11:
		return ErrNoMem
	case -1000:
		return ErrLibusb
	case -1001:
		return ErrThread
	case -1002:
		return ErrStreamingThread
	case -1003:
		return ErrStreamingStopped
	case -9999:
		return ErrOther
	default:
		return fmt.Errorf("AIRSPY_ERROR_UNKNOWN (%d)", code)
	}
}

// Sample Types

type SampleType int

const (
	SampleFloat32IQ SampleType = iota
	SampleFloat32Real
	SampleInt16IQ
	SampleInt16Real
	SampleUint16Real
	SampleRaw
)

// Callbacks

var (
	callbackMutex  sync.Mutex
	callbackIndex  uintptr
	callbackLookup map[uintptr]chan Transfer
)

func init() {
	callbackLookup = make(map[uintptr]chan Transfer)
}

type Transfer struct {
	Samples        interface{}
	SampleCount    int
	DroppedSamples uint64
	SampleType     SampleType
}

//export go_airspy_callback
func go_airspy_callback(data *C.airspy_transfer) C.int {
	callbackMutex.Lock()
	ch, ok := callbackLookup[uintptr(data.ctx)]
	callbackMutex.Unlock()

	if !ok {
		return 0
	}

	sampleType := SampleType(data.sample_type)
	sampleCount := int(data.sample_count)
	valueCount := int(data.sample_count)
	droppedSamples := uint64(data.dropped_samples)

	switch sampleType {
	case SampleFloat32IQ, SampleInt16IQ:
		valueCount *= 2
	}

	sh := &reflect.SliceHeader{
		Data: uintptr(data.samples),
		Len:  valueCount,
		Cap:  valueCount,
	}

	var copiedSamples interface{}

	switch sampleType {
	case SampleFloat32IQ, SampleFloat32Real:
		samples := *(*[]float32)(unsafe.Pointer(sh))
		copied := make([]float32, len(samples))
		copy(copied, samples)
		copiedSamples = copied
	case SampleInt16IQ, SampleInt16Real:
		samples := *(*[]int16)(unsafe.Pointer(sh))
		copied := make([]int16, len(samples))
		copy(copied, samples)
		copiedSamples = copied
	case SampleUint16Real:
		samples := *(*[]uint16)(unsafe.Pointer(sh))
		copied := make([]uint16, len(samples))
		copy(copied, samples)
		copiedSamples = copied
	case SampleRaw:
		// TODO?
	}

	transfer := Transfer{
		copiedSamples,
		sampleCount,
		droppedSamples,
		sampleType,
	}

	// non-blocking channel send
	// transfer will be dropped if unable to send immediately
	// consumer needs to keep up or provide sufficient channel buffering!
	select {
	case ch <- transfer:
	default:
	}

	return 0
}

// Non-Device Functions

// void airspy_lib_version(airspy_lib_version_t* lib_version);
func LibVersion() string {
	var v C.airspy_lib_version_t
	C.airspy_lib_version(&v)
	return fmt.Sprintf("%d.%d.%d", v.major_version, v.minor_version, v.revision)
}

// Device Functions

type Device struct {
	handle *C.struct_airspy_device
}

// int airspy_open(struct airspy_device** device);
func Open() (*Device, error) {
	device := Device{&C.struct_airspy_device{}}
	err := makeError(C.airspy_open(&device.handle))
	return &device, err
}

// int airspy_open_sn(struct airspy_device** device, uint64_t serial_number);
func OpenSerialNumber(serialNumber uint64) (*Device, error) {
	device := Device{&C.struct_airspy_device{}}
	err := makeError(C.airspy_open_sn(&device.handle, C.uint64_t(serialNumber)))
	return &device, err
}

// int airspy_close(struct airspy_device* device);
func (d *Device) Close() error {
	return makeError(C.airspy_close(d.handle))
}

// int airspy_get_samplerates(struct airspy_device* device, uint32_t* buffer, const uint32_t len);
func (d *Device) GetSampleRates() []uint32 {
	var count C.uint32_t
	C.airspy_get_samplerates(d.handle, &count, 0)
	buffer := make([]uint32, count)
	C.airspy_get_samplerates(d.handle, (*C.uint32_t)(&buffer[0]), count)
	return buffer
}

// /* Parameter samplerate can be either the index of a samplerate or directly its value in Hz within the list returned by airspy_get_samplerates() */
// int airspy_set_samplerate(struct airspy_device* device, uint32_t samplerate);
func (d *Device) SetSampleRate(sampleRate uint32) error {
	return makeError(C.airspy_set_samplerate(d.handle, C.uint32_t(sampleRate)))
}

// int airspy_set_conversion_filter_float32(struct airspy_device* device, const float *kernel, const uint32_t len);
// int airspy_set_conversion_filter_int16(struct airspy_device* device, const int16_t *kernel, const uint32_t len);

// int airspy_start_rx(struct airspy_device* device, airspy_sample_block_cb_fn callback, void* rx_ctx);
func (d *Device) StartRx(ch chan Transfer) error {
	var key uintptr

	callbackMutex.Lock()
	key = callbackIndex
	callbackIndex++
	callbackLookup[key] = ch
	callbackMutex.Unlock()

	err := makeError(C.airspy_start_rx(
		d.handle,
		C.airspy_sample_block_cb_fn(C.go_airspy_callback),
		unsafe.Pointer(key)))
	return err
}

// int airspy_stop_rx(struct airspy_device* device);
func (d *Device) StopRx() error {
	// TODO: remove channel from callbackLookup somehow
	return makeError(C.airspy_stop_rx(d.handle))
}

// int airspy_is_streaming(struct airspy_device* device);
func (d *Device) IsStreaming() bool {
	return C.airspy_is_streaming(d.handle) == C.AIRSPY_TRUE
}

// int airspy_si5351c_write(struct airspy_device* device, uint8_t register_number, uint8_t value);
// int airspy_si5351c_read(struct airspy_device* device, uint8_t register_number, uint8_t* value);
// int airspy_config_write(struct airspy_device* device, const uint8_t page_index, const uint16_t length, unsigned char *data);
// int airspy_config_read(struct airspy_device* device, const uint8_t page_index, const uint16_t length, unsigned char *data);
// int airspy_r820t_write(struct airspy_device* device, uint8_t register_number, uint8_t value);
// int airspy_r820t_read(struct airspy_device* device, uint8_t register_number, uint8_t* value);
// /* Parameter value shall be 0=clear GPIO or 1=set GPIO */
// int airspy_gpio_write(struct airspy_device* device, airspy_gpio_port_t port, airspy_gpio_pin_t pin, uint8_t value);
// /* Parameter value corresponds to GPIO state 0 or 1 */
// int airspy_gpio_read(struct airspy_device* device, airspy_gpio_port_t port, airspy_gpio_pin_t pin, uint8_t* value);
// /* Parameter value shall be 0=GPIO Input direction or 1=GPIO Output direction */
// int airspy_gpiodir_write(struct airspy_device* device, airspy_gpio_port_t port, airspy_gpio_pin_t pin, uint8_t value);
// int airspy_gpiodir_read(struct airspy_device* device, airspy_gpio_port_t port, airspy_gpio_pin_t pin, uint8_t* value);
// int airspy_spiflash_erase(struct airspy_device* device);
// int airspy_spiflash_write(struct airspy_device* device, const uint32_t address, const uint16_t length, unsigned char* const data);
// int airspy_spiflash_read(struct airspy_device* device, const uint32_t address, const uint16_t length, unsigned char* data);
// int airspy_board_id_read(struct airspy_device* device, uint8_t* value);

// /* Parameter length shall be at least 128bytes */
// int airspy_version_string_read(struct airspy_device* device, char* version, uint8_t length);
func (d *Device) VersionString() (string, error) {
	version := (*C.char)(C.malloc(C.sizeof_char * 128))
	defer C.free(unsafe.Pointer(version))
	err := makeError(C.airspy_version_string_read(d.handle, version, 128))
	return C.GoString(version), err
}

// int airspy_board_partid_serialno_read(struct airspy_device* device, airspy_read_partid_serialno_t* read_partid_serialno);

// int airspy_set_sample_type(struct airspy_device* device, enum airspy_sample_type sample_type);
func (d *Device) SetSampleType(sampleType SampleType) error {
	return makeError(C.airspy_set_sample_type(d.handle, uint32(sampleType)))
}

// /* Parameter freq_hz shall be between 24000000(24MHz) and 1750000000(1.75GHz) */
// int airspy_set_freq(struct airspy_device* device, const uint32_t freq_hz);
func (d *Device) SetFrequency(freqHz uint32) error {
	return makeError(C.airspy_set_freq(d.handle, C.uint32_t(freqHz)))
}

// /* Parameter value shall be between 0 and 15 */
// int airspy_set_lna_gain(struct airspy_device* device, uint8_t value);
func (d *Device) SetLNAGain(gain int) error {
	return makeError(C.airspy_set_lna_gain(d.handle, C.uint8_t(gain)))
}

// /* Parameter value shall be between 0 and 15 */
// int airspy_set_mixer_gain(struct airspy_device* device, uint8_t value);
func (d *Device) SetMixerGain(gain int) error {
	return makeError(C.airspy_set_mixer_gain(d.handle, C.uint8_t(gain)))
}

// /* Parameter value shall be between 0 and 15 */
// int airspy_set_vga_gain(struct airspy_device* device, uint8_t value);
func (d *Device) SetVGAGain(gain int) error {
	return makeError(C.airspy_set_vga_gain(d.handle, C.uint8_t(gain)))
}

// int airspy_set_lna_agc(struct airspy_device* device, uint8_t value);
func (d *Device) SetLNAAGC(enabled bool) error {
	var value C.uint8_t
	if enabled {
		value = 1
	}
	return makeError(C.airspy_set_lna_agc(d.handle, C.uint8_t(value)))
}

// int airspy_set_mixer_agc(struct airspy_device* device, uint8_t value);
func (d *Device) SetMixerAGC(enabled bool) error {
	var value C.uint8_t
	if enabled {
		value = 1
	}
	return makeError(C.airspy_set_mixer_agc(d.handle, C.uint8_t(value)))
}

// /* Parameter value: 0..21 */
// int airspy_set_linearity_gain(struct airspy_device* device, uint8_t value);
func (d *Device) SetLinearityGain(gain int) error {
	return makeError(C.airspy_set_linearity_gain(d.handle, C.uint8_t(gain)))
}

// /* Parameter value: 0..21 */
// int airspy_set_sensitivity_gain(struct airspy_device* device, uint8_t value);
func (d *Device) SetSensitivityGain(gain int) error {
	return makeError(C.airspy_set_sensitivity_gain(d.handle, C.uint8_t(gain)))
}

// int airspy_set_rf_bias(struct airspy_device* dev, uint8_t value);
func (d *Device) SetRFBias(enabled bool) error {
	var value C.uint8_t
	if enabled {
		value = 1
	}
	return makeError(C.airspy_set_rf_bias(d.handle, C.uint8_t(value)))
}

// int airspy_set_packing(struct airspy_device* device, uint8_t value);
func (d *Device) SetPacking(enabled bool) error {
	var value C.uint8_t
	if enabled {
		value = 1
	}
	return makeError(C.airspy_set_packing(d.handle, C.uint8_t(value)))
}

// const char* airspy_error_name(enum airspy_error errcode);
// const char* airspy_board_id_name(enum airspy_board_id board_id);
// /* Parameter sector_num shall be between 2 & 13 (sector 0 & 1 are reserved) */
// int airspy_spiflash_erase_sector(struct airspy_device* device, const uint16_t sector_num);
