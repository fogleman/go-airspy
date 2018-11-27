# go-airspy

This is a simple Go wrapper for the [libairspy](https://github.com/airspy/airspyone_host) library.

[Airspy](https://airspy.com/products/) is a suite of low-cost, high-performance Software Defined Radios (SDR).

### Installation

    $ brew install airspy
    $ go get -u github.com/fogleman/go-airspy/airspy

### Documentation

https://godoc.org/github.com/fogleman/go-airspy/airspy

### Example

```go
func main() {
	// print the libairspy library version, e.g.
	// "1.0.9"
	fmt.Println(airspy.LibVersion())

	// open the airspy device
	device, err := airspy.Open()
	check(err)

	// print the device version, e.g.
	// "AirSpy NOS v1.0.0-rc10-3-g7120e77 2018-04-28"
	version, err := device.VersionString()
	check(err)
	fmt.Println(version)

	// set some parameters
	check(device.SetSampleType(airspy.SampleFloat32IQ))
	check(device.SetSampleRate(10000000))
	check(device.SetFrequency(98700000))
	check(device.SetLNAGain(11))
	check(device.SetMixerGain(7))
	check(device.SetVGAGain(7))

	// create a channel to receive transfers
	transfers := make(chan airspy.Transfer, 16)

	// kick off a goroutine to process the samples (see below)
	go handler(transfers)

	// start sampling
	check(device.StartRx(transfers))

	// wait a bit
	time.Sleep(100 * time.Millisecond)

	// stop sampling
	check(device.StopRx())

	// close the device
	check(device.Close())
}

func handler(transfers chan airspy.Transfer) {
	// process samples
	var maxMagnitude float64
	for transfer := range transfers {
		samples := transfer.Samples.([]float32)
		fmt.Println(len(samples), "samples")
		for i := 0; i < len(samples); i += 2 {
			I := float64(samples[i])
			Q := float64(samples[i+1])
			x := complex(I, Q)
			m := cmplx.Abs(x)
			if m > maxMagnitude {
				maxMagnitude = m
				fmt.Println(maxMagnitude)
			}
		}
	}
}

func check(err error) {
	if err != nil {
		log.Fatal(err)
	}
}
```
