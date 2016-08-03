# avrdice

Arduino Uno R3 C++ code showing random dice number sequence

[日本語での説明はこちらのPDFをご参照ください](https://github.com/jj1bdx/avrdice/blob/master/mft2016-doc-ja/avrdice-mtf2016.pdf)

## Technical details

* Each digit contains values of 1 to 6
* Hardware random number generation with avrhwrng 
* Use Adafruit RGB LCD connected through I2C
* Use DFR0090 SPI 8-digit LED module (connected to Arduino SPI)
* 8-bit data sampled from avrhwrng each time

## How to build

    # build libavrtools.a 
    (cd AVRTools && make clean && make)
    # build and link
    make

## Acknowlegment

* [Igor Mikolic-Torreira](https://github.com/igormiktor): most of the work in this repository has been done through his [AVRTools](https://github.com/igormiktor/AVRTools) C++ library.

## LICENSE

GPLv3.

AVRTools is released under GPLv3, so this repository is also licensed as GPLv3.

My code modules are released under the MIT License.
