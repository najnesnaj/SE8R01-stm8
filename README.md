This is my "bare metal" software package for sending stuff  with the stm8s103f3p6 

I ordered SE8R01 (search for SE8R01_DataSheet_v1) thinking it was a NRF24
I found some arduino software which I ported to stm8

it works! (tested with arduino receiver : will build a stm8 receiver though :) )


As far as software packages, you'll need at the very least, the SDCC compiler and a way to u
pload the resulting IHX file to the device:

On my Orange Pi:

apt-get install sdcc

(optional if you want serial comms back to the host) apt-get install minicom

git clone http://github.com/vdudouyt/stm8flash
cd stm8flash
make
make install

The hardware programmer is sold on Ebay for around $4 each. Look for "st-link v2". Note that
 a hardware programmer is required to program the stm8 family. There is not a serial uploade
r installed like the stm32 has.

Optional but extremely useful:
Saleae Logic Analyzer (About $7 each) -- to see up to 8 channels of digital waveform. EXTREMELY useful for debugging I2C. Seriously...
PL2303 USB-to-serial cable/converter (about $2 each) -- to be able to "print" from the micro
controller to a minicom terminal

