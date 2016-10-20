Sample code and build process from [here](http://vedder.se/2012/07/get-started-with-stm32f4-on-ubuntu-linux/)

Commands
`make build`
Builds the project

`make clean`

`make flash`

Runs this command: Replace the path with the path to openocd

`sudo openocd -f ~/Documents/coding/iot/openocd/tcl/interface/stlink-v2.cfg -c "set WORKAREASIZE 0x2000" -f ~/Documents/coding/iot/openocd/tcl/target/stm32f4x_stlink.cfg -c "program build/stm32f4_sample.elf verify reset"`


`make`
Builds then flashes
