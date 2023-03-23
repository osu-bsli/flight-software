# flight-software

## TODO
Needs to be done this Sunday (2023-03-26) at the latest!
- [ ] accelerometer 1 -- Lauren, Ayden
	- [ ] initialize the interrupts
	- [ ] write interrupt logic
		- [ ] get data in the right format
		- [ ] write to `data.h`
- [ ] accelerometer 2 (low priority)
- [ ] barometer -- Toby
	- [ ] load library
	- [ ] some kind of process function
- [ ] magnetometer (low priority)
- [ ] gps (low priority)
- [ ] arming logic -- Ram
	- [ ] initialize CAN
	- [ ] write functions to arm/disarm each board (will be called from telemetry.c)
- [ ] `data.c` -- Dersu
	- [ ] make data fields for all sensors
- [ ] `process.c` -- Dersu
	- [ ] call barometer process function
	- [ ] call telemetry process function
	- [ ] call recovery process function
- [ ] telemetry -- Dersu
	- [ ] load the packet parsing library we made
	- [ ] process function
		- [ ] parse packets
		- [ ] read from `data.h` and send data packets
		- [ ] write those packets to the SD card -- Peter
- [ ] recovery
	- [ ] data buffer -- Dersu
	- [ ] process function -- Peter
	- [ ] filtering -- Peter
	- [ ] recovery decision -- Peter
