# flight-software

## Guidelines
- Use prefixes (`fc_`, `fc_altimeter_`, etc.) for global variables and functions to avoid name collisions. [See this](https://softwareengineering.stackexchange.com/questions/404613/why-are-module-specific-prefixes-widely-used-for-function-names-in-c-modules).
- Clean before pushing (Go to Project->Clean, uncheck "Start a build immediately"). Otherwise, the repo will have compiled junk in it.
- "Private" functions should be `static`.
- Match the coding style (`snake_case`, etc.)

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
- [x] `data.c` -- Dersu
	- [x] make data fields for all sensors
- [ ] `process.c` -- Dersu
	- [ ] call init functions
	- [x] call barometer process function
	- [x] call telemetry process function
	- [x] call recovery process function
- [ ] telemetry -- Dersu
	- [x] load the packet parsing library we made
	- [ ] implement arming commands in packet-parser
	- [ ] confirm if all gps data (position, speed, sat count) is received at once
	- [ ] process function
		- [ ] parse packets
		- [x] read from `data.h` and send data packets
		- [ ] write those packets to the SD card -- Peter
	- [ ] tests
- [ ] recovery
	- [x] data buffer -- Dersu
	- [ ] process function -- Peter
	- [ ] filtering -- Peter
	- [ ] recovery decision -- Peter
	- [ ] tests
