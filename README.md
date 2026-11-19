# YACT-CM4

STM32MP157-based communications testbench. 

The STM32MP157 has both ARM Cortex-A7 (CA7) and ARM Cortex-M4 (CM4) cores. Its intended use is for the CA7 to run an embedded linux host while the CM4 runs bare metal/rtos. This repository holds the CM4 subproject for this system.

This project aims to be a digital communications testbench, both for fun and for use. The CA7 hosts a webpage where the user can request certain test functionality and view test results.
The MPU supports [various communications standards](https://wiki.st.com/stm32mpu/wiki/STM32MP15_peripherals_overview) and I intend to cover them all. I also intend to include radio (analog and/or SDR) testing.