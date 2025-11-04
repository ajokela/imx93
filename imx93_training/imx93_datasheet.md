# i.MX 93 Applications Processor Data Sheet

**Source**: dc96c1a2-51aa-40c0-84ae-ef6e7d31713a.pdf
**Extracted**: 2025-11-01
**Method**: Vision LLM (qwen2.5vl:32b)

---


## i.MX 93 Applications Processors Data Sheet for Automotive Products

*(Page 1)*

#### Additional Information

This page is the cover page of the i.MX 93 Applications Processors Data Sheet for Automotive Products. It provides the following key information:
- Document Title: i.MX 93 Applications Processors Data Sheet for Automotive Products
- Revision: Rev. 6.1
- Date: 7 July 2025
- Part Number: IMX93AEC
- Reference to Programming Model: For functional characteristics and the programming model, see i.MX 93 Applications Processor Reference Manual (IMX93RM).


## 1 i.MX 93 introduction

*(Page 2)*

### Table 1. Features

| Subsystem | Features |
| --- | --- |
| Cortex®-A55 MPCore platform | Two Cortex®-A5 processors operating up to 1.7 GHz |
|  | - 32 KB L1 Instruction Cache |
|  | - 32 KB L1 Data Cache |
|  | - 64 KB L2 cache |
|  | - 64 KB per-core L cache |
|  | - Media Processing Engine (MPE) with Arm® Neon™ technology supporting the Advanced Single Instruction Multiple Multiple Data architecture |
|  | - Floating Point Unit (FPU) with support of the Arm® VFPv4-D16 architecture |
|  | - Supports 64-bit Arm® v8-A architecture |
|  | - 256 KB cluster L3 cache |
|  | - Parity/ECC protection on L1 cache, L2 cache, and TLB RAMs |
| Cortex®-M33 core platform | - Standby monitoring with Cortex®-A55 and other high-power modules |
|  | - Cortex®-M33 CPU operating up to 250 MHz |
|  | - Supports FPU up to 25 MHz |
|  | - Supports MPU |
|  | - Supports NVIC |
|  | - Supports FPB |
|  | - Supports DWT and ITM |
|  | - Two-way set-associative 16 KB System Cache with parity support |
|  | - Two-way set-associative 16 KB Code Cache with parity support |
|  | - 256 KB tightly coupled 1 memory (TCM) with parity support |
| Neural Processing Unit (NPU) | - Neural Network performance (256 MACs operating up to 1.0 GHz and 2 OPS/MAC) |

#### Additional Information

The i.MX 93 family represents NXP's latest power-optimized processors for smart home, building control, contactless HMI, IoT edge, Automotive, and Industrial applications. The i.MX 93 includes powerful dual Arm® Cortex®-A55 processors with speeds up to 1.7 GHz integrated with an NPU that accelerates machine learning inference. A general-purpose Arm® Cortex®-M33 running up to 250 MHz is for real-time and low-power processing. Robust control networks are possible via CAN-FD interface. Also, dual 1 Gbps Ethernet controllers, one supporting Time Sensitive Networking (TSN), drive gateway applications with low latency. The i.MX 93 Automotive qualified part is particularly useful for applications such as Driver Monitoring System (DMS), cost-optimized gateway, and general-purpose compute.


## Features

*(Page 3)*

### Table 1. Features ...continued

| Subsystem | Features |
| --- | --- |
|  | - NPU targets 8-bit and 16-bit integer RNN
- Handles 8-bit weights |
| Image Sensor Sensor Interface (SI) | - Standard pixel formats commonly used in many camera input protocols
- Programmable resolutions up to 2K
- Image processing for:
  - Supports up to 2K horizontal resolution
  - Supports pixel rate up to 200 Mpixel/s
  - Image downscaling via decimation and bi-phase filtering
  - Color space conversion
  - Interlaced to progressive conversion |
| On-chip memory | - Boot ROM (256 KB) for Cortex®-A55
- Boot ROM (256 KB) for Cortex®-M33
- On-chip RAM (64 KB) |
| External memory interface | - 16-bit DRAM interface:
  - LPDDR4X/LPDDR4 with inline ECC
  - Supports up to 2 Gbyte DDR memory space
- Three Ultra Secure Digital Host Controller (uSDHC) interfaces:
  - One eMMC 5.1 (8-bit) compliance with HS400 DDR signaling to support up to 400 MB/sec
  - One SDXC (4-bit, no eMMC 5.1, with extended capacity)
  - One SDIO (4-bit, SD/SDIO 3.01 compliance with 200 MHz SDR signaling and up to 100 MB/sec)
- FlexSPI Flash with support for XIP (for Cortex®-A55 in low-power mode) and support for either one Octal SPI or Quad SPI FLASH device. It also supports both Serial NOR and Serial NAND flash using FlexSPI. |
| Pixel Pipeline (PXP) | - BitBlit
- Flexible image composition options—alpha, chroma key
- Porter-Duff operation
- Image rotation (90°, 180°, 270°)
- Image resize
- Color space resize
- Multiple pixel format conversion support (RGB, YUV444, YUV422, YUV420, YUV400) |

#### Additional Information

This page continues the features section of the i.MX 93 Applications Processor datasheet, detailing various subsystems and their capabilities. The features include details about the Neural Processing Unit (NPU), Image Sensor Interface (SI), on-chip memory, external memory interface, and Pixel Pipeline (PXP).

#### Notes

- The table continues on the next page.
- All information is subject to legal disclaimers.

### Table 1. Features ...continued

| Subsystem | Features |
| --- | --- |
|  | - Standard 2D-DMA operation |
| LCDIF Display Display Controller | The LCDIF can drive any of three displays:
- MIPI DSi: up to 1920x1200p60
- LVDS Tx: up to 1366x768p60 or 1280x800p60
- Parallel display: up to 1366x768p60 or 1280x800p60 |
| MIPI CSI-2 Interface | One 2-lane MIPI CSI-2 camera input:
- Complaint with MIPI CSI-2 specification v1.3 and MIPI D-PHY specification v1.2
- Supports up to 2
- Supports 80 Rx data lanes (plus 1 Rx clock lane)
- Supports 80 Mbps – 1.5 Gbps data rate per lane in high speed operation
- Supports 10 Mbps data rate in low power operation |
| MIPI DSI Interface | One 4-lane MIPI DSI display with data supplied by the LCDIF
- Compliant with MIPI DSI specification v1.2 and MIPI D-PHY specification v1.2
- Capable of resolutions DSI achievable with a 200 MHz pixel clock and active pixel rate of 140 Mpixel/s with 24-bit RGB.
- Supports 80 Mbps – 1.5 Gbps data rate per lane in high speed operation
- Supports 10 Mbps data rate in low power operation |
| Audio | - Three SAI interfaces:
  - SAI1 supports 2-lane and SAI3 supports 1 lane
  - SAI2 supports 4 lanes
  - SAI2 and SAI3 support glue-less switching between PCM and stereo DSD operation
- One SPDIF supports raw capture mode that can save all the incoming bits into audio buffer
- 24-bit PDM supports up to 8 microphones (4 lanes) |
| GPIO and input/output multiplexing | - General-purpose input/output (GPIO) modules with interrupt capability
- Input/output multiplexing controller (IOMUXC) to provide centralized pad control |
| Power management | - Temperature sensor with programmable trip points
- Flexible power domain partitioning with internal power switches to support efficient power management |
| Connectivity | - Two USB 2.0 controllers and PHYs interfaces
- Two Controller Area Network (FlexCAN) modules, each optionally supporting flexible data-rate (FD) |

#### Additional Information

This page continues the listing of features for the i.MX 93 Applications Processor, focusing on various subsystems and their capabilities. It includes details about the LCDIF Display Controller, MIPI CSI-2 Interface, MIPI DSI Interface, Audio subsystem, GPIO and input/output multiplexing, Power management, and Connectivity features. Each subsystem's capabilities are described in detail, including resolutions, data rates, and operational modes.

#### Notes

- Table continues on the next page...
- All information provided in this document is subject to legal disclaimers.


## 1.1 Ordering information

*(Page 5)*

### Table 1. Features ...continued

| Subsystem | Features |
| --- | --- |
|  | Two Improved Inter Integrated Circuit (I3C) modules |
|  | Two 32-pin FlexIO modules |
|  | Three Ultra Secure Digital Host Host Controller (uSDHC) interfaces |
|  | Two Ethernet controllers (capable of simultaneous operation): |
|  |   - One Gigabit Ethernet controller with support for Energy Efficient Ethernet (EEE), Ethernet AVB, and IEEE 1588 |
|  |   - One Gigabit Ethernet controller with support for TSN in addition to EEE, Ethernet AVB, and IEEE 1588 |
|  | Eight SPI (SPSI) modules |
|  | Eight Low Power SPI (LPSPI) modules |
|  | Eight Low Power I2C modules |
|  | Eight Low Power Universal Asynchronous Receiver/Transmitter (LPUART) modules: |
|  |   - Programmable baud rates up to 5 Mbps |
|  | One Analog-to-Digital Converter (SAR ADC) module |
|  |   - 12-bit 4-channel with 1 MS/s |
| Security | Trusted Resource Domain Controller (TRDC) |
|  |   - Supports 16 domains |
|  | Arm® TrustZone® (TZ) architecture, including both Trustzone-A and Trustzone-M |
|  | On-chip RAM (OCRAM) secure region protection using OCRAM controller |
|  | EdgeLock® secure enclave |
|  | Battery Backed Security Security Module (BBSM) |
|  |   - Secure real-time clock (RTC) |
| System debug | Arm® CoreSight™ debug and trace technology |
|  | Embedded Trace FIFO (ETF) with 4 KB internal storage to provide trace buffering |
|  | Unified trace capability for dual core Cortex®-A5 and Cortex®-M33 CPUs |
|  | Cross Triggering Interface (CTI) |
|  | Support for 4-pin (JTAG) debug interface and SWD |

#### Additional Information

This page continues the listing of features for the i.MX 93 Applications Processor. It includes details on various subsystems such as communication modules, security features, and system debugging capabilities. The page also introduces the section on ordering information, indicating that Table 2 will provide examples of orderable part numbers.

#### Notes

- The table is marked as 'continued' from a previous page.
- The section on ordering information is introduced but not detailed on this page.


## Ordering Information

*(Page 6)*

### Table 2. Ordering information information

| Part number | Part number differentiator (A55) | Number of Cores | Max speed | NPU | GDET | Camera | Display | Connectivity | Audio | DDR | Package |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| MIMX9352A VTXMAC | 5 | 2 | 1.7 GHz | NPU | Disable | - 2-lane 1080p30 MIPI CSI
- Parallel camera | - 4-lane 1080p60 MIPI6
- Parallel display | - 2x GbE
- 2x USB 2.0 | 7x I2S TDM | 3.7 GT/s | 14 x 14 mm, 0.65 mm pitch |
| MIMX9352A VTXMBC | 5 | 2 | 1.7 GHz | NPU | Enable | - 2-lane 1080p30 MIPI CSI
- Parallel camera | - 4-lane 1080p60 MIPI6
- Parallel display | - 2x GbE
- 2x USB 2.0 | 7x I2S TDM | 3.7 GT/s | 14 x 14 mm, 0.65 mm pitch |
| MIMX9351A VTXMAC | 5 | 1 | 1.7 GHz | NPU | Disable | - 2-lane 1080p30 MIPI CSI
- Parallel camera | - 4-lane 1080p60 MIPI6
- Parallel display | - 2x GbE
- 2x USB 2.0 | 7x I2S TDM | 3.7 GT/s | 14 x 14 mm, 0.65 mm pitch |
| MIMX9332A VTXMAC | 3 | 2 | 1.7 GHz | — | Disable | - 2-lane 1080p30 MIPI CSI
- Parallel camera | - 4-lane 1080p60 MIPI6
- Parallel display | - 2x GbE
- 2x USB 2.0 | 7x I2S TDM | 3.7 GT/s | 14 x 14 mm, 0.65 mm pitch |
| MIMX9331A VTXMAC | 3 | 1 | 1.7 GHz | — | Disable | - 2-lane 1080p30 MIPI CSI
- Parallel camera | - 4-lane 1080p60 MIPI6
- Parallel display | - 2x GbE
- 2x USB 2.0 | 7x I2S TDM | 3.7 GT/s | 14 x 14 mm, 0.65 mm pitch |

#### Additional Information

This page contains ordering information for the i.MX 93 Applications Processor, including part numbers, core configurations, maximum speeds, NPU support, GDET (Graphics Display Engine) status, camera and display interfaces, connectivity options, audio capabilities, DDR specifications, and package dimensions.


## 2 Block diagram

*(Page 7)*

### Part number nomenclature—i.MX 93 processors

| Temperature Tj support | Family | Temperature Tj support | Cortex-A maximum CPU frequency | Package type | Reserved | Special Fuse |
| --- | --- | --- | --- | --- | --- | --- |
| Qualification Level | 93 | + | # | $ | % |  |
| Samples | i.MX9300 | Commercial: 0°C to +85°C | 1.7 GHz | Can be used to designate special versions, e.g., reduced GPU performance | No special fuse |  |
| Mass Production | 93 | Industrial: -40°C to +105°C | 900 MHz | No additional information required | A |  |
| Special | 93 | Extended Industrial: -40°C to +125°C | M | Reserved | X |  |
|  | 93 | Automotive: -40°C to +125°C | D | Full featured with GDET enabled | B |  |
| Sub-family | 11 x 11 mm | 14 x 14 mm, 0.65 mm pitch | VT | C |  |  |
| With NPU (Full-featured) | 14 x 14 mm | 11 x 11 mm, 0.5 mm pitch | VV | Z |  |  |
| Without NPU | 9 x 9 mm | 9 x 9 mm, 0.5 mm pitch, FCBGA208 | VX |  |  |  |
| Reduced NPU |  |  |  |  |  |  |
| #Cortex-A cores | 5 | 3 | 0 |  |  |  |
| Dual-core | 2 | 3 | 0 |  |  |  |
| Single-core | 1 | 1 |  |  |  |  |
| Silicon Revision | Rev. 1.0 | Rev. 2.0 | Rev. 3.0 |  |  |  |

#### Additional Information

Figure 1 describes the part number nomenclature so that characteristics of a specific part number can be identified (e.g., cores, frequency, temperature grade, fuse options, and silicon revision). The primary characteristic which describes which data sheet applies to a specific part is the temperature grade (junction) field. The document explains the different temperature grades and their corresponding data sheets for commercial, industrial, extended industrial, and automotive products. It also provides guidance on ensuring the proper data sheet is used by verifying the temperature grade field and matching it to the appropriate data sheet.

#### Notes

- Some modules shown in this block diagram are not offered on all derivatives. This block diagram may also show less modules than available in some derivatives. See Table 2 for details.


## 3 Special signal signal considerations considerations

*(Page 8)*

#### Additional Information

Figure 2 shows the i.MX 93 system block diagram, which includes the following components and domains:

### System Clock Domain
- **Oscillator**: Provides the base clock frequency.
- **PLLs**: Phase-locked loops for clock generation and distribution.
- **System Clock**: Manages the system clock signals.

### Main CPU Domain
- **2x Cortex-A55**: Dual-core ARM Cortex-A55 processors.
- **Caches**: 32 kB I-cache, 32 kB D-cache, 64 kB L2 Cache, 256 kB L3 Cache (ECC).
- **NEON**: Advanced SIMD engine for media and signal processing.
- **FPU**: Floating-point unit.

### External DRAM
- **x16 LPDDR4/4X**: Low-power DDR memory with Inline ECC.

### Low Power Real Time Domain
- **System Control**: Includes DMA, Watchdog, Periodic Timer, Timer/PWM, and Temperature Sensor.
- **Low Power Security Security MCU**: ARM Cortex-M33 core with 16 kB+16 kB Code+Sys Cache, FPU, MPU, NVIC.
- **Connectivity and I/O**: Includes LPUART, SPI, I2C, CAN-FD, 2-lane i²S TDM, 8-ch PDM Mic Input, MQS.

### EdgeLock Secure Enclave
- **Crypto**: Cryptographic functions.
- **Tamper Detection**: Physical tamper detection mechanisms.
- **Secure Clock**: Secure clock management.
- **Secure Boot**: Secure boot functionality.
- **eFuse Key Storage**: Embedded fuse key storage.
- **Random Number**: Random number generation.

### Performance Acceleration and I/O Domain
- **System Control**: Includes DMA, Watchdog, Periodic Timer, Timer/PWM, and Secure JTAG.
- **ML and Multimedia I/O**: 5-lane i²S TDM, SPDIF, 8 bpp Parallel YUV/RGB Camera, Parallel RGB Display, PXP with 2D Graphics, High-efficiency NPU.
- **Connectivity and I/O**: LPUART, SPI, I2C, CAN-FD, FlexIO, ADC, SDIO, MIPI-CSI, MIPI-DSI, Gigabit Ethernet, USB 2.0.

### Memory
- **eMMC**: Embedded MultiMediaCard interface.
- **SD/SDIO**: Secure Digital and Secure Digital Input/Output interfaces.
- **eMMC 5.1**: Enhanced MultiMediaCard version 5.1.
- **Octal SPI FLASH**: With Inline Crypto.
- **OCRAM**: 640 kB On-Chip RAM with ECC.

### Notes and Footnotes
- Table 3 lists special signal signal considerations for the i.MX 93 processors. Signal names are listed in alphabetical order.
- Package contact assignments can be found in the 'Package information and contact assignments' section.
- Signal descriptions are provided in the i.MX 93 Reference Manual (IMX93RM).

### Block Diagram Description
The block diagram illustrates the i.MX 93 processor's architecture, showing the interconnections between the System Clock Domain, Main CPU Domain, External DRAM, Low Power Real Time Domain, EdgeLock Secure Enclave, and Performance Acceleration and I/O Domain. Each domain contains specific components and peripherals, highlighting the processor's capabilities in terms of processing, memory, security, and connectivity.

#### Notes

- The signal names in Table 3 are listed in alphabetical order.
- Package contact assignments can be found in the 'Package information and contact assignments' section.
- Signal descriptions are provided in the i.MX 93 Reference Manual (IMX93RM).


## 3.1 Unused input and output guidance

*(Page 9)*

### Table 3. Special signal considerations

| Signal Name | Remarks |
| --- | --- |
| CLKIN1/CLKIN2 | CLKIN1 and CLKIN2 are input pins without internal pull-up and pull-down. An external 10K pull-down resistor is recommended if they are not used. |
| NC | These signals are No Connect (NC) and should be unconnected in the application. |
| ONOFF | A brief connection to GND in the OFF mode causes the internal power management state machine to change the state to ON. In the ON mode, a brief connection to GND generates an interrupt (intended to be a software-controllable power-down). A connection to GND for a period of time longer than the value configured in the BBNSM_CTRL[BTN_TIMEOUT] causes a forced OFF (PMIC_ON_REQ output 'L'), as long as there is no pending RTC alarm event or tamper event. |
| POR_B | POR_B has no internal pull-up/down resistor, requires external RTC pull-up resistor to NVCC_BBSM_1P8. It is recommended that POR_B is properly handled during power up/down. Please see the EVK design for details. |
| RTC_XTALI/RTC_XTALO | To hit the exact oscillation frequency, the board capacitors must be reduced to account for the board and chip parasitics. The integrated oscillation amplifier is self-biasing but relatively weak. Care must be taken to limit the parasitic leakage from RTC_XTALI and RTC_XTALO to either the power or the ground (> 100 MΩ). This de-biases the amplifier and reduces the start-up margin. If you want to feed an external low-frequency clock into RTC_XTALI, the RTC_XTALO pin must remain unconnected or driven by a complementary signal. The logic level of this forcing clock must not exceed the NVCC_BBSM_1P8 level and the frequency shall be < 50 kHz under typical conditions. |
| XTALI_24M/XTALO_24M | The system requires 24 MHz on XTAL/XTALO. The crystal cannot be eliminated by an external 24 MHz oscillator. If this clock is used as a reference for ALI/XTALO, then there are strict frequency tolerance and jitter requirements. See Clock sources and relevant interface specifications chapters for details. |

### Table 4. Unused function strapping recommendations

| Function | Ball name | Recommendations if unused |
| --- | --- | --- |
| ADC | ADC_IN0, ADC_IN1, ADC_IN2, ADC_IN3 | Tie to ground |
| TAM | TAM_IN0, ADC_IN1, ADC_IN2, ADC_IN3 | Tie to ground |
| TAMPER | TAMPER0, TAMPER1 | Tie to ground |
| LVDS | VDD_LVDS8P, LVDS_CLK_P, LVDS_CLK_N, LVDS_Dx_P, LVDS_Dx_N, VDD_1P8, LVDS_CLK_P, LVDS_CLK_N, LVDS_Dx_P, LVDS_Dx_N | Tie to ground through 10 KΩ resistors |

#### Additional Information

This page discusses the special signal considerations for the i.MX 93 Applications Processor, including details on CLKIN1/CLKIN2, NC, ONOFF, POR_B, RTC_XTALI/RTC_XTALO, and XTALI_24M/XTALO_24M signals. It also provides guidance on handling unused input and output pins, recommending termination of unused I/Os and power rails to reduce overall board power. Table 4 provides strapping recommendations for unused functions like ADC, TAM, TAMPER, and LVDS.

#### Notes

- The text mentions that Table 4 is recommended for unused function strapping recommendations.
- The text advises that if a function of the i.MX 93 is not used, the I/Os and power rails of that function can be terminated to reduce overall board power.
- The text references Table 5 for recommended connectivities for MIPI and Table 6 for USB connectivity recommendations.


## 4 Electrical characteristics

*(Page 10)*

### Table 4. Unused function strapping recommendations ...continued

| Function | Ball name | Recommendations if unused |
| --- | --- | --- |
| Digital I/O supplies | NVCC_GPIO, NVCC_WAKEUP, NVCC_AON, NVCC_SD2 | Tie to unused through 10 KΩ resistors to ground if entire bank is not used |

### Table 5. MIPI strapping recommendations

| Function | Ball name | Recommendations |
| --- | --- | --- |
| Only MIPI_CSI used | VDD_MIPI_0P8, VDD_MIPI_1P8 | Supply |
| Only MIPI_CSI used | MIPI_DS1_CLK_P, MIPI_DS1_CLK, MIPI_DS1_Dx_P, MIPI_DS1_Dx_N | Not connected |
| Only MIPI_DSI used | VDD_MIPI_0P8, VDD_MIPI_1P8 | Supply |
| Only MIPI_DSI used | MIPI_CS1_CLK_P, MIPI_CS1_CLK_N, MIPI_CS1_Dx_P, MIPI_CS1_Dx_N | Not connected |
| Neither MIPI_CSI nor MIPI_DSI used | VDD_MIPI_0P8, VDD_MIPI_1P8 | Tie to ground |
| Neither MIPI_CSI nor MIPI_DSI used | MIPI_CS1_CLK_P, MIPI_CS1_CLK_N, MIPI_CS1_Dx_P, MIPI_CS1_Dx_N | Not connected |
| Neither MIPI_CSI nor MIPI_DSI used | MIPI_DS1_CLK_P, MIPI_DS1_CLK_N, MIPI_DS1_Dx_P, MIPI_DS1_Dx_N | Not connected |
| Neither MIPI_CSI nor MIPI_DSI used | MIPI_REXT | Tie to ground |

### Table 6. USB strapping recommendations

| Function | Ball name | Recommendations |
| --- | --- | --- |
| Only USB1 used | VDD_USB_3P3, VDD_USB_1P8, VDD_USB_0P8 | Supply |
| Only USB1 used | USB2_VBUS, USB2_D_P, USB2_D_N, USB2_ID, USB2_TXRTUNE | Not connected |
| Only USB2 used | VDD_USB_3P3, VDD_USB_1P8, VDD_USB_0P8 | Supply |
| Only USB2 used | USB1_VBUS, USB1_D_P, USB1_D_N, USB1_ID, USB1_TXRTUNE | Not connected |
| Neither USB1 nor USB2 used | VDD_USB_3P3, VDD_USB_1P8, VDD_USB_0P8 | Tie to ground |
| Neither USB1 nor USB2 used | USB1_VBUS, USB1_D_P, USB1_D_N, USB1_ID, USB1_TXRTUNE | Not connected |
| Neither USB1 nor USB2 used | USB2_VBUS, USB2_D_P, USB2_D_N, USB2_ID, USB2_TXRTUNE | Not connected |

#### Additional Information

This section provides the device and module-level electrical characteristics for the i.MX 93 family of processors.


## 4.1 Chip-level conditions

*(Page 11)*

### i.MX 93 chip-level-level conditions

| For these characteristics, ... | Topic appears ... |
| --- | --- |
| Absolute maximum maximum ratings | Absolute maximum maximum ratings |
| Thermal resistance | Thermal resistance |
| Operating ranges | Operating ranges |
| Operating ranges | Operating ranges |
| Clock sources | Clock sources |
| Maximum supply currents | Maximum supply currents |
| Power modes | Power modes |
| Power supplies requirements requirements and restrictions | Power supplies requirements requirements and restrictions |

### Absolute maximum maximum ratings

| Parameter description | Symbol | Min | Max | Unit | Notes |
| --- | --- | --- | --- | --- | --- |
| Core supplies input input voltages | VDD_SOC | -0.3 | 1.15 | V | — |
| GPIO supply voltage | NVCC_GPIO, NVCC_WAKEUP, NVCC_AON | -0.3 | 3.8 | V | — |
| I/O supply for SD2 | NVCC_SD2 | -0.3 | 3.8 | V | — |
| I/O PHY SD2 | NVCC_SD2 | -0.3 | 3.8 | V | — |
| DDR PHY supply voltage | VDD2_DDR | -0.3 | 1.575 | V | — |
| DDR I/O supply voltage | VDDQ_DDR | -0.3 | 1.575 | V | — |
| I/O supply and I/O Pre-driver supply for BBSM bank | NVCC_BBSM_1P8 | -0.3 | 2.15 | V | — |
| USB VBUS input detected | USB1_VBUS | -0.3 | 3.95 | V | — |
| USB VBUS input detected | USB2_VBUS | -0.3 | 3.95 | V | — |
| Power for USB OTG PHY | VDD_USB_0P8 | -0.3 | 1.15 | V | — |

### Electrical Specifications

| Parameter | Symbol | Min | Typ | Max | Unit | Conditions |
| --- | --- | --- | --- | --- | --- | --- |
| Core supplies input input voltages | VDD_SOC | -0.3 |  | 1.15 | V | — |
| GPIO supply voltage | NVCC_GPIO, NVCC_WAKEUP, NVCC_AON | -0.3 |  | 3.8 | V | — |
| I/O supply for SD2 | NVCC_SD2 | -0.3 |  | 3.8 | V | — |
| I/O PHY SD2 | NVCC_SD2 | -0.3 |  | 3.8 | V | — |
| DDR PHY supply voltage | VDD2_DDR | -0.3 |  | 1.575 | V | — |
| DDR I/O supply voltage | VDDQ_DDR | -0.3 |  | 1.575 | V | — |
| I/O supply and I/O Pre-driver supply for BBSM bank | NVCC_BBSM_1P8 | -0.3 |  | 2.15 | V | — |
| USB VBUS input detected | USB1_VBUS | -0.3 |  | 3.95 | V | — |
| USB VBUS input detected | USB2_VBUS | -0.3 |  | 3.95 | V | — |
| Power for USB OTG PHY | VDD_USB_0P8 | -0.3 |  | 1.15 | V | — |

#### Additional Information

This section provides the device-level electrical characteristics for the IC. See Table 7 for a quick reference to the individual tables and sections. CAUTION: Stresses beyond those listed in the following table may reduce the operating lifetime or cause immediate permanent damage to the device. The table below does not imply functional operation beyond those indicated in the operating ranges and parameters table.

#### Notes

- CAUTION: Stresses beyond those listed in the following table may reduce the operating lifetime or cause immediate permanent damage to the device. The table below does not imply functional operation beyond those indicated in the operating ranges and parameters table.


## 4.1.2 Thermal resistance

*(Page 12)*

### Table 8. Absolute maximum ratings ...continued

| Parameter description | Symbol | Min | Max | Unit | Notes |
| --- | --- | --- | --- | --- | --- |
| VDD_USB_1P8 | VDD_USB_1P8 | -0.3 | 2.15 | V | — |
| VDD_USB_3P3 | VDD_USB_3P3 | -0.3 | 3.95 | V | — |
| MIPI PHY supply voltage | VDD_MIPI_0P8 | -0.3 | 1.15 | V | — |
| MIPI PHY supply voltage | VDD_MIPI_1P8 | -0.3 | 2.15 | V | — |
| LVDS PHY supply voltage | VDD_MIPI_LVDS_1P8 | -0.3 | 2.15 | V | — |
| LVDS PHY supply voltage | VDD_LVDS_1P8 | -0.3 | 2.15 | V | — |
| Analog core supply supply voltage | VDD_ANA_0P8 | -0.3 | 1.15 | V | — |
| Analog core supply supply voltage | VDD_ANAx_1P8 | -0.3 | 2.15 | V | 1 |
| Input/output voltage range | V_in/V_out | -0.3 | OVDD^2 + 0.3 | V | — |
| Storage temperature range | T_STORAGE | -40 | 150 | °C | — |

### Table 9. Electrostatic discharge and latch-up ratings

| Parameter description | Rating | Reference | Comment |
| --- | --- | --- | --- |
| Electrostatic Discharge (ESD) | Human Body Model (HBM) | ±1000 V | JS-001 | — |
| Electrostatic Discharge (ESD) | Charged Device Model (CDM) | ±250 V | JS-002 | — |
| Latch-up (LU) | Immunity Device level Model (CDM) | ±250 V | JS-002 | — |
| Latch-up (LU) | Immunity Class I: @ 25°C ambient temperature | A | JESD78 | — |
| Latch-up (LU) | Immunity Class II: @ 125°C ambient temperature | A | JESD78 | — |

### Table 10. 14 x 14 mm FCBGA thermal resistance data

| Rating | Board Type | Symbol | Values | Unit |
| --- | --- | --- | --- | --- |
| Junction to Ambient Thermal Resistance | JESD51-9, 2s2p | R_θJA | 21.7 | °C/W |
| Junction-to-Top of Package Thermal Resistance | JESD51-9, 2s2p | Ψ_JT | 0.1 | °C/W |

### Electrical Specifications

| Parameter | Symbol | Min | Typ | Max | Unit | Conditions |
| --- | --- | --- | --- | --- | --- | --- |
| VDD_USB_1P8 | VDD_USB_1P8 | -0.3 |  | 2.15 | V |  |
| VDD_USB_3P3 | VDD_USB_3P3 | -0.3 |  | 3.95 | V |  |
| MIPI PHY supply voltage (VDD_MIPI_0P8) | VDD_MIPI_0P8 | -0.3 |  | 1.15 | V |  |
| MIPI PHY supply voltage (VDD_MIPI_1P8) | VDD_MIPI_1P8 | -0.3 |  | 2.15 | V |  |
| LVDS PHY supply voltage (VDD_MIPI_LVDS_1P8) | VDD_MIPI_LVDS_1P8 | -0.3 |  | 2.15 | V |  |
| LVDS PHY supply voltage (VDD_LVDS_1P8) | VDD_LVDS_1P8 | -0.3 |  | 2.15 | V |  |
| Analog core supply supply voltage (VDD_ANA_0P8) | VDD_ANA_0P8 | -0.3 |  | 1.15 | V |  |
| Analog core supply supply voltage (VDD_ANAx_1P8) | VDD_ANAx_1P8 | -0.3 |  | 2.15 | V |  |
| Input/output voltage range | V_in/V_out | -0.3 |  | OVDD^2 + 0.3 | V |  |
| Storage temperature range | T_STORAGE | -40 |  | 150 | °C |  |
| Electrostatic Discharge (ESD) - Human Body Model (HBM) |  |  |  |  |  |  |
| Electrostatic Discharge (ESD) - Charged Device Model (CDM) |  |  |  |  |  |  |
| Latch-up (LU) - Immunity Device level Model (CDM) |  |  |  |  |  |  |
| Latch-up (LU) - Immunity Class I |  |  |  |  |  | @ 25°C ambient temperature |
| Latch-up (LU) - Immunity Class II |  |  |  |  |  | @ 125°C ambient temperature |
| Junction to Ambient Thermal Resistance | R_θJA |  |  |  | °C/W |  |
| Junction-to-Top of Package Thermal Resistance | Ψ_JT |  |  |  | °C/W |  |

#### Additional Information

This page contains technical specifications for the i.MX 93 Applications Processor, including absolute maximum ratings, electrostatic discharge and latch-up ratings, and thermal resistance data for the 14 x 14 mm FCBGA package.

#### Notes

- 1. VDD_ANAx_1P8 refers to VDD_ANA_0P8, VDD_ANA_1P8, and VDD_ANAVDET_1P8.
- 2. OVDD is the I/O supply voltage.


## 4.1.3 Power architecture

*(Page 13)*

### 14 x 14 mm FCPBGA thermal resistance data

| Rating | Board Type | Symbol | Values | Unit |
| --- | --- | --- | --- | --- |
| Junction to Case Thermal Resistance | JESD51-9, 1s | RθJC | 5.4 | °C/W |

### Power supplies of the DRAM controller and PHY

| Power supplies | Modules |
| --- | --- |
| VDD_SOC | SoC synthesized DRAM controller digital logic |
| VDD_ANA_0P8 | DRAM PLL and PHY digital logic |
| VDD_ANAX_1P8 | DRAM 1.1 V PLL and PHY I/O analog supply circuitry |
| VDD2_DDR | 0.6 V DRAM PHY I/O supply for LPDDR4X |
| VDDQ_DDR | 0.6 V DRAM PHY I/O supply for LPDDR4X |

### Electrical Specifications

| Parameter | Symbol | Min | Typ | Max | Unit | Conditions |
| --- | --- | --- | --- | --- | --- | --- |
| Junction to Case Thermal Resistance | RθJC | 5.4 | 5.4 | 5.4 | °C/W | Test board meets JEDEC specification JESD51-9, 1s |

#### Additional Information

Thermal test board meets JEDEC specification JESD51-9. Test board has 40 vias under die shadow mapped according to BGA layout under die. Each via is 0.2 mm in diameter and connects top layer with the first buried plane layer. Thermal resistance data is determined in accordance with JEDEC JESD51-2A natural convection environment. It is solely for thermal performance comparison and not meant to predict package performance in application-specific environments. Junction-to-Case thermal resistance is determined using an isothermal cold plate. Case temperature refers to the package top side surface temperature. The power architecture of i.MX 93 assumes systems are constructed for the case where the PMIC is used to supply all power rails to the processor. The SoC may be powered from discrete parts, but a discrete-based solution is not necessarily BOM cost-optimized. NVCC_BBSM_1P8 must be powered first and stay until the last. The digital logic inside the chip will be supplied with VDD_SOC, which can be nominal, overdrive, or "Low Drive" voltage. The DRAM controller and PHY have multiple external power supplies. For all integrated analog modules, their 1.8 V analog power will be supplied externally through power pads. These supplies are separated from other power pads on the package to keep them clean, but they can be directly shared with other power rails on the board to reduce the number of power supplies from the PMIC. For integrated LVDS PHY, MIPI PHY, and USB PHYs, their 3.3 V (where supported), 1.8 V, and digital power will be supplied externally through power pads. These powers are separated from other power pads on the package to keep them clean, but they can be directly shared with other power rails on the board. For BBSM/RTC, the 1.8 V I/O pre-driver supply and 1.8 V I/O pad supply will also be supplied externally. The BBSM_LP core digital domain logic is supplied by an internal LDO. Figure 3 is the power architecture diagram for the whole chip, showing power supplies and internal LDO regulators.

#### Notes

- 1. Thermal test board meets JEDEC specification for this package (JESD51-9). Test board has 40 vias under die shadow mapped according to BGA layout under die. Each via is 0.2 mm in diameter and connects top layer with the first buried plane layer.
- 2. Determined in accordance to JEDEC JESD51-2A natural convection environment. Thermal resistance data is solely for thermal performance comparison and not meant to predict package performance in application-specific environments.
- 3. Junction-to-Case thermal resistance is determined using an isothermal cold plate. Case temperature refers to the package top side surface temperature.


## 4.1.1 Operating ranges

*(Page 15)*

### Operating ranges

| Parameter Description | Symbol | Min | Typ | Max | Unit | Comment |
| --- | --- | --- | --- | --- | --- | --- |
| Power supply for SoC logic logic and Arm core | VDD_SOC | 0.85 | 0.90 | 0.95 | V | Power supply for SoC, overdrive mode |
|  |  | 0.80 | 0.85 | 0.90 | V | Power supply for SoC, nominal mode |
|  |  | 0.76 | 0.80 | 0.84 | V | Power supply for SoC, low drive mode |
|  |  | 0.61 | 0.65 | 0.70 | V | Power supply for SoC, suspend mode |
| Digital supply for PLLs, temperature sensor, LVCMSOS I/O, MIPI, and USB PHYs | VDD_ANA_0P8 | 0.76 | 0.80 | 0.84 | V |  |
|  | VDD_MIPI_0P8 |  |  |  |  |  |
|  | VDD_MIPI_0P8 |  |  |  |  |  |
|  | VDD_USB_0P8 |  |  |  |  |  |
| 1.8 V supply for PLLs, eFuse, Temperature sensor, LVCMSOS voltage detect reference, ADC, 24 MHz XTAL, LVDS, MIPII, and USB PHYs | VDD_ANAx_1P8 | 1.71 | 1.80 | 1.89 | V | 2 |
|  | VDD_LVDS_1P8 |  |  |  |  |  |
|  | VDD_MIPI_1P8 |  |  |  |  |  |
|  | VDD_USB_1P8 |  |  |  |  |  |
| 3.3 V supply for USB PHY | VDD_USB_3P3 | 3.069 | 3.30 | 3.45 | V |  |
|  | VDD2_3P3 | 1.06 | 1.10 | 1.14 | V |  |
| Voltage supply DRAM PHY | VDD_DRAM | 1.06 | 1.10 | 1.14 | V |  |
| Voltage PHY supply for DRAM PHY I/O | VDDQ_DRAM | 1.06 | 1.10 | 1.14 | V | LPDDR4 |
|  |  | 0.57 | 0.60 | 0.67 | V | LPDDR4X |
| I/O supply I/O and pre-driver supply for GPIO in BBSM bank | NVCC_BBSM_1P8 | 1.62 | 1.80 | 1.98 | V |  |
| Power supply for GPIO when it is in 1.8 V mode | NVCC_AON | 1.62 | 1.80 | 1.98 | V |  |
|  | NVCC_SD2 |  |  |  |  |  |

### Electrical Specifications

| Parameter | Symbol | Min | Typ | Max | Unit | Conditions |
| --- | --- | --- | --- | --- | --- | --- |
| Power supply for SoC logic logic and Arm core | VDD_SOC | 0.85 | 0.90 | 0.95 | V | overdrive mode |
| Power supply for SoC logic logic and Arm core | VDD_SOC | 0.80 | 0.85 | 0.90 | V | nominal mode |
| Power supply for SoC logic logic and Arm core | VDD_SOC | 0.76 | 0.80 | 0.84 | V | low drive mode |
| Power supply for SoC logic logic and Arm core | VDD_SOC | 0.61 | 0.65 | 0.70 | V | suspend mode |
| Digital supply for PLLs, temperature sensor, LVCMSOS I/O, MIPI, and USB PHYs | VDD_ANA_0P8 | 0.76 | 0.80 | 0.84 | V |  |
| 1.8 V supply for PLLs, eFuse, Temperature sensor, LVCMSOS voltage detect reference, ADC, 24 MHz XTAL, LVDS, MIPII, and USB PHYs | VDD_ANAx_1P8 | 1.71 | 1.80 | 1.89 | V | 2 |
| 3.3 V supply for USB PHY | VDD_USB_3P3 | 3.069 | 3.30 | 3.45 | V |  |
| Voltage supply DRAM PHY | VDD_DRAM | 1.06 | 1.10 | 1.14 | V |  |
| Voltage PHY supply for DRAM PHY I/O | VDDQ_DRAM | 1.06 | 1.10 | 1.14 | V | LPDDR4 |
| Voltage PHY supply for DRAM PHY I/O | VDDQ_DRAM | 0.57 | 0.60 | 0.67 | V | LPDDR4X |
| I/O supply I/O and pre-driver supply for GPIO in BBSM bank | NVCC_BBSM_1P8 | 1.62 | 1.80 | 1.98 | V |  |
| Power supply for GPIO when it is in 1.8 V mode | NVCC_AON | 1.62 | 1.80 | 1.98 | V |  |

#### Additional Information

This page contains technical specifications for the operating ranges of the i.MX 93 Applications Processor. It includes details on power supply voltages for various components such as the SoC logic, PLLs, temperature sensors, USB PHYs, and DRAM. The table provides minimum, typical, and maximum voltage values along with specific operating modes or conditions for each parameter.

#### Notes

- If ADC is used, external voltage reference is recommended for VDD_ANA_1P8 to improve the ADC ENOB.
- The table continues on the next page.


## 4.1.5 Maximum frequency of main modules modules

*(Page 16)*

### Operating ranges ...continued

| Parameter supply Description | Symbol | Min | Typ | Max | Unit | Comment |
| --- | --- | --- | --- | --- | --- | --- |
| Power supply for GPIO when it is in 3.3 V mode | NVCC_GPIO | 3.00 | 3.30 | 3.465 | V | — |
| Power supply for WAKEUP | NVCC_WAKEUP | 3.00 | 3.30 | 3.465 | V | — |

### Temperature Ranges

| Parameter supply Description | Symbol | Min | Typ | Max | Unit | Comment |
| --- | --- | --- | --- | --- | --- | --- |
| Junction temperature — Automotive | TJ | -40 | — | +125 | °C | See the application note, i.MX 93 Product Lifetime Usage Usage Estimates for information on product lifetime (power-on hours) for this processor. |
| Ambient temperature — Automotive | Ta | -40 | — | +85 | °C | See the application note, i.MX 93 Product Lifetime Usage Usage Estimates for information on product lifetime (power-on hours) for this processor. |

### Maximum frequency of main modules

| Main modules | Frequency (Low Drive mode) | Frequency (Nominal mode) | Frequency (Overdrive mode) |
| --- | --- | --- | --- |
| EdgeLock® Secure Enclave | 133 MHz | 200 MHz | 250 MHz |
| Cortex®-M33 core | 133 MHz | 200 MHz | 250 MHz |
| Cortex®-A55 core | 0.9 GHz | 1.4 GHz | 1.7 GHz |
| DRAM | 933 MHz | 1400 MHz | 1866 MHz |
| NPU | 500 MHz | 800 MHz | 1000 MHz |

### Electrical Specifications

| Parameter | Symbol | Min | Typ | Max | Unit | Conditions |
| --- | --- | --- | --- | --- | --- | --- |
| Power supply for GPIO when it is in 3.3 V mode | NVCC_GPIO | 3.00 | 3.30 | 3.465 | V | — |
| Power supply for WAKEUP | NVCC_WAKEUP | 3.00 | 3.30 | 3.465 | V | — |
| Junction temperature — Automotive | TJ | -40 | — | +125 | °C | See the application note, i.MX 93 Product Lifetime Usage Usage Estimates for information on product lifetime (power-on hours) for this processor. |
| Ambient temperature — Automotive | Ta | -40 | — | +85 | °C | See the application note, i.MX 93 Product Lifetime Usage Usage Estimates for information on product lifetime (power-on hours) for this processor. |

#### Additional Information

Applying the maximum voltage results in maximum power consumption and heat generation. NXP recommends a voltage set point = (V_min + the supply tolerance). This results in an optimized power/speed ratio. VDD_ANA_1P8 refers to VDD_ANA_1P8, VDD_ANA_1P8, and VDD_ANADET_1P8. TJ minimum temperature supported at startup where TJ = Ta. For more detailed information about the clock, see Chapter Clock Clock Controller Module (CCM) of i.MX 93 Applications Processor Processor Reference Manual. The i.MX 93 processor is designed to function with quartz crystals to generate the frequencies necessary for operation. 24 MHz for the main clock source and 32.768 kHz for the real-time clock. External clock can be injected into RTC_XTALI if the frequency and precision are sufficient. The XTAL input is used to synthesize all of the clocks in the system with the RTC_XTAL input contribution to timekeeping and low-frequency operations.

#### Notes

- 1. Applying the maximum voltage results in maximum power consumption and heat generation. NXP recommends a voltage set point = (V_min + the supply tolerance). This results in an optimized power/speed ratio.
- 2. VDD_ANA_1P8 refers to VDD_ANA_1P8, VDD_ANA_1P8, and VDD_ANADET_1P8.
- 3. TJ minimum temperature supported at startup where TJ = Ta.


## 4.1.6.2 On-chip 24 MHz oscillators

*(Page 17)*

### Table 14. External input clock frequency

| Parameter Description | Symbol | Min | Typ | Max | Unit |
| --- | --- | --- | --- | --- | --- |
| RTC_XTALI Oscillator | f_ckil | — | 32.768 | — | kHz |

### Table 15. Audio external clock frequency

| Parameter Description | Symbol | Low drive mode | Nominal mode | Overdrive mode | Unit |
| --- | --- | --- | --- | --- | --- |
| EXT_CLK maximum frequency | f_ext | 133 | 200 | 200 | MHz |

### Table 16. RTC_OSC

|  | Symbol | Min | Typ | Max | Unit |
| --- | --- | --- | --- | --- | --- |
| Frequency | f | — | 32.768 | — | kHz |
| RTC_XTALI | V_H | 0.9 x NVCC_BBSM_1P8 | — | NVCC_BBSIM_1P8 | V |
|  | V_L | 0 | — | 0.1 x NVCC_BBSM_1P8 | V |
|  | Duty cycle | 45 | — | 55 | % |

### Table 17. 24M quartz specifications specifications

| Symbol | Parameter Description | Min | Typ | Max | Unit |
| --- | --- | --- | --- | --- | --- |
| fXTAL | Frequency | — | 24 | — | MHz |
| CLOAD | Cload | — | 12 | — | pF |
| DL | Drive level | — | — | 100 | µW |
| ESR | ESR | — | — | 120 | Ω |

### Electrical Specifications

| Parameter | Symbol | Min | Typ | Max | Unit | Conditions |
| --- | --- | --- | --- | --- | --- | --- |
| RTC_XTALI Oscillator | f_ckil | — | 32.768 | — | kHz | External oscillator or a crystal with internal oscillator amplifier. |
| EXT_CLK maximum frequency | f_ext | 133 | 200 | 200 | MHz | Audio EXT_CLK signal muxed on either pin SD2_VSELECT or PDM_BIT_STREAM1. |
| Frequency | f | — | 32.768 | — | kHz | For RTC_OSC. |
| V_H | V_H | 0.9 x NVCC_BBSM_1P8 | — | NVCC_BBSIM_1P8 | V | For RTC_XTALI. |
| V_L | V_L | 0 | — | 0.1 x NVCC_BBSM_1P8 | V | For RTC_XTALI. |
| Duty cycle | — | 45 | — | 55 | % | For RTC_XTALI. |
| fXTAL | fXTAL | — | 24 | — | MHz | 24 MHz quartz crystal oscillator. |
| CLOAD | CLOAD | — | 12 | — | pF | Load capacitance for 24 MHz quartz crystal. |
| DL | DL | — | — | 100 | µW | Drive level for 24 MHz quartz crystal. |
| ESR | ESR | — | — | 120 | Ω | Equivalent series resistance for 24 MHz quartz crystal. |

#### Additional Information

The page discusses the external input clock frequency requirements for the i.MX 93 Applications Processor, focusing on the RTC_XTALI Oscillator and its recommended frequency of 32.768 kHz. It also covers the maximum frequency of the external audio clock (EXT_CLK) in different drive modes. The RTC_OSC section specifies the frequency, voltage levels, and duty cycle for the RTC_XTALI pin. The 24 MHz oscillators section describes the specifications for a 24 MHz quartz crystal used in conjunction with an integrated amplifier to form a crystal oscillator. The text explains that the 24 MHz crystal oscillator serves as the reference clock for frequency synthesis on the processor.

#### Notes

- 1. External oscillator or a crystal with internal oscillator amplifier.
- 2. Recommended nominal frequency is 32.768 kHz.
- 1. Audio EXT_CLK signal muxed on either pin SD2_VSELECT or PDM_BIT_STREAM1.
- 1. An external 24 MHz crystal is used in conjunction with the integrated amplifier to form a crystal oscillator that is used as the reference clock for all frequency synthesis on the processor.


## i.MX 93 Applications Processors Data Sheet for Automotive Products

*(Page 18)*

### 32.768 kHz quartz specifications

| Symbol | Parameter Description | MIn | Typ | Max | Unit |
| --- | --- | --- | --- | --- | --- |
| fXTAL | Frequency (crystal mode) 1 | -- | 32.768 | -- | kHz |
| CLOAD | Cload | -- | 12.5 | -- | pF |
| ESR | ESR | -- | -- | 90 | KΩ |

### Maximum supply currents

| Power rail | Max current | Unit |
| --- | --- | --- |
| VDD_SOC | 2700 | mA |
| VDD_ANA | 2700 | mA |
| VDD_ANA_0P8 | 50 | mA |
| VDD_ANAx_1P8 1 | 250 | mA |
| NVCC_BBSM_1P8 | 2 | mA |
| NVCC_GPIO, NVCC_WAKEUP, NVCC_AON | I_max = N x C x V x x (0.5 x F) | -- |
| NVCC_GPIO, NVCC_WAKEUP, NVCC_AON | I_max = N x C x V x x (0.5 x F) | -- |
| VDDQ_DDR | 160 | mA |
| VDD2_DDR | 525 | mA |
| VDD_MIPI_0P8 (for MIPI CSI-2 2-lane Rx PHY) | 18 | mA |
| VDD_MIPI_0P8 (for MIPI-DSI 4-lane Tx PHY) | 33 | mA |
| VDD_MIPI_1P8 (for MIPI CSI-2 2-lane Tx PHY) | 3.5 | mA |
| VDD_MIPI_1P8 (for MIPI CSI-2 2-lane Rx PHY) | 2.5 | mA |

### Electrical Specifications

| Parameter | Symbol | Min | Typ | Max | Unit | Conditions |
| --- | --- | --- | --- | --- | --- | --- |
| Frequency (crystal mode) | fXTAL | -- | 32.768 | -- | kHz | 1 |
| Cload | CLOAD | -- | 12.5 | -- | pF | -- |
| ESR | ESR | -- | -- | 90 | KΩ | -- |

#### Additional Information

Actual working drive level is dependent on real design. Please contact crystal vendor for selecting drive level of crystal. Power consumption is highly dependent on the application. Estimating the maximum supply currents required for power supply design is difficult because the use cases that require maximum supply current are not realistic use cases. Data was collected while running commercial standard benchmarks designed to be compute and graphic intensive. The results provided are intended to be used as guidelines for power supply design. I_max = N x C x V x x (0.5 x F) Where: - N = Number of IO pins supplied by the power line - C = Equivalent external capacitive load - V = IO voltage - (0.5 x F) = Data change rate. Up to 0.5 of the clock rate (F).

#### Notes

- 1. Actual working drive level level is dependent on real design. Please contact crystal vendor for selecting drive level of crystal.


## 4.2 Power modes

*(Page 19)*

### Table 19. Maximum supply currents ...continued

| Power rail | Max current | Unit |
| --- | --- | --- |
| VDD_MIPI_1P8 (for MIPI-DSI 4-lane Tx PHY) | 9.5 | mA |
| VDD_USB_3P3 (for USB PHY) | 25.2 | mA |
| VDD_USB_1P8 (for USB PHY) | 36.2 | mA |
| VDD_USB_0P8 (for USB PHY) | 22.2 | mA |
| VDD_LVDS_1P8 | Max dynamic current 45 | mA |

#### Additional Information

{'section_4_2': 'This section introduces the power modes used in the i.MX 93.', 'section_4_2_1': 'This subsection defines the power mode definitions for the i.MX 93.', 'power_modes': [{'mode': 'RUN Mode', 'description': 'All external power rails are on, the Cortex-A55 is active and running; other internal modules can be on/off based on application.'}, {'mode': 'Low Power RUN Mode', 'description': 'A very low power run mode with all external power rails on. Unnecessary power domains can be off, except AONMIX and required internal modules. Cortex-A55 can be in self-refresh/retention mode. Modules in AONMIX can be used directly. Additional peripherals can be turned on as needed.'}, {'mode': 'IDLE Mode', 'description': 'Cortex-A55 automatically enters this mode when no threads are running and high-speed devices are inactive. Most of the internal logic is clock gated, but remains powered. Compared to RUN mode, power consumption is reduced.'}, {'mode': 'SUSPEND Mode', 'description': "The most power-saving mode where all clocks are off, unnecessary power supplies are off, and all PHYs are power gated. Cortex-A55 is in self-refresh/retention mode. VDD_SOC voltage is reduced to the 'Suspend mode' voltage. Exit time is longer than IDLE, but power consumption is much lower."}, {'mode': 'BBSM Mode (RTC mode)', 'description': 'Only the power for the BBSM domain remains on to keep RTC and BBSM logic alive.'}, {'mode': 'OFF Mode', 'description': 'All power rails are off.'}], 'additional_notes': 'Beyond the defined modes, additional options can be configured in software, such as adjusting clock frequencies or gate clocks through the CCM programming model, or adjusting on-die power gating through the SRC programming model.'}

#### Notes

- {'note_number': '1', 'text': 'VDD_ANAxP8 refers to VDD_ANA0_1P8, VDD_ANA1_1P8, and VDD_ANAVDET_1P8.'}


## 4.2.2 Low power modes

*(Page 20)*

### The power supply states

| Power rail | OFF | BBSM | SUSPEND (Analog on) | IDLE | RUN/LP RUN |
| --- | --- | --- | --- | --- | --- |
| NVCC_BBSM_1P8 | OFF | ON | ON | ON | ON |
| VDD_BBS_1P8 | OFF | OFF | ON | ON | ON |
| VDD_SOC | OFF | OFF | ON | ON | ON |
| VDD2_DDR | OFF | OFF | ON | ON | ON |
| VDDO_<DDR | OFF | OFF | ON | ON | ON |
| NVCC_<XXX> | OFF | OFF | ON | ON | ON |
| VDD_ANA_0P8 | OFF | OFF | ON | ON | ON |
| VDD_MIPI_0P8 | OFF | OFF | ON | ON | ON |
| VDD_USB_0P8 | OFF | OFF | ON | ON | ON |
| VDD_ANA_1P8 | OFF | OFF | ON | ON | ON |
| VDD_LVDS_1P8 | OFF | OFF | ON | ON | ON |
| VDD_MIPI_1P8 | OFF | OFF | ON | ON | ON |
| VDD_USB_1P8 | OFF | OFF | ON | ON | ON |
| VDD_USB_3P3 | OFF | OFF | ON | ON | ON |

### Low power mode definition

|  | IDLE | SUSPEND | BBSM |
| --- | --- | --- | --- |
| CCM LPM mode | WAIT | STOP | N/A |
| Arm Cortex®-A55 CPU0 | OFF | OFF | OFF |
| Arm Cortex-A55 CPU0 | OFF | OFF | OFF |
| Arm Cortex-A55 CPU1 | OFF | OFF | OFF |
| Shared L3 cache | ON | OFF | OFF |
| Display | OFF | OFF | OFF |
| DRAM controller and PHY | ON | OFF | OFF |
| ARM_PLL | OFF | OFF | OFF |
| DRAM_PLL | OFF | OFF | OFF |
| DRAM_PLL_PLL 1/2/3 | OFF | OFF | OFF |
| SYSTEM_PLL 1/2/3 | ON | OFF | OFF |
| XTAL | ON | OFF | OFF |

#### Additional Information

This page discusses low power modes for the i.MX 93 Applications Processor. It defines the power supply states for various power rails across different modes (OFF, BBSM, SUSPEND, IDLE, RUN/LP RUN). Table 20 lists the power supply states for different power rails. Table 21 defines the low power modes (IDLE, SUSPEND, BBSM) for various modules within the processor. The state of each module in these modes is detailed in the table.

#### Notes

- The state of each module in the IDLE, SUSPEND, and BBSM mode are defined in Table 21.
- Table continues on the next page...


## 4.2.3 Chip power in different Low Power Power modes

*(Page 21)*

### Table 21. Low power mode definition ...continued

|  | IDLE | SUSPEND | BBSM |
| --- | --- | --- | --- |
| RTC | ON | ON | ON |
| External DRAM device | Self-Refresh | Self-Refresh | OFF |
| USB PHY | In Low Power State | OFF | OFF |
| DRAM clock | 266 MHz | OFF | OFF |
| NOC clock | 133 MHz | OFF | OFF |
| AXI clock | 133 MHz | OFF | OFF |
| Module clocks | ON as needed | OFF | OFF |
| EdgeLock® Secure Enclave | ON | ON | ON |
| GPIO Wakeup | Yes | Yes | OFF |
| RTC Wakeup | Yes | Yes | Yes |
| RTC remote wakeup | Yes | No¹ | Yes |
| USB remote wakeup | Yes | No¹ | No |
| Other wakeup source | Yes | No² | No |
| WAKEUPMIX | ON | OFF³ | OFF |
| MLMIX | ON | OFF | OFF |
| NICMIX | ON as needed | OFF | OFF |

### Table 22. Chip power in different LP modes

| Mode | Supply | Voltage (V) | Power (mW)¹ |
| --- | --- | --- | --- |
| BBSM | NVCC_BBSM_P8 | 1.8 | 0.14 |

### Electrical Specifications

| Parameter | Symbol | Min | Typ | Max | Unit | Conditions |
| --- | --- | --- | --- | --- | --- | --- |
| Supply Voltage | V | 1.8 | 1.8 | 1.8 | V | BBSM mode |
| Power Consumption | P | 0.14 | 0.14 | 0.14 | mW | BBSM mode |

#### Additional Information

The page discusses low power modes for the i.MX 93 Applications Processor, specifically focusing on IDLE, SUSPEND, and BBSM modes. It provides a detailed table (Table 21) comparing the behavior of various components in these modes, such as RTC, DRAM, USB PHY, clocks, and wakeup sources. Another table (Table 22) provides power consumption details for the BBSM mode. The notes section clarifies conditions for USB remote wakeup, other wakeup sources, and WAKEUPMIX behavior. The section also includes notes on automatic self-refresh for DRAM, software-controlled self-refresh, external PMIC control, and remote wakeup support.

#### Notes

- {'number': '1', 'text': 'USB remote wakeup can be "Yes" if required.'}
- {'number': '2', 'text': 'Other wakeup source can be "Yes" if required.'}
- {'number': '3', 'text': 'WAKEUPMIX can be "ON" if required.'}


## 4.3 Power supplies requirements, requirements and restrictions

*(Page 22)*

### Table 22. Chip power in different LP modes

| Mode | Supply | Voltage (V) | Power (mW)1 |
| --- | --- | --- | --- |
| SUSPEND | NVCC_GPIO, NVCC_SD2 | 3.3 | 2.65 |
| SUSPEND | NVCC_WAKEUP2 | 1.8 | 1.20 |
| SUSPEND | VDDQ_DDR | 0.6 | < 0.01 |
| SUSPEND | VDD2_DDR | 1.1 | 0.25 |
| SUSPEND | VDD_ANA*_1P8 | 1.8 | 1.85 |
| SUSPEND | VDD_ANA*_1P8 | 1.8 | 1.85 |
| SUSPEND | VDD_ANA_0P8 | 0.8 | 0.40 |
| SUSPEND | VDD_MIPI_0P8 | 0.8 | 0.65 |
| SUSPEND | VDD_USB_0P8 | 0.8 | 0.45 |
| SUSPEND | VDD_USB_3P3 | 3.3 | 0.25 |
| SUSPEND | VDD_SOC | 0.65 | 7.40 |
| Total3 |  |  | 15.1 |

#### Additional Information

The system design must comply with power-up sequence, power-down sequence, and steady state guidelines to guarantee reliable operation of the device. Deviation from these sequences may result in excessive current during power-up, prevention of device booting, or irreversible damage to the processor (worst-case scenario). Figure 4 illustrates an example about power sequence of i.MX 93 processors.

#### Notes

- {'note_number': '1', 'text': 'All the power numbers defined in the table are at 25°C. Use case dependent.'}
- {'note_number': '2', 'text': 'To achieve low power consumption values for I/O power rails in SUSPEND mode, configure IOMUX of pins to GPIO input and change PAD control settings to pull-up or pull-down depending on board design before entering SUSPEND mode.'}
- {'note_number': '3', 'text': 'Sum of the listed supply rails.'}


## 4.3.1 Power-up sequence

*(Page 23)*

#### Additional Information

{'power_sequence_diagram': {'description': 'Figure 4 shows the power sequence of i.MX 93 processors, detailing the timing and sequence of power-up and power-down steps for various voltage rails and modes.', 'modes': ['BBSM', 'PWRUP', 'RUN', 'PWRDN', 'BBSM'], 'voltage_rails': ['NVCC_BBSM_P8', 'VDD_SOC', 'VDD_ANA_OP8_VDD_MIPI_OP8_VDD_USB_OP8', 'VDD_LVDS_OP8', 'VDD_USB_OP8', 'VDD_DDR', 'VDDO_DDR (1.1 V LPDDR4)', 'VDD_DDR (0.6 V LPDDR4X)', 'NVCC_XX_P8', 'NVCC_XX_3P3', 'VDD_DDR_3P3', 'NVCC_USB_SD2', 'NVCC_USB_SD'], 'timing_parameters': ['t_ON_DEB', 't_DEB', 't_STEP', 't_POK', 't_OFF_DEB', 't_OFF_STEP']}, 'power_up_sequence': {'steps': ['Turn on NVCC_BBSM_P8', 'Assert PMIC_ON_REQ (SoC will assert this)', 'Turn on VDD_SOC digital voltage supplies', 'Turn on all VDD_*_P8 analog, PHY, and PLL supplies', 'Turn on all remaining 1.8 V supplies (includes VDD_*_P8 analog, PHY, PLL supplies, and any 1.8 V NVCC_XX I/O supplies)', 'Turn on DDR I/O supplies', 'Turn on 3.3 V supplies (includes all 3.3 V NVCC_XX I/O supplies and VDD_DDR_3P3)', 'Release POR_B (must be asserted during the entire power-up sequence)']}, 'power_down_sequence': {'steps': ['Turn off NVCC_BBSM_P8 last', 'Turn off VDD_SOC after other non-BBSM power rails or simultaneously with them', 'No sequence required for other power rails during power down']}, 'note': 'POR_B must be asserted whenever VDD_SOC is powered down, but NVCC_BBSM_P8 is powered up (when the processor is in BBSM mode).'}

#### Notes

- POR_B must be asserted whenever VDD_SOC is powered down, but NVCC_BBSM_P8 is powered up (when the processor is in BBSM mode).


## 4.4 PLL electrical characteristics

*(Page 24)*

### Table 23. PLL electrical parameters

| PLL type | Parameter | Value |
| --- | --- | --- |
| AUDIO_PLL1 | Clock output range | Up to 650 MHz |
|  | Reference clock | 24 MHz |
|  | Lock time time | 50 μs |
|  | Jitter | ±1% of output period, ≥ 50 ps |
| VIDEO_PLL1 | Clock output range | Up to 594 MHz |
|  | Reference output range | Up to 244 MHz |
|  | Reference clock | 24 MHz |
|  | Lock time time | 50 μs |
| SYS_PLL1 | Clock output range | 312.5 MHz — 1 GHz |
|  | Reference clock | 24 MHz |
|  | Lock time time | 70 μs |
| ARM_PLL | Clock output range | 800 MHz — 1700 MHz |
|  | Reference clock | 24 MHz |
|  | Lock time time | 70 μs |
| DRAM_PLL1 | Clock output range | 400 MHz — 1000 MHz |
|  | Reference clock | 24 MHz |
|  | Lock time time | 50 μs |

#### Additional Information

This page discusses the PLL electrical characteristics for the i.MX 93 Applications Processor. It includes details on various PLL types (AUDIO_PLL1, VIDEO_PLL1, SYS_PLL1, ARM_PLL, DRAM_PLL1) and their respective parameters such as clock output range, reference clock, lock time, and jitter. The section also introduces the I/O DC parameters in 4.5, mentioning GPIO, DDR (LPDDR4 and LPDDR4X modes), and LVDS I/O types.

#### Notes

- The parameters in Table 24 are guaranteed per the operating ranges in Table 12, unless otherwise noted.


## 4.5.3 LVDS DC parameters

*(Page 26)*

### LVDS DC Characteristics

| Parameter | Symbol | Test Conditions | Min | Typ | Max | Unit |
| --- | --- | --- | --- | --- | --- | --- |
| Output Differential Voltage | VOD | RLoad = 100 Ω between Pad P and Pad N | 250 | 350 | 450 | mV |
| Output High Voltage | VOH | RLoad = 100 Ω between Pad P and Pad N | 1.25 | — | 1.6 | V |
| Output Low Voltage | VOL | RLoad = 100 Ω between Pad P and Pad N | 0.9 | — | 1.25 | V |
| Offset common mode Voltage | VCM | — | 1.125 | 1.2 | 1.375 | V |
| Tri-state I/O supply supply current | Icc-ovdd | VIN=OVDD or 0 | 0.016 | — | 1700 | nA |
| Tri-state core supply supply current | Icc-vddi | VIN=VDDI or 0 | — | — | 1500 | nA |
| Power Supply current | Icc | VIN=OVDD or 0, RLoad=100 Ω | — | — | 5 | mA |

### Electrical Specifications

| Parameter | Symbol | Min | Typ | Max | Unit | Conditions |
| --- | --- | --- | --- | --- | --- | --- |
| Output Differential Voltage | VOD | 250 | 350 | 450 | mV | RLoad = 100 Ω between Pad P and Pad N |
| Output High Voltage | VOH | 1.25 | — | 1.6 | V | RLoad = 100 Ω between Pad P and Pad N |
| Output Low Voltage | VOL | 0.9 | — | 1.25 | V | RLoad = 100 Ω between Pad P and Pad N |
| Offset common mode Voltage | VCM | 1.125 | 1.2 | 1.375 | V | — |
| Tri-state I/O supply supply current | Icc-ovdd | 0.016 | — | 1700 | nA | VIN=OVDD or 0 |
| Tri-state core supply supply current | Icc-vddi | — | — | 1500 | nA | VIN=VDDI or 0 |
| Power Supply current | Icc | — | — | 5 | mA | VIN=OVDD or 0, RLoad=100 Ω |

#### Additional Information

DDRC operation is contingent upon the board's DDR design adherence to the DDR design and layout requirements stated in the hardware development guide for the i.MX 93 DDR design and layout requirements. The LVDS interface complies with TIA/EIA 644-A standard. See TIA/EIA STANDARD 644-A, "Electrical Characteristics of Low Voltage Differential Signaling (LVDS) Interface Circuits" for details. This section includes the AC parameters of the following I/O types: General Purpose I/O (GPIO), LVDS I/O.

#### Notes

- The LVDS interface complies with TIA/EIA 644-A standard. See TIA/EIA STANDARD 644-A, "Electrical Characteristics of Low Voltage Differential Signaling (LVDS) Interface Circuits" for details.
- The GPIO load circuit and output transition time waveforms are shown in Figure 5 and Figure 6.


## 4.6.1 General purpose I/O (GPIO) AC parameters

*(Page 27)*

### General purpose I/O (GPIO) AC parameters

| Symbol | Description | Min | Typ | Max | Unit | Condition |
| --- | --- | --- | --- | --- | --- | --- |
| tR | TX rise time | 3950 | — | 5950 | ps | Slew rate FSEL1 = 11b, Fast Slew Rate (1.62 V, 1.8 V, 1.98 V), Drive strength strength x1 |
| tF | TX fall time | 4140 | — | 5600 | ps | Slew rate FSEL1 = 11b, Fast Slew Rate (1.62 V, 1.8 V, 1.98 V), Drive strength strength x1 |
| tR | TX rise time | 1890 | — | 2820 | ps | Slew rate FSEL1 = 11b, Fast Slew Rate (1.62 V, 1.8 V, 1.98 V), Drive strength strength x2 |
| tF | TX fall time | 1790 | — | 2560 | ps | Slew rate FSEL1 = 11b, Fast Slew Rate (1.62 V, 1.8 V, 1.98 V), Drive strength strength x2 |
| tR | TX rise time | 675 | — | 1950 | ps | Slew rate FSEL1 = 11b, Fast Slew Rate (1.62 V, 1.8 V, 1.98 V), Drive strength strength x3 |
| tF | TX fall time | 584 | — | 1730 | ps | Slew rate FSEL1 = 11b, Fast Slew Rate (1.62 V, 1.8 V, 1.98 V), Drive strength strength x3 |
| tR | TX rise time | 521 | — | 1320 | ps | Slew rate FSEL1 = 11b, Fast Slew Rate (1.62 V, 1.8 V, 1.98 V), Drive strength strength x4 |
| tF | TX fall time | 442 | — | 748 | ps | Slew rate FSEL1 = 11b, Fast Slew Rate (1.62 V, 1.8 V, 1.98 V), Drive strength strength x4 |
| tR | TX rise time | 454 | — | 742 | ps | Slew rate FSEL1 = 11b, Fast Slew Rate (1.62 V, 1.8 V, 1.98 V), Drive strength strength x5 |

### Electrical Specifications

| Parameter | Symbol | Min | Typ | Max | Unit | Conditions |
| --- | --- | --- | --- | --- | --- | --- |
| TX rise time | tR | 3950 | — | 5950 | ps | Slew rate FSEL1 = 11b, Fast Slew Rate (1.62 V, 1.8 V, 1.98 V), Drive strength strength x1 |
| TX fall time | tF | 4140 | — | 5600 | ps | Slew rate FSEL1 = 11b, Fast Slew Rate (1.62 V, 1.8 V, 1.98 V), Drive strength strength x1 |
| TX rise time | tR | 1890 | — | 2820 | ps | Slew rate FSEL1 = 11b, Fast Slew Rate (1.62 V, 1.8 V, 1.98 V), Drive strength strength x2 |
| TX fall time | tF | 1790 | — | 2560 | ps | Slew rate FSEL1 = 11b, Fast Slew Rate (1.62 V, 1.8 V, 1.98 V), Drive strength strength x2 |
| TX rise time | tR | 675 | — | 1950 | ps | Slew rate FSEL1 = 11b, Fast Slew Rate (1.62 V, 1.8 V, 1.98 V), Drive strength strength x3 |
| TX fall time | tF | 584 | — | 1730 | ps | Slew rate FSEL1 = 11b, Fast Slew Rate (1.62 V, 1.8 V, 1.98 V), Drive strength strength x3 |
| TX rise time | tR | 521 | — | 1320 | ps | Slew rate FSEL1 = 11b, Fast Slew Rate (1.62 V, 1.8 V, 1.98 V), Drive strength strength x4 |
| TX fall time | tF | 442 | — | 748 | ps | Slew rate FSEL1 = 11b, Fast Slew Rate (1.62 V, 1.8 V, 1.98 V), Drive strength strength x4 |
| TX rise time | tR | 454 | — | 742 | ps | Slew rate FSEL1 = 11b, Fast Slew Rate (1.62 V, 1.8 V, 1.98 V), Drive strength strength x5 |

#### Additional Information

This page contains technical specifications for the general-purpose I/O (GPIO) AC parameters of the i.MX 93 Applications Processor. It includes timing parameters such as TX rise time (tR) and TX fall time (tF) under various drive strength conditions. The table provides minimum, typical, and maximum values for these parameters in picoseconds (ps).

#### Notes

- The table continues on the next page.
- All information is subject to legal disclaimers.


## General purpose I/O (GPIO) AC parameters

*(Page 28)*

### General purpose I/O (GPIO) AC parameters

| Symbol | Description | Min | Typ | Max | Unit | Condition |
| --- | --- | --- | --- | --- | --- | --- |
| tF | TX fall time | 380 | — | 554 | ps | Slew rate FSEL1 = 11b, Fast Slew Rate (1.62 V, 1.8 V, 1.98 V), Drive strength x5 |
| tR | TX rise time | 419 | — | 639 | ps | Slew rate FSEL1 = 11b, Fast Slew Rate (1.62 V, 1.8 V, 1.98 V), Drive strength x5 |
| tF | TX fall time | 349 | — | 506 | ps | Slew rate FSEL1 = 11b, Fast Slew Rate (1.62 V, 1.8 V, 1.98 V), Drive strength x6 |
| tR | TX rise time | 4030 | — | 5790 | ps | Slew rate FSEL1 = 11b, Fast Slew Rate (3 V, 3.3 V, 3.465 V), Drive strength x6 |
| tF | TX fall time | 4410 | — | 6290 | ps | Slew rate FSEL1 = 11b, Fast Slew Rate (3 V, 3.3 V, 3.465 V), Drive strength x1 |
| tR | TX rise time | 1870 | — | 2950 | ps | Slew rate FSEL1 = 11b, Fast Slew Rate (3 V, 3.3 V, 3.465 V), Drive strength x1 |
| tF | TX fall time | 1900 | — | 3310 | ps | Slew rate FSEL1 = 11b, Fast Slew Rate (1.62 V, 1.8 V, 1.98 V), Drive strength x2 |
| tR | TX rise time | 774 | — | 1930 | ps | Slew rate FSEL1 = 11b, Fast Slew Rate (3 V, 3.3 V, 3.465 V), Drive strength x2 |
| tF | TX fall time | 719 | — | 2070 | ps | Slew rate FSEL1 = 11b, Fast Slew Rate (3 V, 3.3 V, 3.465 V), Drive strength x3 |
| tR | TX rise time | 598 | — | 1360 | ps | Slew rate FSEL1 = 11b, Fast Slew Rate (3 V, 3.3 V, 3.465 V), Drive strength x3 |
| tF | TX fall time | 490 | — | 1590 | ps | Slew rate FSEL1 = 11b, Fast Slew Rate (3 V, 3.3 V, 3.465 V), Drive strength x4 |

### Electrical Specifications

| Parameter | Symbol | Min | Typ | Max | Unit | Conditions |
| --- | --- | --- | --- | --- | --- | --- |
| TX fall time | tF | 380 | — | 554 | ps | Slew rate FSEL1 = 11b, Fast Slew Rate (1.62 V, 1.8 V, 1.98 V), Drive strength x5 |
| TX rise time | tR | 419 | — | 639 | ps | Slew rate FSEL1 = 11b, Fast Slew Rate (1.62 V, 1.8 V, 1.98 V), Drive strength x5 |
| TX fall time | tF | 349 | — | 506 | ps | Slew rate FSEL1 = 11b, Fast Slew Rate (1.62 V, 1.8 V, 1.98 V), Drive strength x6 |
| TX rise time | tR | 4030 | — | 5790 | ps | Slew rate FSEL1 = 11b, Fast Slew Rate (3 V, 3.3 V, 3.465 V), Drive strength x6 |
| TX fall time | tF | 4410 | — | 6290 | ps | Slew rate FSEL1 = 11b, Fast Slew Rate (3 V, 3.3 V, 3.465 V), Drive strength x1 |
| TX rise time | tR | 1870 | — | 2950 | ps | Slew rate FSEL1 = 11b, Fast Slew Rate (3 V, 3.3 V, 3.465 V), Drive strength x1 |
| TX fall time | tF | 1900 | — | 3310 | ps | Slew rate FSEL1 = 11b, Fast Slew Rate (1.62 V, 1.8 V, 1.98 V), Drive strength x2 |
| TX rise time | tR | 774 | — | 1930 | ps | Slew rate FSEL1 = 11b, Fast Slew Rate (3 V, 3.3 V, 3.465 V), Drive strength x2 |
| TX fall time | tF | 719 | — | 2070 | ps | Slew rate FSEL1 = 11b, Fast Slew Rate (3 V, 3.3 V, 3.465 V), Drive strength x3 |
| TX rise time | tR | 598 | — | 1360 | ps | Slew rate FSEL1 = 11b, Fast Slew Rate (3 V, 3.3 V, 3.465 V), Drive strength x3 |
| TX fall time | tF | 490 | — | 1590 | ps | Slew rate FSEL1 = 11b, Fast Slew Rate (3 V, 3.3 V, 3.465 V), Drive strength x4 |

#### Additional Information

This page contains timing parameters for the general-purpose I/O (GPIO) pins of the i.MX 93 Applications Processor. The table provides AC parameters such as TX fall time (tF) and TX rise time (tR) under various conditions, including different slew rates and drive strengths.

#### Notes

- The table continues on the next page.
- All information is subject to legal disclaimers.
- The document is copyrighted by NXP B.V. (© 2025 NXP B.V. All rights reserved).


## 4.6.2 DDR I/O AC electrical characteristics

*(Page 29)*

### Table 27. General purpose I/O (GPIO) AC parameters

| Symbol | Description | Min | Typ | Max | Unit | Condition |
| --- | --- | --- | --- | --- | --- | --- |
| tR | TX rise time | 543 | — | 1040 | ps | Slew rate FSEL1 = 11b, Fast Slew Rate (3 V, 3.3 V, 3.465 V), Drive strength x5 |
| tF | TX fall time | 401 | — | 1160 | ps | Slew rate FSEL1 = 11b, Fast Slew Rate (3 V, 3.3 V, 3.465 V), Drive strength x5 |
| tR | TX rise time | 505 | — | 887 | ps | Slew rate FSEL1 = 11b, Fast Slew Rate (3 V, 3.3 V, 3.465 V), Drive strength x5 |
| tR | TX rise time | 505 | — | 887 | ps | Slew rate FSEL1 = 11b, Fast Slew Rate (3 V, 3.3 V, 3.465 V), Drive strength x6 |
| tF | TX fall time | 356 | — | 747 | ps | Slew rate FSEL1 = 11b, Fast Slew Rate (3 V, 3.3 V, 3.465 V), Drive strength x6 |

### Electrical Specifications

| Parameter | Symbol | Min | Typ | Max | Unit | Conditions |
| --- | --- | --- | --- | --- | --- | --- |
| TX rise time | tR | 543 | — | 1040 | ps | Slew rate FSEL1 = 11b, Fast Slew Rate (3 V, 3.3 V, 3.465 V), Drive strength x5 |
| TX fall time | tF | 401 | — | 1160 | ps | Slew rate FSEL1 = 11b, Fast Slew Rate (3 V, 3.3 V, 3.465 V), Drive strength x5 |
| TX rise time | tR | 505 | — | 887 | ps | Slew rate FSEL1 = 11b, Fast Slew Rate (3 V, 3.3 V, 3.465 V), Drive strength x5 |
| TX rise time | tR | 505 | — | 887 | ps | Slew rate FSEL1 = 11b, Fast Slew Rate (3 V, 3.3 V, 3.465 V), Drive strength x6 |
| TX fall time | tF | 356 | — | 747 | ps | Slew rate FSEL1 = 11b, Fast Slew Rate (3 V, 3.3 V, 3.465 V), Drive strength x6 |

#### Additional Information

The DDR I/O pads support LPDDR4/LPDDR4X operational modes. The DDRC is compliant with JEDEC-compliant SDRAMs. DDRC operation is contingent upon the board's DDR design adherence to the DDR design design and layout requirements stated in the hardware development guide for the i.MX 93 application processor. The differential output transition time waveform is shown in Figure 7.


## Output transition time waveform

*(Page 30)*

### LVDS AC parameters parameters

| Parameter | Symbol | Test Conditions | Min | Typ | Max | Unit |
| --- | --- | --- | --- | --- | --- | --- |
| Lane skew | tSKew | Rload = 100 Ω | — | 0.25 | — | ns |
| Lane skew | tSKew | Cload = 2 pF | — | — | 0.3 | Unit Interval (UI) |
| Transition Low to High time | tTLH | Cload = 2 pF | — | — | 0.3 | Unit Interval (UI) |
| Transition High to Low time | tTHL | Cload = 2 pF | — | — | 0.3 | Unit Interval (UI) |
| Operating data rate | f | — | — | — | 560 | Mbps |
| Offset peak to peak voltage imbalance | VOSPP | — | — | — | 150 | mV |
| Tri-state I/O supply current imbalance | Icc-ovdd | VIN=OVDD or 0 | 0.016 | — | 1700 | nA |
| Tri-state I/O core supply current imbalance | Icc-vddi | VIN=VDDI or 0 | — | — | 1500 | nA |
| Tri-state core supply supply current | Icc-vddi | VIN=VDDI or 0 | — | — | 1500 | nA |
| Power Supply Supply current | Icc | VIN=OVDD or 0 | — | — | 5 | mA |

### Electrical Specifications

| Parameter | Symbol | Min | Typ | Max | Unit | Conditions |
| --- | --- | --- | --- | --- | --- | --- |
| Lane skew | tSKew | — | 0.25 | — | ns | Rload = 100 Ω |
| Lane skew | tSKew | — | — | 0.3 | Unit Interval (UI) | Cload = 2 pF |
| Transition Low to High time | tTLH | — | — | 0.3 | Unit Interval (UI) | Cload = 2 pF |
| Transition High to Low time | tTHL | — | — | 0.3 | Unit Interval (UI) | Cload = 2 pF |
| Operating data rate | f | — | — | 560 | Mbps | — |
| Offset peak to peak voltage imbalance | VOSPP | — | — | 150 | mV | — |
| Tri-state I/O supply current imbalance | Icc-ovdd | 0.016 | — | 1700 | nA | VIN=OVDD or 0 |
| Tri-state I/O core supply current imbalance | Icc-vddi | — | — | 1500 | nA | VIN=VDDI or 0 |
| Tri-state core supply supply current | Icc-vddi | — | — | 1500 | nA | VIN=VDDI or 0 |
| Power Supply Supply current | Icc | — | — | 5 | mA | VIN=OVDD or 0 |

#### Additional Information

This page discusses the AC parameters of LVDS (Low Voltage Differential Signaling) for the i.MX 93 Applications Processor. It includes a waveform diagram (Figure 7) illustrating output transition times and a table (Table 28) detailing AC parameters such as lane skew, transition times, operating data rate, voltage imbalance, and supply currents.

#### Notes

- {'number': '1', 'text': 'tSKew is the differential time at Vod = 0 voltage between any two different channels.'}
- {'number': '2', 'text': 'This is the typical maximum absolute delay between any two different channels.'}
- {'number': '3', 'text': 'Measurement value levels are 20–80% from output voltage.'}
- {'number': '4', 'text': 'This measurement value is dependent on the operating data rate.'}
- {'number': '5', 'text': 'This is the maximum bit rate that is defined by the supported display types.'}


## 4.7 Differential I/O output buffer impedance

*(Page 31)*

### Table 29. Reset timing parameters

| ID | Parameter | Min | Max | Unit |
| --- | --- | --- | --- | --- |
| CC1 | Duration of POR_B to be qualified as valid. | 1 | — | RTC_XTALI cycle |

### Table 30. WDOGx_B timing parameters

| ID | Parameter | Min | Max | Unit |
| --- | --- | --- | --- | --- |
| CC3 | Duration of WDOG1_B Assertion | 1 | — | RTC_XTALI cycle |

### Electrical Specifications

| Parameter | Symbol | Min | Typ | Max | Unit | Conditions |
| --- | --- | --- | --- | --- | --- | --- |
| POR_B rise/fall times |  | 400 µs |  | 400 µs | µs | Must be 400 µs or less |
| RTC_XTALI frequency |  | 32 kHz |  | 32 kHz | kHz | Approximately 32 kHz |
| RTC_XTALI cycle duration |  | 30 µs |  | 30 µs | µs | One period or approximately 30 µs |

#### Additional Information

The Differential CCM interface is designed to be compatible with TIA/EIA 644-A standard. DDR output driver and ODT impedances are controlled across PVT using ZQ calibration procedure with a 120 ohm ±1% resistor to ground. Programmable drive strength and ODT impedance targets available in the NXP DDR tool are detailed in the device IBIS model. Impedance deviation (calibration accuracy) is ±10% (Maximum/Minimum impedance) across PVT. This section contains the timing and electrical parameters for the modules in each i.MX 93 processor. Figure 8 shows the reset timing diagram and Table 29 lists the timing parameters. Figure 9 shows the WDOG reset timing diagram and Table 30 lists the timing parameters. RTC_XTALI is approximately 32 kHz. RTC_XTALI cycle is one period or approximately 30 µs.

#### Notes

- POR_B rise/fall times must be 400 µs or less.
- RTC_XTALI is approximately 32 kHz. RTC_XTALI cycle is one period or approximately 30 µs.


## 4.8.3 JTAG timing parameters

*(Page 32)*

### Electrical Specifications

| Parameter | Symbol | Min | Typ | Max | Unit | Conditions |
| --- | --- | --- | --- | --- | --- | --- |
| JTAG Test Clock Input Timing | SJ1 | Not specified | Not specified | Not specified | ns | Refer to Figure 10 |
| JTAG Boundary Scan Timing | SJ2, SJ3, SJ4, SJ5, SJ6, SJ7, SJ8, SJ9, SJ10, SJ11, SJ14 | Not specified | Not specified | Not specified | ns | Refer to Figure 11 and Figure 12 |

#### Additional Information

This page discusses JTAG timing parameters for the i.MX 93 Applications Processor. It includes timing diagrams for JTAG test clock input (Figure 10), boundary scan timing (Figure 11), and test access port timing (Figure 12). The DSE[5:0] = 001111 and FSEL[1:0] = 11 are required drive settings to meet the timing. The WDOGx_B output signals are muxed through the IOMUX, as noted in the 'NOTE' section.

#### Notes

- WDOGx_B output signals (for each one of the Watchdog modules) do not have dedicated pins but are muxed out through the IOMUX. See the IOMUXC chapter of the i.MX 93 Applications Processor Reference Manual (IMX93RM) for detailed information.


## 4.8.5 DDR SDRAM-specific parameters (LPDDR4/LPDDR4X)

*(Page 34)*

### SWD timing parameters

| Symbol | Description | Min | Max | Unit |
| --- | --- | --- | --- | --- |
| S0 | SWD_CLK frequency | -- | 50 | MHz |
| S1 | SWD_CLK cycle time | 20 | -- | ns |
| S2 | SWD_CLK pulse width | 10 | -- | ns |
| S3 | Input data setup time | 5 | -- | ns |
| S4 | Input data hold time | 5 | -- | ns |
| S5 | Output data hold valid time | 5 | 14 | ns |
| S5 | Output data valid time | -- | 14 | ns |
| S6 | Output high impedance time | -- | 14 | ns |
| S7 | Output data invalid time | 0 | -- | ns |

### Electrical Specifications

| Parameter | Symbol | Min | Typ | Max | Unit | Conditions |
| --- | --- | --- | --- | --- | --- | --- |
| SWD_CLK frequency | S0 | -- |  | 50 | MHz | N/A |
| SWD_CLK cycle time | S1 | 20 |  | -- | ns | N/A |
| SWD_CLK pulse width | S2 | 10 |  | -- | ns | N/A |
| Input data setup time | S3 | 5 |  | -- | ns | N/A |
| Input data hold time | S4 | 5 |  | -- | ns | N/A |
| Output data hold valid time | S5 | 5 |  | 14 | ns | N/A |
| Output data valid time | S5 | -- |  | 14 | ns | N/A |
| Output high impedance time | S6 | -- |  | 14 | ns | N/A |
| Output data invalid time | S7 | 0 |  | -- | ns | N/A |

#### Additional Information

This page discusses the SWD (Serial Wire Debug) timing parameters for the i.MX 93 Applications Processor. It includes a timing diagram (Figure 13) and a table (Table 32) detailing the timing parameters. The text also mentions that the i.MX 93 Family of processors is designed and tested to work with JEDEC JESD209-compliant LPDDR4/LPDDR4X memory.

#### Notes

- 1. Input timing assumes an input signal slew rate of 3 ns (20%/80%).
- 2. Timing valid for maximum external load CL = 25 pF, which is assumed to be a 10 pF load at the end of a 50 Ω, unterminated, 5-inch microstrip trace on standard FR4 (3.3 pF/inch), (25 pF total with margin). For best signal integrity, the series resistance in the transmission line can be equal to the selected RDS(ON) of the I/O pad output.


## 4.8.5.1 Clock/data/command/address/address pin allocations

*(Page 35)*

### i.MX 93 DRAM controller supported SDRAM configurations

| Parameter | LPDDR4/LPDDR4X |
| --- | --- |
| Number of Controllers | 1 |
| Number of Channels | 1 |
| Number of Chip Selects | 2 |
| Bus Width | 16-bit |
| Maximum supported data rate |  |
| Low drive mode | 1866 MT/s |
| Nominal drive mode | 2880 MT/s |
| Overdrive mode | 3733 MT/s |

### Electrical Specifications

| Parameter | Symbol | Min | Typ | Max | Unit | Conditions |
| --- | --- | --- | --- | --- | --- | --- |
| Maximum supported data rate (Low drive mode) |  | 1866 | 1866 | 1866 | MT/s |  |
| Maximum supported data rate (Nominal drive mode) |  | 2880 | 2880 | 2880 | MT/s |  |
| Maximum supported data rate (Overdrive mode) |  | 3733 | 3733 | 3733 | MT/s |  |
| Maximum data rate for 9x9 mm package |  | 3200 | 3200 | 3200 | MT/s | LPDDR4/LPDDR4X |

#### Additional Information

JEDEC LPDDR4 Specification JESD209-4B, February 2017 JEDEC LPDDR4X Specification JESD209-1, January 2017 Timing diagrams and tolerances required to work with these memories are specified in the respective documents and are not reprinted here. Meeting the necessary timing requirements for a DDR memory system is highly dependent on the components chosen and the design layout of the system as a whole. NXP cannot cover all requirements needed to achieve a design that meets full system performance over temperature, voltage, and part variation. Factors affecting DDR performance include PCB material, trace routing, dielectric material, routing layers, power rail placement, VIA placement, GND and Supply planes layout, and DDR controller/PHY register settings. NXP recommends duplicating NXP validated design layouts for critical power rails, bulk/decoupling capacitors, and DDR trace routing. Processors that demonstrate full DDR performance on NXP validated designs but do not function on customer designs are not considered marginal parts. Customers bear responsibility for properly designing the PCB, simulating operating conditions, and validating the system. These processors use generic names for clock, data, and command address bus signals. Section 4.9 provides information on display and graphic interfaces. Section 4.9.1 describes MIPI D-PHY electrical characteristics.

#### Notes

- {'note_number': '1', 'text': 'For 9 x 9 mm package, the maximum data rate of LPDDR4x/LPDDR4 is 3200 MT/s.'}


## 4.9.1.1 MIPI HS-TX specifications

*(Page 36)*

### Table 34. MIPI high-speed transmitter DC specifications

| Symbol | Parameter | Min | Typ | Max | Unit |
| --- | --- | --- | --- | --- | --- |
| V_CMTX^1 | High Speed Transmit Static Common Common Mode Voltage | 150 | 200 | 250 | mV |
| \|ΔV_CMTX\|(1,0) | V_CMTX mismatch when Output is Differential-1 or Differential-0 | — | — | 5 | mV |
| \|V_OD\|^1 | High Speed Transmit Differential Voltage | 140 | 200 | 270 | mV |
| \|ΔV_OD\| | V_OD mismatch when when Output is Differential-1 or Differential-0 | — | — | 14 | mV |
| V_OHHS | High Speed Output High Voltage | — | — | 360 | mV |
| V_OHHS | High Single Ended Output High Voltage | — | — | 60 | Ω |
| Z_OS | Single Ended Output Output Impedance | 40 | 50 | 62.5 | Ω |
| ΔZ_OS | Single Ended Output Output Impedance Mismatch | — | — | 10 | % |

### Table 35. MIPI high-speed transmitter AC specifications

| Symbol | Parameter | Min | Typ | Max | Unit |
| --- | --- | --- | --- | --- | --- |
| ΔV_CMTX(HF) | Common-level variations above 450 MHz | — | — | 15 | mVRMS |
| ΔV_CMTX(LF) | Common-level variation between 50-450 MHz | — | — | 25 | mVPEAK |
| ΔV_CMTX(LF) | Common-level variation between 50-450 MHz | — | — | 25 | mVPEAK |
| t_R and t_F^1 | Rise Time and Fall Time (20% to 80%) | 100 | — | 0.35 x UI | ps |

### Table 36. MIPI high-speed receiver DC specifications

| Symbol | Parameter | Min | Typ | Max | Unit |
| --- | --- | --- | --- | --- | --- |
| V_IDTH | Differential input high voltage voltage threshold | — | — | 70 | mV |
| V_IDTL | Differential input low voltage threshold | -70 | — | 70 | mV |
| V_IDTLHS | Differential input low voltage threshold | -70 | — | — | mV |
| V_IHHS | Single ended input high voltage | — | — | 460 | mV |
| V_ILHS | Single ended input low voltage | -40 | — | — | mV |
| V_CMRXDC | Input common mode voltage | 70 | — | 330 | mV |
| Z_ID | Differential input impedance | 80 | 100 | 125 | Ω |

### Electrical Specifications

| Parameter | Symbol | Min | Typ | Max | Unit | Conditions |
| --- | --- | --- | --- | --- | --- | --- |
| High Speed Transmit Static Common Common Mode Voltage | V_CMTX^1 | 150 | 200 | 250 | mV | Driving into load impedance anywhere in the Z_PD range |
| V_CMTX mismatch when Output is Differential-1 or Differential-0 | |ΔV_CMTX|(1,0) | — | — | 5 | mV | — |
| High Speed Transmit Differential Voltage | |V_OD|^1 | 140 | 200 | 270 | mV | Driving into load impedance anywhere in the Z_PD range |
| V_OD mismatch when Output is Differential-1 or Differential-0 | |ΔV_OD| | — | — | 14 | mV | — |
| High Speed Output High Voltage | V_OHHS | — | — | 360 | mV | — |
| High Single Ended Output High Voltage | V_OHHS | — | — | 60 | Ω | — |
| Single Ended Output Output Impedance | Z_OS | 40 | 50 | 62.5 | Ω | — |
| Single Ended Output Output Impedance Mismatch | ΔZ_OS | — | — | 10 | % | — |
| Common-level variations above 450 MHz | ΔV_CMTX(HF) | — | — | 15 | mVRMS | — |
| Common-level variation between 50-450 MHz | ΔV_CMTX(LF) | — | — | 25 | mVPEAK | — |
| Rise Time and Fall Time (20% to 80%) | t_R and t_F^1 | 100 | — | 0.35 x UI | ps | UI is the long-term average unit interval |
| Differential input high voltage voltage threshold | V_IDTH | — | — | 70 | mV | — |
| Differential input low voltage threshold | V_IDTL | -70 | — | 70 | mV | — |
| Differential input low voltage threshold | V_IDTLHS | -70 | — | — | mV | — |
| Single ended input high voltage | V_IHHS | — | — | 460 | mV | — |
| Single ended input low voltage | V_ILHS | -40 | — | — | mV | — |
| Input common mode voltage | V_CMRXDC | 70 | — | 330 | mV | — |
| Differential input impedance | Z_ID | 80 | 100 | 125 | Ω | — |

#### Additional Information

This page contains detailed specifications for the MIPI HS-TX and MIPI HS-RX interfaces of the i.MX 93 Applications Processor. It includes DC and AC specifications for both the transmitter and receiver, covering parameters such as voltage levels, impedance, and timing characteristics.

#### Notes

- 1. Value when driving into load impedance anywhere in the Z_PD range.
- 1. UI is the long-term average unit interval.


## 4.9.1.3 MIPI-LP-TX specifications

*(Page 37)*

### Table 37. MIPI high-speed receiver AC specifications

| Symbol | Parameter | Min | Typ | Max | Unit |
| --- | --- | --- | --- | --- | --- |
| ΔV_CM RX(HF)1 | Common mode interference beyond 450 MHz | -- | -- | 50 | mV |
| ΔV_CM RX(LF) | Common mode interference between 50 and 450 MHz | -25 | -- | 25 | mV |
| C_CM | Common mode termination | -- | -- | 60 | pF |

### Table 38. MIPI low-power transmitter DC specifications

| Symbol | Parameter | Min | Typ | Max | Unit |
| --- | --- | --- | --- | --- | --- |
| V_OH1 | Thevenin Output High Level | 1.1 | 1.2 | 1.3 | V |
| V_OL | Thevenin Output Low Level | -50 | -- | 50 | mV |
| Z_OLP2 | Output Impedance of Low Power Power Transmitter | 110 | -- | -- | Ω |

### Table 39. MIPI low-power transmitter AC specifications

| Symbol | Parameter | Min | Typ | Max | Unit |
| --- | --- | --- | --- | --- | --- |
| T_RLP/T_FLP1 | 15% to 85% Rise Time and Fall Time | -- | -- | 25 | ns |
| T_REOT2,3 | 30% to 85% Rise Time and Fall Time | -- | -- | 35 | ns |
| T_LP-PULSE-TX4 | Pulse width of LP exclusive-OR clock: First LP exclusive-OR clock pulse after Stop state or last pulse before Stop state | 40 | -- | -- | ns |
| T_LP-PULSE-TX4 | Pulse width of LP exclusive-OR clock: All other pulses | 20 | -- | -- | ns |
| T_LP-PER-TX | Period of LP exclusive-OR clock | 90 | -- | -- | ns |
| δV/δtSR1,5,6,7 | Slew Rate @ CLOAD = 0 pF | 25 | -- | 500 | mV/ns |
| δV/δtSR1,5,6,7 | Slew Rate @ CLOAD = 5 pF | 25 | -- | 300 | mV/ns |
| δV/δtSR1,5,6,7 | Slew Rate @ CLOAD = 20 pF | 25 | -- | 250 | mV/ns |
| δV/δtSR1,5,6,7 | Slew Rate @ CLOAD = 70 pF | 25 | -- | 150 | mV/ns |
| C_LOAD | Load Capacitance | 0 | -- | 70 | pF |

### Electrical Specifications

| Parameter | Symbol | Min | Typ | Max | Unit | Conditions |
| --- | --- | --- | --- | --- | --- | --- |
| Common mode interference beyond 450 MHz | ΔV_CM RX(HF) | -- | -- | 50 | mV | Peak amplitude of a sine wave superimposed on the receiver inputs. |
| Common mode interference between 50 and 450 MHz | ΔV_CM RX(LF) | -25 | -- | 25 | mV |  |
| Common mode termination | C_CM | -- | -- | 60 | pF |  |
| Thevenin Output High Level | V_OH | 1.1 | 1.2 | 1.3 | V | Core supply variation limited from 1.1 V to 1.3 V. |
| Thevenin Output Low Level | V_OL | -50 | -- | 50 | mV |  |
| Output Impedance of Low Power Power Transmitter | Z_OLP | 110 | -- | -- | Ω | No specified maximum; meets TRLP/TFLP specification. |
| 15% to 85% Rise Time and Fall Time | T_RLP/T_FLP | -- | -- | 25 | ns |  |
| 30% to 85% Rise Time and Fall Time | T_REOT | -- | -- | 35 | ns |  |
| Pulse width of LP exclusive-OR clock (first pulse after Stop state or last pulse before Stop state) | T_LP-PULSE-TX | 40 | -- | -- | ns |  |
| Pulse width of LP exclusive-OR clock (all other pulses) | T_LP-PULSE-TX | 20 | -- | -- | ns |  |
| Period of LP exclusive-OR clock | T_LP-PER-TX | 90 | -- | -- | ns |  |
| Slew Rate @ CLOAD = 0 pF | δV/δtSR | 25 | -- | 500 | mV/ns |  |
| Slew Rate @ CLOAD = 5 pF | δV/δtSR | 25 | -- | 300 | mV/ns |  |
| Slew Rate @ CLOAD = 20 pF | δV/δtSR | 25 | -- | 250 | mV/ns |  |
| Slew Rate @ CLOAD = 70 pF | δV/δtSR | 25 | -- | 150 | mV/ns |  |
| Load Capacitance | C_LOAD | 0 | -- | 70 | pF | Includes low equivalent transmission line capacitance. |

#### Additional Information

This page contains specifications for the MIPI high-speed receiver and low-power transmitter in the i.MX 93 Applications Processor. It includes AC and DC specifications for both components, detailing parameters such as common mode interference, output levels, impedance, rise/fall times, pulse widths, and slew rates under various load capacitance conditions.

#### Notes

- 1. ΔV_CM RX(HF) is the peak amplitude of a sine wave superimposed on the receiver inputs.
- 2. This specification can only be met when limiting the core supply variation from 1.1 V to 1.3 V.
- 3. Although there is no specified maximum for Z_OLP, the LP transmitter output impedance ensures the TRLP/TFLP specification is met.
- 4. The pulse width of the LP exclusive-OR clock is specified for the first pulse after Stop state or last pulse before Stop state, and for all other pulses.
- 5. The rise time of T_REOT starts from the line capacitance that can be up to 50 pF for a transmission line with 2 ns delay.
- 6. The HS common-level at the moment of the differential amplitude drops below 70 mV, due to stopping the differential drive.
- 7. C_LOAD includes the low equivalent transmission line capacitance. The capacitance of TX and RX are assumed to always be < 10 pF.


## 4.9.1.4 MIPI LP-RX specifications

*(Page 38)*

### Table 40. MIPI low power receiver DC specifications

| Symbol | Parameter | Min | Typ | Max | Unit |
| --- | --- | --- | --- | --- | --- |
| V_IH | Logic 1 input voltage | 740 | — | — | mV |
| V_IL | Logic 0 input voltage, not in ULP state | — | — | 550 | mV |
| V_IL_ULPS | Logic 0 input voltage, ULP state | — | — | 300 | mV |
| V_HYST | Input hysteresis | 25 | — | — | mV |

### Table 41. MIPI low power receiver AC specifications

| Symbol | Parameter | Min | Typ | Max | Unit |
| --- | --- | --- | --- | --- | --- |
| eSPIKE^1,2 | Input pulse rejection | — | — | 300 | V.ps |
| T_MIN-RX^3 | Minimum pulse width response | 20 | — | — | ns |
| V_INT | Peak Interference amplitude | — | — | 200 | mV |
| f_INT | Interference frequency | 450 | — | — | MHz |

### Table 42. MIPI contention detector DC specifications

| Symbol | Parameter | Min | Typ | Max | Unit |
| --- | --- | --- | --- | --- | --- |
| V_IHCD | Logic 1 contention threshold | 450 | — | — | mV |
| V_ILCD | Logic 0 contention threshold | — | — | 200 | mV |

### Electrical Specifications

| Parameter | Symbol | Min | Typ | Max | Unit | Conditions |
| --- | --- | --- | --- | --- | --- | --- |
| Logic 1 input voltage | V_IH | 740 | — | — | mV | — |
| Logic 0 input voltage, not in ULP state | V_IL | — | — | 550 | mV | — |
| Logic 0 input voltage, ULP state | V_IL_ULPS | — | — | 300 | mV | — |
| Input hysteresis | V_HYST | 25 | — | — | mV | — |
| Input pulse rejection | eSPIKE^1,2 | — | — | 300 | V.ps | — |
| Minimum pulse width response | T_MIN-RX^3 | 20 | — | — | ns | — |
| Peak Interference amplitude | V_INT | — | — | 200 | mV | — |
| Interference frequency | f_INT | 450 | — | — | MHz | — |
| Logic 1 contention threshold | V_IHCD | 450 | — | — | mV | — |
| Logic 0 contention threshold | V_ILCD | — | — | 200 | mV | — |

#### Additional Information

3. With an additional load capacitance CCM between 0 to 60 pF on the termination center center tap at RX side of the lane. 4. This parameter value can be lower than TLPX due to differences in rise vs. fall signal slopes and trip levels and mismatches between Dp and Dn LP transmitters. Any LP exclusive-OR pulse observed during HS EoT (transition from HS level to LP-11) is glitch behavior as described in Low-Power Receiver-OR section. 5. When the output voltage is between 15% and below 85% of the fully settled LP signal signal levels. 6. Measured as average across any 50 mV segment of the output fully settled LP signal signal levels. 7. This value represents a corner point in a piecewise linear curve. 4.9.1.4 MIPI LP-RX specifications 4.9.1.5 MIPI CD-CD specifications 4.9.2 LCD Controller (LCDIF) timing parameters The DSE[5:0] = 001111 and FSEL[1:0] = 11 are required drive settings to meet the timing. Figure 14 shows the LCDIF timing and Table 43 lists the timing parameters.

#### Notes

- 1. Time-voltage integration of a spike above V_IL when in LP-0 state or below VIH when in LP-1 state.
- 2. An impulse below this value will not change the receiver state.
- 3. An input MIPI LP greater than this value shall toggle the output.


## 4.10 Audio

*(Page 39)*

### Table 43. LCD timing parameters

| ID | Parameter | Symbol | Min | Max | Unit |
| --- | --- | --- | --- | --- | --- |
| L1 | LCD pixel clock frequency | fCLK(LCD) | - | 80 | MHz |
| L | LCD pixel clock frequency | fCLKH(LCD) | - | 80 | MHz |
| L2 | LCD pixel clock high (falling edge capture) | tCLKH(LCD) | 5 | - | ns |
| L3 | LCD pixel clock low (rising edge capture) | tCLKL(LCD) | 5 | - | ns |
| L4 | LCD pixel clock high to data valid valid (falling edge capture) | td(CLKH-DV) | -1.5 | 1.5 | ns |
| L5 | LCD pixel clock low to data valid (rising edge capture) | td(CLKL-DV) | -1.5 | 1.5 | ns |
| L6 | LCD pixel clock high to control signal valid (falling edge capture) | td(CLKH-CTRLV) | -1.5 | 1.5 | ns |
| L7 | LCD pixel clock low to control signal valid (rising edge capture) | td(CLKL-CTRLV) | -1.5 | 1.5 | ns |

### Electrical Specifications

| Parameter | Symbol | Min | Typ | Max | Unit | Conditions |
| --- | --- | --- | --- | --- | --- | --- |
| Output timing valid for maximum external load | CL | 25 |  | 25 | pF | Assumed to be a 10 pF load at the end of a 50 ohm, unterminated, 5-inch microstrip trace on standard FR4 (3.3 pF/inch), (25 pF total with margin). For best signal integrity, the series timing resistance in the transmission line should be equal to the selected RDSON of the I/O pad output driver. |
| Input series timing resistance | Slew rate | 3 |  | 3 | ns (20%/80%) | Assumes an input signal slew rate of 3 ns (20%/80%). |
| Maximum frequency supported | fCLK(LCD) |  |  | 52 | MHz | When NVCC_xxxx operating at 3.3 V. |

### SAI_RCR2 (`Not specified`)

| Bits | Field | Access | Description |
| --- | --- | --- | --- |
| BCI | Bit Clock Invert | RW | Inverts the polarity of the serial clock (SAI_BCLK). |
| BCP | Bit Clock Polarity | RW | Inverts the polarity of the serial clock (SAI_BCLK). |

### SAI_TCR4 (`Not specified`)

| Bits | Field | Access | Description |
| --- | --- | --- | --- |
| FSP | Frame Sync Polarity | RW | Inverts the polarity of the frame sync signal (SAI_FS). |

#### Additional Information

This section provides information about the audio subsystem. SAI switching specifications are provided for both Controller and Target modes. All timings are given for non-inverted serial clock polarity and non-inverted frame sync. DSE[5:0] = 001111 and FSEL[1:0] = 11 are required drive settings to meet the timing. For 50 MHz BCLK operation, BCLK and SYNC must always be in the same direction as the data (source synchronous). SAI transmitter receiver must be in asynchronous mode with BCLK and SYNC configuration as outputs. SAI must be in mode with BCLK and SYNC configuration as inputs. SAI must be in synchronous mode with SAI_RCR2[BCI] = 1.

#### Notes

- 1. Output timing valid for maximum external load CL = 25 pF, which is assumed to be a 10 pF load at the end of a 50 ohm, unterminated, 5-inch microstrip trace on standard FR4 (3.3 pF/inch), (25 pF total with margin). For best signal integrity, the series timing resistance in the transmission line should be equal to the selected RDSON of the I/O pad output driver.
- 2. Input series timing resistance assumes an input signal slew rate of 3 ns (20%/80%).
- 3. The maximum frequency supported is 52 MHz when NVCC_xxxx operating at 3.3 V.


## Controller mode SAI timing

*(Page 40)*

### Controller mode SAI timing (50 MHz)

| Num | Characteristic | Min | Max | Unit |
| --- | --- | --- | --- | --- |
| S1 | SAI_MCLK cycle time | 20 | — | ns |
| S2 | SAI_MCLK pulse width high/low | 40% | 60% | MCLK period |
| S3 | SAI_BCLK cycle time | 20 | — | ns |
| S4 | SAI_BCLK pulse width high/low | 40% | 60% | BCLK period |
| S5 | SAI_BCLK to SAI_FS output valid | — | 3 | BCLK period |
| S5 | SAI_BCLK to SAI_FS output valid | — | 3 | ns |
| S6 | SAI_BCLK to SAI_FS output invalid | -2 | — | ns |
| S7 | SAI_BCLK to SAI_TXD valid | — | 3 | ns |
| S8 | SAI_BCLK to SAI_TXD invalid | -2 | — | ns |
| S9 | SAI_RXD/SAI_FS input setup before SAI_BCLK | 3 | — | ns |
| S10 | SAI_RXD/SAI_FS input hold after SAI_BCLK | 2 | — | ns |

### Controller mode SAI timing (25 MHz)

| Num | Characteristic | Min | Max | Unit |
| --- | --- | --- | --- | --- |
| S1 | SAI_MCLK cycle time | 40 | — | ns |
| S2 | SAI_MCLK pulse width high/low | 40% | 60% | MCLK period |
| S3 | SAI_BCLK cycle time | 40 | — | ns |
| S4 | SAI_BCLK pulse width high/low | 40% | 60% | BCLK period |
| S5 | SAI_BCLK to SAI_FS output valid | — | 3 | ns |
| S6 | SAI_BCLK to SAI_FS output invalid | -2 | — | ns |
| S7 | SAI_BCLK to SAI_TXD valid | — | 3 | ns |
| S8 | SAI_BCLK to SAI_TXD invalid | -2 | — | ns |
| S9 | SAI_RXD/SAI_FS input setup before SAI_BCLK | 8 | — | ns |
| S10 | SAI_RXD/SAI_FS input hold after SAI_BCLK | 0 | — | ns |

### Electrical Specifications

| Parameter | Symbol | Min | Typ | Max | Unit | Conditions |
| --- | --- | --- | --- | --- | --- | --- |
| SAI_MCLK cycle time (50 MHz) |  | 20 |  | — | ns |  |
| SAI_MCLK pulse width high/low (50 MHz) |  | 40% |  | 60% | MCLK period |  |
| SAI_BCLK cycle time (50 MHz) |  | 20 |  | — | ns |  |
| SAI_BCLK pulse width high/low (50 MHz) |  | 40% |  | 60% | BCLK period |  |
| SAI_BCLK to SAI_FS output valid (50 MHz) |  | — |  | 3 | BCLK period |  |
| SAI_BCLK to SAI_FS output valid (50 MHz) |  | — |  | 3 | ns |  |
| SAI_BCLK to SAI_FS output invalid (50 MHz) |  | -2 |  | — | ns |  |
| SAI_BCLK to SAI_TXD valid (50 MHz) |  | — |  | 3 | ns |  |
| SAI_BCLK to SAI_TXD invalid (50 MHz) |  | -2 |  | — | ns |  |
| SAI_RXD/SAI_FS input setup before SAI_BCLK (50 MHz) |  | 3 |  | — | ns |  |
| SAI_RXD/SAI_FS input hold after SAI_BCLK (50 MHz) |  | 2 |  | — | ns |  |
| SAI_MCLK cycle time (25 MHz) |  | 40 |  | — | ns |  |
| SAI_MCLK pulse width high/low (25 MHz) |  | 40% |  | 60% | MCLK period |  |
| SAI_BCLK cycle time (25 MHz) |  | 40 |  | — | ns |  |
| SAI_BCLK pulse width high/low (25 MHz) |  | 40% |  | 60% | BCLK period |  |
| SAI_BCLK to SAI_FS output valid (25 MHz) |  | — |  | 3 | ns |  |
| SAI_BCLK to SAI_FS output invalid (25 MHz) |  | -2 |  | — | ns |  |
| SAI_BCLK to SAI_TXD valid (25 MHz) |  | — |  | 3 | ns |  |
| SAI_BCLK to SAI_TXD invalid (25 MHz) |  | -2 |  | — | ns |  |
| SAI_RXD/SAI_FS input setup before SAI_BCLK (25 MHz) |  | 8 |  | — | ns |  |
| SAI_RXD/SAI_FS input hold after SAI_BCLK (25 MHz) |  | 0 |  | — | ns |  |

#### Additional Information

This page contains timing specifications for the SAI (Serial Audio Interface) module in controller mode for two different clock frequencies: 50 MHz and 25 MHz. The tables detail various timing parameters such as cycle times, pulse widths, and setup/hold times for signals like SAI_MCLK, SAI_BCLK, SAI_FS, SAI_TXD, and SAI_RXD.

#### Notes

- To achieve 50 MHz for BCLK operation, clock must be set in feedback mode.
- Input timing assumes an input signal slew rate of 3 ns (20%/80%).
- Output timing valid for maximum external load CL = 25 pF, which is assumed to be a 10 pF load at the end of a 50 ohm unterminated, 5-inch microstrip trace on standard FR4 (3.3 pF/inch, 25 pF total with margin). For best signal integrity, the series resistance in the transmission line should equal the selected RDSON of the I/O pad output driver.


## SAI timing—Controller mode

*(Page 41)*

### Target mode SAI timing (25 MHz)

| Num | Characteristic | Min | Max | Unit |
| --- | --- | --- | --- | --- |
| S11 | SAI_BCLK cycle time (input) | 40 | — | ns |
| S12 | SAI_BCLK pulse width high/low (input) | 40% | 60% | BCLK period |
| S13 | SAI_FS input setup before SAI_BCLK | 3 | — | ns |
| S14 | SAI_FS input hold after SAI_BCLK | 2 | — | ns |
| S14 | SAI_BCLK to SAI_TXD/SAI_FS output valid | — | 9 | ns |
| S15 | SAI_BCLK to SAI_TXD/SAI_FS output invalid | 0 | — | ns |
| S16 | SAI_RXD setup before SAI_BCLK | 3 | — | ns |
| S17 | SAI_RXD hold after SAI_BCLK | 2 | — | ns |
| S18 | SAI_FS input assertion to SAI_TXD output valid | — | 25 | ns |

### Electrical Specifications

| Parameter | Symbol | Min | Typ | Max | Unit | Conditions |
| --- | --- | --- | --- | --- | --- | --- |
| Output timing valid for maximum external load | CL | 25 pF |  | 25 pF | pF | Assumed to be a 10 pF load at the end of a 50 ohm unterminated, 5-inch microstrip trace on standard FR4 (3.3 pF/inch). |
| Series resistance in transmission line | R | 50 ohm |  | 50 ohm | ohm | Should be equal to the selected RDSON of the I/O pad output driver for best signal integrity. |

#### Additional Information

This page discusses the SAI (Serial Audio Interface) timing specifications for the i.MX 93 Applications Processor in Controller mode. It includes timing diagrams and a table detailing the target mode SAI timing parameters at 25 MHz. The text specifies conditions for output timing, including maximum external load capacitance and transmission line resistance for optimal signal integrity.

#### Notes

- 1. Input timing assumes an input signal slew rate of 3 ns (20%/80%).
- 2. Output timing valid for maximum external load CL = 25 pF, which is assumed to be a 10 pF load at the end of a 50 ohm unterminated, 5-inch microstrip trace on standard FR4 (3.3 pF/inch). For best signal integrity, the series resistance in the transmission line should be equal to the selected RDSON of the I/O pad output driver.
- 3. Applies to the first bit in each frame only if the TCR4[FSE] bit is clear.


## 4.10.2 SPDIF timing parameters

*(Page 42)*

### SPDIF timing timing parameters

| Parameter | Symbol | Timing Parameter Range | Unit |
| --- | --- | --- | --- |
| SPDIF_IN Skew: asynchronous inputs, no specs apply | — | ['—', '0.7'] | ns |
| SPDIF_OUT output (Load = 50 pf) | — | ['—', '1.5'] | ns |
| Skew (Load = 50 pf) | — | ['—', '24.2'] | ns |
| Transition rising rising | — | ['—', '31.3'] | ns |
| Transition falling falling | — | ['—', '31.3'] | ns |
| SPDIF_OUT output (Load = 30 pf) | — | ['—', '1.5'] | ns |
| Skew (Load = 30 pf) | — | ['—', '13.6'] | ns |
| Transition rising rising | — | ['—', '18.0'] | ns |
| Transition falling falling | — | ['—', '18.0'] | ns |
| Modulating Rx clock (SPDIF_SR_CLK) period | srck | 40.0 | — | ns |
| SPDIF_SR_CLK high period | srckph | 16.0 | — | ns |

### Electrical Specifications

| Parameter | Symbol | Min | Typ | Max | Unit | Conditions |
| --- | --- | --- | --- | --- | --- | --- |
| SPDIF_IN Skew | — | — |  | 0.7 | ns | Asynchronous inputs, no specs apply |
| SPDIF_OUT output (Load = 50 pf) | — | — |  | 1.5 | ns | Load = 50 pf |
| Skew (Load = 50 pf) | — | — |  | 24.2 | ns | Load = 50 pf |
| Transition rising rising | — | — |  | 31.3 | ns | Load = 50 pf |
| Transition falling falling | — | — |  | 31.3 | ns | Load = 50 pf |
| SPDIF_OUT output (Load = 30 pf) | — | — |  | 1.5 | ns | Load = 30 pf |
| Skew (Load = 30 pf) | — | — |  | 13.6 | ns | Load = 30 pf |
| Transition rising rising | — | — |  | 18.0 | ns | Load = 30 pf |
| Transition falling falling | — | — |  | 18.0 | ns | Load = 30 pf |
| Modulating Rx clock (SPDIF_SR_CLK) period | srck | 40.0 |  | — | ns | — |
| SPDIF_SR_CLK high period | srckph | 16.0 |  | — | ns | — |

#### Additional Information

This page discusses SPDIF timing parameters for the Sony/Philips Digital Interconnect Format (SPDIF). SPDIF data is sent using bi-phase marking code. The SPDIF data signal is modulated by a clock that is twice the bit rate of the data signal. Table 47 and Figures 17 and 18 show SPDIF timing parameters for SPDIF in Rx mode and Tx mode. The DSE[5:0] = 001111 and FSEL1[1:0] = 11 are required drive settings to meet the timing.

#### Notes

- The table continues on the next page.
- The SPDIF_IN Skew parameter applies to asynchronous inputs with no specifications provided.
- The SPDIF_OUT output parameters are specified for loads of 50 pf and 30 pf.
- The modulating Rx clock (SPDIF_SR_CLK) period and high period are specified.


## 4.10.3 PDM Microphone interface interface timing parameters

*(Page 43)*

### Table 47. SPDIF timing parameters ...continued

| Parameter | Symbol | Timing Parameter Range | Unit |
| --- | --- | --- | --- |
| SPDIF_SR_CLK low period | srckpl | ['Min', 'Max'] | ns |
| Modulating Tx clock (SPDIF_ST_CLK) period | stclkp | ['40.0', '—'] | ns |
| SPDIF_ST_CLK high period | stclkph | ['16.0', '—'] | ns |
| SPDIF_ST_CLK low period | stckpl | ['16.0', '—'] | ns |

### Table 48. PDM timing parameters parameters

| Parameter | Value |
| --- | --- |
| trs, tfs | ≤ floor (kxCLKDIV) - 1 |
| trh, tfh | ≥ 0 |

### Electrical Specifications

| Parameter | Symbol | Min | Typ | Max | Unit | Conditions |
| --- | --- | --- | --- | --- | --- | --- |
| SPDIF_SR_CLK low period | srckpl | 16.0 |  | — | ns |  |
| Modulating Tx clock (SPDIF_ST_CLK) period | stclkp | 40.0 |  | — | ns |  |
| SPDIF_ST_CLK high period | stclkph | 16.0 |  | — | ns |  |
| SPDIF_ST_CLK low period | stckpl | 16.0 |  | — | ns |  |

#### Additional Information

The PDM microphones must meet the setup and hold timing requirements shown in Table 48. The 'k' factor value in Table 48 depends on the selected quality mode as shown in Table 49. These timing requirements apply only if the clock divider is enabled (PDM_CTRL2[CLKDIV] = 0); otherwise, there are no special timing requirements. Depending on the K value, the user must ensure floor(K x CLKDIV) > 1 to avoid timing problems.

#### Notes

- NOTE: These timing requirements apply only if the clock divider is enabled (PDM_CTRL2[CLKDIV] = 0); otherwise, there are no special timing requirements.


## 4.10.4 Medium Quality Sound (MQS) electrical specifications

*(Page 44)*

### K factor value

| Quality factor | K factor |
| --- | --- |
| High Quality | 1/2 |
| Medium Quality, Very Low Quality 0 | 1 |
| Low Quality, Very Low Quality 1 | 2 |
| Very Low Quality 2 | 4 |

### MQS specifications

| Symbol | Description | Min | Typ | Max | Unit |
| --- | --- | --- | --- | --- | --- |
| f_mclk | Bit clock is used to generate the mclk. | -- | 24.576 | 66.5 | MHz |

### Electrical Specifications

| Parameter | Symbol | Min | Typ | Max | Unit | Conditions |
| --- | --- | --- | --- | --- | --- | --- |
| Frequency of mclk | f_mclk | -- | 24.576 | 66.5 | MHz | Depends on software settings |

#### Additional Information

Figure 19 illustrates the timing requirements for the PDM. PDM input/output timing requirements are shown in Figure 19. Medium quality sound (MQS) is used to generate medium quality audio via a standard GPIO in the pinmux, allowing the user to connect stereo speakers or headphones to a power amplifier without an additional DAC chip. Two outputs are asynchronous PWM pulses and their maximum frequency is 1/32 x mclk_frequency. The frequency of mclk depends on software settings.

#### Notes

- 1. Frequency of mclk depends on software settings.
- 2. f_mclk is the bit clock used to generate the mclk.


## 4.11 Analog

*(Page 45)*

### Table 51. ADC electrical electrical specifications

| Symbol | Description | Min | Typ | Max | Unit | Notes |
| --- | --- | --- | --- | --- | --- | --- |
| V_ADIN | Input voltage | V_GND | - | V_DDA | V | 1 |
| f_AD_CK | ADC clock frequency | 20 | - | 80 | MHz | - |
| C_sample | Sample cycles | 5.5 | - | - | Cycle | - |
| C_compare | Fixed compare cycles | - | 58 | - | Cycle | - |
| C_conversion | Conversion cycles | C_conversion = C_sample + C_compare | - | - | Cycle | - |
| C_AD | ADC input cycles | C_conversion = C_sample + C_compare | - | 7 | pF | 2 |
| C_AD_INPUT | ADC input capacitance | - | - | 7 | pF | 2 |
| R_AD_INPUT | ADC input series resistance | - | - | 1.25 | KΩ | - |
| DNL | ADC differential nonlinearity | - | ±2 | - | LSB | 3 |
| INL | ADC integral nonlinearity | - | ±6 | - | LSB | 3 |
| R_AS | Analog source source resistance | - | - | 5 | KΩ | - |
| Bandgap | Output voltage ready time | - | 1 | - | μs | 4 |
| Bandgap | for bandgap | - | 1 | - | μs | 5,6,7,8 |
| ENOB | Effective number of bits: Single-ended mode (11 x 11 mm package, PWM) | - | 9.8 | - | bit | 5,6,7,8 |
| ENOB | Effective number of bits: Single-ended mode (11 x 11 mm package, PWM) | - | 9.4 | - | bit | - |
| ENOB | Effective number of bits: Single-ended mode (11 x 11 mm package, PFM) | - | 9.4 | - | bit | - |
| ENOB | Effective number of bits: Single-ended mode (9 x 9 mm package, PWM) | - | 9.2 | - | bit | - |
| ENOB | Effective number of bits: Single-ended mode (9 x 9 mm package, PFM) | - | 8.5 | - | bit | - |

### Electrical Specifications

| Parameter | Symbol | Min | Typ | Max | Unit | Conditions |
| --- | --- | --- | --- | --- | --- | --- |
| Input voltage | V_ADIN | V_GND | - | V_DDA | V | NVCC_xxxx operating at 3.3 V |
| ADC clock frequency | f_AD_CK | 20 | - | 80 | MHz | NVCC_xxxx operating at 3.3 V |
| Sample cycles | C_sample | 5.5 | - | - | Cycle | - |
| Fixed compare cycles | C_compare | - | 58 | - | Cycle | - |
| Conversion cycles | C_conversion | C_conversion = C_sample + C_compare | - | - | Cycle | - |
| ADC input cycles | C_AD | C_conversion = C_sample + C_compare | - | 7 | pF | - |
| ADC input capacitance | C_AD_INPUT | - | - | 7 | pF | - |
| ADC input series resistance | R_AD_INPUT | - | - | 1.25 | KΩ | - |
| ADC differential nonlinearity | DNL | - | ±2 | - | LSB | - |
| ADC integral nonlinearity | INL | - | ±6 | - | LSB | - |
| Analog source source resistance | R_AS | - | - | 5 | KΩ | - |
| Output voltage ready time | Bandgap | - | 1 | - | μs | - |
| Bandgap | Bandgap | - | 1 | - | μs | - |
| Effective number of bits | ENOB | - | 9.8 | - | bit | Single-ended mode (11 x 11 mm package, PWM) |
| Effective number of bits | ENOB | - | 9.4 | - | bit | Single-ended mode (11 x 11 mm package, PWM) |
| Effective number of bits | ENOB | - | 9.4 | - | bit | Single-ended mode (11 x 11 mm package, PFM) |
| Effective number of bits | ENOB | - | 9.2 | - | bit | Single-ended mode (9 x 9 mm package, PWM) |
| Effective number of bits | ENOB | - | 8.5 | - | bit | Single-ended mode (9 x 9 mm package, PFM) |

#### Additional Information

The following sections introduce the timing and electrical parameters about analog interfaces of i.MX 93 processors. All ADC channels meet the 12-bit single-ended accuracy specifications.

#### Notes

- {'note_number': '1', 'description': 'Input voltage range is from V_GND to V_DDA.'}
- {'note_number': '2', 'description': 'Capacitance and resistance values are typical for the specified conditions.'}
- {'note_number': '3', 'description': 'DNL and INL specifications are for the specified conditions.'}
- {'note_number': '4', 'description': 'Bandgap output voltage ready time is specified under typical conditions.'}
- {'note_number': '5', 'description': 'ENOB specifications are for the specified conditions.'}
- {'note_number': '6', 'description': 'ENOB specifications are for the specified conditions.'}
- {'note_number': '7', 'description': 'ENOB specifications are for the specified conditions.'}
- {'note_number': '8', 'description': 'ENOB specifications are for the specified conditions.'}


## 4.11.2 12-bit ADC input impedance equivalent circuit diagram

*(Page 46)*

### Table 51. ADC electrical specifications

| Symbol | Description | Min | Typ | Max | Unit | Notes |
| --- | --- | --- | --- | --- | --- | --- |
|  | Effective number of bits: Single-ended mode (14 x 14 mm package, PWM) | -- | 10.5 | -- |  |  |
|  | Effective number of bits: Single-ended mode (14 x 14 mm package, PFM) | -- | 10.1 | -- |  |  |

### Electrical Specifications

| Parameter | Symbol | Min | Typ | Max | Unit | Conditions |
| --- | --- | --- | --- | --- | --- | --- |
| Effective number of bits (Single-ended mode, PWM) |  | -- | 10.5 | -- |  | 14 x 14 mm package |
| Effective number of bits (Single-ended mode, PFM) |  | -- | 10.1 | -- |  | 14 x 14 mm package |

#### Additional Information

The page discusses the ADC (Analog-to-Digital Converter) specifications and input impedance for the i.MX 93 Applications Processor. It includes details on effective number of bits (ENOB) for different modes (PWM and PFM), ADC input impedance, and a formula for calculating sample request time. The text also mentions the impact of noise on ADC performance and the effect of capacitive coupling between ADC channels. The ADC specifications are provided for single-ended mode in a 14 x 14 mm package, with typical values of 10.5 ENOB for PWM mode and 10.1 ENOB for PFM mode. The input impedance is described, including an additional R_IOMUX resistance of 350 Ω (from 295 Ω to 405 Ω) if an input goes through the MUX inside the IO and C_P of 2.5 pF. A formula is provided for calculating the sample request time: R_ADCtotal = R_ADIN + R_IOMUX, where R_IOMUX = 350 Ω, C_P = 2.5 pF, and B = 11 for 1/4 LSB settling. The text also mentions that ENOB can be lower than shown if an ADC channel corrupts other ADC channels through capacitive coupling, which may be dominated by board analog parasitics. The page includes a graph (Figure 20) showing the relationship between sample time and R_AS (in ohms).

#### Notes

- 1. On or off channels
- 2. ADC off component channels plus pad capacitance (~ 2 pF)
- 3. After calibration
- 4. Based on simulation
- 5. Noise on the ADC reference reference voltage (VDD_ANA1P8) will result in performance loss of the ADC proportional to the noise present.
- 6. Input data used for test is 1 kHz sine wave.
- 7. Measured at VREFH = 1.8 V and pwrssel = 2.
- 8. ENOB can be lower than shown, if an ADC channel corrupts other ADC channels through capacitive coupling. This coupling may be dominated by board analog parasitics. Care must be taken not to corrupt the desired channel being measured. This coupling becomes worse at higher analog frequencies and with switching waveforms due to the harmonic content.


## 4.12 External peripheral interface interface parameters

*(Page 47)*

### Electrical Specifications

| Parameter | Symbol | Min | Typ | Max | Unit | Conditions |
| --- | --- | --- | --- | --- | --- | --- |
| ADC Input Impedance |  |  |  |  |  | Equation for sampling time calculation |

#### Additional Information

{'block_diagram_description': 'Figure 21 shows the ADC input impedance equivalent circuit diagram, which includes components like R_{AS}, C_{AS}, C_{P}, C_{ADIN}, R_{ADC(total)}, R_{MUX}, Z_{AS}, Z_{ADIN}, R_{DIN}, and the ADC SAR engine. The diagram illustrates the simplified input pin equivalent circuit and the channel select circuit.', 'section_4_12': 'This section provides information on external peripheral interfaces.', 'subsection_4_12_1': 'Describes the Ultra-high-speed SD/SDIO/MMC host interface (uSDHC) AC timing, including SD/eMMC5.1 (single data rate) timing, eMMC5.1/SD3.0 (dual data rate) timing, and SDR50/SDR104 AC timing.', 'subsection_4_12_1_1': 'Focuses on SD3.0/eMMC5.1 (single data rate) AC timing, with specific drive settings (DSE[5:0] = 001111 and FSEL1[1:0] = 11) required to meet timing requirements.', 'figure_22_reference': 'Figure 22 depicts the timing of SD3.0/eMMC5.1.', 'table_52_reference': 'Table 52 lists the SD3.0/eMMC5.1 timing characteristics.'}

#### Notes

- The document is subject to legal disclaimers.
- The information is for the i.MX 93 Applications Processor datasheet, revision 6.1, dated 7 July 2025.
- The content is focused on automotive products (IMX93AEC).


## SD3.0/eMMC5.1 (SDR) timing

*(Page 48)*

### SD3.0/eMMC5.1 (SDR) interface timing timing specification

| ID | Parameter | Symbols | Min | Max | Unit |
| --- | --- | --- | --- | --- | --- |
| SD1 | Clock Frequency (Low Speed) | f_L^3 | 0 | 400 | kHz |
| SD1 | Clock Frequency (SD/SDIO Full Speed/High Speed) | f_PP^4 | 0 | 250/500 | kHz |
| SD1 | Clock Frequency (SDIO Full Speed/High Speed) | f_PP^4 | 0 | 25/50 | MHz |
| SD1 | Clock Frequency (MMC Full Speed/High Speed) | f_PP^5 | 0 | 20/52 | MHz |
| SD1 | Clock Frequency (Identification Mode) | f_OD | 100 | 400 | kHz |
| SD2 | Clock Low Time | t_WL | 7 | - | ns |
| SD3 | Clock High Time | t_WH | 7 | - | ns |
| SD4 | Clock Rise Time | t_RH | - | 3 | ns |
| SD4 | Clock Rise Time | t_TLH | - | 3 | ns |
| SD5 | Clock Fall Time | t_THL | - | 3 | ns |
| SD6 | uSDHC Output Delay | t_CD | -6.6 | 3.6 | ns |
| SD7 | uSDHC Input Setup Time | t_SU | 2.5 | - | ns |
| SD8 | uSDHC Input Hold Time | t_SU | 2.5 | - | ns |
| SD8 | uSDHC Input Hold Time | t_H | 1.5 | - | ns |

### Electrical Specifications

| Parameter | Symbol | Min | Typ | Max | Unit | Conditions |
| --- | --- | --- | --- | --- | --- | --- |
| Clock Frequency (Low Speed) | f_L^3 | 0 |  | 400 | kHz | Low-Speed mode |
| Clock Frequency (SD/SDIO Full Speed/High Speed) | f_PP^4 | 0 |  | 250/500 | kHz | SD/SDIO Full Speed/High Speed mode |
| Clock Frequency (SDIO Full Speed/High Speed) | f_PP^4 | 0 |  | 25/50 | MHz | SDIO Full Speed/High Speed mode |
| Clock Frequency (MMC Full Speed/High Speed) | f_PP^5 | 0 |  | 20/52 | MHz | MMC Full Speed/High Speed mode |
| Clock Frequency (Identification Mode) | f_OD | 100 |  | 400 | kHz | Identification mode |
| Clock Low Time | t_WL | 7 |  | - | ns | SD/SDIO Full Speed/High Speed mode |
| Clock High Time | t_WH | 7 |  | - | ns | SD/SDIO Full Speed/High Speed mode |
| Clock Rise Time | t_RH | - |  | 3 | ns | SD/SDIO Full Speed/High Speed mode |
| Clock Rise Time | t_TLH | - |  | 3 | ns | SD/SDIO Full Speed/High Speed mode |
| Clock Fall Time | t_THL | - |  | 3 | ns | SD/SDIO Full Speed/High Speed mode |
| uSDHC Output Delay | t_CD | -6.6 |  | 3.6 | ns | uSDHC Output |
| uSDHC Input Setup Time | t_SU | 2.5 |  | - | ns | uSDHC Input |
| uSDHC Input Hold Time | t_SU | 2.5 |  | - | ns | uSDHC Input |
| uSDHC Input Hold Time | t_H | 1.5 |  | - | ns | uSDHC Input |

#### Additional Information

This page discusses the SD3.0/eMMC5.1 (SDR) timing specifications for the i.MX 93 Applications Processor. It includes timing parameters for various modes such as Low Speed, Full Speed, High Speed, and Identification Mode. The timing parameters cover clock frequency, clock rise/fall times, setup/hold times, and output delay. The document also includes notes on signal integrity, external load conditions, and voltage ranges for different modes.

#### Notes

- 1. Input timing assumes an input signal slew rate of 3 ns (20%/80%).
- 2. Output timing valid for maximum external CL = 25 pF, assumed to be a 10 pF load at the end of a 50 ohm, unterminated, 5-inch microstrip trace on standard FR4 (3.3 pF/inch), 25 pF total with margin. For best signal integrity, series resistance in transmission line should equal selected RDSON of I/O pad output driver.
- 3. In Low-Speed (Full) mode, card clock must be lower than 400 kHz, voltage ranges from 2.7 to 3.6 V.
- 4. In Normal (Full)-Speed mode for SD/SDIO card, clock frequency can be any value between 0 – 25 MHz. In High-speed mode, frequency can be any value between 0 – 50 MHz.
- 5. In Normal (Full)-Speed mode for MMC card, clock frequency can be any value between 0 – 20 MHz. In High-speed mode, frequency can be any value between 0 – 52 MHz.


## 4.12.1.2 SD3.0/eMMC5.1 (dual data rate) AC timing

*(Page 49)*

### SD3.0/eMMC5.1 (DDR) interface timing timing specification

| ID | Parameter | Symbols | Min | Max | Unit |
| --- | --- | --- | --- | --- | --- |
| SD1 | Clock Frequency (eMMC5.1 DDR) | fPP | 0 | 52 | MHz |
| SD1 | Clock Frequency (SD5.1 DDR) | fPP | 0 | 52 | MHz |
| SD1 | Clock Frequency (SD3.0 DDR) | fPP | 0 | 50 | MHz |
| SD2 | uSD Output Output Delay | tOD | 2.8 | 6.8 | ns |
| SD3 | uSDHC Input Setup Time / Outputs SD_CMD, SDx_DATAX (Reference to CLK) | tISU | 2.4 | - | ns |
| SD4 | uSDHC Input Hold Time / Outputs SD_CMD, SDx_DATAX (Reference to CLK) | tIH | 1.5 | - | ns |

### Electrical Specifications

| Parameter | Symbol | Min | Typ | Max | Unit | Conditions |
| --- | --- | --- | --- | --- | --- | --- |
| Clock Frequency (eMMC5.1 DDR) | fPP | 0 |  | 52 | MHz | N/A |
| Clock Frequency (SD5.1 DDR) | fPP | 0 |  | 52 | MHz | N/A |
| Clock Frequency (SD3.0 DDR) | fPP | 0 |  | 50 | MHz | N/A |
| uSD Output Output Delay | tOD | 2.8 |  | 6.8 | ns | N/A |
| uSDHC Input Setup Time | tISU | 2.4 |  | - | ns | N/A |
| uSDHC Input Hold Time | tIH | 1.5 |  | - | ns | N/A |

#### Additional Information

To satisfy hold timing, the delay difference between clock input and cmd/data input must not exceed 2 ns. Figure 23 depicts the timing of SD3.0/eMMC5.1 (DDR). Table 53 lists the SD3.0/eMMC5.1 (DDR) timing characteristics. Only DATA is sampled on both edges of the clock (not applicable to CMD). Input timing assumes an input signal slew rate of 3 ns (20%/80%). Output timing is valid for a maximum external load CL = 25 pF, which is assumed to be a 10 pF load at the end of a 50 ohm, unterminated, 5-inch microstrip trace on standard FR4 (3.3 pF/inch, 25 pF total with margin). For best signal integrity, the series resistance in the transmission line should be equal to the selected RDSON of the I/O pad output driver.

#### Notes

- 1. Input timing assumes an input signal slew rate of 3 ns (20%/80%).
- 2. Output timing is valid for a maximum external load CL = 25 pF, which is assumed to be a 10 pF load at the end of a 50 ohm, unterminated, 5-inch microstrip trace on standard FR4 (3.3 pF/inch, 25 pF total with margin). For best signal integrity, the series resistance in the transmission line should be equal to the selected RDSON of the I/O pad output driver.


## HS400 Timing

*(Page 50)*

### HS400 interface interface timing specification specification (Nominal and Overdrive mode)

| ID | Parameter | Symbols | Min | Max | Unit |
| --- | --- | --- | --- | --- | --- |
| SD1 | Clock frequency | f_PP | 0 | 200 | MHz |
| SD2 | Clock low time | t_CL | 2.2 | - | ns |
| SD3 | Clock high time | t_CH | 2.2 | - | ns |
| SD4 | Output skew from Data of edge of SCK | t_QSkew1 | 0.45 | - | ns |
| SD5 | Output skew from SCK to Data of edge | t_QSkew2 | 0.45 | - | ns |
| SD6 | uSDHC input skew | t_RQ | - | 0.45 | ns |
| SD7 | uSDHC hold skew | t_RQH | - | 0.45 | ns |

### HS400 interface interface timing specification specification (Low drive mode)

| ID | Parameter | Symbols | Min | Max | Unit |
| --- | --- | --- | --- | --- | --- |
| SD1 | Clock frequency | f_PP | 0 | 133 | MHz |

### Electrical Specifications

| Parameter | Symbol | Min | Typ | Max | Unit | Conditions |
| --- | --- | --- | --- | --- | --- | --- |
| Clock frequency | f_PP | 0 |  | 200 | MHz | Nominal and Overdrive mode |
| Clock low time | t_CL | 2.2 |  |  | ns | Nominal and Overdrive mode |
| Clock high time | t_CH | 2.2 |  |  | ns | Nominal and Overdrive mode |
| Output skew from Data of edge of SCK | t_QSkew1 | 0.45 |  |  | ns | Nominal and Overdrive mode |
| Output skew from SCK to Data of edge | t_QSkew2 | 0.45 |  |  | ns | Nominal and Overdrive mode |
| uSDHC input skew | t_RQ |  |  | 0.45 | ns | Nominal and Overdrive mode |
| uSDHC hold skew | t_RQH |  |  | 0.45 | ns | Nominal and Overdrive mode |
| Clock frequency | f_PP | 0 |  | 133 | MHz | Low drive mode |

#### Additional Information

This page contains timing specifications for the HS400 interface in both Nominal/Overdrive and Low drive modes. It includes details on clock frequency, clock low/high times, output/input skews, and hold skews. The timing diagrams in Figure 24 illustrate the HS400 timing for data transfer between uSDHC and eMMC.

#### Notes

- 1. Input timing assumes an input signal slew rate of 1 ns (20%/80%).
- 2. Output timing valid for maximum external load CL = 15 pF, which is assumed to be an 8 pF load at the end of a 50 ohm, unterminated, 2-inch microstrip trace on standard FR4 (3.3 pF/inch). For best signal integrity, the series resistance of the transmission line should be matched closely to the RDSON of the I/O pad output driver.


## 4.12.1.4 HS200 Mode AC timing

*(Page 51)*

### Table 55. HS400 interface timing specification (Low drive mode)

| ID | Parameter | Symbols | Min | Max | Unit |
| --- | --- | --- | --- | --- | --- |
| SD2 | Clock low time | tCL | 3.3 | — | ns |
| SD3 | Clock high time | tCH | 3.3 | — | ns |
| SD4 | Output skew from data of edge of SCK | tOSkew1 | 0.45 | — | ns |
| SD5 | Output skew from edge of SCK to data | tOSkew1 | 0.45 | — | ns |
| SD5 | Output skew from edge of SCK to data | tOSkew2 | 0.45 | — | ns |
| SD6 | uSDHC input skew | tRQ | — | 0.45 | ns |
| SD7 | uSDHC hold skew | tRQH | — | 0.45 | ns |

### Table 56. HS200 interface timing specification specification (Nominal and Overdrive mode)

| ID | Parameter | Symbols | Min | Max | Unit |
| --- | --- | --- | --- | --- | --- |
| SD1 | Clock Frequency Period | tCLK | 5.0 | — | ns |
| SD2 | Clock Low Time | tCL | 2.2 | — | ns |
| SD3 | Clock High Time | tCH | 2.2 | — | ns |

### Electrical Specifications

| Parameter | Symbol | Min | Typ | Max | Unit | Conditions |
| --- | --- | --- | --- | --- | --- | --- |
| Clock low time (SD2) | tCL | 3.3 |  | — | ns | Low drive mode |
| Clock high time (SD3) | tCH | 3.3 |  | — | ns | Low drive mode |
| Output skew from data of edge of SCK (SD4) | tOSkew1 | 0.45 |  | — | ns | Low drive mode |
| Output skew from edge of SCK to data (SD5) | tOSkew1 | 0.45 |  | — | ns | Low drive mode |
| Output skew from edge of SCK to data (SD5) | tOSkew2 | 0.45 |  | — | ns | Low drive mode |
| uSDHC input skew (SD6) | tRQ | — |  | 0.45 | ns | Low drive mode |
| uSDHC hold skew (SD7) | tRQH | — |  | 0.45 | ns | Low drive mode |
| Clock Frequency Period (SD1) | tCLK | 5.0 |  | — | ns | HS200 mode |
| Clock Low Time (SD2) | tCL | 2.2 |  | — | ns | HS200 mode |
| Clock High Time (SD3) | tCH | 2.2 |  | — | ns | HS200 mode |

#### Additional Information

Figure 25 depicts the timing of HS mode, Table 56 and Table 57 list the HS200 timing characteristics. Input timing assumes an input signal slew rate of 1 ns (20%/80%). Output timing valid for maximum external load CL = 15 pF, which is assumed to be a 2 pF load at the end of a 50 ohm, unterminated, 2-inch microstrip trace on standard FR4 (3.3 pF/inch). For best signal integrity, the series resistance of the transmission line should be matched closely to the RDSON of the I/O pad output driver.

#### Notes

- 1. Input timing assumes an input signal slew rate of 1 ns (20%/80%).
- 2. Output timing valid for maximum external load CL = 15 pF, which is assumed to be a 2 pF load at the end of a 50 ohm, unterminated, 2-inch microstrip trace on standard FR4 (3.3 pF/inch). For best signal integrity, the series resistance of the transmission line should be matched closely to the RDSON of the I/O pad output driver.


## 4.12.1.5 SDR50/SDR104 AC timing

*(Page 52)*

### HS200 interface timing specification (Nominal and Overdrive mode)

| ID | Parameter | Symbols | Min | Max | Unit |
| --- | --- | --- | --- | --- | --- |
| SD5 | uSDHC Output Delay | t_OD | -1.6 | 1 | ns |
| SD8 | uSDHC Input Data Data/Window | t_ODW | 0.475 x t_CLK | — | ns |

### HS200 interface timing specification (Low mode)

| ID | Parameter | Symbols | Min | Max | Unit |
| --- | --- | --- | --- | --- | --- |
| SD1 | Clock Frequency Period | t_CLK | 7.5 | — | ns |
| SD2 | Clock Low Time | t_CL | 3.3 | — | ns |
| SD3 | Clock High Time | t_CH | 3.3 | — | ns |
| SD5 | uSDHC Output Delay | t_OD | -1.6 | 1 | ns |
| SD8 | uSDHC Input Data Data/Window | t_ODW | 0.475 x t_CLK | — | ns |

### Electrical Specifications

| Parameter | Symbol | Min | Typ | Max | Unit | Conditions |
| --- | --- | --- | --- | --- | --- | --- |
| Clock Frequency Period | t_CLK | 7.5 |  | — | ns | Low mode |
| Clock Low Time | t_CL | 3.3 |  | — | ns | Low mode |
| Clock High Time | t_CH | 3.3 |  | — | ns | Low mode |
| uSDHC Output Delay | t_OD | -1.6 |  | 1 | ns | Nominal and Overdrive mode |
| uSDHC Input Data Data/Window | t_ODW | 0.475 x t_CLK |  | — | ns | Nominal and Overdrive mode |

#### Additional Information

HS200 interface timing specification for Nominal and Overdrive mode. HS200 interface timing specification for Low mode. Input timing assumes an input signal slew rate of 1 ns (20%/80%). Output timing valid for maximum external load CL = 15 pF, assumed as 8 pF load at the end of a 50 ohm, unterminated, 2-inch microstrip trace on standard FR4 (3.3 pF/inch). HS200 is for 8 bits while SDR104 is for 4 bits. Figure 26 depicts the timing of SDR50/SDR104. Table 58 and Table 59 list the SDR50/SDR104 timing characteristics.

#### Notes

- 1. Input timing assumes an input signal slew rate of 1 ns (20%/80%).
- 2. Output timing valid for maximum external load CL = 15 pF, which is assumed to be a 8 pF load at the end of a 50 ohm, unterminated, 2-inch microstrip trace on standard FR4 (3.3 pF/inch). For best signal integrity, the series resistance of the transmission line should be matched closely to the RDSON of the I/O pad output driver.
- 3. HS200 is for 8 bits while SDR104 is for 4 bits.


## SDR50/SDR104 Timing

*(Page 53)*

### SDR50/SDR104 interface timing specification (Nominal and Overdrive mode)

| ID | Parameter | Symbols | Min | Max | Unit |
| --- | --- | --- | --- | --- | --- |
| SD1 | Clock Frequency Period | t_CLK | 5 | - | ns |
| SD2 | Clock Low Time | t_CL | 2.2 | - | ns |
| SD3 | Clock High Time | t_CH | 2.2 | - | ns |
| SD4 | uSDHC Output Delay | t_OD | -3 | 1 | ns |
| SD5 | uSDHC Output Delay | t_OD | -1.6 | 1 | ns |
| SD6 | uSDHC Input Setup Time | t_ISU | 2.4 | - | ns |
| SD7 | uSDHC Input Hold Time | t_IH | 1.5 | - | ns |
| SD8 | uSDHC Input Data Window | t_ODW | 0.5 x t_CLK | - | ns |

### Electrical Specifications

| Parameter | Symbol | Min | Typ | Max | Unit | Conditions |
| --- | --- | --- | --- | --- | --- | --- |
| Clock Frequency Period | t_CLK | 5 |  | - | ns | Nominal and Overdrive mode |
| Clock Low Time | t_CL | 2.2 |  | - | ns | Nominal and Overdrive mode |
| Clock High Time | t_CH | 2.2 |  | - | ns | Nominal and Overdrive mode |
| uSDHC Output Delay (SDR50) | t_OD | -3 |  | 1 | ns | Nominal mode |
| uSDHC Output Delay (SDR104) | t_OD | -1.6 |  | 1 | ns | Overdrive mode |
| uSDHC Input Setup Time | t_ISU | 2.4 |  | - | ns | Nominal and Overdrive mode |
| uSDHC Input Hold Time | t_IH | 1.5 |  | - | ns | Nominal and Overdrive mode |
| uSDHC Input Data Window | t_ODW | 0.5 x t_CLK |  | - | ns | SDR104 mode |

#### Additional Information

This page contains timing specifications for the SDR50/SDR104 interface of the i.MX 93 Applications Processor. It includes details on clock frequency, setup and hold times, and output delays for both nominal and overdrive modes. The timing parameters are critical for ensuring proper communication between the uSDHC (Universal Serial Bus Host Controller) and eMMC (Embedded MultiMediaCard) interfaces.

#### Notes

- 1. Input timing assumes an input signal slew rate of 1 ns (20%/80%).
- 2. Output timing valid for maximum external load CL = 15 pF, which is assumed to be an 8 pF load at the end of a 50 ohm, FR4 (3.3 pF/inch) transmission line. For best signal integrity, the series resistance of the transmission line should be matched closely to the RDS(ON) of the I/O pad output driver.
- 3. Data window in SDR100 mode is variable.


## 4.12.1.6 Bus operation operation condition for 3.3 V and 1.8 V signaling

*(Page 54)*

### SDR50/SDR104 interface timing specification (Low drive mode)

| ID | Parameter | Symbols | Min | Max | Unit |
| --- | --- | --- | --- | --- | --- |
| SD1 | Clock Frequency Period | t_CLK | 7.5 | - | ns |
| SD2 | Clock Low Time | t_CL | 3.3 | - | ns |
| SD3 | Clock High Time | t_CH | 3.3 | - | ns |
| SD4 | uSDHC Output Delay | t_OD | -3 | 1 | ns |
| SD5 | uSDHC Output Delay | t_OD | -1.6 | 1 | ns |
| SD6 | uSDHC Input Setup Time | t_ISU | 2.4 | - | ns |
| SD7 | uSDHC Input Hold Time | t_IH | 1.5 | - | ns |
| SD8 | uSDHC Input Data Data Window | t_ODW | 0.5 x t_CLK | - | ns |

### Electrical Specifications

| Parameter | Symbol | Min | Typ | Max | Unit | Conditions |
| --- | --- | --- | --- | --- | --- | --- |
| Clock Frequency Period | t_CLK | 7.5 |  | - | ns | Low drive mode |
| Clock Low Time | t_CL | 3.3 |  | - | ns | Low drive mode |
| Clock High Time | t_CH | 3.3 |  | - | ns | Low drive mode |
| uSDHC Output Delay | t_OD | -3 |  | 1 | ns | SDR50 mode |
| uSDHC Output Delay | t_OD | -1.6 |  | 1 | ns | SDR104 mode |
| uSDHC Input Setup Time | t_ISU | 2.4 |  | - | ns | SDR50 mode |
| uSDHC Input Hold Time | t_IH | 1.5 |  | - | ns | SDR50 mode |
| uSDHC Input Data Data Window | t_ODW | 0.5 x t_CLK |  | - | ns | SDR104 mode |

#### Additional Information

Input timing assumes an input signal slew rate of 1 ns (20%/80%). Output timing valid for maximum external load CL = 15 pF, which is assumed to be a 8 pF load at the end of a 50 ohm, unterminated, 2-inch microstrip trace on standard FR4 (3.3 pF/inch). For best signal integrity, the series resistance of the transmission line should be matched closely to the RDSON of the I/O pad output driver. Data transmission window in SDR100 mode is variable. Bus operation operation condition for 3.3 V and 1.8 V signaling Signaling level of SD/eMMC4.5/5.0/5.1 can be 1.8 V or 3.3 V depending on the working mode. The DC parameters for NVCC_SD2 supplies are identical to those shown in General purpose I/O (GPIO) DC parameters. uSDHC supported modes - All SD 3.0 protocols are supported at full speeds on all three SDHC interfaces. This includes DS, HS, SDR12, SDR25, SDR50, SDR104, and DDR50. - The maximum supported SDR frequency is 200 MHz which is covered in SDR104 mode, and maximum DDR frequency is 50 MHz as a part of DDR50 mode. - eMMC HS400 is only supported on SDHC1 as that is the only one with 8-bit interface. - eMMC HS200 is supported on all three SDHC interfaces because this protocol supports both 4-bit mode and 8-bit mode, which can work on SDHC2 and SDHC3. - eMMC High Speed DDR, High Speed SDR, and less than or equal to 26 MHz MMC legacy protocols are also supported on all three SDHC interfaces.

#### Notes

- 1. Input timing assumes an input signal slew rate of 1 ns (20%/80%).
- 2. Output timing valid for maximum external load CL = 15 pF, which is assumed to be a 8 pF load at the end of a 50 ohm, unterminated, 2-inch microstrip trace on standard FR4 (3.3 pF/inch). For best signal integrity, the series resistance of the transmission line should be matched closely to the RDSON of the I/O pad output driver.
- 3. Data transmission window in SDR100 mode is variable.


## 4.12.2 Ethernet controller (ENET) AC electrical specifications

*(Page 55)*

### ENET2 signal mapping

| Pad name | RGMII | Alt mode | RMII | Alt mode | Direction |
| --- | --- | --- | --- | --- | --- |
| ENET2_MDC | RGMII_MDC | Alt 0 | RMII_MDC | Alt 0 | O |
| ENET2_MDIO | RGMII_MDIO | Alt 0 | RMII_MDIO | Alt 0 | I/O |
| ENET2_TXC | RGMII_TXC | Alt 0 | RMII_TX_ER | Alt 1 | O |
| ENET2_TX_CTL | RGMII_TX_CTL | Alt 0 | RMII_TX_EN | Alt 0 | O |
| ENET2_TD0 | RGMII_TD0 | Alt 0 | RMII_TD0 | Alt 0 | O |
| ENET2_TD1 | RGMII_TD1 | Alt 0 | RMII_TD1 | Alt 0 | O |
| ENET2_TD2 | RGMII_TD2 | Alt 0 | RMII_REF_CLK | Alt 1 | I/O |
| ENET2_TD3 | RGMII_TD3 | Alt 0 | - | Alt 0 | O |
| ENET2_RXC | RGMII_RXC | Alt 0 | RMII_RX_ER | Alt 1 | I |
| ENET2_RX_CTL | RGMII_RX_CTL | Alt 0 | RMII_CRS_DV | Alt 0 | I |
| ENET2_RD0 | RGMII_RD0 | Alt 0 | RMII_RD0 | Alt 0 | I |

### Electrical Specifications

| Parameter | Symbol | Min | Typ | Max | Unit | Conditions |
| --- | --- | --- | --- | --- | --- | --- |
| Maximum supported SDR frequency |  |  |  | 200 MHz | MHz | HS200 mode |
| Maximum supported DDR frequency |  |  |  | 200 MHz | MHz | HS400 mode |
| Maximum supported SDR frequency (multiplexing on GPIO_IO[27:22]) |  |  |  | 50 MHz | MHz | eMMC HS200 mode |
| Maximum supported DDR frequency (multiplexing on GPIO_IO[27:22]) |  |  |  | 52 MHz | MHz | eMMC HS400 mode |
| Maximum supported SDR/DDR frequency (3.3 V IO supply) |  |  |  | 50/52 MHz | MHz | 3.3 V IO supply |

#### Additional Information

The maximum supported SDR frequency is 200 MHz in HS200 mode, and the maximum DDR frequency is 200 MHz in HS400 mode. uSDHC3 supports up to SDR104 (200 MHz) on primary SD3_* pins, but when multiplexing on GPIO_IO[27:22], the following modes are supported: - eMMC High Speed DDR, High Speed SDR, and ≤ 26 MHz MMC legacy protocols - SDR50 (100 MHz) and SDR104 (200 MHz) modes are NOT supported - eMMC HS400 and HS200 modes are NOT supported The maximum supported SDR and DDR frequency is 50 and 52 MHz when multiplexing on GPIO_IO[27:22]. If IO is supplied by 3.3 V, the maximum supported SDR/DDR frequency is 50/52 MHz. Ethernet controller (ENET) AC electrical specifications Ethernet supports the following key features: - ENET AVB - IEEE 1588 - Energy Efficient Ethernet (EEE) - 1.8/3.3 V RMI operation, 1.8 V RGMII operation

#### Notes

- 1. Table continues on the next page.
- 2. RMII_REF_CLK is shared with ENET2_TD2 in Alt 1 mode.


## 4.12.2.2 RMII mode timing

*(Page 56)*

### Table 60. ENET2 signal mapping

| Pad name | RGMII | Alt mode | RMII | Alt mode | Direction |
| --- | --- | --- | --- | --- | --- |
| ENET2_RD1 | RGMII_RD1 | Alt 0 | RMII_RD1 | Alt 0 | I |
| ENET2_RD2 | RGMII_RD2 | Alt 0 | — | Alt 0 | I |
| ENET2_RD3 | RGMII_RD3 | Alt 0 | — | Alt 0 | I |

### Table 61. RMII signal timing

| ID | Characteristic | Min. | Max. | Unit |
| --- | --- | --- | --- | --- |
| M16 | ENET_CLK pulse width high | 35% | 65% | RMII_REF_CLK period |
| M17 | RMII_REF_CLK pulse width low | 35% | 65% | RMII_REF_CLK period |
| M18 | RMII_REF_CLK to ENET0_TXD[1:0], ENET_TX_EN invalid | 2 | — | ns |
| M19 | RMII_REF_CLK to ENET0_TXD[1:0], ENET_TX_EN valid | 2 | 14 | ns |
| M20 | ENET_RX_CLK[ENET0_TXD[1:0], ENET_TX_EN] to RMII_REF_CLK setup | 4 | — | ns |
| M21 | RMII_REF_CLK to ENET_RX_DATA[1:0], ENET_CRS_DV, ENET_RX_ER hold | 2 | — | ns |

### Electrical Specifications

| Parameter | Symbol | Min | Typ | Max | Unit | Conditions |
| --- | --- | --- | --- | --- | --- | --- |
| RMII_REF_CLK frequency |  | 50 MHz - 50 ppm |  | 50 MHz + 50 ppm | Hz | Continuous reference clock |

#### Additional Information

ENET1 is Ethernet QoS with TSN, while ENET2 is Ethernet MAC. The signal can be either input or output. In RMII mode, enet1.RMII_CLK is used as the REF_CLK, which is a 50 MHz ± 50 ppm continuous reference clock. Figure 27 shows RMII mode timings. Table 61 describes the timing parameters (M16–M21) shown in the figure. The timings assume the following configuration: DSE[5:0] = 001111 and FSEL[1:0] = 11.

#### Notes

- 1. ENET1 is Ethernet QoS with TSN, while ENET2 is Ethernet MAC.
- 2. The signal can be either input or output.
- 1. The timings assume the following configuration: DSE[5:0] = 001111 and FSEL[1:0] = 11.


## 4.12.2.3 MII serial management channel timing (ENET_MDIO and ENET_MDC)

*(Page 57)*

### MII serial management channel timing diagram

| ID | Characteristic | Min. | Max. | Unit |
| --- | --- | --- | --- | --- |
| M10 | ENET_MDC falling edge to ENET_MDIO output invalid (min. propagation delay) | -1.5 | — | ns |
| M11 | ENET_MDC falling edge to ENET_MDIO output valid (max. propagation delay) | — | 13 | ns |
| M12 | ENET_MD (input) to ENET_MDC rising edge setup | 13 | — | ns |
| M13 | ENET_MD (input) to ENET_MDC rising edge hold | 0 | — | ns |
| M14 | ENET_MDC pulse width high | 40% | 60% | ENET_MDC period |
| M15 | ENET_MDC pulse width low | 40% | 60% | ENET_MDC period |

### MII serial management channel timing

| ID | Characteristic | Min. | Max. | Unit |
| --- | --- | --- | --- | --- |
| M10 | ENET_MDC falling edge to ENET_MDIO output invalid (min. propagation delay) | -1.5 | — | ns |
| M11 | ENET_MDC falling edge to ENET_MDIO output valid (max. propagation delay) | — | 13 | ns |
| M12 | ENET_MD (input) to ENET_MDC rising edge setup | 13 | — | ns |
| M13 | ENET_MD (input) to ENET_MDC rising edge hold | 0 | — | ns |
| M14 | ENET_MDC pulse width high | 40% | 60% | ENET_MDC period |
| M15 | ENET_MDC pulse width low | 40% | 60% | ENET_MDC period |

### Electrical Specifications

| Parameter | Symbol | Min | Typ | Max | Unit | Conditions |
| --- | --- | --- | --- | --- | --- | --- |
| Input signal slew rate |  | 3 ns |  | 3 ns | ns | 20%/80% transition points |
| Maximum external load capacitance (CL) |  | 25 pF |  | 25 pF | pF | Assumed to be 10 pF load at the end of a 50 ohm microstrip trace |
| MDC frequency |  |  |  | 2.5 MHz | MHz | To be compatible with IEEE 802.3 MII specification |

#### Additional Information

The MDC frequency is designed to be equal to or less than 2.5 MHz to be compatible with the IEEE 802.3 MII specification. Figure 28 shows MII asynchronous input timings. Table 62 describes the timing parameters (M10–M15) shown in the figure. The timings assume the following configuration: DSE[5:0] = 001111 and FSEL1[1:0] = 11. Input timing assumes an input signal slew rate of 3 ns (20%/80%). Output timing valid for maximum external load CL = 25 pF, which is assumed to be a 10 pF load at the end of a 50 ohm unterminated, 5-inch microstrip trace on standard FR4 (3.3 pF/inch, 25 pF total with margin). For best signal integrity, the series resistance in the transmission line should be equal to the selected RDSON of the I/O pad output driver.

#### Notes

- The timings assume the following configuration: DSE[5:0] = 001111 and FSEL1[1:0] = 11.
- Input timing assumes an input signal slew rate of 3 ns (20%/80%).
- Output timing valid for maximum external load CL = 25 pF, which is assumed to be a 10 pF load at the end of a 50 ohm unterminated, 5-inch microstrip trace on standard FR4 (3.3 pF/inch, 25 pF total with margin). For best signal integrity, the series resistance in the transmission line should be equal to the selected RDSON of the I/O pad output driver.


## 4.12.3 Ethernet Quality-of-Service (QoS) electrical specifications

*(Page 58)*

### Table 63. RGMII signal switching specifications

| Symbol | Description | Min. | Max. | Unit |
| --- | --- | --- | --- | --- |
| T_cyc | Clock cycle duration | 7.2 | 8.8 | ns |
| T_skewT | Data to clock output skew at transmitter | -500 | 500 | ps |
| T_skewR | Data to clock input skew at receiver | 1 | 2.6 | ns |
| Duty_G | Duty cycle for Gigabit | 45 | 55 | % |
| Duty_T | Duty cycle for 10/100T | 40 | 60 | % |

### Electrical Specifications

| Parameter | Symbol | Min | Typ | Max | Unit | Conditions |
| --- | --- | --- | --- | --- | --- | --- |
| Clock cycle duration | T_cyc | 7.2 |  | 8.8 | ns | Assumes DSE[5:0] = 001111 and FSEL[1:0] = 11 |
| Data to clock output skew at transmitter | T_skewT | -500 |  | 500 | ps | Assumes DSE[5:0] = 001111 and FSEL[1:0] = 11 |
| Data to clock input skew at receiver | T_skewR | 1 |  | 2.6 | ns | Assumes DSE[5:0] = 001111 and FSEL[1:0] = 11 |
| Duty cycle for Gigabit | Duty_G | 45 |  | 55 | % | Assumes DSE[5:0] = 001111 and FSEL[1:0] = 11 |
| Duty cycle for 10/100T | Duty_T | 40 |  | 60 | % | Assumes DSE[5:0] = 001111 and FSEL[1:0] = 11 |

#### Additional Information

The timings assume the following configuration: DSE[5:0] = 001111 and FSEL[1:0] = 11. Measured as defined in EIA/JESD 8-6 1995 with a timing threshold voltage of VDDQ/2. Output timing valid for maximum external load CL = 15 pF, which is assumed to be a 2-inch microstrip trace on standard FR4 (3.3 pF/inch). For best signal integrity, the series resistance in the transmission line should be matched closely to the selected RDSON of the I/O pad output driver. RGMII timing specifications are only valid for 1.8 V nominal I/O pad supply voltage. RGMII transmit signal timing diagram (Figure 29) RGMII receive signal timing diagram (Figure 30) Ethernet QoS supports the following Time Sensitive Networking (TSN) features:

#### Notes

- The timings assume the following configuration: DSE[5:0] = 001111 and FSEL[1:0] = 11.
- Measured as defined in EIA/JESD 8-6 1995 with a timing threshold voltage of VDDQ/2.
- Output timing valid for maximum external load CL = 15 pF, which is assumed to be a 2-inch microstrip trace on standard FR4 (3.3 pF/inch). For best signal integrity, the series resistance in the transmission line should be matched closely to the selected RDSON of the I/O pad output driver.
- RGMII timing specifications are only valid for 1.8 V nominal I/O pad supply voltage.


## 4.12.3.1 Ethernet QoS Signal Mapping

*(Page 59)*

### ENET QoS Signal Mapping

| Pad name | RGMII | Alt mode | RMII | Alt mode | Direction |
| --- | --- | --- | --- | --- | --- |
| ENET1_MDC | RGMII_MDC | Alt 0 | RMII_MDC | Alt 0 | O |
| ENET1_MDIO | RGMII_MDIO | Alt 0 | RMII_MDIO | Alt 0 | I/O |
| ENET1_TXC | RGMII_TXC | Alt 0 | RMII_TX_ER | Alt 1 | O |
| ENET1_TX_CTL | RGMII_TX_CTL | Alt 0 | RMII_TX_EN | Alt 0 | O |
| ENET1_TD0 | RGMII_TD0 | Alt 0 | RMII_TD0 | Alt 0 | O |
| ENET1_TD1 | RGMII_TD1 | Alt 0 | RMII_TD1 | Alt 0 | O |
| ENET1_TD2 | RGMII_TD2 | Alt 0 | RMII_REF_CLK | Alt 1 | I/O |
| ENET1_TD3 | RGMII_TD3 | Alt 0 | - | Alt 0 | O |
| ENET1_RXC | RGMII_RXC | Alt 0 | RMII_RX_ER | Alt 1 | I |
| ENET1_RX_CTL | RGMII_RX_CTL | Alt 0 | RMII_CRS_DV | Alt 0 | I |
| ENET_RX_CTL | RGMII_RX_CTL | Alt 0 | RMII_CRS_DV | Alt 0 | I |
| ENET1_RD0 | RGMII_RD0 | Alt 0 | RMII_RD0 | Alt 0 | I |
| ENET1_RD1 | RGMII_RD1 | Alt 0 | RMII_RD1 | Alt 0 | I |
| ENET1_RD2 | RGMII_RD2 | Alt 0 | - | Alt 0 | I |
| ENET1_RD3 | RGMII_RD3 | Alt 0 | - | Alt 0 | I |

### Electrical Specifications

| Parameter | Symbol | Min | Typ | Max | Unit | Conditions |
| --- | --- | --- | --- | --- | --- | --- |
| REF_CLK Frequency | f_REF_CLK | 49.5 MHz | 50 MHz | 50.5 MHz | MHz | In RMII mode, enet1.RMII_CLK is used as the REF_CLK |

#### Additional Information

802.1Qbv Enhancements to Scheduling Traffic 802.1bu Frame preemption Time-based Scheduling 1.8 V/3.3 V RMII operation 1.8 V RMII operation, 1.8 V RGMII operation The following timing specs are defined at the chip I/O pin and must be translated appropriately to arrive at timing specs/constraints for the physical interface. ENET1 is Ethernet QoS with TSN, while ENET2 is Ethernet MAC. The signal can be either input or output. In RMII mode, enet1.RMII_CLK is used as the REF_CLK, which is a 50 MHz ± 50 ppm continuous reference clock. Figure 31 shows RMII mode timings. Table 65 describes the timing parameters (M16–M21) shown in the figure.

#### Notes

- 1. ENET1 is Ethernet QoS with TSN, while ENET2 is Ethernet MAC.
- 2. The signal can be either input or output.


## 4.12.3.3 MII serial management management channel channel timing (ENET_MIDIO and ENET_MDC)

*(Page 60)*

### Table 65. RMII signal timing

| ID | signal | Characteristic | Min. | Max. | Unit |
| --- | --- | --- | --- | --- | --- |
| M16 | ENET_CLK pulse width high | ENET_CLK pulse width high | 35% | 65% | RMII_REF_CLK period |
| M17 | RMII_REF_CLK pulse width low | RMII_REF_CLK pulse width low | 35% | 65% | RMII_REF_CLK period |
| M18 | RMII_REF_CLK to ENET0_TXD[1:0], ENET_TX_EN invalid | RMII_REF_CLK to ENET0_TXD[1:0], ENET_TX_EN invalid | 2 | — | ns |
| M19 | RMII_REF_CLK to ENET0_TXD[1:0], ENET_TX_EN valid | RMII_REF_CLK to ENET0_TXD[1:0], ENET_TX_EN valid | — | 14 | ns |
| M20 | ENET_RX_DATA[1:0], ENET_CRS_DV, ENET_RX_ER to RMII_REF_CLK setup | ENET_RX_DATA[1:0], ENET_CRS_DV, ENET_RX_ER to RMII_REF_CLK setup | 4 | — | ns |
| M21 | RMII_REF_CLK to ENET_RX_DATA[1:0], ENET_CRS_DV, ENET_RX_ER hold | RMII_REF_CLK to ENET_RX_DATA[1:0], ENET_CRS_DV, ENET_RX_ER hold | 2 | — | ns |

### Electrical Specifications

| Parameter | Symbol | Min | Typ | Max | Unit | Conditions |
| --- | --- | --- | --- | --- | --- | --- |
| ENET_CLK pulse width high |  | 35% |  | 65% | RMII_REF_CLK period | Assumes DSE[5:0] = 001111 and FSEL[1:0] = 11 |
| RMII_REF_CLK pulse width low |  | 35% |  | 65% | RMII_REF_CLK period | Assumes DSE[5:0] = 001111 and FSEL[1:0] = 11 |
| RMII_REF_CLK to ENET0_TXD[1:0], ENET_TX_EN invalid |  | 2 ns |  | — | ns | Assumes input signal slew rate of 3 ns (20%/80%) |
| RMII_REF_CLK to ENET0_TXD[1:0], ENET_TX_EN valid |  | — |  | 14 ns | ns | Assumes input signal slew rate of 3 ns (20%/80%) |
| ENET_RX_DATA[1:0], ENET_CRS_DV, ENET_RX_ER to RMII_REF_CLK setup |  | 4 ns |  | — | ns | Assumes output load CL = 25 pF (25% total with margin) |
| RMII_REF_CLK to ENET_RX_DATA[1:0], ENET_CRS_DV, ENET_RX_ER hold |  | 2 ns |  | — | ns | Assumes output load CL = 25 pF (25% total with margin) |

#### Additional Information

Figure 31 shows RMII mode signal timing diagram. The MDC frequency is designed to be equal to or less than 2.5 MHz to be compatible with the IEEE 802.3 MII specification. Figure 32 shows MII asynchronous input timings. Table 66 describes the timing parameters (M10–M15) shown in the figure.

#### Notes

- The timings assume the following configuration: DSE[5:0] = 001111 and FSEL[1:0] = 11.
- Input timings assume an input signal slew rate of 3 ns (20%/80%).
- Output timing valid for maximum external load CL = 25 pF, which is assumed to be a 10 pF load at the end of a 50 ohm, unterminated, 5-inch microstrip trace on standard FR4 (3.3 pF/inch), (25 pF total with margin). For best signal integrity, the series resistance in the transmission line should be equal to the selected RDSON of the I/O pad output driver.


## 4.12.3.4 RGMII Signal Switching Specifications

*(Page 61)*

### Table 66. MII Serial Management Management Channel Channel Timing

| ID | Characteristic | Min. | Max. | Unit |
| --- | --- | --- | --- | --- |
| M10 | ENET_MDC falling edge to ENET_MDIO output invalid (min. propagation delay) | -1.5 | — | ns |
| M11 | ENET_MDC falling edge to ENET_MDIO output valid (max. propagation delay) | — | 13 | ns |
| M12 | ENET_MDIO (input) to ENET_MDC rising edge setup | 13 | — | ns |
| M13 | ENET_MDIO (input) to ENET_MDC rising edge hold | 0 | — | ns |
| M14 | ENET_MDC pulse pulse width high | 40% | 60% | ENET_MDC period |
| M15 | ENET_MDC pulse pulse width low | 40% | 60% | ENET_MDC period |

### Table 67. RGMII Signal Switching Specifications

| Symbol | Description | Min. | Max. | Unit |
| --- | --- | --- | --- | --- |
| T_cyc | Clock cycle duration | 7.2 | 8.8 | ns |
| T_skew | Data to clock output skew at transmitter | -500 | 500 | ps |
| T_skewT | Data to clock output skew at transmitter | -500 | 500 | ps |
| T_skewR | Data to clock input skew at receiver | 1 | 2.6 | ns |

### Electrical Specifications

| Parameter | Symbol | Min | Typ | Max | Unit | Conditions |
| --- | --- | --- | --- | --- | --- | --- |
| ENET_MDC falling edge to ENET_MDIO output invalid (min. propagation delay) | M10 | -1.5 |  | — | ns | DSE[5:0] = 001111 and FSEL1[1:0] = 11 |
| ENET_MDC falling edge to ENET_MDIO output valid (max. propagation delay) | M11 | — |  | 13 | ns | DSE[5:0] = 001111 and FSEL1[1:0] = 11 |
| ENET_MDIO (input) to ENET_MDC rising edge setup | M12 | 13 |  | — | ns | DSE[5:0] = 001111 and FSEL1[1:0] = 11 |
| ENET_MDIO (input) to ENET_MDC rising edge hold | M13 | 0 |  | — | ns | DSE[5:0] = 001111 and FSEL1[1:0] = 11 |
| ENET_MDC pulse pulse width high | M14 | 40% |  | 60% | ENET_MDC period | DSE[5:0] = 001111 and FSEL1[1:0] = 11 |
| ENET_MDC pulse pulse width low | M15 | 40% |  | 60% | ENET_MDC period | DSE[5:0] = 001111 and FSEL1[1:0] = 11 |
| Clock cycle duration | T_cyc | 7.2 |  | 8.8 | ns | RGMII interface |
| Data to clock output skew at transmitter | T_skew | -500 |  | 500 | ps | RGMII interface |
| Data to clock output skew at transmitter | T_skewT | -500 |  | 500 | ps | RGMII interface |
| Data to clock input skew at receiver | T_skewR | 1 |  | 2.6 | ns | RGMII interface |

#### Additional Information

This page contains timing diagrams and specifications for MII serial management and RGMII interfaces. It includes timing parameters for MII serial management channel channel timing and RGMII signal switching specifications. The timing diagrams and tables provide details on setup, hold times, pulse widths, and skew values for the interfaces. The notes section provides additional configuration details and assumptions for the timing specifications.

#### Notes

- The timings assume the following configuration: DSE[5:0] = 001111 and FSEL1[1:0] = 11.
- Input timing assumes an input signal slew rate of 3 ns (20%/80%).
- Output timing valid for maximum external load CL = 25 pF, which is assumed to be a 10 pF load at the end of a 50 ohm. Unterminated, 5-inch microstrip trace on standard FR4 (3.3 pF/inch), (25 pF total with margin). For best signal integrity, the series resistance in the transmission line should be equal to the selected RDSON of the I/O pad output driver.


## 4.12.4 LPSPI timing parameters

*(Page 62)*

### Table 67. RGMII signal switching specifications

| Symbol | Description | Min. | Max. | Unit |
| --- | --- | --- | --- | --- |
| Duty_G | Duty cycle for Gigabit | 45 | 55 | % |
| Duty_T | Duty cycle for 10/100T | 40 | 60 | % |

### Electrical Specifications

| Parameter | Symbol | Min | Typ | Max | Unit | Conditions |
| --- | --- | --- | --- | --- | --- | --- |
| Duty cycle for Gigabit (Duty_G) | Duty_G | 45 |  | 55 | % | Assumes DSE[5:0] = 001111 and FSEL[1:0] = 11 |
| Duty cycle for 10/100T (Duty_T) | Duty_T | 40 |  | 60 | % | Assumes DSE[5:0] = 001111 and FSEL[1:0] = 11 |

#### Additional Information

The timings assume the following configuration: DSE[5:0] = 001111 and FSEL[1:0] = 11. Measured as defined in EIA/JESD-8-6 1995 with a timing threshold voltage of VDDQ/2. Output timing valid for maximum external load CL = 15 pF, which is assumed to be an 8 pF load at the end of a 50 ohm, unterminated, 2-inch microstrip trace on standard FR4 (3.3 pF/inch). For best signal integrity, the series resistance in the transmission line should be matched closely to the selected RDSON of the I/O pad output driver. RGMII timing specifications are only valid for 1.8 V nominal I/O pad supply voltage voltage. The Low Power Serial Peripheral Interface (LPSPI) provides a synchronous serial bus with Controller and Peripheral operations. Many of the transfer attributes are programmable. All timing is shown with respect to 20% VDD and 80% VDD thresholds, unless noted, as well as input signal transitions of 3 ns and a 25 pF maximum load on all LPSPI pins. The DSE[5:0] = 001111 and FSEL[1:0] = 11 are required drive settings to meet the timing.

#### Notes

- The timings assume the following configuration: DSE[5:0] = 001111 and FSEL[1:0] = 11.
- Measured as defined in EIA/JESD-8-6 1995 with a timing threshold voltage of VDDQ/2.
- Output timing valid for maximum external load CL = 15 pF, which is assumed to be an 8 pF load at the end of a 50 ohm, unterminated, 2-inch microstrip trace on standard FR4 (3.3 pF/inch). For best signal integrity, the series resistance in the transmission line should be matched closely to the selected RDSON of the I/O pad output driver.
- RGMII timing specifications are only valid for 1.8 V nominal I/O pad supply voltage voltage.


## LPSPI Controller mode timing

*(Page 63)*

### LPSPI Controller mode timing

| Number | Symbol | Description | Min. | Max. | Units | Note |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | fSCK | Frequency of LPSPI clock³ | — | 30 | MHz | 4 |
|  |  |  | — | 60 | MHz | 5 |
| 2 | tSCK | SCK period period | 2 x tperiph | — | ns | 6 |
| 3 | tLead | Enable lead lead time | 1 | — | tperiph | — |
| 4 | tLag | Enable lag lag time | 1 | — | tperiph | — |
|  |  | Clock (SCK) high or low time | tSCK / 2 - 3 | tSCK / 2 + 3 | ns | — |
| 5 | tWSCK | Clock (SCK) high or low time | tSCK / 2 - 3 | tSCK / 2 + 3 | ns | — |
| 6 | tSU | Data setup time (inputs) | 8 | — | ns | 7,8 |
| 7 | tHI | Data hold time (inputs) | 0 | — | ns | 7 |
| 8 | tV | Data valid (after SCK edge) | — | 2.5 | ns | — |
| 9 | tHO | Data hold time (outputs) | -2.5 | — | ns | — |

### Electrical Specifications

| Parameter | Symbol | Min | Typ | Max | Unit | Conditions |
| --- | --- | --- | --- | --- | --- | --- |
| Input timing | — | — | — | — | — | Assumes an input signal slew rate of 3 ns (20%/80%). |
| Output timing | — | — | — | — | — | Valid for maximum external load CL = 25 pF, which is assumed to be a 10 pF load at the end of a 50 ohm. Unterminated, 5-inch microstrip trace on standard FR4 (3.3 pF/inch), (25 pF total with margin). For best signal integrity, the series resistance in the transmission line should equal the selected RDSON of the I/O pad output driver. |
| Maximum frequency | fSCK | — | — | 52 MHz | MHz | When NVCC_xxxx operating at 3.3 V. |
| Data setup time (tSU) | tSU | 8 ns | — | — | ns | For 3.3 V I/O supply, the parameter value is 9 ns in LPSPI Controller mode. |

### LPSPI_CFGR (`—`)

| Bits | Field | Access | Description |
| --- | --- | --- | --- |
| [SAMPLE] | SAMPLE | RW | When set to 1, the data setup time (inputs) / data hold time (inputs) specifications are the same as in Peripheral mode. |

#### Additional Information

This page provides detailed timing parameters for the LPSPI (Low Power SPI) Controller mode of the i.MX 93 Applications Processor. It includes specifications for clock frequency, SCK period, lead/lag times, data setup and hold times, and valid data timing. The document also includes notes on input and output timing assumptions, maximum frequency support, and specific conditions for different operating modes.

#### Notes

- {'note_number': '1', 'text': 'Input timing assumes an input signal slew rate of 3 ns (20%/80%).'}
- {'note_number': '2', 'text': 'Output timing valid for maximum external load CL = 25 pF, which is assumed to be a 10 pF load at the end of a 50 ohm. Unterminated, 5-inch microstrip trace on standard FR4 (3.3 pF/inch), (25 pF total with margin). For best signal integrity, the series resistance in the transmission line should equal the selected RDSON of the I/O pad output driver.'}
- {'note_number': '3', 'text': 'The maximum frequency supported is 52 MHz when NVCC_xxxx operating at 3.3 V.'}
- {'note_number': '4', 'text': 'The clock driver in the LPSPI module for fperiph must guarantee this limit is not exceeded.'}
- {'note_number': '5', 'text': 'In Controller loopback mode when LPSPI_CFGR[SAMPLE] bit is 1.'}
- {'note_number': '6', 'text': 'fperiph = Functional clock / (2 ^ PRESCALE) and tperiph = 1 / fperiph'}
- {'note_number': '7', 'text': 'If LPSPI_CFGR[SAMPLE] bit is 1, the data setup time (inputs) / data hold time (inputs) specifications are the same as in Peripheral mode.'}
- {'note_number': '8', 'text': 'For 3.3 V I/O supply, tSU (Data setup time) parameter value is 9 ns in LPSPI Controller mode.'}


## LPSPI Controller mode timing (CPHA = 0) and (CPHA = 1)

*(Page 64)*

### LPSPI Peripheral mode timing

| Number | Symbol | Description | Min. | Max. | Units | Note |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | fSCK | Frequency of LPSPI clock | 0 | 30 | MHz |  |
| 2 | tSCK | SCK period | 2 x tperiph | 30 | ns | 3 |
| 2 | tSCK | SCK period | 2 x tperiph | - | ns | 3 |
| 3 | tLead | Enable lead time | 1 | - | tperiph |  |

### Electrical Specifications

| Parameter | Symbol | Min | Typ | Max | Unit | Conditions |
| --- | --- | --- | --- | --- | --- | --- |
| Frequency of LPSPI clock | fSCK | 0 |  | 30 | MHz |  |
| SCK period | tSCK | 2 x tperiph |  | 30 | ns |  |
| Enable lead time | tLead | 1 |  | - | tperiph |  |

#### Additional Information

The page contains timing diagrams and specifications for the LPSPI (Low Power Serial Peripheral Interface) controller in the i.MX 93 Applications Processor. It includes two timing diagrams (Figures 35 and 36) for different CPHA (Clock Phase) configurations (CPHA = 0 and CPHA = 1). The diagrams illustrate the timing relationships between various signals such as PCS (Peripheral Chip Select), SCK (Serial Clock), SIN (Serial Input), and SOUT (Serial Output). The tables provide timing parameters for the LPSPI peripheral mode.

#### Notes

- 1. If configured as an output: For LSBF = 1, bit order is LSB, bit 1, ..., bit 6, MSB. For LSBF = 0, bit order is LSB, bit 1, ..., bit 6, MSB.
- 2. LSBF = 0. For LSBF = 1, bit order is LSB, bit 1, ..., bit 6, MSB.
- 3. tperiph is the peripheral clock period.


## LPSPI Peripheral mode timing

*(Page 65)*

### Table 69. LPSPI Peripheral mode timing

| Number | Symbol | Description | Min. | Max. | Units | Note |
| --- | --- | --- | --- | --- | --- | --- |
| 4 | t_Lag | Enable lag time | 1 | — | t_periph | — |
| 5 | t_WSCK | Clock (SCK) high or low time | tSCK / 2 - 5 | tSCK / 2 + 5 | ns | — |
| 6 | t_SU | Data setup time (inputs) | 3 | — | ns | — |
| 7 | t_HI | Data hold time (inputs) | 3 | — | ns | — |
| 8 | t_a | Peripheral access access time | — | 20 | ns | 4 |
| 8 | t_a | Peripheral access access time | — | 20 | ns | 5 |
| 9 | t_dis | Peripheral MISO disable time | — | 20 | ns | 5 |
| 10 | t_V | Data valid (after SCK edge) | — | 8 | ns | 6 |
| 11 | t_HO | Data hold time (outputs) | 0 | — | ns | — |

### Electrical Specifications

| Parameter | Symbol | Min | Typ | Max | Unit | Conditions |
| --- | --- | --- | --- | --- | --- | --- |
| Enable lag time | t_Lag | 1 |  | — | t_periph | — |
| Clock (SCK) high or low time | t_WSCK | tSCK / 2 - 5 |  | tSCK / 2 + 5 | ns | — |
| Data setup time (inputs) | t_SU | 3 |  | — | ns | — |
| Data hold time (inputs) | t_HI | 3 |  | — | ns | — |
| Peripheral access access time | t_a | — |  | 20 | ns | Note 4 |
| Peripheral MISO disable time | t_dis | — |  | 20 | ns | Note 5 |
| Data valid (after SCK edge) | t_V | — |  | 8 | ns | Note 6 |
| Data hold time (outputs) | t_HO | 0 |  | — | ns | — |

#### Additional Information

Input timing assumes an input signal slew rate of 3 ns (20%/80%). Output timing valid for maximum external load CL = 25 pF, which is assumed to be a 10 pF load at the end of a 50 ohm. Unterminated, 5-inch microstrip trace on standard FR4 (3.3 pF/inch), (25 pF total with margin). For best signal integrity, the series resistance in the transmission line should be equal to the selected RDSON of the I/O pad output driver. t_periph = Functional clock / (2 ^ PRESCALE) and t_periph = 1 / f_periph Time to data active from high-impedance state Hold time to data active from high-impedance state When operating at 3.3 V I/O supply, this parameter value is 9 ns.

#### Notes

- {'note_number': '1', 'text': 'Input timing assumes an input signal slew rate of 3 ns (20%/80%).'}
- {'note_number': '2', 'text': 'Output timing valid for maximum external load CL = 25 pF, which is assumed to be a 10 pF load at the end of a 50 ohm. Unterminated, 5-inch microstrip trace on standard FR4 (3.3 pF/inch), (25 pF total with margin). For best signal integrity, the series resistance in the transmission line should be equal to the selected RDSON of the I/O pad output driver.'}
- {'note_number': '3', 'text': 't_periph = Functional clock / (2 ^ PRESCALE) and t_periph = 1 / f_periph'}
- {'note_number': '4', 'text': 'Time to data active from high-impedance state'}
- {'note_number': '5', 'text': 'Hold time to data active from high-impedance state'}
- {'note_number': '6', 'text': 'When operating at 3.3 V I/O supply, this parameter value is 9 ns.'}


## 4.12.5 LPI2C timing parameters

*(Page 66)*

### LPI2C module timing parameters

| Symbol | Description | Min | Max | Unit | Notes |
| --- | --- | --- | --- | --- | --- |
| fSCL | SCL clock frequency | 0 | 100 | kHz | 2 |
|  | Standard mode (Sm) | 0 | 100 | kHz |  |
|  | Fast mode (Fm) | 0 | 400 | kHz |  |
|  | Fast mode Plus (Fm+) | 0 | 1000 | kHz |  |
|  | High mode Plus (Fm+) | 0 | 1000 | kHz |  |
|  | High speed mode (Hs-mode) | 0 | 3400 | kHz |  |
|  | Ultra Fast mode (UFm) | 0 | 5000 | kHz |  |

### Electrical Specifications

| Parameter | Symbol | Min | Typ | Max | Unit | Conditions |
| --- | --- | --- | --- | --- | --- | --- |
| SCL clock frequency | fSCL | 0 |  | 5000 | kHz | Depends on mode (Standard, Fast, Fast+, High+, High speed, Ultra Fast) |

#### Additional Information

LPI2C is a low-power timing Inter-Integrated Circuit (I2C) module that supports an efficient interface to an I2C bus as a controller and/or as a target. The table provides timing parameters for different modes of operation: Standard, Fast, Fast+, High+, High speed, and Ultra Fast. For more details, see UM10204 I2C-bus specification and user manual. Standard, Fast, Fast+, and Ultra Fast modes are supported; High speed mode (HS) in target mode. 4.12.6 Improved Inter-Integrated Circuit Interface (I3C) specifications I3C specifications are timed to/from the VIH and/or VIL signal points. The DSE[5:0] = 001111 and FSEL[1:0] = 11 are required drive settings to meet the timing. 4.12.6.1 I3C Push-Pull Timing Timing Parameters for SDR Mode I3C interface is not supported on GPIO-Standard-plus pad type for 5 V operation. Measurements are with maximum output load of 30 pF, input transition of 1 ns.

#### Notes

- 1. For more details, see UM10204 I2C-bus specification and user manual.
- 2. Standard, Fast, Fast+, and Ultra Fast modes are supported; High speed mode (HS) in target mode.


## I2C Push-Pull Timing Parameters for SDR Mode

*(Page 67)*

### I2C Push-Pull Timing Parameters for SDR Mode

| Symbol | Description | Min | Typ | Max | Unit | Condition |
| --- | --- | --- | --- | --- | --- | --- |
| fSCL | SCL Clock Frequency | 0.01 | 12.5 | 12.9 | MHz | FSCL = 1 / (tDIG_L + tDIG_H) |
| tDIG_L | SCL Clock Low Period 1,2 | 32 | — | — | ns | — |
| tDIG_H | SCL Clock High Period 1 | 32 | — | — | ns | — |
| tSCO | Clock in to Data Out for 3,4 | — | — | 12 | ns | — |
| tCR | SCL Clock Rise Time 5 | — | — | 150e06 * 1 / fSCL (capped at 60) | ns | — |
| tCF | SCL Clock Fall Time 5 | — | — | 150e06 * 1 / fSCL (capped at 60) | ns | — |
| thD_PP | SDA Signal Data Hold in Push-Pull Mode, Slave 6 | 1 | — | — | ns | Applicable for slave and master modes |
| tSU_PP | SDA Signal Signal Data Setup in Push-Pull Mode | 3 | — | N/A | ns | Applicable for slave and master loopback modes |

### Electrical Specifications

| Parameter | Symbol | Min | Typ | Max | Unit | Conditions |
| --- | --- | --- | --- | --- | --- | --- |
| SCL Clock Frequency | fSCL | 0.01 | 12.5 | 12.9 | MHz | FSCL = 1 / (tDIG_L + tDIG_H) |
| SCL Clock Low Period | tDIG_L | 32 | — | — | ns | — |
| SCL Clock High Period | tDIG_H | 32 | — | — | ns | — |
| Clock in to Data Out | tSCO | — | — | 12 | ns | — |
| SCL Clock Rise Time | tCR | — | — | 150e06 * 1 / fSCL (capped at 60) | ns | — |
| SCL Clock Fall Time | tCF | — | — | 150e06 * 1 / fSCL (capped at 60) | ns | — |
| SDA Signal Data Hold in Push-Pull Mode, Slave | thD_PP | 1 | — | — | ns | Applicable for slave and master modes |
| SDA Signal Signal Data Setup in Push-Pull Mode | tSU_PP | 3 | — | N/A | ns | Applicable for slave and master loopback modes |

#### Additional Information

This page contains timing parameters for the I2C Push-Pull mode in SDR (Standard Data Rate) mode. It includes details about SCL clock frequency, rise and fall times, data hold and setup times, and other related timing parameters. The conditions and notes provide additional context for interpreting these values.

#### Notes

- 1. tDIG_L and tDIG_H are the clock Low and High periods as seen at the receiver end of the I2C Bus using VIL and VIH (see Figure 30).
- 2. As both edges are used, the hold time needs to be satisfied for the respective edges; i.e., tCF + 3 for falling edge clocks, and tCR + 3 for rising edge clocks.
- 3. Devices with more than 12 ns of tSCO delay shall set the limitation bit in the BCR, and shall support the GETMXDS CCC to allow the Master to read this value and adjust computations accordingly. For purposes of system design and test conformance, this parameter should be considered together with pad delay, bus capacitance, propagation delay, and clock triggering points.
- 4. The clock maximum rise/fall time is capped at 60 ns, and is not dependent upon the clock frequency.
- 5. The clock maximum rise/fall time is capped at 60 ns. For lower frequency rise and fall, the maximum value is limited at 60 ns.
- 6. thD_PP is a Hold time parameter for Push-Pull Mode that has a different value for Master mode vs. Slave mode. In SDR Mode, the Hold time parameter is referred to as thD_SDR.


## 4.12.7 CAN network network AC electrical electrical specifications specifications

*(Page 68)*

### Electrical Specifications

| Parameter | Symbol | Min | Typ | Max | Unit | Conditions |
| --- | --- | --- | --- | --- | --- | --- |
| Controller out timing (SDA) | tHD_PP | 0.7 x VDD | 0.7 x VDD | 0.7 x VDD | V | Controller output high level |
| Controller out timing (SDA) | tSU_PP | 0.3 x VDD | 0.3 x VDD | 0.3 x VDD | V | Controller output low level |
| Controller out timing (SCL) | tHD_PP | 0.7 x VDD | 0.7 x VDD | 0.7 x VDD | V | Controller output high level |
| Controller out timing (SCL) | tSU_PP | 0.3 x VDD | 0.3 x VDD | 0.3 x VDD | V | Controller output low level |
| Target out timing (SDA) | tSCO | 0.7 x VDD | 0.7 x VDD | 0.7 x VDD | V | Target output high level |
| Target out timing (SDA) | tSU_PP | 0.3 x VDD | 0.3 x VDD | 0.3 x VDD | V | Target output low level |
| Target out timing (SCL) | tHD_PP | 0.7 x VDD | 0.7 x VDD | 0.7 x VDD | V | Target output high level |
| Target out timing (SCL) | tSU_PP | 0.3 x VDD | 0.3 x VDD | 0.3 x VDD | V | Target output low level |
| Controller SDR timing (SDA) | tSU_PP | 0.3 x VDD | 0.3 x VDD | 0.3 x VDD | V | Controller output low level |
| Controller SDR timing (SDA) | tHD_SDR | 0.7 x VDD | 0.7 x VDD | 0.7 x VDD | V | Controller output high level |
| Controller SDR timing (SCL) | tSU_PP | 0.3 x VDD | 0.3 x VDD | 0.3 x VDD | V | Controller output low level |
| Controller SDR timing (SCL) | tHD_SDR | 0.7 x VDD | 0.7 x VDD | 0.7 x VDD | V | Controller output high level |

#### Additional Information

The Controller Area Network (CAN) module is a communication controller implementing the CAN protocol according to the CAN with Flexible Data rate (CAN FD) protocol and the CAN 2.0B protocol specification. The processor has two CAN modules.


## 4.12.8 Timer/Pulse width modulator (TPM) timing parameters

*(Page 69)*

### Table 72. CAN-FD electrical specifications

| Parameters | FlexCAN (Classical and FD) | Unit |
| --- | --- | --- |
| Maximum Baud Rate | 8/8 | Mbps |
| TXD Rise time wcs | 4/4 | ns |
| TXD Fall time wcs | 4/4 | ns |
| TXD Rise time wcs | 4/4 | ns |
| RXD Rise time wcs | 4/4 | ns |
| RXD Fall time wcs | 4/4 | ns |
| TXD | 3.3/3.3 | V |
| RXD | 3.3/3.3 | V |
| Internal delay wcs | 100/50 | ns |
| TX PAD PAD delay wcs | 25/25 | ns |
| TX PAD delay wcs | 25/25 | ns |
| RX PAD delay wcs | 10/10 | ns |
| TX routing delay wcs | 5/5 | ns |
| RX routing delay wcs | 5/5 | ns |
| Transceiver loop loop delay wcs | 250/250 | ns |
| Total loop delay | 395/345 | ns |

### Electrical Specifications

| Parameter | Symbol | Min | Typ | Max | Unit | Conditions |
| --- | --- | --- | --- | --- | --- | --- |
| Maximum Baud Rate |  | 8 | 8 | 8 | Mbps |  |
| TXD Rise time |  | 4 | 4 | 4 | ns |  |
| TXD Fall time |  | 4 | 4 | 4 | ns |  |
| TXD Rise time |  | 4 | 4 | 4 | ns |  |
| RXD Rise time |  | 4 | 4 | 4 | ns |  |
| RXD Fall time |  | 4 | 4 | 4 | ns |  |
| TXD Voltage |  | 3.3 | 3.3 | 3.3 | V |  |
| RXD Voltage |  | 3.3 | 3.3 | 3.3 | V |  |
| Internal delay |  | 50 | 100 | 100 | ns |  |
| TX PAD delay |  | 25 | 25 | 25 | ns |  |
| TX PAD delay |  | 25 | 25 | 25 | ns |  |
| RX PAD delay |  | 10 | 10 | 10 | ns |  |
| TX routing delay |  | 5 | 5 | 5 | ns |  |
| RX routing delay |  | 5 | 5 | 5 | ns |  |
| Transceiver loop delay |  | 250 | 250 | 250 | ns |  |
| Total loop delay |  | 345 | 395 | 395 | ns |  |

#### Additional Information

Tx and Rx ports for both modules are multiplexed with other I/O pins. See the IOMUXC chapter of the device reference manual to see which pins expose Tx and Rx pins; these ports are named CAN_TX and CAN_RX, respectively. The DSE[5:0] = 001111 and FSEL[1:0] = 11 are required drive settings to meet the timing. Please see General purpose I/O (GPIO) AC parameters for timing parameters. This section describes the output timing timing parameters of the TPM. The DSE[5:0] = 001111 and FSEL[1:0] = 11 are required drive settings to meet the timing. Figure 42 depicts the timing of the PWM, and Table 73 lists the TPM timing parameters.

#### Notes

- See the IOMUXC chapter of the device reference manual to identify which pins expose Tx and Rx pins.
- The required drive settings (DSE[5:0] = 001111 and FSEL[1:0] = 11) are mentioned twice for both CAN-FD and TPM timing parameters.
- Refer to General purpose I/O (GPIO) AC parameters for additional timing parameters.
- Figure 42 and Table 73 are referenced for TPM timing details, but not included in this page.


## 4.12.9 FlexSPI timing parameters

*(Page 70)*

### TPM output timing parameters

| ID | Parameter | Min | Max | Unit |
| --- | --- | --- | --- | --- |
| PWM Module Clock Frequency | 0 | 83.3 | MHz |  |
| P1 | PWM output pulse width high | 12 | — | ns |
| P2 | PWM output pulse width low | 12 | — | ns |

### FlexSPI input timing in SDR mode where FlexSPI_MCR[RXCLKSRC] = 0X0 (Nominal and Overdrive mode)

| Symbol | Parameter | Min | Max | Unit |
| --- | --- | --- | --- | --- |
| — | Frequency of operation | — | 66 | MHz |
| F1 | Setup time for incoming data | 6 | — | ns |
| F2 | Hold time for incoming data | 0 | — | ns |

### FlexSPI input timing in SDR mode where FlexSPI_MCR[RXCLKSRC] = 0X0 (Low drive mode)

| Symbol | Parameter | Min | Max | Unit |
| --- | --- | --- | --- | --- |
| — | Frequency of operation | — | 50 | MHz |
| F1 | Setup time for incoming data | 7 | — | ns |
| F2 | Hold time for incoming data | 0 | — | ns |

### Electrical Specifications

| Parameter | Symbol | Min | Typ | Max | Unit | Conditions |
| --- | --- | --- | --- | --- | --- | --- |
| PWM Module Clock Frequency |  | 0 |  | 83.3 | MHz |  |
| PWM output pulse width high |  | 12 |  |  | ns |  |
| PWM output pulse width low |  | 12 |  |  | ns |  |
| Frequency of operation (Nominal and Overdrive mode) |  |  |  | 66 | MHz |  |
| Setup time for incoming data (Nominal and Overdrive mode) |  | 6 |  |  | ns |  |
| Hold time for incoming data (Nominal and Overdrive mode) |  | 0 |  |  | ns |  |
| Frequency of operation (Low drive mode) |  |  |  | 50 | MHz |  |
| Setup time for incoming data (Low drive mode) |  | 7 |  |  | ns |  |
| Hold time for incoming data (Low drive mode) |  | 0 |  |  | ns |  |

#### Additional Information

The FlexSPI interface can work in SDR or DDR modes. FlexSPI_MCR[RXCLKSRC] = 0 and FlexSPI_MCR[RXCLKSRC] = 1 configurations are supported when I/O is supplied by 3.3 V and 1.8 V, while FlexSPI_MCR[RXCLKSRC] = 3 configuration is supported when I/O is supplied by 1.8 V only. Input timing assumes an input slew rate of 1 ns (20%/80%) and Output timing valid for maximum external load CL = 15 pF, which is assumed to be a 8 pF load at the end of a 50 ohm, un-terminated, 2-inch microstrip trace on standard FR4 (3.3 pF/inch). For best signal integrity, the series resistance of the transmission line should be matched closely to the selected RDSON of the I/O pad output driver. The DSE[5:0] = 001111 and FSEL[1:0] = 11 are required drive settings to meet the timing. There are three sources for the internal sample clock of FlexSPI read data: - Dummy read strobe generated by FlexSPI controller and looped back internally (FlexSPI_MCR[RXCLKSRC] = 0x0) - Dummy read strobe generated by FlexSPI controller and looped back through the DQS pad (FlexSPI_MCR[RXCLKSRC] = 0x1) - Read strobe provided by memory device and input from DQS pad (FlexSPI_MCR[RXCLKSRC] = 0x3)

#### Notes

- The maximum frequency supported is 52 MHz when NVCC_xxxx operating at 3.3 V.
- The FlexSPI frequency supported is 52 MHz when FlexSPI_MCR[RXCLKSRC] = 0x0 and FlexSPI_MCR[RXCLKSRC] = 0x3.


## 4.12.9.1.2 SDR mode with FlexSPI_MCR0[RXCLKSRC] = 0x3

*(Page 71)*

### Table 76. FlexSPI input timing in SDR mode where FlexSPI_MCR0[RXCLKSRC] = 0x1 (Nominal and Overdrive mode)

| Symbol | Parameter | Min | Max | Unit |
| --- | --- | --- | --- | --- |
| — | Frequency of operation1 | — | 166 | MHz |
| F1 | Setup time for incoming data data | 1 | — | ns |
| F2 | Hold time for incoming data data | 1 | — | ns |

### Table 77. FlexSPI input timing in SDR mode where FlexSPI_MCR0[RXCLKSRC] = 0x1 (Low drive mode)

| Symbol | Parameter | Min | Max | Unit |
| --- | --- | --- | --- | --- |
| — | Frequency of operation1 | — | 100 | MHz |
| F1 | Setup time for incoming data data | 2 | — | ns |
| F2 | Hold time for incoming data data | 1 | — | ns |

### Table 78. FlexSPI input timing in SDR mode where FlexSPI_MCR0[RXCLKSRC] = 0x3 (case A1) (Nominal and Overdrive mode)

| Symbol | Parameter | Min | Max | Unit |
| --- | --- | --- | --- | --- |
| — | Frequency of operation | — | 200 | MHz |

### Electrical Specifications

| Parameter | Symbol | Min | Typ | Max | Unit | Conditions |
| --- | --- | --- | --- | --- | --- | --- |
| Maximum frequency supported | — | — |  | 52 MHz | MHz | NVCC_xxxx operating at 3.3 V |
| FlexSPI_SCLK frequency | — | — |  | 5 MHz | MHz | NVCC_xxxx operating at 3.3 V |

### FlexSPI_MCR0 (`Not specified`)

| Bits | Field | Access | Description |
| --- | --- | --- | --- |
| RXCLKSRC | RXCLKSRC | RW | Clock source for receive clock |

#### Additional Information

FlexSPI input timing in SDR mode where FlexSPI_MCR0[RXCLKSRC] = 0x1 (Nominal and Overdrive mode) FlexSPI input timing in SDR mode where FlexSPI_MCR0[RXCLKSRC] = 0x1 (Low drive mode) SDR mode with FlexSPI_MCR0[RXCLKSRC] = 0x3 There are two cases when the memory provides both read data and the read strobe in SDR mode: - A1: Memory generates both read data and read strobe on SCK rising edge (or falling edge) - A2: Memory generates read data on SCK falling edge and generates read strobe on SCK rising edge In this mode, it is only working under 1.8 V.

#### Notes

- 1. The maximum frequency supported is 52 MHz when NVCC_xxxx operating at 3.3 V.
- The maximum frequency supported is 52 MHz when NVCC_xxxx operating at 3.3 V.
- FlexSPI_SCLK is 5 MHz when NVCC_xxxx operating at 3.3 V.
- Timing shown is based on the memory generating read data on the SCK falling edge, and FlexSPI controller sampling read data on the falling edge.
- In this mode, it is only working under 1.8 V.


## FlexSPI Input Timing in SDR Mode

*(Page 72)*

### FlexSPI input timing in SDR mode where FlexSPIn_MCR0[RXCLKSRC] = 0x3 (case A1) (Nominal and Overdrive mode)

| Symbol | Parameter | Min | Max | Unit |
| --- | --- | --- | --- | --- |
| T_SCKD | Time from SCK to data valid | - | - | ns |
| T_SCKD | Time from SCK to DQS | - | - | ns |
| T_SCKDQS | Time from SCK to DQS | - | - | ns |
| T_SCKD - T_SCKDQS | Time delta between T_SCKD and T_SCKDQS | -0.6 | 0.6 | ns |

### FlexSPI input timing in SDR mode where FlexSPIn_MCR0[RXCLKSRC] = 0x3 (case A1) (Low drive mode)

| Symbol | Parameter | Min | Max | Unit |
| --- | --- | --- | --- | --- |
| — | Frequency of operation | - | 133 | MHz |
| T_SCKD | Time from SCK to data valid | - | - | ns |
| T_SCKDQS | Time from SCK to DQS | - | - | ns |
| T_SCKD - T_SCKDQS | Time delta between T_SCKD and T_SCKDQS | -2 | 2 | ns |

### FlexSPI input timing in SDR mode where FlexSPIn_MCR0[RXCLKSRC] = 0x3 (case A2) (Nominal and Overdrive mode)

| Symbol | Parameter | Min | Max | Unit |
| --- | --- | --- | --- | --- |
| — | Frequency of operation | - | 200 | MHz |
| T_SCKD | Time from SCK to data valid | - | - | ns |
| T_SCKDQS | Time from SCK to DQS | - | - | ns |
| T_SCKD - T_SCKDQS | Time delta between T_SCKD and T_SCKDQS | -0.6 | 0.6 | ns |

### Electrical Specifications

| Parameter | Symbol | Min | Typ | Max | Unit | Conditions |
| --- | --- | --- | --- | --- | --- | --- |
| Frequency of operation | — | - |  | 133 | MHz | Low drive mode |
| Frequency of operation | — | - |  | 200 | MHz | Nominal and Overdrive mode |
| Time from SCK to data valid | T_SCKD | - |  | - | ns | All modes |
| Time from SCK to DQS | T_SCKDQS | - |  | - | ns | All modes |
| Time delta between T_SCKD and T_SCKDQS | T_SCKD - T_SCKDQS | -2 |  | 2 | ns | Low drive mode |
| Time delta between T_SCKD and T_SCKDQS | T_SCKD - T_SCKDQS | -0.6 |  | 0.6 | ns | Nominal and Overdrive mode |

#### Additional Information

The timing shown is based on the memory generating read data and read strobe on the SCK rising edge. The FlexSPI controller samples read data on the DQS falling edge. FlexSPI input timing in SDR mode where FlexSPIn_MCR0[RXCLKSRC] = 0x3 (case A1) FlexSPI input timing in SDR mode where FlexSPIn_MCR0[RXCLKSRC] = 0x3 (case A2)

#### Notes

- Timing shown is based on the memory generating read data and read strobe on the SCK rising edge. The FlexSPI controller samples read data on the DQS falling edge.


## 4.12.9.1 DDR mode with FlexSPIn_MCR0[RXCLKSRC] = 0x0, 0x1

*(Page 73)*

### Table 81. FlexSPI input timing in SDR mode where FlexSPIn_MCR0[RXCLKSRC] = 0x3 (case A2) (Low drive mode)

| Symbol | Parameter | Value | Unit |
| --- | --- | --- | --- |
|  | Frequency of operation | 133 | MHz |
| T_SCKD | Time from SCK to data valid | -- | ns |
| T_SCKDQS | Time from SCK to DQS | -- | ns |
| T_SCKD - T_SCKDQS | Time delta between T_SCKD and T_SCKDQS | -2 | 2 | ns |
| T_SCKD - T_SCKDQS | Time delta between T_SCKD and T_SCKDQS | -2 | 2 | ns |

### Table 82. FlexSPI timing in DDR mode where FlexSPIn_MCR0[RXCLKSRC] = 0x0 (Nominal, Overdrive, and Low drive mode)

| Symbol | Parameter | Min | Max | Unit |
| --- | --- | --- | --- | --- |
|  | Frequency of operation | -- | 33 | MHz |
| F1 | Setup time for incoming data | 6 | -- | ns |
| F2 | Hold time for incoming data | 0 | -- | ns |

### Table 83. FlexSPI input timing in DDR mode where FlexSPIn_MCR0[RXCLKSRC] = 0x1 (Nominal and Overdrive mode)

| Symbol | Parameter | Min | Max | Unit |
| --- | --- | --- | --- | --- |
|  | Frequency of operation | -- | 83 | MHz |
| F1 | Setup time for incoming data | 1 | -- | ns |
| F2 | Hold time for incoming data | 1 | -- | ns |

### Electrical Specifications

| Parameter | Symbol | Min | Typ | Max | Unit | Conditions |
| --- | --- | --- | --- | --- | --- | --- |
| Frequency of operation | -- | -- |  | 133 | MHz | SDR mode, FlexSPIn_MCR0[RXCLKSRC] = 0x3 |
| Time from SCK to data valid | T_SCKD | -- |  | -- | ns | SDR mode, FlexSPIn_MCR0[RXCLKSRC] = 0x3 |
| Time from SCK to DQS | T_SCKDQS | -- |  | -- | ns | SDR mode, FlexSPIn_MCR0[RXCLKSRC] = 0x3 |
| Time delta between T_SCKD and T_SCKDQS | T_SCKD - T_SCKDQS | -2 |  | 2 | ns | SDR mode, FlexSPIn_MCR0[RXCLKSRC] = 0x3 |
| Frequency of operation | -- | -- |  | 33 | MHz | DDR mode, FlexSPIn_MCR0[RXCLKSRC] = 0x0 |
| Setup time for incoming data | F1 | 6 |  | -- | ns | DDR mode, FlexSPIn_MCR0[RXCLKSRC] = 0x0 |
| Hold time for incoming data | F2 | 0 |  | -- | ns | DDR mode, FlexSPIn_MCR0[RXCLKSRC] = 0x0 |
| Frequency of operation | -- | -- |  | 83 | MHz | DDR mode, FlexSPIn_MCR0[RXCLKSRC] = 0x1 |
| Setup time for incoming data | F1 | 1 |  | -- | ns | DDR mode, FlexSPIn_MCR0[RXCLKSRC] = 0x1 |
| Hold time for incoming data | F2 | 1 |  | -- | ns | DDR mode, FlexSPIn_MCR0[RXCLKSRC] = 0x1 |

#### Additional Information

Timing shown is based on the memory generating data on the SCK falling edge and read strobe on the SCK rising edge. The FlexSPI controller samples read data on a half cycle delayed DQS falling edge. The maximum frequency supported is 52 MHz when NVCC_xxxx operating at 3.3 V.

#### Notes

- Timing shown is based on the memory generating data on the SCK falling edge and read strobe on the SCK rising edge. The FlexSPI controller samples read data on a half cycle delayed DQS falling edge.
- The maximum frequency supported is 52 MHz when NVCC_xxxx operating at 3.3 V.


## 4.12.9.1.4 DDR mode with FlexSPI_MCR0[RXCLKSRC] = 0x3

*(Page 74)*

### Table 84. FlexSPI input timing in DDR mode where FlexSPI_MCR0[RXCLKSRC] = 0x1 (Low drive mode)

| Symbol | Parameter | Min | Max | Unit |
| --- | --- | --- | --- | --- |
| — | Frequency of operation1 | — | 66 | MHz |
| F1 | Setup time for incoming data | 1.5 | — | ns |
| F2 | Hold time for incoming data | 1 | — | ns |

### Table 85. FlexSPI input timing in DDR mode where FlexSPI_MCR0[RXCLKSRC] = 0x3 (Nominal and Overdrive mode)1

| Symbol | Parameter | Min | Max | Unit |
| --- | --- | --- | --- | --- |
| — | Frequency of operation | — | 200 | MHz |
| T_SCKD | Time from SCK to data valid | — | — | ns |
| T_SCKDQS | Time from SCK to DQS | — | — | ns |
| T_SCKD - T_SCKDQS | Time delta between T_SCKD and T_SCKDQS | -0.6 | 0.6 | ns |

### Table 86. FlexSPI input timing in DDR mode where FlexSPI_MCR0[RXCLKSRC] = 0x3 (Low drive mode)

| Symbol | Parameter | Min | Max | Unit |
| --- | --- | --- | --- | --- |
| — | Frequency of operation | — | 133 | MHz |
| T_SCKD | Time from SCK to data valid | — | — | ns |
| T_SCKDQS | Time from SCK to DQS | — | — | ns |
| T_SCKD - T_SCKDQS | Time delta between T_SCKD and T_SCKDQS | -0.9 | 0.9 | ns |

### Electrical Specifications

| Parameter | Symbol | Min | Typ | Max | Unit | Conditions |
| --- | --- | --- | --- | --- | --- | --- |
| Frequency of operation | — | — |  | 66 | MHz | FlexSPI_MCR0[RXCLKSRC] = 0x1 (Low drive mode) |
| Setup time for incoming data | F1 | 1.5 |  | — | ns | FlexSPI_MCR0[RXCLKSRC] = 0x1 (Low drive mode) |
| Hold time for incoming data | F2 | 1 |  | — | ns | FlexSPI_MCR0[RXCLKSRC] = 0x1 (Low drive mode) |
| Frequency of operation | — | — |  | 200 | MHz | FlexSPI_MCR0[RXCLKSRC] = 0x3 (Nominal and Overdrive mode) |
| Time from SCK to data valid | T_SCKD | — |  | — | ns | FlexSPI_MCR0[RXCLKSRC] = 0x3 (Nominal and Overdrive mode) |
| Time from SCK to DQS | T_SCKDQS | — |  | — | ns | FlexSPI_MCR0[RXCLKSRC] = 0x3 (Nominal and Overdrive mode) |
| Time delta between T_SCKD and T_SCKDQS | T_SCKD - T_SCKDQS | -0.6 |  | 0.6 | ns | FlexSPI_MCR0[RXCLKSRC] = 0x3 (Nominal and Overdrive mode) |
| Frequency of operation | — | — |  | 133 | MHz | FlexSPI_MCR0[RXCLKSRC] = 0x3 (Low drive mode) |
| Time from SCK to data valid | T_SCKD | — |  | — | ns | FlexSPI_MCR0[RXCLKSRC] = 0x3 (Low drive mode) |
| Time from SCK to DQS | T_SCKDQS | — |  | — | ns | FlexSPI_MCR0[RXCLKSRC] = 0x3 (Low drive mode) |
| Time delta between T_SCKD and T_SCKDQS | T_SCKD - T_SCKDQS | -0.9 |  | 0.9 | ns | FlexSPI_MCR0[RXCLKSRC] = 0x3 (Low drive mode) |

#### Additional Information

The maximum frequency supported is 52 MHz when NVCC_xxxx operating at 3.3 V. In this mode, it is only working under 1.8 V. These timing specifications are valid only for 1.8 V nominal I/O pad supply voltage.

#### Notes

- 1. The maximum frequency supported is 52 MHz when NVCC_xxxx operating at 3.3 V.
- 1. These timing specifications are valid only for 1.8 V nominal I/O pad supply voltage.


## 4.12.9.2 FlexSPI output/write timing

*(Page 75)*

### FlexSPI output timing in SDR mode (Nominal and Overdrive mode)

| Symbol | Parameter | Min | Max | Unit |
| --- | --- | --- | --- | --- |
| — | Frequency of operation | — | 200 | MHz |
| T_ck | SCK clock period | 5 | — | ns |
| T_DVO | Output data valid time | — | 0.6 | ns |
| T_DHO | Output data hold time | -0.6 | — | ns |
| T_CSS | Chip select output setup time | (T_CSS + 0.5) x T_ck - 0.6 | — | ns |
| T_CSH | Chip select output hold time | (T_CSH x T_ck) - 0.6 | — | ns |

### FlexSPI output timing in SDR mode (Low drive mode)

| Symbol | Parameter | Min | Max | Unit |
| --- | --- | --- | --- | --- |
| — | Frequency of operation | — | 133 | MHz |
| T_ck | SCK clock period | 7.5 | — | ns |
| T_DVO | Output data valid time | — | 2 | ns |
| T_DHO | Output data hold time | -2 | — | ns |

### Electrical Specifications

| Parameter | Symbol | Min | Typ | Max | Unit | Conditions |
| --- | --- | --- | --- | --- | --- | --- |
| Frequency of operation | — | — |  | 200 | MHz | Nominal and Overdrive mode |
| SCK clock period | T_ck | 5 |  | — | ns | Nominal and Overdrive mode |
| Output data valid time | T_DVO | — |  | 0.6 | ns | Nominal and Overdrive mode |
| Output data hold time | T_DHO | -0.6 |  | — | ns | Nominal and Overdrive mode |
| Chip select output setup time | T_CSS | (T_CSS + 0.5) x T_ck - 0.6 |  | — | ns | Nominal and Overdrive mode |
| Chip select output hold time | T_CSH | (T_CSH x T_ck) - 0.6 |  | — | ns | Nominal and Overdrive mode |
| Frequency of operation | — | — |  | 133 | MHz | Low drive mode |
| SCK clock period | T_ck | 7.5 |  | — | ns | Low drive mode |
| Output data valid time | T_DVO | — |  | 2 | ns | Low drive mode |
| Output data hold time | T_DHO | -2 |  | — | ns | Low drive mode |

#### Additional Information

This page describes the FlexSPI output timing specifications for the i.MX 93 Applications Processor in SDR mode. It includes timing parameters for both Nominal/Overdrive mode and Low drive mode. The timing specifications are valid for 1.8 V nominal I/O pad supply voltage. The maximum supported frequency is 52 MHz when NVCC_xxxx is operating at 3.3 V. T_CSS and T_CSH are configurable via the FlexSPI_N_FLSHAxCR1 register.

#### Notes

- 1. These timing specifications are valid only for 1.8 V nominal I/O pad supply voltage.
- 2. The maximum specifications frequency supported is 52 MHz when NVCC_xxxx operating at 3.3 V.
- 3. T_CSS and T_CSH are configured by the FlexSPI_N_FLSHAxCR1 register. See i.MX 93 Applications Processor Reference Manual (IMX93RM) for more details.


## 4.12.9.2.2 DDR mode

*(Page 76)*

### Table 88. FlexSPI output timing in SDR mode (Low drive mode) ...continued

| Symbol | Parameter | Min | Max | Unit |
| --- | --- | --- | --- | --- |
| T_CSS | Chip select output setup time | (T_CS + 0.5) x T_ck - 2 | — | ns |
| T_CSH | Chip select output hold time | (T_CSH x T_ck) - 2 | — | ns |

### Table 89. FlexSPI output timing in DDR mode (Nominal and Overdrive mode)1

| Symbol | Parameter | Min | Max | Unit |
| --- | --- | --- | --- | --- |
| — | Frequency of operation2 | — | 200 | MHz |
| T_ck | SCK clock period | 5 | — | ns |
| T_DVO | Output data valid time | — | 1.815 | ns |
| T_DHO | Output data hold time | 0.615 | — | ns |
| T_CSS | Chip select output setup time | (T_CSS + 0.5) x T_ck - 0.6 | — | ns |
| T_CSH | Chip select output hold time3 | (T_CSH + 0.5) x T_ck - 0.6 | — | ns |

### Electrical Specifications

| Parameter | Symbol | Min | Typ | Max | Unit | Conditions |
| --- | --- | --- | --- | --- | --- | --- |
| Maximum frequency supported | — | — |  | 52 MHz | MHz | NVCC_xxxx operating at 3.3 V |
| Frequency of operation | — | — |  | 200 | MHz | DDR mode |
| SCK clock period | T_ck | 5 |  | — | ns | DDR mode |
| Output data valid time | T_DVO | — |  | 1.815 | ns | DDR mode |
| Output data hold time | T_DHO | 0.615 |  | — | ns | DDR mode |
| Chip select output setup time | T_CSS | (T_CSS + 0.5) x T_ck - 0.6 |  | — | ns | DDR mode |
| Chip select output hold time | T_CSH | (T_CSH + 0.5) x T_ck - 0.6 |  | — | ns | DDR mode |

### FlexSPI_n_MCR0[RXCLKSRC] (`—`)

| Bits | Field | Access | Description |
| --- | --- | --- | --- |
| — | RXCLKSRC | RW | Configuration used to limit the maximum frequency supported |

### FlexSPI_n_FLSHAxCR1 (`—`)

| Bits | Field | Access | Description |
| --- | --- | --- | --- |
| — | — | RW | Configures T_CSS and T_CSH |

#### Additional Information

The maximum frequency supported is 52 MHz when NVCC_xxxx operating at 3.3 V. The actual maximum frequency supported is limited by the FlexSPI_n_MCR0[RXCLKSRC] configuration used. T_CSS and T_CSH are configured by the FlexSPI_n_FLSHAxCR1 register. Figure 48 shows FlexSPI output timing in SDR mode. Specifications in DDR mode are valid only for 1.8 V nominal IO pad supply voltage.

#### Notes

- The maximum frequency supported is 52 MHz when NVCC_xxxx operating at 3.3 V.
- The actual maximum frequency supported is limited by the FlexSPI_n_MCR0[RXCLKSRC] configuration used.
- T_CSS and T_CSH are configured by the FlexSPI_n_FLSHAxCR1 register.
- Specifications in DDR mode are valid only for 1.8 V nominal IO pad supply voltage.


## 4.12.10 LPUART I/O configuration and timing parameters

*(Page 77)*

### Table 90. FlexSPI output timing in DDR mode (Low drive mode)

| Symbol | Parameter | Min | Max | Unit |
| --- | --- | --- | --- | --- |
| — | Frequency of operation | — | 133 | MHz |
| T_ck | SCK clock period | 7.5 | — | ns |
| T_DVO | Output data valid time | — | 2.75 | ns |
| T_DHO | Output data hold time | 0.9 | — | ns |
| T_CSS and T_CSH | Chip select output setup time | (T_css + 0.5) x T_ck - 0.9 | — | ns |
| T_CSS | Chip select output setup time | (T_css + 0.5) x T_ck - 0.9 | — | ns |
| T_CSH | Chip select output hold time | (T_csh + 0.5) x T_ck - 0.9 | — | ns |

### Table 91. FlexIO timing specifications

| Symbol | Descriptions | Min | Typ | Max | Unit | Notes |
| --- | --- | --- | --- | --- | --- | --- |
| t_ODS | Output delay skew between any two FlexIO_Dx pins configured as outputs that toggle on the same internal clock cycle | 0 | — | 12 | ns | 3 |

### Electrical Specifications

| Parameter | Symbol | Min | Typ | Max | Unit | Conditions |
| --- | --- | --- | --- | --- | --- | --- |
| Frequency of operation | — | — |  | 133 | MHz | DDR mode (Low drive mode) |
| SCK clock period | T_ck | 7.5 |  | — | ns | DDR mode (Low drive mode) |
| Output data valid time | T_DVO | — |  | 2.75 | ns | DDR mode (Low drive mode) |
| Output data hold time | T_DHO | 0.9 |  | — | ns | DDR mode (Low drive mode) |
| Chip select output setup time | T_CSS | (T_css + 0.5) x T_ck - 0.9 |  | — | ns | DDR mode (Low drive mode) |
| Chip select output hold time | T_CSH | (T_csh + 0.5) x T_ck - 0.9 |  | — | ns | DDR mode (Low drive mode) |
| Output delay skew | t_ODS | 0 | — | 12 | ns | FlexIO pins configured as outputs toggling on the same internal clock cycle |

### FlexSPI_n_MCR[0]RXCLKSRC (`Not specified`)

| Bits | Field | Access | Description |
| --- | --- | --- | --- |
| Not specified | RXCLKSRC | RW | Configuration used to limit the actual maximum frequency supported |

### FlexSPI_n_FLASHACR1 (`Not specified`)

| Bits | Field | Access | Description |
| --- | --- | --- | --- |
| Not specified | T_CSS and T_CSH | RW | Configures chip select output setup and hold times |

#### Additional Information

The maximum frequency supported is 52 MHz when NVCC_xxxx operating at 3.3 V. The actual maximum frequency supported is limited by the FlexSPI_n_MCR[0]RXCLKSRC configuration used. T_CSS and T_CSH are configured by the FlexSPI_n_FLASHACR1 register. The DSE[5:0] = 001111 and FSEL1[1:0] = 11 are required drive settings to meet the timing.

#### Notes

- The maximum frequency supported is 52 MHz when NVCC_xxxx operating at 3.3 V.
- The actual maximum frequency supported is limited by the FlexSPI_n_MCR[0]RXCLKSRC configuration used, see the FlexSPI DDR input timing specifications.
- T_CSS and T_CSH are configured by the FlexSPI_n_FLASHACR1 register. See i.MX 93 Applications Processor Reference Manual (IMX93RM) for more details.
- The DSE[5:0] = 001111 and FSEL1[1:0] = 11 are required drive settings to meet the timing.


## 5 Boot mode mode configuration

*(Page 78)*

### Table 91. FlexIO timing specifications

| Symbol | Description | Min | Typ | Max | Unit | Notes |
| --- | --- | --- | --- | --- | --- | --- |
| tIDS | Input delay delay skew between any two FlexIO_Dx pins between any two FlexIO_Dx pins configured as inputs that are sampled on the same internal clock cycle | 0 | — | 12 | ns | 3 |

### Electrical Specifications

| Parameter | Symbol | Min | Typ | Max | Unit | Conditions |
| --- | --- | --- | --- | --- | --- | --- |
| Input delay delay skew (tIDS) | tIDS | 0 | — | 12 | ns | Between any two FlexIO_Dx pins configured as inputs that are sampled on the same internal clock cycle |

#### Additional Information

{'usb_phy_parameters': {'compliance': 'Meets electrical compliance requirements of USB 2.0 Specification (including ECNs and errata), On-The-Go and Embedded Host Supplement to the Universal Serial Bus Revision 2.0 Specification (including ECNs and errata)', 'connections': {'usb_vbus_pin': 'Cannot directly connect to 5 V VBUS voltage on USB 2.0 link', 'isolation': 'Each USBx_VBUS pin must be isolated by an external 30 KΩ 1% precision resistor', 'calibration': 'USB 2.0 PHY uses USBx_TXRTUNE and an external resistor to calibrate USB_DP/DN 45 Ω source impedance. External resistor value is 200 Ω 1% precision on each of USBx_TXRTUNE pad to ground'}}, 'boot_mode_configuration': {'overview': 'Provides information on boot mode configuration pins allocation and boot devices interfaces allocation', 'supported_modes': ['Normal Boot Mode', 'Boot from Internal Fuse Mode', 'Boot from Serial Download Boot Mode'], 'boot_types': [{'type': 'Single Boot', 'description': 'Cortex-A55 core loads all containers and images, while Cortex-M33 core waits for firmware'}, {'type': 'Low Power Boot (LPB)', 'description': 'Only Cortex-M33 core is running after POR. Cortex-A55 core cannot be triggered by Cortex-M33 firmware'}], 'selection': 'Boot modes can be selected via different boot mode pins or overridden by fuses', 'detailed_config': "See 'Fuse Map' and 'System Boot' chapter in i.MX 93 Reference Manual (IMX93RM)"}, 'boot_mode_pins': {'overview': 'Four boot mode pins are used to select boot mode'}}

#### Notes

- {'number': '1', 'text': 'Input timing assumes an input signal slew rate of 3 ns (20%/80%)'}
- {'number': '2', 'text': 'Output timing assumes valid for maximum external load CL = 25 pF, which is assumed to be a 10 pF load at the end of a 50 ohm. Unterminated, 5-inch microstrip trace on standard FR4 (3.3 pF/inch, 25 pF total with margin). For best signal integrity, the series resistance in the transmission line should be equal to the selected RDSON of the I/O pad output driver'}
- {'number': '3', 'text': 'Assume pins muxed on same VDD_IO domain with same load'}


## Fuses and associated pins used for boot

*(Page 79)*

### Fuses and associated pins used for boot

| BOOT_MODE[3:0] | Function |
| --- | --- |
| x000 | Boot from Internal Fuses |
| 0001 | Serial Download (USB1) |
| 0010 | uSDHC1 8-bit eMMC 5.1 |
| 0011 | uSDHC2 4-bit SD 3.0 |
| 0100 | FlexSPI Serial NOR |
| 0101 | FlexSPI Serial NAND 2K |
| 0110 | Reserved |
| 0111 | Reserved |
| 1000 | LPB: Boot from Internal Fuses |
| 1001 | LPB: Serial Downloader (USB1) |
| 1010 | LPB: uSDHC1 8-bit 1.8 V eMMC 5.1 |
| 1011 | LPB: uSDHC2 4-bit SD 3.0 |
| 1100 | LPB: FlexSPI Serial NOR |
| 1101 | LPB: FlexSPI Serial NAND 2K |
| 1110 | Reserved |
| 1111 | Reserved |

#### Additional Information

HW samples the boot CFG pins before ROM starts, these pins should be mapped to Boot CFG pins by default. Once HW samples the boot CFG pins and stores the boot CFG in CMC register, the register should be latched. The register value is no more changes and reflecting the pins status. Additional boot options are also supported for both Normal Boot Boot Mode and Internal Fuse Fuse mode: - All boot modes support a range of speeds, timings, and protocol formats. - eMMC and SD boot can be supported from any USDHC instance 1 or 2. - Serial NOR boot supports 1-bit, 4-bit, and 8-bit mode. - Serial NAND boot supports 1-bit, 4-bit, and 8-bit mode (8-bit Serial NAND). BOOT_MODE pins are multiplexed over other functional pins. The functional I/O that are multiplexed with these pins must be selected subject to two criteria: - Functional I/O must not be used if they are inputs to the SoC, which could potentially be constantly driven by external components. Such functional mode driving may interfere with the need for the board to pull these pins a certain way while POR is asserted. - Functional I/O must not be used if they are outputs of the SoC, which will be connected to components on the board that may misinterpret the signals as valid signals if they are toggled.

#### Notes

- HW samples the boot CFG pins before ROM starts, these pins should be mapped to Boot CFG pins by default.
- Once HW samples the boot CFG pins and stores the boot CFG in CMC register, the register should be latched. The register value is no more changes and reflecting the pins status.
- BOOT_MODE pins are multiplexed over other functional pins. The functional I/O that are multiplexed with these pins must be selected subject to two criteria:


## 5.2 Boot device interface allocation

*(Page 80)*

### Table 93. Boot through FlexSPI

| Signal name | PAD name | ALT |
| --- | --- | --- |
| FlexSPIA_DATA00 | SD3_DATA0 | ALT1 |
| FlexSPIA_DATA11 | SD3_DATA1 | ALT1 |
| FlexSPIA_DATA22 | SD3_DATA2 | ALT1 |
| FlexSPIA_DATA33 | SD3_DATA3 | ALT1 |
| FlexSPIA_DQS | SD1_STROBE | ALT1 |
| FlexSPIA_SSQS | SD1_STROBE | ALT1 |
| FlexSPIA_SS_0_B | SD3_CMD | ALT1 |
| FlexSPIA_SCLK | SD3_CLK | ALT1 |
| FlexSPIA_DATA44 | SD1_DATA4 | ALT1 |
| FlexSPIA_DATA55 | SD1_DATA5 | ALT1 |
| FlexSPIA_DATA66 | SD1_DATA6 | ALT1 |
| FlexSPIA_DATA66 | SD1_DATA6 | ALT1 |
| FlexSPIA_DATA77 | SD1_DATA7 | ALT1 |

### Table 94. Boot through uSDHC1

| Signal name | PAD name | ALT |
| --- | --- | --- |
| USDHC1_CMD | SD1_CMD | ALT0 |
| USDHC1_CLK | SD1_CLK | ALT0 |
| USDHC1_CLK | SD1_CLK | ALT0 |
| USDHC1_DATA0 | SD1_DATA0 | ALT0 |

#### Additional Information

The page discusses the boot device interface allocation for the i.MX 93 Applications Processor. It supports three kinds of boot devices: Primary Boot Device, Recovery Boot Device, and Serial Download Boot Device. The primary boot device is selected by Boot Config pins in Normal Boot or Internal Fuses Boot mode. Valid options for the primary boot device include SD/eMMC/FlexSPI NOR/FlexSPI NAND. The recovery boot device is only from SPI1/2/3/4. Both Cortex-M33 and Cortex-A55 support serial download download mode via USB1. The tables list the interfaces used for booting through FlexSPI and uSDHC1.

#### Notes

- The tables describe the interface's specific modes and IOMUXC allocation, which are configured during boot when appropriate.
- The tables continue on the next page.


## i.MX 93 Applications Processors Data Sheet for Automotive Products

*(Page 81)*

### Table 94. Boot through uSDHC1...continued

| Signal name | PAD name | ALT |
| --- | --- | --- |
| USDHCI_DATA1 | SD1_DATA1 | ALT0 |
| USDHCI_DATA2 | SD1_DATA2 | ALT0 |
| USDHCI_DATA3 | SD1_DATA3 | ALT0 |
| USDHCI_DATA4 | SD1_DATA4 | ALT0 |
| USDHCI_DATA5 | SD1_DATA5 | ALT0 |
| USDHCI_DATA6 | SD1_DATA6 | ALT0 |
| USDHCI_DATA6 | SD1_DATA6 | ALT0 |
| USDHCI_DATA7 | SD1_DATA7 | ALT0 |
| USDHCI_RESET | SD1_DATA5 | ALT2 |

### Table 95. Boot through uSDHC2

| Signal name | PAD name | ALT |
| --- | --- | --- |
| USDHCI_CMD | SD2_CMD | ALT0 |
| USDHCI_CLK | SD2_CLK | ALT0 |
| USDHCI_DATA00 | SD2_DATA0 | ALT0 |
| USDHCI_DATA11 | SD2_DATA1 | ALT0 |
| USDHCI_DATA22 | SD2_DATA2 | ALT0 |
| USDHCI_DATA33 | SD2_DATA3 | ALT0 |
| USDHCI_DATA33 | SD2_DATA3 | ALT0 |
| USDHCI_RESET | SD2_RESET_B | ALT0 |
| USDHCI_VSELECT | SD2_VSELECT | ALT0 |

### Table 96. Boot through SPI1

| Signal name | PAD name | ALT |
| --- | --- | --- |
| SPI1_PCS1 | PDM_BIT_STREAM0 | ALT2 |
| SPI1_SIN | SAI1_TXC | ALT2 |
| SPI1_SOUT | SAI1_RXD0 | ALT2 |
| SPI1_SCK | SAI1_TXD0 | ALT2 |
| SPI1_PCS0 | SAI1_TXFS | ALT2 |

#### Additional Information

This page contains pin configuration information for booting through different interfaces (uSDHC1, uSDHC2, and SPI1) on the i.MX 93 Applications Processor. The tables detail the signal names, corresponding PAD names, and alternate function (ALT) settings for each pin used in these interfaces.


## 6 Package information and contact assignments

*(Page 82)*

### Table 97. Boot through SPI2

| Signal name | PAD name | ALT |
| --- | --- | --- |
| SPI2_PCS1 | PDM_BIT_STREAM1 | ALT2 |
| SPI2_SIN | UART1_RXD | ALT2 |
| SPI2_SOUT | UART1_RXD | ALT2 |
| SPI2_SCK | UART1_TXD | ALT2 |
| SPI2_PCS0 | UART1_TXD | ALT2 |

### Table 98. Boot through SPI3

| Signal name | PAD name | ALT |
| --- | --- | --- |
| SPI3_PCS1 | GPIO_IO07 | ALT1 |
| SPI3_SIN | GPIO_IO09 | ALT1 |
| SPI3_SOUT | GPIO_IO10 | ALT1 |
| SPI3_SCK | GPIO_IO11 | ALT1 |
| SPI3_PCS0 | GPIO_IO08 | ALT1 |

### Table 99. Boot through SPI4

| Signal name | PAD name | ALT |
| --- | --- | --- |
| SPI4_PCS1 | GPIO_IO17 | ALT5 |
| SPI4_PCS2 | GPIO_IO16 | ALT5 |
| SPI4_SIN | GPIO_IO19 | ALT5 |
| SPI4_SOUT | GPIO_IO20 | ALT5 |
| SPI4_SCK | GPIO_IO21 | ALT5 |
| SPI4_PCS0 | GPIO_IO18 | ALT5 |

#### Additional Information

USB1 interfaces are dedicated pins, thus no IOMUX options. This section includes the contact assignment information and mechanical package drawing.

#### Notes

- USB1 interfaces are dedicated pins, thus no IOMUX options.


## 6.1.1 14 x 14 mm, 0.65 mm pitch, ball matrix

*(Page 83)*

#### Additional Information

This page describes the package specifications for the i.MX 93 Applications Processor in the IMX93AEC variant. The package is a 14 x 14 mm, 0.65 mm pitch, ball matrix FCBGA package. Figure 50 is referenced, which shows the top, bottom, and side views of the package.


## 14 x 14 mm BGA, case x package top, bottom, and side Views

*(Page 84)*

#### Additional Information

This page provides mechanical outline drawings for the i.MX 93 Applications Processor in a 14 x 14 mm BGA package (case x). The drawings include top, bottom, and side views with detailed dimensions and specifications. Key details include:
- Package type: FC-PBGA-306 I/O
- Package size: 14 x 14 x 1.144 mm
- Ball pitch: 0.65 mm
- Number of balls: 306
- Index area: Located at pin A1
- Seating plane: Defined with specific tolerances
- Ball diameter: 0.39 mm (typical)
- Ball pitch: 0.65 mm
- Package height: 1.144 mm
- Revision: X0
- Drawing number: 98ASA01893D
- Date: 17 May 2022

#### Notes

- Released for external assembly only. This design only meets external design and assembly rules. Must be reviewed only and updated before being assembled internally.
- Print version not to scale.
- All information provided in this document is subject to legal disclaimers.


## 6.1.2 14 x 14 mm supplies contact assignments and functional contact assignments

*(Page 85)*

### 14 x 14 mm supplies contact list assignment

| Supply Rail Rail Name | Ball(s) Position(s) | Remark |
| --- | --- | --- |
| NVCC_AON | L16 | — |
| NVCC_BBSM_1P8 | G12 | — |
| NVCC_GPIO | N15, N16 | — |
| NVCC_SD2 | R16 | — |
| NVCC_WAKEUP | R10, R12, W8 | — |
| VDD_ANA_0P8 | J15, J16, R8 | — |
| VDD_ANA_0P8 | J5, J6, R4 | — |
| VDD_ANA_1P8 | F16, G16 | — |
| VDD_ANA_1P8 | R8 | — |
| VDD_ANAVDET_1P8 | L15 | — |
| VDD_BBSM_0P8_CAP | G14 | — |
| VDD_LVDS_1P8 | F6 | — |
| VDD_MIPI_1P8 | F8 | — |
| VDD_MIPI_0P8 | G8 | — |
| VDD_MIPI_1P8 | F8 | — |
| VDD_SOC | J9, J10, J11, J12, J13, K9, K10, K12, K13, M9, M10, M12, M13, N9, N10, N11, N12, N13 | — |
| VDD_USB_0P8 | F10 | — |
| VDD_USB_0P8 | F10 | — |
| VDD_USB_1P8 | E8 | — |
| VDD_USB_3P3 | G10 | — |
| VDD2_DDR | L7, N6, N7, R6, T6 | — |
| VDDQ_DDR | G6, J6, J7, L6 | — |
| VSS | A1, A21, C2, C4, C6, C8, C10, C12, C14, C16, C18, E3, E19, G3, G19, H8, H10, H12, H14, J3, J5, J8, J14, J19, K11, L1, L3, L5, L8, L14, L19, M11, N3, N5, N8, N14, N19, P8, P10, P12, P14, R3, R19, T1, U3, U19, W4, W6, W10, W12, W14, W16, W18, AA1, AA21 | — |

#### Additional Information

This page discusses the contact assignments for the 14 x 14 mm package of the i.MX 93 Applications Processor. It provides a detailed list of supply rail names, their corresponding ball positions, and any remarks. Table 100 shows the device connection list for ground, sense, and reference contact signals. Table 101 (not fully visible) is mentioned to show an alpha-sorted list of functional contact assignments for the 14 x 14 mm package.


## 14 x 14 mm functional contact assignment

*(Page 86)*

### 14 x 14 mm functional contact assignment

| Ball name | 14 x 14 ball | Power group | Ball Types | Default setting | Default setting | Default setting | Status while reset is asserted |
| --- | --- | --- | --- | --- | --- | --- | --- |
| ADC_IN0 | B19 | VDD_ANA_P8 | ANALOG | — | — | — | Input without PU1 / PD2 |
| ADC_IN01 | A20 | VDD_ANA_1_P8 | ANALOG | — | — | — | Input without PU / PD |
| ADC_IN1 | A20 | VDD_ANA_1_P8 | ANALOG | — | — | — | Input without PU / PD |
| ADC_IN2 | B20 | VDD_ANA_1_P8 | ANALOG | — | — | — | Input without PU / PD |
| ADC_IN3 | B21 | VDD_ANA_1_P8 | ANALOG | — | — | — | Input without PU / PD |
| ADC_INK | B21 | VDD_ANA_1_P8 | ANALOG | — | — | — | Input without PU / PD |
| CCM_CLKO1 | AA2 | NVCC_WAKEUP | GPIO | Alt0 | CCMSRCGPCMIX.CLK01 | — | Output low |
| CCM_CLKO2 | Y | NVCC_WAKEUP | GPIO | Alt0 | CCMSRCGPCMIX.CLK02 | — | Output low |
| CCM_CLKO2 | Y3 | NVCC_WAKEUP | GPIO | Alt | CCMSRCGPCMIX.CLK02 | — | Output low |
| CCM_CLKO3 | U4 | NVCC_WAKEUP | GPIO | Alt5 | GPIO4.IO[28] | — | Input with PD |
| CCM_CLKO4 | V4 | NVCC_WAKEUP | GPIO | Alt5 | GPIO4.IO[29] | — | Input with PD |
| CLKIN1 | B17 | VDD_ANA_P8 | ANALOG | — | — | — | Input without PU / PD |
| CLKIN2 | A18 | VDD_ANA_1_P8 | ANALOG | — | — | — | Input without PU / PD |
| DAP_TCLK_SWCLK | Y1 | NVCC_WAKEUP | GPIO | Alt0 | DAP.TCLK_SWCLK | — | Input with PD |
| DAP_TDI | W1 | NVCC_WAKEUP | GPIO | Alt0 | DAP.TDI | — | Input with PU |
| DAP_TDO_TRACESWO | Y2 | NVCC_WAKEUP | GPIO | Alt0 | DAP.TDO_TRACESWO | — | Input without PU / PD |
| DAP_TMS_SWDIO | W2 | NVCC_WAKEUP | GPIO | Alt0 | DAP.TMS_SWDIO | — | Input with PU |
| DRAM_CA0_A | H2 | VDD2_DDR | DDR | — | — | — | — |
| DRAM_CA1_A | G1 | VDD2_DDR | DDR | — | — | — | — |
| DRAM_CA2_A | F2 | VDD2_DDR | DDR | — | — | — | — |
| DRAM_CA3_A | E1 | VDD2_DDR | DDR | — | — | — | — |
| DRAM_CA3_A | E1 | VDD2_DDR | DDR | — | — | — | — |
| DRAM_CA4_A | E2 | VDD2_DDR | DDR | — | — | — | — |
| DRAM_CA5_A | D1 | VDD2_DDR | DDR | — | — | — | — |

#### Additional Information

This page contains a table detailing the functional contact assignment for a 14 x 14 mm ball grid array (BGA) package of the i.MX 93 Applications Processor. The table lists ball names, their corresponding ball positions, power groups, ball types, default settings, default functions, and the status while reset is asserted.

#### Notes

- 1. PU: Pull-up resistor
- 2. PD: Pull-down resistor

### 14 x 14 mm functional contact assignment ...continued

| Ball name | 14 x 14 | Power group | Ball Types | Default setting | Default setting | Default setting | Status while reset is asserted |
| --- | --- | --- | --- | --- | --- | --- | --- |
| DRAM_CK_C_A | G5 | VDD2_DDR | DDR | — | — | — | — |
| DRAM_CK_T_A | G4 | VDD2_DDR | DDR | — | — | — | — |
| DRAM_CKE0_A | H1 | VDD2_DDR | DDR | — | — | — | — |
| DRAM_CKE0_A | H1 | VDD2_DDR | DDR | — | — | — | — |
| DRAM_CKE1_A | J4 | VDD_DDR | DDR | — | — | — | — |
| DRAM_CS0_A | F1 | VDD2_DDR | DDR | — | — | — | — |
| DRAM_CS1_A | G2 | VDD2_DDR | DDR | — | — | — | — |
| DRAM_DMI0_A | L2 | VDDQ_DDR | DDR | — | — | — | — |
| DRAM_DMI1_A | T2 | VDDQ_DDR | DDR | — | — | — | — |
| DRAM_DMI1_A | N | VDDQ_DDR | DDR | — | — | — | — |
| DRAM_DQ00_A | N1 | VDDO_DDR | DDR | — | — | — | — |
| DRAM_DQ01_A | N2 | VDDO_DDR | DDR | — | — | — | — |
| DRAM_DQ02_A | M1 | VDDQ_DDR | DDR | — | — | — | — |
| DRAM_DQ03_A | M2 | VDDQ_DDR | DDR | — | — | — | — |
| DRAM_DQ04_A | K1 | VDDQ_DDR | DDR | — | — | — | — |
| DRAM_DQ05_A | K2 | VDD_DDR | DDR | — | — | — | — |
| DRAM_DQ05_A | K2 | VDDQ_DDR | DDR | — | — | — | — |
| DRAM_DQ06_A | J1 | VDD_DDR | DDR | — | — | — | — |
| DRAM_DQ07_A | J2 | VDD_DDR | DDR | — | — | — | — |
| DRAM_DQ08_A | V1 | VDD_DDR | DDR | — | — | — | — |
| DRAM_DQ09_A | V2 | VDD_DDR | DDR | — | — | — | — |
| DRAM_DQ10_A | U2 | VDD_DDR | DDR | — | — | — | — |
| DRAM_DQ11_A | U1 | VDDQ_DDR | DDR | — | — | — | — |
| DRAM_DQ11_A | U1 | VDDQ_DDR | DDR | — | — | — | — |
| DRAM_DQ12_A | R1 | VDD_DDR | DDR | — | — | — | — |
| DRAM_DQ13_A | R2 | VDD_DDR | DDR | — | — | — | — |
| DRAM_DQ13_A | R22 | VDD_DDR | DDR | — | — | — | — |
| DRAM_DQ14_A | P2 | VDD_DDR | DDR | — | — | — | — |

#### Additional Information

This page is part of the i.MX 93 Applications Processor datasheet, specifically detailing the 14 x 14 mm functional contact assignment for the DDR interface. The table lists ball names, their corresponding 14 x 14 ball positions, power groups, ball types, and default settings. The table continues on the next page.

### 14 x 14 mm functional contact assignment

| Ball name | 14 x 14 | Power group | Ball Types | Default setting | Default setting | Status while reset is asserted |
| --- | --- | --- | --- | --- | --- | --- |
| DRAM_DQ15_A | P1 | VDDQ_DDR | DDR | — | — | — |
| DRAM_DQS0_C_A | L4 | VDDQ_DDR | — | — | — | — |
| DRAM_DQS0_T | N4 | VDDQ_DDR | DDRCCLK | — | — | — |
| DRAM_DQS0_T_A | N4 | VDDQ_DDR | DDRCCLK | — | — | — |
| DRAM_DQS1_C_A | R5 | VDDQ_DDR | — | — | — | — |
| DRAM_DQS1_T_A | R4 | VDDQ_DDR | DDRCCLK | — | — | — |
| DRAM_MTEST1 | D4 | VDD2_DDR | DDR | — | — | — |
| DRAM_RESET_RESET_N | D2 | VDDQ_DDR | DDR | — | — | — |
| DRAM_ZQ | E4 | VDDQ_DDR | DDR | — | — | — |
| ENET1_MDC | AA11 | NVCC_WAKEUP | GPIO | Alt5 | GPIO4.IO[0] | Input with PD |
| ENET_MDC | AA11 | NVCC_WAKEUP | GPIO | Alt5 | GPIO4.IO[0] | Input with PD |
| ENET_MDIO | AA10 | NVCC_WAKEUP | GPIO | Alt5 | GPIO4.IO[1] | Input with PD |
| ENET_RD0 | AA8 | NVCC_WAKEUP | GPIO | Alt5 | GPIO4.IO[10] | Input with PD |
| ENET_RD1 | Y9 | NVCC_WAKEUP | GPIO | Alt5 | GPIO4.IO[11] | Input with PD |
| ENET_RD2 | AA9 | NVCC_WAKEUP | GPIO | Alt5 | GPIO4.IO[12] | Input with PD |
| ENET_RD3 | Y10 | NVCC_WAKEUP | GPIO | Alt5 | GPIO4.IO[13] | Input with PD |
| ENET1_RD3_CTL | Y10 | NVCC_WAKEUP | GPIO | Alt5 | GPIO4.IO[13] | Input with PD |
| ENET1_RX_CTL | Y8 | NVCC_WAKEUP | GPIO | Alt5 | GPIO4.IO[8] | Input with PD |
| ENET1_RXC | AA7 | NVCC_WAKEUP | GPIO | Alt5 | GPIO4.IO[9] | Input with PD |
| ENET1_TD0 | W11 | NVCC_WAKEUP | GPIO | Alt5 | GPIO4.IO[5] | Input with PD |
| ENET1_TD1 | T12 | NVCC_WAKEUP | GPIO | Alt5 | GPIO4.IO[4] | Input with PD |
| ENET1_TD2 | U12 | NVCC_WAKEUP | GPIO | Alt5 | GPIO4.IO[3] | Input with PD |
| ENET1_TD3 | V12 | NVCC_WAKEUP | GPIO | Alt5 | GPIO4.IO[2] | Input with PD |
| ENET1_TX_CTL | V10 | NVCC_WAKEUP | GPIO | Alt5 | GPIO4.IO[6] | Input with PD |
| ENET1_TXC | U10 | NVCC_WAKEUP | GPIO | Alt5 | GPIO4.IO[7] | Input with PD |
| ENET2_MDC | Y7 | NVCC_WAKEUP | GPIO | Alt5 | GPIO4.IO[14] | Input with PD |

#### Additional Information

This page contains a continuation of Table 101, which details the functional contact assignment for the 14 x 14 mm ball grid array (BGA) package of the i.MX 93 Applications Processor. The table lists ball names, their corresponding 14 x 14 ball positions, power groups, ball types, default settings, default functions, and the status while reset is asserted.

#### Notes

- The table continues on the next page.

### 14 x 14 mm functional contact assignment

| Ball name | 14 x 14 ball | Power group | Ball Types | Default setting | Default setting | Default setting | Status while reset is asserted |
| --- | --- | --- | --- | --- | --- | --- | --- |
| ENET_MDIO | AA6 | NVCC_WAKEUP | GPIO | Alt5 | GPIO4.IO[15] | Input with PD |  |
| ENET2_RD0 | AA4 | NVCC_WAKEUP | GPIO | Alt5 | GPIO4.IO[24] | Input with PD |  |
| ENET_RD0 | Y | NVCC_WAKEUP | GPIO | Alt5 | GPIO4.IO[25] | Input with PD |  |
| ENET2_RD1 | Y5 | NVCC_WAKEUP | GPIO | Alt5 | GPIO4.IO[25] | Input with PD |  |
| ENET2_RD2 | AA5 | NVCC_WAKEUP | GPIO | Alt5 | GPIO4.IO[26] | Input with PD |  |
| ENET2_RD3 | Y6 | NVCC_WAKEUP | GPIO | Alt5 | GPIO4.IO[27] | Input with PD |  |
| ENET_RX_CTL | Y4 | NVCC_WAKEUP | GPIO | Alt5 | GPIO4.IO[22] | Input with PD |  |
| ENET_RXC | AA3 | NVCC_WAKEUP | GPIO | Alt5 | GPIO4.IO[23] | Input with PD |  |
| ENET2_TD0 | T8 | NVCC_WAKEUP | GPIO | Alt5 | GPIO4.IO[19] | Input with PD |  |
| ENET_TD0 | T8 | NVCC_WAKEUP | GPIO | Alt5 | GPIO4.IO[19] | Input with PD |  |
| ENET2_TD1 | U8 | NVCC_WAKEUP | GPIO | Alt5 | GPIO4.IO[18] | Input with PD |  |
| ENET2_TD2 | V8 | NVCC_WAKEUP | GPIO | Alt5 | GPIO4.IO[17] | Input with PD |  |
| ENET2_TD3 | T10 | NVCC_WAKEUP | GPIO | Alt5 | GPIO4.IO[16] | Input with PD |  |
| ENET_TX_CTL | V6 | NVCC_WAKEUP | GPIO | Alt5 | GPIO4.IO[20] | Input with PD |  |
| ENET2_TXC | U6 | NVCC_WAKEUP | GPIO | Alt5 | GPIO4.IO[21] | Input with PD |  |
| GPIO_IO00 | J21 | NVCC_GPIO | GPIO | Alt0 | GPIO2.IO[0] | Input with PD |  |
| GPIO_IO01 | J20 | NVCC_GPIO | GPIO | Alt0 | GPIO2.IO[1] | Input with PD |  |
| GPIO_IO02 | K20 | NVCC_GPIO | GPIO | Alt0 | GPIO2.IO[2] | Input with PD |  |
| GPIO_IO03 | K21 | NVCC_GPIO | GPIO | Alt0 | GPIO2.IO[3] | Input with PD |  |
| GPIO_IO04 | L17 | NVCC_GPIO | GPIO | Alt0 | GPIO2.IO[4] | Input with PD |  |
| GPIO_IO05 | L18 | NVCC_GPIO | GPIO | Alt0 | GPIO2.IO[5] | Input with PD |  |
| GPIO_IO06 | L20 | NVCC_GPIO | GPIO | Alt0 | GPIO2.IO[6] | Input with PD |  |
| GPIO_IO06 | L20 | NVCC_GPIO | GPIO | Alt0 | GPIO2.IO[6] | Input with PD |  |
| GPIO_IO07 | L21 | NVCC_GPIO | GPIO | Alt0 | GPIO2.IO[7] | Input with PD |  |
| GPIO_IO08 | M20 | NVCC_GPIO | GPIO | Alt0 | GPIO2.IO[8] | Input with PD |  |
| GPIO_IO09 | M21 | NVCC_GPIO | GPIO | Alt0 | GPIO2.IO[9] | Input with PD |  |

#### Additional Information

This page contains a continuation of Table 101, which details the functional contact assignment for the 14 x 14 mm ball grid array (BGA) package of the i.MX 93 Applications Processor. The table lists ball names, their corresponding 14 x 14 ball identifiers, power groups, ball types, default settings (modes and functions), and the status while reset is asserted.

#### Notes

- The table continues on the next page.

### 14 x 14 mm functional contact assignment

| Ball name | 14 x 14 | Power group | Ball Types | Default setting | Default setting | Status while reset is asserted |
| --- | --- | --- | --- | --- | --- | --- |
| GPIO_IO10 | N17 | NVCC_GPIO | GPIO | Alt0 | GPIO2.IO[10] | Input with PD |
| GPIO_IO11 | N18 | NVCC_GPIO | GPIO | Alt0 | GPIO2.IO[11] | Input with PD |
| GPIO_IO12 | N20 | NVCC_GPIO | GPIO | Alt0 | GPIO2.IO[12] | Input with PD |
| GPIO_IO12 | N20 | NVCC_GPIO | GPIO | Alt0 | GPIO2.IO[12] | Input with PD |
| GPIO_IO13 | N21 | NVCC_GPIO | GPIO | Alt0 | GPIO2.IO[13] | Input with PD |
| GPIO_IO14 | P20 | NVCC_GPIO | GPIO | Alt0 | GPIO2.IO[14] | Input with PD |
| GPIO_IO15 | P21 | NVCC_GPIO | GPIO | Alt0 | GPIO2.IO[15] | Input with PD |
| GPIO_IO16 | R21 | NVCC_GPIO | GPIO | Alt0 | GPIO2.IO[16] | Input with PD |
| GPIO_IO17 | R20 | NVCC_GPIO | GPIO | Alt0 | GPIO2.IO[17] | Input with PD |
| GPIO_IO17 | R2 | NVCC_GPIO | GPIO | Alt0 | GPIO2.IO[17] | Input with PD |
| GPIO_IO18 | R18 | NVCC_GPIO | GPIO | Alt0 | GPIO2.IO[18] | Input with PD |
| GPIO_IO19 | R17 | NVCC_GPIO | GPIO | Alt0 | GPIO2.IO[19] | Input with PD |
| GPIO_IO20 | T20 | NVCC_GPIO | GPIO | Alt0 | GPIO2.IO[20] | Input with PD |
| GPIO_IO21 | T21 | NVCC_GPIO | GPIO | Alt0 | GPIO2.IO[21] | Input with PD |
| GPIO_IO22 | U18 | NVCC_GPIO | GPIO | Alt0 | GPIO2.IO[22] | Input with PD |
| GPIO_IO23 | U20 | NVCC_GPIO | GPIO | Alt0 | GPIO2.IO[23] | Input with PD |
| GPIO_IO23 | U20 | NVCC_GPIO | GPIO | Alt0 | GPIO2.IO[23] | Input with PD |
| GPIO_IO24 | U21 | NVCC_GPIO | GPIO | Alt0 | GPIO2.IO[24] | Input with PD |
| GPIO_IO25 | V21 | NVCC_GPIO | GPIO | Alt0 | GPIO2.IO[25] | Input with PD |
| GPIO_IO26 | V20 | NVCC_GPIO | GPIO | Alt0 | GPIO2.IO[26] | Input with PD |
| GPIO_IO27 | W21 | NVCC_GPIO | GPIO | Alt0 | GPIO2.IO[27] | Input with PD |
| GPIO_IO28 | W20 | NVCC_GPIO | GPIO | Alt0 | GPIO2.IO[28] | Input with PD |
| GPIO_IO29 | Y21 | NVCC_GPIO | GPIO | Alt0 | GPIO2.IO[29] | Input with PD |
| GPIO_IO29 | Y21 | NVCC_GPIO | GPIO | Alt0 | GPIO2.IO[29] | Input with PD |
| I2C1_SCL | C20 | NVCC_AON | GPIO | Alt5 | GPIO1.IO[0] | Input with PD |
| I2C1_SDA | C21 | NVCC_AON | GPIO | Alt5 | GPIO1.IO[1] | Input with PD |
| I2C2_SCL | D20 | NVCC_AON | GPIO | Alt5 | GPIO1.IO[2] | Input with PD |

#### Additional Information

This page contains a continuation of Table 101, which details the functional contact assignment for the 14 x 14 mm ball grid array (BGA) package of the i.MX 93 Applications Processor. The table lists ball names, their corresponding 14 x 14 ball positions, power groups, ball types, default modes, default functions, and the status while reset is asserted.

#### Notes

- The table continues on the next page.

### 14 x 14 mm functional contact assignment ...continued

| Ball name | 14 x 14 | Power group | Ball Types | Default setting | Default setting | Default setting | Status while reset is asserted |
| --- | --- | --- | --- | --- | --- | --- | --- |
| I2C2_SDA | D21 | NVCC_AON | GPIO | Alt5 | GPIO1.IO[3] | Input with PD |  |
| LVDS_D0_P | B5 | VDD_LVDS_1P8 | PHY | — | GPIO1[3] | — | — |
| LVDS_D0_N | A5 | VDD_LVDS_1P8 | PHY | — | — | — | — |
| LVDS_D1_P | B4 | VDD_LVDS_1P8 | PHY | — | — | — | — |
| LVDS_D1_N | A4 | VDD_LVDS_1P8 | PHY | — | — | — | — |
| LVDS_D2_P | B2 | VDD_LVDS_1P8 | PHY | — | — | — | — |
| LVDS_D2_N | A2 | VDD_LVDS_1P8 | PHY | — | — | — | — |
| LVDS_D3_P | C1 | VDD_LVDS_1P8 | PHY | — | — | — | — |
| LVDS_D3_N | B1 | VDD_LVDS_1P8 | PHY | — | — | — | — |
| LVDS_CLK_P | B3 | VDD_LVDS_1P8 | PHY | — | — | — | — |
| LVDS_CLK_N | A3 | VDD_LVDS_1P8 | PHY | — | — | — | — |
| MIPI_CSI1_CLK_N | D10 | MIPI_CSI1_VPH | PHY | — | — | — | — |
| MIPI_CSI1_CLK_P | E10 | MIPI_CSI1_VPH | PHY | — | — | — | — |
| MIPI_CSI1_D0_N | A11 | MIPI_CSI1_VPH | PHY | — | — | — | — |
| MIPI_CSI1_D0_P | B11 | MIPI_CSI1_VPH | PHY | — | — | — | — |
| MIPI_CSI1_D1_N | A10 | MIPI_CSI1_VPH | PHY | — | — | — | — |
| MIPI_CSI1_D1_P | B10 | MIPI_CSI1_VPH | PHY | — | — | — | — |
| MIPI_DSI1_CLK_N | D6 | MIPI_DSI1_VPH | PHY | — | — | — | — |
| MIPI_DSI1_CLK_P | E6 | MIPI_DSI1_VPH | PHY | — | — | — | — |
| MIPI_DSI1_CLK | A6 | MIPI_DSI1_VPH | PHY | — | — | — | — |
| MIPI_DSI1_D0_N | A6 | MIPI_DSI1_VPH | PHY | — | — | — | — |
| MIPI_DSI1_D0_P | B6 | MIPI_DSI1_VPH | PHY | — | — | — | — |
| MIPI_DSI1_D1_N | A7 | MIPI_DSI1_VPH | PHY | — | — | — | — |
| MIPI_DSI1_D1_P | B7 | MIPI_DSI1_VPH | PHY | — | — | — | — |

#### Additional Information

This page contains a continuation of Table 101, which details the functional contact assignment for the 14 x 14 mm ball grid array (BGA) package of the i.MX 93 Applications Processor. The table lists ball names, their corresponding 14 x 14 ball positions, power groups, ball types, default settings, and status while reset is asserted.

#### Notes

- The table continues on the next page.

### 14 x 14 mm functional contact assignment ...continued

| Ball name | 14 x 14 | Power group | Ball Types | Default setting | Default setting | Status while reset is asserted |
| --- | --- | --- | --- | --- | --- | --- |
| MIPI_DSI1_D2_N | A8 | MIPI_DSI1_VPH | PHY | — | — | — |
| MIPI_DSI1_D2_P | B8 | MIPI_DSI1_VPH | PHY | — | — | — |
| MIPI_DSI1_D | A9 | MIPI_DSI1_VPH | PHY | — | — | — |
| MIPI_DSI1_D3_N | A9 | MIPI_DSI1_VPH | PHY | — | — | — |
| MIPI_DSI1_D3_P | B9 | MIPI_DSI1_VPH | PHY | — | — | — |
| MIPI_REXT | D8 | MIPI_DSI1_VPH | PHY | — | BBSMMIX.ONOFF | — |
| ONOFF | A19 | NVCC_BSM1_P8 | GPIO | Alt0 | BBSMMIX.ONOFF | Input without PU / PD |
| PDM_DSI_STREAM | J | NVCC_BSM8 | GPIO | Alt5 | GPIO1.IO[9] | Input with PD |
| PDM_BIT_STREAM0 | J17 | NVCC_AON | GPIO | Alt5 | GPIO1.IO[9] | Input with PD |
| PDM_BIT_STREAM1 | G18 | NVCC_AON | GPIO | Alt5 | GPIO1.IO[10] | Input with PD |
| PDM_CLK | G17 | NVCC_AON | GPIO | Alt5 | GPIO1.IO[8] | Input with PD |
| PMIC_ON_REQ | A17 | NVCC_BSM1_P8 | GPIO | Alt0 | BBSMMIX.PMIC_ON_REQ | Output high without PU / PD |
| PMIC_STBY_REQ | B18 | NVCC_BSM1_P8 | GPIO | Alt0 | BBSMMIX.PMIC_STBY_REQ | Output low without PU / PD |
| POR_B | A16 | NVCC_BSM1_P8 | GPIO | Alt0 | BBSMMIX.POR_B | Input without PU / PD |
| RTC_XTALI | E16 | NVCC_BSM1_P8 | ANALOG | Alt0 | BBSMMIX.RTC | — |
| RTC_XTALO | D16 | NVCC_BSM1_P8 | ANALOG | — | — | — |
| SAI1_RXD0 | H20 | NVCC_AON | GPIO | Alt5 | GPIO1.IO[14] | Input with PD |
| SAI1_TXC | G20 | NVCC_AON | GPIO | Alt5 | GPIO1.IO[12] | Input with PD |
| SAI1_TXC | G20 | NVCC_AON | GPIO | Alt5 | GPIO1.IO[13] | Input with PD |
| SAI1_TXD0 | H21 | NVCC_AON | GPIO | Alt5 | CCMSRCGPCMIX.BOOT_MODE[3] | Input with PD |
| SAI1_TXFS | G21 | NVCC_AON | GPIO | Alt5 | CCMSRCGPCMIX.BOOT_MODE[2] | Input with PD |

#### Additional Information

This page contains a continuation of Table 101, which details the functional contact assignment for a 14 x 14 mm ball grid array (BGA) package. The table lists ball names, their corresponding 14 x 14 ball positions, power groups, ball types, default settings, default functions, and their status while the reset is asserted.

#### Notes

- The table continues on the next page.

### 14 x 14 mm functional contact assignment

| Ball name | 14 x 14 ball | Power group | Ball Types | Default setting | Default setting | Default setting | Status while reset is asserted |
| --- | --- | --- | --- | --- | --- | --- | --- |
| SD1_CLK | Y11 | NVCC_WAKEUP | GPIO | Alt5 | GPIO3.IO[8] | Input with PD |  |
| SD1_CMD | AA12 | NVCC_WAKEUP | GPIO | Alt5 | GPIO3.IO[9] | Input with PD |  |
| SD1_DATA0 | AA14 | NVCC_WAKEUP | GPIO | Alt5 | GPIO3.IO[10] | Input with PD |  |
| SD1_DATA0 | AA14 | NVCC_WAKEUP | GPIO | Alt5 | GPIO3.IO[10] | Input with PD |  |
| SD1_DATA1 | AA15 | NVCC_WAKEUP | GPIO | Alt5 | GPIO3.IO[11] | Input with PD |  |
| SD1_DATA2 | AA16 | NVCC_WAKEUP | GPIO | Alt5 | GPIO3.IO[12] | Input with PD |  |
| SD1_DATA3 | AA13 | NVCC_WAKEUP | GPIO | Alt5 | GPIO3.IO[13] | Input with PD |  |
| SD1_DATA4 | Y13 | NVCC_WAKEUP | GPIO | Alt5 | GPIO3.IO[14] | Input with PD |  |
| SD1_DATA5 | Y14 | NVCC_WAKEUP | GPIO | Alt5 | GPIO3.IO[15] | Input with PD |  |
| SD1_DATA5 | Y14 | NVCC_WAKEUP | GPIO | Alt5 | GPIO3.IO[15] | Input with PD |  |
| SD1_DATA6 | Y15 | NVCC_WAKEUP | GPIO | Alt5 | GPIO3.IO[16] | Input with PD |  |
| SD1_DATA7 | Y16 | NVCC_WAKEUP | GPIO | Alt5 | GPIO3.IO[17] | Input with PD |  |
| SD1_STROBE | Y12 | NVCC_WAKEUP | GPIO | Alt5 | GPIO3.IO[18] | Input without PU / PD |  |
| SD_CD_B | Y1 | NVCC_SD2 | GPIO | Alt5 | GPIO3.IO[0] | Input with PD |  |
| SD2_CD_B | Y17 | NVCC_SD2 | GPIO | Alt5 | GPIO3.IO[0] | Input with PD |  |
| SD2_CLK | AA19 | NVCC_SD2 | GPIO | Alt5 | GPIO3.IO[1] | Input with PD |  |
| SD2_CMD | Y19 | NVCC_SD2 | GPIO | Alt5 | GPIO3.IO[2] | Input with PD |  |
| SD2_DATA0 | Y18 | NVCC_SD2 | GPIO | Alt5 | GPIO3.IO[3] | Input with PD |  |
| SD2_DATA1 | AA18 | NVCC_SD2 | GPIO | Alt5 | GPIO3.IO[4] | Input with PD |  |
| SD2_DATA1 | AA18 | NVCC_SD2 | GPIO | Alt5 | GPIO3.IO[4] | Input with PD |  |
| SD2_DATA2 | Y20 | NVCC_SD2 | GPIO | Alt5 | GPIO3.IO[5] | Input with PD |  |
| SD2_DATA3 | AA20 | NVCC_SD2 | GPIO | Alt5 | GPIO3.IO[6] | Input with PD |  |
| SD2_RESET_B | AA17 | NVCC_SD2 | GPIO | Alt5 | GPIO3.IO[7] | Input with PD |  |
| SD2_VSELECT | V18 | NVCC_WAKEUP | GPIO | Alt5 | GPIO3.IO[19] | Input with PD |  |
| SD3_CLK | V16 | NVCC_WAKEUP | GPIO | Alt5 | GPIO3.IO[20] | Input with PD |  |
| SD3_CMD | U16 | NVCC_WAKEUP | GPIO | Alt5 | GPIO3.IO[21] | Input with PD |  |
| SD3_CMD | U16 | NVCC_WAKEUP | GPIO | Alt5 | GPIO3.IO[21] | Input with PD |  |
| SD3_DATA0 | T16 | NVCC_WAKEUP | GPIO | Alt5 | GPIO3.IO[22] | Input with PD |  |

#### Additional Information

This page contains a continuation of Table 101, which details the functional contact assignment for the 14 x 14 mm ball grid array (BGA) package of the i.MX 93 Applications Processor. The table lists ball names, their corresponding 14 x 14 ball positions, power groups, ball types, default settings, default functions, and their status while reset is asserted.

#### Notes

- The table continues on the next page.

### 14 x 14 mm functional contact assignment

| Ball name | 14 x 14 ball | Power group | Ball Types | Default setting | Default setting | Default setting | Status while reset is asserted |
| --- | --- | --- | --- | --- | --- | --- | --- |
| SD3_DATA1 | V14 | NVCC_WAKEUP | GPIO | Alt5 | GPIO3.IO[23] | Input with PD |  |
| SD3_DATA2 | U14 | NVCC_WAKEUP | GPIO | Alt5 | GPIO3.IO[24] | Input with PD |  |
| SD3_DATA2 | T14 | NVCC_WAKEUP | GPIO | Alt5 | GPIO3.IO[25] | Input with PD |  |
| SD3_DATA3 | T14 | NVCC_BBSM_1P8 | GPIO | Alt5 | GPIO3.IO[25] | Input with PD |  |
| TAMPER0 | B16 | NVCC_BBSM_1P8 | GPIO | Alt0 | BBSMMIX.TAMPER0 | Input with PD |  |
| TAMPER1 | F14 | NVCC_BBSM_1P8 | GPIO | Alt0 | BBSMMIX.TAMPER1 | Input with PD |  |
| UART1_RXD | E20 | NVCC_AON | GPIO | Alt5 | GPIO1.IO[4] | Input with PD |  |
| UART1_TXD | E21 | NVCC_AON | GPIO | Alt5 | GPIO1.IO[5] | Input with PD |  |
| UART2_RXD | F20 | NVCC_AON | GPIO | Alt5 | GPIO1.IO[6] | Input with PD |  |
| UART2_TXD | F21 | NVCC_AON | GPIO | Alt5 | GPIO1.IO[7] | Input with PD |  |
| USB_D_N | A14 | VDD_USB_3P3 | PHY | - | - | - |  |
| USB1_D_N | A14 | VDD_USB_3P3 | PHY | - | - | - |  |
| USB1_D_P | B14 | VDD_USB_3P3 | PHY | - | - | - |  |
| USB1_ID | C11 | VDD_USB_1P8 | PHY | - | - | - |  |
| USB1_TXRTUNE | D12 | VDD_USB_1P8 | PHY | - | - | - |  |
| USB1_VBUS | F12 | VDD_USB_3P3 | PHY | - | - | - |  |
| USB1_VBUS | F12 | VDD_USB_3P3 | PHY | - | - | - |  |
| USB2_D_N | A15 | VDD_USB_3P3 | PHY | - | - | - |  |
| USB2_D_P | B15 | VDD_USB_3P3 | PHY | - | - | - |  |
| USB2_ID | E12 | VDD_USB_1P8 | PHY | - | - | - |  |
| USB2_TXRTUNE | D14 | VDD_USB_1P8 | PHY | - | - | - |  |
| USB2_VBUS | E14 | VDD_USB_3P3 | PHY | - | - | - |  |
| WDOG_ANY | J18 | NVCC_AON | GPIO | Alt0 | WDOG1.WDOG_ANY | Input with PU |  |
| WDOG_ANY | J18 | NVCC_AON_1P8 | GPIO | Alt0 | WDOG1.WDOG_ANY | Input with PU |  |
| XTALI_24M | D18 | VDD_ANA_ANA_1P8 | ANALOG | - | - | - |  |

#### Additional Information

This page contains a continuation of Table 101, which details the functional contact assignment for a 14 x 14 mm ball grid array (BGA) package. The table lists ball names, their corresponding 14 x 14 ball positions, power groups, ball types, default settings, default functions, and their status while reset is asserted.

#### Notes

- The table continues on the next page.
- All information provided in this document is subject to legal disclaimers.


## 6.1.3 14 x 14 mm, 0.65 mm pitch, ball map

*(Page 95)*

### Table 101. 14 x 14 mm functional contact assignment ...continued

| Ball name | 14 x 14 ball | Power group | Ball Types | Default setting | Default setting | Default setting |
| --- | --- | --- | --- | --- | --- | --- |
| XTALO_24M | E1 | VDD_ANA_8 | ANALOG | — | — | — |
| XTALO_24M | E18 | VDD_ANA_1P8 | ANALOG | — | — | — |

#### Additional Information

The page discusses the ball map for the i.MX 93 Applications Processor with a 14 x 14 mm, 0.65 mm pitch ball grid array (BGA). Table 101 provides a continuation of the functional contact assignment for the balls on the BGA. The ball map is described in Table 102, which is referenced but not shown on this page. Pin configurations include ball names, their corresponding ball numbers, power groups, and ball types. Default settings for modes, functions, and reset status are indicated as '—' for the balls shown.

#### Notes

- The table is marked as '...continued', indicating it spans multiple pages.
- The ball map for the i.MX 93 is referenced in Table 102, which is not shown on this page.
- The ball types listed are 'ANALOG', indicating the functional nature of the pins.


## 14 x 14 mm, 0.65 mm pitch, ball map

*(Page 96)*

### 14 x 14 mm, 0.65 mm pitch, ball map

|  | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 | 12 | 13 | 14 | 15 | 16 | 17 | 18 | 19 | 20 | 21 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| A | VSS | LVD_S_D2_N | LVD_S_CL_K_N | LVD_S_D1_N | LVD_S_D0_N | MIPI_DSI_1_D0_N | MIPI_DSI_1_D1_N | MIPI_DSI_1_D2_N | MIPI_DSI_1_D3_N | MIPI_CSI_1_D1_N | MIPI_CSI_1_D0_N | NC_A12 | NC_A13 | USB_1_D_N | USB_2_D_N | USB_BOR_B | POR | PMIC_ON_REQ | CLKI_N2 | ONO_FF | ADC_IN1 | VSS |
| B | LVD_S_D3_P | LVD_S_D2_P | LVD_S_CL_K_P | LVD_S_D1_P | LVD_S_D0_P | MIPI_DSI_1_D0_P | MIPI_DSI_1_D1_P | MIPI_DSI_1_D2_P | MIPI_DSI_1_D3_P | MIPI_CSI_1_D1_P | MIPI_CSI_1_D0_P | NC_B12 | NC_B13 | USB_1_D_P | USB_2_D_P | TAMPER0 | CLKI_N1 | PMIC_STB_EQ | ADC_IN0 | ADC_IN2 | ADC_IN3 | B |
| C | LVD_S_D3_P | VSS | VSS | VSS | VSS | VSS | VSS | VSS | VSS | VSS | VSS | USB_1_ID | VSS | VSS | VSS | VSS | VSS | VSS | I2C1_SCL | I2C1_SD | C |
| D | DRA_A5_A | DRA_ESE_TES_T1 | DRA_M_C_M_R_M_M | MIPI_DSI_1_CL_L | MIPI_DSI_1_CL_R | MIPI_CSI_1_TX | USB_2_TX | USB_2_ID | RXC_XTA_LO | XTAL_I_24_M | I2C2_SCL | I2C2_SD_A | D |  |  |  |  |  |  |  |  |
| E | DRA_M_C_A3_A_A4_A | DRA_M_C_M_Z | DRA_T1 | MIPI_DSI_1_CL_B_1P_8 | MIPI_CSI_1_TX | USB_1_VB | USB_2_VB | RTC_XTA_LI | XTAL_O_24_M | VSS | UAR_T1_R | UAR_T1_T | E |  |  |  |  |  |  |  |  |
| F | DRA_M_C_M_C | DRA_M_C_M_Q | DRA_S0_A_A2_A | VDD_LVD_S_1P_I_1P_B_0P_1 | VDD_MIP | VDD_US_1_VB | USB_2_ID | TAMPER1 | VDD_AN_A0_1 | UAR_T2_R | UAR_T2_T | F |  |  |  |  |  |  |  |  |  |
| G | DRA_M_C_M_C | DRA_M_C_M_C | DRA_M_C_M_C | VDD_Q_D | VDD_MIP | VDD_US_C_B | VDD_BBS | VDD_AN_CLK_BIT_STR | PDM_TXC | PDM_TXF | G |  |  |  |  |  |  |  |  |  |  |

#### Additional Information

This page contains a ball map for the i.MX 93 Applications Processor in a 14 x 14 mm package with a 0.65 mm pitch. The table lists the pin names, functions, and ball numbers for each pin in the package. The table continues on the next page.

#### Notes

- The table continues on the next page.

### 14 x 14 mm, 0.65 mm pitch, ball map

|  | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 | 12 | 13 | 14 | 15 | 16 | 17 | 18 | 19 | 20 | 21 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| H | DRA M_C KE0_ A Q06_ A | DRA M_C A0_A Q07_ A |  |  |  |  | VSS |  | VSS |  | VSS |  | VSS |  | VSS |  |  |  | SAI1_RX D0 | SAI1_TXD 0 | H |
| J | DRA M_D Q04_ A | DRA M_D Q05_ A | VSS | DRA M_C KE1_ A | VSS | VDD Q_D DR | VDD Q_D DR | VSS | VDD SO_C | VDD SO_C | VDD SO_C | VDD SO_C | VDD SO_C | VDD SO_C | VDD AN_8 | VDD AN_BIT_8 | PDM_G_A | WDO_NY | VSS | GPIO_I00_1 | GPIO_I00_0 | J |
| K | DRA M_D Q02_ A | DRA M_D Q03_ A |  |  | VSS | VDD Q_D DR | VDD Q_D DR | VSS | VDD SO_C | VDD SO_C | VDD SO_C | VDD SO_C | VDD SO_C | VDD SO_C | VDD AN_AVD_0 | NVC_C_ON | GPIO_I00_2 | GPIO_I00_3 | K |  |  |
| L | VSS | DRA M_D Q00_ A | VSS | DRA M_D Q01_ A | VSS | VDD 2_DD R | VDD 2_DD R | VSS |  |  |  |  |  |  |  |  | GPIO_I00_4 | GPIO_I00_5 | GPIO_I00_6 | GPIO_I00_7 | L |
| M | DRA M_D Q02_ A | DRA M_D Q03_ A |  |  |  |  |  |  | VDD SO_C | VDD SO_C | VDD SO_C | VDD SO_C | VDD SO_C | VDD SO_C | VDD AN_STR_P8 |  |  |  |  | GPIO_I00_8 | GPIO_I00_9 | M |
| N | DRA M_D Q00_ A | DRA M_D Q01_ A | VSS | DRA M_D Q02_ A | VSS | VDD 2_DD R | VDD 2_DD R | VSS | VDD SO_C | VDD SO_C | VDD SO_C | VDD SO_C | VDD SO_C | VDD SO_C | NVC_C_G | NVC_C_G | GPIO_I01_0 | GPIO_I01_1 | GPIO_I01_2 | GPIO_I01_3 | N |

#### Additional Information

This page contains a ball map for the i.MX 93 Applications Processor in a 14 x 14 mm package with a 0.65 mm pitch. The table lists the ball numbers (1-21) and their corresponding pin functions or signals. The ball map is part of a larger table that continues on the next page.

#### Notes

- The table continues on the next page.
- The ball map provides the pin configuration for the i.MX 93 Applications Processor in a specific package type.

### 14 x 14 mm, 0.65 mm pitch, ball map

| 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 | 12 | 13 | 14 | 15 | 16 | 17 | 18 | 19 | 20 | 21 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| DRA M_D Q15_A | DRA M_D Q14_A |  |  |  |  | VSS | VSS | VSS | VSS |  |  |  |  |  |  |  | GPIO _IO1_4 | GPIO _IO1_5 |  |  |
| DRA M_D Q12_A | DRA M_D Q13_A | VSS | DRA M_D QS1_R | DRA M_D QS1_R | VDD | VDD | NVC C_W_C_W | NVC C_W_C_W | VDD | NVC C_S_D2 | GPIO _IO1_9 | GPIO _IO1_8 | VSS | GPIO _IO1_7 | GPIO _IO1_6 |  |  |  |  |  |
| VSS | DRA M_D T_A_A |  | VDD 2_DD_R | ENE T2_T | ENE T2_T | ENE T2_T | SD3_DAT_D1 | SD3_DAT_D1 |  |  | GPIO _IO2_0 | GPIO _IO2_1 |  |  |  |  |  |  |  |  |
| DRA M_D Q11_A | DRA M_D Q10_A | CCM _CLK_O3 | ENE T2_T_XC | ENE T2_T_XC | ENE T1_T_XC | ENE T1_T_XC | SD3_DAT_A2 | SD3_CMD |  | GPIO _IO2_2 | GPIO _IO2_3 | GPIO _IO2_4 |  |  |  |  |  |  |  |  |
| DRA M_D Q08_A | DRA M_D Q09_A | CCM _CLK_O4 | ENE T2_T_CT | ENE T2_T_CT | ENE T1_T_CT | ENE T1_T_CT | SD3_CLK | SD2_VSE_LEC_T |  | GPIO _IO2_6 | GPIO _IO2_5 | GPIO _IO2_8 | GPIO _IO2_7 |  |  |  |  |  |  |  |
| DAP _TDI_TM | VSS | VSS | NVC C_W_AKE_UP | VSS | ENE T1_T_D0 | VSS | VSS | VSS | VSS | VSS | VSS | GPIO _IO2_9 |  |  |  |  |  |  |  |  |
| DAP _TCL_TD | CCM _CLK_O2 | ENE T2_R_D1 | ENE T2_R_D3 | ENE T2_M_DC | ENE T1_R_D1 | ENE T1_R_D3 | SD1_STR_OBE | SD1_DAT_A4 | SD1_DAT_A5 | SD1_DAT_A6 | SD1_DAT_A7 | SD1_CD_B | SD2_DAT_A0 | SD2_CMD | SD2_DAT_A2 | SD2_DAT_IO2_9 |  |  |  |  |

#### Additional Information

This page contains a ball map for the i.MX 93 Applications Processor in a 14 x 14 mm package with a 0.65 mm pitch. The table lists the ball positions and their corresponding functions or connections.

#### Notes

- The table continues on the next page.
- All information is subject to legal disclaimers.
- The document is for the IMX93AEC variant of the i.MX 93 Applications Processor.

### 14 x 14 mm, 0.65 mm pitch, ball map

|  | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 | 12 | 13 | 14 | 15 | 16 | 17 | 18 | 19 | 20 | 21 |  |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
|  | WCLK | RAC | X_CT |  | X_CT |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |
|  | K | ESW | L |  | L |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |
|  | VSS | CCM | ENE | ENE | ENE | ENE | ENE | ENE | ENE | ENE | ENE | SD1_ | SD1_ | SD1_ | SD1_ | SD2_ | SD2_ | SD2_ | SD2_ | VSS | AA |  |
| AA | _CLK | T2_R | T2_R | T2_M | T1_R | T1_R | T1_R | T1_M | T1_M | CMD | DAT | DAT | DAT | RES | DAT | CLK | DAT | VSS | AA |  |  |  |
|  | O1 | XC | D0 | D2 | DIO | XC | D0 | D2 | DIO | DC | A3 | A0 | A1 | A2 | ET_B | A1 | A3 |  |  |  |  |  |
|  | 1 | 2 | 3 | 5 | 6 | 7 | 8 | 9 | 10 | 11 | 12 | 13 | 14 | 15 | 16 | 17 | 18 | 19 | 20 | 21 |  |  |

#### Additional Information

This page contains a ball map for the i.MX 93 Applications Processor in a 14 x 14 mm package with a 0.65 mm pitch. The ball map lists the pin names and their corresponding functions for each ball number. The ball map is part of the data sheet for the IMX93AEC product.


## 6.2 DDR pin pin function list

*(Page 100)*

### DDR pin function list

| Ball name | LPDDR4/LPDDR4x |
| --- | --- |
| DRAM_DQS_T_A | DQSA_T[0] |
| DRAM_DQ0_A | DQSA[0] |
| DRAM_DQ0_C_A | DQSA_C[0] |
| DRAM_DMI0_A | DM/DBIA[0] |
| DRAM_DQ0_A | DQA[0] |
| DRAM_DQ01_A | DQA[1] |
| DRAM_DQ02_A | DQA[2] |
| DRAM_DQ03_A | DQA[3] |
| DRAM_DQ04_A | DQA[4] |
| DRAM_DQ05_A | DQA[5] |
| DRAM_DQ06_A | DQA[6] |
| DRAM_DQ07_A | DQA[7] |
| DRAM_DQS1_T_A | DQSA_T[1] |
| DRAM_DQS1_C_A | DQSA_C[1] |
| DRAM_DQS1_C_A | DQSA_C[1] |
| DRAM_DMI1_A | DM/DBIA[1] |
| DRAM_DQ08_A | DQA[8] |
| DRAM_DQ09_A | DQA[9] |
| DRAM_DQ10_A | DQA[10] |
| DRAM_DQ11_A | DQA[11] |
| DRAM_DQ11_A | DQA[11] |
| DRAM_DQ12_A | DQA[12] |
| DRAM_DQ13_A | DQA[13] |
| DRAM_DQ14_A | DQA[14] |
| DRAM_DQ15_A | DQA[15] |
| DRAM_RESET_N | RESET_N |

#### Additional Information

This page contains a list of DDR pin functions for the i.MX 93 Applications Processor, specifically for the LPDDR4/LPDDR4x interface. The table maps the ball names to their corresponding functions in the LPDDR4/LPDDR4x interface.

#### Notes

- The table continues on the next page.


## 7 Revision history

*(Page 101)*

### Table 103. DDR pin function list ...continued

| DDR Pin Name | Function |
| --- | --- |
| DRAM_MTRST1 | — |
| DRAM_CKE0_A | CKEA[0] |
| DRAM_CKE1_A | CKEA[1] |
| DRAM_CS0_A | CSA[0] |
| DRAM_CS1_A | CSA[1] |
| DRAM_CK_T_A | CLKA_T |
| DRAM_CK_A | CLKA_T |
| DRAM_CK_C_A | CLKA_C |
| DRAM_CA0_C | CAA[0] |
| DRAM_CA1_C | CAA[1] |
| DRAM_CA2_C | CAA[2] |
| DRAM_CA_C | CAA[3] |
| DRAM_CA3_C | CAA[3] |
| DRAM_CA4_C | CAA[4] |
| DRAM_CA5_C | CAA[5] |
| DRAM_ZQ1 | — |

### Table 104. i.MX 93 Data Sheet document revision history

| Rev. Number | Date | Substantive Change(s) |
| --- | --- | --- |
| IMX93AEC v6.1 | 07 July 2025 | ['Removed the I/O pin information from Ordering information', 'Updated the Package type in Figure 1', 'Removed Section 9 x 9 mm package information'] |
| IMX93AEC v6.0 | 04 June 2025 | ['Updated Ordering information', 'Updated Figure 1', 'Updated the descriptions about DRAM and LPUART, and ADC in Table 1; Removed FlexIO from Table 1', 'Added LPDDR4 in Figure 2; updated ADC and LPUART in Figure 2'] |

#### Additional Information

The page contains a continuation of the DDR pin function list (Table 103) and a revision history table (Table 104) for the i.MX 93 Applications Processor datasheet. The DDR pin function list includes pin names and their corresponding functions related to DRAM control signals. The revision history table provides details on changes made to the datasheet across different revisions, including removal of I/O pin information, updates to figures, and modifications to tables.

#### Notes

- {'note_number': '1', 'description': 'DRAM_ZQ can be connected with a 120 Ω ±1% resistor to GND.'}


## i.MX 93 Data Sheet document revision history...continued

*(Page 102)*

### Table 104. i.MX 93 Data Sheet document revision history...continued

| Rev. Number | Date | Substantive Change(s) |
| --- | --- | --- |
|  |  | ['Updated the description about ONOFF and XTALI_24/XTALO_24M in Table 3', 'Added Table 11 about ONOFF and XTALI_24/XTALO_24M in Table 3', 'Updated Table 15', 'Updated Table 27', 'Updated the description of DDR SDRAM-specific parameters (LPDDR4/LPDDR4X) and Clock/data/command/address pin allocations', 'Added a footnote in Table 43', 'Updated the unit of C_sample, C_compare, and C_conversion to cycle in Table 51; removed the maximum value of C_sample, C_compare, and second ADC conversion clock in Table 51; added ENOB values in Table 51', 'Removed Table. ADC compare and second ADC conversion clock in Table 51; added ENOB values in Table 51', 'Removed Table. ADC electrical specifications (VREFH = VDD_ANAx_p8 and VADIN_max ≤ VREFH)', 'Added a footnote in Table 68', 'Updated Table 71', 'Updated Table 72', 'Removed BCAN, BCANXL, and minimum operating frequency from Table 72', 'Added the maximum frequency footnotes in FlexSPI timing parameters', 'Updated LPB descriptions in Boot mode configuration'] |
| Rev. 5 | 01/2025 | ['Added new part number in Table 2', 'Updated the definition of special fuse in Figure 1', 'Removed the definition Module list', 'Updated the Module descriptions', 'Updated the descriptions of External clock sources', 'Updated the NVCC_BBSM_1P8 in Figure 4', 'Updated the footnote of Table 29', 'Updated the values of V_IDTH, V_IDL, V_IHS, and V_HHS in Table 36', 'Updated Figure 50', 'Updated the default function of SAI1.RXD0 in Table 101'] |
| Rev. 4 | 08/2024 | ['Updated Table 2', 'Updated Figure 1', 'Updated Clock sources', 'Added a footnote in Figure 4', 'Updated a Table 24 and Table 25', 'Updated Table 26 and Table 28'] |

#### Additional Information

This page contains the revision history of the i.MX 93 Applications Processor datasheet, detailing changes made in revisions 4 and 5, including updates to tables, figures, descriptions, and removal of certain tables.

#### Notes

- CONTINUES ON NEXT PAGE


## Table 104. i.MX 93 Data Sheet document revision history...continued

*(Page 103)*

### Table 104. i.MX 93 Data Sheet document revision history...continued

| Rev. Number | Date | Substantive Change(s) |
| --- | --- | --- |
|  |  | ['Updated JTAG timing parameters', 'Updated SWD timing parameters', 'Updated LCD Controller (LCDIF) timing parameters', 'Updated SAI switching specifications', 'Updated SPDIF timing parameters', 'Updated Ultra-high-speed SD/SDIO/MMC host interface (uSDHC) AC timing', 'Updated Tables 60, 61, 62, 63, and 64', 'Updated LPSPi timing parameters', 'Updated I3C specifications', 'Updated CAN network AC electrical specifications', 'Updated CAN network electrical specifications', 'Updated Timer/Pulse width modulator (TPM) timing parameters', 'Updated FlexSPI timing parameters', 'Updated FlexIO electrical specifications', 'Updated Table 92'] |
| Rev. 3 | 12/2023 | ['Updated Table 2. Ordering information', 'Added FlexIO information in Table i.MX 93 modules list', 'Updated Figure 1. Part number nomenclature—i.MX', 'Updated Figure 2', 'Updated Special signal considerations', 'Updated Table 4. Special signal considerations', 'Updated Table 9. Absolute maximum maximum ratings', 'Updated Table 10. Electrostatic discharge and latch up ratings', 'Added Section 4.1.2.3, 14 x 14 mm FCBGA package package thermal characteristics', 'Updated Table 14. Operating ranges', 'Added Table 17. External clock frequency', 'Updated Table 21. Maximum supply currents', 'Added a note in Section 4.2.1, Power mode definition', 'Updated Table 22. The power supply states', 'Added footnotes in Table 23. Low power mode definition', 'Added Table 24. Chip power in different LP modes'] |

#### Additional Information

This page contains a continuation of the document revision history for the i.MX 93 Applications Processor datasheet. It lists substantive changes made in Revision 3, dated December 2023, including updates to various tables, figures, and sections related to timing parameters, electrical specifications, and operating conditions.

#### Notes

- The table continues on the next page.
- All information provided in this document is subject to legal disclaimers.
- © 2025 NXP B.V. All rights reserved.


## i.MX 93 Data Sheet document revision history...continued

*(Page 104)*

### Table 104. i.MX 93 Data Sheet document revision history...continued

| Rev. Number | Date | Substantive Change(s) |
| --- | --- | --- |
|  |  | ['Updated Table 26. GPIO DC parameters, Table 27. Additional leakage parameters, and Table 27', 'Updated the operating frequency in Table 30. LVDS AC parameters', 'Updated ENOB values', 'Updated ENOB values in Table 54. ADC electrical specificatificans (VREFTH = VDDA_ANA_1P8 and VDDA_ANAxP8 and VADIN_max ≤ VREFH)', 'Updated Section 4.12.2, FlexSPI timing', 'Updated Section 4.12.9, FlexSPI timing parameters', 'Update the signal name of RMI_RX_ER in Table 63. ENET signal signal mapping and Table 67. ENET QOS signal mapping', 'Updated the naming of ENET_CLK to RMI_REF_CLK, ENET_TD, and ENET_RD in Section 4.12.2, RMI in Section 4.12.2, RMI', 'Removed USB 3.0 information from Section 4.12.12, USB PHY parameters parameters', 'Added 14 x 14 mm package package information'] |
| Rev. 2 | 08/2023 | ['Updated the term "Consumer" to "Commercial"', 'Updated JTAG pin description in Table 1', 'Updated Table 2. Ordering information', 'Updated remarks in Table 4. Special signal considerations', 'Added ADC and TAMPER pin pin information in Table 5. Unused function strapping recommendations', 'Updated NVCC_BBSM_P8 description in Section 4.1.3, Power architecture', 'Updated NVCC descriptions in Section 4.1.6.1, External clock sources sources', 'Updated Figure 7, "Output transition time waveform"', 'Updated Table 26. GPIO DC parameters and Table 27. Additional leakage parameters parameters', 'Updated JTAG_TRST information from Section 4.8.3, JTAG timing parameters', 'Updated Table 63. ENET2 signal mapping and Table 67. ENET QOS signal mapping mapping', 'Updated footnotes of Table 68 and Table 69'] |
| Rev. 1 | 04/2023 | ['Updated Table 98. Fuses and associated pins used for boot'] |
| Rev. 1 | 04/2023 | ['Initial version'] |


## Legal information

*(Page 105)*

### Data sheet status status

| Document status[1][2] | Product status[3] | Definition |
| --- | --- | --- |
| Objective [short] data sheet | Development | This document contains data from the objective specification for product development. |
| Preliminary [short] data sheet | Qualification | This document contains data from the preliminary specification specification. |
| Product [short] data sheet | Production | This document contains the product specification. |

#### Additional Information

This page contains legal information and definitions related to the i.MX 93 Applications Processor datasheet. It includes details about the status of the data sheet, definitions of terms like 'draft', 'short data sheet', and 'product specification', and disclaimers regarding the use and liability of the information provided. The page also includes footnotes providing additional context and instructions for consulting the most recent documents and accessing the latest product status information online.

#### Notes

- Please consult the most recently issued document before initiating or completing a design.
- The term 'short data sheet' is explained in section 'Definitions'.
- The product status of device(s) described in this document may have changed since this document was published and may differ in case of multiple devices. The latest product status information is available on the Internet at URL https://www.nxp.com.


## Applications

*(Page 106)*

#### Additional Information

This page contains legal and disclaimer information related to the use of NXP Semiconductors products, specifically the i.MX 93 Applications Processor. Key points include:
- Applications described are for illustrative purposes only; NXP does not guarantee suitability for specific uses.
- Customers are responsible for the design and operation of their applications and products using NXP products.
- NXP accepts no liability for assistance with applications or customer product design.
- Customers are solely responsible for determining the suitability of NXP products for their applications.
- NXP products are not qualified for use in automotive applications unless explicitly stated.
- Customers must ensure compliance with all legal, regulatory, safety, and security requirements.
- NXP does not accept liability related to any default, damage, costs, or problems based on weakness or default in the customer's applications or products.
- Limiting values (stress above one or more limiting values) can cause permanent damage to the device.
- NXP products are sold subject to general terms and conditions of commercial sale.
- No offer or license is made for products open for acceptance or grant of applications.
- Security considerations are discussed, including vulnerabilities and compliance with security standards.
- NXP has a Product Security Incident Response Team (PSIRT) for managing security vulnerabilities.
- NXP B.V. is not an operating company and does not distribute or sell products.

#### Notes

- Applications described are for illustrative purposes only.
- NXP does not guarantee suitability for specific uses without further testing or modification.
- Customers are responsible for compliance with all legal, regulatory, safety, and security requirements.
- NXP does not accept liability related to any default, damage, costs, or problems based on weakness or default in the customer's applications or products.
- Limiting values can cause permanent damage to the device.
- NXP products are sold subject to general terms and conditions of commercial sale.
- No offer or license is made for products open for acceptance or grant of applications.
- Security considerations include vulnerabilities and compliance with security standards.
- NXP has a Product Security Incident Response Team (PSIRT) for managing security vulnerabilities.
- NXP B.V. is not an operating company and does not distribute or sell products.


## Trademarks

*(Page 107)*

#### Additional Information

This page contains a section titled 'Trademarks' which lists various trademarks and registered trademarks related to NXP Semiconductors and other companies. The text includes a notice stating that all referenced brands, product names, service names, and trademarks are the property of their respective owners. The trademarks listed are as follows:
- NXP (wordmark and logo)
- EdgeLock
- Synopsys & Designware
- Synopsys
- AMBA, Arm, Arm7, Arm7TDMI, Arm9, Arm11, Artisan, big.LITTLE, Cordio, CoreLink, CoreSight, Cortex, DesignStart, DynamiQ, Jazelle, Keil, Mali, Mbed, Mbed Enabled, NEON, POP, RealView, SecurCore, Socrates, Thumb, TrustZone, ULINK, ULINK2, ULINK-ME, ULINK-PLUS, ULINKpro, µVision, Versatile

The text also includes copyright notices and permissions for the use of certain trademarks.

#### Notes

- Notice: All referenced brands, product names, service names, and trademarks are the property of their respective owners.
- NXP — wordmark and logo are trademarks of NXP B.V.
- EdgeLock — is a trademark of NXP B.V.
- Synopsys & Designware — are registered trademarks of Synopsys, Inc.
- Synopsys — Portions Copyright © 2018-2022 Synopsys, Inc. Used with permission. All rights reserved.
- AMBA, Arm, Arm7, Arm7TDMI, Arm9, Arm11, Artisan, big.LITTLE, Cordio, CoreLink, CoreSight, Cortex, DesignStart, DynamiQ, Jazelle, Keil, Mali, Mbed, Mbed Enabled, NEON, POP, RealView, SecurCore, Socrates, Thumb, TrustZone, ULINK, ULINK2, ULINK-ME, ULINK-PLUS, ULINKpro, µVision, Versatile — are trademarks and/or registered trademarks of Arm Limited (or its subsidiaries or affiliates) in the US and/or elsewhere. The related technology may be protected by any or all of patents, copyrights, designs and trade secrets. All rights reserved.


## Contents

*(Page 108)*

#### Additional Information

This page contains the table of contents for the i.MX 93 Applications Processor datasheet. It lists various sections and subsections with their corresponding page numbers. The contents cover topics such as introduction, ordering information, block diagrams, special considerations, electrical characteristics, power modes, I/O parameters, timing parameters, memory controller specifications, and more. The document is titled 'i.MX 93 Applications Processors Data Sheet for Automotive Products' and is Revision 6.1 dated 7 July 2025.

#### Notes

- Please be aware that important notices concerning this document and the product(s) described herein have been included in section 'Legal information'.
