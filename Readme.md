# README.md

## LoRaWAN-Based Beehive Monitoring System (UCR)

This repository contains the documentation and implementation details for a long-range, low-power temperature and humidity monitoring system developed at the **University of California, Riverside (UCR)**. This system replaces a legacy Bluetooth Low Energy (BLE) architecture to ensure colony health through precise thermal monitoring.

---

## 1. Project Motivation

Beehives are often located in remote areas with no cellular coverage and no fixed power. The previous BLE system failed because:

* 
**Range:** BLE required gateways to be placed near the hives, leading to frequent hardware theft.


* 
**Power:** High current draw (200mA) depleted batteries in two weeks, requiring frequent hive openings that stressed the bees.


* 
**Reliability:** A single gateway failure caused a total data blackout.



The new **LoRaWAN** system provides kilometers of range and years of battery life.

---

## 2. Hardware Requirements

### End Node (The Hive Sensor)

* 
**MCU/Radio:** LoRa-E5 (WLE5JC) Dev Board (ARM Cortex-M4 + LoRa SoC).


* 
**Sensor:** Sensirion SHT45 (High-precision temperature/humidity).



### Gateway (The Receiver)

* 
**Host:** Raspberry Pi 4.


* 
**Concentrator:** WM1302 LoRaWAN Hat (SX1302 chip) tuned to 915 MHz (US Band).



---

## 3. Hardware Assembly

### Sensor Wiring

The SHT45 connects to the LoRa-E5 via the **I2C2** peripheral:

1. 
**VIN** on SHT45  **3V3** on LoRa-E5.


2. 
**GND** on SHT45  **GND** on LoRa-E5.


3. 
**SCL** on SHT45  **PB14** (I2C2 SCL) on LoRa-E5.


4. 
**SDA** on SHT45  **PA15** (I2C2 SDA) on LoRa-E5.



### Gateway Setup

1. Attach the **WM1302 Hat** to the Raspberry Pi 4 GPIO pins.


2. Connect a **915 MHz antenna** before powering on to prevent hardware damage.



---

## 4. Firmware Implementation

The firmware is built using **STM32CubeWL Middleware**.

### Low Power Logic

* 
**Task Management:** Uses `stm32_seq.h` to run tasks efficiently without a heavy RTOS.


* 
**Sleep Mode:** The MCU enters **Stop2 Mode** between measurements to maximize battery life.



### Sensor Communication (`lora_app.c`)

* 
**Command:** The MCU sends the `0xFD` command for high-precision readings.


* 
**Delay:** A 20ms non-blocking delay is required for the sensor to complete its internal conversion.


* 
**Validation:** The 6-byte payload includes **CRC-8 checksums** to ensure data integrity.



---

## 5. Data Uplink & Backend

* 
**Payload Format:** Uses **Cayenne LPP** to minimize Time-on-Air (ToA).


* 
**Channels:** Temperature is mapped to Channel 1 (Type 103) in **0.1°C increments**.


* 
**Security:** Payloads are **AES-128 encrypted** before transmission.


* 
**Decoding:** A custom JavaScript decoder in **ChirpStack** reconstructs the integers into decimal values for visualization.



---

## 6. Testing and Performance

* 
**Validation:** OTAA activation and sensor accuracy were verified using the ST-LINK debugger and thermal variance tests.


* **Range Data:**
* 
**Lab (LoS):** ~40% Packet Delivery Rate (PDR).


* 
**Terrace (~30m):** ~25% PDR.




* 
**Redundancy:** High-frequency uplink (every 3 seconds) ensures a high-precision data curve despite packet loss.



---

## 7. Future Work

* 
**Edge Intelligence:** Implement local anomaly detection on the MCU to reduce radio transmissions.


* 
**Power:** Add **Solar Energy Harvesting** for infinite autonomous operation.


* 
**Analytics:** Integrate with **InfluxDB** and **Grafana**.



---

**Student:** Rishi Patel **Advisor:** Dr. Hyoseung Kim **University of California, Riverside** 

Would you like me to generate the specific C code for the `lora_app.c` sensor measurement function?
