## 1. System Overview

The system replaces a high-maintenance Bluetooth Low Energy (BLE) setup that suffered from range limitations (requiring gateways to be placed near hives, leading to theft) and high power consumption (~200mA peak). The new LoRaWAN architecture uses a Long Range (LoRa) radio link to allow the gateway to be placed in a secure location far from the apiary.

### Core Components

* 
**End Node:** LoRa-E5 (WLE5JC) Development Board (ARM Cortex-M4 + LoRa SoC).


* 
**Sensor:** Sensirion SHT45 (High-precision Temperature/Humidity) connected via I2C.


* 
**Gateway:** Raspberry Pi 4 + WM1302 LoRaWAN Hat (SX1302 chipset).


* 
**Software:** STM32CubeWL SDK, Semtech Packet Forwarder, and ChirpStack Network Server.



---

## 2. Hardware Assembly

### Sensor Wiring

The Sensirion SHT45 communicates with the LoRa-E5 via the I2C2 peripheral.

1. Connect **VIN** on the SHT45 to **3V3** on the LoRa-E5 board.


2. Connect **GND** to **GND**.


3. Connect **SCL** to **PB14** (I2C2 SCL).


4. Connect **SDA** to **PA15** (I2C2 SDA).



### Gateway Setup

1. Mount the **WM1302 Hat** onto the Raspberry Pi 4 GPIO headers.


2. Connect the **915 MHz antenna** to the U.FL/SMA connector on the Hat.


* *Warning:* Never power the gateway without the antenna attached, as it can damage the concentrator chip.


3. Connect the Pi to the internet via **Ethernet or Wi-Fi** for the backhaul.



---

## 3. Firmware Development

The firmware is built on the `LoRaWAN_End_Node` example from the STM32WL SDK.

### Low Power Configuration

To ensure the battery lasts longer than the previous 14-day limit, the MCU must enter **Stop2 Mode** between measurement cycles.

* Use the `stm32_seq.h` sequencer to schedule tasks without using a full RTOS.


* Ensure all unused GPIOs are set to Analog mode to minimize leakage current.



### Sensor Logic (`lora_app.c`)

1. 
**Command:** Issue the `0xFD` command for a high-precision measurement.


2. 
**Timing:** Use a **20ms non-blocking delay** to allow the sensor to finish conversion.


3. 
**Data Integrity:** Read 6 bytes from the sensor; verify the **CRC-8 checksums** for both temperature and humidity.



### Payload Structure

To minimize Time-on-Air (ToA) and save energy, use the **Cayenne LPP** format:

* 
**Channel 1 (Type 103):** Temperature (Value  10).


* 
**Channel 2 (Type 115):** Packet Counter.


* 
*Security:* The stack automatically applies **AES-128 encryption** to the payload.



---

## 4. Gateway and Backend Configuration

### Packet Forwarder

The Raspberry Pi runs the **Semtech Packet Forwarder**, which bridges RF packets to IP packets (UDP/JSON).

1. Configure `global_conf.json` with the correct frequency plan (US915 for UCR).


2. Ensure the Gateway ID in the config matches the one registered in ChirpStack.



### ChirpStack Decoding

In the ChirpStack web console, add a **Javascript Codec** to decode the hex payload.

```javascript
// Example decoder logic
function decodeUplink(input) {
  var temp = (input.bytes[2] << 8 | input.bytes[3]) / 10.0; [cite_start]// Channel 1 [cite: 64, 65]
  return { data: { temperature: temp } };
}

```

---

## 5. Testing and Validation

1. 
**OTAA Handshake:** Monitor the ChirpStack console during power-up to ensure a successful Join-Request/Join-Accept cycle.


2. 
**Thermal Test:** Blow warm air onto the SHT45 and verify the real-time data spike in the backend.


3. 
**Range Check:** Expect a **Packet Delivery Rate (PDR)** of roughly 25% at 30 meters in non-line-of-sight conditions; the 3-second uplink frequency provides sufficient redundancy to handle this loss.



---

## 6. Future Improvements

* 
**Energy Harvesting:** Integrate a solar panel to eliminate battery swaps entirely.


* 
**Edge AI:** Implement local anomaly detection on the STM32WL to only transmit data when a temperature deviation is detected.


* 
**Visualization:** Connect ChirpStack to **InfluxDB and Grafana** for professional-grade analytics.

