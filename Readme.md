This README provides a comprehensive, step-by-step guide for researchers or students continuing the **LoRaWAN-Based Beehive Monitoring Project** at the University of California, Riverside (UCR).

---

# LoRaWAN-Based Beehive Monitoring System (UCR)

This project implements a long-range, low-power temperature and humidity monitoring system for the UCR beehives. It replaces a legacy Bluetooth Low Energy (BLE) system that suffered from range limitations, hardware theft, and high maintenance requirements.

## 1. System Overview

The architecture is designed for remote, infrastructure-less environments where cellular signals are non-existent and fixed power is unavailable.

### Technical Specifications


**End Node:** LoRa-E5 (WLE5JC) Development Board (ARM Cortex-M4 + LoRa SoC).



**Sensor:** Sensirion SHT45 connected via I2C (Pins PA15/PB14).



**Gateway:** Raspberry Pi 4 + WM1302 LoRaWAN Hat (915 MHz US Band).


* 
**Backend:** ChirpStack LoRaWAN Network Server.



---

## 2. Hardware Assembly

### Sensor Wiring (End Node)

The Sensirion SHT45 uses the I2C2 peripheral for communication.

1. **VIN** on SHT45  **3V3** on LoRa-E5.
2. **GND** on SHT45  **GND** on LoRa-E5.
3. **SCL** on SHT45  **PB14** (I2C2 SCL) on LoRa-E5.


4. **SDA** on SHT45  **PA15** (I2C2 SDA) on LoRa-E5.



### Gateway Assembly

1. Mount the **WM1302 Hat** onto the Raspberry Pi 4 GPIO headers.


2. Connect the **915 MHz high-gain antenna** to the U.FL/SMA connector.



*Critical:* Do not power the Pi without an antenna attached, as it can damage the SX1302 concentrator.





---

## 3. Firmware Configuration

The firmware is built using the **STM32CubeWL Middleware**.

### Power Optimization

To avoid the bi-weekly battery swaps of the old system (which caused thermal shock to the bees), the node must be ultra-low power.


**MCU State:** Program the MCU to enter **Stop2 Mode** between tasks.


  
**Sequencer:** Use `stm32_seq.h` to manage measurement and transmission tasks without RTOS overhead.



### Measurement Logic (`lora_app.c`)

1. **Command:** Send measurement command `0xFD` (High-Precision) via `HAL_I2C_Master_Transmit`.


2. **Delay:** Implement a **20ms non-blocking delay** for sensor conversion.


3. **Read:** Receive 6 bytes via `HAL_I2C_Master_Receive` and verify the **CRC-8 checksums**.



---

## 4. Data Uplink & Payload

To minimize Time-on-Air (ToA) and energy consumption, use the **Cayenne LPP** format.


**Channel 1 (Type 103):** Temperature (Precision: $0.1^{\circ}$C).



**Channel 2 (Type 115):** Packet Counter.



**Security:** Payloads are automatically **AES-128 encrypted** by the stack.



---

## 5. Gateway & Network Server (ChirpStack)

### Semtech Packet Forwarder

The Raspberry Pi bridges RF packets to the internet.

1. Enable **SPI** on the Raspberry Pi.
2. Configure the packet forwarder to listen to the SX1302 chip.
3. Point the forwarder to your ChirpStack instance.



### Custom Payload Decoder (JavaScript)

In the ChirpStack console, use a decoder to reconstruct the 16-bit signed integer into a decimal value.

```javascript
function decodeUplink(input) {
  // Map Channel 1 (Temperature)
  var temp = (input.bytes[2] << 8 | input.bytes[3]) / 10.0;
  return {
    data: { temperature: temp }
  };
}

```

---

## 6. Testing & Validation

1. **OTAA Handshake:** Confirm "Join Request/Accept" in the ChirpStack console and the ST-LINK debugger.


2. **Thermal Variance:** Introduce a heat source near the sensor and observe immediate shifts in the server packets.


3. **Redundancy:** Note that while range increases packet loss (e.g., 25% PDR at 30m), the 3-second uplink frequency maintains a consistent data curve.



---

## 7. Future Roadmap


**Solar Integration:** Add energy harvesting for infinite autonomy.



**Edge Intelligence:** Perform local anomaly detection on the Cortex-M4 to reduce transmission frequency.



**Visualization:** Bridge the backend with **InfluxDB** and **Grafana** for professional analytics.



---

**Advisor:** Dr. Hyoseung Kim **Location:** University of California, Riverside (UCR) 

