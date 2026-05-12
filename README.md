# ESP32 HVAC Control System

https://wokwi.com/projects/463782844657849345

- FreeRTOS multitasking architecture
- Interrupt-driven emergency and override buttons
- Software debouncing for reliable inputs
- 12-bit ESP32 ADC temperature sensing
- DHT22 digital temperature sensor integration
- Mutex-protected shared system data
- Industrial fail-safe safety logic
- Overheat protection and emergency shutdown
- Buzzer alarm system

| State | Condition | System Behaviour |
|------|----------|------------------|
| Normal | < 40°C | System idle |
| Warning | > 40°C | Cooling activated |
| Critical | > 60°C | Shutdown + alarm |
| Emergency | Button pressed | Immediate shutdown |
| Fault | Sensor error | Safe mode |