# Grouped Signal Connections

## LV-Box <-> Accumulator

| Signal Name  | Harness Label  | Start Point  | End Point  | Voltage Domain  | Wire Gauge  |
| --- | --- | --- | --- | --- | --- |
| TSAL+12-Green  | TSAL+12V  | LV-Box  | Accumulator  | TSAL  | 0.5  |
| TSAL-GND-Green  | TSAL-GND  | LV-Box  | Accumulator  | TSAL  | 0.5  |
| +24-Accu  | +24-Accu  | LV-Box  | Accumulator  | 24V  | 0.5  |
| AMS-OK  | AMS-OK  | LV-Box  | Accumulator  | 24V  | 0.5  |
| IMD-OK  | IMD-OK  | LV-Box  | Accumulator  | 24V  | 0.5  |


## LV-Box <-> DC-Link

| Signal Name  | Harness Label  | Start Point  | End Point  | Voltage Domain  | Wire Gauge  |
| --- | --- | --- | --- | --- | --- |
| TSAL+12-Red  | TSAL+12V  | LV-Box  | DC-Link  | TSAL  | 0.5  |
| TSAL-GND-Red  | TSAL-GND  | LV-Box  | DC-Link  | TSAL  | 0.5  |
| +24-BSPD  | +24-CTRL  | LV-Box  | DC-Link  | 24V  | 0.5  |
| +24-DC-Link  | +24-CTRL  | LV-Box  | DC-Link  | 24V  | 0.5  |
| GND-BSPD  | GND-CTRL  | LV-Box  | DC-Link  | 24V  | 0.5  |


## LV-Box <-> APPS

| Signal Name  | Harness Label  | Start Point  | End Point  | Voltage Domain  | Wire Gauge  |
| --- | --- | --- | --- | --- | --- |
| +24-APPS  | +24-CTRL  | LV-Box  | APPS  | 24V  | 0.5  |


## LV-Box <-> ASSI-Left

| Signal Name  | Harness Label  | Start Point  | End Point  | Voltage Domain  | Wire Gauge  |
| --- | --- | --- | --- | --- | --- |
| +12  | +12  | LV-Box  | ASSI-Left  | 12V  | 0.5  |
| Yellow  | Yellow  | LV-Box  | ASSI-Left  | 12V  | 0.5  |
| Blue  | Blue  | LV-Box  | ASSI-Left  | 12V  | 0.5  |


## LV-Box <-> ASSI-Rear

| Signal Name  | Harness Label  | Start Point  | End Point  | Voltage Domain  | Wire Gauge  |
| --- | --- | --- | --- | --- | --- |
| +12  | +12  | LV-Box  | ASSI-Rear  | 12V  | 0.5  |
| Yellow  | Yellow  | LV-Box  | ASSI-Rear  | 12V  | 0.5  |
| Blue  | Blue  | LV-Box  | ASSI-Rear  | 12V  | 0.5  |


## LV-Box <-> ASSI-Right

| Signal Name  | Harness Label  | Start Point  | End Point  | Voltage Domain  | Wire Gauge  |
| --- | --- | --- | --- | --- | --- |
| +12  | +12  | LV-Box  | ASSI-Right  | 12V  | 0.5  |
| Yellow  | Yellow  | LV-Box  | ASSI-Right  | 12V  | 0.5  |
| Blue  | Blue  | LV-Box  | ASSI-Right  | 12V  | 0.5  |


## LV-Box <-> Brake-Light

| Signal Name  | Harness Label  | Start Point  | End Point  | Voltage Domain  | Wire Gauge  |
| --- | --- | --- | --- | --- | --- |
| +12  | +12  | LV-Box  | Brake-Light  | 24V  | 0.5  |
| Brake-Light  | Brake-Light  | LV-Box  | Brake-Light  | 24V  | 0.5  |


## LV-Box <-> Buzzer

| Signal Name  | Harness Label  | Start Point  | End Point  | Voltage Domain  | Wire Gauge  |
| --- | --- | --- | --- | --- | --- |
| +24  | +24  | LV-Box  | Buzzer  | 24V  | 0.5  |
| Sound1  | S1  | LV-Box  | Buzzer  | 24V  | 0.5  |
| Sound2  | S2  | LV-Box  | Buzzer  | 24V  | 0.5  |


## LV-Box <-> Dashboard

| Signal Name  | Harness Label  | Start Point  | End Point  | Voltage Domain  | Wire Gauge  |
| --- | --- | --- | --- | --- | --- |
| TSAL+12-Dash  | TSAL+12V  | LV-Box  | Dashboard  | TSAL  | 0.5  |
| AMS-Status  | AMS-Status  | LV-Box  | Dashboard  | 5V  | TBD  |
| IMD-Status  | IMD-Status  | LV-Box  | Dashboard  | 5V  | TBD  |
| R2D  | R2D  | LV-Box  | Dashboard  | 5V  | TBD  |


## LV-Box <-> Datalogger

| Signal Name  | Harness Label  | Start Point  | End Point  | Voltage Domain  | Wire Gauge  |
| --- | --- | --- | --- | --- | --- |
| +12-DataLogger  | +12-DL  | LV-Box  | Datalogger  | 12V  | 0.5  |
| GND-DataLogger  | GND-DL  | LV-Box  | Datalogger  | 12V  | 0.5  |


## LV-Box <-> Fan Left

| Signal Name  | Harness Label  | Start Point  | End Point  | Voltage Domain  | Wire Gauge  |
| --- | --- | --- | --- | --- | --- |
| +24-FanL  | +24-FanL  | LV-Box  | Fan Left  | 24V  | 0.5  |
| GND-FanL  | GND-FanL  | LV-Box  | Fan Left  | 24V  | 0.5  |


## LV-Box <-> Fan Right

| Signal Name  | Harness Label  | Start Point  | End Point  | Voltage Domain  | Wire Gauge  |
| --- | --- | --- | --- | --- | --- |
| +24-FanR  | +24-FanR  | LV-Box  | Fan Right  | 24V  | 0.5  |
| GND-FanR  | GND-FanR  | LV-Box  | Fan Right  | 24V  | 0.5  |


## LV-Box <-> Inverter

| Signal Name  | Harness Label  | Start Point  | End Point  | Voltage Domain  | Wire Gauge  |
| --- | --- | --- | --- | --- | --- |
| +24-INV  | +24-CTRL  | LV-Box  | Inverter  | 24V  | 0.5  |
| RFE  | +24-CTRL  | LV-Box  | Inverter  | 24V  | 0.5  |
| GND-INV  | GND-CTRL  | LV-Box  | Inverter  | 24V  | 0.5  |
| GNDE-INV  | GND-CTRL  | LV-Box  | Inverter  | 24V  | 0.5  |


## LV-Box <-> Pump

| Signal Name  | Harness Label  | Start Point  | End Point  | Voltage Domain  | Wire Gauge  |
| --- | --- | --- | --- | --- | --- |
| +24-Pump  | +24-Pump  | LV-Box  | Pump  | 24V  | 0.5  |
| GND-Pump  | GND-Pump  | LV-Box  | Pump  | 24V  | 0.5  |


## LV-Box <-> Side-Panel

| Signal Name  | Harness Label  | Start Point  | End Point  | Voltage Domain  | Wire Gauge  |
| --- | --- | --- | --- | --- | --- |
| +5  | +5V  | LV-Box  | Side-Panel  | 5V  | TBD  |
| Reset  | Reset  | LV-Box  | Side-Panel  | 5V  | TBD  |


## LV-Box <-> TSAL-LED

| Signal Name  | Harness Label  | Start Point  | End Point  | Voltage Domain  | Wire Gauge  |
| --- | --- | --- | --- | --- | --- |
| TSAL+12-LED  | TSAL+12V  | LV-Box  | TSAL-LED  | TSAL  | 0.5  |
| TSAL-GND-LED  | TSAL-GND  | LV-Box  | TSAL-LED  | TSAL  | 0.5  |


## LV-Box <-> TSMS

| Signal Name  | Harness Label  | Start Point  | End Point  | Voltage Domain  | Wire Gauge  |
| --- | --- | --- | --- | --- | --- |
| HVIL  | HVIL  | LV-Box  | TSMS  | 24V  | 0.5  |
| TSMS  | TSMS  | LV-Box  | TSMS  | 24V  | 0.5  |


## Accumulator <-> DC-Link

| Signal Name  | Harness Label  | Start Point  | End Point  | Voltage Domain  | Wire Gauge  |
| --- | --- | --- | --- | --- | --- |
| Discharge-Disable  | Discharge-Disable  | DC-Link  | Accumulator  | 24V  | 0.5  |


## Accumulator <-> Chassis

| Signal Name  | Harness Label  | Start Point  | End Point  | Voltage Domain  | Wire Gauge  |
| --- | --- | --- | --- | --- | --- |
| GND-Accu  | GND-Accu  | Chassis  | Accumulator  | 24V  | 1.5  |


## Accumulator <-> Dashboard

| Signal Name  | Harness Label  | Start Point  | End Point  | Voltage Domain  | Wire Gauge  |
| --- | --- | --- | --- | --- | --- |
| TS-Activation  | TS-Activation  | Accumulator  | Dashboard  | 5V  | TBD  |
| TS-OFF  | TS-OFF  | Accumulator  | Dashboard  | 5V  | TBD  |


## Accumulator <-> Inverter

| Signal Name  | Harness Label  | Start Point  | End Point  | Voltage Domain  | Wire Gauge  |
| --- | --- | --- | --- | --- | --- |
| FRG/Run  | FRG  | Inverter  | Accumulator  | 24V  | 0.5  |


## Accumulator <-> TSAL-LED

| Signal Name  | Harness Label  | Start Point  | End Point  | Voltage Domain  | Wire Gauge  |
| --- | --- | --- | --- | --- | --- |
| TS-OFF  | TS-OFF  | TSAL-LED  | Accumulator  | 5V  | TBD  |


## Accumulator <-> TSMS

| Signal Name  | Harness Label  | Start Point  | End Point  | Voltage Domain  | Wire Gauge  |
| --- | --- | --- | --- | --- | --- |
| HVIL  | HVIL  | TSMS  | Accumulator  | 24V  | 0.5  |


## DC-Link <-> TSAL-LED

| Signal Name  | Harness Label  | Start Point  | End Point  | Voltage Domain  | Wire Gauge  |
| --- | --- | --- | --- | --- | --- |
| TSAL-Red-Enable  | TS-Active  | TSAL-LED  | DC-Link  | 5V  | TBD  |


## Chassis <-> APPS

| Signal Name  | Harness Label  | Start Point  | End Point  | Voltage Domain  | Wire Gauge  |
| --- | --- | --- | --- | --- | --- |
| GND-APPS  | GND-APPS  | Chassis  | APPS  | 24V  | 0.5  |


## Chassis <-> Dashboard

| Signal Name  | Harness Label  | Start Point  | End Point  | Voltage Domain  | Wire Gauge  |
| --- | --- | --- | --- | --- | --- |
| GND-Dash  | GND-Dash  | Chassis  | Dashboard  | TSAL  | 0.5  |

