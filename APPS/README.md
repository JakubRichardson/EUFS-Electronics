# APPS

## Setup

In the `C:\Users\<user>\AppData\Local\Arduino15\packages\Seeeduino\hardware\renesas_uno\1.2.0\variants\XIAORA4M1\pins_arduino.h` file change:

```
#define PIN_CAN0_TX       (10)
#define PIN_CAN0_RX       (9)
```

To:

```
#define PIN_CAN0_TX       (18)
#define PIN_CAN0_RX       (17)
```