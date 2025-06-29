# EUFS Electronics

Code repo for electronics/hardware on the EUFS ADS entry

## Accumulator
- [BMSMS](./BMSMS/BMSMS.ino)
- [Pre-Charge](./Pre-Charge/Pre-Charge.ino)

## LV Controls
- [APPS](./APPS/APPS.ino)
- [Control-PCB](./Control-PCB/Control-PCB.ino)
- [VCU-IO](./VCU-IO/VCU-IO.ino)


Plan:
- APPS
    - Reads pedal position
    - Only operate when in manual driving mode -> Use mailbox - future
    - Sends position request -> Use mailbox
    - Add plausibilty check

- Control-PCB
    - Reads message with brake pressure from APPS. Uses threshold to control brake light -> Use mailbox for single message
    - Reads state message to control sounds and lights -> Use mailbox

- VCU-IO
    - Sends AS status -> Send every 250ms
    - Activates inverter when in manual driving -> One shot
    - If APPS not received for 100ms or implausibility -> disable drive

- Inverter
    - Reads activation and acceleration commands
    - Sends RFE signal for VCU-IO to decide state

- AI-Computer
    - Requests acceleration
    - Replaces APPS in autonomous mode
    - Acceleration routed through VCU
