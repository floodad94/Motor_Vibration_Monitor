
---

# Step 2 — Fill `docs/modbus-register-map.md`

Paste this:

```md
# Modbus Register Map

Each ESP32 sensor node acts as a Modbus RTU slave.

## Slave Addresses

- Node 1 → Address 1
- Node 2 → Address 2
- Node 3 → Address 3
- Node 4 → Address 4
- Node 5 → Address 5

## Input Registers (Read-Only)

| Address | Name | Description |
|--------|------|------------|
| 30001 | Node ID | Logical node ID |
| 30002 | Status | 0=Normal, 1=Warning, 2=Fault |
| 30003 | X RMS (mg) | X-axis RMS acceleration |
| 30004 | Y RMS (mg) | Y-axis RMS acceleration |
| 30005 | Z RMS (mg) | Z-axis RMS acceleration |
| 30006 | Overall RMS (mg) | Combined RMS |
| 30007 | BDU ×10 | Bearing damage units |
| 30008 | Piezo Peak | Raw peak value |
| 30009 | Piezo Level | Processed level |
| 30010 | Spike Count | Impact counter |
| 30011 | Error Flags | Fault bits |
| 30012 | Uptime | Seconds |

## Holding Registers (Writable)

| Address | Name | Description |
|--------|------|------------|
| 40001 | Warning BDU ×10 | Warning threshold |
| 40002 | Fault BDU ×10 | Fault threshold |
| 40003 | Piezo Warning | Spike threshold |
| 40004 | Piezo Fault | Spike fault level |
| 40005 | LED Enable | 0/1 |