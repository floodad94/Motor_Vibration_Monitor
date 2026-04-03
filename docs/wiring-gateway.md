# Gateway Wiring

## MAX485 to ESP32

| MAX485 | ESP32 |
|--------|------|
| RO | GPIO 16 |
| DI | GPIO 17 |
| DE | GPIO 18 |
| RE | GPIO 18 |

## RS-485 Bus

Topology:

Gateway → Node1 → Node2 → Node3 → Node4 → Node5

All nodes share:
- A line
- B line