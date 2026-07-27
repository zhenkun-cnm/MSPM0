# Vehicle Synchronization

## Sync status

- Sync revision: `2`
- Vehicle 1 repository: `big-car` (`C:\ti\mspm0_project_big_car`)
- Vehicle 2 repository: `vehicle2` (`C:\ti\mspm0_project`)
- Protocol state: `BUILD_VERIFIED_TARGET_TEST_PENDING`

## UART2 vehicle-link contract

| Item | Value |
| --- | --- |
| Link | UART2, 115200 baud, 8-N-1 |
| Wiring | Vehicle 1 PB17/TX -> Vehicle 2 PB18/RX; Vehicle 2 PB17/TX -> Vehicle 1 PB18/RX; common GND |
| Vehicle 1 node ID | `0x01` |
| Vehicle 2 node ID | `0x02` |
| Broadcast ID | `0xFF` |
| Maximum payload | 20 bytes |
| ACK timeout / retries | 50 ms / 3 |
| Frame receive timeout | 20 ms |

Frame: `AA 55 | seq | src | dst | cmd | len | data[0..len-1] | xor | 0D 0A`.

`xor` is the XOR of `seq`, `src`, `dst`, `cmd`, `len`, and every data byte. Unicast messages request ACK; broadcasts do not. ACK command `0xF0` carries the acknowledged sequence number in `data[0]`.

## Message registry

| Command | Direction | Payload | Meaning | Status |
| --- | --- | --- | --- | --- |
| `0x01` | either direction | none | PING | implemented |
| `0xF0` | automatic response | original `seq` | ACK | implemented |
| `0x10`-`0xDF` | only after registration | define before coding | application messages | reserved |

No unregistered command may be implemented or transmitted as a business message.

## Task board

| ID | Owner | Task | Acceptance | Status |
| --- | --- | --- | --- | --- |
| V1-001 | Vehicle 1 | Register and start car communication UART2 task; node ID `0x01`. | Keil build succeeds and boot log reports UART2 ready. | build verified; target pending |
| V2-001 | Vehicle 2 | Build with `CAR_COMM_NODE_ID=2U`; retain UART2 task. | Keil build succeeds and boot log reports node ID 2. | build verified; target pending |
| INT-001 | Both | Cross-wire UART2 and test ping/message exchange. | `comm ping 02`, `comm send 01 10 55`, ACK and receive counters match. | pending |

## Local settings

- Vehicle 1 TX/RX queue depth: 8 / 8.
- Vehicle 2 TX/RX queue depth: 2 / 2.
- Queue depth is local buffering, not a wire-protocol requirement.
- Vehicle 1 build: `Code=88032 RO=14356 RW=496 ZI=30392`, 0 Error / 0 Warning.
- Vehicle 2 build: `Code=94280 RO=15748 RW=508 ZI=30868`, 0 Error / 0 Warning.

## Vehicle 2 release record

- Firmware source commit: `8ede22b` (`feat: configure vehicle 2 communication link`).
- Release branch: `vehicle2`; target remote: `github-target` (`https://github.com/zhenkun-xianren/MSPM0.git`).
- Keil clean rebuild (2026-07-27): `Code=94280 RO=15748 RW=508 ZI=30868`, 0 Error / 0 Warning.
- Pending integration: cross-wire UART2 with common ground; run `comm ping 02` and `comm send 01 10 55`; confirm ACK, receive, and retry counters on both vehicles. Also capture Vehicle 2 boot evidence reporting node ID 2 and UART2 ready.

## Update rules

1. Read this file before changing vehicle-link code.
2. Add or modify a message only after updating the registry and task board in both repository copies.
3. Mark a task complete only after both the build result and target-board evidence are recorded.
