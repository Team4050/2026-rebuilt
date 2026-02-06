# Wiring Diagram and CAN ID Assignments

## CAN Bus Topology

```mermaid
flowchart LR
    RIO[["RoboRIO 2.0"]]

    subgraph canfd["CAN-FD Bus"]
        direction LR
        CANivore([CANivore])
        FL["<b>Front Left</b><br/>Drive: 11 · Steer: 12<br/>Encoder: 13"]
        BL["<b>Back Left</b><br/>Drive: 21 · Steer: 22<br/>Encoder: 23"]
        BR["<b>Back Right</b><br/>Drive: 31 · Steer: 32<br/>Encoder: 33"]
        FR["<b>Front Right</b><br/>Drive: 41 · Steer: 42<br/>Encoder: 43"]
        PIG(["Pigeon 2<br/>ID: 10"])
        TERM[/"120Ω"/]

        CANivore ==> FL ==> BL ==> BR ==> FR ==> PIG ==> TERM
    end

    subgraph can["Standard CAN Bus"]
        direction LR
        S1["Spark Max<br/>ID: 51"]
        S2["Spark Max<br/>ID: 52"]
        SN["Spark Max<br/>ID: 53+"]
        PDH[("Rev PDH<br/>ID: 1")]

        S1 ==> S2 -.-> SN ==> PDH
    end

    RIO ---|USB| CANivore
    RIO ==> S1
```

## CAN ID Summary

### CAN-FD Bus (via CANivore)

| Device                   | CAN ID |
| ------------------------ | ------ |
| Pigeon 2 IMU             | 10     |
| FL Drive Motor (TalonFX) | 11     |
| FL Steer Motor (TalonFX) | 12     |
| FL CANcoder              | 13     |
| BL Drive Motor (TalonFX) | 21     |
| BL Steer Motor (TalonFX) | 22     |
| BL CANcoder              | 23     |
| BR Drive Motor (TalonFX) | 31     |
| BR Steer Motor (TalonFX) | 32     |
| BR CANcoder              | 33     |
| FR Drive Motor (TalonFX) | 41     |
| FR Steer Motor (TalonFX) | 42     |
| FR CANcoder              | 43     |

### Standard CAN Bus (via RoboRIO)

| Device      | CAN ID |
| ----------- | ------ |
| Spark Max 1 | 51     |
| Spark Max 2 | 52     |
| Spark Max N | 53+    |
| Rev PDH     | 1      |

## ID Schema

- **Swerve modules**: `<module number><device type>`
  - Module numbers: 1 (FL), 2 (BL), 3 (BR), 4 (FR)
  - Device types: 1 (drive), 2 (steer), 3 (encoder)
- **Spark Max controllers**: Starting at 51, incrementing
- **Pigeon 2 IMU**: 10
- **Rev PDH**: 1
