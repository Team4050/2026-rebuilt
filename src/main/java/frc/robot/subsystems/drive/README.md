# Drive Subsystem

Swerve drive implementation using the AdvantageKit IO layer pattern. This architecture separates hardware interaction from control logic, enabling deterministic replay and physics simulation. These files should not need to be manually modified.

## Architecture Overview

```mermaid
graph TB
    subgraph Drive Subsystem
        D[Drive]
        M0[Module 0<br/>Front Left]
        M1[Module 1<br/>Front Right]
        M2[Module 2<br/>Back Left]
        M3[Module 3<br/>Back Right]
    end

    subgraph IO Layer
        GIO[GyroIO]
        MIO[ModuleIO]
    end

    subgraph Hardware Implementations
        GP2[GyroIOPigeon2]
        MTFX[ModuleIOTalonFX]
    end

    subgraph Simulation
        MSIM[ModuleIOSim]
    end

    subgraph Threading
        POT[PhoenixOdometryThread]
    end

    D --> M0 & M1 & M2 & M3
    D --> GIO
    M0 & M1 & M2 & M3 --> MIO
    GIO -.-> GP2
    MIO -.-> MTFX
    MIO -.-> MSIM
    MTFX --> POT
    GP2 --> POT
```

## Components

### Drive.java

Main subsystem class. Coordinates four swerve modules and a gyro to provide:

- Velocity-based driving via `runVelocity(ChassisSpeeds)`
- Pose estimation using `SwerveDrivePoseEstimator` with vision measurement fusion
- PathPlanner integration for autonomous path following
- SysId routines for drive characterization
- Defensive X-lock stance via `stopWithX()`

### Module.java

Wrapper around a `ModuleIO` implementation. Handles:

- Periodic input logging via AdvantageKit
- Odometry position calculation from high-frequency samples
- Setpoint optimization (cosine scaling, direction optimization)
- Hardware disconnect alerting

### ModuleIO.java

Interface defining swerve module hardware operations:

- Drive/turn motor open-loop and closed-loop control
- Sensor input collection (position, velocity, current, voltage)
- High-frequency odometry data (timestamps, positions)

### ModuleIOTalonFX.java

Hardware implementation using CTRE TalonFX motors and CANcoder. Features:

- Configurable voltage or torque-current (FOC) control modes
- Fused/Remote/Sync CANcoder feedback options
- Motion Magic for turn positioning
- Current limiting at slip threshold

### ModuleIOSim.java

Physics simulation using WPILib `DCMotorSim`. Provides:

- Voltage-based motor simulation with configurable gains
- Simulated PID control for velocity and position
- 50Hz odometry sampling (sufficient for simulation)

### GyroIO.java

Interface for gyroscope hardware:

- Yaw position and velocity
- High-frequency odometry samples for pose estimation

### GyroIOPigeon2.java

CTRE Pigeon2 IMU implementation. Registers yaw signal with the odometry thread for synchronized sampling.

### PhoenixOdometryThread.java

Background thread for high-frequency odometry sampling (250Hz on CANivore, 100Hz on RIO CAN). Features:

- Synchronized sampling using Phoenix `waitForAll` on CAN FD
- Latency-compensated timestamps
- Thread-safe signal registration via queues

## Data Flow

```mermaid
sequenceDiagram
    participant OT as PhoenixOdometryThread
    participant HW as Hardware (TalonFX/Pigeon2)
    participant IO as ModuleIO/GyroIO
    participant M as Module
    participant D as Drive

    loop 250Hz (CANivore) / 100Hz (RIO)
        OT->>HW: waitForAll / refreshAll
        HW-->>OT: Position signals
        OT->>OT: Queue samples with timestamps
    end

    loop 50Hz (Robot periodic)
        D->>D: Acquire odometryLock
        D->>IO: updateInputs()
        IO->>OT: Drain queues
        IO-->>M: Populated inputs
        M->>D: getOdometryPositions()
        D->>D: Update pose estimator
        D->>D: Release odometryLock
    end
```

## Key Constants

| Constant             | Source       | Description                            |
| -------------------- | ------------ | -------------------------------------- |
| `ODOMETRY_FREQUENCY` | `Drive.java` | 250Hz (CAN FD) or 100Hz (standard CAN) |
| `DRIVE_BASE_RADIUS`  | `Drive.java` | Calculated from module positions       |
| `PP_CONFIG`          | `Drive.java` | PathPlanner robot configuration        |

Module-specific constants (gear ratios, offsets, PID gains) are sourced from `TunerConstants`.
