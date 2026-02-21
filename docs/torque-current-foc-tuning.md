# Enabling TorqueCurrentFOC for CTRE Swerve

This guide covers switching the CTRE Phoenix 6 swerve drivetrain from Voltage to TorqueCurrentFOC closed-loop output and tuning the gains correctly. It is specific to our robot (Kraken X60 motors, 21.43:1 steer ratio, FusedCANcoder feedback, Phoenix Pro).

## Background: Why TorqueCurrentFOC?

TorqueCurrentFOC commands motors in terms of **torque current (Amps)** rather than voltage. Benefits:

- More linear torque response (current maps directly to torque)
- Better low-speed control
- Back-EMF is handled by FOC commutation, simplifying feedforward tuning

Note: Phoenix Pro already provides FOC **commutation** (the ~15% power boost) for all control modes, including open-loop voltage. TorqueCurrentFOC as a **closed-loop output type** is a separate concept — it changes what units the PID/FF controller outputs in.

## What Controls Use Which Output Type

Understanding which settings affect which driving modes is critical:

| Driving Mode | Steer Motor Control | Drive Motor Control |
|---|---|---|
| Teleop (default command) | **Closed-loop** (MotionMagicExpo) — uses `kSteerClosedLoopOutput` | **Open-loop** voltage — uses `DriveRequestType.OpenLoopVoltage` from RobotContainer |
| Auto / PathPlanner | **Closed-loop** (MotionMagicExpo) — uses `kSteerClosedLoopOutput` | **Closed-loop** velocity — uses `kDriveClosedLoopOutput` |
| SysId | Depends on routine | Depends on routine |

Key takeaway: **Steer gains always matter** (steer is always closed-loop). **Drive gains only matter for closed-loop drive** (auto paths, velocity requests). Teleop open-loop driving is unaffected by `kDriveClosedLoopOutput`.

## The Gain Unit Problem

When switching from Voltage to TorqueCurrentFOC, the **Slot0 gain units change**:

| Gain | Voltage Mode | TorqueCurrentFOC Mode |
|---|---|---|
| kP | Volts / rotation | **Amps** / rotation |
| kI | Volts / (rotation * sec) | **Amps** / (rotation * sec) |
| kD | Volts / (rotation/sec) | **Amps** / (rotation/sec) |
| kS | Volts | **Amps** |
| kV | Volts / (rotation/sec) | **Amps** / (rotation/sec) |
| kA | Volts / (rotation/sec^2) | **Amps** / (rotation/sec^2) |

The Phoenix Tuner swerve project generator outputs gains for **Voltage mode**. If you change `kSteerClosedLoopOutput` to `TorqueCurrentFOC` without re-tuning, the same numeric gain values are now interpreted as Amps instead of Volts, which produces very different behavior.

### MotionMagicExpo Profile Parameters Are Separate

CTRE's `SwerveModule.java` internally hardcodes the MotionMagicExpo profile parameters (which define the motion profile shape):

```
MotionMagicExpo_kV = 0.12 * SteerMotorGearRatio  (= 2.57 for our 21.43:1 ratio)
MotionMagicExpo_kA = 1.2  / SteerMotorGearRatio  (= 0.056 for our ratio)
```

These are **always in Volts** regardless of closed-loop output type and are **not configurable** through `SwerveModuleConstants` — the swerve module code always overwrites them. They are not the same as the Slot0 kV/kA feedforward gains, even though they share the same names.

### Why kV Must Change for TorqueCurrentFOC

In Voltage mode, the Slot0 kV feedforward compensates for **back-EMF** (the voltage the spinning motor generates that opposes the applied voltage). A typical steer kV of ~2.66 V/(rot/s) makes sense for this.

In TorqueCurrentFOC mode, back-EMF is **already handled by the FOC commutation**. The Slot0 kV feedforward only needs to compensate for viscous friction, which is very small. Using the Voltage-mode value of 2.66 as an Amps feedforward injects ~12A of unnecessary current during steer motion, causing massive overshoot.

**For TorqueCurrentFOC, Slot0 kV should be 0 (or near-zero).**

## Step-by-Step: Enabling TorqueCurrentFOC

### 1. Change the Output Type Enums

In `TunerConstants.java`:

```java
private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.TorqueCurrentFOC;
private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.TorqueCurrentFOC;
```

### 2. Update Steer Gains

Replace the Voltage-mode steer gains with TorqueCurrentFOC gains:

```java
// Voltage mode defaults (DO NOT USE with TorqueCurrentFOC):
//   kP=100, kD=0.5, kS=0.1, kV=2.66, kA=0

// TorqueCurrentFOC starting gains:
private static final Slot0Configs steerGains = new Slot0Configs()
        .withKP(100)   // Amps per rotation of error — start here, tune down if oscillating
        .withKI(0)
        .withKD(0.5)   // Amps per (rot/s) of error derivative — tune for damping
        .withKS(0.2)   // Amps to overcome static friction — small value
        .withKV(0)     // ZERO — back-EMF handled by FOC commutation
        .withKA(0)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
```

The critical change is **kV = 0**. The kS bump from 0.1 to 0.2 compensates for friction in current terms; adjust empirically.

### 3. Update Drive Gains

For closed-loop drive (auto/path following):

```java
// TorqueCurrentFOC drive gains:
private static final Slot0Configs driveGains =
        new Slot0Configs().withKP(3).withKI(0).withKD(0).withKS(0).withKV(0);
```

Same principle — kV=0 because back-EMF is handled. kP=3 Amps/rotation is a reasonable starting point. Fine-tune with SysId.

### 4. Deploy and Verify Basic Function

Before any detailed tuning:

1. Deploy code
2. Enable the robot, push the stick forward
3. The robot should drive straight without unwanted rotation
4. If it rotates: press Left Bumper (`seedFieldCentric`) to reset the gyro heading and verify the rotation was a one-time heading corruption vs. an ongoing issue

## Tuning the Steer Gains

### On Blocks (Wheels Off Ground)

1. **kP — Stiffness**: Manually twist a module while the robot is enabled. It should snap back to its target angle.
   - Oscillates / buzzes around target: **reduce kP** (try 80, 60, 40)
   - Sluggish return: **increase kP** (try 120, 150)
   - Goal: fast, crisp return with no sustained oscillation

2. **kD — Damping**: Helps prevent overshoot at the cost of response speed.
   - Module overshoots then settles: **increase kD** (try 1.0, 1.5, 2.0)
   - Module feels jerky or noisy: **decrease kD** (try 0.2, 0.1)
   - Goal: one clean motion to target, minimal overshoot

3. **kS — Static Friction**: Overcome friction to start moving.
   - Command a small angle change. If the module hesitates before moving: **increase kS** (try 0.3, 0.5)
   - Typical range: 0.1–0.5A for Krakens through a 21:1 steer ratio

### On the Ground

4. **Drive around** and verify modules track targets accurately. Check module angles in AdvantageScope, Shuffleboard, or Tuner X self-test. All four modules should show minimal position error during driving.

5. **Spin-in-place test**: Command pure rotation. All modules should point tangent to the robot center. If any module lags or overshoots, revisit kP/kD.

6. **Quick direction change test**: Drive forward then quickly command a strafe. The modules need to snap 90 degrees cleanly. This stresses the motion profile and feedback loop together.

## Tuning the Drive Gains (For Auto)

Drive TorqueCurrentFOC gains only matter for closed-loop velocity control (autonomous paths, velocity-mode drive requests). Tune these when you start working on auto:

### Using SysId

1. In `CommandSwerveDrivetrain.java`, set the active routine:
   ```java
   private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;
   ```

2. Start SignalLogger (via Tuner X or code)

3. Run all four SysId tests with the controller bindings:
   - Back + Y: Dynamic forward
   - Back + X: Dynamic reverse
   - Start + Y: Quasistatic forward
   - Start + X: Quasistatic reverse

4. Export the log and open in WPILib SysId tool

5. Fit the data to get kS, kV, kA in **Voltage** terms, then convert:
   - For TorqueCurrentFOC: set kV = 0 (back-EMF handled), keep kS small
   - Use kP from the SysId fit as a starting point, adjusting for Amps units

6. Alternatively, start with kP=3, kV=0, kS=0 and tune kP up until auto paths track accurately.

## Known Pitfalls

- **Do not copy Voltage-mode gains into TorqueCurrentFOC.** The units are fundamentally different. kV is the most dangerous — a Voltage-mode kV of ~2.66 as a current feedforward will cause severe overshoot.

- **MotionMagicExpo profile params cannot be changed via TunerConstants.** They are hardcoded in `SwerveModule.java` as `0.12 * gearRatio` and `1.2 / gearRatio`. To override, you must apply a configuration to the steer motor after construction.

- **Steer current limit matters more with TorqueCurrentFOC.** The 60A stator current limit in `steerInitialConfigs` directly caps the controller output. If kP is too high, you'll saturate at 60A and the response will be nonlinear. For kP=100, saturation happens at 0.6 rotations (216 degrees) of error, which is fine for normal operation.

- **Teleop drive is unaffected by kDriveClosedLoopOutput.** The `DriveRequestType.OpenLoopVoltage` in RobotContainer bypasses closed-loop drive entirely. To use TorqueCurrentFOC for teleop driving, you would change to `DriveRequestType.Velocity`, but this requires tuned drive gains first.

## Reference: Our Configuration

| Parameter | Value |
|---|---|
| Steer motors | Kraken X60 (TalonFX) |
| Steer gear ratio | 21.43:1 |
| Steer feedback | FusedCANcoder |
| Steer current limit | 60A stator |
| Drive gear ratio | 6.75:1 |
| Coupling ratio | 3.57:1 |
| Wheel radius | 2 inches |
| Drivetrain | 19" x 19" (9.5" from center) |
