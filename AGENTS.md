# AGENTS.md

Guidance for Claude Code and other AI agents working in this FRC Java repository. Team 4050, 2026 season.

## Build & Tooling

```bash
./gradlew build              # Compile + spotless format check
./gradlew simulateJava       # Run WPILib simulation with GUI
./gradlew spotlessApply      # Auto-format code (Eclipse Java Format)
```

Spotless auto-formatting runs before compilation. A git pre-commit hook enforces formatting via `installGitHook`. Always run `spotlessApply` before committing if the build fails on formatting.

## Robot Lifecycle

```
Main.java
  -> RobotBase.startRobot(Robot::new)
      -> Robot constructor
          1. configureLogging()      Set up Epilogue, DataLogManager
          2. configureLimeLight()    Vision camera setup
          3. new RobotContainer()
              a. Instantiate subsystems (as fields)
              b. initRobotState()    Register subsystems with RobotState singleton
              c. configureBindings() Wire triggers to commands
```

### TimedRobot Periodic Loop (20ms)

Every cycle, `robotPeriodic()` runs regardless of mode:
1. `robotState.periodic()` -- updates telemetry, vision, match data
2. `CommandScheduler.getInstance().run()` -- ticks all active commands

Mode-specific callbacks (`autonomousInit`, `teleopInit`, etc.) handle mode transitions. The scheduler does the actual work -- keep mode callbacks minimal.

### Mode Transitions

- **Disabled -> Autonomous**: `autonomousInit()` fetches and schedules the auto command from `RobotContainer.getAutonomousCommand()`.
- **Autonomous -> Teleop**: `teleopInit()` cancels any running auto command. Default commands resume automatically.
- **Any -> Test**: `testInit()` cancels all commands for isolated testing.
- **Any -> Disabled**: Subsystems continue running `periodic()` but commands are canceled.

## Subsystem Conventions

### Structure

Every subsystem:
- Extends `SubsystemBase` (auto-registers with CommandScheduler)
- Declares hardware (motors, sensors, encoders) as **private fields**
- Exposes actions via **public command factory methods**, never raw motor access
- Uses `periodic()` only for housekeeping (odometry, telemetry caching) -- not command logic

```java
public class ExampleSubsystem extends SubsystemBase {
    // Private hardware
    private final SparkMax motor;
    private final RelativeEncoder encoder;

    // Configuration constants as static final fields
    private static final double MAX_POSITION = 54.5;

    public ExampleSubsystem(int motorId) {
        motor = new SparkMax(motorId, MotorType.kBrushless);
        encoder = motor.getEncoder();
        // Configure motor on construction
    }

    // Private action methods (called by commands)
    private void up() { /* ... */ }
    private void stop() { motor.stopMotor(); }

    // Public command factories
    public Command upCommand() {
        return startEnd(this::up, this::stop).withName("Example: Up");
    }

    // Public telemetry getters
    public double getPosition() { return encoder.getPosition(); }
}
```

### Command Factory Naming

Command factory methods on subsystems follow the pattern `<action>Command()`:
- `upCommand()`, `downCommand()`, `stopCommand()`
- `deployOverrideCommand()`, `toggleDeployCommand()`
- `inCommand()`, `outCommand()`

Always call `.withName("Subsystem: Action")` on returned commands for dashboard/log readability.

### Motor Configuration

**REV SparkMax** (used for mechanisms):
```java
var config = new SparkMaxConfig();
config.smartCurrentLimit(40);
config.idleMode(IdleMode.kBrake);
// ... additional config
var result = motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
if (result != REVLibError.kOk) {
    DriverStation.reportWarning("Motor config failed: " + result, false);
}
```

**CTRE TalonFX** (used for swerve drive): Configured via `TunerConstants.java`. Do not manually configure swerve motors.

### The Homeable Interface

For mechanisms that home via stall detection, implement `Homeable`:
```java
public interface Homeable {
    void driveToHome();          // Run motor toward home position
    void stopHoming();           // Stop the homing motor
    double getHomingCurrent();   // Current draw (stall detection)
    double getHomingVelocity();  // Velocity (near-zero = stalled)
    void onHomeComplete();       // Reset encoder, set known position
}
```

The reusable `Home` command works with any `Homeable` subsystem, detecting stall conditions over time.

## Command Conventions

### Creation Patterns (prefer top to bottom)

1. **Subsystem factory methods** -- for single-subsystem actions:
   ```java
   intake.inCommand()  // Returns startEnd(...) with requirements
   ```

2. **Inline lambda commands** -- for bindings:
   ```java
   runOnce(() -> isRobotCentric.set(true))
   new RunCommand(this::shoot, subsystem1, subsystem2)
   ```

3. **Static factory classes** -- for multi-subsystem orchestration:
   ```java
   // Unload.java provides factory methods, not a Command subclass
   Unload.primeCommand(leftUnloader, rightUnloader)
   ```

4. **Custom Command subclasses** -- only when internal state is needed:
   ```java
   // Home.java tracks timers, current, velocity for stall detection
   public class Home extends Command { ... }
   ```

### Command Lifecycle

| Method | When | Purpose |
|---|---|---|
| `initialize()` | Once on schedule | Set starting state, reset timers |
| `execute()` | Every 20ms | Continuous work |
| `isFinished()` | Every cycle | Return `true` to end naturally |
| `end(interrupted)` | Once on end | Cleanup; `interrupted` flag tells you why |

### Requirements

Every command that uses a subsystem must declare it via `addRequirements()` (in constructor) or by passing the subsystem to factory methods like `new RunCommand(action, subsystem)`. The scheduler uses requirements to prevent conflicting commands.

### Composition Rules

- Once a command is passed to a composition (`sequence`, `parallel`, `race`, `deadline`), it **cannot be reused** elsewhere. Doing so throws at runtime.
- Compositions inherit the union of all member subsystem requirements.
- Use `Commands.defer(() -> ..., Set.of(subsystem))` for commands that must be constructed at runtime (e.g., chooser-selected routines).

### Trigger Bindings

All bindings live in `RobotContainer.configureBindings()`:

| Method | Behavior |
|---|---|
| `.onTrue(cmd)` | Schedule on rising edge (false -> true) |
| `.whileTrue(cmd)` | Schedule on rising edge, cancel when false |
| `.toggleOnTrue(cmd)` | Toggle command on each rising edge |

Triggers support `.and()`, `.or()`, `.negate()` for composition. Use `.debounce(seconds)` to prevent rapid activation.

## WPILib Units Library

Use `edu.wpi.first.units.Units` for all physical quantities. This prevents unit conversion errors and makes intent explicit.

```java
import static edu.wpi.first.units.Units.*;
```

### Creating Measures

```java
// Constants (immutable, allocation is fine)
LinearVelocity maxSpeed = MetersPerSecond.of(4.58);
Distance wheelRadius = Inches.of(2);
Angle deployAngle = Degrees.of(181);
```

### Converting to Raw Doubles

Use `.in()` when interfacing with APIs that take doubles:
```java
double speedMps = maxSpeed.in(MetersPerSecond);
double radiansPerSec = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
```

### Type-Safe Arithmetic

The Units library enforces dimensional correctness at compile time:
```java
Distance / Time -> LinearVelocity     // Valid
Distance + Angle                      // Compile error
```

### MutableMeasure for Periodic Code

Default arithmetic creates new objects each call. In 20ms loops, use `MutableMeasure` to avoid GC pressure:

```java
private final MutDistance position = Meters.mutable(0);

@Override
public void periodic() {
    position.mut_replace(encoder.getPosition(), Meters);
    // Use position without allocating
}
```

Mutation methods: `mut_plus()`, `mut_minus()`, `mut_times()`, `mut_divide()`, `mut_replace()`.

**Warning**: Mutable objects change everywhere they're referenced. Never store a reference for later comparison without `.copy()`.

### Where to Use Units

- **Constants**: Always use Units for physical quantities (speeds, distances, angles, currents).
- **Public APIs**: Subsystem getters and command parameters should use `Measure` types when practical.
- **Internal control loops**: Convert to doubles at the boundary with vendor APIs (SparkMax, TalonFX) that require raw values.
- **SysId routines**: Use `Volts.of()`, `Second`, etc. for characterization.

### Common Unit Conversions

```java
// Angle conversions
double radians = Units.degreesToRadians(degrees);
Angle angle = Degrees.of(90);
double rads = angle.in(Radians);

// Speed conversions
double maxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
double maxLinearSpeed = maxSpeed.in(MetersPerSecond);

// WPILib math uses SI internally -- always convert at the boundary
```

## Telemetry

### Epilogue (Primary)

Annotation-based logging that auto-publishes to NetworkTables and `.wpilog`:

```java
@Logged(defaultNaming = Logged.Naming.USE_HUMAN_NAME)
public class Robot extends TimedRobot { ... }
```

- `@Logged` on a class: logs all fields and public no-arg methods.
- `@NotLogged`: excludes a field/method.
- Minimum importance set to `DEBUG` in this project.
- Supports primitives, Strings, `StructSerializable`, `Measure`, `Sendable`, suppliers.

**Performance**: Cache CAN device reads in `periodic()` rather than querying the same device in multiple logged methods. Monitor CPU time at `/Epilogue/Stats/Last Run`.

### SmartDashboard

Used only for types Epilogue cannot handle: `Field2d`, `SendableChooser`, `CommandScheduler`, and custom Elastic widgets. Prefer Epilogue for everything else.

### DataLogManager

Mirrors all NetworkTables to `.wpilog` on disk. Configured once in `Robot` constructor.

### Elastic Dashboard

Layout stored in `src/main/deploy/dashboard.json`. Notifications sent via `Elastic.sendAlert()` utility.

### Disabled Vendor Logging

Both `REV StatusLogger` and `CTRE SignalLogger` are disabled in favor of Epilogue.

## RobotState (Singleton Telemetry Hub)

`RobotState.getInstance()` is the central hub for cross-cutting state:

- Subsystems register via `addDrivetrain()`, `addClimber()`, etc.
- `periodic()` runs every cycle: updates odometry, match data, vision measurements.
- Exposes getters: `getChassisPose()`, `getIntakeCurrent()`, `isOurScoringPeriod()`, etc.
- Publishes Field2d and swerve visualization (dev mode only).
- Null-safe: getters return `0.0` / defaults if subsystem not registered.

## Constants & Configuration

All configuration lives in `Constants.java`, organized by nested class:

```java
public final class Constants {
    public static final boolean DEV_MODE = ...;  // true unless on main or event/* branch

    public static final class Drivetrain { /* CAN IDs */ }
    public static final class Subsystems { /* CAN IDs */ }
    public static final class Vision { /* camera config */ }
}
```

### CAN ID Scheme

- **Drivetrain**: `<module><device>` -- module 1-4 (FL, BL, BR, FR), device 1-3 (drive, steer, encoder). Example: `11` = FL drive, `43` = FR encoder.
- **Other subsystems**: 50s range. Example: `51` = intake roller, `53` = climber leader.
- **Gyro**: Pigeon2 on CAN ID `10`.
- **CANivore bus**: Named `"Drivetrain"` for all swerve devices.

### DEV_MODE

`Constants.DEV_MODE` is `true` on any branch that isn't `main` or `event/*`. Use it to gate expensive dashboard widgets, extra logging, or diagnostic features that shouldn't run at competition.

## Swerve Drive Architecture

Two-file design built on CTRE Phoenix 6:

- **`TunerConstants.java`** (generated by Tuner X, then customized): Swerve hardware config -- motor types, sensor types, PID gains, gear ratios, module positions, CAN IDs. Defines `TunerSwerveDrivetrain` inner class. **Do not regenerate without reviewing diffs carefully.**
- **`Drivetrain.java`** (subsystem): Extends `TunerSwerveDrivetrain`, implements `Subsystem`. Adds command integration, SysId routines, operator perspective handling (alliance-based field orientation), and vision measurement injection.

### Simulation

Drivetrain runs a 4ms physics loop via `Notifier` (faster than the 20ms robot loop for realistic PID behavior). Other subsystems rely on SparkMax's built-in simulation.

## Vendor Libraries

Managed as JSON files in `vendordeps/`. Each must match the WPILib year.

| Library | Purpose | Hardware |
|---|---|---|
| Phoenix6 | Swerve drive framework | TalonFX, CANcoder, Pigeon2 |
| REVLib | Mechanism motor control | SparkMax, NEO, encoders |
| WPILibNewCommands | Command-based framework | -- |

## Documentation Sources

**Never read or inspect dependency JAR files directly.** Use online JavaDoc via WebFetch:

- **WPILib**: https://github.wpilib.org/allwpilib/docs/release/java/index.html
- **CTRE Phoenix 6**: https://api.ctr-electronics.com/phoenix6/stable/java/
- **REV Robotics**: https://codedocs.revrobotics.com/java/
- **Limelight**: https://limelightlib-wpijava-reference.limelightvision.io/frc/robot/package-summary.html

To look up a class, navigate to its package path on the relevant site (e.g., `com.ctre.phoenix6.swerve.SwerveDrivetrain` -> `https://api.ctr-electronics.com/phoenix6/stable/java/com/ctre/phoenix6/swerve/SwerveDrivetrain.html`).

## Key Rules for AI Agents

1. **Do not edit generated files** (`TunerConstants.java`, `BuildConstants.java`) unless explicitly asked.
2. **Run `./gradlew spotlessApply`** if you modify Java files, or the pre-commit hook will reject.
3. **Use the Units library** for new physical quantities. Do not introduce raw numeric conversions where Units can be used.
4. **Name all commands** with `.withName("Subsystem: Action")`.
5. **Declare requirements** on every command that touches a subsystem.
6. **Prefer command factories on subsystems** over standalone command classes.
7. **Keep subsystem internals private**. Expose actions as commands, state as getters.
8. **Cache CAN reads** in `periodic()`. Never query the same CAN device multiple times per cycle.
9. **Constants go in `Constants.java`**, organized in the appropriate nested class.
10. **Test with `./gradlew build`** before considering a change complete.
