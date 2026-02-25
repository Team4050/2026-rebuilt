# Telemetry Guide

How telemetry works in this project, how to add telemetry to a new subsystem, and how to use `RobotState`. Also serves as a reference for WPILib's Epilogue library.

## Overview

Telemetry flows through a tiered system:

1. **Epilogue** (primary) — Automatic logging via `@Logged` annotations. Publishes to NetworkTables (which DataLogManager then records to disk).
2. **SmartDashboard** (secondary) — Used only for complex types Epilogue can't handle (`Field2d`, `SendableChooser`, custom swerve widget).
3. **DataLogManager** — Records all NetworkTables to `.wpilog` files on disk for post-match analysis.
4. **Elastic Dashboard** — Frontend that displays the data. Layout is in `src/main/deploy/dashboard.json`.

Subsystems do **not** log data directly. Instead, they expose getter methods, and `RobotState` wraps those methods with `@Logged` annotations. This centralizes all telemetry in one place.

## Adding Telemetry to a New Subsystem

### Step 1: Write getter methods in your subsystem

Your subsystem just needs public getter methods that return the data you want to log. No annotations needed on the subsystem itself.

```java
public class Elevator extends SubsystemBase {
  private final SparkMax motor = new SparkMax(Constants.Subsystems.elevatorId, MotorType.kBrushless);
  private final RelativeEncoder encoder = motor.getEncoder();

  // Action methods...

  // Getter methods for telemetry — RobotState will call these
  public double getPosition() {
    return encoder.getPosition();
  }

  public double getVelocity() {
    return encoder.getVelocity();
  }
}
```

### Step 2: Register the subsystem in RobotState

Add a section to `RobotState.java` with:

- A private field to hold the subsystem reference
- An `addXxx()` registration method
- Public getter methods that delegate to the subsystem (automatically logged by the class-level `@Logged`)

```java
// ===================== Elevator =====================

private Elevator elevator;

public void addElevator(Elevator elevator) {
  this.elevator = elevator;
}

public double getElevatorPosition() {
  if (elevator == null) {
    return 0.0;
  }
  return elevator.getPosition();
}

public double getElevatorVelocity() {
  if (elevator == null) {
    return 0.0;
  }
  return elevator.getVelocity();
}
```

Since `RobotState` has class-level `@Logged` with `USE_HUMAN_NAME` naming (OPT_OUT strategy), all public no-arg methods are logged automatically — no per-method `@Logged` needed. The `get` prefix is stripped and camelCase is split into words (e.g., `getElevatorPosition` → `Elevator Position`).

### Step 3: Register in RobotContainer

In `RobotContainer.initRobotState()`, call your new registration method:

```java
private void initRobotState() {
  RobotState rs = RobotState.getInstance();
  rs.addDrivetrain(drivetrain);
  rs.addIntake(intakeSub);
  rs.addClimber(climber);
  rs.addElevator(elevator);  // <-- add this
}
```

That's it. Epilogue discovers `RobotState` through `Robot` (which calls `Epilogue.bind(this)`) and automatically logs every `@Logged` field and method.

## RobotState Reference

`RobotState` is a singleton that serves two purposes:

1. **Telemetry hub** — Centralizes all `@Logged` data so Epilogue publishes it under one namespace (`/Robot/Robot State/`).
2. **Cross-cutting state** — Provides shared data that multiple subsystems or commands might need (match period, vision estimates, scoring shift status).

### Accessing RobotState

```java
RobotState rs = RobotState.getInstance();
```

Available from anywhere — subsystems, commands, or `RobotContainer`.

### Periodic updates

`RobotState.periodic()` is called every robot cycle (20ms) from `Robot.robotPeriodic()`. It:

- Updates match time and alliance scoring shift status
- Polls the Limelight for vision pose estimates and feeds them to the drivetrain's pose estimator

### SmartDashboard (when Epilogue isn't enough)

Some types can't be serialized by Epilogue. Use `SmartDashboard.putData()` for these:

- `Field2d` — field visualization widget
- `SendableChooser<Command>` — dashboard dropdown selectors
- Custom `Sendable` lambdas — e.g., the swerve drive widget for Elastic
- `CommandScheduler` — shows active commands

These are set up in `RobotState.addDrivetrain()` and `RobotContainer.configureBindings()`.

## Epilogue Reference

Epilogue is WPILib's annotation-based logging framework. You annotate classes, fields, and methods, and the annotation processor generates logger classes at compile time (`build/generated/sources/annotationProcessor/`). At runtime, `Epilogue.bind()` hooks these loggers into the robot's periodic loop.

### How it works

1. **Compile time**: The annotation processor scans `@Logged` classes and generates a `ClassSpecificLogger<T>` subclass for each (e.g., `RobotStateLogger`). It also generates a top-level `Epilogue` class with static logger instances and configuration methods.
2. **Runtime**: `Epilogue.bind(this)` (called from `Robot`'s constructor) registers a periodic callback that invokes `Epilogue.update(robot)` every cycle.
3. **Object graph traversal**: The generated `RobotLogger` logs all fields on `Robot`. For each field that is itself a `@Logged`-annotated type (like `robotState`), it delegates to that type's generated logger via `backend.getNested("Robot State")`. This recurses through the whole object graph.
4. **Phase offset**: Logging runs offset by half the robot period (10ms by default), spreading CPU load between the main loop and logging.

Private fields are accessed via `VarHandle` (reflection) in the generated code. Public methods are called directly.

### Supported types

| Category         | Types                                                                                                                                |
| ---------------- | ------------------------------------------------------------------------------------------------------------------------------------ |
| Primitives       | `byte`, `char`, `short`, `int`, `long`, `float`, `double`, `boolean`                                                                 |
| Strings          | `String`                                                                                                                             |
| Primitive arrays | `byte[]`, `int[]`, `long[]`, `float[]`, `double[]`, `boolean[]`                                                                      |
| String arrays    | `String[]`                                                                                                                           |
| Struct types     | Any `StructSerializable` — all WPILib geometry/kinematics types (`Pose2d`, `Rotation2d`, `ChassisSpeeds`, `SwerveModuleState`, etc.) |
| Struct arrays    | e.g., `SwerveModuleState[]`, `Rotation2d[]`                                                                                          |
| Collections      | `List<String>`, `Set<String>`, `List<StructSerializable>`, `Set<StructSerializable>`                                                 |
| Measure (units)  | `Measure<T>` — logged as a double in the base unit (e.g., meters, radians/s, seconds)                                                |
| Sendable         | Any `Sendable` object (excluding Commands and Subsystems)                                                                            |
| Suppliers        | `BooleanSupplier`, `IntSupplier`, `LongSupplier`, `DoubleSupplier` (including `Trigger`)                                             |
| Nested objects   | Any class/interface/enum annotated with `@Logged`                                                                                    |

**Not supported**: wrapper types (`Integer`, `Boolean`), static fields/methods, non-public methods, methods with parameters, `Mechanism2D`.

### `@Logged` annotation

Can be placed on a **class**, **field**, or **method**.

**Parameters:**

| Parameter       | Default              | Description                                                                                        |
| --------------- | -------------------- | -------------------------------------------------------------------------------------------------- |
| `name`          | `""` (use code name) | Custom name for the logged entry. Overrides auto-generated naming.                                 |
| `importance`    | `DEBUG`              | Importance level: `DEBUG`, `INFO`, or `CRITICAL`. Data below the configured minimum is skipped.    |
| `strategy`      | `OPT_OUT`            | Class-level only. `OPT_OUT` logs everything unless excluded; `OPT_IN` logs only annotated members. |
| `defaultNaming` | `USE_CODE_NAME`      | Class-level only. How auto-generated names are derived (see Naming below).                         |

#### Class-level `@Logged` (OPT_OUT — our default)

When placed on a class, Epilogue logs:

- **All fields** (public or private) with a loggable type
- **All public methods** that take no arguments and return a loggable type
- Static fields/methods are always excluded

Use `@NotLogged` to exclude specific fields or methods.

```java
@Logged
public class MySubsystemState {
  private double position;          // logged (private field — still logged)
  private Rotation2d angle;         // logged (StructSerializable)
  @NotLogged
  private boolean internalFlag;     // excluded

  public double getVelocity() { }  // logged (public, no args, returns double)
  private double helper() { }      // NOT logged (non-public)
  public void setSpeed(double s) {} // NOT logged (has parameters)
}
```

#### Field/method-level `@Logged` (OPT_IN)

When the class itself is NOT annotated, selectively annotate individual members:

```java
public class Arm {
  @Logged
  public double getAngle() { return encoder.getPosition(); }

  public double getDebugValue() { }  // not logged (no annotation)
}
```

### Naming

The `defaultNaming` parameter on class-level `@Logged` controls how NetworkTables keys are derived:

| Naming                    | Field `m_position` | Method `getAngle()` |
| ------------------------- | ------------------ | ------------------- |
| `USE_CODE_NAME` (default) | `m_position`       | `getAngle`          |
| `USE_HUMAN_NAME`          | `Position`         | `Angle`             |

With `USE_HUMAN_NAME`, `m_` prefixes are stripped from fields, `get` prefixes are stripped from methods, camelCase is split into words, and the first letter of each word is capitalized (e.g., `visionTagCount` → `Vision Tag Count`, `getChassisPose` → `Chassis Pose`). The `is` prefix is kept (e.g., `isOurScoringPeriod` → `Is Our Scoring Period`). **Caution**: if a field and method resolve to the same human name, compilation fails.

A custom `name` parameter always takes precedence: `@Logged(name = "Arm Angle")`.

**This project uses `USE_HUMAN_NAME`**, so `get` prefixes are stripped and camelCase is split into words (e.g., `getChassisPose` → `Chassis Pose`, `visionTagCount` → `Vision Tag Count`).

### Importance levels

| Level      | Use for                                               |
| ---------- | ----------------------------------------------------- |
| `DEBUG`    | Raw sensor values, internal state. Default level.     |
| `INFO`     | Higher-level data (pose estimates, subsystem state).  |
| `CRITICAL` | Essential data that should always be present in logs. |

Set per-member: `@Logged(importance = Logged.Importance.CRITICAL)`

Filtered at runtime via `config.minimumImportance`. Data below the threshold is not logged.

### `@NotLogged`

Excludes a field or method from logging, overriding the class-level `@Logged`:

```java
@Logged
public class RobotState {
  @NotLogged
  private boolean needGameDataCheck;  // excluded from telemetry
}
```

Redundant on private methods (they're never logged anyway), but fine for documentation clarity.

### Configuration

Configured in `Robot.configureLogging()` before `Epilogue.bind(this)`:

```java
Epilogue.configure(config -> {
  config.minimumImportance = Logged.Importance.DEBUG;
  config.errorHandler = ErrorHandler.crashOnError();  // in simulation
});
Epilogue.bind(this);
```

| Option              | Type                | Default             | Description                                        |
| ------------------- | ------------------- | ------------------- | -------------------------------------------------- |
| `backend`           | `EpilogueBackend`   | `NTEpilogueBackend` | Where data is written.                             |
| `root`              | `String`            | `"Robot"`           | Root NT path. All data appears under `/{root}/...` |
| `minimumImportance` | `Logged.Importance` | `DEBUG`             | Minimum importance level to log.                   |
| `errorHandler`      | `ErrorHandler`      | Prints to stdout    | How errors during logging are handled.             |

#### Backends

- **`NTEpilogueBackend`** (default) — Publishes to NetworkTables. Data is visible to dashboards over the network, and DataLogManager mirrors it to disk.
- **`FileBackend`** — Writes directly to a DataLog file, bypassing NetworkTables. Use when you don't need live dashboard display: `new FileBackend(DataLogManager.getLog())`.

Our project uses the default NT backend so data is both visible on dashboards and recorded to disk via DataLogManager.

#### Error handlers

| Handler                                       | Behavior                                                                                                   |
| --------------------------------------------- | ---------------------------------------------------------------------------------------------------------- |
| `ErrorHandler.printErrorMessages()` (default) | Prints errors to console, logging continues.                                                               |
| `ErrorHandler.crashOnError()`                 | Re-throws exceptions. Use in simulation to catch bugs early.                                               |
| `ErrorHandler.disabling(int maxErrors)`       | Disables individual loggers after exceeding the error threshold. Good for competition to prevent log spam. |

Our project uses `crashOnError()` in simulation and the default (print) on the real robot.

### Custom loggers

For third-party classes you can't annotate (vendor libraries), create a custom logger:

```java
@CustomLoggerFor(VendorMotor.class)
public class VendorMotorLogger extends ClassSpecificLogger<VendorMotor> {
  public VendorMotorLogger() {
    super(VendorMotor.class);
  }

  @Override
  public void update(EpilogueBackend backend, VendorMotor motor) {
    if (Epilogue.shouldLog(Logged.Importance.DEBUG)) {
      backend.log("Faults", motor.getFaults());
    }
    backend.log("Voltage", motor.getAppliedVoltage());
  }
}
```

Rules: one custom logger per type, must have a public no-arg constructor, must be in the robot project (not a dependency).

### Performance monitoring

Epilogue logs its own execution time to `/Epilogue/Stats/Last Run` (in milliseconds). Watch this value if you suspect logging is consuming too much CPU — especially when querying CAN devices frequently.

### Common pitfalls

- **Null fields**: A `null` field with a loggable type causes a `NullPointerException` (handled by the configured error handler). Initialize fields or use `@NotLogged`.
- **Expensive methods**: Epilogue calls methods every cycle. Avoid doing heavy work in getter methods — cache values in `periodic()` if needed.
- **Wrapper types**: `Integer`, `Boolean`, etc. are not supported. Use primitives.
- **Runtime polymorphism**: If a field is declared as type `T`, Epilogue logs it as `T` regardless of the runtime subclass.
