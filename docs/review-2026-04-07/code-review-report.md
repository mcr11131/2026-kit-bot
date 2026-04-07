# Code Review Report
## Repo: 2026-kit-bot | Date: 2026-04-07

## Summary
- **Overall health: 7/10** — Solid kit bot with good logging, brownout protection, and a clean command structure. The Kraken X60/TalonFX integration is done correctly. Main issues are in the Climb command logic, some dead code, and a few edge cases.
- **Total findings: 18** (2 Critical, 4 High, 7 Medium, 5 Low)

---

## 🔴 Critical Issues

### C1: Climb command has conflicting button logic — second block always wins
**File:** `commands/Climb.java` lines 52-63  
**Severity:** Critical  

The `execute()` method has two independent if/else blocks that both call `setRight()` and `rightstop()`. The second block (buttons 1 & 4) always overwrites the first block (POV Up/Down), so POV controls are effectively dead code — *and worse*, if the driver holds POV Up, the second block immediately calls `rightstop()` because buttons 1 and 4 aren't pressed.

```java
// Block 1: POV controls (output immediately overwritten by Block 2)
if(driverController.povUp().getAsBoolean()) {
  climbSubsystem.setRight(CLIMBER_UP_SPEED);
} else if (driverController.povDown().getAsBoolean()) {
  climbSubsystem.setRight(CLIMBER_DOWN_SPEED);
} else {
  climbSubsystem.rightstop();  // This runs, then Block 2 overwrites it
}

// Block 2: ALWAYS executes after Block 1, overwriting its output
if(driverController.button(1).getAsBoolean()) {
  climbSubsystem.setRight(CLIMBER_UP_SPEED);
} else if (driverController.button(4).getAsBoolean()) {
  climbSubsystem.setRight(CLIMBER_DOWN_SPEED);
} else {
  climbSubsystem.rightstop();  // ← If POV was pressed but btn 1/4 weren't, this stops the motor
}
```

**Fix:** Merge into a single if/else chain:
```java
if (driverController.povUp().getAsBoolean() || driverController.button(1).getAsBoolean()) {
  climbSubsystem.setRight(CLIMBER_UP_SPEED);
} else if (driverController.povDown().getAsBoolean() || driverController.button(4).getAsBoolean()) {
  climbSubsystem.setRight(CLIMBER_DOWN_SPEED);
} else {
  climbSubsystem.rightstop();
}
```

### C2: Button 4 conflict — Drive direction toggle vs Climb down
**File:** `commands/Climb.java` line 59, `commands/Drive.java` line 72  
**Severity:** Critical  

Button 4 (Y button) is used for *both* direction toggle in `Drive` and climb down in `Climb`. Since both commands run simultaneously (Drive is default command on driveSubsystem, Climb is default on climbSubsystem), pressing Y will toggle drive direction AND command the climber down at the same time. This is almost certainly unintended.

**Fix:** Remap one of them. Options:
- Move climb down to a different button (e.g. button 8/Start, or operator controller)
- Move direction toggle to back button (button 7 is already "Reverse Direction" in the controls string but isn't used in code?)

---

## 🟠 High Priority

### H1: LEDSubsystem.configureAlliance() crashes if alliance is empty
**File:** `subsystems/LEDSubsystem.java` line 44  
**Severity:** High  

`DriverStation.getAlliance()` returns `Optional<Alliance>`. Calling `.get()` without `.isPresent()` check throws `NoSuchElementException` when alliance isn't set (practice mode, early connection).

**Fix:**
```java
public void configureAlliance() {
    var alliance = DriverStation.getAlliance();
    LEDPattern alliancePattern;
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
        alliancePattern = red;
    } else if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
        alliancePattern = blue;
    } else {
        alliancePattern = puce;
    }
    defaultColor(alliancePattern);
}
```

### H2: LEDS command calls configureAlliance() in initialize() — runs once then never updates
**File:** `commands/LEDS.java` line 26  
**Severity:** High  

Since LEDS is the default command, `initialize()` runs once when the command starts. If alliance data isn't available yet (common during early boot), the LEDs will show the wrong color and never update. Alliance info often arrives after a few seconds.

**Fix:** Move alliance check to `execute()` with a flag to avoid redundant updates, or call it periodically:
```java
private boolean allianceSet = false;

@Override
public void execute() {
    if (!allianceSet && DriverStation.getAlliance().isPresent()) {
        ledSubsystem.configureAlliance();
        allianceSet = true;
    }
}
```

### H3: TalonFX sticky faults not cleared on feederRoller
**File:** `subsystems/CANFuelSubsystem.java` around line 76  
**Severity:** High  

`intakeLauncherRoller.clearFaults()` is called for the SparkMax, but `feederRoller.clearFaults()` is never called for the TalonFX. Sticky faults from a previous match/brownout can cause unexpected behavior.

**Fix:** Add after the TalonFX config apply:
```java
feederRoller.clearStickyFaults();
```

### H4: SpinUp.end() stops ALL motors — kills launcher mid-LaunchSequence
**File:** `commands/SpinUp.java` line 32  
**Severity:** High  

`SpinUp.end()` calls `fuelSubsystem.stop()` which stops ALL three motors. In `LaunchSequence`, SpinUp runs first (with timeout), then Launch starts. But when SpinUp ends (via timeout), it stops everything *before* Launch's `initialize()` runs. There's a brief dead period where the flywheel decelerates.

This is a known WPILib sequencing behavior — each command in a SequentialCommandGroup has its `end()` called before the next command's `initialize()`.

**Fix:** Have SpinUp.end() only stop what it started, or don't stop the feeder:
```java
@Override
public void end(boolean interrupted) {
    if (interrupted) {
        fuelSubsystem.stop(); // Only full-stop if interrupted, not on normal end
    }
    // Don't stop feeder — Launch will take over immediately
}
```

---

## 🟡 Medium Priority

### M1: Unused import in RobotContainer
**File:** `RobotContainer.java` line 28  
`import java.rmi.dgc.Lease;` — completely unrelated to FRC. Remove it.

### M2: Unused `down` and `limitSwitch` parameters in Climb command
**File:** `commands/Climb.java` lines 22-23  
The constructor takes `boolean down` and `DigitalInput limitSwitch` but neither is ever used in `execute()`. The limit switch should be checked to prevent over-extending the climber.

**Fix:** Either remove the unused params or add limit switch protection:
```java
if (driverController.povDown().getAsBoolean() && limitSwitch.get()) {
  climbSubsystem.setRight(CLIMBER_DOWN_SPEED);
}
```

### M3: ClimberConstants has unused `speed` field that shadows FuelConstants.speed
**File:** `Constants.java` line 91 (ClimberConstants) and line 82 (FuelConstants)  
Both inner classes define `public static final double speed = 1;` — neither is used anywhere. They also shadow each other if both are static-imported. Remove them.

### M4: No voltage compensation on TalonFX (Kraken X60)
**File:** `subsystems/CANFuelSubsystem.java`  
The SparkMax motors all have `voltageCompensation(12)` configured for brownout protection, but the TalonFX feeder config only sets peak voltage limits (which is different). Phoenix 6's voltage compensation is done via `VoltageOut` control mode rather than `DutyCycleOut`.

**Fix:** Consider switching to `VoltageOut` control for consistent behavior, or accept that `DutyCycleOut` with ramp limiting is sufficient. At minimum, document the decision.

### M5: `limelightDiagDone` flag has a logic bug — only sets true inside the table loop
**File:** `Robot.java` lines 81-93  
If no NT table contains "limelight", the flag never gets set to `true`, so the diagnostic scan runs every 20ms forever (minor CPU waste, not critical since it's just string comparisons).

**Fix:** Set the flag after the loop regardless:
```java
if (!limelightDiagDone) {
    // ... scan tables ...
    limelightDiagDone = true;  // Move outside the for loop
}
```

### M6: Tracking command speedToDrive() has no minimum speed
**File:** `commands/Tracking.java` lines 80-92  
The tracking proportional control doesn't have a minimum speed threshold to overcome static friction (unlike TurnToAngle which does). Robot may jitter near the target without settling.

### M7: Commented-out LED code in RobotContainer and ClimbSubsystem
**File:** `RobotContainer.java` lines 68-82, `subsystems/CANClimbSubsystem.java` (multiple blocks)  
Large blocks of commented-out code make the codebase harder to read. Since this is in git, delete the dead code.

---

## 🟢 Low Priority / Suggestions

### L1: Inconsistent naming — "Ball", "Fuel", "Auger", "Launcher", "Feeder", "Flywheel"
Multiple files use different names for the same mechanisms. The class javadoc in CANFuelSubsystem still says "CANBallSubsystem". Standardize terminology.

### L2: SmartDashboard key mismatch risk
Speed constants are published with one name in the subsystem constructor (e.g., `"Intaking feeder speed"`) and read back in commands. If a key is misspelled in either place, it silently falls back to the default. Consider using constants for SD key strings.

### L3: No TalonFX firmware version check
**File:** `subsystems/CANFuelSubsystem.java`  
Phoenix 6 API behavior can change between firmware versions. Consider logging `feederRoller.getVersion()` on init to catch firmware mismatches in the pit.

### L4: HubTracker assumes specific 2026 game mechanics
The HUB shift system is well-implemented, but note: if the actual 2026 game doesn't use this exact shift mechanic, this code is dead weight. Fine for now.

### L5: No unit tests
Zero test files. Consider adding at least subsystem unit tests using WPILib's HAL simulation to catch regressions.

---

## Architecture Notes

**Good stuff:**
- Clean command-based architecture with proper subsystem requirements
- Excellent telemetry — comprehensive DataLog + SmartDashboard coverage
- Brownout protection in the drive subsystem is well thought out
- Encoder health monitoring is a nice defensive feature
- TalonFX/Phoenix 6 integration is solid — current limits, ramp rates, status signals all correct
- The AlignToScore state machine is well-structured with proper edge case handling

**Kraken X60 / TalonFX (CAN 18) specific:**
- Configuration looks correct: 60A stator limit, 40A supply limit, 100ms ramp
- DutyCycleOut control is appropriate for open-loop duty cycle control
- Status signal caching (feederCurrentSignal, etc.) is the right pattern for Phoenix 6
- `.refresh().getValueAsDouble()` pattern in periodic() is correct

**Areas to improve:**
- Button mapping conflicts need resolving before competition
- The climb subsystem is half-removed (climberOne commented out everywhere) — clean it up
- Consider moving SmartDashboard speed tuning to a Preferences table for persistence across deploys

---

## Recommendations

### Must-do (blocks competition readiness)
1. Fix Climb button logic (C1) — POV controls don't work
2. Resolve button 4 conflict between Drive and Climb (C2)
3. Fix LEDSubsystem alliance crash (H1)
4. Clear TalonFX sticky faults (H3)

### Should-do (before first match)
5. Fix SpinUp → Launch motor gap in LaunchSequence (H4)
6. Fix LEDS alliance timing (H2)
7. Add limit switch protection to climber (M2)
8. Clean up commented-out code (M7)

### Nice-to-do (tech debt)
9. Remove unused imports and constants (M1, M3)
10. Standardize mechanism naming (L1)
11. Add firmware version logging (L3)
12. Add unit tests (L5)
