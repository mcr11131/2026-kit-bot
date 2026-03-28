# POWER MANAGEMENT ARCHITECTURE ANALYSIS
**2026 Robot Codebase Brownout Prevention Assessment**

## Executive Summary

The current robot codebase has **significant architectural gaps** in power management that create high risk for brownout conditions during competition. While the drive subsystem implements basic brownout protection, other subsystems lack coordination and voltage-aware scaling, creating scenarios where multiple high-current systems can simultaneously drain the battery below operational thresholds.

**Critical Risk Level:** ⚠️ **HIGH** - Brownout conditions likely during simultaneous multi-subsystem operations

---

## Current Power Architecture Assessment

### ✅ Implemented Power Management Features

1. **Drive Subsystem Brownout Protection**
   - Voltage-based motor output scaling (`getBrownoutScale()`)
   - Brownout thresholds: 7.5V min → 9.5V recovery
   - Minimum power scale: 45% during low voltage conditions
   - Hardware brownout detection via `RobotController.isBrownedOut()`

2. **Current Limiting**
   - All NEO motors configured with 40A smart current limits
   - Secondary current limits implemented on drive motors
   - Motor temperature monitoring on drive and fuel systems

3. **Power Monitoring Infrastructure**
   - Comprehensive current, voltage, and power logging (drive subsystem)
   - PowerDistribution integration for total current monitoring
   - SmartDashboard integration for real-time power metrics

### ❌ Missing Critical Features

1. **Subsystem Power Coordination Layer**
   - No centralized power manager
   - Subsystems operate independently without awareness of system-wide power state
   - No inter-subsystem communication about power consumption

2. **Inconsistent Brownout Protection**
   - **Fuel Subsystem:** No voltage-based scaling - continues full power during low voltage
   - **Climber Subsystem:** No brownout protection whatsoever
   - **LED Subsystem:** Not analyzed for power consumption impact

3. **Resource Arbitration System**
   - No priority hierarchy for critical vs. non-critical operations
   - No mechanism to shed non-essential loads during power constraints
   - No coordinated power allocation during simultaneous operations

---

## Brownout Risk Factors Identified

### 🔴 Critical Risk Areas

1. **Simultaneous Multi-Subsystem Operations**
   ```java
   // AlignToScore.java - Runs drive + fuel simultaneously
   driveSubsystem.driveArcade(driveSpeed, rotationSpeed);
   fuelSubsystem.setIntakeLauncherRoller(LAUNCHING_AUGER_SPEED);
   fuelSubsystem.setFeederRoller(LAUNCHING_FLYWHEEL_SPEED);
   ```
   - **Risk:** Drive system reduces power due to voltage drop, but fuel system maintains full power
   - **Impact:** Cascading voltage drop leading to brownout

2. **Unprotected High-Draw Operations**
   - **Climbing:** High-torque operations with no voltage awareness
   - **Launch Sequence:** Multiple fuel motors at full power regardless of voltage
   - **Intake Operations:** Can run simultaneously with other systems

3. **Power State Ignorance**
   ```java
   // CANFuelSubsystem.java - No brownout consideration
   public void setIntakeLauncherRoller(double speed) {
       intakeLauncherRoller.set(speed); // Always full requested power
   }
   ```

### 🟡 Medium Risk Areas

1. **Current Limit Independence**
   - Each subsystem sets 40A limits independently
   - No coordination of total system current draw
   - Potential for 160A+ total draw (4 drive + 3 fuel + 1 climber motors)

2. **Inadequate Monitoring**
   - Climber subsystem: No current/voltage monitoring in periodic()
   - Missing total power consumption awareness in command layer

---

## Missing Power Management Patterns

### 1. Centralized Power Management Architecture

**Missing:** Power Manager singleton class responsible for:
- System-wide voltage monitoring
- Current budget allocation
- Brownout condition coordination
- Power state broadcasting to subsystems

**Current State:** Each subsystem makes independent power decisions

### 2. Voltage-Aware Scaling Framework

**Missing:** Common interface for voltage-based output scaling
```java
// Recommended pattern (not implemented):
public interface VoltageAware {
    void updateVoltageScale(double voltageScale);
    double getCurrentDraw();
    PowerPriority getPriority();
}
```

**Current State:** Only drive subsystem implements voltage scaling

### 3. Power Priority System

**Missing:** Hierarchical power allocation during constraints
- **Critical:** Safety systems, basic drive
- **High:** Core competition functions
- **Medium:** Enhanced features
- **Low:** Cosmetic functions (LEDs, etc.)

**Current State:** All systems treated equally

### 4. Smart Load Shedding

**Missing:** Automatic reduction of non-critical loads during power stress
- LED dimming/disabling during high-draw operations
- Reduced auxiliary motor speeds
- Deferred non-critical operations

### 5. Operation Sequencing

**Missing:** Intelligent scheduling of high-draw operations
- Avoid simultaneous acceleration of all systems
- Stagger motor startups to reduce inrush current
- Coordinate high-torque operations

---

## Recommended Architectural Changes

### 🔴 Critical Priority Fixes

1. **Implement Universal Brownout Protection**
   ```java
   // Add to CANFuelSubsystem and CANClimbSubsystem
   private double getBrownoutScale() {
       // Copy from CANDriveSubsystem implementation
   }
   ```
   - **Timeline:** Immediate (can be done in 1-2 hours)
   - **Impact:** Prevents uncoordinated full-power operations during voltage drops

2. **Create PowerManager Singleton**
   ```java
   public class PowerManager {
       private static double systemVoltageScale = 1.0;
       
       public static void updateSystemPowerState() {
           // Centralized brownout calculation
           // Broadcast to all subsystems
       }
   }
   ```
   - **Timeline:** 1 day development + testing
   - **Impact:** Coordinated system-wide power management

3. **Add Power Monitoring to All Subsystems**
   - Extend fuel and climber subsystems with current/voltage logging
   - **Timeline:** 2-4 hours
   - **Impact:** Visibility into actual power consumption patterns

### 🟡 High Priority Enhancements

4. **Implement Power Priority System**
   ```java
   public enum PowerPriority { CRITICAL, HIGH, MEDIUM, LOW }
   ```
   - Define priority levels for each subsystem/operation
   - Implement priority-based power scaling
   - **Timeline:** 2-3 days
   - **Impact:** Intelligent power allocation during constraints

5. **Add Operation Coordination**
   - Modify commands to check system power state before high-draw operations
   - Implement "power-aware" command decorators
   - **Timeline:** 1-2 days per major command
   - **Impact:** Prevents simultaneous high-current scenarios

6. **Implement Smart Current Budgeting**
   - Total system current budget management
   - Per-subsystem current allocation based on priority
   - **Timeline:** 3-4 days
   - **Impact:** Proactive brownout prevention

### 🟢 Medium Priority Optimizations

7. **Advanced Load Shedding**
   - Automatic LED dimming during high power operations  
   - Non-essential motor speed reduction
   - **Timeline:** 1-2 days
   - **Impact:** Extended operational capability under power stress

8. **Predictive Power Management**
   - Battery state estimation
   - Proactive power scaling based on remaining capacity
   - **Timeline:** 1 week
   - **Impact:** Consistent performance throughout match

---

## Implementation Priority Matrix

| Fix | Impact | Effort | Timeline | Priority |
|-----|---------|---------|-----------|-----------|
| Universal brownout protection | High | Low | 2 hours | **Critical** |
| Power monitoring expansion | Medium | Low | 4 hours | **Critical** |
| PowerManager singleton | High | Medium | 1 day | **Critical** |
| Command power coordination | High | Medium | 2-3 days | **High** |
| Power priority system | Medium | High | 3-4 days | **High** |
| Current budgeting | High | High | 1 week | **High** |
| Smart load shedding | Medium | Medium | 2 days | **Medium** |
| Predictive management | Low | High | 1 week | **Medium** |

---

## Specific Code Changes Required

### 1. CANFuelSubsystem.java - Add Brownout Protection
```java
private double getBrownoutScale() {
    if (RobotController.isBrownedOut()) {
        return FUEL_BROWNOUT_MIN_SCALE; // Add to Constants
    }
    double normalizedVoltage = MathUtil.clamp(
        (RobotController.getBatteryVoltage() - FUEL_BROWNOUT_MIN_VOLTAGE)
        / (FUEL_BROWNOUT_RECOVERY_VOLTAGE - FUEL_BROWNOUT_MIN_VOLTAGE),
        0.0, 1.0);
    return FUEL_BROWNOUT_MIN_SCALE + normalizedVoltage * (1.0 - FUEL_BROWNOUT_MIN_SCALE);
}

public void setIntakeLauncherRoller(double speed) {
    double scale = getBrownoutScale();
    intakeLauncherRoller.set(speed * scale);
}
```

### 2. CANClimbSubsystem.java - Add Power Management
```java
// Add current/voltage monitoring to periodic()
@Override
public void periodic() {
    double current = climberTwo.getOutputCurrent();
    double voltage = climberTwo.getAppliedOutput() * climberTwo.getBusVoltage();
    double temp = climberTwo.getMotorTemperature();
    
    SmartDashboard.putNumber("Climb/Current", current);
    SmartDashboard.putNumber("Climb/Voltage", voltage);
    SmartDashboard.putNumber("Climb/Temperature", temp);
}

private double getBrownoutScale() {
    // Implement brownout scaling similar to drive subsystem
}
```

### 3. AlignToScore.java - Add Power Coordination
```java
@Override
public void execute() {
    // Check system power state before simultaneous operations
    double systemVoltage = RobotController.getBatteryVoltage();
    boolean allowSimultaneous = systemVoltage > SIMULTANEOUS_OPERATION_MIN_VOLTAGE;
    
    if (allowSimultaneous) {
        // Normal operation - drive + fuel
        driveSubsystem.driveArcade(driveSpeed, rotationSpeed);
        // Continue with fuel operations...
    } else {
        // Power-constrained mode - prioritize alignment over fuel
        if (aligned) {
            driveSubsystem.driveArcade(0, 0);
            // Only then run fuel systems
        } else {
            driveSubsystem.driveArcade(driveSpeed, rotationSpeed);
            fuelSubsystem.stop();
        }
    }
}
```

---

## Testing and Validation Plan

1. **Power Consumption Baseline**
   - Log current draw patterns during typical operations
   - Identify peak consumption scenarios
   - Map voltage drop patterns under load

2. **Brownout Threshold Testing**
   - Controlled testing with discharged batteries
   - Validate scaling behavior at various voltage levels
   - Test recovery behavior as voltage rises

3. **Multi-Subsystem Stress Testing**
   - Simultaneous operation testing
   - Power coordination verification
   - Brownout prevention validation

---

## Conclusion

The current power management architecture has significant gaps that create high risk for brownout conditions during competition. The drive subsystem demonstrates good power management practices, but these patterns need to be extended system-wide for robust operation.

**Immediate action required:** Implement universal brownout protection across all subsystems before competition. The architectural foundation exists in the drive subsystem and can be replicated with minimal effort.

**Long-term recommendation:** Develop centralized power management architecture for future robot iterations to prevent these issues from recurring.

---

*Analysis completed by: Archie 🏗️*  
*Date: March 28, 2026*  
*Codebase: /home/sgoss/projects/2026-kit-bot*