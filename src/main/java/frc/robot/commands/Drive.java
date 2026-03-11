// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.OperatorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDriveSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Drive extends Command {
  /** Creates a new Drive. */
  CANDriveSubsystem driveSubsystem;
  CommandGenericHID controller;
  boolean cameraFront = true;
  boolean toggleLock = false;
  private final SlewRateLimiter turnLimiter = new SlewRateLimiter(TURN_SLEW_RATE);
  private double limitedThrottle = 0.0;
  private double lastThrottleTimestamp = 0.0;

  public Drive(CANDriveSubsystem driveSystem, CommandGenericHID driverController) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSystem);
    driveSubsystem = driveSystem;
    controller = driverController;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turnLimiter.reset(0.0);
    limitedThrottle = 0.0;
    lastThrottleTimestamp = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  // Cheesy Drive (Curvature Drive) - Split arcade controls:
  // - Left stick Y-axis: throttle (push forward = drive forward)
  // - Right stick X-axis: steering
  // - Left stick click (button 9): quick turn for sharp turns while moving
  @Override
  public void execute() {
    // Get throttle from left stick Y-axis (inverted: pushing forward is negative on axis)
    double throttle = applyThrottleAccelLimit(
        -MathUtil.applyDeadband(controller.getRawAxis(1), DRIVE_DEADBAND) * DRIVE_SCALING);
    // Get turn rate from right stick X-axis (inverted so right stick right = turn right)
    double turn = turnLimiter.calculate(
        -MathUtil.applyDeadband(controller.getRawAxis(4), TURN_DEADBAND) * ROTATION_SCALING);

    // Quick turn mode enabled when left stick is clicked (button 9)
    boolean quickTurn = controller.button(9).getAsBoolean();

    // Apply direction reversal if needed
    if (!cameraFront) {
      throttle = -throttle;
    }

    // If no throttle but steering input, spin in place at 25% power
    if (Math.abs(throttle) < 1e-3 && Math.abs(turn) > 1e-3) {
      driveSubsystem.spinInPlace(-0.3 * Math.signum(turn));
      return;
    }

    // Use cheesy drive (curvature drive)
    driveSubsystem.driveCurvature(throttle, turn, quickTurn);

    // Switch robot direction when Y button (4) is pressed - toggles front/back
    if (controller.button(4).getAsBoolean() == true && toggleLock == false) {
      if (cameraFront == true) {
        // Switch: front becomes back, back becomes front
        cameraFront = false;
        // Prevents it from looping
        toggleLock = true;
      } else {
        // Switch back: back becomes front, front becomes back
        cameraFront = true;
        toggleLock = true;
      }
    } else if (controller.button(4).getAsBoolean() == false) {
      toggleLock = false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turnLimiter.reset(0.0);
    limitedThrottle = 0.0;
    driveSubsystem.driveArcade(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private double applyThrottleAccelLimit(double requestedThrottle) {
    double now = Timer.getFPGATimestamp();
    double dt = now - lastThrottleTimestamp;
    lastThrottleTimestamp = now;

    if (dt <= 0.0) {
      dt = 0.02;
    }

    // Allow instantaneous decel to zero, but do not immediately accelerate through zero
    // into the opposite direction.
    if (Math.signum(requestedThrottle) != Math.signum(limitedThrottle)
        && Math.abs(requestedThrottle) > 1e-3
        && Math.abs(limitedThrottle) > 1e-3) {
      limitedThrottle = 0.0;
      return limitedThrottle;
    }

    // Only limit increases in commanded speed magnitude.
    if (Math.abs(requestedThrottle) > Math.abs(limitedThrottle)) {
      double maxDelta = THROTTLE_ACCEL_SLEW_RATE * dt;
      limitedThrottle = MathUtil.clamp(
          requestedThrottle,
          limitedThrottle - maxDelta,
          limitedThrottle + maxDelta);
    } else {
      limitedThrottle = requestedThrottle;
    }

    return limitedThrottle;
  }
}
