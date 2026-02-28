// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;
import static frc.robot.Constants.FuelConstants.*;

/**
 * Full scoring sequence — hold one button to:
 * 1. Align to AprilTag (aim + drive to 5ft)
 * 2. Auto spin-up when aligned
 * 3. Auto launch fuel
 * Release button to stop everything.
 */
public class AlignToScore extends Command {
  private final CANDriveSubsystem driveSubsystem;
  private final CANFuelSubsystem fuelSubsystem;

  // Target distance from the AprilTag in meters (5 feet = 1.524m)
  private static final double TARGET_DISTANCE_METERS = 1.524;

  // Tolerances — how close is "good enough"
  private static final double AIM_TOLERANCE_DEGREES = 2.0;
  private static final double DISTANCE_TOLERANCE_METERS = 0.1; // ~4 inches

  // Speed limits to keep movements controlled
  private static final double MAX_ROTATION_SPEED = 0.5;
  private static final double MAX_DRIVE_SPEED = 0.4;

  // Proportional gains — tune these on the real robot
  private static final double AIM_KP = 0.02;
  private static final double DISTANCE_KP = 0.8;

  // Launcher state tracking
  private enum LaunchState { ALIGNING, SPINNING_UP, LAUNCHING }
  private LaunchState launchState;
  private final Timer spinUpTimer = new Timer();

  public AlignToScore(CANDriveSubsystem driveSystem, CANFuelSubsystem fuelSystem) {
    addRequirements(driveSystem, fuelSystem);
    driveSubsystem = driveSystem;
    fuelSubsystem = fuelSystem;
  }

  @Override
  public void initialize() {
    launchState = LaunchState.ALIGNING;
    spinUpTimer.reset();
    SmartDashboard.putBoolean("Align/Ready to Shoot", false);
    SmartDashboard.putString("Align/Status", "Searching...");
  }

  @Override
  public void execute() {
    // No target — stop and wait
    if (!LimelightHelpers.getTV("limelight")) {
      driveSubsystem.driveArcade(0, 0);
      SmartDashboard.putBoolean("Align/Ready to Shoot", false);
      SmartDashboard.putString("Align/Status", "No target");
      return;
    }

    // --- ROTATION: center on the tag using tx ---
    double tx = LimelightHelpers.getTX("limelight");
    double rotationSpeed = 0;
    boolean aimed = Math.abs(tx) < AIM_TOLERANCE_DEGREES;

    if (!aimed) {
      // Proportional control: turn faster when further off-center
      rotationSpeed = -tx * AIM_KP;
      rotationSpeed = MathUtil.clamp(rotationSpeed, -MAX_ROTATION_SPEED, MAX_ROTATION_SPEED);
      // Add minimum speed to overcome static friction
      if (Math.abs(rotationSpeed) < 0.05) {
        rotationSpeed = Math.copySign(0.05, rotationSpeed);
      }
    }

    // --- DISTANCE: drive to 5 feet using 3D pose ---
    double[] targetPose = LimelightHelpers.getTargetPose_CameraSpace("limelight");
    double driveSpeed = 0;
    boolean atDistance = false;

    if (targetPose.length >= 3) {
      // targetPose[2] is the Z component — forward distance from camera to tag in meters
      double currentDistance = targetPose[2];
      double distanceError = currentDistance - TARGET_DISTANCE_METERS;
      atDistance = Math.abs(distanceError) < DISTANCE_TOLERANCE_METERS;

      SmartDashboard.putNumber("Align/Distance (ft)", currentDistance * 3.28084);
      SmartDashboard.putNumber("Align/Distance Error (ft)", distanceError * 3.28084);

      if (!atDistance) {
        // Proportional control: drive faster when further from target distance
        driveSpeed = distanceError * DISTANCE_KP;
        driveSpeed = MathUtil.clamp(driveSpeed, -MAX_DRIVE_SPEED, MAX_DRIVE_SPEED);
        // Minimum speed to overcome friction
        if (Math.abs(driveSpeed) < 0.05) {
          driveSpeed = Math.copySign(0.05, driveSpeed);
        }
      }
    }

    // Drive: forward/backward for distance, rotation for aim
    driveSubsystem.driveArcade(driveSpeed, rotationSpeed);

    // Update dashboard
    boolean aligned = aimed && atDistance;
    SmartDashboard.putBoolean("Align/Ready to Shoot", aligned);
    SmartDashboard.putNumber("Align/TX", tx);

    // --- LAUNCH STATE MACHINE ---
    switch (launchState) {
      case ALIGNING:
        // Wait until aligned, then start spin-up
        if (aligned) {
          launchState = LaunchState.SPINNING_UP;
          spinUpTimer.restart();
          // Spin up launcher, feeder stays off
          fuelSubsystem.setIntakeLauncherRoller(
              SmartDashboard.getNumber("Launching launcher speed", LAUNCHING_LAUNCHER_SPEED));
          fuelSubsystem.setFeederRoller(
              SmartDashboard.getNumber("Spin-up feeder speed", SPIN_UP_FEEDER_SPEED));
          SmartDashboard.putString("Align/Status", "Spinning up...");
        } else if (!aimed && !atDistance) {
          SmartDashboard.putString("Align/Status", "Aiming + Driving...");
        } else if (!aimed) {
          SmartDashboard.putString("Align/Status", "Aiming...");
        } else {
          SmartDashboard.putString("Align/Status", "Driving to distance...");
        }
        break;

      case SPINNING_UP:
        // If we lose alignment, go back to aligning
        if (!aligned) {
          launchState = LaunchState.ALIGNING;
          fuelSubsystem.stop();
          SmartDashboard.putString("Align/Status", "Lost target, re-aligning...");
          break;
        }
        // After spin-up time, start launching
        if (spinUpTimer.hasElapsed(SPIN_UP_SECONDS)) {
          launchState = LaunchState.LAUNCHING;
          fuelSubsystem.setIntakeLauncherRoller(
              SmartDashboard.getNumber("Launching launcher speed", LAUNCHING_LAUNCHER_SPEED));
          fuelSubsystem.setFeederRoller(
              SmartDashboard.getNumber("Launching feeder speed", LAUNCHING_FEEDER_SPEED));
          SmartDashboard.putString("Align/Status", "LAUNCHING!");
        } else {
          SmartDashboard.putString("Align/Status", "Spinning up...");
        }
        break;

      case LAUNCHING:
        // Keep launching, but if we lose alignment badly, stop
        if (!aimed && Math.abs(tx) > AIM_TOLERANCE_DEGREES * 3) {
          launchState = LaunchState.ALIGNING;
          fuelSubsystem.stop();
          SmartDashboard.putString("Align/Status", "Lost aim, re-aligning...");
        } else {
          SmartDashboard.putString("Align/Status", "LAUNCHING!");
        }
        break;
    }
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.driveArcade(0, 0);
    fuelSubsystem.stop();
    spinUpTimer.stop();
    SmartDashboard.putBoolean("Align/Ready to Shoot", false);
    SmartDashboard.putString("Align/Status", "Stopped");
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
