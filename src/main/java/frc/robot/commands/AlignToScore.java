// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CANDriveSubsystem;

/**
 * Aligns the robot to an AprilTag for scoring fuel.
 * - Rotates to center on the tag (using tx)
 * - Drives forward/backward to reach the target distance (using 3D pose)
 * - Shows a "ready to shoot" indicator on SmartDashboard
 */
public class AlignToScore extends Command {
  private final CANDriveSubsystem driveSubsystem;

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

  public AlignToScore(CANDriveSubsystem driveSystem) {
    addRequirements(driveSystem);
    driveSubsystem = driveSystem;
  }

  @Override
  public void initialize() {
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
    boolean ready = aimed && atDistance;
    SmartDashboard.putBoolean("Align/Ready to Shoot", ready);
    SmartDashboard.putNumber("Align/TX", tx);

    if (ready) {
      SmartDashboard.putString("Align/Status", "READY TO SHOOT");
    } else if (!aimed && !atDistance) {
      SmartDashboard.putString("Align/Status", "Aiming + Driving...");
    } else if (!aimed) {
      SmartDashboard.putString("Align/Status", "Aiming...");
    } else {
      SmartDashboard.putString("Align/Status", "Driving to distance...");
    }
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.driveArcade(0, 0);
    SmartDashboard.putBoolean("Align/Ready to Shoot", false);
    SmartDashboard.putString("Align/Status", "Stopped");
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
