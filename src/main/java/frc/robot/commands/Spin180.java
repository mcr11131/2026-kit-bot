// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDriveSubsystem;
import static frc.robot.Constants.SimConstants.*;

/**
 * Spins the robot 180 degrees in place using encoders.
 * Calculates the required wheel travel distance based on track width.
 */
public class Spin180 extends Command {
  private final CANDriveSubsystem driveSubsystem;
  private double leftStartPosition;
  private double rightStartPosition;

  // Distance each wheel needs to travel for a 180-degree turn (in meters)
  // Arc length = (track_width / 2) * π radians
  private static final double SPIN_DISTANCE = (TRACK_WIDTH_METERS / 2.0) * Math.PI;

  // Rotation speed (0.0 to 1.0) - full speed for quick spins
  private static final double SPIN_SPEED = 1.0;

  public Spin180(CANDriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    // Record starting positions
    leftStartPosition = driveSubsystem.getLeftPosition();
    rightStartPosition = driveSubsystem.getRightPosition();
  }

  @Override
  public void execute() {
    // Spin in place by driving wheels in opposite directions at full speed
    driveSubsystem.driveTank(SPIN_SPEED, -SPIN_SPEED);
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.driveTank(0, 0);
  }

  @Override
  public boolean isFinished() {
    // Check if either wheel has traveled the required distance
    // Using absolute value because one wheel goes forward, one backward
    double leftDistance = Math.abs(driveSubsystem.getLeftPosition() - leftStartPosition);
    double rightDistance = Math.abs(driveSubsystem.getRightPosition() - rightStartPosition);

    return leftDistance >= SPIN_DISTANCE || rightDistance >= SPIN_DISTANCE;
  }
}
