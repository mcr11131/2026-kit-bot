// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDriveSubsystem;

/**
 * Spins the robot 180 degrees in place.
 * Uses a timed rotation - adjust SPIN_TIME to calibrate for exactly 180 degrees.
 */
public class Spin180 extends Command {
  private final CANDriveSubsystem driveSubsystem;
  private final Timer timer = new Timer();

  // Time to spin 180 degrees - tune this value based on your robot
  // Increased to 2.0 seconds to make it more noticeable for testing
  private static final double SPIN_TIME = 2.0;

  // Rotation speed (0.0 to 1.0) - adjust if spinning too fast/slow
  // Increased to 0.7 for more obvious spinning
  private static final double SPIN_SPEED = 0.7;

  public Spin180(CANDriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    timer.restart();
  }

  @Override
  public void execute() {
    // Spin in place by driving wheels in opposite directions
    driveSubsystem.driveTank(SPIN_SPEED, -SPIN_SPEED);
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.driveTank(0, 0);
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(SPIN_TIME);
  }
}
