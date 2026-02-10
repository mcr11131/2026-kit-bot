// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.DriveConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDriveSubsystem;

public class AutoDrive extends Command {
  private final CANDriveSubsystem driveSubsystem;
  private final double xSpeed;
  private final double zRotation;
  private final double distanceMeters;

  private double startLeftPosition;
  private double startRightPosition;

  public AutoDrive(CANDriveSubsystem driveSystem, double xSpeed, double zRotation, double distanceMeters) {
    addRequirements(driveSystem);
    driveSubsystem = driveSystem;
    this.xSpeed = xSpeed;
    this.zRotation = zRotation;
    this.distanceMeters = distanceMeters;
  }

  @Override
  public void initialize() {
    startLeftPosition = driveSubsystem.getLeftPosition();
    startRightPosition = driveSubsystem.getRightPosition();
  }

  @Override
  public void execute() {
    // Use encoder difference to correct heading drift when driving straight.
    // If the left side has traveled further than the right, the robot has veered
    // right, so we steer left (subtract correction) to compensate.
    double leftDelta = driveSubsystem.getLeftPosition() - startLeftPosition;
    double rightDelta = driveSubsystem.getRightPosition() - startRightPosition;
    double error = leftDelta - rightDelta;
    double correction = error * STRAIGHT_KP;

    driveSubsystem.driveArcade(xSpeed, zRotation - correction);
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.driveArcade(0, 0);
  }

  @Override
  public boolean isFinished() {
    double leftDistance = Math.abs(driveSubsystem.getLeftPosition() - startLeftPosition);
    double rightDistance = Math.abs(driveSubsystem.getRightPosition() - startRightPosition);
    double averageDistance = (leftDistance + rightDistance) / 2.0;
    return averageDistance >= distanceMeters;
  }
}
