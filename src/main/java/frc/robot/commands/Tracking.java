// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.OperatorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.LimelightHelpers;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Tracking extends Command {
  /** Creates a new Drive. */
  CANDriveSubsystem driveSubsystem;

  public Tracking(CANDriveSubsystem driveSystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSystem);
    driveSubsystem = driveSystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  // The Y axis of the controller is inverted so that pushing the
  // stick away from you (a negative value) drives the robot forwards (a positive
  // value). The X axis is scaled down so the rotation is more easily
  // controllable.
  @Override
  public void execute() {
    if (!LimelightHelpers.getTV("limelight")) {
      driveSubsystem.driveArcade(0, 0);
      return;
    }
    double xOffset = LimelightHelpers.getTX("limelight");
    driveSubsystem.driveArcade(
        0,
        -MathUtil.applyDeadband(speedToDrive(xOffset), 0.05) * ROTATION_SCALING);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.driveArcade(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public double speedToDrive(double offset){
double speed;
if (Math.abs(offset)>4){
  speed = offset/35;
  if(speed > 1) {
    speed = 1;
  } else if (speed < -1) {
    speed = -1;
  }
} else {
  speed = 0;
}
  return speed;
}
}

