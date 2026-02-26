// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.OperatorConstants.*;

import edu.wpi.first.math.MathUtil;
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

  public Drive(CANDriveSubsystem driveSystem, CommandGenericHID driverController) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSystem);
    driveSubsystem = driveSystem;
    controller = driverController;
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
    if (cameraFront == true) {
      driveSubsystem.driveTank( // set to .driveArcade and change the second axis to 4 for arcade drive
          -MathUtil.applyDeadband(controller.getRawAxis(1), 0.05) * DRIVE_SCALING,
          -MathUtil.applyDeadband(controller.getRawAxis(5), 0.05) * ROTATION_SCALING);

    } else {
      driveSubsystem.driveTank( // set to .driveArcade and change the second axis to 4 for arcade drive
          -MathUtil.applyDeadband(-controller.getRawAxis(1), 0.05) * DRIVE_SCALING,
          -MathUtil.applyDeadband(-controller.getRawAxis(5), 0.05) * ROTATION_SCALING);
    }
    //Switch robot direction
    if (controller.button(7).getAsBoolean() == true && toggleLock == false) {
      if (cameraFront == true) {
        //Direction of robot
        cameraFront = false;
        //Prevents it from looping
        toggleLock = true;
      } else {
        cameraFront = true;
        toggleLock = true;
      }
    } else if (controller.button(7).getAsBoolean() == false) {
      toggleLock = false;
    }
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
}
