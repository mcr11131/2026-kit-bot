// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CANClimbSubsystem;
import static frc.robot.Constants.ClimberConstants.*;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Climb extends Command {
  /** Creates a new Intake. */

  CANClimbSubsystem climbSubsystem;
  boolean down;
  DigitalInput limitSwitch;
  CommandGenericHID driverController;


  public Climb(CANClimbSubsystem climbSystem, boolean down, DigitalInput limitSwitch, CommandGenericHID driverController) {
    addRequirements(climbSystem);
    this.climbSubsystem = climbSystem;
    this.down = down;
    this.limitSwitch = limitSwitch;
    this.driverController = driverController;
    
  }

  // Called when the command is initially scheduled. Set the rollers to the
  // appropriate values for intaking
  @Override
  public void initialize() {
    // Climb control is handled entirely in execute() via button state checks
  }

  // Called every time the scheduler runs while the command is scheduled. This
  // command doesn't require updating any values while running
  @Override
  public void execute() {
    if(driverController.povUp().getAsBoolean()) {
      climbSubsystem.setRight(CLIMBER_UP_SPEED);
    } else if (driverController.povDown().getAsBoolean()) {
      climbSubsystem.setRight(CLIMBER_DOWN_SPEED);
    } else {
      climbSubsystem.rightstop();
    }
    
    if(driverController.button(1).getAsBoolean()) {
      climbSubsystem.setRight(CLIMBER_UP_SPEED);
    } else if (driverController.button(4).getAsBoolean()) {
      climbSubsystem.setRight(CLIMBER_DOWN_SPEED);
    } else {
      climbSubsystem.rightstop();
    }
    
  }

  // Called once the command ends or is interrupted. Stop the rollers
  @Override
  public void end(boolean interrupted) {
    climbSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}


/*
if downRight.pressed
  run Right down
else if upRight.pressed
  run Right up
else
  Right.Stop
*/