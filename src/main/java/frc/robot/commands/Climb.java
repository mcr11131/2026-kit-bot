// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.subsystems.CANClimbSubsystem;
import static frc.robot.Constants.ClimberConstants.*;


public class Climb extends Command {

  CANClimbSubsystem climbSubsystem;
  DigitalInput limitSwitch;
  CommandGenericHID driverController;


  public Climb(CANClimbSubsystem climbSystem, DigitalInput limitSwitch, CommandGenericHID driverController) {
    addRequirements(climbSystem);
    this.climbSubsystem = climbSystem;
    this.limitSwitch = limitSwitch;
    this.driverController = driverController;
  }

  @Override
  public void initialize() {
    // Climb control is handled entirely in execute() via button state checks
  }
  @Override
  public void execute() {
    // C1 fix: merged into single if/else chain so POV and buttons both work
    // C2 fix: removed button 4 (Y) conflict with Drive direction toggle
    // M2 fix: added limit switch protection for downward movement
    if (driverController.povUp().getAsBoolean() || driverController.button(1).getAsBoolean()) {
      climbSubsystem.setRight(CLIMBER_UP_SPEED);
    } else if (driverController.povDown().getAsBoolean()) {
      // Only allow down if limit switch isn't triggered
      if (limitSwitch.get()) {
        climbSubsystem.setRight(CLIMBER_DOWN_SPEED);
      } else {
        climbSubsystem.rightstop();
      }
    } else {
      climbSubsystem.rightstop();
    }
  }

  @Override
  public void end(boolean interrupted) {
    climbSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}