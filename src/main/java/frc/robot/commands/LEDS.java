// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LEDSubsystem;



/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LEDS extends Command {
  /** Creates a new Intake. */

  LEDSubsystem ledSubsystem;


  public LEDS(LEDSubsystem ledSystem) {
    addRequirements(ledSystem);
    this.ledSubsystem = ledSystem;
    
  }

  @Override
  public void initialize() {
    ledSubsystem.configureAlliance();
  }

  // Called every time the scheduler runs while the command is scheduled. This
  // command doesn't require updating any values while running
  @Override
  public void execute() {
   
    
  }

  // Called once the command ends or is interrupted. Stop the rollers
  @Override
  public void end(boolean interrupted) {
   
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