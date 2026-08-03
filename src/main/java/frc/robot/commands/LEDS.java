// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;

public class LEDS extends Command {

  LEDSubsystem ledSubsystem;
  private boolean allianceSet = false;


  public LEDS(LEDSubsystem ledSystem) {
    addRequirements(ledSystem);
    this.ledSubsystem = ledSystem;
  }

  @Override
  public void initialize() {
    allianceSet = false;
    // Try immediately — may not be available yet
    if (ledSubsystem.isAllianceKnown()) {
      ledSubsystem.configureAlliance();
      allianceSet = true;
    }
    // A color looks better than no color, also allows us to see if it is connected yet
    //also will not set if the color has been set already
     else if (!allianceSet){  
      ledSubsystem.setPuce();
      }
  }

  // H2 fix: retry alliance color until FMS data arrives
  @Override
  public void execute() {
    if (!allianceSet && ledSubsystem.isAllianceKnown()) {
      ledSubsystem.configureAlliance();
      allianceSet = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    //at the end of a match or if the robot is disabled, set to puce
    ledSubsystem.setPuce();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}