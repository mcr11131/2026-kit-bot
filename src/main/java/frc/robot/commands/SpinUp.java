// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CANFuelSubsystem;
import static frc.robot.Constants.FuelConstants.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SpinUp extends Command {
  /** Creates a new Intake. */

  CANFuelSubsystem fuelSubsystem;

  public SpinUp(CANFuelSubsystem fuelSystem) {
    addRequirements(fuelSystem);
    this.fuelSubsystem = fuelSystem;
  }

  // Called when the command is initially scheduled. Spin up ONLY the feeder motor
  // for 1 second before launching using velocity control
  @Override
  public void initialize() {
    // Only spin the feeder motor during spin-up, launcher stays off
    fuelSubsystem.setIntakeLauncherRoller(0);
    fuelSubsystem.setFeederRoller(SmartDashboard.getNumber("Spin-up feeder RPM", SPIN_UP_FEEDER_RPM));
  }

  // Called every time the scheduler runs while the command is scheduled. This
  // command doesn't require updating any values while running
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted. Stop the rollers
  @Override
  public void end(boolean interrupted) {
    fuelSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
