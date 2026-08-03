// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static frc.robot.Constants.OperatorConstants.*;

import static frc.robot.Constants.FuelConstants.*;

import frc.robot.commands.AutoDrive;
import frc.robot.commands.Climb;
import frc.robot.commands.Drive;
import frc.robot.commands.Eject;
import frc.robot.commands.ExampleAuto;
import frc.robot.commands.Intake;
import frc.robot.commands.LEDS;
import frc.robot.commands.Launch;
import frc.robot.commands.LaunchSequence;
import frc.robot.commands.AlignToScore;
import frc.robot.commands.SpinUp;
import frc.robot.commands.TrenchAuto;
import frc.robot.commands.TurnToAngle;
import frc.robot.subsystems.CANClimbSubsystem;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.LEDSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final CANDriveSubsystem driveSubsystem = new CANDriveSubsystem();
  private final CANFuelSubsystem fuelSubsystem = new CANFuelSubsystem();
  private final CANClimbSubsystem climbSubsystem = new CANClimbSubsystem();
  private final LEDSubsystem ledSubsystem = new LEDSubsystem();

  // The driver's controller
  private final CommandGenericHID driverController = new CommandGenericHID(DRIVER_CONTROLLER_PORT);
  // The operator's controller
  // CURRENTLY NOT IN USE
  private final CommandGenericHID overrideController = new CommandGenericHID(
      OVERRIDE_CONTROLLER_PORT);

  // The autonomous chooser
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  private final DigitalInput downLimitSwitch = new DigitalInput(0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureBindings();
    configureAutonomousChooser();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)}
   * constructor with an arbitrary predicate, or via the named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for {@link CommandPS4Controller PS4}/
   * {@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // Toggle intake on/off with left bumper - press once to start, press again to
    // stop
    driverController.button(5).toggleOnTrue(new Intake(fuelSubsystem));
    // Toggle launch on/off with right bumper - press once to start launching, press
    // again to stop
    driverController.button(6).toggleOnTrue(new LaunchSequence(fuelSubsystem));
    // While the A button is held on the operator controller, eject fuel back out
    // the intake
    driverController.button(2).whileTrue(new Eject(fuelSubsystem));

    // Hold X to auto-align + launch, release to stop
    driverController.button(3).whileTrue(new AlignToScore(driveSubsystem, fuelSubsystem));

    //override controls. Pressing this button claims the drive subsystem over the default command of drive
    // and runs it with input from the secondary controller instead.
    overrideController.button(3).whileTrue(new Drive(driveSubsystem, overrideController)); 


    //all launch commands for operator as well
    // Toggle intake on/off with left bumper - press once to start, press again to
    // stop
    driverController.button(5).toggleOnTrue(new Intake(fuelSubsystem));

    // Toggle launch on/off with right bumper - press once to start launching, press
    // again to stop
    driverController.button(6).toggleOnTrue(new LaunchSequence(fuelSubsystem));

    // While the A button is held on the operator controller, eject fuel back out
    // the intake
    driverController.button(2).whileTrue(new Eject(fuelSubsystem));







    // Set the default command for the drive subsystem to the command provided by
    // factory with the values provided by the joystick axes on the driver
    // controller. The Y axis of the controller is inverted so that pushing the
    // stick away from you (a negative value) drives the robot forwards (a positive
    // value)
    driveSubsystem.setDefaultCommand(new Drive(driveSubsystem, driverController));

    fuelSubsystem.setDefaultCommand(fuelSubsystem.run(() -> fuelSubsystem.stop()));

    climbSubsystem.setDefaultCommand(new Climb(climbSubsystem, downLimitSwitch, driverController));

    ledSubsystem.setDefaultCommand(new LEDS(ledSubsystem));

  }


  private void configureAutonomousChooser() {
    autoChooser.setDefaultOption("Middle Position", new ExampleAuto(driveSubsystem, fuelSubsystem));
    autoChooser.addOption("Left Position", createLeftPositionAuto());
    autoChooser.addOption("Right Position", createRightPositionAuto());
    autoChooser.addOption("Trench Blocker", new TrenchAuto(driveSubsystem));
    autoChooser.addOption("Drive Forward", createDriveForwardAuto());
    autoChooser.addOption("Score And Drive", createScoreAndDriveAuto());
    autoChooser.addOption("Drive Turn Drive", createDriveTurnDriveAuto());
    // Isaacs fault
    autoChooser.addOption("Spin", createSpinAuto());

    SmartDashboard.putData("Auto Mode", autoChooser);
  }

  private Command createDriveForwardAuto() {
    return new AutoDrive(driveSubsystem, 0.6, 0.0, 2.0);
  }

  private Command createScoreAndDriveAuto() {
    return Commands.sequence(
        new SpinUp(fuelSubsystem).withTimeout(SPIN_UP_SECONDS),
        new Launch(fuelSubsystem).withTimeout(2.0),
        new AutoDrive(driveSubsystem, 0.6, 0.0, 1.5));
  }

  private Command createDriveTurnDriveAuto() {
    return Commands.sequence(
        Commands.runOnce(driveSubsystem::resetHeading, driveSubsystem),
        new AutoDrive(driveSubsystem, 0.6, 0.0, 1.0),
        new TurnToAngle(driveSubsystem, 90.0).withTimeout(2.5),
        new AutoDrive(driveSubsystem, 0.6, 0.0, 1.0));
  }

  private Command createLeftPositionAuto() {
    return Commands.sequence(
        new AutoDrive(driveSubsystem, 0.5, 0.25, 1.5),
        new SpinUp(fuelSubsystem).withTimeout(SPIN_UP_SECONDS),
        new Launch(fuelSubsystem).withTimeout(14));
  }

  private Command createRightPositionAuto() {
    return Commands.sequence(
        new AutoDrive(driveSubsystem, 0.5, -0.25, 1.5),
        new SpinUp(fuelSubsystem).withTimeout(SPIN_UP_SECONDS),
        new Launch(fuelSubsystem).withTimeout(15));
  }

  private Command createSpinAuto() {
    return Commands.sequence(new AutoDrive(driveSubsystem, 0, 3, 10));

  }

  /**
   * Sets rumble intensity on the driver controller to alert for endgame.
   * @param intensity Rumble intensity from 0.0 to 1.0
   */
  public void setRumble(double intensity) {
    driverController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, intensity);
  }

  /** Expose drive subsystem for button 4 direction toggle (used only by Drive command). */
  public CANDriveSubsystem getDriveSubsystem() {
    return driveSubsystem;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Return the currently selected autonomous routine from the dashboard chooser
    return autoChooser.getSelected();
  }
}
