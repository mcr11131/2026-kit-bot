// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static frc.robot.Constants.OperatorConstants.*;

import frc.robot.commands.Climb;
import frc.robot.commands.Drive;
import frc.robot.commands.Eject;
import frc.robot.commands.ExampleAuto;
import frc.robot.commands.Intake;
import frc.robot.commands.LaunchSequence;
import frc.robot.commands.Tracking;
import frc.robot.subsystems.CANClimbSubsystem;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;

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


  // The driver's controller
  private final CommandGenericHID driverController = new CommandGenericHID(DRIVER_CONTROLLER_PORT);

  // The operator's controller
  //CURRENTLY NOT IN USE
  private final CommandGenericHID operatorController = new CommandGenericHID(
      OPERATOR_CONTROLLER_PORT);


  // The autonomous chooser
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  private final DigitalInput downLimitSwitch = new DigitalInput(0);


  //The encoders
  //Encoders are in 10 and 12
  //private final Encoder leftEncoder = new Encoder(Constants.DriveConstants.LEFT_LEADER_ID, 10);
  //private final Encoder rightEncoder = new Encoder(Constants.DriveConstants.RIGHT_LEADER_ID, 12);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureBindings();

    // Set the options to show up in the Dashboard for selecting auto modes. If you
    // add additional auto modes you can add additional lines here with
    // autoChooser.addOption
    autoChooser.setDefaultOption("Autonomous", new ExampleAuto(driveSubsystem, fuelSubsystem));
    autoChooser.addOption("Do Nothing", new InstantCommand());

    // Publish the auto chooser to SmartDashboard so drivers can select auto mode
    SmartDashboard.putData("Auto Mode", autoChooser);
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

    // Toggle intake on/off with left bumper - press once to start, press again to stop
    driverController.button(5).toggleOnTrue(new Intake(fuelSubsystem));
    // Toggle launch on/off with right bumper - press once to start launching, press again to stop
    driverController.button(6).toggleOnTrue(new LaunchSequence(fuelSubsystem));
    // While the A button is held on the operator controller, eject fuel back out
    // the intake
    driverController.button(2).whileTrue(new Eject(fuelSubsystem));

    //Align to april tag when X is pushed
    driverController.button(3).whileTrue(new Tracking(driveSubsystem));

    //Switch way robot is facing by reversing speed when back button is clicked
    //driverController.button(7).toggleOnTrue();



    // Set the default command for the drive subsystem to the command provided by
    // factory with the values provided by the joystick axes on the driver
    // controller. The Y axis of the controller is inverted so that pushing the
    // stick away from you (a negative value) drives the robot forwards (a positive
    // value)
    driveSubsystem.setDefaultCommand(new Drive(driveSubsystem, driverController));

    fuelSubsystem.setDefaultCommand(fuelSubsystem.run(() -> fuelSubsystem.stop()));

    climbSubsystem.setDefaultCommand(new Climb(climbSubsystem, false, downLimitSwitch, driverController));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
