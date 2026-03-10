// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.FuelConstants.*;
import static frc.robot.Constants.SimConstants.*;

public class CANFuelSubsystem extends SubsystemBase {
  private final SparkMax feederRoller;
  private final SparkMax intakeLauncherRoller;
  private final SparkMax intakeRoller;

  // Data logging entries
  private final DoubleLogEntry launcherCurrentLog;
  private final DoubleLogEntry feederCurrentLog;
  private final DoubleLogEntry launcherVoltageLog;
  private final DoubleLogEntry feederVoltageLog;
  private final DoubleLogEntry launcherTempLog;
  private final DoubleLogEntry feederTempLog;

  // Simulation support
  private SparkMaxSim launcherSim;
  private SparkMaxSim feederSim;
  private FlywheelSim launcherFlywheelSim;
  private FlywheelSim feederFlywheelSim;

  /** Creates a new CANBallSubsystem. */
  public CANFuelSubsystem() {
    // create brushless motors for each of the motors on the launcher mechanism
    intakeLauncherRoller = new SparkMax(AUGER_MOTOR_ID, MotorType.kBrushless);
    feederRoller = new SparkMax(FLYWHEEL_MOTOR_ID, MotorType.kBrushless);
    intakeRoller = new SparkMax(INTAKE_MOTOR_ID, MotorType.kBrushless);

    // create the configuration for the feeder roller, set a current limit and apply
    // the config to the controller
    SparkMaxConfig feederConfig = new SparkMaxConfig();
    feederConfig.smartCurrentLimit(FLYWHEEL_MOTOR_CURRENT_LIMIT);
    feederRoller.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // create the configuration for the launcher roller, set a current limit, set
    // the motor to inverted so that positive values are used for both intaking and
    // launching, and apply the config to the controller
    SparkMaxConfig launcherConfig = new SparkMaxConfig();
    launcherConfig.inverted(true);
    launcherConfig.smartCurrentLimit(AUGER_MOTOR_CURRENT_LIMIT);
    intakeLauncherRoller.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //Intake configuring, including current limit and safe reset parameters...
    SparkMaxConfig intakeConfig = new SparkMaxConfig();
    intakeConfig.smartCurrentLimit(INTAKE_MOTOR_CURRENT_LIMIT);
    intakeRoller.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // put default speed values for various fuel operations onto the dashboard
    // all commands using this subsystem pull values from the dashboard to allow
    // you to tune the values easily, and then replace the values in Constants.java
    // with your new values. For more information, see the Software Guide.
    SmartDashboard.putNumber("Intaking feeder speed", INTAKING_FLYWHEEL_SPEED);
    SmartDashboard.putNumber("Intaking launcher speed", INTAKING_AUGER_SPEED);
    SmartDashboard.putNumber("Launching feeder speed", LAUNCHING_FLYWHEEL_SPEED);
    SmartDashboard.putNumber("Launching launcher speed", LAUNCHING_AUGER_SPEED);
    SmartDashboard.putNumber("Spin-up feeder speed", SPIN_UP_FLYWHEEL_SPEED);
    SmartDashboard.putNumber("Eject feeder speed", EJECT_FLYWHEEL_SPEED);
    SmartDashboard.putNumber("Eject launcher speed", EJECT_AUGER_SPEED);

    // Initialize data logging
    var log = DataLogManager.getLog();
    launcherCurrentLog = new DoubleLogEntry(log, "/fuel/launcherCurrent");
    feederCurrentLog = new DoubleLogEntry(log, "/fuel/feederCurrent");
    launcherVoltageLog = new DoubleLogEntry(log, "/fuel/launcherVoltage");
    feederVoltageLog = new DoubleLogEntry(log, "/fuel/feederVoltage");
    launcherTempLog = new DoubleLogEntry(log, "/fuel/launcherTemp");
    feederTempLog = new DoubleLogEntry(log, "/fuel/feederTemp");

    // Initialize simulation objects
    DCMotor neo = DCMotor.getNEO(1);
    launcherSim = new SparkMaxSim(intakeLauncherRoller, neo);
    feederSim = new SparkMaxSim(feederRoller, neo);
    launcherFlywheelSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(neo, LAUNCHER_MOI, 1.0), neo);
    feederFlywheelSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(neo, FEEDER_MOI, 1.0), neo);
  }

  // A method to set the speed (percentage) of the intake/launcher roller
  public void setIntakeLauncherRoller(double speed) {
    intakeLauncherRoller.set(speed);
  }

  // A method to set the speed (percentage) of the feeder roller
  public void setFeederRoller(double speed) {
    feederRoller.set(speed);
  } 
  
  // A method to set the speed (percentage) of the intake roller
  public void setIntakeRoller(double speed) {
    intakeRoller.set(speed);
  }

  // A method to stop the rollers
  public void stop() {
    feederRoller.set(0);
    intakeLauncherRoller.set(0);
    intakeRoller.set(0);
  }

  @Override
  public void periodic() {
    // Read motor metrics
    double launcherCurrent = intakeLauncherRoller.getOutputCurrent();
    double feederCurrent = feederRoller.getOutputCurrent();
    double launcherVoltage = intakeLauncherRoller.getAppliedOutput() * intakeLauncherRoller.getBusVoltage();
    double feederVoltage = feederRoller.getAppliedOutput() * feederRoller.getBusVoltage();
    double launcherTemp = intakeLauncherRoller.getMotorTemperature();
    double feederTemp = feederRoller.getMotorTemperature();

    // Log to SmartDashboard for real-time viewing
    SmartDashboard.putNumber("Fuel/Launcher Current", launcherCurrent);
    SmartDashboard.putNumber("Fuel/Feeder Current", feederCurrent);
    SmartDashboard.putNumber("Fuel/Launcher Voltage", launcherVoltage);
    SmartDashboard.putNumber("Fuel/Feeder Voltage", feederVoltage);
    SmartDashboard.putNumber("Fuel/Launcher Temperature", launcherTemp);
    SmartDashboard.putNumber("Fuel/Feeder Temperature", feederTemp);

    // Log to data log files for post-match analysis
    launcherCurrentLog.append(launcherCurrent);
    feederCurrentLog.append(feederCurrent);
    launcherVoltageLog.append(launcherVoltage);
    feederVoltageLog.append(feederVoltage);
    launcherTempLog.append(launcherTemp);
    feederTempLog.append(feederTemp);
  }

  @Override
  public void simulationPeriodic() {
    double vbus = RobotController.getBatteryVoltage();

    launcherFlywheelSim.setInputVoltage(launcherSim.getAppliedOutput() * vbus);
    feederFlywheelSim.setInputVoltage(feederSim.getAppliedOutput() * vbus);

    launcherFlywheelSim.update(0.02);
    feederFlywheelSim.update(0.02);

    double launcherRPM = launcherFlywheelSim.getAngularVelocityRPM();
    double feederRPM = feederFlywheelSim.getAngularVelocityRPM();

    launcherSim.iterate(launcherRPM, vbus, 0.02);
    feederSim.iterate(feederRPM, vbus, 0.02);
  }
}
