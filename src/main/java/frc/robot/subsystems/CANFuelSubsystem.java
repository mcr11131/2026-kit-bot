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

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.FuelConstants.*;
import static frc.robot.Constants.SimConstants.*;

public class CANFuelSubsystem extends SubsystemBase {
  private final TalonFX feederRoller;
  private final DutyCycleOut feederDutyCycle = new DutyCycleOut(0);
  private final SparkMax intakeLauncherRoller;
  private final SparkMax intakeRoller;

  // TalonFX status signals for telemetry
  private final StatusSignal<Current> feederCurrentSignal;
  private final StatusSignal<Voltage> feederVoltageSignal;
  private final StatusSignal<Temperature> feederTempSignal;

  // Data logging entries
  private final DoubleLogEntry launcherCurrentLog;
  private final DoubleLogEntry feederCurrentLog;
  private final DoubleLogEntry launcherVoltageLog;
  private final DoubleLogEntry feederVoltageLog;
  private final DoubleLogEntry launcherTempLog;
  private final DoubleLogEntry feederTempLog;

  // Simulation support
  private SparkMaxSim launcherSim;
  private FlywheelSim launcherFlywheelSim;

  /** Creates a new CANFuelSubsystem. */
  public CANFuelSubsystem() {
    // create brushless motors for each of the motors on the launcher mechanism
    intakeLauncherRoller = new SparkMax(AUGER_MOTOR_ID, MotorType.kBrushless);
    feederRoller = new TalonFX(FLYWHEEL_MOTOR_ID);
    intakeRoller = new SparkMax(INTAKE_MOTOR_ID, MotorType.kBrushless);

    // configure the Kraken X60 (TalonFX) feeder roller — current limits and ramp
    TalonFXConfiguration feederConfig = new TalonFXConfiguration();
    feederConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    feederConfig.CurrentLimits.StatorCurrentLimit = FLYWHEEL_MOTOR_CURRENT_LIMIT;
    feederConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    feederConfig.CurrentLimits.SupplyCurrentLimit = 40;
    feederConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.1; // 100ms ramp
    feederConfig.Voltage.PeakForwardVoltage = 12;
    feederConfig.Voltage.PeakReverseVoltage = -12;
    feederRoller.getConfigurator().apply(feederConfig);

    // H3 fix: clear sticky faults from previous runs/brownouts
    feederRoller.clearStickyFaults();

    // L3: log TalonFX firmware version for pit diagnostics
    System.out.println("[Fuel] Kraken X60 (CAN " + FLYWHEEL_MOTOR_ID + ") firmware: " + feederRoller.getVersion());

    // cache status signals for periodic telemetry
    feederCurrentSignal = feederRoller.getStatorCurrent();
    feederVoltageSignal = feederRoller.getMotorVoltage();
    feederTempSignal = feederRoller.getDeviceTemp();

    // Clear any sticky faults from previous runs
    intakeLauncherRoller.clearFaults();

    // create the configuration for the launcher roller, set a current limit, set
    // the motor to inverted so that positive values are used for both intaking and
    // launching, and apply the config to the controller
    SparkMaxConfig launcherConfig = new SparkMaxConfig();
    launcherConfig.inverted(true);
    launcherConfig.smartCurrentLimit(AUGER_MOTOR_CURRENT_LIMIT);
    launcherConfig.voltageCompensation(12);  // BROWNOUT FIX: Consistent performance as battery voltage drops
    launcherConfig.openLoopRampRate(0.1);   // BROWNOUT FIX: Prevent current spikes (100ms to full power)
    intakeLauncherRoller.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //Intake configuring, including current limit and safe reset parameters...
    SparkMaxConfig intakeConfig = new SparkMaxConfig();
    intakeConfig.smartCurrentLimit(INTAKE_MOTOR_CURRENT_LIMIT);
    intakeConfig.voltageCompensation(12);  // BROWNOUT FIX: Consistent performance as battery voltage drops
    intakeConfig.openLoopRampRate(0.1);   // BROWNOUT FIX: Prevent current spikes (100ms to full power)
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
    launcherFlywheelSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(neo, LAUNCHER_MOI, 1.0), neo);
  }

  // A method to set the speed (percentage) of the intake/launcher roller
  public void setIntakeLauncherRoller(double speed) {
    intakeLauncherRoller.set(speed);
  }

  // A method to set the speed (percentage) of the feeder roller (Kraken X60 / TalonFX)
  public void setFeederRoller(double speed) {
    feederRoller.setControl(feederDutyCycle.withOutput(speed));
  }

  // A method to set the speed (percentage) of the intake roller
  public void setIntakeRoller(double speed) {
    intakeRoller.set(speed);
  }

  // A method to stop the rollers
  public void stop() {
    feederRoller.setControl(feederDutyCycle.withOutput(0));
    intakeLauncherRoller.set(0);
    intakeRoller.set(0);
  }

  @Override
  public void periodic() {
    // Status indicators so drivers can see what's active
    SmartDashboard.putBoolean("Intake/Launcher Active", intakeLauncherRoller.get() != 0);

    // Read motor metrics
    double launcherCurrent = intakeLauncherRoller.getOutputCurrent();
    double feederCurrent = feederCurrentSignal.refresh().getValueAsDouble();
    double launcherVoltage = intakeLauncherRoller.getAppliedOutput() * intakeLauncherRoller.getBusVoltage();
    double feederVoltage = feederVoltageSignal.refresh().getValueAsDouble();
    double launcherTemp = intakeLauncherRoller.getMotorTemperature();
    double feederTemp = feederTempSignal.refresh().getValueAsDouble();

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
    launcherFlywheelSim.update(0.02);

    double launcherRPM = launcherFlywheelSim.getAngularVelocityRPM();
    launcherSim.iterate(launcherRPM, vbus, 0.02);

    // TalonFX (Kraken X60) simulation is handled by Phoenix 6 internally
  }
}
