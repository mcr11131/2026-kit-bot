// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.FuelConstants.*;
import static frc.robot.Constants.SimConstants.*;

public class CANFuelSubsystem extends SubsystemBase {
  private final SparkMax feederRoller;
  private final SparkMax intakeLauncherRoller;

  // Encoders for velocity feedback
  private final RelativeEncoder feederEncoder;
  private final RelativeEncoder launcherEncoder;

  // PID controllers for closed-loop velocity control
  private final SparkClosedLoopController feederPIDController;
  private final SparkClosedLoopController launcherPIDController;

  // Simulation support
  private SparkMaxSim launcherSim;
  private SparkMaxSim feederSim;
  private FlywheelSim launcherFlywheelSim;
  private FlywheelSim feederFlywheelSim;

  /** Creates a new CANBallSubsystem. */
  public CANFuelSubsystem() {
    // create brushless motors for each of the motors on the launcher mechanism
    intakeLauncherRoller = new SparkMax(INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushless);
    feederRoller = new SparkMax(FEEDER_MOTOR_ID, MotorType.kBrushless);

    // Get encoders from the SparkMax controllers (built-in to NEO motors)
    feederEncoder = feederRoller.getEncoder();
    launcherEncoder = intakeLauncherRoller.getEncoder();

    // Get PID controllers from the SparkMax controllers
    feederPIDController = feederRoller.getClosedLoopController();
    launcherPIDController = intakeLauncherRoller.getClosedLoopController();

    // create the configuration for the feeder roller, set a current limit,
    // configure PID gains for velocity control, and apply the config to the controller
    SparkMaxConfig feederConfig = new SparkMaxConfig();
    feederConfig.smartCurrentLimit(FEEDER_MOTOR_CURRENT_LIMIT);
    feederConfig.closedLoop
        .pid(FEEDER_KP, FEEDER_KI, FEEDER_KD)
        .velocityFF(FEEDER_KFF);
    feederRoller.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // create the configuration for the launcher roller, set a current limit, set
    // the motor to inverted so that positive values are used for both intaking and
    // launching, configure PID gains for velocity control, and apply the config to the controller
    SparkMaxConfig launcherConfig = new SparkMaxConfig();
    launcherConfig.inverted(true);
    launcherConfig.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);
    launcherConfig.closedLoop
        .pid(LAUNCHER_KP, LAUNCHER_KI, LAUNCHER_KD)
        .velocityFF(LAUNCHER_KFF);
    intakeLauncherRoller.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // put default RPM values for various fuel operations onto the dashboard
    // all commands using this subsystem pull values from the dashboard to allow
    // you to tune the values easily, and then replace the values in Constants.java
    // with your new values. For more information, see the Software Guide.
    SmartDashboard.putNumber("Intaking feeder RPM", INTAKING_FEEDER_RPM);
    SmartDashboard.putNumber("Intaking launcher RPM", INTAKING_LAUNCHER_RPM);
    SmartDashboard.putNumber("Launching feeder RPM", LAUNCHING_FEEDER_RPM);
    SmartDashboard.putNumber("Launching launcher RPM", LAUNCHING_LAUNCHER_RPM);
    SmartDashboard.putNumber("Spin-up feeder RPM", SPIN_UP_FEEDER_RPM);
    SmartDashboard.putNumber("Eject feeder RPM", EJECT_FEEDER_RPM);
    SmartDashboard.putNumber("Eject launcher RPM", EJECT_LAUNCHER_RPM);

    // Initialize simulation objects
    DCMotor neo = DCMotor.getNEO(1);
    launcherSim = new SparkMaxSim(intakeLauncherRoller, neo);
    feederSim = new SparkMaxSim(feederRoller, neo);
    launcherFlywheelSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(neo, LAUNCHER_MOI, 1.0), neo);
    feederFlywheelSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(neo, FEEDER_MOI, 1.0), neo);
  }

  // A method to set the velocity (RPM) of the intake/launcher roller using closed-loop control
  public void setIntakeLauncherRoller(double rpm) {
    launcherPIDController.setReference(rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  // A method to set the velocity (RPM) of the feeder roller using closed-loop control
  public void setFeederRoller(double rpm) {
    feederPIDController.setReference(rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  // Get current launcher velocity in RPM (for debugging/tuning)
  public double getLauncherVelocity() {
    return launcherEncoder.getVelocity();
  }

  // Get current feeder velocity in RPM (for debugging/tuning)
  public double getFeederVelocity() {
    return feederEncoder.getVelocity();
  }

  // A method to stop the rollers
  public void stop() {
    feederRoller.set(0);
    intakeLauncherRoller.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Display current velocities on SmartDashboard for tuning
    SmartDashboard.putNumber("Launcher Actual RPM", getLauncherVelocity());
    SmartDashboard.putNumber("Feeder Actual RPM", getFeederVelocity());
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
