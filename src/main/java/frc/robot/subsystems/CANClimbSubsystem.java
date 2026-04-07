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
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ClimberConstants.*;
import static frc.robot.Constants.SimConstants.*;
public class CANClimbSubsystem extends SubsystemBase {
  private final SparkMax climberTwo;

  // Simulation support
  private SparkMaxSim climber2Sim;

  // Telemetry log entries
  private DoubleLogEntry logLeftClimbOutput;
  private DoubleLogEntry logRightClimbOutput;
  private DoubleLogEntry logLeftClimbCurrent;
  private DoubleLogEntry logRightClimbCurrent;

  /** Creates a new CANClimbSubsystem. */
  public CANClimbSubsystem() {
    climberTwo = new SparkMax(CLIMBER_TWO, MotorType.kBrushless);

    // create the configuration for the climber, set a current limit and apply
    // the config to the controller
    SparkMaxConfig climbConfig = new SparkMaxConfig();
    climbConfig.smartCurrentLimit(CLIMBER_TWO_CURRENT_LIMIT);
    climbConfig.voltageCompensation(12);  // BROWNOUT FIX: Consistent performance as battery voltage drops
    climbConfig.openLoopRampRate(0.2);   // BROWNOUT FIX: Slower ramp for high-torque climber operation
    climbConfig.secondaryCurrentLimit(35.0);  // BROWNOUT FIX: Lower secondary limit for climber
    climberTwo.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Clear any sticky faults from previous runs
    climberTwo.clearFaults();

    // Initialize telemetry log entries
    DataLog log = DataLogManager.getLog();
    logLeftClimbOutput = new DoubleLogEntry(log, "/climb/leftOutput");
    logRightClimbOutput = new DoubleLogEntry(log, "/climb/rightOutput");
    logLeftClimbCurrent = new DoubleLogEntry(log, "/climb/leftCurrentAmps");
    logRightClimbCurrent = new DoubleLogEntry(log, "/climb/rightCurrentAmps");
    
    // Initialize simulation objects
    DCMotor neo = DCMotor.getNEO(1);
    climber2Sim = new SparkMaxSim(climberTwo, neo);
  }

  public void setRight(double speed) {
    climberTwo.set(speed);
  }
  

  public void stop() {
    climberTwo.set(0);
  }

  public void rightstop() {
    climberTwo.set(0);
  }

  @Override
  public void periodic() {
    // Status indicators for climb motors (climberOne removed)
    SmartDashboard.putBoolean("Right Climb Active", climberTwo.get() != 0);

    // Log telemetry for post-match analysis
    logRightClimbOutput.append(climberTwo.getAppliedOutput());
    logRightClimbCurrent.append(climberTwo.getOutputCurrent());
  }

}