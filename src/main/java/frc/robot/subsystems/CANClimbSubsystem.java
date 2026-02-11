// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ClimberConstants.*;
import static frc.robot.Constants.SimConstants.*;

public class CANClimbSubsystem extends SubsystemBase {
  private final SparkMax climberOne;
  private final SparkMax climberTwo;

  // Simulation support
  private SparkMaxSim climber1Sim;
  private SparkMaxSim climber2Sim;

  // Creates a new CANBallSubsystem.
  public CANClimbSubsystem() {
    // create brushed motors for each of the motors on the launcher mechanism
    climberOne = new SparkMax(CLIMBER_ONE, MotorType.kBrushless);
    climberTwo = new SparkMax(CLIMBER_TWO, MotorType.kBrushless);

    // create the configuration for the feeder roller, set a current limit and apply
    // the config to the controller
    SparkMaxConfig climbConfig = new SparkMaxConfig();
    climbConfig.smartCurrentLimit(CLIMBER_ONE_CURRENT_LIMIT);
    climberOne.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    climberTwo.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    // put default values for various fuel operations onto the dashboard
    // all commands using this subsystem pull values from the dashbaord to allow
    // you to tune the values easily, and then replace the values in Constants.java
    // with your new values. For more information, see the Software Guide.
    //SmartDashboard.putNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE);
    //SmartDashboard.putNumber("Intaking intake roller value", INTAKING_INTAKE_VOLTAGE);
    //SmartDashboard.putNumber("Launching feeder roller value", LAUNCHING_FEEDER_VOLTAGE);
    //SmartDashboard.putNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_VOLTAGE);
    //SmartDashboard.putNumber("Spin-up feeder roller value", SPIN_UP_FEEDER_VOLTAGE);

    // Initialize simulation objects
    DCMotor neo = DCMotor.getNEO(1);
    climber1Sim = new SparkMaxSim(climberOne, neo);
    climber2Sim = new SparkMaxSim(climberTwo, neo);
  }

  // A method to set the voltage of the intake roller
  public void setClimbers(double speed) {
    climberOne.set(speed);
    climberTwo.set(-speed);
  }

  

  // A method to stop the rollers
  public void stop() {
    climberOne.set(0);
    climberTwo.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /*
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
  */
}