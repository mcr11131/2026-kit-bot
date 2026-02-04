// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.DriveConstants.*;

public class CANDriveSubsystem extends SubsystemBase {
  private final SparkMax leftLeader;
  private final SparkMax leftFollower;
  private final SparkMax rightLeader;
  private final SparkMax rightFollower;

  private final DifferentialDrive drive;

  public CANDriveSubsystem() {
    // create brushed motors for drive
    leftLeader = new SparkMax(LEFT_LEADER_ID, MotorType.kBrushed);
    leftFollower = new SparkMax(LEFT_FOLLOWER_ID, MotorType.kBrushed);
    rightLeader = new SparkMax(RIGHT_LEADER_ID, MotorType.kBrushed);
    rightFollower = new SparkMax(RIGHT_FOLLOWER_ID, MotorType.kBrushed);

    // set up differential drive class
    drive = new DifferentialDrive(leftLeader, rightLeader);

    // Set can timeout. Because this project only sets parameters once on
    // construction, the timeout can be long without blocking robot operation. Code
    // which sets or gets parameters during operation may need a shorter timeout.
    leftLeader.setCANTimeout(250);
    rightLeader.setCANTimeout(250);
    leftFollower.setCANTimeout(250);
    rightFollower.setCANTimeout(250);

    // Voltage compensation helps the robot perform more similarly on different
    // battery voltages (at the cost of a little bit of top speed on a fully charged
    // battery). The current limit helps prevent tripping breakers.
    // Resetting in case a new controller is swapped in and persisting in case of a
    // controller reset due to breaker trip.

    // Configure leaders first
    SparkMaxConfig leaderConfig = new SparkMaxConfig();
    leaderConfig.voltageCompensation(12);
    leaderConfig.smartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);

    // Left side inverted so that positive values drive both sides forward
    SparkMaxConfig leftLeaderConfig = new SparkMaxConfig().apply(leaderConfig);
    leftLeaderConfig.inverted(true);
    leftLeader.configure(leftLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    rightLeader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Configure followers to match their respective leaders
    SparkMaxConfig leftFollowerConfig = new SparkMaxConfig().apply(leaderConfig);
    leftFollowerConfig.follow(leftLeader);
    leftFollower.configure(leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig rightFollowerConfig = new SparkMaxConfig().apply(leaderConfig);
    rightFollowerConfig.follow(rightLeader);
    rightFollower.configure(rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
  }

  public void driveArcade(double xSpeed, double zRotation) {
    drive.arcadeDrive(xSpeed, zRotation);
  }

}
