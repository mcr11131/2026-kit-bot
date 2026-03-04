// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.sim.SparkMaxSim;

import com.studica.frc.AHRS;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.SimConstants.*;
import static frc.robot.Constants.VisionConstants.*;

public class CANDriveSubsystem extends SubsystemBase {
  private final SparkMax leftLeader;
  private final SparkMax leftFollower;
  private final SparkMax rightLeader;
  private final SparkMax rightFollower;

  private final SparkClosedLoopController leftController;
  private final SparkClosedLoopController rightController;
  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;

  // Data logging entries
  private final DoubleLogEntry leftCurrentLog;
  private final DoubleLogEntry rightCurrentLog;
  private final DoubleLogEntry leftVoltageLog;
  private final DoubleLogEntry rightVoltageLog;
  private final DoubleLogEntry leftTempLog;
  private final DoubleLogEntry rightTempLog;
  private final DoubleLogEntry leftVelocityLog;
  private final DoubleLogEntry rightVelocityLog;
  private final DoubleLogEntry leftPositionLog;
  private final DoubleLogEntry rightPositionLog;
  private final DoubleLogEntry batteryVoltageLog;

  // NavX data logging entries
  private final DoubleLogEntry navxYawLog;
  private final DoubleLogEntry navxPitchLog;
  private final DoubleLogEntry navxRollLog;
  private final DoubleLogEntry navxAccelXLog;
  private final DoubleLogEntry navxAccelYLog;
  private final DoubleLogEntry navxAngleLog;
  private final DoubleLogEntry navxRateLog;

  // NavX Gyro
  private final AHRS navx;

  // Simulation support
  private SparkMaxSim leftLeaderSim;
  private SparkMaxSim leftFollowerSim;
  private SparkMaxSim rightLeaderSim;
  private SparkMaxSim rightFollowerSim;
  private DifferentialDrivetrainSim drivetrainSim;

  public CANDriveSubsystem() {
    // create brushed motors for drive
    leftLeader = new SparkMax(LEFT_LEADER_ID, MotorType.kBrushed);
    leftFollower = new SparkMax(LEFT_FOLLOWER_ID, MotorType.kBrushed);
    rightLeader = new SparkMax(RIGHT_LEADER_ID, MotorType.kBrushed);
    rightFollower = new SparkMax(RIGHT_FOLLOWER_ID, MotorType.kBrushed);

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
    leaderConfig.closedLoopRampRate(DRIVE_OPEN_LOOP_RAMP_RATE);
    leaderConfig.openLoopRampRate(DRIVE_OPEN_LOOP_RAMP_RATE);

    // Configure encoder conversion factors so readings are in meters and m/s
    double positionFactor = (2 * Math.PI * WHEEL_RADIUS_METERS) / DRIVE_GEAR_RATIO;
    double velocityFactor = positionFactor / 60.0;
    leaderConfig.encoder
        .countsPerRevolution(ENCODER_CPR)
        .positionConversionFactor(positionFactor)
        .velocityConversionFactor(velocityFactor);

    // Configure velocity PID + feedforward on the SparkMax
    leaderConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(DRIVE_KP)
        .velocityFF(DRIVE_KFF);

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

    leftController = leftLeader.getClosedLoopController();
    rightController = rightLeader.getClosedLoopController();
    leftEncoder = leftLeader.getEncoder();
    rightEncoder = rightLeader.getEncoder();

    // Initialize NavX gyro (SPI on MXP port)
    // Wrapped in try-catch so robot still works if NavX isn't connected
    AHRS tempNavx = null;
    try {
      tempNavx = new AHRS(AHRS.NavXComType.kMXP_SPI);
      System.out.println("[Drive] NavX initialized successfully");
    } catch (Exception e) {
      System.err.println("[Drive] WARNING: NavX failed to initialize - running without gyro: " + e.getMessage());
    }
    navx = tempNavx;

    // Initialize data logging
    var log = DataLogManager.getLog();
    leftCurrentLog = new DoubleLogEntry(log, "/drive/leftCurrent");
    rightCurrentLog = new DoubleLogEntry(log, "/drive/rightCurrent");
    leftVoltageLog = new DoubleLogEntry(log, "/drive/leftVoltage");
    rightVoltageLog = new DoubleLogEntry(log, "/drive/rightVoltage");
    leftTempLog = new DoubleLogEntry(log, "/drive/leftTemp");
    rightTempLog = new DoubleLogEntry(log, "/drive/rightTemp");
    leftVelocityLog = new DoubleLogEntry(log, "/drive/leftVelocity");
    rightVelocityLog = new DoubleLogEntry(log, "/drive/rightVelocity");
    leftPositionLog = new DoubleLogEntry(log, "/drive/leftPosition");
    rightPositionLog = new DoubleLogEntry(log, "/drive/rightPosition");
    batteryVoltageLog = new DoubleLogEntry(log, "/drive/batteryVoltage");

    // NavX data log entries
    navxYawLog = new DoubleLogEntry(log, "/navx/yaw");
    navxPitchLog = new DoubleLogEntry(log, "/navx/pitch");
    navxRollLog = new DoubleLogEntry(log, "/navx/roll");
    navxAccelXLog = new DoubleLogEntry(log, "/navx/accelX");
    navxAccelYLog = new DoubleLogEntry(log, "/navx/accelY");
    navxAngleLog = new DoubleLogEntry(log, "/navx/angle");
    navxRateLog = new DoubleLogEntry(log, "/navx/rate");

    // Initialize simulation objects for all four motors
    DCMotor driveMotor = DCMotor.getCIM(2);
    leftLeaderSim = new SparkMaxSim(leftLeader, driveMotor);
    leftFollowerSim = new SparkMaxSim(leftFollower, driveMotor);
    rightLeaderSim = new SparkMaxSim(rightLeader, driveMotor);
    rightFollowerSim = new SparkMaxSim(rightFollower, driveMotor);
    drivetrainSim = new DifferentialDrivetrainSim(
        driveMotor,
        DRIVE_GEAR_RATIO,
        DRIVE_MOI,
        ROBOT_MASS_KG,
        WHEEL_RADIUS_METERS,
        TRACK_WIDTH_METERS,
        null); // standard measurement noise
  }

  @Override
  public void periodic() {
    // Read motor metrics
    double leftCurrent = leftLeader.getOutputCurrent();
    double rightCurrent = rightLeader.getOutputCurrent();
    double leftVoltage = leftLeader.getAppliedOutput() * leftLeader.getBusVoltage();
    double rightVoltage = rightLeader.getAppliedOutput() * rightLeader.getBusVoltage();
    double leftTemp = leftLeader.getMotorTemperature();
    double rightTemp = rightLeader.getMotorTemperature();
    double leftVelocity = leftEncoder.getVelocity();
    double rightVelocity = rightEncoder.getVelocity();
    double leftPosition = leftEncoder.getPosition();
    double rightPosition = rightEncoder.getPosition();
    double batteryVoltage = RobotController.getBatteryVoltage();

    // Log to SmartDashboard for real-time viewing
    SmartDashboard.putNumber("Drive/Left Current", leftCurrent);
    SmartDashboard.putNumber("Drive/Right Current", rightCurrent);
    SmartDashboard.putNumber("Drive/Left Voltage", leftVoltage);
    SmartDashboard.putNumber("Drive/Right Voltage", rightVoltage);
    SmartDashboard.putNumber("Drive/Left Temperature", leftTemp);
    SmartDashboard.putNumber("Drive/Right Temperature", rightTemp);
    SmartDashboard.putNumber("Drive/Left Velocity", leftVelocity);
    SmartDashboard.putNumber("Drive/Right Velocity", rightVelocity);
    SmartDashboard.putNumber("Drive/Left Position", leftPosition);
    SmartDashboard.putNumber("Drive/Right Position", rightPosition);
    SmartDashboard.putNumber("Drive/Battery Voltage", batteryVoltage);

    // Log to data log files for post-match analysis
    leftCurrentLog.append(leftCurrent);
    rightCurrentLog.append(rightCurrent);
    leftVoltageLog.append(leftVoltage);
    rightVoltageLog.append(rightVoltage);
    leftTempLog.append(leftTemp);
    rightTempLog.append(rightTemp);
    leftVelocityLog.append(leftVelocity);
    rightVelocityLog.append(rightVelocity);
    leftPositionLog.append(leftPosition);
    rightPositionLog.append(rightPosition);
    batteryVoltageLog.append(batteryVoltage);

    // NavX gyro data — real-time dashboard (guarded in case NavX isn't connected)
    if (navx != null) {
      double yaw = navx.getYaw();
      double pitch = navx.getPitch();
      double roll = navx.getRoll();
      double accelX = navx.getWorldLinearAccelX();
      double accelY = navx.getWorldLinearAccelY();
      double angle = navx.getAngle();
      double rate = navx.getRate();

      SmartDashboard.putNumber("NavX/Yaw", yaw);
      SmartDashboard.putNumber("NavX/Pitch", pitch);
      SmartDashboard.putNumber("NavX/Roll", roll);
      SmartDashboard.putNumber("NavX/Accel X", accelX);
      SmartDashboard.putNumber("NavX/Accel Y", accelY);
      SmartDashboard.putNumber("NavX/Angle (continuous)", angle);
      SmartDashboard.putNumber("NavX/Turn Rate (deg/s)", rate);
      SmartDashboard.putBoolean("NavX/Connected", navx.isConnected());
      SmartDashboard.putBoolean("NavX/Calibrating", navx.isCalibrating());

    } else {
      SmartDashboard.putBoolean("NavX/Connected", false);
    }

    // Limelight data — always visible on dashboard (guarded against network errors)
    try {
      SmartDashboard.putBoolean("Limelight/Target Valid", LimelightHelpers.getTV(LIMELIGHT_NAME));
      SmartDashboard.putNumber("Limelight/TX", LimelightHelpers.getTX(LIMELIGHT_NAME));
      SmartDashboard.putNumber("Limelight/TY", LimelightHelpers.getTY(LIMELIGHT_NAME));
      SmartDashboard.putNumber("Limelight/Tag ID", LimelightHelpers.getFiducialID(LIMELIGHT_NAME));
      SmartDashboard.putNumber("Limelight/TA (area)", LimelightHelpers.getTA(LIMELIGHT_NAME));
      double[] pose = LimelightHelpers.getTargetPose_CameraSpace(LIMELIGHT_NAME);
      if (pose.length >= 3) {
        SmartDashboard.putNumber("Limelight/Distance (ft)", pose[2] * 3.28084);
      }
    } catch (Exception e) {
      // Limelight not available — don't crash the robot
    }

    // NavX data log files for post-match analysis (only if NavX is available)
    if (navx != null) {
      navxYawLog.append(navx.getYaw());
      navxPitchLog.append(navx.getPitch());
      navxRollLog.append(navx.getRoll());
      navxAccelXLog.append(navx.getWorldLinearAccelX());
      navxAccelYLog.append(navx.getWorldLinearAccelY());
      navxAngleLog.append(navx.getAngle());
      navxRateLog.append(navx.getRate());
    }
  }

  @Override
  public void simulationPeriodic() {
    double vbus = RobotController.getBatteryVoltage();
    drivetrainSim.setInputs(
        leftLeaderSim.getAppliedOutput() * vbus,
        rightLeaderSim.getAppliedOutput() * vbus);
    drivetrainSim.update(0.02);

    double leftVelocityRPM =
        drivetrainSim.getLeftVelocityMetersPerSecond()
            / (WHEEL_RADIUS_METERS * 2 * Math.PI) * 60.0 * DRIVE_GEAR_RATIO;
    double rightVelocityRPM =
        drivetrainSim.getRightVelocityMetersPerSecond()
            / (WHEEL_RADIUS_METERS * 2 * Math.PI) * 60.0 * DRIVE_GEAR_RATIO;

    leftLeaderSim.iterate(leftVelocityRPM, vbus, 0.02);
    leftFollowerSim.iterate(leftVelocityRPM, vbus, 0.02);
    rightLeaderSim.iterate(rightVelocityRPM, vbus, 0.02);
    rightFollowerSim.iterate(rightVelocityRPM, vbus, 0.02);

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(drivetrainSim.getCurrentDrawAmps()));
  }

  public double getLeftPosition() {
    return leftEncoder.getPosition();
  }

  public double getRightPosition() {
    return rightEncoder.getPosition();
  }

  public double getLeftVelocity() {
    return leftEncoder.getVelocity();
  }

  public double getRightVelocity() {
    return rightEncoder.getVelocity();
  }

  public double getHeading() {
    return navx != null ? navx.getYaw() : 0.0;
  }

  public void resetHeading() {
    if (navx != null) navx.reset();
  }

  public void driveArcade(double xSpeed, double zRotation) {
    var speeds = DifferentialDrive.arcadeDriveIK(xSpeed, zRotation, true);
    leftController.setSetpoint(speeds.left * MAX_SPEED_MPS, ControlType.kVelocity);
    rightController.setSetpoint(speeds.right * MAX_SPEED_MPS, ControlType.kVelocity);
  }

  public void driveTank(double lSpeed, double rSpeed) {
    var speeds = DifferentialDrive.tankDriveIK(lSpeed, rSpeed, true);
    leftController.setSetpoint(speeds.left * TANK_SPEED_MODIFIER, ControlType.kDutyCycle);
    rightController.setSetpoint(speeds.right * TANK_SPEED_MODIFIER, ControlType.kDutyCycle);
  }

  // Direct duty cycle spin - bypasses speed modifiers for reliable in-place rotation
  public void spinInPlace(double speed) {
    leftController.setSetpoint(speed, ControlType.kDutyCycle);
    rightController.setSetpoint(-speed, ControlType.kDutyCycle);
  }

  // Cheesy Drive (Curvature Drive) - provides car-like steering
  // When moving forward, turning is proportional to speed (like steering a car)
  // allowTurnInPlace enables quick turning when stationary or moving slowly
  public void driveCurvature(double xSpeed, double zRotation, boolean allowTurnInPlace) {
    var speeds = DifferentialDrive.curvatureDriveIK(xSpeed, zRotation, allowTurnInPlace);
    leftController.setSetpoint(speeds.left * TANK_SPEED_MODIFIER, ControlType.kDutyCycle);
    rightController.setSetpoint(speeds.right * TANK_SPEED_MODIFIER, ControlType.kDutyCycle);
  }


}

