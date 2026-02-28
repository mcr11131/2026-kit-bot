// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.SimConstants.*;

public class CANDriveSubsystem extends SubsystemBase {
  private final SparkMax leftLeader;
  private final SparkMax leftFollower;
  private final SparkMax rightLeader;
  private final SparkMax rightFollower;

  private final SparkClosedLoopController leftController;
  private final SparkClosedLoopController rightController;
  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;

  // Simulation support
  private SparkMaxSim leftLeaderSim;
  private SparkMaxSim leftFollowerSim;
  private SparkMaxSim rightLeaderSim;
  private SparkMaxSim rightFollowerSim;
  private DifferentialDrivetrainSim drivetrainSim;

  // Telemetry log entries for post-match analysis
  private DoubleLogEntry logLeftVelocity;
  private DoubleLogEntry logRightVelocity;
  private DoubleLogEntry logLeftPosition;
  private DoubleLogEntry logRightPosition;
  private DoubleLogEntry logLeftOutput;
  private DoubleLogEntry logRightOutput;
  private DoubleLogEntry logLeftCurrent;
  private DoubleLogEntry logRightCurrent;
  private DoubleLogEntry logBrownoutScale;
  private BooleanLogEntry logEncoderWarning;

  // Encoder health tracking — detect stuck encoders while motors are commanded
  private double encoderWarningStartTime = -1;
  private static final double ENCODER_WARNING_THRESHOLD_SECONDS = 2.0;
  private static final double MOTOR_COMMAND_THRESHOLD = 0.1;
  private boolean encoderWarningActive = false;

  // Track last commanded speeds for status display
  private double lastCommandedLeft = 0;
  private double lastCommandedRight = 0;

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

    // Clear any sticky faults from previous runs
    leftLeader.clearFaults();
    leftFollower.clearFaults();
    rightLeader.clearFaults();
    rightFollower.clearFaults();

    // Initialize telemetry log entries
    DataLog log = DataLogManager.getLog();
    logLeftVelocity = new DoubleLogEntry(log, "/drive/leftVelocityMps");
    logRightVelocity = new DoubleLogEntry(log, "/drive/rightVelocityMps");
    logLeftPosition = new DoubleLogEntry(log, "/drive/leftPositionM");
    logRightPosition = new DoubleLogEntry(log, "/drive/rightPositionM");
    logLeftOutput = new DoubleLogEntry(log, "/drive/leftOutput");
    logRightOutput = new DoubleLogEntry(log, "/drive/rightOutput");
    logLeftCurrent = new DoubleLogEntry(log, "/drive/leftCurrentAmps");
    logRightCurrent = new DoubleLogEntry(log, "/drive/rightCurrentAmps");
    logBrownoutScale = new DoubleLogEntry(log, "/drive/brownoutScale");
    logEncoderWarning = new BooleanLogEntry(log, "/drive/encoderWarning");

    // Initialize brake mode toggle on dashboard
    SmartDashboard.putBoolean("Brake Mode", false);

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
    // Encoder health check - warn if motors are commanded but encoders read zero
    boolean motorsCommanded = Math.abs(lastCommandedLeft) > MOTOR_COMMAND_THRESHOLD
        || Math.abs(lastCommandedRight) > MOTOR_COMMAND_THRESHOLD;
    boolean encodersStuck = Math.abs(leftEncoder.getVelocity()) < 0.01
        && Math.abs(rightEncoder.getVelocity()) < 0.01;

    if (motorsCommanded && encodersStuck) {
      if (encoderWarningStartTime < 0) {
        encoderWarningStartTime = Timer.getFPGATimestamp();
      }
      encoderWarningActive = (Timer.getFPGATimestamp() - encoderWarningStartTime)
          > ENCODER_WARNING_THRESHOLD_SECONDS;
      SmartDashboard.putBoolean("Encoder Warning", encoderWarningActive);
    } else {
      encoderWarningStartTime = -1;
      encoderWarningActive = false;
      SmartDashboard.putBoolean("Encoder Warning", false);
    }

    // Status indicators
    SmartDashboard.putNumber("Left Velocity (m/s)", leftEncoder.getVelocity());
    SmartDashboard.putNumber("Right Velocity (m/s)", rightEncoder.getVelocity());
    SmartDashboard.putString("Drive Mode", "Tank");

    // Brownout indicator
    double brownout = getBrownoutScale();
    SmartDashboard.putBoolean("Brownout Active", brownout < 1.0);

    // Log telemetry for post-match analysis
    logLeftVelocity.append(leftEncoder.getVelocity());
    logRightVelocity.append(rightEncoder.getVelocity());
    logLeftPosition.append(leftEncoder.getPosition());
    logRightPosition.append(rightEncoder.getPosition());
    logLeftOutput.append(leftLeader.getAppliedOutput());
    logRightOutput.append(rightLeader.getAppliedOutput());
    logLeftCurrent.append(leftLeader.getOutputCurrent());
    logRightCurrent.append(rightLeader.getOutputCurrent());
    logBrownoutScale.append(brownout);
    logEncoderWarning.append(encoderWarningActive);
  }

  /**
   * Returns a speed scaling factor based on battery voltage.
   * Below 8V, scales to 50% to prevent brownout-induced Rio reboots.
   * Between 8V and 9V, linearly scales from 50% to 100%.
   */
  private double getBrownoutScale() {
    double voltage = RobotController.getBatteryVoltage();
    if (voltage < 8.0) {
      return 0.5;
    } else if (voltage < 9.0) {
      return 0.5 + 0.5 * (voltage - 8.0);
    }
    return 1.0;
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

  public void driveArcade(double xSpeed, double zRotation) {
    double scale = getBrownoutScale();
    var speeds = DifferentialDrive.arcadeDriveIK(xSpeed * scale, zRotation * scale, true);
    lastCommandedLeft = speeds.left;
    lastCommandedRight = speeds.right;
    leftController.setReference(speeds.left * MAX_SPEED_MPS, ControlType.kVelocity);
    rightController.setReference(speeds.right * MAX_SPEED_MPS, ControlType.kVelocity);
  }

   public void driveTank(double lSpeed, double rSpeed) {
    double scale = getBrownoutScale();
    var speeds = DifferentialDrive.tankDriveIK(lSpeed * scale, rSpeed * scale, true);
    lastCommandedLeft = speeds.left;
    lastCommandedRight = speeds.right;
    leftController.setReference(speeds.left * TANK_SPEED_MODIFIER, ControlType.kDutyCycle);
    rightController.setReference(speeds.right * TANK_SPEED_MODIFIER, ControlType.kDutyCycle);
  }


}
