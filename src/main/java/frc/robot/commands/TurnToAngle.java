package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDriveSubsystem;

/**
 * Turns the robot to a specific heading using the NavX gyro and PID control.
 * The target angle is in degrees, where 0 is the direction the robot was facing
 * when the gyro was last reset.
 */
public class TurnToAngle extends Command {
  private final CANDriveSubsystem driveSubsystem;
  private final PIDController pidController;
  private final double targetAngleDegrees;

  private static final double kP = 0.02;
  private static final double kI = 0.0;
  private static final double kD = 0.005;
  private static final double TOLERANCE_DEGREES = 2.0;
  private static final double MAX_TURN_SPEED = 0.6;
  private static final double MIN_TURN_SPEED = 0.08;

  /**
   * @param driveSubsystem The drive subsystem
   * @param targetAngleDegrees The target heading in degrees (positive = counterclockwise)
   */
  public TurnToAngle(CANDriveSubsystem driveSubsystem, double targetAngleDegrees) {
    this.driveSubsystem = driveSubsystem;
    this.targetAngleDegrees = targetAngleDegrees;

    pidController = new PIDController(kP, kI, kD);
    pidController.enableContinuousInput(-180, 180);
    pidController.setTolerance(TOLERANCE_DEGREES);

    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    pidController.reset();
    pidController.setSetpoint(targetAngleDegrees);
    SmartDashboard.putString("TurnToAngle/Status", "Turning to " + targetAngleDegrees + "°");
  }

  @Override
  public void execute() {
    double currentAngle = driveSubsystem.getHeading();
    double output = pidController.calculate(currentAngle);

    // Clamp output and add minimum speed to overcome static friction
    output = MathUtil.clamp(output, -MAX_TURN_SPEED, MAX_TURN_SPEED);
    if (Math.abs(output) < MIN_TURN_SPEED && !pidController.atSetpoint()) {
      output = Math.copySign(MIN_TURN_SPEED, output);
    }

    driveSubsystem.driveArcade(0, output);

    SmartDashboard.putNumber("TurnToAngle/Current", currentAngle);
    SmartDashboard.putNumber("TurnToAngle/Error", pidController.getPositionError());
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.driveArcade(0, 0);
    SmartDashboard.putString("TurnToAngle/Status", interrupted ? "Interrupted" : "Done");
  }

  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }
}