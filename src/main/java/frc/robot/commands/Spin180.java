package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDriveSubsystem;

/**
 * Spins the robot 180 degrees in place using the NavX gyro for accuracy.
 */
public class Spin180 extends Command {
  private final CANDriveSubsystem driveSubsystem;
  private final PIDController pidController;
  private double targetAngle;

  private static final double kP = 0.02;
  private static final double kI = 0.0;
  private static final double kD = 0.005;
  private static final double TOLERANCE_DEGREES = 3.0;
  private static final double MAX_SPIN_SPEED = 0.7;
  private static final double MIN_SPIN_SPEED = 0.1;

  public Spin180(CANDriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    pidController = new PIDController(kP, kI, kD);
    pidController.enableContinuousInput(-180, 180);
    pidController.setTolerance(TOLERANCE_DEGREES);
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    // Target is 180 degrees from current heading
    double current = driveSubsystem.getHeading();
    targetAngle = current + 180;
    // Wrap to -180..180
    if (targetAngle > 180) targetAngle -= 360;
    if (targetAngle < -180) targetAngle += 360;

    pidController.reset();
    pidController.setSetpoint(targetAngle);
  }

  @Override
  public void execute() {
    double output = pidController.calculate(driveSubsystem.getHeading());
    output = MathUtil.clamp(output, -MAX_SPIN_SPEED, MAX_SPIN_SPEED);
    if (Math.abs(output) < MIN_SPIN_SPEED && !pidController.atSetpoint()) {
      output = Math.copySign(MIN_SPIN_SPEED, output);
    }
    driveSubsystem.driveArcade(0, output);
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.driveArcade(0, 0);
  }

  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }
}