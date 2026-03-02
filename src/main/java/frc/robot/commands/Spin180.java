package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDriveSubsystem;
import static frc.robot.Constants.DriveConstants.*;

/**
 * Spins the robot 180 degrees in place using the NavX gyro for accuracy.
 */
public class Spin180 extends Command {
  private final CANDriveSubsystem driveSubsystem;
  private final PIDController pidController;
  private double targetAngle;

  public Spin180(CANDriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    pidController = new PIDController(TURN_KP, TURN_KI, TURN_KD);
    pidController.enableContinuousInput(-180, 180);
    pidController.setTolerance(TURN_TOLERANCE_DEGREES);
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
    output = MathUtil.clamp(output, -MAX_TURN_SPEED, MAX_TURN_SPEED);
    if (Math.abs(output) < MIN_TURN_SPEED && !pidController.atSetpoint()) {
      output = Math.copySign(MIN_TURN_SPEED, output);
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