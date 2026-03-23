// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Motor controller IDs for drivetrain motors
    public static final int LEFT_LEADER_ID = 10;
    public static final int LEFT_FOLLOWER_ID = 11;
    public static final int RIGHT_LEADER_ID = 12;
    public static final int RIGHT_FOLLOWER_ID = 13;

    // CIM free speed 5310 RPM / 8.45 gear ratio × 2π × 0.076m wheel radius / 60
    public static final double MAX_SPEED_MPS = 5.0;
    public static final double DRIVE_KFF = 0.2; // ≈ 1 / MAX_SPEED_MPS
    public static final double DRIVE_KP = 0.1;

    // Proportional gain for encoder-based heading correction when driving straight.
    // Increase if the robot still drifts, decrease if it oscillates.
    public static final double STRAIGHT_KP = 1.5;

    // NEO drivetrain motors need a 40A hard cap to avoid overheating the motors.
    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 40;
    public static final double DRIVE_MOTOR_SECONDARY_CURRENT_LIMIT = 40.0;

    // NavX PID constants for turning commands (TurnToAngle, Spin180)
    public static final double TURN_KP = 0.02;
    public static final double TURN_KI = 0.0;
    public static final double TURN_KD = 0.005;
    public static final double TURN_TOLERANCE_DEGREES = 2.0;
    public static final double MAX_TURN_SPEED = 0.6;
    public static final double MIN_TURN_SPEED = 0.08;

    // Command-layer throttle limiting handles acceleration; keep motor controller ramping off
    // so the robot can decelerate immediately.
    public static final double DRIVE_OPEN_LOOP_RAMP_RATE = 0.0;
    public static final double DRIVE_BROWNOUT_RECOVERY_VOLTAGE = 9.5;
    public static final double DRIVE_BROWNOUT_MIN_VOLTAGE = 7.5;
    public static final double DRIVE_BROWNOUT_MIN_SCALE = 0.45;
  }

  public static final class FuelConstants {
    // Motor controller IDs for Fuel Mechanism motors
    public static final int FLYWHEEL_MOTOR_ID = 18;
    public static final int AUGER_MOTOR_ID = 15;
    public static final int INTAKE_MOTOR_ID = 17;

    // Current limit for fuel mechanism motors (NEOs — 40A protects windings)
    public static final int FLYWHEEL_MOTOR_CURRENT_LIMIT = 40;
    public static final int AUGER_MOTOR_CURRENT_LIMIT = 40;
    public static final int INTAKE_MOTOR_CURRENT_LIMIT = 40;


    // Speed values for various fuel operations (percentage -1.0 to 1.0)
    // These values can be tuned via SmartDashboard during testing
    //INTAKE
    public static final double INTAKING_FLYWHEEL_SPEED = -0.25;     // Feeder pulls balls in - 60% power
    public static final double INTAKING_AUGER_SPEED = 1;  // Launcher pulls balls in (reversed)
    public static final double INTAKING_INTAKE_SPEED = -0.8;
    //LAUNCH
    public static final double LAUNCHING_FLYWHEEL_SPEED = 1;    // Feeder pushes balls out
    public static final double LAUNCHING_AUGER_SPEED = -1.0;  // Launcher shoots balls
    public static final double LAUNCHING_INTAKE_SPEED = -1.0;
    //SPIN UP
    public static final double SPIN_UP_FLYWHEEL_SPEED = 1.0;      // Feeder feeds forward during spin-up
    public static final double SPIN_UP_SECONDS = 1;
    //EJECT
    public static final double EJECT_FLYWHEEL_SPEED = -0.25;       // Feeder pushes balls backward
    public static final double EJECT_AUGER_SPEED = -1.0;      // Launcher ejects balls
    public static final double EJECT_INTAKE_SPEED = 1.0;

    //General climb speed
    public static final double speed = 1;
  }

  public static final class ClimberConstants {
    //Climber Sparks
    //climber one removed
    //public static final int CLIMBER_ONE = 14;
    public static final int CLIMBER_TWO = 16;

    // Current limit for climber motors (NEOs — 40A protects windings)
    public static final int CLIMBER_ONE_CURRENT_LIMIT = 40;
    public static final int CLIMBER_TWO_CURRENT_LIMIT = 40;

    //Climber voltage
    public static final double CLIMBER_DOWN_SPEED = -0.2;
    public static final double CLIMBER_UP_SPEED = 0.2;

    //CLIMBER SPEED
    public static final double speed = 1;
  }

  public static final class SimConstants {
    public static final double TRACK_WIDTH_METERS = 0.546; // 21.5 inches
    public static final double WHEEL_RADIUS_METERS = 0.076; // 6 inch wheels
    public static final double ROBOT_MASS_KG = 50.0; // ~110 lbs
    public static final double DRIVE_GEAR_RATIO = 8.45; // AM14U5 kit chassis
    public static final double DRIVE_MOI = 7.5; // kg*m²
    public static final double LAUNCHER_MOI = 0.005; // kg*m²
    public static final double FEEDER_MOI = 0.003; // kg*m²
  }

  public static final class VisionConstants {
    public static final String LIMELIGHT_NAME = "limelight-brain";
  }

  public static final class OperatorConstants {
    // Port constants for driver and operator controllers. These should match the
    // values in the Joystick tab of the Driver Station software
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;

    // This value is multiplied by the joystick value when rotating the robot to
    // help avoid turning too fast and being difficult to control
    public static final double DRIVE_SCALING = 0.6;
    public static final double ROTATION_SCALING = -1;
    public static final double DRIVE_DEADBAND = 0.15;
    public static final double TURN_DEADBAND = 0.15;
    public static final double THROTTLE_ACCEL_SLEW_RATE = 1.5;
    public static final double TURN_SLEW_RATE = 3.0;
  }
}
