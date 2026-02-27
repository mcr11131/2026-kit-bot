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

    // Encoder counts per revolution for REV Through Bore Encoder
    public static final int ENCODER_CPR = 8192;

    // CIM free speed 5310 RPM / 8.45 gear ratio × 2π × 0.076m wheel radius / 60
    public static final double MAX_SPEED_MPS = 5.0;
    public static final double DRIVE_KFF = 0.2; // ≈ 1 / MAX_SPEED_MPS
    public static final double DRIVE_KP = 0.1;

    //max speed percent for tank control mode (0 to 1)
    public static final double TANK_SPEED_MODIFIER = 0.5;

    // Proportional gain for encoder-based heading correction when driving straight.
    // Increase if the robot still drifts, decrease if it oscillates.
    public static final double STRAIGHT_KP = 1.5;

    // Current limit for drivetrain motors. 80A allows full power operation
    // while still protecting against breaker trips. CIM motors can handle 80A continuous.
    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 80;

    // Ramp rate in seconds from 0 to full throttle. Reduces current spikes and smooths acceleration.
    public static final double DRIVE_OPEN_LOOP_RAMP_RATE = 0.3;
  }

  public static final class FuelConstants {
    // Motor controller IDs for Fuel Mechanism motors
    public static final int FEEDER_MOTOR_ID = 17;
    public static final int INTAKE_LAUNCHER_MOTOR_ID = 15;

    // Current limit for fuel mechanism motors
    public static final int FEEDER_MOTOR_CURRENT_LIMIT = 60;
    public static final int LAUNCHER_MOTOR_CURRENT_LIMIT = 60;

    // Speed values for various fuel operations (percentage -1.0 to 1.0)
    // These values can be tuned via SmartDashboard during testing
    public static final double INTAKING_FEEDER_SPEED = 0.5;     // Feeder pulls balls in - 50% power
    public static final double INTAKING_LAUNCHER_SPEED = -1.0;  // Launcher pulls balls in (reversed)

    public static final double LAUNCHING_FEEDER_SPEED = 1.0;    // Feeder pushes balls out
    public static final double LAUNCHING_LAUNCHER_SPEED = 1.0;  // Launcher shoots balls

    public static final double SPIN_UP_FEEDER_SPEED = 1.0;      // Feeder feeds forward during spin-up
    public static final double SPIN_UP_SECONDS = 1;

    public static final double EJECT_FEEDER_SPEED = -1.0;       // Feeder pushes balls backward
    public static final double EJECT_LAUNCHER_SPEED = 1.0;      // Launcher ejects balls

    //General climb speed
    public static final double speed = 1;
  }

  public static final class ClimberConstants {
    //Climber Sparks
    public static final int CLIMBER_ONE = 14;
    public static final int CLIMBER_TWO = 16;

    // Current limit and nominal voltage for fuel mechanism motors.
    public static final int CLIMBER_ONE_CURRENT_LIMIT = 60;
    public static final int CLIMBER_TWO_CURRENT_LIMIT = 60;

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

  public static final class OperatorConstants {
    // Port constants for driver and operator controllers. These should match the
    // values in the Joystick tab of the Driver Station software
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;

    // This value is multiplied by the joystick value when rotating the robot to
    // help avoid turning too fast and beign difficult to control
    public static final double DRIVE_SCALING = 1;
    public static final double ROTATION_SCALING = 1;
  }
}
