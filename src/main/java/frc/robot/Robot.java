// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  // HUB status tracking for fuel scoring windows
  private final HubTracker hubTracker = new HubTracker();

  // Endgame rumble alert
  private boolean endgameRumbled = false;
  private double rumbleStartTime = -1;
  private static final double ENDGAME_TIME = 30.0; // seconds remaining when endgame starts
  private static final double RUMBLE_DURATION = 5.0;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {


    // Start comprehensive data logging to USB drive
    // Logs will be saved to /home/lvuser/logs/ on the roboRIO
    DataLogManager.start();

    // Log all DriverStation data (joystick inputs, match info, etc.)
    DriverStation.startDataLog(DataLogManager.getLog());

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // Add Limelight camera stream to Shuffleboard/SmartDashboard
    HttpCamera limelightFeed = new HttpCamera("limelight", "http://10.111.31.200:5800");
    CameraServer.addCamera(limelightFeed);

    // Log all NetworkTables changes (SmartDashboard values, subsystem status)
    DataLogManager.logNetworkTables(true);
    DataLogManager.log("Robot initialized - logging active");

    // Publish the full button layout so drivers can reference controls on-screen
    SmartDashboard.putString("Controls",
        "LB(5): Toggle Intake | RB(6): Toggle Launch | A(2): Eject (hold) | "
        + "X(3): AprilTag Align+Score (hold) | Back(7): Reverse Direction | "
        + "POV Up/Down: Climb Up/Down | Btn1: Climb Up");

    // Log command starts and finishes for debugging
    CommandScheduler.getInstance().onCommandInitialize(
        command -> { String msg = "[CMD] Started: " + command.getName(); System.out.println(msg); DataLogManager.log(msg); });
    CommandScheduler.getInstance().onCommandFinish(
        command -> { String msg = "[CMD] Finished: " + command.getName(); System.out.println(msg); DataLogManager.log(msg); });

    // Used to track usage of Kitbot code, please do not remove.
    HAL.report(tResourceType.kResourceType_Framework, 10);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  private boolean limelightDiagDone = false;

  @Override
  public void robotPeriodic() {
    // One-time diagnostic: print all NT tables containing "limelight" to find the correct name
    // M5 fix: set flag after loop so it doesn't run every 20ms if no limelight table found
    if (!limelightDiagDone) {
      var tables = NetworkTableInstance.getDefault().getTable("").getSubTables();
      for (String table : tables) {
        if (table.toLowerCase().contains("limelight") || table.toLowerCase().contains("lime")) {
          System.out.println("[LIMELIGHT DIAG] Found NT table: " + table);
          var entries = NetworkTableInstance.getDefault().getTable(table).getKeys();
          for (String entry : entries) {
            System.out.println("[LIMELIGHT DIAG]   " + table + "/" + entry);
          }
        }
      }
      limelightDiagDone = true;
    }
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // Display battery voltage for driver awareness
    double voltage = RobotController.getBatteryVoltage();
    SmartDashboard.putNumber("Battery Voltage", voltage);

    // Log warning if voltage drops dangerously low
    if (voltage < 8.0) {
      DataLogManager.log("WARNING: Battery voltage critically low: " + String.format("%.2f", voltage) + "V");
    }
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    DataLogManager.log(">> DISABLED");
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    DataLogManager.log(">> AUTO INIT");
    hubTracker.reset();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    DataLogManager.log("Auto selected: " + (m_autonomousCommand != null ? m_autonomousCommand.getName() : "NONE"));

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
     CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    DataLogManager.log(">> TELEOP INIT");

    // Read FMS game data to determine HUB shift order
    hubTracker.readGameData();
    endgameRumbled = false;
    rumbleStartTime = -1;
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    double matchTime = DriverStation.getMatchTime();

    // Update HUB active/inactive status on dashboard
    hubTracker.update(matchTime);

    // Vibrate controller when endgame starts (30s remaining)
    if (matchTime <= ENDGAME_TIME && matchTime > 0 && !endgameRumbled) {
      endgameRumbled = true;
      rumbleStartTime = Timer.getFPGATimestamp();
      m_robotContainer.setRumble(1.0);
      DataLogManager.log(">> ENDGAME ALERT - Time to climb!");
    }

    // Stop rumble after 5 seconds
    if (rumbleStartTime > 0 && (Timer.getFPGATimestamp() - rumbleStartTime) >= RUMBLE_DURATION) {
      m_robotContainer.setRumble(0);
      rumbleStartTime = -1;
    }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
