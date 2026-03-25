// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Shooter.ShooterState;
import frc.robot.subsystems.Intake.IntakeState;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  public RobotContainer robotContainer;
  private Command m_AutonomousCommand;

  // Swerve Drive Varibles
  public static final CTREConfigs ctreConfigs = new CTREConfigs();

  public boolean rightBumperWasPressed = false;
  public boolean leftBumperWasPressed = false;
  public boolean AButtonWasPressed = false;
  public boolean YButtonWasPressed = false;

  double limelightAprilTagLastError;

  // subsystems
  public Swerve swerve;
  public Intake intake;
  public Shooter shooter;
  public Vision vision;

   private final Field2d field = new Field2d();
  //public Climber climber;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  public Robot() {
    SignalLogger.enableAutoLogging(false);
    swerve = Swerve.getInstance();
    intake = Intake.getInstance();
    shooter = Shooter.getInstance();
    vision = new Vision("limelight-april");
    // climber = Climber.getInstance();
    robotContainer = new RobotContainer();

     SmartDashboard.putData("Field", field);
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
  @Override
  public void robotPeriodic() {


     CommandScheduler.getInstance().run();

    swerve.swerveCurrents();
    RobotTelemetry();
            SmartDashboard.putBoolean("EEEEEE", false);

  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different
   * autonomous modes using the dashboard. The sendable chooser code works with
   * the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
   * chooser code and
   * uncomment the getString line to get the auto name from the text box below the
   * Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure
   * below with additional strings. If using the SendableChooser make sure to add
   * them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    Command m_AutonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_AutonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_AutonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
@Override
public void autonomousPeriodic() {
    // REMOVED: swerve.swerveOdometry.update(...) — Swerve.periodic() handles this
    RobotTelemetry();
}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {// Destroy Auto Commands When Switching To TeleOP
    if (m_AutonomousCommand != null) {
      CommandScheduler.getInstance().cancel(m_AutonomousCommand);
    }

    shooter.stopLaunchers();
    shooter.state = ShooterState.OFF;

    intake.stop();
    intake.state = IntakeState.OFF;

    swerve.zeroHeading();
    RobotTelemetry();
  }

  /** This function is called periodically during operator control. */
@Override
public void teleopPeriodic() {
    Driver1Controls();
    SwerveDrive(false);
    Driver2Controls();
    RobotTelemetry();
}
  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
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

  // Add Telemetry Data for Robot
  private void RobotTelemetry() {

  }

  private void Driver1Controls() {
    // Back to robot centric while button seven is pushed
    if (Constants.Controllers.driver1.getRawButton(2)) {
      swerve.zeroHeading();
      System.out.println("Gyro reset");
    }
    if (Constants.Controllers.driver1.getRawButton(4)) {
      
    } else {
      limelightAprilTagLastError = 0;
      // System.out.println("limelight button not pressed, setting last tx to 0");
    }
    if (Constants.Controllers.driver1.getRawButton(3)) {//climber up
      //climber.climbUp();
    }
        if (Constants.Controllers.driver1.getRawButton(5)) {//climber down
      //climber.climbDown();
    }
  }

  private void Driver2Controls () {

    // Right Bumper = 6
    // if (Constants.Controllers.driver2Handler.getRawButtonPressEvent(6)) {
    //   if (shooter.state == ShooterState.OFF) {
    //     shooter.startLaunchers();
    //     shooter.state = ShooterState.ON;
    //   }
    //   else {
    //     shooter.stopLaunchers();
    //     shooter.state = ShooterState.OFF;
    //   }
    // }
    if (Constants.Controllers.driver2.getRightBumperButtonPressed() && !rightBumperWasPressed) {
      if (shooter.state == ShooterState.OFF) {
        shooter.startLaunchers();
        shooter.state = ShooterState.ON;
      }
      else {
        shooter.stopLaunchers();
        shooter.state = ShooterState.OFF;
      }
    }
    rightBumperWasPressed = Constants.Controllers.driver2.getRightBumperButtonPressed();

    if (Constants.Controllers.driver2.getRightTriggerAxis() > 0) {
      shooter.spindex();
      shooter.startFeeder();
    }
    else {
      shooter.stopSpindex();
      shooter.stopFeeder();
    }
    
    if (Constants.Controllers.driver2.getPOV() == 270) {
      shooter.turretClockwise();
    }
    else if (Constants.Controllers.driver2.getPOV() == 90) {
      shooter.turretCounterclockwise();
    }
    else {
     shooter.turretStop();
    }

    // Left Bumper = 5
    // if (Constants.Controllers.driver2Handler.getRawButtonPressEvent(5)) {
    //   if (intake.state == IntakeState.OFF) {
    //     intake.intake();
    //     intake.state = IntakeState.ON;
    //   }
    //   else {
    //     intake.stop();
    //     intake.state = IntakeState.OFF;
    //   }
    // }
    if (Constants.Controllers.driver2.getLeftBumperButtonPressed() && !leftBumperWasPressed) {
      if (intake.state == IntakeState.OFF) {
        intake.intake();
        intake.state = IntakeState.ON;
      }
      else {
        intake.stop();
        intake.state = IntakeState.OFF;
      }
    }
    leftBumperWasPressed = Constants.Controllers.driver2.getLeftBumperButtonPressed();

    
    if (Constants.Controllers.driver2.getAButtonPressed()) {
      intake.extend();
    }

    if (Constants.Controllers.driver2.getYButtonPressed()) {
      intake.contract();
    }

    // TODO: Add intake 

    // Constants.Controllers.driver2Handler.update();

  }

  // private void limelightAprilTagAim (boolean isFieldRel) {
  //   double currentGyro = swerve.gyro.getRotation2d().getDegrees();
  //   double mappedAngle = 0.0f;
  //   double angy = ((currentGyro % 360.0f));
  //   if (currentGyro >= 0.0f) {
  //     if (angy > 180) {
  //       mappedAngle = angy - 360.0f;
  //     } else {
  //       mappedAngle = angy;
  //     }
  //   } else {
  //     if (Math.abs(angy) > 180.0f) {
  //       mappedAngle = angy + 360.0f;
  //     } else {
  //       mappedAngle = angy;
  //     }
  //   }
  //   double tx = vision.aprilTagX.getFloat(700);
  //   // System.out.println("tx april: " + tx);
  //   double tx_max = 30.0f; // detemined empirically as the limelights field of view
  //   double error = 0.0f;
  //   double kP = 2.0f; // should be between 0 and 1, but can be greater than 1 to go even faster
  //   double kD = 0.0f; // should be between 0 and 1
  //   double steering_adjust = 0.0f;
  //   double acceptable_error_threshold = 10.0f / 360.0f; // 15 degrees allowable
  //   if (tx != 0.0f) { // use the limelight if it recognizes anything, and use the gyro otherwise
  //     error = 1.0f * (tx / tx_max) * (31.65 / 180); // scaling error between -1 and 1, with 0 being dead on, and 1
  //                                                    // being 180 degrees away
  //   } else {
  //     error = mappedAngle / 180.0f; // scaling error between -1 and 1, with 0 being dead on, and 1 being 180 degrees
  //                                   // away
  //   }
  //   if (limelightAprilTagLastError == 0.0f) {
  //     limelightAprilTagLastError = tx;
  //   }
  //   double error_derivative = error - limelightAprilTagLastError;
  //   limelightAprilTagLastError = tx; // setting limelightlasterror for next loop

  //   if (Math.abs(error) > acceptable_error_threshold) { // PID with a setpoint threshold
  //     steering_adjust = (kP * error + kD * error_derivative);
  //   }

  //   final double xSpeed = MathUtil.applyDeadband(Constants.Controllers.driver1.getRawAxis(1),
  //       Constants.Controllers.stickDeadband);
  //   final double ySpeed = MathUtil.applyDeadband(Constants.Controllers.driver1.getRawAxis(0),
  //       Constants.Controllers.stickDeadband);
  //   swerve.drive(new Translation2d(xSpeed, ySpeed).times(Constants.Swerve.maxSpeed),
  //       steering_adjust * Constants.Swerve.maxAngularVelocity, isFieldRel, false);

  //   // System.out.println("raw angle: " + currentGyro + ", mapped angle: " +
  //   // mappedAngle + ", april tag error: " + error);
  // }


  private void SwerveDrive(boolean isFieldRel) {
    double xSpeed = 0, ySpeed = 0, rot = 0;
    if (Constants.Controllers.driver1.getRawButton(1)) {
        xSpeed = MathUtil.applyDeadband(Constants.Controllers.driver1.getRawAxis(0), Constants.Controllers.stickDeadband);
        ySpeed = MathUtil.applyDeadband(Constants.Controllers.driver1.getRawAxis(1), Constants.Controllers.stickDeadband);
        rot = MathUtil.applyDeadband(Constants.Controllers.driver1.getRawAxis(3), Constants.Controllers.stickDeadband); // negated
    }

    swerve.drive(new Translation2d(xSpeed, ySpeed).times(Constants.Swerve.maxSpeed),
        rot * Constants.Swerve.maxAngularVelocity, isFieldRel, false);
  }

}
