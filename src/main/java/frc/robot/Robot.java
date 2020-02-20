/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private TalonFX leftMaster, rightMaster, leftFollower, rightFollower, centerController;
  private TalonSRX gyroHost, intakeArm;
  private VictorSPX intakeRoller;
  private PigeonIMU gyro;
  private XboxController xbox;
  private double currentHeading, previousHeading;
  private boolean driveMode;

  private boolean limelightHasValidTarget = false;
  private double limelightDriveCommand = 0.0;
  private double limelightSteerCommand = 0.0;
  private boolean limelightLight = false;

  public enum LimelightState {
    VISION_TARGET,
    BALL
  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    leftMaster = new TalonFX(Constants.LEFT_MASTER_ID);
    leftMaster.setInverted(Constants.LEFT_INVERTED);
    leftMaster.configStatorCurrentLimit(Constants.TALON_CURRENT_LIMIT);
    leftFollower = new TalonFX(Constants.LEFT_SLAVE_ID);
    leftFollower.follow(leftMaster);
    leftFollower.setInverted(Constants.LEFT_INVERTED);
    leftFollower.configStatorCurrentLimit(Constants.TALON_CURRENT_LIMIT);

    rightMaster = new TalonFX(Constants.RIGHT_MASTER_ID);
    rightMaster.setInverted(Constants.RIGHT_INVERTED);
    rightMaster.configStatorCurrentLimit(Constants.TALON_CURRENT_LIMIT);
    rightFollower = new TalonFX(Constants.RIGHT_SLAVE_ID);
    rightFollower.follow(rightMaster);
    rightFollower.setInverted(Constants.RIGHT_INVERTED);
    rightFollower.configStatorCurrentLimit(Constants.TALON_CURRENT_LIMIT);

    centerController = new TalonFX(Constants.CENTER_ID);
    centerController.setInverted(false);
    centerController.configStatorCurrentLimit(Constants.TALON_CURRENT_LIMIT);

    gyroHost = new TalonSRX(Constants.HOST_TALON_ID);
    gyroHost.configFactoryDefault();
    gyro = new PigeonIMU(gyroHost);

    xbox = new XboxController(0);

    currentHeading = 0.0;
    previousHeading = getIMUYPR()[0];

    intakeArm = new TalonSRX(7);
    intakeRoller = new VictorSPX(10);

    intakeArm.overrideLimitSwitchesEnable(true);

    driveMode = true;
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putString("driveMode", (driveMode) ? "Static Drive" : "Field Centric");
    SmartDashboard.putNumber("heading", currentHeading);
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString line to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure below with additional strings. If using the SendableChooser
   * make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    double rightY = -Math.max(-1, Math.min(1, xbox.getY(Hand.kRight) / Math.sqrt(2) * 2));
    double leftY = -Math.max(-1, Math.min(1, xbox.getY(Hand.kLeft) / Math.sqrt(2) * 2));
    double rightX = Math.max(-1, Math.min(1, xbox.getX(Hand.kRight) / Math.sqrt(2) * 2));
    double leftX = -Math.max(-1, Math.min(1, xbox.getX(Hand.kLeft) / Math.sqrt(2) * 2));

    rightY = Helper.deadband(Helper.round(Math.pow(rightY, 3), 2), Constants.JOYSTICK_DEADBAND);
    leftY = Helper.deadband(Helper.round(Math.pow(leftY, 3), 2), Constants.JOYSTICK_DEADBAND);
    rightX = Helper.deadband(Helper.round(Math.pow(rightX, 3), 2), Constants.JOYSTICK_DEADBAND);
    leftX = Helper.deadband(Helper.round(Math.pow(leftX, 3), 2), Constants.JOYSTICK_DEADBAND);

    if (xbox.getBackButton()) {
      currentHeading = 0.0;
    } else {
      currentHeading += getIMUYPR()[0] - previousHeading;
      previousHeading = getIMUYPR()[0];
    }

    if (xbox.getXButtonPressed()) {
      driveMode = !driveMode;
      // System.out.println((driveMode) ? "Static Drive" : "Field Centric");
    }

    if (xbox.getBButton()){
      intakeRoller.set(VictorSPXControlMode.PercentOutput, Helper.deadband(xbox.getTriggerAxis(Hand.kLeft), 0.03));
    } else {
      intakeRoller.set(VictorSPXControlMode.PercentOutput, -Helper.deadband(xbox.getTriggerAxis(Hand.kLeft), 0.03));
    }

    int pov = xbox.getPOV();
    if(pov == 0){
      intakeArm.set(ControlMode.PercentOutput, -0.15);
    }
    if(pov == 180){
      intakeArm.set(ControlMode.PercentOutput, 0.075);
    }
    if(pov == -1){
      intakeArm.set(ControlMode.PercentOutput, 0);
    }

    if (xbox.getBumper(Hand.kLeft)) {
      limelightLight = true;
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(limelightLight ? 3 : 1);
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
      updateLimelightTracking(LimelightState.VISION_TARGET);

      if (limelightHasValidTarget) {
        drive(0.4 * limelightDriveCommand + 0.6 * rightY, rightX, 0.5 * limelightSteerCommand + 0.7 * leftX);
        // System.out.println("Drive value: " + limelightDriveCommand + " Steer value: "
        // + limelightSteerCommand);
      } else {
        System.out.println("Target Not Found");
      }
    } else if (xbox.getBumper(Hand.kRight)) {
      limelightLight = true;
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1);
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(limelightLight ? 3 : 1);
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
      updateLimelightTracking(LimelightState.BALL);
      if (limelightHasValidTarget) {
        drive(0.4 * limelightDriveCommand + 0.6 * rightY, rightX, 0.5 * limelightSteerCommand + 0.7 * leftX);
        // System.out.println("Drive value: " + limelightDriveCommand + " Steer value: "
        // + limelightSteerCommand);
      } else {
        System.out.println("Target Not Found");
      }
    } else {
      limelightLight = false;
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(limelightLight ? 3 : 1);
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);

      if (driveMode) {
        drive(rightY, rightX, leftX); // Static Drive
      } else {
        // driveFieldCentric(rightY, rightX, leftX, Helper.round(currentHeading, 2));
        driveFieldCentric(rightY, rightX, leftX, (int) currentHeading);
      }
    }

    // 19600 raw sensor units per 100ms; 2048 CPR talon fx; 45.8205810234 m/s
    SmartDashboard.putNumber("Velocity", (leftMaster.getSelectedSensorVelocity() + rightMaster.getSelectedSensorVelocity()) / 2);
    // gyroHost.set(ControlMode.PercentOutput, leftY);
  }

  /**
   * This function implements a simple method of generating driving and steering
   * commands based on the tracking data from a limelight camera.
   */
  public void updateLimelightTracking(LimelightState state) {
    // These numbers must be tuned for your Robot! Be careful!
    final double STEER_K = 0.03; // how hard to turn toward the target
    final double DRIVE_K = 0.26; // how hard to drive fwd toward the target
    final double DESIRED_TARGET_AREA = (state == LimelightState.VISION_TARGET) ? 10.0 : 50.0;//10.0; // Area of the target when the robot reaches the wall
    final double MAX_DRIVE = 0.7; // Simple speed limit so we don't drive too fast

    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
    double ts = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ts").getDouble(0);

    if (tv < ((state == LimelightState.VISION_TARGET) ? 1.0 : 0.7)
        || (state == LimelightState.BALL && (Math.abs(tx) > 25 || Math.abs(ty) > 18.5))) {
      limelightHasValidTarget = false;
      limelightDriveCommand = 0.0;
      limelightSteerCommand = 0.0;
      return;
    }

    limelightHasValidTarget = true;

    // Start with proportional steering
    double steer_cmd = tx * STEER_K;
    limelightSteerCommand = steer_cmd;

    // try to drive forward until the target area reaches our desired area
    double drive_cmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K;

    // don't let the robot drive too fast into the goal
    if (drive_cmd > MAX_DRIVE) {
      drive_cmd = MAX_DRIVE;
    }
    limelightDriveCommand = drive_cmd;
  }

  private double forwardLimit = 0.5;
  private double sidwaysLimit = 0.5; // 0.4

  private double angularPercentage = 0.35; // FOR CALCULATIONS (RATIO OF ANGULAR)

  public void drive(double forwardVelocity, double sidewaysVelocity, double angularVelocity) {
    double rightPercentage = forwardVelocity * (1 - angularPercentage) + angularVelocity * angularPercentage;
    double leftPercentage = forwardVelocity * (1 - angularPercentage) - angularVelocity * angularPercentage;
    double sidewaysPercentage = sidewaysVelocity * (1 - angularPercentage);

    leftMaster.set(ControlMode.PercentOutput, leftPercentage * forwardLimit);
    rightMaster.set(ControlMode.PercentOutput, rightPercentage * forwardLimit);
    centerController.set(ControlMode.PercentOutput, sidewaysPercentage * sidwaysLimit);
  }

  public void driveFieldCentric(double forwardVelocity, double sidewaysVelocity, double angularVelocity,
      double currentAngle) {
    // System.out.println("inputForward: " + forwardVelocity + ", inputSideways: " +
    // sidewaysVelocity + ", angularVel: " + angularVelocity + ", angle: " +
    // currentAngle);
    double angleRad = Math.toRadians(currentAngle);
    double modifiedForward = forwardVelocity * Math.cos(angleRad) + sidewaysVelocity * Math.sin(-angleRad);
    double modifiedSideways = forwardVelocity * Math.sin(angleRad) + sidewaysVelocity * Math.cos(angleRad);
    // System.out.println("forward: " + modifiedForward + ", sideways: " +
    // modifiedSideways);
    drive(modifiedForward, modifiedSideways, angularVelocity);
  }

  // Get Yaw Pitch and Roll from gyro
  public double[] getIMUYPR() {
    double[] ypr = new double[3];
    gyro.getYawPitchRoll(ypr);
    return ypr;
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
