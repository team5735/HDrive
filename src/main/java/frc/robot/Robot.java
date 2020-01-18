/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.networktables.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private CANSparkMax leftMaster, rightMaster, leftFollower, rightFollower, centerController;
  private TalonSRX gyroHost;
  private PigeonIMU gyro;
  private XboxController xbox;
  private double currentHeading, previousHeading;
  private boolean driveState;

  private boolean limelightHasValidTarget = false;
  private double limelightDriveCommand = 0.0;
  private double limelightSteerCommand = 0.0;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    leftMaster = new CANSparkMax(4, MotorType.kBrushless);
    leftMaster.setInverted(false);
    leftFollower = new CANSparkMax(5, MotorType.kBrushless);
    leftFollower.follow(leftMaster);
    leftFollower.setInverted(true);
    rightMaster = new CANSparkMax(1, MotorType.kBrushless);
    rightMaster.setInverted(true);
    rightFollower = new CANSparkMax(3, MotorType.kBrushless);
    rightFollower.follow(rightMaster);
    rightFollower.setInverted(true);
    centerController = new CANSparkMax(2, MotorType.kBrushless);
    centerController.setInverted(true);
    gyroHost = new TalonSRX(6);
    gyro = new PigeonIMU(gyroHost);

    xbox = new XboxController(0);

    currentHeading = 0.0;
    previousHeading = getIMUYPR()[0];

    driveState = false ;
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
    SmartDashboard.putBoolean("driveMode", driveState);
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

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

    updateLimelightTracking();

    double rightY = -Math.max(-1, Math.min(1, xbox.getY(Hand.kRight) / Math.sqrt(2) * 2));
    double leftY = -Math.max(-1, Math.min(1, xbox.getY(Hand.kLeft) / Math.sqrt(2) * 2));
    double rightX = Math.max(-1, Math.min(1, xbox.getX(Hand.kRight) / Math.sqrt(2) * 2));
    double leftX = Math.max(-1, Math.min(1, xbox.getX(Hand.kLeft) / Math.sqrt(2) * 2));

    rightY = Helper.deadband(Helper.round(Math.pow(rightY, 3), 2), Constants.JOYSTICK_DEADBAND);
    leftY = Helper.deadband(Helper.round(Math.pow(leftY, 3), 2), Constants.JOYSTICK_DEADBAND);
    rightX = Helper.deadband(Helper.round(Math.pow(rightX, 3), 2), Constants.JOYSTICK_DEADBAND);
    leftX = Helper.deadband(Helper.round(Math.pow(leftX, 3), 2), Constants.JOYSTICK_DEADBAND);

    if (xbox.getYButtonPressed()) {
      currentHeading = 0.0;
    } else {
      currentHeading += getIMUYPR()[0] - previousHeading;
      previousHeading = getIMUYPR()[0];
    }

    if (xbox.getXButtonPressed()) {
      driveState = !driveState;
      System.out.println((driveState) ? "Static Drive" : "Field Centric");
    }


    if (driveState) {
      boolean auto = xbox.getAButton();

      if (auto) {
        if (limelightHasValidTarget) {
          // m_Drive.arcadeDrive(0.5 * m_LimelightDriveCommand, 0.5 *
          // m_LimelightSteerCommand);
          drive(0.4 * limelightDriveCommand + 0.6 * rightY, rightX, 0.5 * limelightSteerCommand + 0.7 * leftX);
          System.out.println("Drive value: " + limelightDriveCommand + " Steer value: " + limelightSteerCommand);
        } else {
          System.out.println("Target Not Found");
        }
      } else {
        drive(rightY, rightX, 0.3 * leftX); // Static Drive
      }
    } else {
      // driveFieldCentric(rightY, rightX, leftX, Helper.round(currentHeading, 2));
      driveFieldCentric(rightY, rightX, leftX, (int) currentHeading);
    }
  }

  /**
   * This function implements a simple method of generating driving and steering
   * commands based on the tracking data from a limelight camera.
   */
  public void updateLimelightTracking() {
    // These numbers must be tuned for your Robot! Be careful!
    final double STEER_K = 0.03; // how hard to turn toward the target
    final double DRIVE_K = 0.26; // how hard to drive fwd toward the target
    final double DESIRED_TARGET_AREA = 10.0; // Area of the target when the robot reaches the wall
    final double MAX_DRIVE = 0.7; // Simple speed limit so we don't drive too fast

    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
    double ts = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ts").getDouble(0);

    if (tv < 1.0) {
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
  private double sidwaysLimit = 0.52; // 0.4

  private double angularPercentage = 0.3; // FOR CALCULATIONS (RATIO OF ANGULAR)

  public void drive(double forwardVelocity, double sidewaysVelocity, double angularVelocity) {
    double rightPercentage = forwardVelocity * (1 - angularPercentage) + angularVelocity * angularPercentage;
    double leftPercentage = forwardVelocity * (1 - angularPercentage) - angularVelocity * angularPercentage;
    double sidewaysPercentage = sidewaysVelocity * (1 - angularPercentage);

    leftMaster.set(leftPercentage * forwardLimit);
    rightMaster.set(rightPercentage * forwardLimit);
    centerController.set(sidewaysPercentage * sidwaysLimit);
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
