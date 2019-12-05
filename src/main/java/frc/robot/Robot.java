/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private CANSparkMax leftMaster, rightMaster, leftFollower, rightFollower, centerController;

  private XboxController xbox;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    leftMaster = new CANSparkMax(4, MotorType.kBrushless);
    leftMaster.setInverted(true);
    leftFollower = new CANSparkMax(5, MotorType.kBrushless);
    leftFollower.follow(leftMaster);
    rightMaster = new CANSparkMax(1, MotorType.kBrushless);
    rightFollower = new CANSparkMax(3, MotorType.kBrushless);
    rightFollower.follow(rightMaster);
    centerController = new CANSparkMax(2, MotorType.kBrushless);

    xbox = new XboxController(0);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
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

    double rightY = -Math.max(-1, Math.min(1, xbox.getY(Hand.kRight)/Math.sqrt(2)*2));
    double leftY = -Math.max(-1, Math.min(1, xbox.getY(Hand.kLeft)/Math.sqrt(2)*2));
    double rightX = Math.max(-1, Math.min(1, xbox.getX(Hand.kRight)/Math.sqrt(2)*2));
    double leftX = Math.max(-1, Math.min(1, xbox.getX(Hand.kLeft)/Math.sqrt(2)*2));

    rightY = Math.pow(rightY, 3);
    leftY = Math.pow(leftY, 3);
    rightX = Math.pow(rightX, 3);
    leftX = Math.pow(leftX, 3);

    drive(rightY, rightX, leftX);
  }

  private double forwardLimit = 0.25;
  private double sidwaysLimit = 0.3;

  private double angularPercentage = 0.3; // FOR CALCULATIONS (RATIO OF ANGULAR)

  public void drive(double forwardVelocity, double sidewaysVelocity, double angularVelocity) {
    double rightPercentage = forwardVelocity * (1 - angularPercentage)  - angularVelocity * angularPercentage;
    double leftPercentage = forwardVelocity * (1 - angularPercentage) + angularVelocity * angularPercentage;
    double sidewaysPercentage = sidewaysVelocity * (1 - angularPercentage);

    leftMaster.set(leftPercentage * forwardLimit);
    rightMaster.set(rightPercentage * forwardLimit);
    centerController.set(sidewaysPercentage * sidwaysLimit);
  }

  public void driveFieldCentric(double forwardVelocity, double sidewaysVelocity, double angularVelocity, double currentAngle) {
      double modifiedForward = forwardVelocity * Math.cos(-currentAngle) + sidewaysVelocity * Math.sin(-currentAngle);
      double modifiedSideways = forwardVelocity * Math.sin(currentAngle) + sidewaysVelocity * Math.cos(currentAngle);
      drive(modifiedForward, modifiedSideways, angularVelocity);
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
