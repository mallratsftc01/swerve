// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain {
  public static final double kMaxSpeed = 4.47; // was 3 meters per second
  public static final double kMaxAngularSpeed = 4.41 * 2 * Math.PI; // was Math.PI for 1/2 rotation per second


        // these are distance from the center to the wheel in meters. .381 is 1.25 feet or 16 inches
        // test swerve drive has 33 inch diagonals
/*
        private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
        private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
        private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
        private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);
*/
private final Translation2d m_frontLeftLocation = new Translation2d(0.4445, 0.4445);
private final Translation2d m_frontRightLocation = new Translation2d(0.4445, -0.4445);
private final Translation2d m_backLeftLocation = new Translation2d(-0.4445, 0.4445);
private final Translation2d m_backRightLocation = new Translation2d(-0.4445, -0.4445);

  private final SwerveModule m_frontLeft = new SwerveModule(3, 4, 1);
  private final SwerveModule m_frontRight = new SwerveModule(1, 2, 0);
  private final SwerveModule m_backLeft = new SwerveModule(5, 6, 2);
  private final SwerveModule m_backRight = new SwerveModule(7, 8, 3);

//  private final Gyro_EPRA m_gyro = new Gyro_EPRA();
private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          m_kinematics,
          m_gyro.getRotation2d(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
          });

  public Drivetrain() {
    m_gyro.reset();
  }

  public void ResetDrives () {

    /* 
    m_frontLeft.resetEncoder();
    m_frontRight.resetEncoder();
    m_backLeft.resetEncoder();
    m_backRight.resetEncoder();
    */
    m_gyro.reset();

    SmartDashboard.putString("Gyro has been reset", java.time.LocalTime.now().toString());
  }
  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);

SmartDashboard.putString("gyro", m_gyro.getRotation2d().toString());
//SmartDashboard.putString("kine", m_kinematics.)


    SmartDashboard.putString("module 0", swerveModuleStates[0].toString());
    SmartDashboard.putString("module 1", swerveModuleStates[1].toString());
    SmartDashboard.putString("module 2", swerveModuleStates[2].toString());
    SmartDashboard.putString("module 3", swerveModuleStates[3].toString());
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        });
  }
}
