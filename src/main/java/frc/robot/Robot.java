// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.concurrent.TimeUnit;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class Robot extends TimedRobot {
  private final XboxController m_controller = new XboxController(0);
  private final XboxController m_controller2 = new XboxController(1);
  private final Drivetrain m_swerve = new Drivetrain();

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  private final CANSparkMax m_intakeMotor1 = new CANSparkMax(9, MotorType.kBrushless);
  private final CANSparkMax m_intakeMotor2 = new CANSparkMax(10, MotorType.kBrushless);

  @Override
  public void autonomousPeriodic() {
    driveWithJoystick(false);
    m_swerve.updateOdometry();
  }

  @Override
  public void teleopInit() {
/*
    try{
    TimeUnit.SECONDS.sleep(10);
    } catch (Exception e) {}
*/    
  }

  @Override
  public void teleopPeriodic() {

/* comment this out and restart the robot with the wheels misaligned to see if the wheels try to align */

    driveWithJoystick(true);
    if (m_controller.getAButton() && m_controller.getYButton()) {
      m_swerve.ResetDrives(); 
    }
  }

  private void driveWithJoystick(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final var xSpeed =
        -m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftY() * 0.5, 0.1))
            * Drivetrain.kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed =
        -m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftX() * 0.5, 0.1))
            * Drivetrain.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot = /* FWF - removed a * -1 here to try to fix the park/rotate problem */
        m_rotLimiter.calculate(MathUtil.applyDeadband(m_controller.getRightX() * 0.25, 0.1))
            * Drivetrain.kMaxAngularSpeed;

    m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative);
/* 
    m_intakeMotor1.set(m_controller2.getLeftY());
    m_intakeMotor2.set(m_controller2.getRightY());

/ */
    int reverse = (m_controller2.getAButton()? 1 : -1);
    m_intakeMotor1.set(reverse * m_controller2.getLeftY());
    m_intakeMotor2.set(m_controller2.getLeftY());


  }

}
