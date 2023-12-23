// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Preferences;
//import edu.wpi.first.wpilibj.AnalogEncoder;

public class SwerveModule {
  private static final double kWheelRadius = 0.0508;
  private static final int kEncoderResolution = 4096;

  private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration =
      2 * Math.PI; // radians per second squared

  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private final RelativeEncoder m_driveEncoder;
  private final AnalogInput m_turningEncoder;

  private double encoderOffset = 0;
  
  private int moduleNumber = 0;
  private final RelativeEncoder m_turningEncoderREV;

  // FWF - stole this from 6328's code, each gear reduction written out. Final is 6.75. 39.37 converts inches to meters so we can be european fancy
  //private final double driveAfterEncoderReduction = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
  private final double driveAfterEncoderReduction = (4.0 / 39.37) * Math.PI * (1/6.75);

  private final double turnAfterEncoderReduction = -1 * (7/150);
  
  // Gains are for example purposes only - must be determined for your own robot!
  private final PIDController m_drivePIDController = new PIDController(1, 0, 0);

  // Gains are for example purposes only - must be determined for your own robot!
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          16.70,
          0,
          0 ,
          new TrapezoidProfile.Constraints(
              kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

  /**
   * Constructs a SwerveModule with a drive motor, turning motor and turning encoder.
   *
   * @param driveMotorChannel PWM output for the drive motor.
   * @param turningMotorChannel PWM output for the turning motor.
   * @param turningEncoderChannelA DIO input for the turning encoder channel A
   */
  public SwerveModule(int driveMotorChannel, int turningMotorChannel, int turningEncoderChannelA) {
    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    m_driveMotor.setInverted (true);
    m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);
    m_turningMotor.setInverted (true);

    m_driveEncoder = m_driveMotor.getEncoder();
    m_turningEncoder = new AnalogInput(turningEncoderChannelA);

    m_turningEncoderREV = m_turningMotor.getEncoder();

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    m_driveEncoder.setPositionConversionFactor(driveAfterEncoderReduction);
    m_driveEncoder.setVelocityConversionFactor(driveAfterEncoderReduction / 60.0);

    m_turningEncoderREV.setPositionConversionFactor(turnAfterEncoderReduction);

    // Set the distance (in this case, angle) in radians per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * pi) divided by the
    // encoder resolution.
//    m_turningEncoder.setDistancePerRotation(-2 * Math.PI);

    moduleNumber = turningEncoderChannelA;

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

        // load the encoder offset
//    encoderOffset = Preferences.getDouble("encoder" + moduleNumber, encoderOffset);
// hard coding the offset because its better?
switch (moduleNumber) {
  case 0: 
  encoderOffset = 0.034; //6.248;
  break;
  case 1: 
  encoderOffset = 2.17; //4.101;
  break;
  case 2: 
  encoderOffset = 3.31; //2.967;
  break;
  case 3: 
  encoderOffset = 0.5; //5.724;
  break;
}

  
 SmartDashboard.putNumber("offset" + moduleNumber, encoderOffset);
    /* set the target state to the current state */
    SwerveModuleState tmpState = new SwerveModuleState (0, getPosition().angle);
//    setDesiredState(tmpState);
  }
  private double encoderValue () {
    var retVal =  m_turningEncoder.getVoltage() / RobotController.getVoltage5V(); // convert voltage to %
    retVal = 2.0 * Math.PI * retVal;    // get % of circle encoder is reading
SmartDashboard.putNumber("raw encoder deg " + moduleNumber, Math.round(retVal * (180/Math.PI)));

    retVal = (retVal + encoderOffset) % (2.0 * Math.PI);    // apply offset for this encoder and map it back onto [0, 2pi]
      // might need this so we're in the same range as the pid controller is expecting.
//    retVal = retVal - Math.PI;

SmartDashboard.putNumber("encoder " + moduleNumber, Math.round(100 *retVal)/100.0);
SmartDashboard.putNumber("encoder " + moduleNumber + " deg", Math.round(retVal *(180/Math.PI)));

    return (retVal);
}

  public void resetEncoder(){
    m_driveMotor.getEncoder().setPosition(0.0);

        // get the turning encoder and write it to preferences
    encoderOffset = m_turningEncoder.getVoltage() / RobotController.getVoltage5V(); // convert voltage to %
    encoderOffset = 2.0 * Math.PI * encoderOffset;    // get % of circle encoder is reading
    SmartDashboard.putNumber("raw offset" + moduleNumber, encoderOffset);
    SmartDashboard.putNumber("raw offset Deg " + moduleNumber, Math.round(encoderOffset * (180/Math.PI)));
    encoderOffset = (2.0 * Math.PI) - encoderOffset;
//    Preferences.initDouble("encoder" + moduleNumber, encoderOffset);
    Preferences.setDouble("encoder" + moduleNumber, encoderOffset * (180/Math.PI));
    SmartDashboard.putNumber("new offset" + moduleNumber, Math.round(encoderOffset * (180/Math.PI)));
  }
  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveEncoder.getVelocity(), new Rotation2d(encoderValue()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveEncoder.getPosition(), new Rotation2d(encoderValue()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, new Rotation2d(encoderValue()));

    // Calculate the drive output from the drive PID controller.

      /* this bit from the example uses driveencoder.getrate() - Get the current rate of the 
       * encoder. Units are distance per second as scaled by the value from setDistancePerPulse().
       * rev encoder doesn't have this value only 
       * .getVelocity - Get the velocity of the motor. This returns the native units of 'rotations per second'
       *  by default, and can be changed by a scale factor using setVelocityConversionFactor().
       */

    final double driveOutput =
        m_drivePIDController.calculate(m_driveEncoder.getVelocity(), state.speedMetersPerSecond);

    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);
    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        m_turningPIDController.calculate(encoderValue(), state.angle.getRadians());

    final double turnFeedforward =
        m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);
SmartDashboard.putNumber("target " + moduleNumber, state.angle.getDegrees());

    m_driveMotor.setVoltage(driveOutput + driveFeedforward);
    m_turningMotor.setVoltage(turnOutput + turnFeedforward);
  }
}
