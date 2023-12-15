package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.ADIS16470_IMU;

/*
 *
 *  creating this so we can have the same properties and methods as gyro in the WPI library
 *  the IMU gyro is missing getRotation2d
 * 
 */

public class Gyro_EPRA implements Gyro{
    private final ADIS16470_IMU m_gyro;

    public Gyro_EPRA () {
        m_gyro = new ADIS16470_IMU();
    }
    public void calibrate() {	 //Calibrate the gyro.
        m_gyro.calibrate();
    }
    public double getAngle()	{// Return the heading of the robot in degrees.
        return m_gyro.getAngle();
    }
    public double getRate()	{// Return the rate of rotation of the gyro.
        return m_gyro.getRate();
    }
    public Rotation2d getRotation2d()	{// Return the heading of the robot as a Rotation2d.
        return new Rotation2d(Math.toRadians(m_gyro.getAngle()));
    }
    public void reset()	{// Reset the gyro.
        m_gyro.reset();
    }
    public void close () {
        m_gyro.close();
    }    
}
