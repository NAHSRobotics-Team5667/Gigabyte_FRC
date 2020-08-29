package frc.lib.motion;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import frc.lib.geometry.RobotPoint;

/**
 * A localizer class to keep track of the robot's position at all times
 * for a more efficient integration of autonomy into an FRC robot. This
 * can be utilized during the teleoperated control period for drivers to
 * use as an external reference of the field.
 */


public class TankLocalizer {
    public double x = 0;
    public double y = 0;
    public double theta = 0;
    public double distance = 0;

    public Encoder m_leftMaster;
    public Encoder m_leftSlave;
    public Encoder m_rightMaster;
    public Encoder m_rightSlave;
    public AHRS m_gyro;
    
    public final int WHEEL_DIAMETER = 6;
    public final double TICKS_PER_INCH = 1024 / (WHEEL_DIAMETER * Math.PI);

    /**
     * 
     * @param m_leftMaster  left master wheel encoder
     * @param m_rightMaster right master wheel encoder
     * @param m_leftSlave   left slave wheel encoder
     * @param m_rightSlave  right slave wheel encoder
     * @param m_gyro        AHRS imu
     */
    public TankLocalizer(Encoder m_leftMaster, Encoder m_rightMaster, Encoder m_leftSlave, Encoder m_rightSlave, AHRS m_gyro) {
        this.m_leftMaster = m_leftMaster;
        this.m_rightMaster = m_rightMaster;
        this.m_leftSlave = m_leftSlave;
        this.m_rightSlave = m_rightSlave;
        this.m_gyro = m_gyro;
    }

    /**
     * @return average of left encoder values
     */
    public double getEncoderLeft() {
        return ((double) (m_leftMaster.get() + m_leftSlave.get()) / 2) / TICKS_PER_INCH;
    }

    /**
     * @return average of right encoder values
     */
    public double getEncoderRight() {
        return ((double) (m_rightMaster.get() + m_rightSlave.get()) / 2) / TICKS_PER_INCH;
    }

    /**
     * @param startPosition starting position of the robot on the field
     */
    public void setRobotStart(RobotPoint startPosition) {
        this.x = startPosition.x;
        this.y = startPosition.y;
        this.theta= startPosition.theta;
    }

    /**
     * @return the current robot position in inches on the field
     */
    public RobotPoint getRobotPosition() {
        double position = (getEncoderLeft() + getEncoderRight()) / 2;
        distance = position - distance;

        theta = m_gyro.getAngle();
        for (int i = 1; i * 360 < theta; i++) theta -= 360;

        x += distance * Math.cos(theta);
        y += distance * Math.sin(theta);

        return new RobotPoint(x, y, theta);
    }
}