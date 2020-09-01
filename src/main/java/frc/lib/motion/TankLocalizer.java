package frc.lib.motion;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import frc.lib.geometry.*;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

/**
 * A localizer class to keep track of the robot's position at all times
 * for a more efficient integration of autonomy into an FRC robot. This
 * can be utilized during the teleoperated control period for drivers to
 * use as an external reference of the field.
 */
public class TankLocalizer {
    public static double x = 0;
    public static double y = 0;
    public static double theta = 0;
    public static double distance = 0;
    public static double position = 0;

    public static WPI_TalonFX m_leftMaster;
    public static WPI_TalonFX m_leftSlave;
    public static WPI_TalonFX m_rightMaster;
    public static WPI_TalonFX m_rightSlave;
    public static AHRS m_navx;

    /**
     * @param m_drivetrain drivetrain subsystem in use
     */
    public TankLocalizer(Drivetrain m_drivetrain) {
        m_leftMaster = m_drivetrain.m_leftMaster;
        m_rightMaster = m_drivetrain.m_rightMaster;
        m_leftSlave = m_drivetrain.m_leftSlave;
        m_rightSlave = m_drivetrain.m_rightSlave;
        m_navx = m_drivetrain.m_navx;
    }

    /**
     * @return average of left encoder values
     */
    public static double getValueLeft() { // TODO: account for gear ratio
        return ((double) (m_leftMaster.getSelectedSensorPosition() + m_leftSlave.getSelectedSensorPosition()) / 2) / Constants.TICKS_PER_INCH;
    }

    /**
     * @return average of right encoder values
     */
    public static double getValueRight() { // TODO: account for gear ratio
        return ((double) (m_rightMaster.getSelectedSensorPosition() + m_rightSlave.getSelectedSensorPosition()) / 2) / Constants.TICKS_PER_INCH;
    }

    /**
     * @param startPosition starting position of the robot on the field
     */
    public static void setStartPosition(Pose2d startPosition) {
        x = startPosition.x;
        y = startPosition.y;
        theta = startPosition.theta;
    }

    /**
     * @return the current robot position in inches on the field
     */
    public static Pose2d getRobotPosition() {
        // using encoders
        // double position = (getValueLeft() + getValueRight()) / 2;

        // using accelerometer
        // TODO: get velocity of robot
        double velocity = m_navx.getWorldLinearAccelX(); // change to Y if orientation is different
        position += velocity;

        distance = position - distance;

        theta = m_navx.getAngle();
        for (int i = 1; i * 360 < theta; i++) theta -= 360;

        x += distance * Math.cos(theta);
        y += distance * Math.sin(theta);

        return new Pose2d(x, y, theta);
    }
}