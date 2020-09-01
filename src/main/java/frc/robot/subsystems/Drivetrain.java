package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * The class for the Drivetrain subsystem. This class deals with the movement of the drivetrain
 * as well as gyroscopic movements.
 */
public class Drivetrain extends SubsystemBase {
    public WPI_TalonFX m_leftMaster, m_rightMaster, m_leftSlave, m_rightSlave;
    public AHRS m_navx;

    public DifferentialDriveOdometry m_odometry;

    public Drivetrain() {
        m_leftMaster = new WPI_TalonFX(Constants.LEFT_MASTER);
        m_rightMaster = new WPI_TalonFX(Constants.RIGHT_MASTER);
        m_leftSlave = new WPI_TalonFX(Constants.LEFT_SLAVE);
        m_rightSlave = new WPI_TalonFX(Constants.RIGHT_SLAVE);

        m_leftSlave.follow(m_leftMaster);
        m_rightSlave.follow(m_rightMaster);

        m_navx = new AHRS(SPI.Port.kMXP);

        m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(m_navx.getAngle()));
    }

    public double getValueLeft() {
        return (((m_leftMaster.getSelectedSensorPosition() + m_leftSlave.getSelectedSensorPosition()) / 2) * Constants.GEAR_RATIO) * Constants.TICKS_PER_METER;
    }

    /**
     * @return value of left side of the drivetrain, with regard to gear ratio and meters per tick.
     */
    public double getValueRight() {
        return (((m_rightMaster.getSelectedSensorPosition() + m_rightSlave.getSelectedSensorPosition()) / 2) * Constants.GEAR_RATIO) * Constants.TICKS_PER_METER;
    }

    @Override
    public void periodic() {
        // update robot position
        m_odometry.update(Rotation2d.fromDegrees(m_navx.getAngle()), getValueLeft(), getValueRight());
    }
}