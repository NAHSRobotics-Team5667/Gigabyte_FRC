/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // drive constants
    public static final int LEFT_MASTER = 0; // placeholder
    public static final int RIGHT_MASTER = 1; // placeholder
    public static final int LEFT_SLAVE = 2; // placeholder
    public static final int RIGHT_SLAVE = 3; // placeholder

    public static final double WHEEL_RADIUS_INCHES = 6;
    public static final double WHEEL_RADIUS_METERS = 6 / 39.37; // convert inches to meters
    public static final double GEAR_RATIO = 1; // placeholder, driving/driven
    public static final double TICKS_PER_METER = (2048 / (WHEEL_RADIUS_METERS * Math.PI)) * GEAR_RATIO;
    public static final double TICKS_PER_INCH = (2048 / (WHEEL_RADIUS_INCHES * Math.PI)) * GEAR_RATIO;
}
