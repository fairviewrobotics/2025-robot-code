/* Black Knights Robotics (C) 2025 */
package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class DrivetrainConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double MAX_SPEED_METERS_PER_SECOND = 4.8;
    public static final double MAX_ANGULAR_SPEED = 2 * Math.PI; // radians per second

    public static final double DIRECTION_SLEW_RATE = 18.0; // rads/sec
    public static final double MAGNITUDE_SLEW_RATE = 22.0; // meters/second (1 = 100%)
    public static final double ROTATIONAL_SLEW_RATE = 30.0; // meters/second (1 = 100%)

    // Chassis configuration
    public static final double TRACK_WIDTH = Units.inchesToMeters(26.5);
    // Distance between centers of right and left wheels on robot
    public static final double WHEEL_BASE = Units.inchesToMeters(26.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics DRIVE_KINEMATICS =
            new SwerveDriveKinematics(
                    new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
                    new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
                    new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
                    new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double FRONT_LEFT_CHASSIS_ANGULAR_OFFSET = -Math.PI / 2;
    public static final double FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET = 0;
    public static final double REAR_LEFT_CHASSIS_ANGULAR_OFFSET = Math.PI;
    public static final double REAR_RIGHT_CHASSIS_ANGULAR_OFFSET = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int FRONT_LEFT_DRIVING_CAN_ID = 11;
    public static final int REAR_LEFT_DRIVING_CAN_ID = 13;
    public static final int FRONT_RIGHT_DRIVING_CAN_ID = 15;
    public static final int REAR_RIGHT_DRIVING_CAN_ID = 17;

    public static final int FRONT_LEFT_TURNING_CAN_ID = 10;
    public static final int REAR_LEFT_TURNING_CAN_ID = 12;
    public static final int FRONT_RIGHT_TURNING_CAN_ID = 14;
    public static final int REAR_RIGHT_TURNING_CAN_ID = 16;

    public static final boolean GYRO_REVERSED = false;

    // MODULE CONSTANTS

    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int DRIVING_MOTOR_PINION_TEETH = 14;
    public static final int BEVEL_GEAR_TEETH = 45;
    public static final int FIRST_STAGE_SPUR_GEAR_TEETH = 22;
    public static final int BEVEL_PINION_TEETH = 15;

    public static final int DRIVING_MOTOR_FREE_SPEED = 6784;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double DRIVING_MOTOR_FREE_SPEED_RPS =
            (double) DRIVING_MOTOR_FREE_SPEED / 60;
    public static final double WHEEL_DIAMETER_METERS = 0.0762;
    public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double DRIVING_MOTOR_REDUCTION =
            (double) (BEVEL_GEAR_TEETH * FIRST_STAGE_SPUR_GEAR_TEETH)
                    / (DRIVING_MOTOR_PINION_TEETH * BEVEL_PINION_TEETH);
    public static final double DRIVE_WHEEL_FREE_SPEED_RPS =
            (DRIVING_MOTOR_FREE_SPEED_RPS * WHEEL_CIRCUMFERENCE_METERS) / DRIVING_MOTOR_REDUCTION;
}
