/* Black Knights Robotics (C) 2025 */
package frc.robot.constants;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class ArmConstants {
    public static final int PIVOT_MOTOR_ID = 22;
    public static final double PIVOT_P = 0.25;
    public static final double PIVOT_I = 0.0;
    public static final double PIVOT_D = 0.0;
    public static final double PIVOT_MAX_VELOCITY = 4 * Math.PI;
    public static final double PIVOT_MAX_ACCELERATION = 4 * Math.PI;
    public static final TrapezoidProfile.Constraints PIVOT_CONSTRAINTS =
            new TrapezoidProfile.Constraints(PIVOT_MAX_VELOCITY, PIVOT_MAX_ACCELERATION);

    public static final double PIVOTENCODEROFFSET = 1.477;

    public static final double PIVOT_KS = 0.0;
    public static final double PIVOT_KG = 0.02;
    public static final double PIVOT_KV = 0.0;
    public static final double PIVOT_KA = 0.0;

    public static final double PIVOT_TOLERANCE = 0.01;

    public static final double PIVOT_FEED_ANGLE = 0.0;
    public static final double PIVOT_PLACE_ANGLE = 0.0;

    public static final double PIVOT_MAX_ANGLE = Math.PI / 2;
    public static final double PIVOT_MIN_ANGLE = -Math.PI / 2;

    public static final double PIVOT_START_ANGLE = 0.0;

    public static final int LEFT_MOTOR_ID = 18;
    public static final int RIGHT_MOTOR_ID = 19;

    public static final int HAND_LINEBREAK_ID = 3;
}
