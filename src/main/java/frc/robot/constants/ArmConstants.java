package frc.robot.constants;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class ArmConstants {
    public static final int PIVOT_MOTOR_ID = 1;
    public static final double PIVOT_P = 0.0;
    public static final double PIVOT_I = 0.0;
    public static final double PIVOT_D = 0.0;
    public static final double PIVOT_MAX_VELOCITY = 0.0;
    public static final double PIVOT_MAX_ACCELERATION = 0.0;
    public static final TrapezoidProfile.Constraints PIVOT_CONSTRAINTS = new TrapezoidProfile.Constraints(PIVOT_MAX_VELOCITY, PIVOT_MAX_ACCELERATION);

    public static final double PIVOT_KS = 0.0;
    public static final double PIVOT_KG = 0.0;
    public static final double PIVOT_KV = 0.0;
    public static final double PIVOT_KA = 0.0;

    public static final double PIVOT_TOLERANCE = 0.0;

    public static final double PIVOT_FEED_ANGLE = 0.0;
    public static final double PIVOT_PLACE_ANGLE = 0.0;

    public static final double PIVOT_MAX_ANGLE = 0.0;
    public static final double PIVOT_MIN_ANGLE = 0.0;

    public static final double PIVOT_START_ANGLE = 0.0;

    public static final int LEFT_MOTOR_ID = 0;
    public static final int RIGHT_MOTOR_ID = 0;

    public static final double HAND_P = 0;
    public static final double HAND_I = 0;
    public static final double HAND_D = 0;
    public static final double HAND_MAX_VELOCITY = 0.0;
    public static final double HAND_MAX_ACCELERATION = 0.0;
    public static final TrapezoidProfile.Constraints HAND_CONSTRAINTS = new TrapezoidProfile.Constraints(HAND_MAX_VELOCITY, HAND_MAX_ACCELERATION);

    public static final int HAND_LINEBREAK_ID = 0;
}
