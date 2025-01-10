package frc.robot.constants;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class ElevatorConstants {

    public static final int LEFT_MOTOR_ID = 0;
    public static final int RIGHT_MOTOR_ID = 0;

    public static final double ELEVATOR_P = 0;
    public static final double ELEVATOR_I = 0;
    public static final double ELEVATOR_D = 0;
    public static final double ELEVATOR_TOLERANCE = 0;

    public static final double ELEVATOR_KS = 0;
    public static final double ELEVATOR_KV = 0;
    public static final double ELEVATOR_KG = 0;
    public static final double ELEVATOR_KA = 0;

    public static final TrapezoidProfile.Constraints CONSTRAINTS = new TrapezoidProfile.Constraints(1, 0.5);

    public static final int TOP_LINEBREAK_ID = 0;
    public static final int BOTTOM_LINEBREAK_ID = 0;

    public static final double ELEVATOR_ZEROING_VOLTAGE = 0;

    public static final double ROTATIONS_TO_METERS = 1;
}
