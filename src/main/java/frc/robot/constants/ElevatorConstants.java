/* Black Knights Robotics (C) 2025 */
package frc.robot.constants;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class ElevatorConstants {

    public static final int LEFT_MOTOR_ID = 21;
    public static final int RIGHT_MOTOR_ID = 20;

    public static final double ELEVATOR_P = 0.0;
    public static final double ELEVATOR_I = 0;
    public static final double ELEVATOR_D = 0;
    public static final double ELEVATOR_TOLERANCE = 0.05;

    public static final double ELEVATOR_KS = 0.1;
    public static final double ELEVATOR_KV = 0;
    public static final double ELEVATOR_KG = 0.30;
    public static final double ELEVATOR_KA = 0;

    public static final TrapezoidProfile.Constraints CONSTRAINTS =
            new TrapezoidProfile.Constraints(5.0, 5.0);

    public static final int TOP_LINEBREAK_ID = 1;
    public static final int BOTTOM_LINEBREAK_ID = 0;

    public static final double ELEVATOR_ZEROING_VOLTAGE = 0.0;
    // 12.9 rotations to top, 0.592m to top
    public static final double ROTATIONS_TO_METERS = 0.590 / 38.32; // 0.592 / 12.9

    public static final double CURRENT_MAX = 20;

    public static final double ELEVATOR_MIN = 0.0;
    public static final double ELEVATOR_MAX = 0.58;
}
