/* Black Knights Robotics (C) 2025 */
package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator3d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.utils.Camera;
import java.util.ArrayList;

public class OdometrySubsystem extends SubsystemBase {
    private final ArrayList<Camera> cameras;

    private SwerveDrivePoseEstimator3d poseEstimator =
            new SwerveDrivePoseEstimator3d(
                    DrivetrainConstants.DRIVE_KINEMATICS,
                    new Rotation3d(),
                    new SwerveModulePosition[4],
                    new Pose3d(),
                    new Matrix<>(
                            Nat.N4(),
                            Nat.N1(),
                            new double[] {
                                VisionConstants.WHEEL_TRUST,
                                VisionConstants.WHEEL_TRUST,
                                Math.toRadians(5)
                            }),
                    new Matrix<>(
                            Nat.N4(),
                            Nat.N1(),
                            new double[] {
                                VisionConstants.VISION_TRUST,
                                VisionConstants.VISION_TRUST,
                                Math.toRadians(5)
                            }));

    public OdometrySubsystem(ArrayList<Camera> cameras) {
        this.cameras = cameras;
    }

    /**
     * Get the current estimated pose of the robot
     *
     * @return A {@link Pose3d} of the robot
     */
    public Pose3d getRobotPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Add odometry data from wheels
     *
     * @param gyroRotation A {@link Rotation3d} from the gyro
     * @param swerveModulePositions 4 {@link SwerveModulePosition} objects
     */
    public void addWheelOdometry(
            Rotation3d gyroRotation, SwerveModulePosition[] swerveModulePositions) {
        if (swerveModulePositions.length != 4) {
            System.out.println("Error: Wrong length for module positions");
            return;
        }

        this.poseEstimator.update(gyroRotation, swerveModulePositions);
    }

    @Override
    public void periodic() {
        for (Camera c : this.cameras) {
            if (c.getPoseFieldSpace(this.getRobotPose()).isPresent()) {
                if (c.getDistanceFromTag() > 3) return; // TODO: Tune
                this.poseEstimator.addVisionMeasurement(
                        c.getPoseFieldSpace(this.getRobotPose()).get(), c.getTimestamp());
            }
        }
    }
}
