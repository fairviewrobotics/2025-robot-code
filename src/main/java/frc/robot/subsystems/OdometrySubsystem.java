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
import frc.robot.utils.ConfigManager;
import java.util.ArrayList;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

public class OdometrySubsystem extends SubsystemBase {
    private ArrayList<Camera> cameras = new ArrayList<>();

    private static final Logger LOGGER = LogManager.getLogger();

    private final SwerveDrivePoseEstimator3d poseEstimator =
            new SwerveDrivePoseEstimator3d(
                    DrivetrainConstants.DRIVE_KINEMATICS,
                    new Rotation3d(),
                    new SwerveModulePosition[] {
                        new SwerveModulePosition(),
                        new SwerveModulePosition(),
                        new SwerveModulePosition(),
                        new SwerveModulePosition()
                    },
                    new Pose3d(),
                    new Matrix<>(
                            Nat.N4(),
                            Nat.N1(),
                            new double[] {
                                ConfigManager.getInstance()
                                        .get("odom_wheel_trust", VisionConstants.WHEEL_TRUST),
                                ConfigManager.getInstance()
                                        .get("odom_wheel_trust", VisionConstants.WHEEL_TRUST),
                                ConfigManager.getInstance()
                                        .get("odom_wheel_trust_theta", Math.toRadians(5)),
                              1
                            }),
                    new Matrix<>(
                            Nat.N4(),
                            Nat.N1(),
                            new double[] {
                                ConfigManager.getInstance()
                                        .get("odom_vision_trust", VisionConstants.VISION_TRUST),
                                ConfigManager.getInstance()
                                        .get("odom_vision_trust", VisionConstants.VISION_TRUST),
                                ConfigManager.getInstance()
                                        .get("odom_vision_trust_theta", Math.toRadians(5)),
                              1
                            }));

    public OdometrySubsystem() {}

    public void addCamera(Camera camera) {
        this.cameras.add(camera);
    }

    /**
     * Get the current estimated pose of the robot
     *
     * @return A {@link Pose3d} of the robot
     */
    public Pose3d getRobotPose() {
        return this.poseEstimator.getEstimatedPosition();
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
            LOGGER.error("Wrong length for module positions");
            return;
        }

        this.poseEstimator.update(gyroRotation, swerveModulePositions);
    }

    @Override
    public void periodic() {
        for (Camera c : this.cameras) {
            if (c.getPoseFieldSpace(this.getRobotPose()).isPresent()) {
                if (c.getDistanceFromTag()
                        > ConfigManager.getInstance().get("vision_cutoff_distance", 3)) return;
                LOGGER.debug("Added vision measurement from `{}`", c.getName());
                this.poseEstimator.addVisionMeasurement(
                        c.getPoseFieldSpace(this.getRobotPose()).get(), c.getTimestamp());
            }
        }
    }
}
