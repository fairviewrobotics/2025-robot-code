/* Black Knights Robotics (C) 2025 */
package frc.robot.framework;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator3d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.utils.Camera;
import frc.robot.utils.ConfigManager;
import frc.robot.utils.NetworkTablesUtils;
import java.util.ArrayList;
import java.util.Optional;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

public class Odometry {
    /** List of cameras used for vision-based measurements to refine odometry. */
    private ArrayList<Camera> cameras = new ArrayList<>();

    /** Singleton instance of the OdometrySubsystem. */
    private static Odometry INSTANCE = null;

    /** Logger for recording debug and error messages related to odometry subsystem operations. */
    private static final Logger LOGGER = LogManager.getLogger();

    private final NetworkTablesUtils NTTelemetry = NetworkTablesUtils.getTable("Telemetry");

    private Optional<Pose3d> targetPose = Optional.of(new Pose3d());

    /** Pose estimator for the robot, combining wheel-based odometry and vision measurements. */
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

    private Odometry() {}

    /**
     * Get the instance of Odometry, creating a new one if it doesn't exist
     *
     * @return The instance
     */
    public static Odometry getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Odometry();
        }
        return INSTANCE;
    }

    /**
     * Add a camera to the Odometry for vision-based localization.
     *
     * @param camera The {@link Camera} to add to the system.
     */
    public void addCamera(Camera camera) {
        this.cameras.add(camera);
    }

    /**
     * Get a camera by name
     *
     * @param name The name of the camera
     * @return Either an empty optional if the camera does not exist, or the camera
     */
    public Optional<Camera> getCamera(String name) {
        for (Camera c : this.cameras) {
            if (c.getName().equalsIgnoreCase(name)) return Optional.of(c);
        }

        return Optional.empty();
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
     * @return
     */
    public Optional<Pose3d> getTargetPose() {
        return this.targetPose;
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

    public void periodic() {
        NTTelemetry.setArrayEntry(
                "Pose",
                new double[] {
                    this.getRobotPose().getX(),
                    this.getRobotPose().getY(),
                    this.getRobotPose().getRotation().getZ()
                });
        for (Camera c : this.cameras) {
            Optional<Pose3d> pose = c.getPoseFieldSpace(this.getRobotPose());
            if (pose.isPresent()) {
                if (1 // FIXME: Fix once
                        > ConfigManager.getInstance().get("vision_cutoff_distance", 3)) return;
                LOGGER.debug("Added vision measurement from `{}`", c.getName());
                this.targetPose =
                        Optional.of(
                                new Pose3d(
                                        c.getTargetPose().getX() + this.getRobotPose().getX(),
                                        c.getTargetPose().getY() + this.getRobotPose().getY(),
                                        c.getTargetPose().getZ(),
                                        c.getTargetPose().getRotation()));
                this.poseEstimator.addVisionMeasurement(pose.get(), c.getTimestamp());
            } else {
                this.targetPose = Optional.empty();
            }
        }
    }
}
