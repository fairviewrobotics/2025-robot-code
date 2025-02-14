/* Black Knights Robotics (C) 2025 */
package frc.robot.utils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import java.util.Optional;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

/** */
public class Camera {
    private Optional<NetworkTablesUtils> limelightTable;

    private Optional<PhotonCamera> photonCamera;
    private Optional<PhotonPoseEstimator> photonPoseEstimator;

    private Transform3d camOffset;
    private double photonTimestamp;
    private Pose3d targetPose;
    private String name;

    private final CameraType camType;

    private static final Logger LOGGER = LogManager.getLogger();

    /**
     * Util class for cameras
     *
     * @param name The name of the camera
     * @param camType The {@link CameraType} of the camera
     */
    public Camera(String name, CameraType camType, Transform3d camOffset) {
        this.camType = camType;
        this.camOffset = camOffset;
        this.name = name;

        switch (this.camType) {
            case PHOTONVISION -> {
                photonCamera = Optional.of(new PhotonCamera(name));
                photonPoseEstimator =
                        Optional.of(
                                new PhotonPoseEstimator(
                                        AprilTagFieldLayout.loadField(
                                                AprilTagFields.k2025Reefscape),
                                        PhotonPoseEstimator.PoseStrategy
                                                .MULTI_TAG_PNP_ON_COPROCESSOR,
                                        this.camOffset));
                limelightTable = Optional.empty();
            }
            case LIMELIGHT -> {
                photonCamera = Optional.empty();
                photonPoseEstimator = Optional.empty();
                limelightTable = Optional.of(NetworkTablesUtils.getTable(name));
            }
        }
    }

    /**
     * Get the robots position relative to the field (blue side)
     *
     * @param prevPosition The previous position of the robot
     * @return Either an empty optional or the a {@link Pose3d} of the robot
     */
    public Optional<Pose3d> getPoseFieldSpace(Pose3d prevPosition) {
        switch (this.camType) {
            case PHOTONVISION -> {
                assert this.photonPoseEstimator.isPresent();
                assert this.photonCamera.isPresent();

                PhotonPipelineResult res;
                try {
                    res = this.photonCamera.get().getAllUnreadResults().get(0);
                    Transform3d bestCamToTarget = res.getBestTarget().getBestCameraToTarget();
                    this.targetPose =
                            new Pose3d(
                                            bestCamToTarget.getTranslation(),
                                            bestCamToTarget.getRotation())
                                    .transformBy(this.camOffset);
                } catch (Exception e) {
                    return Optional.empty();
                }

                if (!res.hasTargets()) return Optional.empty();

                this.photonPoseEstimator.get().setReferencePose(prevPosition);
                return photonPoseEstimator
                        .get()
                        .update(res)
                        .map(
                                (e) -> {
                                    this.photonTimestamp = e.timestampSeconds;
                                    return e.estimatedPose;
                                });
            }
            case LIMELIGHT -> {
                return getPose3dLimelight();
            }
        }
        return Optional.empty();
    }

    private Optional<Pose3d> getPose3dLimelight() {
        assert this.limelightTable.isPresent();

        double[] rawPose =
                this.limelightTable.get().getArrayEntry("botpose_wpiblue", new double[0]);
        if (rawPose.length != 6) return Optional.empty();

        double[] targetPoseRaw =
                this.limelightTable.get().getArrayEntry("targetpose_cameraspace", new double[0]);
        if (targetPoseRaw.length == 6) {
            Pose3d cameraPose =
                    new Pose3d(
                            targetPoseRaw[0],
                            targetPoseRaw[1],
                            targetPoseRaw[2],
                            new Rotation3d(0.0, 0.0, Math.toRadians(targetPoseRaw[5])));
            this.targetPose = cameraPose.transformBy(this.camOffset);
        }

        return Optional.of(
                new Pose3d(
                                new Translation3d(rawPose[0], rawPose[1], rawPose[2]),
                                new Rotation3d(0.0, 0.0, Math.toRadians(rawPose[5])))
                        .transformBy(camOffset));
    }

    /**
     * Get the timestamp in seconds since the last frame
     *
     * @return The timestamp in seconds since the last frame
     */
    public double getTimestamp() {
        switch (this.camType) {
            case LIMELIGHT -> {
                assert this.limelightTable.isPresent();
                return Timer.getFPGATimestamp()
                        - (this.limelightTable.get().getEntry("tl", Double.POSITIVE_INFINITY)
                                / 1000)
                        - (this.limelightTable.get().getEntry("cl", Double.POSITIVE_INFINITY)
                                / 1000);
            }
            case PHOTONVISION -> {
                assert this.photonCamera.isPresent();

                return photonTimestamp;
            }
        }
        return 0;
    }

    /**
     * Get the name of the camera
     *
     * @return The name of the camera
     */
    public String getName() {
        return this.name;
    }

    /** */
    public Pose3d getTargetPose() {
        return this.targetPose;
    }

    public enum CameraType {
        LIMELIGHT,
        PHOTONVISION
    }
}
