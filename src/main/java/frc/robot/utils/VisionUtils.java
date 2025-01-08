package frc.robot.utils;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.constants.VisionConstants;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import java.util.Objects;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;

public class VisionUtils {

    //Note Vision Limelight
    public static NetworkTable noteLimelight = NetworkTableInstance.getDefault().getTable("limelight-note");

    private static NetworkTable notePhoton = NetworkTableInstance.getDefault().getTable("photonvision").getSubTable("note");
    public static double height = 0.4;
    public static double forward = 0.4;

    private static boolean enabled = true;

    /**
     * Toggle if vision is enable or not
     * @param b Set if vision is enabled or not
     */
    public static void visionEnabled(boolean b) {
        enabled = b;
    }

    /**
     * Gets the requested entry from as well as optionally adding the alliance. Used internally
     * @param pose The pose you want as defined by https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api#apriltag-and-3d-data
     * @param useAlliance Whether to add on the alliance
     * @return The pose from the limelight
     */
    private static Optional<Pose3d> getPose(String pose, boolean useAlliance) {
        if (!enabled) return Optional.empty();

//        String suffix = (useAlliance && DriverStation.getAlliance().isPresent()) ?
//                ((DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) ? "_wpiblue" : "_wpired") : "";


        String suffix = useAlliance ? "_wpiblue" : "";
        double[] returnedPose = NetworkTableInstance.getDefault().getTable("limelight").getEntry(pose + suffix).getDoubleArray(new double[0]);
        if (returnedPose.length == 0 || (Objects.equals(getPose3d(returnedPose).toPose2d(), new Pose2d(new Translation2d(0, 0), new Rotation2d(0))))) return Optional.empty();

        return Optional.of(getPose3d(returnedPose));
    }

    /**
     * Converts a double array from limelight to a {@link Pose3d} with an origin of the Blue Side
     * @param returnedPose A double array of values, normally returned from limelight
     * @return A {@link Pose3d} of the robots current position
     */
    private static Pose3d getPose3d(double[] returnedPose) {
        Pose3d visionPos = new Pose3d(
            new Translation3d(returnedPose[0], returnedPose[1], returnedPose[2]),
            new Rotation3d(0.0, 0.0, Math.toRadians(returnedPose[5]))
        );

//        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
//            visionPos = new Pose3d(
//                new Translation3d(VisionConstants.fieldLenMeters - visionPos.getX(),
//                        VisionConstants.fieldHighMeters - visionPos.getY(), visionPos.getZ()),
//                new Rotation3d(0.0, 0.0, visionPos.getRotation().getZ())
//            );
//        }
        return visionPos;
    }

    /**
     * Gets the bots position relative to the field. Used by odometry
     * @return The bots position
     * */
    public static Optional<Pose3d> getBotPoseFieldSpace() {
        return getPose("botpose", true);
    }

    /**
     * Gets the bots position relative to the target. Used by AlignCommand
     * @return The bots position
     * */
    public static Optional<Pose3d> getBotPoseTargetSpace() {
        return getPose("botpose_targetspace", false);
    }

    /**
     * Gets the distance from the first visible AprilTag
     * @return The distance (in meters) from the AprilTag
     */
    public static double getDistanceFromTagLimelight() {

        AtomicReference<Pose3d> returnedPose = new AtomicReference<>(new Pose3d());
        getBotPoseTargetSpace().ifPresent(returnedPose::set);

        return Math.abs(returnedPose.get().getZ());

    }

    /**
     * Gets the latency from the pipeline
     * @return The latency
     */
    public static double getLatencyPipeline() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tl").getDouble(0.0);
    }

    /**
     * Gets the latency from capturing
     * @return The latency
     */
    public static double getLatencyCapture() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("cl").getDouble(0.0);
    }

    public static double getNoteTX(){
        return notePhoton.getEntry("targetYaw").getDouble(0.0);
    }

    public static double getNoteTY() {
        return noteLimelight.getEntry("ty").getDouble(0.0);
    }

    public static boolean noteHasTarget(){
        return notePhoton.getEntry("hasTarget").getBoolean(false);
    }

    public static double YDistanceToNote() {
        return height * Math.tan(getNoteTY());
    }

    public static double XDistanceToNote() {
        return YDistanceToNote() * Math.tan(getNoteTX());
    }

    public static double distanceToNote() {
        return Math.sqrt(
                Math.pow(YDistanceToNote(), 2)
                + Math.pow(XDistanceToNote(), 2)
        );
    }


    public static double getThor() {
        return notePhoton.getEntry("targetPixelsX").getDouble(-1);
    }




    // PHOTON VISION

    public static PhotonCamera getPhotonAprilCamera() {
        return new PhotonCamera("apriltag");
    }

    public static PhotonCamera getPhotonNoteCamera() {
        return new PhotonCamera("note");
    }

    public static Transform3d getPhotonAprilRobotToCamera() {
        return VisionConstants.APRILTAG_PHOTON_CAM_TRANSFORM;
    }

    /**
     * Get estimated position from PhotonVision
     * @return Estimated robot pose and timestamp within an {@link EstimatedRobotPose}
     */
    public static Optional<EstimatedRobotPose> getEstimatedGlobalPose(PhotonPoseEstimator photonPoseEstimator, Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }

}
