/* Black Knights Robotics (C) 2025 */
package frc.robot.framework;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.constants.ScoringConstants;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class CoralQueueTest {
    private CoralQueue coralQueue;

    @BeforeEach
    void setUp() {
        coralQueue = new CoralQueue();
    }

    @Test
    void testAddToQueue() {
        coralQueue.addPosition("0L1");
        coralQueue.addPosition("2L4");
        coralQueue.addPosition("10L4");

        CoralQueue.CoralPosition expected =
                new CoralQueue.CoralPosition(
                        "0L1",
                        new Pose2d(5.0, 7.0, Rotation2d.fromDegrees(12.0)),
                        ScoringConstants.ScoringHeights.L1);
        assertEquals(expected, coralQueue.getNext());

        expected =
                new CoralQueue.CoralPosition(
                        "2L4",
                        new Pose2d(5.0, 0.0, Rotation2d.fromDegrees(0.1)),
                        ScoringConstants.ScoringHeights.L4);
        assertEquals(expected, coralQueue.getNext());

        expected =
                new CoralQueue.CoralPosition(
                        "10L4",
                        new Pose2d(3.0, 2.0, Rotation2d.fromDegrees(0.0)),
                        ScoringConstants.ScoringHeights.L4);

        assertEquals(expected, coralQueue.getNext());
    }

    @Test
    void testListToQueue() {
        coralQueue.listToQueue("10L4,0L1,2L3");
        CoralQueue.CoralPosition expected =
                new CoralQueue.CoralPosition(
                        "10L4",
                        new Pose2d(3.0, 2.0, Rotation2d.fromDegrees(0.0)),
                        ScoringConstants.ScoringHeights.L4);

        assertEquals(expected, coralQueue.getNext());

        expected =
                new CoralQueue.CoralPosition(
                        "0L1",
                        new Pose2d(5.0, 7.0, Rotation2d.fromDegrees(12.0)),
                        ScoringConstants.ScoringHeights.L1);

        assertEquals(expected, coralQueue.getNext());

        expected =
                new CoralQueue.CoralPosition(
                        "2L3",
                        new Pose2d(5.0, 0.0, Rotation2d.fromDegrees(0.1)),
                        ScoringConstants.ScoringHeights.L3);

        assertEquals(expected, coralQueue.getNext());
    }

    @Test
    void testSkipNextValue() {
        coralQueue.listToQueue("10L4,0L1");
        coralQueue.stepForwards();
        CoralQueue.CoralPosition expected =
                new CoralQueue.CoralPosition(
                        "0L1",
                        new Pose2d(5.0, 7.0, Rotation2d.fromDegrees(12.0)),
                        ScoringConstants.ScoringHeights.L1);
        assertEquals(expected, coralQueue.getNext());
    }

    @Test
    void testBlueAlliancePositionMapping() {
        // Simulate Blue Alliance (Station ID 4-6)
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue3);
        DriverStationSim.notifyNewData();

        coralQueue.addPosition("0L1");
        coralQueue.addPosition("2L4");
        coralQueue.addPosition("10L4");

        CoralQueue.CoralPosition expected =
                new CoralQueue.CoralPosition(
                        "0L1",
                        new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.3)),
                        ScoringConstants.ScoringHeights.L1);
        assertEquals(expected, coralQueue.getNext());

        expected =
                new CoralQueue.CoralPosition(
                        "2L4",
                        new Pose2d(3.0, 12.0, Rotation2d.fromDegrees(9.0)),
                        ScoringConstants.ScoringHeights.L4);
        assertEquals(expected, coralQueue.getNext());

        expected =
                new CoralQueue.CoralPosition(
                        "10L4",
                        new Pose2d(13.0, 24.0, Rotation2d.fromDegrees(45.0)),
                        ScoringConstants.ScoringHeights.L4);

        assertEquals(expected, coralQueue.getNext());

        DriverStationSim.setAllianceStationId(AllianceStationID.Red1); // Set back to red
        DriverStationSim.notifyNewData();
    }
}
