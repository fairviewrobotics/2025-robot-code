/* Black Knights Robotics (C) 2025 */
package frc.robot.framework;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.AllianceStationID;
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
        coralQueue.addPosition("1L1");
        coralQueue.addPosition("3L4");
        coralQueue.addPosition("11L4");

        CoralQueue.CoralPosition expected =
                new CoralQueue.CoralPosition(
                        "1L1",
                        ScoringConstants.CORAL_POSITIONS[0],
                        ScoringConstants.ScoringHeights.L1);
        assertEquals(expected, coralQueue.getNext());

        expected =
                new CoralQueue.CoralPosition(
                        "3L4",
                        ScoringConstants.CORAL_POSITIONS[2],
                        ScoringConstants.ScoringHeights.L4);
        assertEquals(expected, coralQueue.getNext());

        expected =
                new CoralQueue.CoralPosition(
                        "11L4",
                        ScoringConstants.CORAL_POSITIONS[10],
                        ScoringConstants.ScoringHeights.L4);

        assertEquals(expected, coralQueue.getNext());
    }

    @Test
    void testListToQueue() {
        coralQueue.listToQueue("11L4,1L1,3L3");
        CoralQueue.CoralPosition expected =
                new CoralQueue.CoralPosition(
                        "11L4",
                        ScoringConstants.CORAL_POSITIONS[10],
                        ScoringConstants.ScoringHeights.L4);

        assertEquals(expected, coralQueue.getNext());

        expected =
                new CoralQueue.CoralPosition(
                        "1L1",
                        ScoringConstants.CORAL_POSITIONS[0],
                        ScoringConstants.ScoringHeights.L1);

        assertEquals(expected, coralQueue.getNext());

        expected =
                new CoralQueue.CoralPosition(
                        "3L3",
                        ScoringConstants.CORAL_POSITIONS[2],
                        ScoringConstants.ScoringHeights.L3);

        assertEquals(expected, coralQueue.getNext());
    }

    @Test
    void testSkipNextValue() {
        coralQueue.listToQueue("10L4,1L1");
        coralQueue.stepForwards();
        CoralQueue.CoralPosition expected =
                new CoralQueue.CoralPosition(
                        "1L1",
                        ScoringConstants.CORAL_POSITIONS[0],
                        ScoringConstants.ScoringHeights.L1);
        assertEquals(expected, coralQueue.getNext());
    }

    @Test
    void testBlueAlliancePositionMapping() {
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue3);
        DriverStationSim.notifyNewData();

        coralQueue.addPosition("6L1");
        coralQueue.addPosition("12L4");
        coralQueue.addPosition("9L4");

        CoralQueue.CoralPosition expected =
                new CoralQueue.CoralPosition(
                        "6L1",
                        ScoringConstants.CORAL_POSITIONS[5 + 12],
                        ScoringConstants.ScoringHeights.L1);
        assertEquals(expected, coralQueue.getNext());

        expected =
                new CoralQueue.CoralPosition(
                        "12L4",
                        ScoringConstants.CORAL_POSITIONS[11 + 12],
                        ScoringConstants.ScoringHeights.L4);
        assertEquals(expected, coralQueue.getNext());

        expected =
                new CoralQueue.CoralPosition(
                        "9L4",
                        ScoringConstants.CORAL_POSITIONS[8 + 12],
                        ScoringConstants.ScoringHeights.L4);

        assertEquals(expected, coralQueue.getNext());

        DriverStationSim.setAllianceStationId(AllianceStationID.Red1); // Set back to red
        DriverStationSim.notifyNewData();
    }
}
