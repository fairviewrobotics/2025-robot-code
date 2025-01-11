/* Black Knights Robotics (C) 2025 */
package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import org.junit.jupiter.api.Test;
import org.mockito.Mockito;

public class NetworkTableUtilsTest {
    private final NetworkTablesUtils utils;

    public NetworkTableUtilsTest() {
        NetworkTable table = Mockito.mock(NetworkTable.class);
        NetworkTableEntry entry = Mockito.mock(NetworkTableEntry.class);

        Mockito.when(table.getEntry("test")).thenReturn(entry);

        Mockito.when(entry.getDouble(0.0)).thenReturn(10.0);
        Mockito.when(entry.getInteger(0L)).thenReturn(5L);
        Mockito.when(entry.getBoolean(false)).thenReturn(true);
        Mockito.when(entry.getString("wrong")).thenReturn("right");

        Mockito.when(entry.getDoubleArray(new double[] {0.0})).thenReturn(new double[] {1.0});
        Mockito.when(entry.getIntegerArray(new long[] {0})).thenReturn(new long[] {1});
        Mockito.when(entry.getStringArray(new String[] {"wrong"}))
                .thenReturn(new String[] {"right"});
        Mockito.when(entry.getBooleanArray(new boolean[] {false})).thenReturn(new boolean[] {true});

        this.utils = NetworkTablesUtils.getTable(table);
    }

    @Test
    public void testGetEntryDouble() {
        double res = this.utils.getEntry("test", 0.0);
        assertEquals(10.0, res);
    }

    @Test
    public void testGetEntryInteger() {
        long res = this.utils.getEntry("test", 0L);
        assertEquals(5L, res);
    }

    @Test
    public void testGetEntryBoolean() {
        boolean res = this.utils.getEntry("test", false);
        assertTrue(res);
    }

    @Test
    public void testGetEntryString() {
        String res = this.utils.getEntry("test", "wrong");
        assertEquals("right", res);
    }

    @Test
    public void testGetArrayEntryDouble() {
        double[] res = this.utils.getArrayEntry("test", new double[] {0.0});
        assertEquals(1.0, res[0]);
    }

    @Test
    public void testGetArrayEntryInteger() {
        long[] res = this.utils.getArrayEntry("test", new long[] {0});
        assertEquals(1L, res[0]);
    }

    @Test
    public void testGetEntryArrayBoolean() {
        boolean[] res = this.utils.getArrayEntry("test", new boolean[] {false});
        assertTrue(res[0]);
    }

    @Test
    public void testGetArrayEntryString() {
        String[] res = this.utils.getArrayEntry("test", new String[] {"wrong"});
        assertTrue("right".equalsIgnoreCase(res[0]));
    }
}
