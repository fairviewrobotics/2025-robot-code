/* Black Knights Robotics (C) 2025 */
package frc.robot.utils;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.EnumSet;

/** Wrapper class for network tables to maker interaction easier */
public class NetworkTablesUtils {
    private final NetworkTable table;

    /**
     * Construct a NetworkTablesUtils
     *
     * @param tableName The name of the table as it appears in network tables
     */
    private NetworkTablesUtils(String tableName) {
        table = NetworkTableInstance.getDefault().getTable(tableName);
    }

    private NetworkTablesUtils(NetworkTable table) {
        this.table = table;
    }

    /**
     * Get a table from NetworkTables
     *
     * @param tableName The table name as it appears in Network Tables
     * @return An instance of {@link NetworkTablesUtils} for the specified table
     */
    public static NetworkTablesUtils getTable(String tableName) {
        return new NetworkTablesUtils(tableName);
    }

    /**
     * Get a table from network tables
     *
     * @param table A {@link NetworkTable}
     * @return An instance of {@link NetworkTablesUtils}
     */
    public static NetworkTablesUtils getTable(NetworkTable table) {
        return new NetworkTablesUtils(table);
    }

    /**
     * Add a listener to the network table
     *
     * @param events A {@link EnumSet} of {@link NetworkTableEvent.Kind} for what events to listen
     *     to
     * @param listener The {@link NetworkTable.TableEventListener}
     */
    public void addListener(
            EnumSet<NetworkTableEvent.Kind> events, NetworkTable.TableEventListener listener) {
        this.table.addListener(events, listener);
    }

    /**
     * Get a double entry from network tables
     *
     * @param key The key as it appears in network table
     * @param defaultValue Default value in case the key is invalid
     * @return Either the default value or the corresponding value in network tables
     */
    public double getEntry(String key, double defaultValue) {
        return this.table.getEntry(key).getDouble(defaultValue);
    }

    /**
     * Get a long entry from network tables
     *
     * @param key The key as it appears in network table
     * @param defaultValue Default value in case the key is invalid
     * @return Either the default value or the corresponding value in network tables
     */
    public long getEntry(String key, long defaultValue) {
        return this.table.getEntry(key).getInteger(defaultValue);
    }

    /**
     * Get a boolean entry from network tables
     *
     * @param key The key as it appears in network table
     * @param defaultValue Default value in case the key is invalid
     * @return Either the default value or the corresponding value in network tables
     */
    public boolean getEntry(String key, boolean defaultValue) {
        return this.table.getEntry(key).getBoolean(defaultValue);
    }

    /**
     * Get a String entry from network tables
     *
     * @param key The key as it appears in network table
     * @param defaultValue Default value in case the key is invalid
     * @return Either the default value or the corresponding value in network tables
     */
    public String getEntry(String key, String defaultValue) {
        return this.table.getEntry(key).getString(defaultValue);
    }

    /**
     * Get a double array entry from network tables
     *
     * @param key The key as it appears in network table
     * @param defaultValues Default values in case the key is invalid
     * @return Either the default value or the corresponding value in network tables
     */
    public double[] getArrayEntry(String key, double[] defaultValues) {
        return this.table.getEntry(key).getDoubleArray(defaultValues);
    }

    /**
     * Get a long array entry from network tables
     *
     * @param key The key as it appears in network table
     * @param defaultValues Default values in case the key is invalid
     * @return Either the default value or the corresponding value in network tables
     */
    public long[] getArrayEntry(String key, long[] defaultValues) {
        return this.table.getEntry(key).getIntegerArray(defaultValues);
    }

    /**
     * Get a boolean array entry from network tables
     *
     * @param key The key as it appears in network table
     * @param defaultValues Default values in case the key is invalid
     * @return Either the default value or the corresponding value in network tables
     */
    public boolean[] getArrayEntry(String key, boolean[] defaultValues) {
        return this.table.getEntry(key).getBooleanArray(defaultValues);
    }

    /**
     * Get a String array entry from network tables
     *
     * @param key The key as it appears in network table
     * @param defaultValues Default values in case the key is invalid
     * @return Either the default value or the corresponding value in network tables
     */
    public String[] getArrayEntry(String key, String[] defaultValues) {
        return this.table.getEntry(key).getStringArray(defaultValues);
    }

    /**
     * Set a double entry in network tables
     *
     * @param key The key for the value
     * @param value The value that will be set
     */
    public void setEntry(String key, double value) {
        this.table.getEntry(key).setDouble(value);
    }

    /**
     * Set a long entry in network tables
     *
     * @param key The key for the value
     * @param value The value that will be set
     */
    public void setEntry(String key, long value) {
        this.table.getEntry(key).setInteger(value);
    }

    /**
     * Set a boolean entry in network tables
     *
     * @param key The key for the value
     * @param value The value that will be set
     */
    public void setEntry(String key, boolean value) {
        this.table.getEntry(key).setBoolean(value);
    }

    /**
     * Set a String entry in network tables
     *
     * @param key The key for the value
     * @param value The value that will be set
     */
    public void setEntry(String key, String value) {
        this.table.getEntry(key).setString(value);
    }

    /**
     * Set a double array entry in network tables
     *
     * @param key The key for the value
     * @param value The value that will be set
     */
    public void setArrayEntry(String key, double[] value) {
        this.table.getEntry(key).setDoubleArray(value);
    }

    /**
     * Set a long array entry in network tables
     *
     * @param key The key for the value
     * @param value The value that will be set
     */
    public void setArrayEntry(String key, long[] value) {
        this.table.getEntry(key).setIntegerArray(value);
    }

    /**
     * Set a boolean array entry in network tables
     *
     * @param key The key for the value
     * @param value The value that will be set
     */
    public void setArrayEntry(String key, boolean[] value) {
        this.table.getEntry(key).setBooleanArray(value);
    }

    /**
     * Set a String array entry in network tables
     *
     * @param key The key for the value
     * @param value The value that will be set
     */
    public void setArrayEntry(String key, String[] value) {
        this.table.getEntry(key).setStringArray(value);
    }

    /**
     * Check if an entry exists in network tables
     *
     * @param key The key as it appears in network tables
     * @return If the key exists
     */
    public boolean keyExists(String key) {
        return this.table.getEntry(key).exists();
    }

    /**
     * Return the raw network table
     *
     * @return The raw network table
     */
    public NetworkTable getNetworkTable() {
        return this.table;
    }
}
