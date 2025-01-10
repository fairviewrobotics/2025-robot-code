package frc.robot.utils;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.Optional;

public class NetworkTablesUtils {
    private final NetworkTable table;
    /**
     * Construct a NetworkTablesUtils
     * @param tableName The name of the table as it appears in network tables
     */
    private NetworkTablesUtils(String tableName) {
        table = NetworkTableInstance.getDefault().getTable(tableName);
    }

    /**
     * Get a table from NetworkTables
     * @param tableName The table name as it appears in Network Tables
     * @return An instance of {@link NetworkTablesUtils} for the specified table
     */
    public static NetworkTablesUtils getTable(String tableName) {
        return new NetworkTablesUtils(tableName);
    }

    /**
     * Get an entry from the table
     * @param key The key as it appears in network tables
     * @param defaultValue The default value
     * @return Either default value if the value doesn't exist in network tables or the corresponding value in NT
     * @param <T> The type of the entry (String, Double, or Boolean)
     */
    @SuppressWarnings("unchecked")
    public<T> T getEntry(String key, T defaultValue) {
        Types type = Types.getFromTypeName(defaultValue.getClass().getTypeName());
        switch (type) {
            case DOUBLE -> {
                return (T) (Double) this.table.getEntry(key).getDouble((Double) defaultValue);
            }
            case BOOLEAN -> {
                return (T) (Boolean) this.table.getEntry(key).getBoolean((Boolean) defaultValue);
            }
            case STRING -> {
                return (T) this.table.getEntry(key).getString((String) defaultValue);
            }
            case UNKNOWN -> {
                System.out.println("Error: Unknown type"); // TODO: Setup an error utils or something
            }
            default -> {
                System.out.println("Error: Use getArrayEntry for array type");
            }
        }
        return defaultValue;
    }

    /**
     * Get a array entry from the table
     * @param key The key as it appears in network tables
     * @param defaultValues The default values
     * @return Either the default values or the corresponding array in NT
     * @param <T> The type of the entry (String[], Double[], or Boolean[])
     */
    @SuppressWarnings("unchecked")
    public<T> T[] getArrayEntry(String key, T[] defaultValues) {
        Types type = Types.getFromTypeName(defaultValues.getClass().getTypeName());
        switch (type) {
            case DOUBLE_ARRAY -> {
                return (T[]) this.table.getEntry(key).getDoubleArray((Double[]) defaultValues);
            }
            case BOOLEAN_ARRAY -> {
                return (T[]) this.table.getEntry(key).getBooleanArray((Boolean []) defaultValues);
            }
            case STRING_ARRAY -> {
                return (T[]) this.table.getEntry(key).getStringArray((String[]) defaultValues);
            }
            case UNKNOWN -> {
                System.out.println("Error: Unknown type"); // TODO: Setup an error utils or something
            }
            default -> {
                System.out.println("Error: Use getEntry for non-array type");
            }
        }
        return defaultValues;
    }

    /**
     * Set an entry in network tables
     * @param key The key for the entry in network tables
     * @param value The value to set in NT
     * @param <T> The type of value (String, Double, or Boolean)
     */
    public<T> void setEntry(String key, T value) {
        Types type = Types.getFromTypeName(value.getClass().getTypeName());
        switch (type) {
            case DOUBLE -> this.table.getEntry(key).setDouble((Double) value);
            case BOOLEAN -> this.table.getEntry(key).setBoolean((Boolean) value);
            case STRING -> this.table.getEntry(key).setString((String) value);
            case UNKNOWN -> System.out.println("Error: Unknown type"); // TODO: Setup an error utils or something
            default -> System.out.println("Error: Use getArrayEntry for array type");

        }
    }

    /**
     * Set an array entry in network tables
     * @param key The key for the entry in network tables
     * @param value The value to set in NT
     * @param <T> The type of value (String[], Double[], or Boolean[])
     */
    public<T> void setArrayEntry(String key, T[] value) {
        Types type = Types.getFromTypeName(value.getClass().getTypeName());
        switch (type) {
            case DOUBLE -> this.table.getEntry(key).setDoubleArray((Double[]) value);
            case BOOLEAN -> this.table.getEntry(key).setBooleanArray((Boolean[]) value);
            case STRING -> this.table.getEntry(key).setDefaultStringArray((String[]) value);
            case UNKNOWN -> System.out.println("Error: Unknown type"); // TODO: Setup an error utils or something
            default -> System.out.println("Error: Use getArrayEntry for array type");

        }
    }

    /**
     * Enum for different types
     */
    private enum Types {
        DOUBLE,
        BOOLEAN,
        STRING,
        DOUBLE_ARRAY,
        BOOLEAN_ARRAY,
        STRING_ARRAY,
        UNKNOWN;

        /**
         * Gets the type from the getTypeName() function
         * Example:
         * <pre>
         * {@code
         * T foo = (T) 0.5;
         * System.out.println(Types.getFromTypeName(foo.getClass.getTypeName());
         * }
         * </pre]>
         * @param typeName The type name of the class
         * @return The {@link Types} type of the class
         */
        public static Types getFromTypeName(String typeName) {
            switch (typeName) {
                case "double" -> {
                    return DOUBLE;
                }
                case "boolean" -> {
                    return BOOLEAN;
                }
                case "java.lang.String" -> {
                    return STRING;
                }
                case "double[]" -> {
                    return DOUBLE_ARRAY;
                }
                case "boolean[]" -> {
                    return BOOLEAN_ARRAY;
                }
                case "java.lang.String[]" -> {
                    return STRING_ARRAY;
                }
                default -> {
                    return UNKNOWN;
                }
            }
        }
    }
}
