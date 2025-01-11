/* Black Knights Robotics (C) 2025 */
package frc.robot.utils;

import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.*;
import java.nio.file.Path;
import java.util.EnumSet;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

public class ConfigManager {
    private static ConfigManager INSTANCE;

    private final File configFile =
            Path.of(Filesystem.getDeployDirectory().toPath().toString(), "tuning.json").toFile();

    private JSONObject json;

    private NetworkTablesUtils NTTune = NetworkTablesUtils.getTable("Tune");

    /**
     * Get the instance of the config manager
     *
     * @return Instance of config manager
     */
    public static synchronized ConfigManager getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new ConfigManager();
        }

        return INSTANCE;
    }

    /** Util class to allow for good network table tuning */
    // TODO: Add support for vales besides doubles
    public ConfigManager() {
        try {
            if (configFile.createNewFile() || configFile.length() == 0) {
                System.out.println("[INFO] Created tuning file");
                this.json = this.getDefault();
                this.saveConfig();
            }
        } catch (IOException e) {
            System.out.println("[WARN] Failed to create config file: " + e);
        }

        this.json = parseConfig();
        this.initListener();
    }

    /** Add a listener to network tables for a change in one of the tuning values */
    @SuppressWarnings("unchecked")
    private void initListener() {
        NTTune.addListener(
                (EnumSet.of(NetworkTableEvent.Kind.kValueAll)),
                (table, key1, event) -> {
                    this.json.put(key1, table.getValue(key1).getValue());
                    System.out.println(
                            "[DEBUG] Updated ["
                                    + key1
                                    + "] to "
                                    + table.getEntry(key1).getDouble(-1));
                    this.saveConfig();
                });
    }

    /**
     * Get the default settings (used to create the json file if it does not exist)
     *
     * @return A default json object
     */
    @SuppressWarnings("unchecked")
    public JSONObject getDefault() {
        JSONObject defaultSettings = new JSONObject();

        // INTAKE
        defaultSettings.put("intake_notein_speed", 0.0);
        defaultSettings.put("intake_shoot_speed", 0.0);

        // SHOOTER
        defaultSettings.put("shooter_speaker", 0.0);
        defaultSettings.put("shooter_high_pass", 0.0);
        defaultSettings.put("shooter_low_pass", 0.0);
        defaultSettings.put("shooter_amp", 0.0);

        // ARM
        defaultSettings.put("placeholder", 0.0);

        return defaultSettings;
    }

    public void saveDefault() {
        this.json = getDefault();
        this.saveConfig();
    }

    /**
     * Get a integer value from the config
     *
     * @param key The key in the json
     * @param defaultValue A default value in case we fail to get the key
     * @return The value from the key
     */
    public long get(String key, long defaultValue) {
        if (!NTTune.keyExists(key)) {
            System.out.println(
                    "[INFO] "
                            + key
                            + " does not exist in network tables, creating a setting to "
                            + defaultValue);

            NTTune.setEntry(key, defaultValue);
        }
        return getInteger(key, defaultValue);
    }

    /**
     * Get a double value from the config
     *
     * @param key The key in the json
     * @param defaultValue A default value in case we fail to get the key
     * @return The value from the key
     */
    public double get(String key, double defaultValue) {
        if (!NTTune.keyExists(key)) {
            System.out.println(
                    "[INFO] "
                            + key
                            + " does not exist in network tables, creating a setting to "
                            + defaultValue);

            NTTune.setEntry(key, defaultValue);
        }
        return getDouble(key, defaultValue);
    }

    /**
     * Get a String value from the config
     *
     * @param key The key in the json
     * @param defaultValue A default value in case we fail to get the key
     * @return The value from the key
     */
    public String get(String key, String defaultValue) {
        if (!NTTune.keyExists(key)) {
            System.out.println(
                    "[INFO] "
                            + key
                            + " does not exist in network tables, creating a setting to "
                            + defaultValue);

            NTTune.setEntry(key, defaultValue);
        }
        return getString(key, defaultValue);
    }

    /**
     * Get a boolean value from the config
     *
     * @param key The key in the json
     * @param defaultValue A default value in case we fail to get the key
     * @return The value from the key
     */
    public boolean get(String key, boolean defaultValue) {
        if (!NTTune.keyExists(key)) {
            System.out.println(
                    "[INFO] "
                            + key
                            + " does not exist in network tables, creating a setting to "
                            + defaultValue);

            NTTune.setEntry(key, defaultValue);
        }
        return getBoolean(key, defaultValue);
    }

    /**
     * Get a double from the config
     *
     * @param key The key in the json
     * @param defaultValue A default value
     * @return A double (as an {@link Object})
     */
    private double getDouble(String key, double defaultValue) {
        double res = defaultValue;
        try {
            res = (double) this.json.get(key);
        } catch (ClassCastException e) {
            System.out.println("[WARN] Failed to get " + key + " as a double");
        }

        return res;
    }

    /**
     * Get an integer from the config
     *
     * @param key The key in the json
     * @param defaultValue A default value
     * @return A long (as an {@link Object})
     */
    private long getInteger(String key, long defaultValue) {
        long res = defaultValue;
        try {
            res = (long) this.json.get(key);
        } catch (ClassCastException e) {
            System.out.println("[WARN] Failed to get " + key + " as a long");
        }

        return res;
    }

    /**
     * Get a Boolean from the config
     *
     * @param key The key in the json
     * @param defaultValue A default value
     * @return A boolean (as an {@link Object})
     */
    private boolean getBoolean(String key, boolean defaultValue) {
        boolean res = defaultValue;
        try {
            res = (boolean) this.json.get(key);
        } catch (ClassCastException e) {
            System.out.println("[WARN] Failed to get " + key + " as a boolean");
        }

        return res;
    }

    /**
     * Get a string from the config
     *
     * @param key The key in the json
     * @param defaultValue A default value
     * @return A string (as an {@link Object})
     */
    private String getString(String key, String defaultValue) {
        String res = defaultValue;
        try {
            res = (String) this.json.get(key);
        } catch (ClassCastException e) {
            System.out.println("[WARN] Failed to get " + key + " as a string");
        }

        return res;
    }

    /**
     * Set a value
     *
     * @param key The key for the json file
     * @param value The value to set
     */
    @SuppressWarnings("unchecked")
    public <T> void set(String key, T value) {
        this.json.put(key, value);
        this.saveConfig();
    }

    /** Save the config to the config file location */
    public void saveConfig() {
        try (PrintWriter printWriter = new PrintWriter(this.configFile)) {
            printWriter.println(this.json.toJSONString());
        } catch (FileNotFoundException e) {
            System.out.println("[WARN] Failed to save file: " + configFile + ": " + e);
        }
    }

    /**
     * Parse the config file
     *
     * @return The parsed config as a {@link JSONObject}
     */
    private JSONObject parseConfig() {
        JSONObject jObj = new JSONObject();
        JSONParser parser = new JSONParser();
        try {
            Object obj = parser.parse(new FileReader(this.configFile));
            jObj = (JSONObject) obj;
        } catch (IOException | ParseException e) {
            System.out.println("[ERROR] An error occurred: " + e);
        }
        return jObj;
    }
}
