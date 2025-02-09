/* Black Knights Robotics (C) 2025 */
package frc.robot.utils;

import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.*;
import java.nio.file.Path;
import java.util.EnumSet;
import java.util.Iterator;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

public class ConfigManager {
    private static ConfigManager INSTANCE;

    private final File configFile =
            Path.of(Filesystem.getDeployDirectory().toPath().toString(), "tuning.json").toFile();

    private JSONObject json;

    private NetworkTablesUtils NTTune = NetworkTablesUtils.getTable("Tune");

    private static final Logger LOGGER = LogManager.getLogger();

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
                LOGGER.info("Created config file");
                this.json = this.getDefault();
                this.saveConfig();
            }
        } catch (IOException e) {

            LOGGER.warn("Failed to create config file", e);
        }

        this.json = parseConfig();
        this.initNtValues();
        this.initListener();
    }

    /** Initialize the network table values */
    @SuppressWarnings("unchecked")
    public void initNtValues() {
        Iterator<String> keys = this.json.keySet().iterator();
        LOGGER.info(keys);
        while (keys.hasNext()) {
            String key = keys.next();
            LOGGER.info("Initializing [{}] network table entry", key);
            this.NTTune.getNetworkTable().getEntry(key).setValue(this.json.get(key));
        }
    }

    /** Add a listener to network tables for a change in one of the tuning values */
    @SuppressWarnings("unchecked")
    private void initListener() {
        NTTune.addListener(
                (EnumSet.of(NetworkTableEvent.Kind.kValueAll)),
                (table, key1, event) -> {
                    Object value = table.getValue(key1).getValue();
                    this.json.put(key1, value);
                    LOGGER.debug("Updated [{}] to `{}`", key1, value.toString());
                    // table.getEntry(key1).getDouble(-1));

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

        return defaultSettings;
    }

    public void saveDefault() {
        this.json = getDefault();
        this.saveConfig();
    }

    /**
     * Get an integer value from the config
     *
     * @param key The key in the json
     * @param defaultValue A default value in case we fail to get the key
     * @return The value from the key
     */
    @SuppressWarnings("unchecked")
    public long get(String key, long defaultValue) {
        double v = Long.valueOf(defaultValue).doubleValue();
        if (!NTTune.keyExists(key)) {
            LOGGER.info("{} does not exist, creating a setting to {}", key, defaultValue);
            NTTune.setEntry(key, v);
            this.json.put(key, v);
        }
        return (long) getDouble(key, v);
    }

    /**
     * Get a double value from the config
     *
     * @param key The key in the json
     * @param defaultValue A default value in case we fail to get the key
     * @return The value from the key
     */
    @SuppressWarnings("unchecked")
    public double get(String key, double defaultValue) {
        if (!NTTune.keyExists(key)) {
            LOGGER.info("{} does not exist, creating a setting to {}", key, defaultValue);
            NTTune.setEntry(key, defaultValue);
            this.json.put(key, defaultValue);
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
    @SuppressWarnings("unchecked")
    public String get(String key, String defaultValue) {
        if (!NTTune.keyExists(key)) {
            LOGGER.info("{} does not exist, creating a setting to {}", key, defaultValue);
            NTTune.setEntry(key, defaultValue);
            this.json.put(key, defaultValue);
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
    @SuppressWarnings("unchecked")
    public boolean get(String key, boolean defaultValue) {
        if (!NTTune.keyExists(key)) {
            LOGGER.info("{} does not exist, creating a setting to {}", key, defaultValue);
            NTTune.setEntry(key, defaultValue);
            this.json.put(key, defaultValue);
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
        } catch (Exception e) {
            LOGGER.warn("Failed to get {} as a double", key, e);
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
        } catch (Exception e) {
            LOGGER.warn("Failed to get {} as a boolean", key, e);
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
            LOGGER.warn("Failed to get {} as a string", key, e);
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
            LOGGER.warn("Failed to save file: {}", configFile, e);
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
            LOGGER.error("An error occurred while parsing the config file", e);
        }
        return jObj;
    }
}
