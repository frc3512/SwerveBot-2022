package frc.lib.util;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public final class NetworkTableUtil {
  public static NetworkTableEntry makeDoubleEntry(String name, double defaultValue) {
    NetworkTableInstance instance = NetworkTableInstance.getDefault();
    NetworkTableEntry entry = instance.getEntry(name);
    entry.setDefaultDouble(defaultValue);

    return entry;
  }

  public static NetworkTableEntry makeDoubleEntry(String name) {
    return makeDoubleEntry(name, 0.0);
  }

  public static NetworkTableEntry makeBooleanEntry(String name, boolean defaultValue) {
    NetworkTableInstance instance = NetworkTableInstance.getDefault();
    NetworkTableEntry entry = instance.getEntry(name);
    entry.setDefaultBoolean(defaultValue);

    return entry;
  }

  public static NetworkTableEntry makeBooleanEntry(String name) {
    return makeBooleanEntry(name, false);
  }

  public static NetworkTableEntry makeStringEntry(String name, String defaultValue) {
    NetworkTableInstance instance = NetworkTableInstance.getDefault();
    NetworkTableEntry entry = instance.getEntry(name);
    entry.setDefaultString(defaultValue);

    return entry;
  }

  public static NetworkTableEntry makeStringEntry(String name) {
    return makeStringEntry(name, "");
  }

  public static NetworkTableEntry makeDoubleArrayEntry(String name, double... defaultValue) {
    NetworkTableInstance instance = NetworkTableInstance.getDefault();
    NetworkTableEntry entry = instance.getEntry(name);
    entry.setDefaultDoubleArray(defaultValue);

    return entry;
  }

  public static NetworkTableEntry makeDoubleArrayEntry(String name) {
    double[] array = {};
    return makeDoubleArrayEntry(name, array);
  }
}
