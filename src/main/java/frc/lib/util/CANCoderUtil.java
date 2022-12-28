package frc.lib.util;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;

/** Sets status frames for the CTRE CANCoder. */
public class CANCoderUtil {
  public enum CCUsage {
    kAll,
    kSensorDataOnly,
    kFaultsOnly,
    kMinimal
  }

  /**
   * This function allows reducing a CANCoder's CAN bus utilization by reducing the periodic status
   * frame period of nonessential frames from 10ms to 255ms.
   *
   * <p>See https://docs.ctre-phoenix.com/en/stable/ch18_CommonAPI.html#cancoder for a description
   * of the status frames.
   *
   * @param cancoder The CANCoder to adjust the status frames on.
   * @param usage The status frame feedback to enable. kAll is the default when a CANCoder
   *     isconstructed.
   */
  public static void setCANCoderBusUsage(CANCoder cancoder, CCUsage usage) {
    if (usage == CCUsage.kAll) {
      cancoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10);
      cancoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 10);
    } else if (usage == CCUsage.kSensorDataOnly) {
      cancoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10);
      cancoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 100);
    } else if (usage == CCUsage.kFaultsOnly) {
      cancoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 100);
      cancoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 10);
    } else if (usage == CCUsage.kMinimal) {
      cancoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 100);
      cancoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 100);
    }
  }
}
