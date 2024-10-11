package frc.robot.utils;

import com.ctre.phoenix6.HootReplay;
import com.ctre.phoenix6.HootReplay.SignalData;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Utils;

public class SignalHandler {

  private SignalHandler() {
    SignalLogger.start();
  }

  /**
   * Retrieves a signal's value either from replay data (if available) or writes a real-time value.
   *
   * @param signalPath The path of the signal
   * @param realTimeValue The real-time value to write if not in replay mode
   * @return SignalData if replay is active, otherwise StatusCode of the write operation
   */
  public static <T> SignalData<T> getOrWriteSignal(String signalPath, T realTimeValue) {
    SignalData<T> signalData = new SignalData<>();
    signalData.name = signalPath;
    signalData.timestampSeconds = Utils.getCurrentTimeSeconds();
    return HootReplay.isPlaying()
        ? readValue(signalData, signalPath, realTimeValue)
        : writeValue(signalData, signalPath, realTimeValue);
  }

  @SuppressWarnings("unchecked")
  private static <T> SignalData<T> readValue(
      SignalData<T> signalData, String signalPath, T realTimeValue) {
    // Retrieve signal data from replay based on type
    if (realTimeValue instanceof byte[]) {
      return (SignalData<T>) HootReplay.getRaw(signalPath);
    } else if (realTimeValue instanceof Boolean) {
      return (SignalData<T>) HootReplay.getBoolean(signalPath);
    } else if (realTimeValue instanceof Long) {
      return (SignalData<T>) HootReplay.getInteger(signalPath);
    } else if (realTimeValue instanceof Float) {
      return (SignalData<T>) HootReplay.getFloat(signalPath);
    } else if (realTimeValue instanceof Double) {
      return (SignalData<T>) HootReplay.getDouble(signalPath);
    } else if (realTimeValue instanceof String) {
      return (SignalData<T>) HootReplay.getString(signalPath);
    } else if (realTimeValue instanceof boolean[]) {
      return (SignalData<T>) HootReplay.getBooleanArray(signalPath);
    } else if (realTimeValue instanceof long[]) {
      return (SignalData<T>) HootReplay.getIntegerArray(signalPath);
    } else if (realTimeValue instanceof float[]) {
      return (SignalData<T>) HootReplay.getFloatArray(signalPath);
    } else if (realTimeValue instanceof double[]) {
      return (SignalData<T>) HootReplay.getDoubleArray(signalPath);
    } else {
      signalData.status = StatusCode.NotFound;
      return signalData;
    }
  }

  private static <T> SignalData<T> writeValue(
      SignalData<T> signalData, String signalPath, T realTimeValue) {
    // Write the real-time value if not in replay mode
    StatusCode status;
    if (realTimeValue instanceof byte[]) {
      status = SignalLogger.writeRaw(signalPath, (byte[]) realTimeValue);
    } else if (realTimeValue instanceof Boolean) {
      status = SignalLogger.writeBoolean(signalPath, (Boolean) realTimeValue);
    } else if (realTimeValue instanceof Long) {
      status = SignalLogger.writeInteger(signalPath, (Long) realTimeValue);
    } else if (realTimeValue instanceof Float) {
      status = SignalLogger.writeFloat(signalPath, (Float) realTimeValue);
    } else if (realTimeValue instanceof Double) {
      status = SignalLogger.writeDouble(signalPath, (Double) realTimeValue);
    } else if (realTimeValue instanceof String) {
      status = SignalLogger.writeString(signalPath, (String) realTimeValue);
    } else if (realTimeValue instanceof boolean[]) {
      status = SignalLogger.writeBooleanArray(signalPath, (boolean[]) realTimeValue);
    } else if (realTimeValue instanceof long[]) {
      status = SignalLogger.writeIntegerArray(signalPath, (long[]) realTimeValue);
    } else if (realTimeValue instanceof float[]) {
      status = SignalLogger.writeFloatArray(signalPath, (float[]) realTimeValue);
    } else if (realTimeValue instanceof double[]) {
      status = SignalLogger.writeDoubleArray(signalPath, (double[]) realTimeValue);
    } else {
      status = StatusCode.NotFound;
    }

    // Populate the SignalData with the real-time value and status
    signalData.status = status;
    signalData.value = realTimeValue;
    return signalData;
  }
}
