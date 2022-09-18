package frc.lib.math;

public class Conversions {

  public static double neoToDegrees(double counts, double gearRatio) {
    return counts * (360.0 / gearRatio);
  }

  public static double degreesToNeo(double degrees, double gearRatio) {
    double ticks = degrees / (360.0 / gearRatio);
    return ticks;
  }

  public static double neoToMPS(double RPM, double circumference, double gearRatio) {
    double wheelRPM = RPM / gearRatio;
    double wheelMPS = (wheelRPM * circumference) / 60;
    return wheelMPS;
  }

  public static double MPSToNeo(double velocity, double circumference, double gearRatio) {
    double rawRPM = ((velocity * 60) / circumference);
    double wheelRPM = rawRPM * gearRatio;
    return wheelRPM;
  }
}
