package frc.robot.subsystems.shooter;

import edu.wpi.first.math.util.Units;

public final class ShooterConstants {
  private ShooterConstants() {}

  // FIXME: Set your real CAN IDs
  public static final int leaderMotorCanId = 32;
  public static final int followerMotorCanId = 33;

  public static final String canBusName = "rio";

  public static final boolean leaderInverted = false;
  public static final boolean followerOpposeLeader = true;
  public static final double shooterGearRatio = 24.0 / 18.0; // motor rotations per shooter rotation
  public static final int supplyCurrentLimitAmps = 50;
  public static final double simFreeSpeedRps = 110.0;
  public static final double simStallCurrentAmps = 80.0;

  // Velocity control gains (rotations/sec units)
  public static final double velocityKp = 0.15;
  public static final double velocityKi = 0.0;
  public static final double velocityKd = 0.0;
  public static final double velocityKv = 0.12;

  public static final double spinupRpm = 4200.0;
  public static final double ampRpm = 1800.0;

  // Auto-RPM shot model constants (fixed-angle shooter).
  public static final double fixedShotAngleDeg = 45.0;
  public static final double releaseHeightMeters = 0.70;
  public static final double wheelDiameterMeters = Units.inchesToMeters(4.0);
  public static final double gravityMetersPerSecondSquared = 9.81;
  public static final double rpmCompensation = 1.0;
  public static final double autoRpmOffset = 500.0;
  public static final double dragRpmPerMeter = 120.0;
  public static final double minAutoRpm = 1500.0;
  public static final double maxAutoRpm = 6500.0;
}
