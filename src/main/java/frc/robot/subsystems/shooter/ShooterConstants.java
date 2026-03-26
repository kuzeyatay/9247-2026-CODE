package frc.robot.subsystems.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public final class ShooterConstants {
  private ShooterConstants() {}

  // FIXME: Set your real CAN IDs
  public static final int leaderMotorCanId = 8;
  public static final int followerMotorCanId = 7;

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

  public static final double spinupRpm = 2800.0;
  public static final double ampRpm = 1800.0;
  public static final double smartShootFixedPresetRpm = spinupRpm;
  public static final double smartShootReadyToleranceRpm = 100.0;
  public static final double smartShootLoadCompensationRpm = 150.0;
  public static final boolean smartShootWaitForHubAlignment = false;
  public static final double smartShootHubAlignmentToleranceDeg = 2.0;

  public static final double minAutoRpm = 1500.0;
  public static final double maxAutoRpm = 6500.0;
  public static final InterpolatingDoubleTreeMap autoHubRpmLookup = createAutoHubRpmLookup();
  public static final InterpolatingDoubleTreeMap remotePassRpmLookup = createRemotePassRpmLookup();

  private static InterpolatingDoubleTreeMap createAutoHubRpmLookup() {
    InterpolatingDoubleTreeMap table = new InterpolatingDoubleTreeMap();
    table.put(1.00, 2500.0);
    table.put(1.60, 2800.0);
    table.put(2.20, 3100.0);
    table.put(2.80, 3400.0);
    table.put(3.40, 3700.0);
    table.put(4.00, 4000.0);
    table.put(4.60, 4300.0);
    table.put(5.20, 4600.0);
    table.put(5.80, 4900.0);
    table.put(6.40, 5200.0);
    table.put(7.00, 5600.0);
    return table;
  }

  private static InterpolatingDoubleTreeMap createRemotePassRpmLookup() {
    InterpolatingDoubleTreeMap table = new InterpolatingDoubleTreeMap();
    table.put(0.00, 1500.0);
    table.put(1.00, 2000.0);
    table.put(2.10, 2500.0);
    table.put(4.13, 3000.0);
    table.put(6.17, 3500.0);
    table.put(7.70, 4000.0);
    table.put(9.60, 4500.0);
    table.put(11.00, 5000.0);
    return table;
  }
}
