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

  public static final double spinupRpm = 3500.0;
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
    table.put(1.00, 1620.0);
    table.put(1.50, 1820.0);
    table.put(2.00, 2020.0);
    table.put(2.50, 2200.0);
    table.put(3.00, 2380.0);
    table.put(3.50, 2550.0);
    table.put(4.00, 2710.0);
    table.put(4.50, 2870.0);
    table.put(5.00, 3020.0);
    table.put(5.50, 3170.0);
    table.put(6.00, 3310.0);
    return table;
  }

  private static InterpolatingDoubleTreeMap createRemotePassRpmLookup() {
    InterpolatingDoubleTreeMap table = new InterpolatingDoubleTreeMap();
    table.put(0.00, 1500.0);
    table.put(1.00, 1500.0);
    table.put(2.10, 2000.0);
    table.put(4.13, 2800.0);
    table.put(6.17, 3450.0);
    table.put(7.70, 4600.0);
    table.put(9.60, 5500.0);
    table.put(11.00, 6000.0);
    return table;
  }
}
