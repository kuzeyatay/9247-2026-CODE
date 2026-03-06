package frc.robot.subsystems.indexer;

public final class IndexerConstants {
  private IndexerConstants() {}

  // FIXME: Set your real CAN IDs
  public static final int leaderMotorCanId = 31;
  public static final int followerMotorCanId = 34;

  public static final boolean inverted = false;
  public static final boolean followerOpposeLeader = true;
  public static final int supplyCurrentLimitAmps = 35;
  public static final double indexerGearRatio = 24.0 / 15.0; // motor rotations per indexer rotation
  public static final double simFreeSpeedRps = 70.0;
  public static final double simStallCurrentAmps = 55.0;

  // Velocity control gains (rotations/sec units)
  public static final double velocityKp = 0.08;
  public static final double velocityKi = 0.0;
  public static final double velocityKd = 0.0;
  public static final double velocityKv = 0.10;

  public static final double intakeRpm = 1800.0;
  public static final double feedToShooterRpm = 3200.0;
  public static final double reverseRpm = -1200.0;
}
