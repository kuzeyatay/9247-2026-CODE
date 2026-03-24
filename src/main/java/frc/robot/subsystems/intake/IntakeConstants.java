package frc.robot.subsystems.intake;

public final class IntakeConstants {
  private IntakeConstants() {}

  // FIXME: Set your real CAN IDs
  public static final int armLeaderMotorCanId = 5;
  public static final int armFollowerMotorCanId = 1;
  public static final int rollerMotorCanId = 6;

  public static final String canBusName = "rio";

  public static final boolean armInverted = true;
  public static final boolean armFollowerOpposeLeader = true;
  public static final int armSupplyCurrentLimitAmps = 40;
  public static final boolean armVoltageModeSupplyCurrentLimitEnable = false;
  public static final int armVoltageModeSupplyCurrentLimitAmps = 80;
  public static final double armGearRatio = (64.0 / 12.0); // Motor rotations per arm rotation
  public static final int armLeaderAbsoluteEncoderDioChannel = 0;
  public static final boolean armLeaderAbsoluteEncoderInverted = true;
  public static final double armAbsoluteEncoderGearRatio = 1.0;
  public static final double armAbsoluteEncoderPositionFactor =
      (2.0 * Math.PI) / armAbsoluteEncoderGearRatio; // Encoder rotations -> arm radians
  public static final double armAbsoluteEncoderVelocityFactor =
      (2.0 * Math.PI) / armAbsoluteEncoderGearRatio; // Encoder rotations/sec -> arm rad/sec
  public static final double armLeaderAbsoluteEncoderOffsetRad = 0.0;

  // Arm setpoints in leader absolute-encoder radians.
  public static final double closedAngleRad = 1.800; // 1.740
  public static final double openAngleRad = 2.500; // 2.984
  public static final double stowAngleRad = closedAngleRad;
  public static final double intakeAngleRad = openAngleRad;
  public static final double minAngleRad = closedAngleRad;
  public static final double maxAngleRad = openAngleRad;
  public static final double armHorizontalReferenceRad =
      closedAngleRad - (Math.PI / 2.0); // Assumes closed is vertical; tune on robot

  // Closed-loop gains (tune on robot)
  public static final double armKp = 30.0;
  public static final double armKi = 0.0;
  public static final double armKd = 0.4;
  public static final double armKsVolts = 0.15;
  public static final double armKgVolts = 0.35;
  public static final double armKvVoltsPerRadPerSec = 0.08;
  public static final double armTestLiftVolts = 8.0;

  // Motion profile (motor-rotations units/sec and units/sec^2)
  public static final double armMotionMagicCruiseRotPerSec = 3.0;
  public static final double armMotionMagicAccelRotPerSecSq = 8.0;
  public static final double armProfileCruiseRadPerSec =
      (armMotionMagicCruiseRotPerSec / armGearRatio) * 2.0 * Math.PI;
  public static final double armProfileAccelRadPerSecSq =
      (armMotionMagicAccelRotPerSecSq / armGearRatio) * 2.0 * Math.PI;
  public static final double armPeakTorqueCurrentAmps = 80.0;
  public static final double armVoltageModePeakTorqueCurrentAmps = 160.0;

  public static final boolean rollerInverted = true;
  public static final int rollerSupplyCurrentLimitAmps = 40;
  public static final double rollerVelocityKp = 0.10;
  public static final double rollerVelocityKi = 0.0;
  public static final double rollerVelocityKd = 0.0;
  public static final double rollerVelocityKv = 0.12;
  public static final double rollerIntakeRpm = 2400.0;
  public static final double rollerOuttakeRpm = -1200.0;

  // Sim properties
  public static final double simArmLengthMeters = 0.35;
  public static final double simArmMassKg = 4.0;
  public static final double simArmFreeSpeedRps = 85.0;
  public static final double simRollerFreeSpeedRps = 90.0;
  public static final double simStallCurrentAmps = 60.0;
}
