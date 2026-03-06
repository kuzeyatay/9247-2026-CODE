package frc.robot.subsystems.intake;

public final class IntakeConstants {
  private IntakeConstants() {}

  // FIXME: Set your real CAN IDs
  public static final int armLeaderMotorCanId = 30;
  public static final int armFollowerMotorCanId = 34;
  public static final int rollerMotorCanId = 35;

  public static final String canBusName = "rio";

  public static final boolean armInverted = false;
  public static final boolean armFollowerOpposeLeader = true;
  // Backward-compatible aliases for any stale references.

  public static final int armSupplyCurrentLimitAmps = 40;
  public static final double armGearRatio = 3.12; // Motor rotations per arm rotation

  // Arm angle setpoints (radians)
  public static final double stowAngleRad = Math.toRadians(90.0);
  public static final double intakeAngleRad = Math.toRadians(130.0);
  public static final double minAngleRad = Math.toRadians(0.0);
  public static final double maxAngleRad = Math.toRadians(140.0);

  // Closed-loop gains (tune on robot)
  public static final double armKp = 30.0;
  public static final double armKi = 0.0;
  public static final double armKd = 0.4;

  // Motion profile (motor-rotations units/sec and units/sec^2)
  public static final double armMotionMagicCruiseRotPerSec = 3.0;
  public static final double armMotionMagicAccelRotPerSecSq = 8.0;

  public static final boolean rollerInverted = false;
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
