package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.FieldConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public final class SuperstructureCommands {
  public enum SmartShootMode {
    AUTO_HUB,
    FIXED_PRESET
  }

  private enum SmartShootZone {
    ALLIANCE,
    TRANSITION,
    REMOTE
  }

  private enum SmartShootProfile {
    AUTO_HUB,
    FIXED_PRESET,
    REMOTE_PASS
  }

  private record SmartShootState(
      SmartShootZone zone,
      SmartShootMode requestedMode,
      SmartShootProfile profile,
      double distanceMeters,
      double baseRpm,
      boolean waitForAlignment) {}

  private SuperstructureCommands() {}

  private static final double shootSweepMinAngleRad = Units.degreesToRadians(100);
  private static final double shootSweepMaxAngleRad = IntakeConstants.openAngleRad;
  private static final double shootSweepPeriodSeconds = 3.0;
  private static final double shootSweepVoltageAmplitudeVolts = 4.0;
  private static final double autonomousShootSpinupSeconds = 10;
  private static final double autonomousShootDurationSeconds = 3.0;
  private static final double smartShootTransitionWidthMeters = 0.60;

  /** Runs intake + indexer to acquire a note. */
  public static Command intake(Intake intake, Indexer indexer) {
    return Commands.runEnd(
        () -> {
          intake.toIntakePosition();
          intake.setRollerVelocityRpm(IntakeConstants.rollerIntakeRpm);
          indexer.setVelocityRpm(IndexerConstants.intakeRpm);
        },
        () -> {
          intake.stopRoller();
          indexer.stop();
        },
        intake,
        indexer);
  }

  /** Reverses intake + indexer to clear a jam. */
  public static Command outtake(Intake intake, Indexer indexer) {
    return Commands.runEnd(
        () -> {
          intake.toIntakePosition();
          intake.setRollerVelocityRpm(IntakeConstants.rollerOuttakeRpm);
          indexer.setVelocityRpm(IndexerConstants.reverseRpm);
        },
        () -> {
          intake.stopRoller();
          indexer.stop();
        },
        intake,
        indexer);
  }

  /** Spins shooter only (no feed). */
  public static Command spinUpShooter(Shooter shooter) {
    return Commands.runEnd(
        () -> shooter.setVelocityRpm(ShooterConstants.spinupRpm), shooter::stop, shooter);
  }

  /** Spins shooter to the lookup-table hub RPM based on current field distance. */
  public static Command spinUpShooterAutoRpm(Drive drive, Vision vision, Shooter shooter) {
    return Commands.runEnd(
        () -> shooter.setVelocityRpm(calculateAutoHubShooterRpm(drive)), shooter::stop, shooter);
  }

  /** Runs shooter and indexer together for teleop hold-to-shoot. */
  public static Command shoot(Intake intake, Shooter shooter, Indexer indexer) {
    return Commands.runEnd(
        () -> {
          intake.setAngleRad(calculateShootSweepAngleRad());
          shooter.setVelocityRpm(ShooterConstants.spinupRpm);
          indexer.setVelocityRpm(IndexerConstants.feedToShooterRpm);
        },
        () -> {
          intake.toIntakePosition();
          shooter.stop();
          indexer.stop();
        },
        intake,
        shooter,
        indexer);
  }

  /** Smart shoot flow with zone-based pass logic and optional fixed hub preset mode. */
  public static Command shootAutoRpm(
      Drive drive,
      Vision vision,
      Intake intake,
      Shooter shooter,
      Indexer indexer,
      Supplier<SmartShootMode> shotModeSupplier) {
    return Commands.sequence(
            Commands.run(
                    () ->
                        applySmartShootSetpoints(
                            drive, vision, intake, shooter, indexer, shotModeSupplier, false),
                    intake,
                    shooter,
                    indexer)
                .until(() -> isSmartShootReady(drive, vision, shooter, shotModeSupplier)),
            Commands.run(
                () ->
                    applySmartShootSetpoints(
                        drive, vision, intake, shooter, indexer, shotModeSupplier, true),
                intake,
                shooter,
                indexer))
        .finallyDo(
            (interrupted) -> {
              intake.toIntakePosition();
              shooter.stop();
              indexer.stop();
            });
  }

  /** Spins up for a fixed delay, then feeds continuously until interrupted. */
  public static Command shootTimed(
      Intake intake, Shooter shooter, Indexer indexer, double spinupSeconds) {
    return Commands.sequence(
            Commands.run(
                    () -> {
                      intake.setAngleRad(calculateShootSweepAngleRad());
                      shooter.setVelocityRpm(ShooterConstants.spinupRpm);
                    },
                    intake,
                    shooter)
                .withTimeout(spinupSeconds),
            Commands.run(
                () -> {
                  intake.setAngleRad(calculateShootSweepAngleRad());
                  shooter.setVelocityRpm(ShooterConstants.spinupRpm);
                  indexer.setVelocityRpm(IndexerConstants.feedToShooterRpm);
                },
                intake,
                shooter,
                indexer))
        .finallyDo(
            (interrupted) -> {
              intake.toIntakePosition();
              shooter.stop();
              indexer.stop();
            });
  }

  /**
   * Spins up for a fixed delay, then feeds continuously while swinging the intake arm by voltage.
   */
  public static Command shootTimedVoltage(
      Intake intake, Shooter shooter, Indexer indexer, double spinupSeconds) {
    return Commands.sequence(
            Commands.run(
                    () -> {
                      intake.setArmVoltage(calculateShootSweepVoltageVolts());
                      shooter.setVelocityRpm(ShooterConstants.spinupRpm);
                    },
                    intake,
                    shooter)
                .withTimeout(spinupSeconds),
            Commands.run(
                () -> {
                  intake.setArmVoltage(calculateShootSweepVoltageVolts());
                  shooter.setVelocityRpm(ShooterConstants.spinupRpm);
                  indexer.setVelocityRpm(IndexerConstants.feedToShooterRpm);
                },
                intake,
                shooter,
                indexer))
        .finallyDo(
            (interrupted) -> {
              intake.toIntakePosition();
              shooter.stop();
              indexer.stop();
            });
  }

  /** Applies a fixed open-loop voltage to the intake arm for lift testing. */
  public static Command armLiftVoltageTest(Intake intake) {
    return Commands.runEnd(
        () -> intake.setArmVoltage(IntakeConstants.armTestLiftVolts),
        intake::toStowPosition,
        intake);
  }

  /** Standalone autonomous routine that only shoots the preloaded note. */
  public static Command autonomousShootOnly(Intake intake, Shooter shooter, Indexer indexer) {
    return shootTimedVoltage(intake, shooter, indexer, autonomousShootSpinupSeconds)
        .withTimeout(autonomousShootDurationSeconds);
  }

  private static double calculateShootSweepAngleRad() {
    double timeInCycle = Timer.getFPGATimestamp() % shootSweepPeriodSeconds;
    double normalized = timeInCycle / shootSweepPeriodSeconds;
    double midpoint = (shootSweepMinAngleRad + shootSweepMaxAngleRad) / 2.0;
    double amplitude = (shootSweepMaxAngleRad - shootSweepMinAngleRad) / 2.0;
    return midpoint + amplitude * Math.sin(2.0 * Math.PI * normalized - Math.PI / 2.0);
  }

  private static double calculateShootSweepVoltageVolts() {
    double timeInCycle = Timer.getFPGATimestamp() % shootSweepPeriodSeconds;
    double normalized = timeInCycle / shootSweepPeriodSeconds;
    return normalized < 0.5 ? shootSweepVoltageAmplitudeVolts : -shootSweepVoltageAmplitudeVolts;
  }

  private static void applySmartShootSetpoints(
      Drive drive,
      Vision vision,
      Intake intake,
      Shooter shooter,
      Indexer indexer,
      Supplier<SmartShootMode> shotModeSupplier,
      boolean feeding) {
    SmartShootState state = getSmartShootState(drive, shotModeSupplier);
    double commandedRpm =
        MathUtil.clamp(
            state.baseRpm() + (feeding ? ShooterConstants.smartShootLoadCompensationRpm : 0.0),
            0.0,
            ShooterConstants.maxAutoRpm);

    intake.setArmVoltage(calculateShootSweepVoltageVolts());
    shooter.setVelocityRpm(commandedRpm);
    if (feeding) {
      indexer.setVelocityRpm(IndexerConstants.feedToShooterRpm);
    } else {
      indexer.stop();
    }

    logSmartShootState(vision, state, commandedRpm, feeding);
  }

  private static boolean isSmartShootReady(
      Drive drive, Vision vision, Shooter shooter, Supplier<SmartShootMode> shotModeSupplier) {
    SmartShootState state = getSmartShootState(drive, shotModeSupplier);
    boolean shooterReady = shooter.isAtSetpoint();
    boolean alignmentReady = isAlignmentReady(vision, state);
    Logger.recordOutput("Shooter/SmartShoot/ShooterReady", shooterReady);
    Logger.recordOutput("Shooter/SmartShoot/AlignmentReady", alignmentReady);
    return shooterReady && alignmentReady;
  }

  private static boolean isAlignmentReady(Vision vision, SmartShootState state) {
    if (!state.waitForAlignment()) {
      return true;
    }
    if (!vision.hasAllowedAlignTarget()) {
      return false;
    }
    return Math.abs(vision.getAllowedAlignTargetX().getDegrees())
        <= ShooterConstants.smartShootHubAlignmentToleranceDeg;
  }

  private static SmartShootState getSmartShootState(
      Drive drive, Supplier<SmartShootMode> shotModeSupplier) {
    Translation2d robotTranslation = drive.getPose().getTranslation();
    double hubDistanceMeters = getFriendlyHubDistanceMeters(robotTranslation);
    SmartShootZone zone = getSmartShootZone(robotTranslation);
    SmartShootMode requestedMode = shotModeSupplier.get();
    if (requestedMode == null) {
      requestedMode = SmartShootMode.AUTO_HUB;
    }

    if (zone == SmartShootZone.REMOTE) {
      // Use hub distance as a simple proxy until a dedicated pass target is modeled.
      return new SmartShootState(
          zone,
          requestedMode,
          SmartShootProfile.REMOTE_PASS,
          hubDistanceMeters,
          ShooterConstants.remotePassRpmLookup.get(hubDistanceMeters),
          false);
    }

    if (requestedMode == SmartShootMode.FIXED_PRESET) {
      return new SmartShootState(
          zone,
          requestedMode,
          SmartShootProfile.FIXED_PRESET,
          hubDistanceMeters,
          ShooterConstants.smartShootFixedPresetRpm,
          false);
    }

    return new SmartShootState(
        zone,
        requestedMode,
        SmartShootProfile.AUTO_HUB,
        hubDistanceMeters,
        calculateAutoHubShooterRpm(drive),
        ShooterConstants.smartShootWaitForHubAlignment);
  }

  private static double calculateAutoHubShooterRpm(Drive drive) {
    double distanceMeters = getFriendlyHubDistanceMeters(drive.getPose().getTranslation());
    double commandedRpm =
        MathUtil.clamp(
            ShooterConstants.autoHubRpmLookup.get(distanceMeters),
            ShooterConstants.minAutoRpm,
            ShooterConstants.maxAutoRpm);
    Logger.recordOutput("Shooter/SmartShoot/HubDistanceMeters", distanceMeters);
    Logger.recordOutput("Shooter/SmartShoot/HubLookupRpm", commandedRpm);
    return commandedRpm;
  }

  private static SmartShootZone getSmartShootZone(Translation2d robotTranslation) {
    double blueX = getBlueAllianceRelativeTranslation(robotTranslation).getX();
    double transitionStart =
        FieldConstants.LinesVertical.allianceZone - smartShootTransitionWidthMeters;
    if (blueX < transitionStart) {
      return SmartShootZone.ALLIANCE;
    }
    if (blueX < FieldConstants.LinesVertical.hubCenter) {
      return SmartShootZone.TRANSITION;
    }
    return SmartShootZone.REMOTE;
  }

  private static Translation2d getFriendlyHubCenter() {
    return isRedAlliance()
        ? FieldConstants.Hub.oppTopCenterPoint.toTranslation2d()
        : FieldConstants.Hub.topCenterPoint.toTranslation2d();
  }

  private static double getFriendlyHubDistanceMeters(Translation2d robotTranslation) {
    return robotTranslation.getDistance(getFriendlyHubCenter());
  }

  private static Translation2d getBlueAllianceRelativeTranslation(Translation2d fieldTranslation) {
    if (!isRedAlliance()) {
      return fieldTranslation;
    }
    return new Translation2d(
        FieldConstants.fieldLength - fieldTranslation.getX(), fieldTranslation.getY());
  }

  private static boolean isRedAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
  }

  private static void logSmartShootState(
      Vision vision, SmartShootState state, double commandedRpm, boolean feeding) {
    Logger.recordOutput("Shooter/SmartShoot/Zone", state.zone().name());
    Logger.recordOutput("Shooter/SmartShoot/RequestedMode", state.requestedMode().name());
    Logger.recordOutput("Shooter/SmartShoot/Profile", state.profile().name());
    Logger.recordOutput("Shooter/SmartShoot/DistanceMeters", state.distanceMeters());
    Logger.recordOutput("Shooter/SmartShoot/BaseRpm", state.baseRpm());
    Logger.recordOutput("Shooter/SmartShoot/CommandedRpm", commandedRpm);
    Logger.recordOutput(
        "Shooter/SmartShoot/LoadCompensationRpm", ShooterConstants.smartShootLoadCompensationRpm);
    Logger.recordOutput("Shooter/SmartShoot/Feeding", feeding);
    Logger.recordOutput(
        "Shooter/SmartShoot/AlignmentErrorDeg",
        vision.hasAllowedAlignTarget() ? vision.getAllowedAlignTargetX().getDegrees() : Double.NaN);
  }
}
