package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
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
import org.littletonrobotics.junction.Logger;

public final class SuperstructureCommands {
  private SuperstructureCommands() {}

  private static final double shootSweepMinAngleRad = Units.degreesToRadians(100);
  private static final double shootSweepMaxAngleRad = IntakeConstants.openAngleRad;
  private static final double shootSweepPeriodSeconds = 3.0;
  private static final double shootSweepVoltageAmplitudeVolts = 4.0;

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

  /** Spins shooter to an automatically calculated RPM from visible hub AprilTag distance. */
  public static Command spinUpShooterAutoRpm(Drive drive, Vision vision, Shooter shooter) {
    return Commands.runEnd(
        () -> shooter.setVelocityRpm(calculateAutoShooterRpm(drive, vision)),
        shooter::stop,
        shooter);
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

  /** Runs shooter and indexer together using auto-calculated shooter RPM from hub tags. */
  public static Command shootAutoRpm(
      Drive drive, Vision vision, Intake intake, Shooter shooter, Indexer indexer) {
    return Commands.runEnd(
        () -> {
          intake.setAngleRad(calculateShootSweepAngleRad());
          shooter.setVelocityRpm(calculateAutoShooterRpm(drive, vision));
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

  private static double calculateAutoShooterRpm(Drive drive, Vision vision) {
    var robotPose = drive.getPose();
    double distanceMeters = vision.getAllowedAlignTargetDistanceMeters(robotPose);
    double targetHeightMeters = FieldConstants.Hub.height;
    Logger.recordOutput("Shooter/AutoAim/HorizontalDistanceMeters", distanceMeters);
    Logger.recordOutput("Shooter/AutoAim/TargetHeightMeters", targetHeightMeters);

    if (!Double.isFinite(distanceMeters) || distanceMeters <= 0.0) {
      Logger.recordOutput("Shooter/AutoAim/RobotRadialVelocityMps", 0.0);
      Logger.recordOutput("Shooter/AutoAim/TargetLaunchSpeedMps", 0.0);
      return ShooterConstants.spinupRpm;
    }

    double angleRad = Units.degreesToRadians(ShooterConstants.fixedShotAngleDeg);
    double deltaHeightMeters = targetHeightMeters - ShooterConstants.releaseHeightMeters;

    // Projectile equation solved for launch speed with fixed angle.
    double denominator =
        2.0
            * Math.pow(Math.cos(angleRad), 2.0)
            * (distanceMeters * Math.tan(angleRad) - deltaHeightMeters);
    if (denominator <= 0.0) {
      return ShooterConstants.spinupRpm;
    }

    double launchSpeedMetersPerSec =
        Math.sqrt(
            (ShooterConstants.gravityMetersPerSecondSquared * distanceMeters * distanceMeters)
                / denominator);

    // Add field-relative robot translation projected onto line of fire (toward target).
    Rotation2d bearingToTarget = vision.getAllowedAlignTargetBearing(robotPose);
    ChassisSpeeds fieldSpeeds = drive.getFieldRelativeSpeeds();
    double robotRadialVelocityMps =
        fieldSpeeds.vxMetersPerSecond * bearingToTarget.getCos()
            + fieldSpeeds.vyMetersPerSecond * bearingToTarget.getSin();

    double targetHorizontalLaunchMps = launchSpeedMetersPerSec * Math.cos(angleRad);
    double compensatedHorizontalLaunchMps = targetHorizontalLaunchMps - robotRadialVelocityMps;
    if (compensatedHorizontalLaunchMps <= 0.0) {
      return ShooterConstants.minAutoRpm;
    }

    double compensatedLaunchSpeedMetersPerSec = compensatedHorizontalLaunchMps / Math.cos(angleRad);
    Logger.recordOutput("Shooter/AutoAim/RobotRadialVelocityMps", robotRadialVelocityMps);
    Logger.recordOutput("Shooter/AutoAim/TargetLaunchSpeedMps", compensatedLaunchSpeedMetersPerSec);

    double wheelRps =
        compensatedLaunchSpeedMetersPerSec / (Math.PI * ShooterConstants.wheelDiameterMeters);
    double rpm = wheelRps * 60.0 * ShooterConstants.rpmCompensation;
    double dragCompensationRpm = ShooterConstants.dragRpmPerMeter * distanceMeters;
    double commandedRpm =
        MathUtil.clamp(
            rpm + dragCompensationRpm + ShooterConstants.autoRpmOffset,
            0.0,
            ShooterConstants.maxAutoRpm);
    Logger.recordOutput("Shooter/AutoAim/DragCompensationRpm", dragCompensationRpm);
    Logger.recordOutput("Shooter/AutoAim/OffsetCompensationRpm", ShooterConstants.autoRpmOffset);
    Logger.recordOutput("Shooter/AutoAim/CalculatedRpmRaw", rpm);
    Logger.recordOutput("Shooter/AutoAim/CalculatedRpmCommanded", commandedRpm);

    return commandedRpm;
  }
}
