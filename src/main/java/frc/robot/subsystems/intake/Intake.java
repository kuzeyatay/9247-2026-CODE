package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

public class Intake extends SubsystemBase {
  private enum ArmControlMode {
    POSITION,
    CURRENT_TO_TARGET
  }

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final LoggedMechanism2d mechanism = new LoggedMechanism2d(1.0, 1.0);
  private final LoggedMechanismLigament2d measuredLigament;
  private final LoggedMechanismLigament2d setpointLigament;
  private ArmControlMode armControlMode = ArmControlMode.POSITION;
  private double armSetpointRad = stowAngleRad;
  private double armCurrentTargetRad = stowAngleRad;
  private double armCurrentCommandAmps = 0.0;
  private double rollerSetpointRpm = 0.0;

  public Intake(IntakeIO io) {
    this.io = io;
    var root = mechanism.getRoot("IntakeRoot", 0.5, 0.2);
    measuredLigament =
        root.append(
            new LoggedMechanismLigament2d(
                "Measured", simArmLengthMeters, 0.0, 6, new Color8Bit(Color.kLime)));
    setpointLigament =
        root.append(
            new LoggedMechanismLigament2d(
                "Setpoint", simArmLengthMeters * 0.9, 0.0, 3, new Color8Bit(Color.kOrange)));
  }

  public void setAngleRad(double angleRad) {
    armSetpointRad = clampArmAngle(angleRad);
    armControlMode = ArmControlMode.POSITION;
    io.setPositionRad(armSetpointRad);
  }

  public void setRollerVelocityRpm(double rpm) {
    rollerSetpointRpm = rpm;
    io.setRollerVelocityRpm(rpm);
  }

  public void stopRoller() {
    setRollerVelocityRpm(0.0);
  }

  public void toIntakePosition() {
    setCurrentControlledTarget(intakeAngleRad, armOpenCurrentAmps);
  }

  public void toShootPosition() {
    setAngleRad(shootAngleRad);
  }

  public void toStowPosition() {
    setCurrentControlledTarget(stowAngleRad, armCloseCurrentAmps);
  }

  public void stop() {
    stopRoller();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    updateCurrentControlledArm();
    measuredLigament.setAngle(Units.radiansToDegrees(inputs.armPositionRad));
    setpointLigament.setAngle(Units.radiansToDegrees(armSetpointRad));
    Logger.processInputs("Intake", inputs);
    Logger.recordOutput("Intake/ArmSetpointRad", armSetpointRad);
    Logger.recordOutput("Intake/ArmCurrentTargetRad", armCurrentTargetRad);
    Logger.recordOutput("Intake/ArmCurrentCommandAmps", armCurrentCommandAmps);
    Logger.recordOutput("Intake/ArmControlMode", armControlMode.name());
    Logger.recordOutput("Intake/RollerSetpointRpm", rollerSetpointRpm);
    Logger.recordOutput("Intake/Mechanism2d", mechanism);
  }

  private void setCurrentControlledTarget(double targetRad, double currentAmps) {
    armSetpointRad = clampArmAngle(targetRad);
    armCurrentTargetRad = armSetpointRad;
    armCurrentCommandAmps = currentAmps;
    armControlMode = ArmControlMode.CURRENT_TO_TARGET;
    updateCurrentControlledArm();
  }

  private void updateCurrentControlledArm() {
    if (armControlMode != ArmControlMode.CURRENT_TO_TARGET) {
      return;
    }

    double errorRad = armCurrentTargetRad - inputs.armPositionRad;
    if (Math.abs(errorRad) <= armCurrentControlToleranceRad) {
      io.setArmCurrentAmps(0.0);
      return;
    }

    io.setArmCurrentAmps(Math.copySign(Math.abs(armCurrentCommandAmps), errorRad));
  }

  private static double clampArmAngle(double angleRad) {
    return Math.max(minAngleRad, Math.min(maxAngleRad, angleRad));
  }
}
