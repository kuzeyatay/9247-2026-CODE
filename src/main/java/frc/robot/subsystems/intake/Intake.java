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
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final LoggedMechanism2d mechanism = new LoggedMechanism2d(1.0, 1.0);
  private final LoggedMechanismLigament2d measuredLigament;
  private final LoggedMechanismLigament2d setpointLigament;
  private double armSetpointRad = stowAngleRad;
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
    armSetpointRad = angleRad;
    io.setPositionRad(angleRad);
  }

  public void setRollerVelocityRpm(double rpm) {
    rollerSetpointRpm = rpm;
    io.setRollerVelocityRpm(rpm);
  }

  public void stopRoller() {
    setRollerVelocityRpm(0.0);
  }

  public void toIntakePosition() {
    setAngleRad(intakeAngleRad);
  }

  public void toStowPosition() {
    setAngleRad(stowAngleRad);
  }

  public void stop() {
    stopRoller();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    measuredLigament.setAngle(Units.radiansToDegrees(inputs.armPositionRad));
    setpointLigament.setAngle(Units.radiansToDegrees(armSetpointRad));
    Logger.processInputs("Intake", inputs);
    Logger.recordOutput("Intake/ArmSetpointRad", armSetpointRad);
    Logger.recordOutput("Intake/RollerSetpointRpm", rollerSetpointRpm);
    Logger.recordOutput("Intake/Mechanism2d", mechanism);
  }
}
