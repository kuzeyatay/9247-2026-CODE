package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class IntakeIOSim implements IntakeIO {
  private final SingleJointedArmSim armSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60(1),
          armGearRatio,
          SingleJointedArmSim.estimateMOI(simArmLengthMeters, simArmMassKg),
          simArmLengthMeters,
          minAngleRad,
          maxAngleRad,
          true,
          stowAngleRad);
  private final PIDController controller = new PIDController(armKp, armKi, armKd);

  private double setpointRad = stowAngleRad;
  private double rollerVelocitySetpointRps = 0.0;

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    double appliedVolts = controller.calculate(armSim.getAngleRads(), setpointRad);
    armSim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    armSim.update(0.02);

    inputs.connected = true;
    inputs.armPositionRad = armSim.getAngleRads();
    inputs.armVelocityRadPerSec = armSim.getVelocityRadPerSec();
    inputs.rollerVelocityRps = rollerVelocitySetpointRps;
    inputs.positionRad = inputs.armPositionRad;
    inputs.velocityRadPerSec = inputs.armVelocityRadPerSec;
    inputs.velocityRps = inputs.rollerVelocityRps;
    inputs.appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);
    inputs.currentAmps = 2.0 * Math.abs(inputs.appliedVolts / 12.0) * simStallCurrentAmps;
  }

  @Override
  public void setPositionRad(double positionRad) {
    setpointRad = MathUtil.clamp(positionRad, minAngleRad, maxAngleRad);
  }

  @Override
  public void setRollerVelocityRpm(double rpm) {
    rollerVelocitySetpointRps = rpm / 60.0;
  }
}
