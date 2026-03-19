package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class IntakeIOSim implements IntakeIO {
  private enum ArmControlMode {
    POSITION,
    CURRENT
  }

  private final SingleJointedArmSim armSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60(2),
          armGearRatio,
          SingleJointedArmSim.estimateMOI(simArmLengthMeters, simArmMassKg),
          simArmLengthMeters,
          minAngleRad,
          maxAngleRad,
          true,
          stowAngleRad);
  private final PIDController controller = new PIDController(armKp, armKi, armKd);

  private ArmControlMode armControlMode = ArmControlMode.POSITION;
  private double setpointRad = stowAngleRad;
  private double armCurrentSetpointAmps = 0.0;
  private double rollerVelocitySetpointRps = 0.0;

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    double appliedVolts =
        switch (armControlMode) {
          case POSITION -> controller.calculate(armSim.getAngleRads(), setpointRad);
          case CURRENT -> (armCurrentSetpointAmps / armPeakTorqueCurrentAmps) * 12.0;
        };
    armSim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    armSim.update(0.02);

    inputs.connected = true;
    inputs.armAbsoluteEncoderConnected = true;
    inputs.armLeaderAbsoluteEncoderConnected = true;
    inputs.armFollowerAbsoluteEncoderConnected = true;
    inputs.armPositionRad = armSim.getAngleRads();
    inputs.armVelocityRadPerSec = armSim.getVelocityRadPerSec();
    inputs.armAbsoluteEncoderPositionRad = inputs.armPositionRad;
    inputs.armAbsoluteEncoderRawPositionRotations =
        inputs.armPositionRad / armAbsoluteEncoderPositionFactor;
    inputs.armLeaderAbsoluteEncoderPositionRad = inputs.armPositionRad;
    inputs.armFollowerAbsoluteEncoderPositionRad = inputs.armPositionRad;
    inputs.armLeaderAbsoluteEncoderRawPositionRotations =
        inputs.armAbsoluteEncoderRawPositionRotations;
    inputs.armFollowerAbsoluteEncoderRawPositionRotations =
        inputs.armAbsoluteEncoderRawPositionRotations;
    inputs.armAbsoluteEncoderSyncErrorRad = 0.0;
    inputs.armMotorPositionRad = inputs.armPositionRad;
    inputs.rollerVelocityRps = rollerVelocitySetpointRps;
    inputs.positionRad = inputs.armPositionRad;
    inputs.velocityRadPerSec = inputs.armVelocityRadPerSec;
    inputs.velocityRps = inputs.rollerVelocityRps;
    inputs.appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);
    inputs.currentAmps = 2.0 * Math.abs(inputs.appliedVolts / 12.0) * simStallCurrentAmps;
  }

  @Override
  public void setPositionRad(double positionRad) {
    armControlMode = ArmControlMode.POSITION;
    setpointRad = MathUtil.clamp(positionRad, minAngleRad, maxAngleRad);
  }

  @Override
  public void setArmCurrentAmps(double amps) {
    armControlMode = ArmControlMode.CURRENT;
    armCurrentSetpointAmps =
        MathUtil.clamp(amps, -armPeakTorqueCurrentAmps, armPeakTorqueCurrentAmps);
  }

  @Override
  public void setRollerVelocityRpm(double rpm) {
    rollerVelocitySetpointRps = rpm / 60.0;
  }
}
