package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class IntakeIOSim implements IntakeIO {
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
  private final ProfiledPIDController controller =
      new ProfiledPIDController(
          armKp,
          armKi,
          armKd,
          new TrapezoidProfile.Constraints(armProfileCruiseRadPerSec, armProfileAccelRadPerSecSq));
  private final ArmFeedforward feedforward =
      new ArmFeedforward(armKsVolts, armKgVolts, armKvVoltsPerRadPerSec);

  private double setpointRad = stowAngleRad;
  private double armVoltageSetpointVolts = 0.0;
  private boolean armVoltageControlEnabled = false;
  private double rollerVelocitySetpointRps = 0.0;

  public IntakeIOSim() {
    controller.reset(stowAngleRad);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    double appliedVolts;
    if (armVoltageControlEnabled) {
      controller.reset(armSim.getAngleRads());
      appliedVolts = armVoltageSetpointVolts;
    } else {
      double feedbackVolts = controller.calculate(armSim.getAngleRads(), setpointRad);
      TrapezoidProfile.State profileSetpoint = controller.getSetpoint();
      double feedforwardVolts =
          feedforward.calculate(
              sensorFrameRadToHorizontalRad(profileSetpoint.position), profileSetpoint.velocity);
      appliedVolts = feedbackVolts + feedforwardVolts;
    }
    armSim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    armSim.update(0.02);

    inputs.connected = true;
    inputs.armAbsoluteEncoderConnected = true;
    inputs.armLeaderAbsoluteEncoderConnected = true;
    inputs.armPositionRad = armSim.getAngleRads();
    inputs.armVelocityRadPerSec = armSim.getVelocityRadPerSec();
    inputs.armAbsoluteEncoderPositionRad = inputs.armPositionRad;
    inputs.armAbsoluteEncoderRawPositionRotations =
        inputs.armPositionRad / armAbsoluteEncoderPositionFactor;
    inputs.armLeaderAbsoluteEncoderPositionRad = inputs.armPositionRad;
    inputs.armLeaderAbsoluteEncoderRawPositionRotations =
        inputs.armAbsoluteEncoderRawPositionRotations;
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
    setpointRad = MathUtil.clamp(positionRad, minAngleRad, maxAngleRad);
    armVoltageControlEnabled = false;
    armVoltageSetpointVolts = 0.0;
  }

  @Override
  public void setArmVoltage(double volts) {
    armVoltageSetpointVolts = MathUtil.clamp(volts, -12.0, 12.0);
    armVoltageControlEnabled = true;
  }

  @Override
  public void setRollerVelocityRpm(double rpm) {
    rollerVelocitySetpointRps = rpm / 60.0;
  }

  private static double sensorFrameRadToHorizontalRad(double armSensorFrameRad) {
    return armSensorFrameRad - armHorizontalReferenceRad;
  }
}
