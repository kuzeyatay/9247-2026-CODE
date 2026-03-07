package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeIOKraken implements IntakeIO {
  private final TalonFX armLeaderMotor =
      new TalonFX(IntakeConstants.armLeaderMotorCanId, IntakeConstants.canBusName);
  private final TalonFX armFollowerMotor =
      new TalonFX(IntakeConstants.armFollowerMotorCanId, IntakeConstants.canBusName);
  private final TalonFX rollerMotor =
      new TalonFX(IntakeConstants.rollerMotorCanId, IntakeConstants.canBusName);
  private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0.0);
  private final VelocityVoltage rollerVelocityRequest = new VelocityVoltage(0.0);

  public IntakeIOKraken() {
    TalonFXConfiguration armConfig = new TalonFXConfiguration();
    armConfig.MotorOutput =
        new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(
                IntakeConstants.armInverted
                    ? InvertedValue.Clockwise_Positive
                    : InvertedValue.CounterClockwise_Positive);
    armConfig.CurrentLimits =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(IntakeConstants.armSupplyCurrentLimitAmps)
            .withSupplyCurrentLimitEnable(true);
    armConfig.Slot0 =
        new Slot0Configs()
            .withKP(IntakeConstants.armKp)
            .withKI(IntakeConstants.armKi)
            .withKD(IntakeConstants.armKd);
    armConfig.MotionMagic =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(IntakeConstants.armMotionMagicCruiseRotPerSec)
            .withMotionMagicAcceleration(IntakeConstants.armMotionMagicAccelRotPerSecSq);
    armLeaderMotor.getConfigurator().apply(armConfig);
    armFollowerMotor.getConfigurator().apply(armConfig);
    armFollowerMotor.setControl(
        new Follower(
            IntakeConstants.armLeaderMotorCanId,
            IntakeConstants.armFollowerOpposeLeader
                ? MotorAlignmentValue.Opposed
                : MotorAlignmentValue.Aligned));
    armLeaderMotor.setPosition(armRadToMotorRot(IntakeConstants.stowAngleRad));

    TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    rollerConfig.MotorOutput =
        new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(
                IntakeConstants.rollerInverted
                    ? InvertedValue.Clockwise_Positive
                    : InvertedValue.CounterClockwise_Positive);
    rollerConfig.CurrentLimits =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(IntakeConstants.rollerSupplyCurrentLimitAmps)
            .withSupplyCurrentLimitEnable(true);
    rollerConfig.Slot0 =
        new Slot0Configs()
            .withKP(IntakeConstants.rollerVelocityKp)
            .withKI(IntakeConstants.rollerVelocityKi)
            .withKD(IntakeConstants.rollerVelocityKd)
            .withKV(IntakeConstants.rollerVelocityKv);
    rollerMotor.getConfigurator().apply(rollerConfig);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.connected = true;
    double armMotorPositionRot = armLeaderMotor.getPosition().getValueAsDouble();
    double armMotorVelocityRps = armLeaderMotor.getVelocity().getValueAsDouble();
    inputs.armPositionRad = motorRotToArmRad(armMotorPositionRot);
    inputs.armVelocityRadPerSec = motorVelocityToArmRadPerSec(armMotorVelocityRps);
    inputs.rollerVelocityRps = rollerMotor.getVelocity().getValueAsDouble();
    inputs.positionRad = inputs.armPositionRad;
    inputs.velocityRadPerSec = inputs.armVelocityRadPerSec;
    inputs.velocityRps = inputs.rollerVelocityRps;
    inputs.appliedVolts = armLeaderMotor.getMotorVoltage().getValueAsDouble();
    inputs.currentAmps =
        armLeaderMotor.getStatorCurrent().getValueAsDouble()
            + armFollowerMotor.getStatorCurrent().getValueAsDouble()
            + rollerMotor.getStatorCurrent().getValueAsDouble();
  }

  @Override
  public void setPositionRad(double positionRad) {
    double clamped =
        Math.max(IntakeConstants.minAngleRad, Math.min(IntakeConstants.maxAngleRad, positionRad));
    armLeaderMotor.setControl(positionRequest.withPosition(armRadToMotorRot(clamped)));
  }

  @Override
  public void setRollerVelocityRpm(double rpm) {
    rollerMotor.setControl(rollerVelocityRequest.withVelocity(rpm / 60.0));
  }

  private static double armRadToMotorRot(double armRad) {
    return (armRad / (2.0 * Math.PI)) * IntakeConstants.armGearRatio;
  }

  private static double motorRotToArmRad(double motorRot) {
    return (motorRot / IntakeConstants.armGearRatio) * 2.0 * Math.PI;
  }

  private static double motorVelocityToArmRadPerSec(double motorVelocityRps) {
    return (motorVelocityRps / IntakeConstants.armGearRatio) * 2.0 * Math.PI;
  }
}
