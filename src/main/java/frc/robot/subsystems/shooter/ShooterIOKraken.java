package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ShooterIOKraken implements ShooterIO {
  private final TalonFX leaderMotor = new TalonFX(leaderMotorCanId, canBusName);
  private final TalonFX followerMotor = new TalonFX(followerMotorCanId, canBusName);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);

  public ShooterIOKraken() {
    TalonFXConfiguration leaderConfig = new TalonFXConfiguration();
    leaderConfig.MotorOutput =
        new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Coast)
            .withInverted(
                leaderInverted
                    ? InvertedValue.Clockwise_Positive
                    : InvertedValue.CounterClockwise_Positive);
    leaderConfig.CurrentLimits =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(supplyCurrentLimitAmps)
            .withSupplyCurrentLimitEnable(true);
    leaderConfig.Slot0 =
        new Slot0Configs()
            .withKP(velocityKp)
            .withKI(velocityKi)
            .withKD(velocityKd)
            .withKV(velocityKv);
    leaderMotor.getConfigurator().apply(leaderConfig);

    TalonFXConfiguration followerConfig = new TalonFXConfiguration();
    followerConfig.MotorOutput = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast);
    followerConfig.CurrentLimits =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(supplyCurrentLimitAmps)
            .withSupplyCurrentLimitEnable(true);
    followerMotor.getConfigurator().apply(followerConfig);

    followerMotor.setControl(
        new Follower(
            leaderMotorCanId,
            followerOpposeLeader ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned));
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.leaderConnected = true;
    inputs.followerConnected = true;
    inputs.leaderVelocityRps = leaderMotor.getVelocity().getValueAsDouble();
    inputs.followerVelocityRps = followerMotor.getVelocity().getValueAsDouble();
    inputs.leaderAppliedVolts = leaderMotor.getMotorVoltage().getValueAsDouble();
    inputs.followerAppliedVolts = followerMotor.getMotorVoltage().getValueAsDouble();
    inputs.leaderCurrentAmps = leaderMotor.getStatorCurrent().getValueAsDouble();
    inputs.followerCurrentAmps = followerMotor.getStatorCurrent().getValueAsDouble();
  }

  @Override
  public void setVelocityRpm(double rpm) {
    leaderMotor.setControl(velocityRequest.withVelocity(rpm / 60.0));
  }
}
