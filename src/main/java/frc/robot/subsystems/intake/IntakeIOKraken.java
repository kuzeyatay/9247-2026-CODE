package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;

public class IntakeIOKraken implements IntakeIO {
  private enum ArmControlMode {
    POSITION,
    CURRENT
  }

  private final TalonFX armLeaderMotor =
      new TalonFX(IntakeConstants.armLeaderMotorCanId, IntakeConstants.canBusName);
  private final TalonFX armFollowerMotor =
      new TalonFX(IntakeConstants.armFollowerMotorCanId, IntakeConstants.canBusName);
  private final TalonFX rollerMotor =
      new TalonFX(IntakeConstants.rollerMotorCanId, IntakeConstants.canBusName);
  private final DutyCycleEncoder armLeaderAbsoluteEncoder =
      new DutyCycleEncoder(IntakeConstants.armLeaderAbsoluteEncoderDioChannel);
  private final DutyCycleEncoder armFollowerAbsoluteEncoder =
      new DutyCycleEncoder(IntakeConstants.armFollowerAbsoluteEncoderDioChannel);
  private final VoltageOut armVoltageRequest = new VoltageOut(0.0);
  private final TorqueCurrentFOC armCurrentRequest = new TorqueCurrentFOC(0.0);
  private final VelocityVoltage rollerVelocityRequest = new VelocityVoltage(0.0);
  private final PIDController armController =
      new PIDController(IntakeConstants.armKp, IntakeConstants.armKi, IntakeConstants.armKd);
  private final Debouncer armLeaderAbsoluteEncoderConnectedDebounce = new Debouncer(0.25);
  private final Debouncer armFollowerAbsoluteEncoderConnectedDebounce = new Debouncer(0.25);

  private ArmControlMode armControlMode = ArmControlMode.POSITION;
  private double armSetpointRad = IntakeConstants.stowAngleRad;
  private double armCurrentSetpointAmps = 0.0;
  private boolean armMotorPositionSynchronized = false;
  private double lastArmPositionRad = IntakeConstants.stowAngleRad;
  private double lastTimestampSec = Timer.getFPGATimestamp();

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
    armConfig.TorqueCurrent.PeakForwardTorqueCurrent = IntakeConstants.armPeakTorqueCurrentAmps;
    armConfig.TorqueCurrent.PeakReverseTorqueCurrent = -IntakeConstants.armPeakTorqueCurrentAmps;
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
    double armMotorPositionRot = armLeaderMotor.getPosition().getValueAsDouble();
    double armMotorVelocityRps = armLeaderMotor.getVelocity().getValueAsDouble();
    double armMotorPositionRad = motorRotToArmRad(armMotorPositionRot);
    double armLeaderAbsoluteEncoderRawPositionRotations = armLeaderAbsoluteEncoder.get();
    double armFollowerAbsoluteEncoderRawPositionRotations = armFollowerAbsoluteEncoder.get();
    double armLeaderAbsoluteEncoderPositionRad =
        absoluteEncoderRotationsToArmRad(
            armLeaderAbsoluteEncoderRawPositionRotations,
            IntakeConstants.armLeaderAbsoluteEncoderInverted,
            IntakeConstants.armLeaderAbsoluteEncoderOffsetRad);
    double armFollowerAbsoluteEncoderPositionRad =
        absoluteEncoderRotationsToArmRad(
            armFollowerAbsoluteEncoderRawPositionRotations,
            IntakeConstants.armFollowerAbsoluteEncoderInverted,
            IntakeConstants.armFollowerAbsoluteEncoderOffsetRad);
    boolean armLeaderAbsoluteEncoderPresent = armLeaderAbsoluteEncoder.isConnected();
    boolean armFollowerAbsoluteEncoderPresent = armFollowerAbsoluteEncoder.isConnected();
    boolean armLeaderAbsoluteEncoderConnected =
        armLeaderAbsoluteEncoderConnectedDebounce.calculate(armLeaderAbsoluteEncoderPresent);
    boolean armFollowerAbsoluteEncoderConnected =
        armFollowerAbsoluteEncoderConnectedDebounce.calculate(armFollowerAbsoluteEncoderPresent);
    boolean armAbsoluteEncoderConnected =
        armLeaderAbsoluteEncoderConnected || armFollowerAbsoluteEncoderConnected;

    if (armAbsoluteEncoderConnected) {
      synchronizeArmMotorPosition(
          fuseArmPositionRad(
              armLeaderAbsoluteEncoderConnected,
              armLeaderAbsoluteEncoderPositionRad,
              armFollowerAbsoluteEncoderConnected,
              armFollowerAbsoluteEncoderPositionRad));
    } else {
      armMotorPositionSynchronized = false;
    }

    double armPositionRad =
        fuseArmPositionRad(
            armLeaderAbsoluteEncoderConnected,
            armLeaderAbsoluteEncoderPositionRad,
            armFollowerAbsoluteEncoderConnected,
            armFollowerAbsoluteEncoderPositionRad,
            armMotorPositionRad);
    double armAbsoluteEncoderPositionRad =
        armAbsoluteEncoderConnected ? armPositionRad : armMotorPositionRad;
    double armVelocityRadPerSec = motorVelocityToArmRadPerSec(armMotorVelocityRps);
    double timestampSec = Timer.getFPGATimestamp();
    double dtSec = timestampSec - lastTimestampSec;
    if (armAbsoluteEncoderConnected && dtSec > 1e-6) {
      armVelocityRadPerSec = MathUtil.angleModulus(armPositionRad - lastArmPositionRad) / dtSec;
    }

    runArmControl(armPositionRad);

    lastArmPositionRad = armPositionRad;
    lastTimestampSec = timestampSec;

    inputs.connected = armAbsoluteEncoderConnected;
    inputs.armAbsoluteEncoderConnected = armAbsoluteEncoderConnected;
    inputs.armLeaderAbsoluteEncoderConnected = armLeaderAbsoluteEncoderConnected;
    inputs.armFollowerAbsoluteEncoderConnected = armFollowerAbsoluteEncoderConnected;
    inputs.armAbsoluteEncoderPositionRad = armAbsoluteEncoderPositionRad;
    inputs.armAbsoluteEncoderRawPositionRotations =
        armLeaderAbsoluteEncoderConnected
            ? armLeaderAbsoluteEncoderRawPositionRotations
            : armFollowerAbsoluteEncoderRawPositionRotations;
    inputs.armLeaderAbsoluteEncoderPositionRad = armLeaderAbsoluteEncoderPositionRad;
    inputs.armFollowerAbsoluteEncoderPositionRad = armFollowerAbsoluteEncoderPositionRad;
    inputs.armLeaderAbsoluteEncoderRawPositionRotations =
        armLeaderAbsoluteEncoderRawPositionRotations;
    inputs.armFollowerAbsoluteEncoderRawPositionRotations =
        armFollowerAbsoluteEncoderRawPositionRotations;
    inputs.armAbsoluteEncoderSyncErrorRad =
        armLeaderAbsoluteEncoderConnected && armFollowerAbsoluteEncoderConnected
            ? Math.abs(
                MathUtil.angleModulus(
                    armLeaderAbsoluteEncoderPositionRad - armFollowerAbsoluteEncoderPositionRad))
            : 0.0;
    inputs.armMotorPositionRad = armMotorPositionRad;
    inputs.armPositionRad = armPositionRad;
    inputs.armVelocityRadPerSec = armVelocityRadPerSec;
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
    if (armControlMode != ArmControlMode.POSITION) {
      armController.reset();
    }
    armControlMode = ArmControlMode.POSITION;
    armSetpointRad =
        Math.max(IntakeConstants.minAngleRad, Math.min(IntakeConstants.maxAngleRad, positionRad));
  }

  @Override
  public void setArmCurrentAmps(double amps) {
    armControlMode = ArmControlMode.CURRENT;
    armCurrentSetpointAmps =
        MathUtil.clamp(
            amps,
            -IntakeConstants.armPeakTorqueCurrentAmps,
            IntakeConstants.armPeakTorqueCurrentAmps);
  }

  @Override
  public void setRollerVelocityRpm(double rpm) {
    rollerMotor.setControl(rollerVelocityRequest.withVelocity(rpm / 60.0));
  }

  private void runArmControl(double armPositionRad) {
    switch (armControlMode) {
      case POSITION -> armLeaderMotor.setControl(
          armVoltageRequest.withOutput(
              MathUtil.clamp(
                  armController.calculate(armPositionRad, armSetpointRad), -12.0, 12.0)));
      case CURRENT -> armLeaderMotor.setControl(
          armCurrentRequest.withOutput(armCurrentSetpointAmps));
    }
  }

  private void synchronizeArmMotorPosition(double armPositionRad) {
    if (armMotorPositionSynchronized) {
      return;
    }

    armLeaderMotor.setPosition(armRadToMotorRot(armPositionRad));
    armController.reset();
    armMotorPositionSynchronized = true;
  }

  private static double fuseArmPositionRad(
      boolean leaderConnected,
      double leaderPositionRad,
      boolean followerConnected,
      double followerPositionRad) {
    if (leaderConnected && followerConnected) {
      return MathUtil.inputModulus(
          leaderPositionRad + 0.5 * MathUtil.angleModulus(followerPositionRad - leaderPositionRad),
          0.0,
          2.0 * Math.PI);
    }
    if (leaderConnected) {
      return leaderPositionRad;
    }
    if (followerConnected) {
      return followerPositionRad;
    }
    return IntakeConstants.stowAngleRad;
  }

  private static double fuseArmPositionRad(
      boolean leaderConnected,
      double leaderPositionRad,
      boolean followerConnected,
      double followerPositionRad,
      double fallbackPositionRad) {
    if (leaderConnected || followerConnected) {
      return fuseArmPositionRad(
          leaderConnected, leaderPositionRad, followerConnected, followerPositionRad);
    }
    return fallbackPositionRad;
  }

  private static double armRadToMotorRot(double armRad) {
    return (armRad / (2.0 * Math.PI)) * IntakeConstants.armGearRatio;
  }

  private static double absoluteEncoderRotationsToArmRad(
      double absoluteEncoderRotations, boolean inverted, double offsetRad) {
    double wrappedRotations = MathUtil.inputModulus(absoluteEncoderRotations, 0.0, 1.0);
    double adjustedRotations = inverted ? 1.0 - wrappedRotations : wrappedRotations;
    return MathUtil.inputModulus(
        adjustedRotations * IntakeConstants.armAbsoluteEncoderPositionFactor - offsetRad,
        0.0,
        IntakeConstants.armAbsoluteEncoderPositionFactor);
  }

  private static double motorRotToArmRad(double motorRot) {
    return (motorRot / IntakeConstants.armGearRatio) * 2.0 * Math.PI;
  }

  private static double motorVelocityToArmRadPerSec(double motorVelocityRps) {
    return (motorVelocityRps / IntakeConstants.armGearRatio) * 2.0 * Math.PI;
  }
}
