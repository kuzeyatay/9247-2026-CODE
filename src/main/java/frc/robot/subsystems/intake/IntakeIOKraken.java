package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;

public class IntakeIOKraken implements IntakeIO {
  private final TalonFX armLeaderMotor =
      new TalonFX(IntakeConstants.armLeaderMotorCanId, IntakeConstants.canBusName);
  private final TalonFX armFollowerMotor =
      new TalonFX(IntakeConstants.armFollowerMotorCanId, IntakeConstants.canBusName);
  private final TalonFX rollerMotor =
      new TalonFX(IntakeConstants.rollerMotorCanId, IntakeConstants.canBusName);
  private final DutyCycleEncoder armAbsoluteEncoder =
      new DutyCycleEncoder(IntakeConstants.armLeaderAbsoluteEncoderDioChannel);
  private final VoltageOut armVoltageRequest = new VoltageOut(0.0);
  private final VoltageOut armFollowerVoltageRequest = new VoltageOut(0.0);
  private final VelocityVoltage rollerVelocityRequest = new VelocityVoltage(0.0);
  private final Follower armFollowerRequest =
      new Follower(
          IntakeConstants.armLeaderMotorCanId,
          IntakeConstants.armFollowerOpposeLeader
              ? MotorAlignmentValue.Opposed
              : MotorAlignmentValue.Aligned);
  private final ProfiledPIDController armController =
      new ProfiledPIDController(
          IntakeConstants.armKp,
          IntakeConstants.armKi,
          IntakeConstants.armKd,
          new TrapezoidProfile.Constraints(
              IntakeConstants.armProfileCruiseRadPerSec,
              IntakeConstants.armProfileAccelRadPerSecSq));
  private final ArmFeedforward armFeedforward =
      new ArmFeedforward(
          IntakeConstants.armKsVolts,
          IntakeConstants.armKgVolts,
          IntakeConstants.armKvVoltsPerRadPerSec);
  private final Debouncer armAbsoluteEncoderConnectedDebounce = new Debouncer(0.25);

  private double armSetpointRad = IntakeConstants.stowAngleRad;
  private double armVoltageSetpointVolts = 0.0;
  private boolean armVoltageControlEnabled = false;
  private boolean armFollowerEnabled = false;
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
    enableArmFollower();
    armLeaderMotor.setPosition(armRadToMotorRot(IntakeConstants.stowAngleRad));
    armController.reset(IntakeConstants.stowAngleRad);

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
    double armAbsoluteEncoderRawPositionRotations = armAbsoluteEncoder.get();
    double armAbsoluteEncoderPositionRad =
        absoluteEncoderRotationsToArmRad(
            armAbsoluteEncoderRawPositionRotations,
            IntakeConstants.armLeaderAbsoluteEncoderInverted,
            IntakeConstants.armLeaderAbsoluteEncoderOffsetRad);
    boolean armAbsoluteEncoderPresent = armAbsoluteEncoder.isConnected();
    boolean armAbsoluteEncoderConnected =
        armAbsoluteEncoderConnectedDebounce.calculate(armAbsoluteEncoderPresent);

    if (armAbsoluteEncoderConnected) {
      synchronizeArmMotorPosition(armAbsoluteEncoderPositionRad);
    } else {
      armMotorPositionSynchronized = false;
    }

    double armPositionRad =
        armAbsoluteEncoderConnected ? armAbsoluteEncoderPositionRad : armMotorPositionRad;
    double armVelocityRadPerSec = motorVelocityToArmRadPerSec(armMotorVelocityRps);
    double timestampSec = Timer.getFPGATimestamp();
    double dtSec = timestampSec - lastTimestampSec;
    if (armAbsoluteEncoderConnected && dtSec > 1e-6) {
      armVelocityRadPerSec = MathUtil.angleModulus(armPositionRad - lastArmPositionRad) / dtSec;
    }

    runArmControl(armAbsoluteEncoderConnected, armPositionRad);

    lastArmPositionRad = armPositionRad;
    lastTimestampSec = timestampSec;

    inputs.connected = armAbsoluteEncoderConnected;
    inputs.armAbsoluteEncoderConnected = armAbsoluteEncoderConnected;
    inputs.armLeaderAbsoluteEncoderConnected = armAbsoluteEncoderConnected;
    inputs.armAbsoluteEncoderPositionRad = armAbsoluteEncoderPositionRad;
    inputs.armAbsoluteEncoderRawPositionRotations = armAbsoluteEncoderRawPositionRotations;
    inputs.armLeaderAbsoluteEncoderPositionRad = armAbsoluteEncoderPositionRad;
    inputs.armLeaderAbsoluteEncoderRawPositionRotations = armAbsoluteEncoderRawPositionRotations;
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
    armSetpointRad =
        Math.max(IntakeConstants.minAngleRad, Math.min(IntakeConstants.maxAngleRad, positionRad));
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
    rollerMotor.setControl(rollerVelocityRequest.withVelocity(rpm / 60.0));
  }

  private void runArmControl(boolean armAbsoluteEncoderConnected, double armPositionRad) {
    if (!armAbsoluteEncoderConnected) {
      enableArmFollower();
      armController.reset(armPositionRad);
      armLeaderMotor.setControl(armVoltageRequest.withOutput(0.0));
      return;
    }

    if (armVoltageControlEnabled) {
      armController.reset(armPositionRad);
      armLeaderMotor.setControl(armVoltageRequest.withOutput(armVoltageSetpointVolts));
      armFollowerMotor.setControl(
          armFollowerVoltageRequest.withOutput(
              IntakeConstants.armFollowerOpposeLeader
                  ? -armVoltageSetpointVolts
                  : armVoltageSetpointVolts));
      armFollowerEnabled = false;
      return;
    }

    enableArmFollower();
    double feedbackVolts = armController.calculate(armPositionRad, armSetpointRad);
    TrapezoidProfile.State profileSetpoint = armController.getSetpoint();
    double feedforwardVolts =
        armFeedforward.calculate(
            sensorFrameRadToHorizontalRad(profileSetpoint.position), profileSetpoint.velocity);
    armLeaderMotor.setControl(
        armVoltageRequest.withOutput(
            MathUtil.clamp(feedbackVolts + feedforwardVolts, -12.0, 12.0)));
  }

  private void enableArmFollower() {
    if (armFollowerEnabled) {
      return;
    }

    armFollowerMotor.setControl(armFollowerRequest);
    armFollowerEnabled = true;
  }

  private void synchronizeArmMotorPosition(double armPositionRad) {
    if (armMotorPositionSynchronized) {
      return;
    }

    armLeaderMotor.setPosition(armRadToMotorRot(armPositionRad));
    armController.reset(armPositionRad);
    armMotorPositionSynchronized = true;
  }

  private static double sensorFrameRadToHorizontalRad(double armSensorFrameRad) {
    return armSensorFrameRad - IntakeConstants.armHorizontalReferenceRad;
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
