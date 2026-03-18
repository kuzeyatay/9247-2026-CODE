package frc.robot.subsystems.indexer;

import static frc.robot.subsystems.indexer.IndexerConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import java.util.function.DoubleSupplier;

public class IndexerIONeo implements IndexerIO {
  private static final double encoderPositionFactor = 1.0 / indexerGearRatio;
  private static final double encoderVelocityFactor = 1.0 / (60.0 * indexerGearRatio);

  private final SparkMax leaderMotor = new SparkMax(leaderMotorCanId, MotorType.kBrushless);
  private final SparkMax followerMotor = new SparkMax(followerMotorCanId, MotorType.kBrushless);
  private final RelativeEncoder leaderEncoder = leaderMotor.getEncoder();
  private final RelativeEncoder followerEncoder = followerMotor.getEncoder();
  private final SparkClosedLoopController leaderController = leaderMotor.getClosedLoopController();
  private final SparkClosedLoopController followerController =
      followerMotor.getClosedLoopController();
  private final Debouncer connectedDebounce = new Debouncer(0.2);

  public IndexerIONeo() {
    var config = new SparkMaxConfig();
    config
        .inverted(inverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(supplyCurrentLimitAmps)
        .voltageCompensation(12.0);
    config.encoder.positionConversionFactor(encoderPositionFactor);
    config.encoder.velocityConversionFactor(encoderVelocityFactor);
    config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(velocityKp, velocityKi, velocityKd);
    config
        .signals
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    tryUntilOk(
        leaderMotor,
        5,
        () ->
            leaderMotor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(
        followerMotor,
        5,
        () ->
            followerMotor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    sparkStickyFault = false;

    double[] leaderVelocityRps = new double[] {0.0};
    double[] followerVelocityRps = new double[] {0.0};
    double[] leaderAppliedVolts = new double[] {0.0};
    double[] followerAppliedVolts = new double[] {0.0};
    double[] leaderCurrentAmps = new double[] {0.0};
    double[] followerCurrentAmps = new double[] {0.0};

    ifOk(leaderMotor, leaderEncoder::getVelocity, (value) -> leaderVelocityRps[0] = value);
    ifOk(followerMotor, followerEncoder::getVelocity, (value) -> followerVelocityRps[0] = value);
    ifOk(
        leaderMotor,
        new DoubleSupplier[] {leaderMotor::getAppliedOutput, leaderMotor::getBusVoltage},
        (values) -> leaderAppliedVolts[0] = values[0] * values[1]);
    ifOk(
        followerMotor,
        new DoubleSupplier[] {followerMotor::getAppliedOutput, followerMotor::getBusVoltage},
        (values) -> followerAppliedVolts[0] = values[0] * values[1]);
    ifOk(leaderMotor, leaderMotor::getOutputCurrent, (value) -> leaderCurrentAmps[0] = value);
    ifOk(followerMotor, followerMotor::getOutputCurrent, (value) -> followerCurrentAmps[0] = value);

    double followerVelocitySign = followerOpposeLeader ? -1.0 : 1.0;
    inputs.connected = connectedDebounce.calculate(!sparkStickyFault);
    inputs.velocityRps =
        (leaderVelocityRps[0] + followerVelocitySign * followerVelocityRps[0]) / 2.0;
    inputs.appliedVolts = (leaderAppliedVolts[0] + followerAppliedVolts[0]) / 2.0;
    inputs.currentAmps = leaderCurrentAmps[0] + followerCurrentAmps[0];
  }

  @Override
  public void setVelocityRpm(double rpm) {
    double mechanismRps = rpm / 60.0;
    leaderController.setSetpoint(mechanismRps, ControlType.kVelocity);
    followerController.setSetpoint(
        followerOpposeLeader ? -mechanismRps : mechanismRps, ControlType.kVelocity);
  }
}
