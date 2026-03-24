package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private double setpointRpm = 0.0;

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  public void setVelocityRpm(double rpm) {
    setpointRpm = rpm;
    io.setVelocityRpm(rpm);
  }

  public void stop() {
    setVelocityRpm(0.0);
  }

  public double getSetpointRpm() {
    return setpointRpm;
  }

  public double getVelocityRpm() {
    double totalVelocityRps = 0.0;
    int sampleCount = 0;

    if (inputs.leaderConnected) {
      totalVelocityRps += Math.abs(inputs.leaderVelocityRps);
      sampleCount++;
    }
    if (inputs.followerConnected) {
      totalVelocityRps += Math.abs(inputs.followerVelocityRps);
      sampleCount++;
    }
    if (sampleCount == 0) {
      totalVelocityRps = Math.abs(inputs.leaderVelocityRps);
      sampleCount = 1;
    }

    return (totalVelocityRps / sampleCount) * 60.0;
  }

  public boolean isAtSetpoint() {
    return setpointRpm > 0.0
        && Math.abs(getVelocityRpm() - setpointRpm) <= ShooterConstants.smartShootReadyToleranceRpm;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
    Logger.recordOutput("Shooter/SetpointRpm", setpointRpm);
    Logger.recordOutput("Shooter/MeasuredVelocityRpm", getVelocityRpm());
    Logger.recordOutput("Shooter/AtSetpoint", isAtSetpoint());
  }
}
