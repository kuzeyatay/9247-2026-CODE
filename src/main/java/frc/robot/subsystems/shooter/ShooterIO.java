package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  class ShooterIOInputs {
    public boolean leaderConnected = false;
    public boolean followerConnected = false;
    public double leaderVelocityRps = 0.0;
    public double followerVelocityRps = 0.0;
    public double leaderAppliedVolts = 0.0;
    public double followerAppliedVolts = 0.0;
    public double leaderCurrentAmps = 0.0;
    public double followerCurrentAmps = 0.0;
  }

  default void updateInputs(ShooterIOInputs inputs) {}

  default void setVelocityRpm(double rpm) {}
}
