package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

public class ShooterIOSim implements ShooterIO {
  private double velocitySetpointRps = 0.0;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    double appliedPercent = velocitySetpointRps / simFreeSpeedRps;
    inputs.leaderConnected = true;
    inputs.followerConnected = true;
    inputs.leaderVelocityRps = velocitySetpointRps;
    inputs.followerVelocityRps = velocitySetpointRps * (followerOpposeLeader ? -1.0 : 1.0);
    inputs.leaderAppliedVolts = appliedPercent * 12.0;
    inputs.followerAppliedVolts = appliedPercent * 12.0;
    inputs.leaderCurrentAmps = Math.abs(appliedPercent) * simStallCurrentAmps;
    inputs.followerCurrentAmps = Math.abs(appliedPercent) * simStallCurrentAmps;
  }

  @Override
  public void setVelocityRpm(double rpm) {
    velocitySetpointRps = rpm / 60.0;
  }
}
