package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

public class ShooterIOSim implements ShooterIO {
  private double velocitySetpointMotorRps = 0.0;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    double appliedPercent = velocitySetpointMotorRps / simFreeSpeedRps;
    double shooterRps = velocitySetpointMotorRps / shooterGearRatio;
    inputs.leaderConnected = true;
    inputs.followerConnected = true;
    inputs.leaderVelocityRps = shooterRps;
    inputs.followerVelocityRps = shooterRps * (followerOpposeLeader ? -1.0 : 1.0);
    inputs.leaderAppliedVolts = appliedPercent * 12.0;
    inputs.followerAppliedVolts = appliedPercent * 12.0;
    inputs.leaderCurrentAmps = Math.abs(appliedPercent) * simStallCurrentAmps;
    inputs.followerCurrentAmps = Math.abs(appliedPercent) * simStallCurrentAmps;
  }

  @Override
  public void setVelocityRpm(double rpm) {
    velocitySetpointMotorRps = (rpm / 60.0) * shooterGearRatio;
  }
}
