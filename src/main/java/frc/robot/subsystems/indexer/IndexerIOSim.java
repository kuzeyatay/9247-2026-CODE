package frc.robot.subsystems.indexer;

import static frc.robot.subsystems.indexer.IndexerConstants.*;

public class IndexerIOSim implements IndexerIO {
  private double velocitySetpointRps = 0.0;

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    double appliedPercent = velocitySetpointRps / simFreeSpeedRps;
    inputs.connected = true;
    inputs.velocityRps = velocitySetpointRps;
    inputs.appliedVolts = appliedPercent * 12.0;
    inputs.currentAmps = Math.abs(appliedPercent) * simStallCurrentAmps;
  }

  @Override
  public void setVelocityRpm(double rpm) {
    velocitySetpointRps = rpm / 60.0;
  }
}
