package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
  private double setpointRpm = 0.0;

  public Indexer(IndexerIO io) {
    this.io = io;
  }

  public void setVelocityRpm(double rpm) {
    setpointRpm = rpm;
    io.setVelocityRpm(rpm);
  }

  public void stop() {
    setVelocityRpm(0.0);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);
    Logger.recordOutput("Indexer/SetpointRpm", setpointRpm);
  }
}
