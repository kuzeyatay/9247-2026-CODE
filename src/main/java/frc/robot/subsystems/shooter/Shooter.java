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

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
    Logger.recordOutput("Shooter/SetpointRpm", setpointRpm);
  }
}
