package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  class IntakeIOInputs {
    public boolean connected = false;
    public double armPositionRad = 0.0;
    public double armVelocityRadPerSec = 0.0;
    public double rollerVelocityRps = 0.0;
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double velocityRps = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  default void updateInputs(IntakeIOInputs inputs) {}

  default void setPositionRad(double positionRad) {}

  default void setRollerVelocityRpm(double rpm) {}
}
