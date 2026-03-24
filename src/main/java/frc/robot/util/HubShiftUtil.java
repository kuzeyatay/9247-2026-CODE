// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import java.util.Optional;
import java.util.function.Supplier;

public final class HubShiftUtil {
  public enum ShiftEnum {
    TRANSITION,
    SHIFT1,
    SHIFT2,
    SHIFT3,
    SHIFT4,
    ENDGAME,
    AUTO,
    DISABLED
  }

  public record ShiftInfo(
      ShiftEnum currentShift, double elapsedTime, double remainingTime, boolean active) {}

  private static final ShiftEnum[] teleopShifts = {
    ShiftEnum.TRANSITION,
    ShiftEnum.SHIFT1,
    ShiftEnum.SHIFT2,
    ShiftEnum.SHIFT3,
    ShiftEnum.SHIFT4,
    ShiftEnum.ENDGAME
  };

  private static final Timer shiftTimer = new Timer();
  private static final double[] shiftStartTimes = {0.0, 10.0, 35.0, 60.0, 85.0, 110.0};
  private static final double[] shiftEndTimes = {10.0, 35.0, 60.0, 85.0, 110.0, 140.0};

  private static final double minFuelCountDelay = 1.0;
  private static final double maxFuelCountDelay = 2.0;
  private static final double shiftEndFuelCountExtension = 3.0;
  // Keep shifted timing self-contained until a launch calculator is added to this codebase.
  private static final double minTimeOfFlight = 0.0;
  private static final double maxTimeOfFlight = 0.0;
  private static final double approachingActiveFudge = -(minTimeOfFlight + minFuelCountDelay);
  private static final double endingActiveFudge =
      shiftEndFuelCountExtension - (maxTimeOfFlight + maxFuelCountDelay);

  public static final double autoEndTime = 20.0;
  public static final double teleopDuration = 140.0;
  private static final boolean[] activeSchedule = {true, true, false, true, false, true};
  private static final boolean[] inactiveSchedule = {true, false, true, false, true, true};
  private static final double timeResetThreshold = 3.0;

  private static double shiftTimerOffset = 0.0;
  private static Supplier<Optional<Boolean>> allianceWinOverride = Optional::empty;

  static {
    shiftTimer.start();
  }

  private HubShiftUtil() {}

  public static Optional<Boolean> getAllianceWinOverride() {
    return allianceWinOverride.get();
  }

  public static void setAllianceWinOverride(
      Supplier<Optional<Boolean>> allianceWinOverrideSupplier) {
    allianceWinOverride =
        allianceWinOverrideSupplier != null ? allianceWinOverrideSupplier : Optional::empty;
  }

  public static Alliance getFirstActiveAlliance() {
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

    Optional<Boolean> winOverride = getAllianceWinOverride();
    if (winOverride.isPresent()) {
      return winOverride.get() ? oppositeAlliance(alliance) : alliance;
    }

    String message = DriverStation.getGameSpecificMessage();
    if (!message.isEmpty()) {
      return switch (Character.toUpperCase(message.charAt(0))) {
        case 'R' -> Alliance.Blue;
        case 'B' -> Alliance.Red;
        default -> oppositeAlliance(alliance);
      };
    }

    return oppositeAlliance(alliance);
  }

  /** Starts the hub shift timer at the beginning of auto or teleop. */
  public static void initialize() {
    shiftTimerOffset = 0.0;
    shiftTimer.restart();
  }

  public static ShiftInfo getOfficialShiftInfo() {
    return getShiftInfo(getSchedule(), shiftStartTimes, shiftEndTimes);
  }

  public static ShiftInfo getShiftedShiftInfo() {
    boolean[] shiftSchedule = getSchedule();
    if (shiftSchedule[1]) {
      double[] shiftedShiftStartTimes = {
        0.0,
        10.0,
        35.0 + endingActiveFudge,
        60.0 + approachingActiveFudge,
        85.0 + endingActiveFudge,
        110.0 + approachingActiveFudge
      };
      double[] shiftedShiftEndTimes = {
        10.0,
        35.0 + endingActiveFudge,
        60.0 + approachingActiveFudge,
        85.0 + endingActiveFudge,
        110.0 + approachingActiveFudge,
        140.0
      };
      return getShiftInfo(shiftSchedule, shiftedShiftStartTimes, shiftedShiftEndTimes);
    }

    double[] shiftedShiftStartTimes = {
      0.0,
      10.0 + endingActiveFudge,
      35.0 + approachingActiveFudge,
      60.0 + endingActiveFudge,
      85.0 + approachingActiveFudge,
      110.0
    };
    double[] shiftedShiftEndTimes = {
      10.0 + endingActiveFudge,
      35.0 + approachingActiveFudge,
      60.0 + endingActiveFudge,
      85.0 + approachingActiveFudge,
      110.0,
      140.0
    };
    return getShiftInfo(shiftSchedule, shiftedShiftStartTimes, shiftedShiftEndTimes);
  }

  private static Alliance oppositeAlliance(Alliance alliance) {
    return alliance == Alliance.Blue ? Alliance.Red : Alliance.Blue;
  }

  private static boolean[] getSchedule() {
    Alliance currentAlliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    return getFirstActiveAlliance() == currentAlliance ? activeSchedule : inactiveSchedule;
  }

  private static ShiftInfo getShiftInfo(
      boolean[] currentSchedule, double[] currentShiftStartTimes, double[] currentShiftEndTimes) {
    double timerValue = shiftTimer.get();
    double currentTime = timerValue - shiftTimerOffset;
    double stateTimeElapsed = currentTime;
    double stateTimeRemaining = 0.0;
    boolean active = false;
    ShiftEnum currentShift = ShiftEnum.DISABLED;

    if (DriverStation.isAutonomousEnabled()) {
      stateTimeElapsed = currentTime;
      stateTimeRemaining = Math.max(0.0, autoEndTime - currentTime);
      active = true;
      currentShift = ShiftEnum.AUTO;
    } else if (DriverStation.isTeleopEnabled()) {
      double fieldTeleopTime = teleopDuration - DriverStation.getMatchTime();

      if (DriverStation.isFMSAttached()
          && fieldTeleopTime <= teleopDuration - 5.0
          && Math.abs(fieldTeleopTime - currentTime) >= timeResetThreshold) {
        shiftTimerOffset += currentTime - fieldTeleopTime;
        currentTime = timerValue - shiftTimerOffset;
      }

      int currentShiftIndex = currentShiftStartTimes.length - 1;
      for (int i = 0; i < currentShiftStartTimes.length; i++) {
        if (currentTime >= currentShiftStartTimes[i] && currentTime < currentShiftEndTimes[i]) {
          currentShiftIndex = i;
          break;
        }
      }

      stateTimeElapsed = Math.max(0.0, currentTime - currentShiftStartTimes[currentShiftIndex]);
      stateTimeRemaining = Math.max(0.0, currentShiftEndTimes[currentShiftIndex] - currentTime);

      if (currentShiftIndex > 0
          && currentSchedule[currentShiftIndex] == currentSchedule[currentShiftIndex - 1]) {
        stateTimeElapsed =
            Math.max(0.0, currentTime - currentShiftStartTimes[currentShiftIndex - 1]);
      }

      if (currentShiftIndex < currentShiftEndTimes.length - 1
          && currentSchedule[currentShiftIndex] == currentSchedule[currentShiftIndex + 1]) {
        stateTimeRemaining =
            Math.max(0.0, currentShiftEndTimes[currentShiftIndex + 1] - currentTime);
      }

      active = currentSchedule[currentShiftIndex];
      currentShift = teleopShifts[currentShiftIndex];
    }

    return new ShiftInfo(currentShift, stateTimeElapsed, stateTimeRemaining, active);
  }
}
