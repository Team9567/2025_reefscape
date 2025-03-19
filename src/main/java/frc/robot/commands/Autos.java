// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.commands.DriveDistanceCommand;
import frc.robot.Constants.AutosConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.AlgaePickerSubsystem;
import frc.robot.subsystems.ChassieSubSystem;
import frc.robot.subsystems.CoralRoller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command simpleAutoLeft(ChassieSubSystem subsystem, CoralRoller coralSubsystem) {
    if (coralSubsystem != null) {
      return new DriveDistanceCommand(AutosConstants.k_leftDist1, subsystem)
        .andThen(new TurnToAngle(AutosConstants.k_leftAngle1, subsystem))
        .andThen(new DriveDistanceCommand(AutosConstants.k_leftDist2, subsystem))
        .andThen(coralSubsystem.runRoller(coralSubsystem, () -> AutosConstants.k_rollerForwardSpeed, () -> AutosConstants.k_rollerReverseSpeed));
    }
    else {
      return new DriveDistanceCommand(AutosConstants.k_leftDist1, subsystem)
        .andThen(new TurnToAngle(AutosConstants.k_leftAngle1, subsystem))
        .andThen(new DriveDistanceCommand(AutosConstants.k_leftDist2, subsystem));
    }
  }

  public static Command simpleAutoMiddle(ChassieSubSystem subsystem, CoralRoller coralSubsystem) {
    if (coralSubsystem != null) {
      return new DriveDistanceCommand(AutosConstants.k_middleDist1, subsystem)
      .withTimeout(4.0)
        .andThen(coralSubsystem.runRoller(coralSubsystem, () -> AutosConstants.k_rollerForwardSpeed, () -> AutosConstants.k_rollerReverseSpeed));
    }
    else {
      return new DriveDistanceCommand(AutosConstants.k_middleDist1, subsystem);
    }
  }

  public static Command simpleAutoSide(ChassieSubSystem subsystem, CoralRoller coralSubsystem) {
    if (coralSubsystem != null) {
      return new DriveDistanceCommand(AutosConstants.k_sideDist1, subsystem)
      .withTimeout(4.0)
        .andThen(coralSubsystem.runRoller(coralSubsystem, () -> AutosConstants.k_rollerForwardSpeed, () -> AutosConstants.k_rollerReverseSpeed).withTimeout(1.0));
    }
    else {
      return new DriveDistanceCommand(AutosConstants.k_sideDist1, subsystem);
    }
  }

  public static Command sidePlusCoral(ChassieSubSystem subsystem, CoralRoller coralSubsystem) {
    if (coralSubsystem != null) {
      return simpleAutoSide(subsystem, coralSubsystem)
        .andThen(new DriveDistanceCommand(AutosConstants.k_fromReefDist1, subsystem))
        .andThen(new TurnToAngle(AutosConstants.k_fromReefAngle1, subsystem))
        .andThen(new DriveDistanceCommand(AutosConstants.k_toSourceDist1, subsystem))
        .andThen(new TurnToAngle(AutosConstants.k_toSourceAngle1, subsystem))
        .andThen(new DriveDistanceCommand(AutosConstants.k_toSourceDist2, subsystem))
        // TO-DO - wait for coral
        .andThen(new DriveDistanceCommand(AutosConstants.k_fromSourceDist1, subsystem))
        .andThen(new TurnToAngle(AutosConstants.k_fromSourceAngle1, subsystem))
        .andThen(new DriveDistanceCommand(AutosConstants.k_toReefDist1, subsystem))
        .andThen(new TurnToAngle(AutosConstants.k_toReefAngle1, subsystem))
        .andThen(new DriveDistanceCommand(AutosConstants.k_toReefDist2, subsystem))
        .andThen(coralSubsystem.runRoller(coralSubsystem, () -> AutosConstants.k_rollerForwardSpeed, () -> AutosConstants.k_rollerReverseSpeed));
    }
    else {
      return new DriveDistanceCommand(AutosConstants.k_sideDist1, subsystem);
    }
  }

  public static Command simpleAutoRight(ChassieSubSystem subsystem, CoralRoller coralSubsystem) {
    if (coralSubsystem != null) {
      return new DriveDistanceCommand(AutosConstants.k_rightDist1, subsystem)
        .andThen(new TurnToAngle(AutosConstants.k_rightAngle1, subsystem))
        .andThen(new DriveDistanceCommand(AutosConstants.k_rightDist2, subsystem))
        .andThen(coralSubsystem.runRoller(coralSubsystem, () -> AutosConstants.k_rollerForwardSpeed, () -> AutosConstants.k_rollerReverseSpeed));
    }
    else {
      return new DriveDistanceCommand(AutosConstants.k_rightDist1, subsystem)
        .andThen(new TurnToAngle(AutosConstants.k_rightAngle1, subsystem))
        .andThen(new DriveDistanceCommand(AutosConstants.k_rightDist2, subsystem));
    }
  }

  public static Command simpleAuto(ChassieSubSystem subsystem, CoralRoller coralSubsystem) {
    if (coralSubsystem != null) {
      return new DriveDistanceCommand(AutosConstants.k_middleDist1, subsystem)
      .withTimeout(4.0);
    }
    else {
      return new DriveDistanceCommand(AutosConstants.k_middleDist1, subsystem);
    }
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
