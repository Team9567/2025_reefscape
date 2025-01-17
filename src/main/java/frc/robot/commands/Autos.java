// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.commands.DriveDistanceCommand;
import frc.robot.Constants.AutosConstants;
import frc.robot.subsystems.ChassieSubSystem;
import frc.robot.subsystems.CoralRoller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command simpleAutoLeft(ChassieSubSystem subsystem, CoralRoller coralSubsystem) {
    return new DriveDistanceCommand(AutosConstants.k_leftDist1, subsystem)
      .andThen(new TurnToAngle(AutosConstants.k_leftAngle1, subsystem))
      .andThen(new DriveDistanceCommand(AutosConstants.k_leftDist2, subsystem))
      .andThen(coralSubsystem.runRoller(coralSubsystem, () -> AutosConstants.k_rollerForwardSpeed, () -> AutosConstants.k_rollerReverseSpeed));
    
  }

  public static Command simpleAutoMiddle(ChassieSubSystem subsystem, CoralRoller coralSubsystem) {
    return new DriveDistanceCommand(AutosConstants.k_middleDist1, subsystem)
      .andThen(coralSubsystem.runRoller(coralSubsystem, () -> AutosConstants.k_rollerForwardSpeed, () -> AutosConstants.k_rollerReverseSpeed));
  }

  public static Command simpleAutoRight(ChassieSubSystem subsystem, CoralRoller coralSubsystem) {
    return new DriveDistanceCommand(AutosConstants.k_rightDist1, subsystem)
      .andThen(new TurnToAngle(AutosConstants.k_rightAngle1, subsystem))
      .andThen(new DriveDistanceCommand(AutosConstants.k_rightDist2, subsystem))
      .andThen(coralSubsystem.runRoller(coralSubsystem, () -> AutosConstants.k_rollerForwardSpeed, () -> AutosConstants.k_rollerReverseSpeed));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
