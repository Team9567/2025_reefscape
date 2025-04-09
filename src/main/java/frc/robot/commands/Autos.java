// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.AutosConstants;
import frc.robot.subsystems.AlgaeBat;
import frc.robot.subsystems.AlgaePickerSubsystem;
import frc.robot.subsystems.ChassieSubSystem;
import frc.robot.subsystems.CoralRoller;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command simpleAutoLeft(ChassieSubSystem subsystem, CoralRoller coralSubsystem) {
    if (coralSubsystem != null) {
      return new DriveDistanceCommand(AutosConstants.k_leftDist1, subsystem)
          .andThen(new TurnToAngle(AutosConstants.k_leftAngle1, subsystem))
          .andThen(new DriveDistanceCommand(AutosConstants.k_leftDist2, subsystem))
          .andThen(coralSubsystem.runRoller(coralSubsystem, () -> AutosConstants.k_rollerForwardSpeed,
              () -> AutosConstants.k_rollerReverseSpeed));
    } else {
      return new DriveDistanceCommand(AutosConstants.k_leftDist1, subsystem)
          .andThen(new TurnToAngle(AutosConstants.k_leftAngle1, subsystem))
          .andThen(new DriveDistanceCommand(AutosConstants.k_leftDist2, subsystem));
    }
  }

  public static Command simpleAutoMiddle(ChassieSubSystem subsystem, CoralRoller coralSubsystem) {
    if (coralSubsystem != null) {
      return new DriveDistanceCommand(AutosConstants.k_middleDist1, subsystem)
          .withTimeout(2.0)
          .andThen(coralSubsystem.runRoller(coralSubsystem, () -> AutosConstants.k_rollerAutoSpeed,
              () -> AutosConstants.k_rollerReverseSpeed).withTimeout(1.0));
    } else {
      return new DriveDistanceCommand(AutosConstants.k_middleDist1, subsystem);
    }
  }

  public static Command simpleAutoSide(ChassieSubSystem subsystem, CoralRoller coralSubsystem) {
    if (coralSubsystem != null) {
      return new DriveDistanceCommand(AutosConstants.k_sideDist1, subsystem)
          .withTimeout(4.0)
          .andThen(coralSubsystem.runRoller(coralSubsystem, () -> AutosConstants.k_rollerForwardSpeed,
              () -> AutosConstants.k_rollerReverseSpeed).withTimeout(1.0));
    } else {
      return new DriveDistanceCommand(AutosConstants.k_sideDist1, subsystem);
    }
  }

  public static Command knockAlgae(ChassieSubSystem subSystem, AlgaeBat batsubSystem, AlgaePickerSubsystem pickerSubsystem) {
    ParallelCommandGroup turnAndRaiseBat = new ParallelCommandGroup(
        new TurnToAngle(AutosConstants.k_algaebatTurn, subSystem).withTimeout(3.0),
        batsubSystem.extendBat(batsubSystem)
        );

    ParallelCommandGroup swingBatAndRun = new ParallelCommandGroup(
        batsubSystem.returnBat(batsubSystem),
        new InstantCommand(() -> {
        }).withTimeout(1).andThen(new DriveDistanceCommand(AutosConstants.k_algaebatDist2, subSystem).withTimeout(1)));

    return new DriveDistanceCommand(AutosConstants.k_algaebatDist1, subSystem).withTimeout(1)
        .andThen(new InstantCommand(() -> {
          SmartDashboard.putString("auto/step", "retreat from reef");
        }))
        .andThen(turnAndRaiseBat.withTimeout(2.0))
        .andThen(new InstantCommand(() -> {
          SmartDashboard.putString("auto/step", "retreat from reef");
        }))
        .andThen(new DriveDistanceCommand(AutosConstants.k_algaebatDist1, subSystem)
            .withTimeout(1))
        .andThen(new InstantCommand(() -> {
          SmartDashboard.putString("auto/step", "approach reef");
        }))

        .andThen(swingBatAndRun);
  }

  public static Command midCoralPlusAlgae(ChassieSubSystem subSystem, CoralRoller coralSubsystem, AlgaeBat batsubSystem, AlgaePickerSubsystem pickerSubsystem) {
    if (coralSubsystem != null) {
      return simpleAutoMiddle(subSystem, coralSubsystem)
          .andThen(knockAlgae(subSystem, batsubSystem, pickerSubsystem).withTimeout(5.25))
          .andThen(new TurnToAngle(20, subSystem))
          .andThen(new ReachAndGrab(pickerSubsystem)
          .withTimeout(4.0));

    } else {
      return new DriveDistanceCommand(AutosConstants.k_sideDist1, subSystem);
    }

  }

  public static Command sideCoralPlusAlgae(ChassieSubSystem subSystem, CoralRoller coralSubsystem, AlgaeBat batsubSystem, AlgaePickerSubsystem pickerSubsystem) {
    if (coralSubsystem != null) {
      return simpleAutoSide(subSystem, coralSubsystem)
          .andThen(knockAlgae(subSystem, batsubSystem, pickerSubsystem));

    } else {
      return new DriveDistanceCommand(AutosConstants.k_sideDist1, subSystem);
    }

  }

  public static Command simpleAutoRight(ChassieSubSystem subsystem, CoralRoller coralSubsystem) {
    if (coralSubsystem != null) {
      return new DriveDistanceCommand(AutosConstants.k_rightDist1, subsystem)
          .andThen(new TurnToAngle(AutosConstants.k_rightAngle1, subsystem))
          .andThen(new DriveDistanceCommand(AutosConstants.k_rightDist2, subsystem))
          .andThen(coralSubsystem.runRoller(coralSubsystem, () -> AutosConstants.k_rollerForwardSpeed,
              () -> AutosConstants.k_rollerReverseSpeed));
    } else {
      return new DriveDistanceCommand(AutosConstants.k_rightDist1, subsystem)
          .andThen(new TurnToAngle(AutosConstants.k_rightAngle1, subsystem))
          .andThen(new DriveDistanceCommand(AutosConstants.k_rightDist2, subsystem));
    }
  }

  public static Command simpleAuto(ChassieSubSystem subsystem, CoralRoller coralSubsystem) {
    if (coralSubsystem != null) {
      return new DriveDistanceCommand(AutosConstants.k_middleDist1, subsystem)
          .withTimeout(4.0);
    } else {
      return new DriveDistanceCommand(AutosConstants.k_middleDist1, subsystem);
    }
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
