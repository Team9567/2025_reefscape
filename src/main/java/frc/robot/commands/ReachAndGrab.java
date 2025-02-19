// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.subsystems.AlgaePickerSubsystem;

/** A command that will turn the robot to the specified angle. */
public class ReachAndGrab extends Command {
    AlgaePickerSubsystem picker;

    /**
     * Turns to robot to the specified angle.
     *
     * @param distance The distance to drive
     * @param drive    The drive subsystem to use
     */
    public ReachAndGrab(AlgaePickerSubsystem drive) {
        picker = drive;

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        picker.runpivotmotor(AlgaeConstants.ALGAE_ARM_REACH_SPEED);
        picker.runintakemotor(AlgaeConstants.INTAKE_MOTOR_SPEED);

    }

    @Override
    public boolean isFinished() {
        boolean pivotMotordone = false;
        boolean intakeMotordone = false;
        if (picker.arminintakeposition()) {
            picker.runpivotmotor(0);
            pivotMotordone = true;
        }
        if (picker.algaeinrange()) {
            picker.runintakemotor(0);
            intakeMotordone = true;
        }

        // End when the controller is at the reference.
        return pivotMotordone && intakeMotordone;

    }
}
