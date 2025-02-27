// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        picker.setBrake(true);
    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("arm in intake position", picker.arminintakeposition());
        picker.runintakemotor(AlgaeConstants.INTAKE_MOTOR_SPEED);
        if (picker.arminintakeposition()) {
            picker.runpivotmotor(0);
            picker.setBrake(false);
        }else{
            picker.setBrake(true);
            picker.runpivotmotor(AlgaeConstants.ALGAE_ARM_REACH_SPEED);
        }
    }

    @Override
    public void end(boolean interrupted)
    {
        picker.runpivotmotor(0);
        picker.runintakemotor(0);
    }
}
