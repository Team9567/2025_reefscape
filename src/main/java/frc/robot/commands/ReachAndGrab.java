// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.subsystems.AlgaePickerSubsystem;

/** A command that will reach and grab for algae */
public class ReachAndGrab extends Command {
    AlgaePickerSubsystem picker;

    /** 
     * @param subsystem    The picker subsystem to use
     */
    public ReachAndGrab(AlgaePickerSubsystem subsystem) {
        picker = subsystem;
        addRequirements(picker);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        picker.runpivotmotor(AlgaeConstants.ALGAE_ARM_REACH_SPEED);
        picker.runintakemotor(AlgaeConstants.INTAKE_MOTOR_SPEED);
        picker.setBrake(false, false);
        SmartDashboard.putBoolean("reachandgrab", true);
    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("arm in intake position", picker.arminintakeposition());
        picker.runintakemotor(AlgaeConstants.INTAKE_MOTOR_SPEED);
        if (picker.arminintakeposition()) {
            picker.runpivotmotor(AlgaeConstants.ALGAE_ARM_REACH_SPEED2);
            picker.setBrake(false, false);
        }else{
            picker.setBrake(false, false);
            picker.runpivotmotor(AlgaeConstants.ALGAE_ARM_REACH_SPEED);
        }
    }

    @Override
    public void end(boolean interrupted)
    {
        picker.runpivotmotor(0);
        picker.runintakemotor(0);
        SmartDashboard.putBoolean("reachandgrab",false);
    }
   

}

