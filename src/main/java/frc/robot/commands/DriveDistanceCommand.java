// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.ChassisConstants;
import frc.robot.Constants.MathUtils;
import frc.robot.subsystems.ChassieSubSystem;

/** A command that will turn the robot to the specified angle. */
public class DriveDistanceCommand extends PIDCommand {
  ChassieSubSystem drivetrain;
  

  /**
   * Turns to robot to the specified angle.
   *
   * @param distance The distance to drive
   * @param drive    The drive subsystem to use
   */
  public DriveDistanceCommand(double distance, ChassieSubSystem drive) {
    super(
        new PIDController(ChassisConstants.kDriveP, ChassisConstants.kDriveI, ChassisConstants.kDriveD),
        // Close loop on heading
        drive::getAverageTicks,
        // Set reference to target
        distance,
        // Pipe output to turn robot
        output -> drive.arcadeDrive(MathUtils.Clamp(output, ChassisConstants.kDriveClamp),0),
        // Require the drive
        drive);
       
    drivetrain = drive;

    SmartDashboard.putData("drivetrain/distance PID controller", this.getController());


    // Set the controller to be continuous (because it is an angle controller)
    // getController().enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is
    // stationary at the
    // setpoint before it is considered as having reached the reference
    getController()
        .setTolerance(ChassisConstants.kDriveToleranceInches, ChassisConstants.kDriveToleranceInchesPerS);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.disableramp();
    drivetrain.zeroEncoders();
        SmartDashboard.putNumber("drivetrain/encoderticks", drivetrain.getAverageTicks());
    
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    SmartDashboard.putNumber("drivetrain/encoderticks", drivetrain.getAverageTicks());
    return getController().atSetpoint();

  }

  @Override
  public void end(boolean interupted) {
    drivetrain.enableramp();
  }
}
