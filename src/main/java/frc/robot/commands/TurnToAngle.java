package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.ChassisConstants;
import frc.robot.Constants.MathUtils;
import frc.robot.subsystems.ChassieSubSystem;

/** A command that will turn the robot to the specified angle. */
public class TurnToAngle extends PIDCommand {
  ChassieSubSystem drivetrain;

  /**
   * Turns to robot to the specified angle.
   *
   * @param targetAngleDegrees The angle to turn to
   * @param drive              The drive subsystem to use
   */
  public TurnToAngle(double targetAngleDegrees, ChassieSubSystem drive) {
    super(
        new PIDController (ChassisConstants.kTurnP, ChassisConstants.kTurnI, ChassisConstants.kTurnD),
        // Close loop on heading
        drive::getHeading,
        // Set reference to target
        targetAngleDegrees,
       
        // Pipe output to turn robot
        output -> drive.arcadeDrive(0,MathUtils.Clamp(output, ChassisConstants.kTurnClamp)),
        // Require the drive
        drive);
    drivetrain = drive;
    SmartDashboard.putData("drivetrain/turn PID controller", this.getController());
    // Set the controller to be continuous (because it is an angle controller)
    getController().enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is
    // stationary at the
    // setpoint before it is considered as having reached the reference
    getController()
        .setTolerance(ChassisConstants.kTurnToleranceDeg, ChassisConstants.kTurnRateToleranceDegPerS);

  }

  @Override
  public void initialize() {
  
    drivetrain.disableramp();
    drivetrain.zeroHeading();

    //getController().reset();
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return getController().atSetpoint();

  }

  @Override
  public void end(boolean interupted) {
    drivetrain.enableramp();
  }
}

