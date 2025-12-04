// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ButtonConstants;
import frc.robot.Constants.ChassisConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ChassisSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ChassisSubsystem m_chassisSubsystem = new ChassisSubsystem();
  private boolean m_inLowGear = false;

  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandJoystick m_driverController =
      new CommandJoystick(OperatorConstants.kDriverControllerPort);


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandJoystick m_controllerController =
      new CommandJoystick(ControllerConstants.kControllerControllerPort);

      SendableChooser<Command> autochooser = new SendableChooser<>();
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    RunCommand chassisDefault = new RunCommand(
      () -> {
        SmartDashboard.putBoolean("M_inLowGear", m_inLowGear);
        if (m_inLowGear) {
          m_chassisSubsystem.arcadeDrive(
            m_driverController.getRawAxis(1) * ChassisConstants.kLowGearSpeed,
            m_driverController.getRawAxis(4) * ChassisConstants.kLowGearSpeed
          );
        }
        else {
          m_chassisSubsystem.arcadeDrive(m_driverController.getRawAxis(1), m_driverController.getRawAxis(4));
        }
      }, m_chassisSubsystem);
  
    m_chassisSubsystem.setDefaultCommand(chassisDefault);

    InstantCommand lowGearEnable = new InstantCommand(
      () -> {
        m_inLowGear = true;
        SmartDashboard.putBoolean("M_inLowGear", m_inLowGear);
      });
    m_driverController.button(ButtonConstants.kButtonRB).onTrue(lowGearEnable);

    InstantCommand lowGearDis = new InstantCommand(
      () -> {
        m_inLowGear = false;
        SmartDashboard.putBoolean("M_inLowGear", m_inLowGear);
      });
    m_driverController.button(ButtonConstants.kButtonRB).onFalse(lowGearDis);

    m_intakeSubsystem.setDefaultCommand(m_elevatorSubsystem.runElevatorSpeed(m_controllerController.getRawAxis(1)));
    m_controllerController.button(ButtonConstants.kButtonLB).whileTrue(m_intakeSubsystem.runIntakeSpeed(0.2));
    m_controllerController.button(ButtonConstants.kButtonRB).whileTrue(m_intakeSubsystem.runIntakeSpeed(-0.2));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autochooser.getSelected();
  }
}
