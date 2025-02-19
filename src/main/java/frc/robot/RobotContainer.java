// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.RollerConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.ButtonConstants;
import frc.robot.Constants.ChassisConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ReachAndGrab;
import frc.robot.commands.DriveDistanceCommand;
import frc.robot.commands.TurnToAngle;
import frc.robot.subsystems.ChassieSubSystem;
import frc.robot.subsystems.CoralRoller;
import frc.robot.subsystems.AlgaePickerSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ChassieSubSystem m_ChassieSubsystem = new ChassieSubSystem();
  private boolean m_inLowGear = false;
  private CoralRoller m_coralRoller;
  private AlgaePickerSubsystem m_algaePicker;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandJoystick m_driverController =
      new CommandJoystick(OperatorConstants.kDriverControllerPort);


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandJoystick m_controllerController =
      new CommandJoystick(ControllerConstants.kControllerControllerPort);

      SendableChooser<Command> autochooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    if (RobotConstants.k_IsCompBot) {
      m_coralRoller = new CoralRoller();
      m_algaePicker = new AlgaePickerSubsystem();
    }
    // Configure the trigger bindings
    configureBindings();
    autochooser.addOption("left", Autos.simpleAutoLeft(m_ChassieSubsystem, m_coralRoller));
    autochooser.addOption("middle", Autos.simpleAutoMiddle(m_ChassieSubsystem, m_coralRoller));
    autochooser.addOption("right", Autos.simpleAutoRight(m_ChassieSubsystem, m_coralRoller));
    SmartDashboard.putData("AutoPosition", autochooser);

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
    new Trigger(m_ChassieSubsystem::exampleCondition)
        .onTrue(new DriveDistanceCommand(10, m_ChassieSubsystem));
    
    if (m_coralRoller != null) {
      m_coralRoller.setDefaultCommand(m_coralRoller.runRoller(m_coralRoller, ( ) -> 0,() -> 0));
      m_controllerController.button(ButtonConstants.kButtonX).whileTrue(m_coralRoller.runRoller(m_coralRoller, () -> RollerConstants.ROLLER_EJECT_VALUE, () -> RollerConstants.ROLLER_EJECT_VALUE2));
      m_controllerController.button(ButtonConstants.kButtonRB).whileTrue(m_coralRoller.runRoller(m_coralRoller, () -> RollerConstants.ROLLER_REVERSE_VALUE, () -> RollerConstants.ROLLER_REVERSE_VALUE2));
      m_controllerController.button(ButtonConstants.kButtonY).whileTrue(m_coralRoller.runRoller(m_coralRoller, () -> RollerConstants.ROLLER_SLOW_EJECT_VALUE, () -> RollerConstants.ROLLER_EJECT_VALUE2));
    }

    if (m_algaePicker != null) {
      m_controllerController.button(ButtonConstants.kButtonA).whileTrue(new ReachAndGrab(m_algaePicker));
      m_controllerController.button(ButtonConstants.kButtonB).whileTrue(m_algaePicker.returnArm(m_algaePicker));
    }
  
    m_ChassieSubsystem.setDefaultCommand(new RunCommand(
      () -> {
        if (m_inLowGear) {
          m_ChassieSubsystem.arcadeDrive(
            m_driverController.getRawAxis(1) * ChassisConstants.kLowGearSpeed,
            m_driverController.getRawAxis(4) * ChassisConstants.kLowGearSpeed
          );
        }
        else {
          m_ChassieSubsystem.arcadeDrive(m_driverController.getRawAxis(1), m_driverController.getRawAxis(4));
        }
      }, m_ChassieSubsystem));

    m_driverController.button(ButtonConstants.kButtonX).whileTrue(new DriveDistanceCommand(24, m_ChassieSubsystem));
    m_driverController.button(ButtonConstants.kButtonB).whileTrue(new TurnToAngle(90, m_ChassieSubsystem));
    m_driverController.button(ButtonConstants.kButtonA).whileTrue(new TurnToAngle(15, m_ChassieSubsystem));
    m_driverController.button(ButtonConstants.kButtonRB).onTrue(new RunCommand(
      () -> {
        m_inLowGear = true;
      }));
    m_driverController.button(ButtonConstants.kButtonRB).onFalse(new RunCommand(
      () -> {
        m_inLowGear = false;
      }));
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
