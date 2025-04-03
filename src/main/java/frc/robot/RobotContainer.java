// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import au.grapplerobotics.CanBridge;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ButtonConstants;
import frc.robot.Constants.ChassisConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.RollerConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.CoralJostlerCommand;
import frc.robot.commands.DriveDistanceCommand;
import frc.robot.commands.ReachAndGrab;
import frc.robot.commands.TurnToAngle;
import frc.robot.subsystems.AlgaePickerSubsystem;
import frc.robot.subsystems.AlgaeBat;
import frc.robot.subsystems.ChassieSubSystem;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CoralRoller;
import au.grapplerobotics.CanBridge;

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
  private AlgaeBat m_algaeBat;
  private Climber m_climber;
  

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
      m_algaeBat = new AlgaeBat();
      m_climber = new Climber();
      // Creates UsbCamera
      CameraServer.startAutomaticCapture();
      // enable lasercan config bridge
      //CanBridge.runTCP();
    }
    // Configure the trigger bindings
    configureBindings();
    autochooser.addOption("middle", Autos.simpleAutoMiddle(m_ChassieSubsystem, m_coralRoller));
    autochooser.addOption("long", Autos.simpleAutoSide(m_ChassieSubsystem, m_coralRoller));
    autochooser.addOption("midplusalgae", Autos.midCoralPlusAlgae(m_ChassieSubsystem, m_coralRoller, m_algaeBat, m_algaePicker));
    autochooser.addOption("longplusalgae", Autos.sideCoralPlusAlgae(m_ChassieSubsystem, m_coralRoller, m_algaeBat, m_algaePicker));
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
      m_controllerController.button(ButtonConstants.kButtonLB).whileTrue(new CoralJostlerCommand(m_coralRoller));
    }

    if (m_algaePicker != null) {
      m_algaePicker.setDefaultCommand(m_algaePicker.holdAlgae(m_algaePicker));
      m_controllerController.button(ButtonConstants.kButtonA).whileTrue(new ReachAndGrab(m_algaePicker));
      m_controllerController.button(ButtonConstants.kButtonB).whileTrue(m_algaePicker.scoreAlgae(m_algaePicker));
    }

    if (m_algaeBat != null) {
      //m_algaeBat.setDefaultCommand(m_AlgaePicker.holdAlgae(m_algaePicker));
      m_controllerController.axisGreaterThan(ButtonConstants.kAxisLT, 0.5).whileTrue(m_algaeBat.extendBat(m_algaeBat));
      m_controllerController.axisGreaterThan(ButtonConstants.kAxisRT, 0.5).whileTrue(m_algaeBat.returnBat(m_algaeBat));
    }

    if(m_climber != null) {
      m_controllerController.button(ButtonConstants.kButtonStart).whileTrue(m_climber.extendClimber(m_climber));
      m_controllerController.button(ButtonConstants.kButtonBack).whileTrue(m_climber.reverseClimber(m_climber));
    }

  
    RunCommand chassisDefault = new RunCommand(
      () -> {
        SmartDashboard.putBoolean("M_inLowGear", m_inLowGear);
        if (m_inLowGear) {
          m_ChassieSubsystem.arcadeDrive(
            m_driverController.getRawAxis(1) * ChassisConstants.kLowGearSpeed,
            m_driverController.getRawAxis(4) * ChassisConstants.kLowGearSpeed
          );
        }
        else {
          m_ChassieSubsystem.arcadeDrive(m_driverController.getRawAxis(1), m_driverController.getRawAxis(4));
        }
      }, m_ChassieSubsystem);
      chassisDefault.setName("chassisDefault");
  
    m_ChassieSubsystem.setDefaultCommand(chassisDefault);

    m_driverController.button(ButtonConstants.kButtonX).whileTrue(new DriveDistanceCommand(24, m_ChassieSubsystem));
    m_driverController.button(ButtonConstants.kButtonB).whileTrue(new TurnToAngle(90, m_ChassieSubsystem));
    m_driverController.button(ButtonConstants.kButtonA).whileTrue(new TurnToAngle(15, m_ChassieSubsystem));

    InstantCommand lowGearEnable = new InstantCommand(
      () -> {
        m_inLowGear = true;
        SmartDashboard.putBoolean("M_inLowGear", m_inLowGear);
      });
    lowGearEnable.setName("lowGearEnable");
    m_driverController.button(ButtonConstants.kButtonRB).onTrue(lowGearEnable);

    InstantCommand lowGearDis = new InstantCommand(
      () -> {
        m_inLowGear = false;
        SmartDashboard.putBoolean("M_inLowGear", m_inLowGear);
      });
    lowGearDis.setName("lowGearDis");
    m_driverController.button(ButtonConstants.kButtonRB).onFalse(lowGearDis);
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
