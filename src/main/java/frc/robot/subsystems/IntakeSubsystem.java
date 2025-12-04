
package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanDevices;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  SparkMax m_intakeMotor = new SparkMax(CanDevices.IntakeMotor.id, MotorType.kBrushless);
  RelativeEncoder m_intakeMotorEncoder = m_intakeMotor.getEncoder();
  PIDController m_intakeController = new PIDController(IntakeConstants.kPV, IntakeConstants.kIV, IntakeConstants.kDV);
  SimpleMotorFeedforward m_intakeFeedforward = new SimpleMotorFeedforward(0.18, 0.14);

  public IntakeSubsystem() {
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(IntakeConstants.kMotorCurrentLimit)
        .voltageCompensation(IntakeConstants.kMotorVoltageCompensation);
    config.softLimit
        .forwardSoftLimitEnabled(false)
        .reverseSoftLimitEnabled(false);
    config.openLoopRampRate(IntakeConstants.kMotorRampTime);
    config.encoder.positionConversionFactor(IntakeConstants.kPositionConversionFactor);
    config.encoder.velocityConversionFactor(1.0 / 60.0);

    config.inverted(IntakeConstants.kMotorInverted);
    m_intakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void runIntakeRaw(double speed) {
    m_intakeMotor.set(speed);
  }

  public Command runIntakeSpeed(double speed) {
    return Commands.run(
        () -> {
          m_intakeMotor.setVoltage(speed);
        }, this);
  }

  public Command runIntakeVelocity(double RPS) {
    return Commands.run(
        () -> {
          double forward = m_intakeFeedforward.calculate(RPS);
          double velocity = m_intakeMotor.getEncoder().getVelocity();
          double pidV = m_intakeController.calculate(velocity, RPS);
          m_intakeMotor.setVoltage(forward + pidV);
        }, this);
  }
}
