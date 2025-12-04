package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanDevices;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    SparkMax m_liftMotor = new SparkMax(CanDevices.ElevatorLiftMotor.id, MotorType.kBrushless);
    SparkMax m_liftMotorFollower = new SparkMax(CanDevices.ElevatorLiftMotorFollower.id, MotorType.kBrushless);
    public RelativeEncoder m_liftMotorAEncoder = m_liftMotor.getEncoder();
    public RelativeEncoder m_liftMotorBEncoder = m_liftMotorFollower.getEncoder();
    PIDController m_elevatorController = new PIDController(ElevatorConstants.kPV, ElevatorConstants.kIV,
            ElevatorConstants.kDV);
    ElevatorFeedforward m_elevatorFeedforward = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV, ElevatorConstants.kA);

    public ElevatorSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();
        config
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(80);
        config.softLimit
                .forwardSoftLimitEnabled(false)
                .reverseSoftLimitEnabled(false);
        config.openLoopRampRate(ElevatorConstants.kMotorRampTime);
        config.encoder.positionConversionFactor(ElevatorConstants.kPositionConversionFactor);

        config.inverted(ElevatorConstants.kMotorInverted);
        m_liftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        config.inverted(ElevatorConstants.kMotorFollowerInverted);
        config.follow(m_liftMotor);
        m_liftMotorFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

  public void runElevatorRaw(double speed) {
    m_liftMotor.set(speed);
  }

  public Command runElevatorSpeed(double speed) {
    return Commands.run(
        () -> {
          m_liftMotor.setVoltage(speed);
        }, this);
  }

  public Command runElevatorVelocity(double meters_per_minute) {
    return Commands.run(
        () -> {
          double forward = m_elevatorFeedforward.calculate(meters_per_minute);
          double velocity = m_liftMotor.getEncoder().getVelocity();
          double pidV = m_elevatorController.calculate(velocity, meters_per_minute);
          m_liftMotor.setVoltage(forward + pidV);
        }, this);
  }
}
