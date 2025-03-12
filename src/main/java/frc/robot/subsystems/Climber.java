package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
    private final SparkMax climberMotor;
    private final DigitalInput limitChannel;
    private boolean initialized = false;

    public Climber() {
        // Set up the roller motor as a brushless motor
        climberMotor = new SparkMax(ClimberConstants.CLIMBER_MOTOR_ID, MotorType.kBrushless);

        // Set can timeout. Because this project only sets parameters once on
        // construction, the timeout can be long without blocking robot operation. Code
        // which sets or gets parameters during operation may need a shorter timeout.
        climberMotor.setCANTimeout(250);

        resetClimber();
        limitChannel = new DigitalInput(ClimberConstants.CLIMBER_LIMIT_PORT);
    }

    public void setMotorConfig(boolean softLimitEnabled, int currentLimit, PersistMode persistMode) {
        SparkMaxConfig climberConfig = new SparkMaxConfig();
        climberConfig.voltageCompensation(ClimberConstants.CLIMBER_MOTOR_VOLTAGE_COMP);
        climberConfig.smartCurrentLimit(currentLimit);
        climberConfig.inverted(false);
        SmartDashboard.putBoolean("climber/limitenabled", softLimitEnabled);
        climberConfig.softLimit
                .forwardSoftLimitEnabled(softLimitEnabled)
                .reverseSoftLimitEnabled(softLimitEnabled);
        // if (softLimitEnabled) {
        SmartDashboard.putNumber("climber/upLimit", ClimberConstants.CLIMBER_MOTOR_UP_LIMIT);
        SmartDashboard.putNumber("climber/downLimit", ClimberConstants.CLIMBER_MOTOR_DOWN_LIMIT);
        climberConfig.softLimit.forwardSoftLimit(ClimberConstants.CLIMBER_MOTOR_UP_LIMIT)
                .reverseSoftLimit(ClimberConstants.CLIMBER_MOTOR_DOWN_LIMIT);
        // }
        climberMotor.configure(climberConfig, ResetMode.kResetSafeParameters, persistMode);
    }

    public boolean getLimitSwitch() {
        return !limitChannel.get();
    }

    public void resetClimber()
    {
        // Create and apply configuration for roller motor. Voltage compensation helps
        // the roller behave the same as the battery
        // voltage dips. The current limit helps prevent breaker trips or burning out
        // the motor in the event the roller stalls.
        initialized = false;
        setMotorConfig(false, ClimberConstants.CLIMBER_MOTOR_HOMING_CURRENT_LIMIT, PersistMode.kPersistParameters);
    }

    public void initializeClimber() {
        SmartDashboard.putBoolean("climber/limitswitch", getLimitSwitch());
        SmartDashboard.putBoolean("climber/initialized", initialized);
        SmartDashboard.putNumber("climber/speed", climberMotor.get());
        SmartDashboard.putNumber("climber/encoder", climberMotor.getEncoder().getPosition());
        SmartDashboard.putBoolean("climber/safetoclimb", climberMotor.getEncoder().getPosition() > ClimberConstants.CLIMBER_MOTOR_SAFELIMIT);
        if (initialized == true) {
            return;
        }
        if (getLimitSwitch() == false) {
            climberMotor.set(ClimberConstants.CLIMBER_MOTOR_INITIALIZE_SPEED);

        } else {
            climberMotor.set(0);
            climberMotor.getEncoder().setPosition(0);
            setMotorConfig(true, ClimberConstants.CLIMBER_MOTOR_CLIMBING_CURRENT_LIMIT, PersistMode.kNoPersistParameters);
            initialized = true;
            SmartDashboard.putData(doResetClimber(this));
        }
    }

    @Override
    public void periodic() {
        initializeClimber();
    }

    // Command to run the roller with joystick inputs
    public Command extendClimber(Climber climberSubsystem) {

        return Commands.runEnd(
                () -> climberMotor.set(ClimberConstants.CLIMBER_MOTOR_UP_SPEED),
                () -> climberMotor.set(0),
                climberSubsystem)
                .until(
                        () -> climberMotor.get() < .01

                );
    }

    public Command stopClimber(Climber climberSubsystem) {

        return Commands.run(
                () -> climberMotor.set(0), climberSubsystem);
    }

    public Command reverseClimber(Climber climberSubsystem) {

        return Commands.runEnd(
                () -> climberMotor.set(ClimberConstants.CLIMBER_MOTOR_DOWN_SPEED),
                () -> climberMotor.set(0),
                climberSubsystem)
                .until(
                        () -> climberMotor.get() > -.01

                );

    }

    public Command doResetClimber(Climber climberSubsystem) {
        return Commands.run(
                () -> {
                    resetClimber();
                }, climberSubsystem);
    }
}