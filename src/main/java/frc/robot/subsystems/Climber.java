package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Climber extends SubsystemBase{
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

        // Create and apply configuration for roller motor. Voltage compensation helps
        // the roller behave the same as the battery
        // voltage dips. The current limit helps prevent breaker trips or burning out
        // the motor in the event the roller stalls.
        setMotorConfig(false);
        limitChannel = new DigitalInput(ClimberConstants.CLIMBER_LIMIT_PORT);
    }

    public void setMotorConfig(boolean softLimitEnabled) {
        SparkMaxConfig climberConfig = new SparkMaxConfig();
        climberConfig.voltageCompensation(ClimberConstants.CLIMBER_MOTOR_VOLTAGE_COMP);
        climberConfig.smartCurrentLimit(ClimberConstants.CLIMBER_MOTOR_CURRENT_LIMIT);
        climberConfig.inverted(false);
        climberConfig.softLimit
            .forwardSoftLimitEnabled(softLimitEnabled)
            .reverseSoftLimitEnabled(softLimitEnabled);
        if (softLimitEnabled) {
            climberConfig.softLimit.forwardSoftLimit(ClimberConstants.CLIMBER_MOTOR_UP_LIMIT)
            .reverseSoftLimit(ClimberConstants.CLIMBER_MOTOR_DOWN_LIMIT);
        }
        climberMotor.configure(climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public boolean getLimitSwitch() {
        return limitChannel.get();
    }

    public void initializeClimber() {
        if (initialized == true) {
            return;
        }
        if (getLimitSwitch() == false) {
            climberMotor.set(ClimberConstants.CLIMBER_MOTOR_INITIALIZE_SPEED);
        }
        
        else {
            climberMotor.set(0);
            climberMotor.getEncoder().setPosition(0);
            setMotorConfig(true);
            initialized = true;
            
        }
    }

    @Override
    public void periodic() {
        initializeClimber();
    }

    // Command to run the roller with joystick inputs
    public Command extendClimber(Climber climberSubsystem) {
        
        return Commands.run(
                () -> climberMotor.set(ClimberConstants.CLIMBER_MOTOR_UP_LIMIT), climberSubsystem)
                        .until(
                            () -> climberMotor.get() < .01

                        );

    }
    public Command reverseClimber(Climber climberSubsystem) {
        
        return Commands.run(
                () -> climberMotor.set(ClimberConstants.CLIMBER_MOTOR_DOWN_LIMIT), climberSubsystem)
                        .until(
                            () -> climberMotor.get() > -.01

                        );

    }
}