package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;
import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

public class AlgaePickerSubsystem extends SubsystemBase {
    final SparkMax algaeMotor;

    public AlgaePickerSubsystem() {
        // Set up the roller motor as a brushless motor
        algaeMotor = new SparkMax(AlgaeConstants.ALGAE_MOTOR_ID, MotorType.kBrushless);

        algaeMotor.setCANTimeout(250);

        SparkMaxConfig algaeConfig = new SparkMaxConfig();
        algaeConfig.voltageCompensation(AlgaeConstants.ALGAE_MOTOR_VOLTAGE_COMP);
        algaeConfig.smartCurrentLimit(AlgaeConstants.ALGAE_MOTOR_CURRENT_LIMIT);
        algaeMotor.configure(algaeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command runAlgaePicker(
            AlgaePickerSubsystem algaeSubsystem, DoubleSupplier forward, DoubleSupplier reverse) {
        return Commands.run(
            () -> algaeMotor.set(forward.getAsDouble() - reverse.getAsDouble()), algaeSubsystem);
    }
}
