package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeBatConstants;

public class AlgaeBat extends SubsystemBase{
    final SparkMax pivotMotor;
    final AbsoluteEncoder pivotEncoder;
    LaserCan algaeRanger;

    public AlgaeBat() {
        // Set up the pivot motor as a brushless motor
        pivotMotor = new SparkMax(AlgaeBatConstants.PIVOT_MOTOR_ID, MotorType.kBrushless);
        pivotEncoder = pivotMotor.getAbsoluteEncoder();

        pivotMotor.setCANTimeout(250);
    }

    public void runpivotmotor(double speed) {
        pivotMotor.set(speed);
    }

    public double getPivotAngle() {
        return pivotEncoder.getPosition();
    }

    public Command extendBat(
            AlgaeBat algaeBat) {
        return Commands.startEnd(
                () -> {pivotMotor.set(AlgaeBatConstants.ALGAE_BAT_REACH_SPEED);
                SmartDashboard.putBoolean("reachforAlgae", true);},
                () -> {pivotMotor.set(0);
                SmartDashboard.putBoolean("reachforAlgae", false);},
                algaeBat)
                .onlyWhile(
                        () -> (arminintakeposition()));
    }

    public Command returnBat(
            AlgaePickerSubsystem algaeSubsystem) {
        return Commands.startEnd(
                () -> {
                    pivotMotor.set(AlgaeBatConstants.ALGAE_BAT_RETURN_SPEED);
                    setBrake(false, false);
                    SmartDashboard.putBoolean("ReturnArm", true);
                },
                () -> {pivotMotor.set(0);
                SmartDashboard.putBoolean("ReturnArm", false);},
                algaeSubsystem)
                .onlyWhile(
                        () -> (!arminhomeposition()));
    }
}
