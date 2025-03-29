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
import frc.robot.Constants.AlgaeConstants;

public class AlgaeBat extends SubsystemBase{
    final SparkMax pivotMotor;
    LaserCan algaeRanger;

    public AlgaeBat() {
        // Set up the pivot motor as a brushless motor
        pivotMotor = new SparkMax(AlgaeBatConstants.PIVOT_MOTOR_ID, MotorType.kBrushless);
        pivotMotor.getEncoder().setPosition(0);
        pivotMotor.setCANTimeout(250);

        SparkMaxConfig algaeBatConfig = new SparkMaxConfig();
        algaeBatConfig.voltageCompensation(AlgaeBatConstants.ALGAE_BAT_MOTOR_VOLTAGE_COMP);
        algaeBatConfig.smartCurrentLimit(AlgaeBatConstants.ALGAE_BAT_MOTOR_CURRENT_LIMIT);
        algaeBatConfig.idleMode(IdleMode.kBrake);
        algaeBatConfig.softLimit.forwardSoftLimit(AlgaeBatConstants.ALGAE_BAT_KNOCK_POSITION);
        algaeBatConfig.softLimit.reverseSoftLimit(AlgaeBatConstants.ALGAE_BAT_HOME_POSITION);
        algaeBatConfig.softLimit.forwardSoftLimitEnabled(true);
        algaeBatConfig.softLimit.reverseSoftLimitEnabled(true);
        pivotMotor.configure(algaeBatConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void runpivotmotor(double speed) {
        pivotMotor.set(speed);
    }

    public boolean batInKnockPosition() {
        double pivotposition = getPivotAngle();
        boolean pivotKnock = pivotposition >= AlgaeBatConstants.ALGAE_BAT_KNOCK_POSITION;
        
        return pivotKnock;
    }

    public boolean batInHomePosition() {
        double pivotposition = getPivotAngle();
        boolean pivothome = pivotposition >= AlgaeBatConstants.ALGAE_BAT_HOME_POSITION && pivotposition < AlgaeBatConstants.ALGAE_BAT_HOME_POSITION + 0.01;        
        
        return pivothome;
    }

    public double getPivotAngle() {
        return pivotMotor.getEncoder().getPosition();
    }

    public void setBrake(boolean enabled, boolean sync) {
        SparkMaxConfig config = new SparkMaxConfig();
        if (enabled) {
            config
                    .idleMode(IdleMode.kBrake);
        }

        else {
            config
                    .idleMode(IdleMode.kCoast);
        }

        if (sync)
        {
            pivotMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        }
        else
        {
            pivotMotor.configureAsync(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        }
    }

    public Command extendBat(
            AlgaeBat algaeBat) {
        return Commands.startEnd(
                () -> {
                double extensionAngle = AlgaeBatConstants.ALGAE_BAT_KNOCK_POSITION - getPivotAngle();
                double scalingFactor = extensionAngle / (AlgaeBatConstants.ALGAE_BAT_KNOCK_POSITION - AlgaeBatConstants.ALGAE_BAT_HOME_POSITION);
                pivotMotor.set((AlgaeBatConstants.ALGAE_BAT_REACH_SPEED * scalingFactor) + 0.02);
                SmartDashboard.putBoolean("extendBat", true);},
                () -> {pivotMotor.set(0);
                SmartDashboard.putBoolean("extendBat", false);},
                algaeBat)
                .onlyWhile(
                        () -> (!batInKnockPosition()));
    }

    public Command returnBat(
            AlgaeBat algaeBat) {
        return Commands.startEnd(
                () -> {
                    double extensionAngle = AlgaeBatConstants.ALGAE_BAT_HOME_POSITION - getPivotAngle();
                    double scalingFactor = extensionAngle / (AlgaeBatConstants.ALGAE_BAT_HOME_POSITION - AlgaeBatConstants.ALGAE_BAT_KNOCK_POSITION);
                    pivotMotor.set((AlgaeBatConstants.ALGAE_BAT_RETURN_SPEED * scalingFactor) + 0.02);
                    SmartDashboard.putBoolean("ReturnBat", true);
                },
                () -> {pivotMotor.set(0);
                SmartDashboard.putBoolean("ReturnBat", false);},
                algaeBat)
                .onlyWhile(
                        () -> (!batInHomePosition()));
    }
}
