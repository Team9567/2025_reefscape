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
import frc.robot.Constants.AlgaeConstants;

public class AlgaePickerSubsystem extends SubsystemBase {
    final SparkMax pivotMotor;
    final SparkFlex intakeMotor;
    final AbsoluteEncoder pivotEncoder;
    LaserCan algaeRanger;

    public AlgaePickerSubsystem() {
        // Set up the roller motor as a brushless motor
        pivotMotor = new SparkMax(AlgaeConstants.PIVOT_MOTOR_ID, MotorType.kBrushless);
        intakeMotor = new SparkFlex(AlgaeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
        pivotEncoder = pivotMotor.getAbsoluteEncoder();
        /*
        algaeRanger = new LaserCan(AlgaeConstants.ALGAE_RANGER_ID);
        try {
            algaeRanger.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS);
        }
        
        catch (ConfigurationFailedException e) {
            System.out.println("LaserCan error " + e);
        }
        */
        pivotMotor.setCANTimeout(250);
        intakeMotor.setCANTimeout(250);

        SparkMaxConfig algaeConfig = new SparkMaxConfig();
        algaeConfig.voltageCompensation(AlgaeConstants.ALGAE_MOTOR_VOLTAGE_COMP);
        algaeConfig.smartCurrentLimit(AlgaeConstants.ALGAE_MOTOR_CURRENT_LIMIT);
        algaeConfig.idleMode(IdleMode.kBrake);
        pivotMotor.configure(algaeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // According to Dan, a timing delay between config requests has been observed to correct heisenbugs
        Timer.delay(0.1);
        SparkFlexConfig intakeConfig = new SparkFlexConfig();
        intakeConfig.voltageCompensation(AlgaeConstants.ALGAE_MOTOR_VOLTAGE_COMP);
        intakeConfig.smartCurrentLimit(AlgaeConstants.ALGAE_MOTOR_CURRENT_LIMIT);
        intakeConfig.idleMode(IdleMode.kBrake);
        intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public int distanceToAlgaeInMm() {
        LaserCan.Measurement measurement = algaeRanger.getMeasurement();
        if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            return measurement.distance_mm;
         }
         return -1;
    }

    public void runpivotmotor(double speed){
        pivotMotor.set(speed);
    }
    public void runintakemotor(double speed){
        intakeMotor.set(speed);
    }
    public boolean algaeinrange(){
        return distanceToAlgaeInMm() > AlgaeConstants.SENSOR_LIMIT || distanceToAlgaeInMm() == -1;
    }
    public boolean arminintakeposition(){
        return pivotEncoder.getPosition() < AlgaeConstants.ALGAE_ARM_INTAKE_POSITION || pivotEncoder.getPosition() > AlgaeConstants.ALGAE_ARM_HOME_POSITION;
    }
    public boolean arminhomeposition(){
        return pivotEncoder.getPosition() > AlgaeConstants.ALGAE_ARM_HOME_POSITION && pivotEncoder.getPosition() < AlgaeConstants.ALGAE_ARM_HOME_POSITION + 0.5;
    }
    public double getPivotAngle()
    {
        return pivotEncoder.getPosition();
    }
    public void setBrake(boolean enabled)
    {
        SparkMaxConfig config = new SparkMaxConfig();
        if (enabled)
        {
            config
                .idleMode(IdleMode.kBrake);
        }

        else
        {
            config
                .idleMode(IdleMode.kCoast);
        }
        pivotMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public Command reachForAlgae(
            AlgaePickerSubsystem algaeSubsystem) {
        return Commands.startEnd(
            () -> pivotMotor.set(AlgaeConstants.ALGAE_ARM_REACH_SPEED),
            () -> pivotMotor.set(0),
            algaeSubsystem)
            .onlyWhile(
                () -> (arminintakeposition())
            );

    }

    public Command getAlgae(
            AlgaePickerSubsystem algaeSubsystem) {
        return Commands.startEnd(
            () -> intakeMotor.set(AlgaeConstants.INTAKE_MOTOR_SPEED),
            () -> intakeMotor.set(0),
            algaeSubsystem)
            .onlyWhile(
                () -> (algaeinrange())
            );

    }

    public Command returnArm(
            AlgaePickerSubsystem algaeSubsystem) {
        return Commands.startEnd(
            () -> pivotMotor.set(AlgaeConstants.ALGAE_ARM_RETURN_SPEED),
            () -> pivotMotor.set(0),
            algaeSubsystem)
            .onlyWhile(
                () -> (!arminhomeposition())
            );
    }

    public Command scoreAlgae(
            AlgaePickerSubsystem algaeSubsystem) {
        return Commands.startEnd(
            () -> intakeMotor.set(AlgaeConstants.SHOOT_MOTOR_SPEED),
            () -> intakeMotor.set(0),
            algaeSubsystem)
            .onlyWhile(
                () -> (distanceToAlgaeInMm() > AlgaeConstants.SENSOR_LIMIT || distanceToAlgaeInMm() == -1)
            );
    }

@Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("armAngle", pivotEncoder.getPosition());
    SmartDashboard.putNumber("armSpeed", pivotMotor.get());
  }
}
