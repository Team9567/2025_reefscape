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
    SparkMax pivotMotor;
    SparkFlex intakeMotor;
    AbsoluteEncoder pivotEncoder;
    LaserCan algaeRanger;

    public AlgaePickerSubsystem() {
        // Set up the pivot and intake motors as brushless motors
        if (AlgaeConstants.kIsEnabled) {

            pivotMotor = new SparkMax(AlgaeConstants.PIVOT_MOTOR_ID, MotorType.kBrushless);
            intakeMotor = new SparkFlex(AlgaeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
            pivotEncoder = pivotMotor.getAbsoluteEncoder();
        
            /*
            * algaeRanger = new LaserCan(AlgaeConstants.ALGAE_RANGER_ID);
             * try {
            * algaeRanger.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS);
             * }
             * 
             * catch (ConfigurationFailedException e) {
             * System.out.println("LaserCan error " + e);
             * }
             */
            pivotMotor.setCANTimeout(250);
            intakeMotor.setCANTimeout(250);

            SparkMaxConfig algaeConfig = new SparkMaxConfig();
            algaeConfig.voltageCompensation(AlgaeConstants.ALGAE_MOTOR_VOLTAGE_COMP);
            algaeConfig.smartCurrentLimit(AlgaeConstants.ALGAE_MOTOR_CURRENT_LIMIT);
            algaeConfig.idleMode(IdleMode.kCoast);
            pivotMotor.configure(algaeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            // According to Dan, a timing delay between config requests has been observed to
            // correct heisenbugs
            Timer.delay(0.1);
            SparkFlexConfig intakeConfig = new SparkFlexConfig();
            intakeConfig.voltageCompensation(AlgaeConstants.ALGAE_MOTOR_VOLTAGE_COMP);
            intakeConfig.smartCurrentLimit(AlgaeConstants.ALGAE_MOTOR_CURRENT_LIMIT);
            intakeConfig.idleMode(IdleMode.kBrake);
            intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }
    }

    public int distanceToAlgaeInMm() {
        if (AlgaeConstants.kIsEnabled) {
            LaserCan.Measurement measurement = algaeRanger.getMeasurement();
            if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
                return measurement.distance_mm;
            }
            return -1;
        }
        return 0;
    }

    public void runpivotmotor(double speed) {
        if (AlgaeConstants.kIsEnabled) {
            pivotMotor.set(speed);
        }
    } 
    

    public void runintakemotor(double speed) {
        if (AlgaeConstants.kIsEnabled) {
            intakeMotor.set(speed);
        }
    }

    public boolean algaeinrange() {
        return distanceToAlgaeInMm() > AlgaeConstants.SENSOR_LIMIT || distanceToAlgaeInMm() == -1;
    }

    public boolean arminintakeposition() {
        if (AlgaeConstants.kIsEnabled) {
            double pivotposition = pivotEncoder.getPosition();
            boolean pivotintakepart1 = pivotposition < AlgaeConstants.ALGAE_ARM_INTAKE_POSITION;

            boolean pivotintakepart2 =  pivotposition > AlgaeConstants.ALGAE_ARM_INTAKE_POSITION + 0.5;
            // SmartDashboard.putNumber("pivotposition",pivotposition );
            // SmartDashboard.putNumber("pivotintakelimit2", AlgaeConstants.ALGAE_ARM_INTAKE_POSITION + 0.5);
            //SmartDashboard.putBoolean("pivotintakepart1", pivotintakepart1);
            //SmartDashboard.putBoolean("pivotintakepart2", pivotintakepart2);
            //SmartDashboard.putBoolean("pivotinintakeposition", pivotintakepart1 || pivotintakepart2);
        
            return pivotintakepart1 || pivotintakepart2;
        }
        return false;
    }

    public boolean arminhomeposition() {
        if (AlgaeConstants.kIsEnabled) {
            double pivotposition = pivotEncoder.getPosition();
            boolean pivothomepart1 = pivotposition > AlgaeConstants.ALGAE_ARM_HOME_POSITION;
            boolean pivothomepart2 = pivotposition < AlgaeConstants.ALGAE_ARM_HOME_POSITION + 0.5;
            //SmartDashboard.putBoolean("pivothomepart1", pivothomepart1);
            //SmartDashboard.putBoolean("pivothomepart2", pivothomepart2);
            //SmartDashboard.putBoolean("pivotinhomeposition",pivothomepart1 && pivothomepart2);        
        
            return pivothomepart1 && pivothomepart2;
        }
        return false;
    }

    public double getPivotAngle() {
        if (AlgaeConstants.kIsEnabled) {
            return pivotEncoder.getPosition();
        }
        return 0;
    }


    public double getWrappedPivotAngle() {
        double position = getPivotAngle();
        /*
        if (position > 0.5) {
            position -= 1.0;
        }
            */
        return position;
    }

    public void setBrake(boolean enabled, boolean sync) {
        if (AlgaeConstants.kIsEnabled) {
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
    }

    public Command reachForAlgae(AlgaePickerSubsystem algaeSubsystem) {
        if (AlgaeConstants.kIsEnabled) {
            return Commands.startEnd(
                    () -> {pivotMotor.set(AlgaeConstants.ALGAE_ARM_REACH_SPEED);
                    //SmartDashboard.putBoolean("reachforAlgae", true);
                    },
                    () -> {pivotMotor.set(0);
                    //SmartDashboard.putBoolean("reachforAlgae", false);
                    },
                    algaeSubsystem)
                    .onlyWhile(
                            () -> (arminintakeposition()));
        }
        return run (()->{});
    }

    public Command holdAlgae(AlgaePickerSubsystem algaeSubsystem, double intakespeed) {
        if (AlgaeConstants.kIsEnabled) {
            return Commands.run(
                    () -> {
                        double wrappedPivotAngle = algaeSubsystem.getWrappedPivotAngle();
                        double extensionAngle = AlgaeConstants.ALGAE_ARM_HOME_POSITION - wrappedPivotAngle;
                        double scalingFactor = extensionAngle / (AlgaeConstants.ALGAE_ARM_HOME_POSITION - AlgaeConstants.ALGAE_ARM_INTAKE_POSITION);
                        //intakeMotor.set(AlgaeConstants.INTAKE_HOLD_MOTOR_SPEED);
                        intakeMotor.set(intakespeed);
                        pivotMotor.set((AlgaeConstants.PIVOT_HOLD_MOTOR_SPEED * scalingFactor) + 0.02);
                        setBrake(false, false);
                    
                    },
                algaeSubsystem);
        }
        return run(()->{});
    }

    public Command returnArm(AlgaePickerSubsystem algaeSubsystem) {
        if (AlgaeConstants.kIsEnabled) {
            return Commands.startEnd(
                    () -> {
                        pivotMotor.set(AlgaeConstants.ALGAE_ARM_RETURN_SPEED);
                        intakeMotor.set(AlgaeConstants.INTAKE_HOLD_MOTOR_SPEED);
                        setBrake(false, false);
                        //SmartDashboard.putBoolean("ReturnArm", true);
                    },
                    () -> {pivotMotor.set(0);
                    //SmartDashboard.putBoolean("ReturnArm", false);
                    },
                    algaeSubsystem)
                    .onlyWhile(
                            () -> (!arminhomeposition()));
        }
        return run(()->{});
    }

    public Command scoreAlgae(AlgaePickerSubsystem algaeSubsystem) {
        if (AlgaeConstants.kIsEnabled) {
            return Commands.startEnd(
                    () -> intakeMotor.set(AlgaeConstants.SHOOT_MOTOR_SPEED),
                    () -> intakeMotor.set(0),
                    algaeSubsystem);
        }
        return run(()->{}); 
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        //SmartDashboard.putNumber("armAngle", pivotEncoder.getPosition());
        //SmartDashboard.putNumber("armSpeed", pivotMotor.get());
        //SmartDashboard.putNumber("intake speed", intakeMotor.get());
        arminhomeposition();
        arminintakeposition();
    }
}
