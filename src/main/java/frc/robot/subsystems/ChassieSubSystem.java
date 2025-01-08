// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.studica.frc.AHRS;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
public class ChassieSubSystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  DifferentialDrive m_drivetrain;
  public ChassieSubSystem() {
    SparkMax leftFront = new SparkMax(ChassisConstants.kLeftFrontCanId, MotorType.kBrushless);
    SparkMax rightFront = new SparkMax(ChassisConstants.kRightFrontCanId, MotorType.kBrushless);
    SparkMax leftRear = new SparkMax(ChassisConstants.kLeftRearCanId, MotorType.kBrushless);
    SparkMax rightRear = new SparkMax(ChassisConstants.kRightRearCanId, MotorType.kBrushless);
    for(SparkMax motor: new SparkMax[]{
      leftFront,rightFront,leftRear,rightRear}){
        SparkMaxConfig config = new SparkMaxConfig();
        config
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(60);
        config.softLimit
          .forwardSoftLimitEnabled(false)
          .reverseSoftLimitEnabled(false);
        config.closedLoopRampRate(0);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
       
    }
    SparkMaxConfig leftrearConfig = new SparkMaxConfig();
    leftrearConfig.follow(leftFront);
    leftrearConfig.inverted(false);
    leftRear.configure(leftrearConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    SparkMaxConfig RightrearConfig = new SparkMaxConfig();
    RightrearConfig.follow(rightFront);
    RightrearConfig.inverted(false);
    rightRear.configure(RightrearConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    
    SparkMaxConfig leftfrontConfig = new SparkMaxConfig();
    
    leftfrontConfig.inverted(false);
    leftFront.configure(leftfrontConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    SparkMaxConfig RightfrontConfig = new SparkMaxConfig();
    RightfrontConfig.inverted(true);
    rightFront.configure(RightfrontConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  
    
    m_drivetrain = new DifferentialDrive(leftFront, rightFront);
  }

  

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
