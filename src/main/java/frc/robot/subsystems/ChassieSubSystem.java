// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ChassisConstants;

public class ChassieSubSystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  DifferentialDrive m_drivetrain;
  SparkMax m_leftFront;
  SparkMax m_rightFront;
  SparkMax m_leftRear;
  SparkMax m_rightRear;
  private AHRS m_gyro;

  public ChassieSubSystem() {
    m_gyro = new AHRS(ChassisConstants.kGyroPort);
    m_leftFront = new SparkMax(ChassisConstants.kLeftFrontCanId, MotorType.kBrushless);
    m_rightFront = new SparkMax(ChassisConstants.kRightFrontCanId, MotorType.kBrushless);
    m_leftRear = new SparkMax(ChassisConstants.kLeftRearCanId, MotorType.kBrushless);
    m_rightRear = new SparkMax(ChassisConstants.kRightRearCanId, MotorType.kBrushless);
    for (SparkMax motor : new SparkMax[] {
        m_leftFront, m_rightFront, m_leftRear, m_rightRear }) {
      SparkMaxConfig config = new SparkMaxConfig();
      config
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(50);
      config.softLimit
          .forwardSoftLimitEnabled(false)
          .reverseSoftLimitEnabled(false);
      config.closedLoopRampRate(ChassisConstants.kMotorRampTime);
      config.encoder.positionConversionFactor(ChassisConstants.kPositionConversionFactor);
      motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }
    SparkMaxConfig leftrearConfig = new SparkMaxConfig();
    leftrearConfig.follow(m_leftFront);
    leftrearConfig.inverted(false);
    m_leftRear.configure(leftrearConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    SparkMaxConfig RightrearConfig = new SparkMaxConfig();
    RightrearConfig.follow(m_rightFront);
    RightrearConfig.inverted(false);
    m_rightRear.configure(RightrearConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig leftfrontConfig = new SparkMaxConfig();

    leftfrontConfig.inverted(false);

    m_leftFront.configure(leftfrontConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    SparkMaxConfig RightfrontConfig = new SparkMaxConfig();
    RightfrontConfig.inverted(true);

    m_rightFront.configure(RightfrontConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    m_drivetrain = new DifferentialDrive(m_leftFront, m_rightFront);
  }


  public void arcadeDrive(double xSpeed, double zRotation) {
    m_drivetrain.arcadeDrive(xSpeed, zRotation);
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
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
    SmartDashboard.putNumber("drivetrain/encoderticks", getAverageTicks());
    SmartDashboard.putNumber("angle", m_gyro.getAngle());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public double getAverageTicks() {
    return (m_leftFront.getEncoder().getPosition() + m_rightFront.getEncoder().getPosition()) / 2;
  }

  public void disableramp() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoopRampRate(0);
    m_leftFront.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    m_rightFront.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void enableramp() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoopRampRate(ChassisConstants.kMotorRampTime);
    m_leftFront.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    m_rightFront.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

  }

  public void zeroEncoders() {
    m_leftFront.getEncoder().setPosition(0);
    m_rightFront.getEncoder().setPosition(0);
  }

  public double getHeading() {
    return Math.IEEEremainder(m_gyro.getAngle(), 360) * (ChassisConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public void zeroHeading() {
    m_gyro.reset();
  }
}
