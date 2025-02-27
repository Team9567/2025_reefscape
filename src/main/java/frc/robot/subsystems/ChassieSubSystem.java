// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.studica.frc.AHRS;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ChassisConstants;

public class ChassieSubSystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  DifferentialDrive m_drivetrain;
  SparkFlex m_leftFront;
  SparkFlex m_rightFront;
  SparkFlex m_leftRear;
  SparkFlex m_rightRear;
  private AHRS m_gyro;

  public ChassieSubSystem() {
    m_gyro = new AHRS(ChassisConstants.kGyroPort);
    m_gyro.reset();
    Timer.delay(0.1);


    m_leftFront = new SparkFlex(ChassisConstants.kLeftFrontCanId, MotorType.kBrushless);
    m_rightFront = new SparkFlex(ChassisConstants.kRightFrontCanId, MotorType.kBrushless);
    m_leftRear = new SparkFlex(ChassisConstants.kLeftRearCanId, MotorType.kBrushless);
    m_rightRear = new SparkFlex(ChassisConstants.kRightRearCanId, MotorType.kBrushless);
    for (SparkFlex motor : new SparkFlex[] {
        m_leftFront, m_rightFront, m_leftRear, m_rightRear }) {
      SparkFlexConfig config = new SparkFlexConfig();
      config
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(80);
      config.softLimit
          .forwardSoftLimitEnabled(false)
          .reverseSoftLimitEnabled(false);
      config.openLoopRampRate(ChassisConstants.kMotorRampTime);
      config.encoder.positionConversionFactor(ChassisConstants.kPositionConversionFactor);
      motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }
    SparkFlexConfig leftrearConfig = new SparkFlexConfig();
    leftrearConfig.follow(m_leftFront);
    leftrearConfig.inverted(false);
    m_leftRear.configure(leftrearConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    SparkFlexConfig RightrearConfig = new SparkFlexConfig();
    RightrearConfig.follow(m_rightFront);
    RightrearConfig.inverted(false);
    m_rightRear.configure(RightrearConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    SparkFlexConfig leftfrontConfig = new SparkFlexConfig();

    leftfrontConfig.inverted(false);

    m_leftFront.configure(leftfrontConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    SparkFlexConfig RightfrontConfig = new SparkFlexConfig();
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
    SmartDashboard.putNumber("power", m_leftFront.get());
    SmartDashboard.putNumber("current", m_leftFront.getOutputCurrent());
    SmartDashboard.putNumber("voltage", m_leftFront.getBusVoltage());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public double getAverageTicks() {
    return (m_leftFront.getEncoder().getPosition() + m_rightFront.getEncoder().getPosition()) / 2;
  }

  public void disableramp() {
    SparkFlexConfig config = new SparkFlexConfig();
    config.closedLoopRampRate(0);
    m_leftFront.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    m_rightFront.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void enableramp() {
    SparkFlexConfig config = new SparkFlexConfig();
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
