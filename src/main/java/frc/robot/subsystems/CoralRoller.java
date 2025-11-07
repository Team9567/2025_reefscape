// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RollerConstants;

/** Class to run the rollers over CAN */
public class CoralRoller extends SubsystemBase {
  private SparkMax rollerMotor;
  PIDController rollerController = new PIDController(RollerConstants.ROLLER_VOLTAGE_P, 0, 0);
  SimpleMotorFeedforward rollerFeedforward = new SimpleMotorFeedforward(0.18, 0.14);
  LaserCan horizontalRange;
  LaserCan verticalRange;

  public CoralRoller() {
    if(RollerConstants.k_isEnabled){
      // Set up the roller motor as a brushless motor
      rollerMotor = new SparkMax(RollerConstants.ROLLER_MOTOR_ID, MotorType.kBrushless);
      
      // Create and apply configuration for roller motor. Voltage compensation helps
      // the roller behave the same as the battery
      // voltage dips. The current limit helps prevent breaker trips or burning out
      // the motor in the event the roller stalls.
      SparkMaxConfig config = new SparkMaxConfig();
      config
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(RollerConstants.ROLLER_MOTOR_CURRENT_LIMIT)
          .voltageCompensation(RollerConstants.ROLLER_MOTOR_VOLTAGE_COMP)
          .inverted(true);
      config.softLimit
          .forwardSoftLimitEnabled(false)
          .reverseSoftLimitEnabled(false);
      config.encoder.positionConversionFactor(RollerConstants.ROLLER_GEAR_RATIO);
      config.encoder.velocityConversionFactor(1.0/60.0);
      rollerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      // Set can timeout. Because this project only sets parameters once on
      // construction, the timeout can be long without blocking robot operation. Code
      // which sets or gets parameters during operation may need a shorter timeout.
      rollerMotor.setCANTimeout(250);

      horizontalRange = new LaserCan(RollerConstants.HORIZONTAL_RANGE_ID);
      try {
        horizontalRange.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS);
      }

      catch (ConfigurationFailedException e) {
        System.out.println("LaserCan error " + e);
      }

      verticalRange = new LaserCan(RollerConstants.VERTICAL_RANGE_ID);
      try {
        horizontalRange.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS);
      }

      catch (ConfigurationFailedException e) {
        System.out.println("LaserCan error " + e);
      }
      SmartDashboard.putData("roller/vpid", rollerController);
      //SmartDashboard.putData("roller/vff", rollerFeedforward);
    }
  }

  public void runRollerRaw(double speed) {
    if(RollerConstants.k_isEnabled){
      rollerMotor.set(speed);
    }
  }

  public Command runRollerVelocity(double RPS) {
    if(RollerConstants.k_isEnabled){
      return Commands.run(
        () ->  {
          double forward = rollerFeedforward.calculate(RPS);
          double pidV = rollerController.calculate(rollerMotor.getEncoder().getVelocity(), RPS);
          double velocity = rollerMotor.getEncoder().getVelocity();
          SmartDashboard.putNumber("roller/ff", forward);
          SmartDashboard.putNumber("roller/pid", pidV);
          SmartDashboard.putNumber("roller/RPS", RPS);
          SmartDashboard.putNumber("roller/ticks", rollerMotor.getEncoder().getPosition());
          SmartDashboard.putNumber("roller/velocity", velocity);
          rollerMotor.setVoltage(forward + pidV);
        }, this);
    } else {
      return Commands.none();
    }
  }

  public boolean hasCoral() {
    if(RollerConstants.k_isEnabled){
      int vert = getRangeVert();
      if (vert > -1 && vert < RollerConstants.VERTICAL_RANGE_UPPER_LIMIT) {
        return true;
      }
      int horz = getRangeHorz();
      if (horz > -1 && horz < RollerConstants.HORIZONTAL_RANGE_UPPER_LIMIT) {
        return true;
      }
    }
    return false;
  }

  public int getRangeHorz() {
    if (RollerConstants.k_isEnabled){
      LaserCan.Measurement horzMeasurement = horizontalRange.getMeasurement();
      int horz = -1;
      if (horzMeasurement != null && horzMeasurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
        horz = horzMeasurement.distance_mm;
      }
      SmartDashboard.putNumber("coral/horizontal", horz);
      return horz;
    } else {
      return -1;
    }
  }

  public int getRangeVert() {
    if(RollerConstants.k_isEnabled){
      LaserCan.Measurement vertMeasurement = verticalRange.getMeasurement();
      int vert = -1;
      if (vertMeasurement != null && vertMeasurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
        vert = vertMeasurement.distance_mm;
      }
      SmartDashboard.putNumber("coral/vertical", vert);
      return vert;
    } else {
      return -1;
    }
  }

  @Override
  public void periodic() {
    getRangeHorz();
    getRangeVert();
    SmartDashboard.putBoolean("coral/has", hasCoral());
    SmartDashboard.putNumber("roller/ticks", rollerMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("roller/velocity", rollerMotor.getEncoder().getVelocity());
  }



  // Command to run the roller with joystick inputs
  public Command runRoller(CoralRoller rollerSubsystem, DoubleSupplier forward, DoubleSupplier reverse) {
    if(RollerConstants.k_isEnabled){
      return Commands.run(
        () -> rollerMotor.set(forward.getAsDouble() - reverse.getAsDouble()), rollerSubsystem);
    } else {
      return Commands.none();
    }
  }
}

