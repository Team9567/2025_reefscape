// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.studica.frc.AHRS.NavXComType;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class ControllerConstants {
    public static final int kControllerControllerPort = 1;
  }

  public static class ButtonConstants {
    public static final int kButtonX = 3;
    public static final int kButtonY = 4;
    public static final int kButtonB = 2;
    public static final int kButtonA = 1;
    public static final int kButtonLB = 5;
    public static final int kButtonRB = 6;
    public static final int kButtonLeftStick = 9;
    public static final int kButtonRightStick = 10;
    public static final int kButtonBack = 7;
    public static final int kButtonStart = 8;
  }
  
  public static class ChassisConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kLeftFrontCanId = 2;
    public static final int kRightFrontCanId = 4;
    public static final int kLeftRearCanId = 1;
    public static final int kRightRearCanId = 3;
    public static final double kDriveP = 1;
    public static final double kDriveI = 0;
    public static final double kDriveD = 0.12;
    public static final double kTurnP = 0.06;
    public static final double kTurnI = 0.0;
    public static final double kTurnD  = 0.004;
    public static final double kDriveToleranceInches = 1.0;
    public static final double kDriveToleranceInchesPerS = 0;
    public static final double kTurnToleranceDeg = 1.5;
    public static final double kTurnRateToleranceDegPerS = 1;
    public static final double kWheelDiameterInches = 6.17;
    public static final double kGearRatio = 8.46;
    public static final double kPositionConversionFactor = (kWheelDiameterInches * Math.PI) / kGearRatio;
    public static final double kMotorRampTime = 2.0;
    public static final double kDriveClamp = 0.5;
    public static final double kTurnClamp = 0.5;
    public static final boolean kGyroReversed = true;
    public static final NavXComType kGyroPort = NavXComType.kMXP_SPI;
    public static final double kLowGearSpeed = 0.35;
   
  }
  public static class MathUtils {
    public static double Clamp(double input, double limit) {
        if (input > limit) {
          return limit;
         } else if (input < -1 * limit){
            return -1 * limit; 
         } else{
            return input;
          }
        }
    }

  public static final class RollerConstants {
    public static final int ROLLER_MOTOR_ID = 5;
    public static final int ROLLER_MOTOR_CURRENT_LIMIT = 60;
    public static final double ROLLER_MOTOR_VOLTAGE_COMP = 10;
    public static final double ROLLER_EJECT_VALUE = 0.30;
    public static final double ROLLER_EJECT_VALUE2 = 0.0;
    public static final double ROLLER_SLOW_EJECT_VALUE = 0.20;
    public static final double ROLLER_REVERSE_VALUE = 0.0;
    public static final double ROLLER_REVERSE_VALUE2 = 0.65;
    public static final double ROLLER_GEAR_RATIO = 2.0;
  }

  public static final class AlgaeConstants {
    public static final int PIVOT_MOTOR_ID = 6;
    public static final int INTAKE_MOTOR_ID = 7;
    public static final int ALGAE_RANGER_ID = 15;
    public static final int ALGAE_MOTOR_CURRENT_LIMIT = 60;
    public static final double ALGAE_MOTOR_VOLTAGE_COMP = 10;
    public static final double ALGAE_EJECT_VALUE = 0.44;
    public static final double ALGAE_ARM_REACH_SPEED = 0.1;
    public static final double ALGAE_ARM_HOME_POSITION = 0.0;
    public static final double ALGAE_ARM_INTAKE_POSITION = ALGAE_ARM_HOME_POSITION + 0.19; // ~70degrees/360
    public static final double INTAKE_MOTOR_SPEED = 0.1;
    public static final int SENSOR_LIMIT = 750;
    public static final double ALGAE_ARM_RETURN_SPEED = -0.1;
    public static final double SHOOT_MOTOR_SPEED = -0.1;
  }

  public static final class ClimberConstants {
    public static final int CLIMBER_MOTOR_ID = 8;
    public static final int CLIMBER_MOTOR_CURRENT_LIMIT = 40;
    public static final double CLIMBER_MOTOR_VOLTAGE_COMP = 11;
    public static final double CLIMBER_MOTOR_INITIALIZE_SPEED = -0.60;
    public static final double CLIMBER_MOTOR_DOWN_LIMIT = 0.0;
    public static final double CLIMBER_MOTOR_UP_LIMIT = 1140.0;
    public static final int CLIMBER_LIMIT_PORT = 0;
    public static final double CLIMBER_MOTOR_UP_SPEED = 1.00;
    public static final double CLIMBER_MOTOR_DOWN_SPEED = -0.50;
  }

  public static final class AutosConstants {
    public static final double k_leftDist1 = 58;
    public static final double k_leftAngle1 = -45;
    public static final double k_leftDist2 = 89;
    public static final double k_rightDist1 = 56;
    public static final double k_rightAngle1 = 45;
    public static final double k_rightDist2 = 88;
    public static final double k_middleDist1 = 88;
    public static final double k_rollerForwardSpeed = 0.0;
    public static final double k_rollerReverseSpeed = 0.0;
  }

  public static final class RobotConstants {
    public static final boolean k_IsCompBot = true;
  }
}
