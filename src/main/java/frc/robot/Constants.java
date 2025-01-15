// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
  
  public static class ChassisConstants {
    public static final int kLeftFrontCanId = 1;
    public static final int kRightFrontCanId = 3;
    public static final int kLeftRearCanId = 2;
    public static final int kRightRearCanId = 4;
    public static final double kDriveP = 1;
    public static final double kDriveI = 0;
    public static final double kDriveD = 0;
    public static final double kDriveToleranceInches = 0.25;
    public static final double kDriveToleranceInchesPerS = 0;
    public static final double kWheelDiameterInches = 4;
    public static final double kGearRatio = 1.0;
    public static final double kPositionConversionFactor = (kWheelDiameterInches * Math.PI) / kGearRatio;
    public static final double kMotorRampTime = 0;
    public static final double kDriveClamp = 0.5;

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
    public static final double ROLLER_EJECT_VALUE = 0.44;
  }
}
