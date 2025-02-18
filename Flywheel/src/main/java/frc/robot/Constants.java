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
     public static final int kOperatorControllerPort = 1;
  }


  public static class CanIds {

    public static final int FLYWHEEL1 = 4;
     public static final int FLYWHEEL2 = 5;
      public static final int FLYWHEEL3 = 6;

    public static final int CONVEYOR1 = 1;
    
    public static final int ELEVATOR1 = 2;

  }



  public static class MotorSetPoint{

    public static final double FLYWHEEL_FUll_SPEED = 5000;
    public static final double FLYWHEEL_HALF_SPEED = 2500;
    public static final double FLYWHEEL_REVERSE = -5000;

    public static final double FLYWHEEL_TOLERANCE = 10 ;

    public static final int REVERSE_Conveyor = -500;
    public static final int RUN_CONVEYOR = 1000;

    public static final double ELEVATOR_MAX_HEIGHT = 50 ;
    public static final double ELEVATOR_MIN_HEIGHT = 0 ;

    public static final double ELEVATOR_TOLERANCE = 0.5 ;
  }
}
