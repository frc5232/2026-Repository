// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;

  }
  public class climberConstants{
    public static final TalonFXConfiguration CLIMB_MOTOR_CONFIG = new TalonFXConfiguration();
    static{
      CLIMB_MOTOR_CONFIG.Slot0.kP = 20;
    }
  }
  public class visionConstant{
    public static final Transform3d camera1Pos = new Transform3d(new Translation3d(0.07, 0.48, -0.1),
      new Rotation3d(0, 180, 0));
  private static final Transform3d camera2Pos = new Transform3d(new Translation3d(0.45, 0.12, -0.1),
      new Rotation3d(0, 180, 90));
  private static final Transform3d camera3Pos = new Transform3d(new Translation3d(0.14, 0. - 0.21, -0.1),
      new Rotation3d(0, 180, 180));
  private static final Transform3d camera4Pos = new Transform3d(new Translation3d(-0.25, 0.15, -0.1),
      new Rotation3d(0, 180, 270));
  }
  public class talonIntakeCon {
    public static double ENCODER_DOWN_POSITION = 0.32;
    public static double ENCODER_STARTING_POSITION = 0.93;
    public static int INTAKE_MOTOR_ID = 8;
    public static final TalonFXConfiguration SPIN_MOTOR_CONFIG = new TalonFXConfiguration();
    public static int SPIN_MOTOR_ID = 55;
    public static final TalonFXConfiguration INTAKE_MOTOR_CONFIG = new TalonFXConfiguration();
    static {
      // in case forget intake motor up and down is kP of 6 kD of 0.005
      INTAKE_MOTOR_CONFIG.Slot0.kP = 2;
      INTAKE_MOTOR_CONFIG.Slot0.kI = 0;
      INTAKE_MOTOR_CONFIG.Slot0.kD = 0.03;
      SPIN_MOTOR_CONFIG.Slot0.kP = 0.4;
    }

  }
  public class shooterMotorCon{
    public static final TalonFXConfiguration CONSTANT_MOTOR_CONFIG = new TalonFXConfiguration();
    public static final TalonFXConfiguration CONSTANT_MOTOR2_CONFIG = new TalonFXConfiguration();
    static{
      CONSTANT_MOTOR_CONFIG.Slot0.kP = 0.4;
      CONSTANT_MOTOR2_CONFIG.Slot0.kP = 0.4;
    }
  }
  
  
}