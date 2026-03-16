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
  
  public class visionConstant{
    public static final Transform3d camera1Pos = new Transform3d(new Translation3d(0.07, 0.48, -0.1),
      new Rotation3d(0, 180, 0));
  }
  public class talonIntakeCon {
    public static int ENCODER_ID_CONSTANT = 0;
    
    public static double ENCODER_DOWN_POSITION = 0.89;
    public static double ENCODER_STARTING_POSITION = 0.18;
    public static int INTAKE_MOTOR_ID = 8;
    public static final TalonFXConfiguration SPIN_MOTOR_CONFIG = new TalonFXConfiguration();
    public static int SPIN_MOTOR_ID = 55;
    public static final TalonFXConfiguration INTAKE_MOTOR_CONFIG = new TalonFXConfiguration();
    
    // intake tooth is 16
    
    static {
      // in case forget intake motor up and down is kP of 6 kD of 0.005
      INTAKE_MOTOR_CONFIG.Slot0.kP = 2;
      INTAKE_MOTOR_CONFIG.Slot0.kI = 0;
      INTAKE_MOTOR_CONFIG.Slot0.kD = 0.03;
      SPIN_MOTOR_CONFIG.Slot0.kP = 0.4;
    }

  }
  public class shooterMotorCon{
    public static final TalonFXConfiguration UPPER_MOTOR_CONFIG= new TalonFXConfiguration();
    public static final TalonFXConfiguration LOWER_MOTOR_CONFIG = new TalonFXConfiguration();
    public static final TalonFXConfiguration INDEX_MOTOR_CONFIG = new TalonFXConfiguration();
    public static final int lowerShooter = 6;
    public static final int upperShooter = 1;
    public static final int indexer = 3;
    static{
     UPPER_MOTOR_CONFIG.Slot0.kP = 0.4;
     LOWER_MOTOR_CONFIG.Slot0.kP = 0.4;
     INDEX_MOTOR_CONFIG.Slot0.kP = 0.4;


    }
  }
  
  
}