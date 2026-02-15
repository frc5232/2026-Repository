// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.time.chrono.MinguoChronology;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.Talon;

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
    public static double encoderConstant = 0.62;
    public static double encoderDownPos = 0.93;

  }

  public class talonIntakeCon {

    public static int INTAKE_MOTOR_ID = 8;
    public static final TalonFXConfiguration SPIN_MOTOR_CONFIG = new TalonFXConfiguration();
    public static int SPIN_MOTOR_ID = 9;
    public static final TalonFXConfiguration INTAKE_MOTOR_CONFIG = new TalonFXConfiguration();
    static {
      // in case forget intake motor up and down is kP of 6 kD of 0.005
      INTAKE_MOTOR_CONFIG.Slot0.kP = 6;
      INTAKE_MOTOR_CONFIG.Slot0.kI = 0;
      INTAKE_MOTOR_CONFIG.Slot0.kD = 0.005;

    }

  }
}
