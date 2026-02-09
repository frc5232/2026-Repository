// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.Talon;

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
  public class talonIntakeCon{
     public static int INTAKE_MOTOR_ID = 8;
     public static final TalonFXConfiguration mSpinConfiguration = new TalonFXConfiguration();
     public static int SPIN_MOTOR_ID = 55;
     public static final TalonFXConfiguration mIntakeMotorConfig = new TalonFXConfiguration();
     static{
      
      mSpinConfiguration.Voltage.PeakForwardVoltage = 3;
      mSpinConfiguration.Voltage.PeakReverseVoltage = -3;
      mSpinConfiguration.Slot0.kP = 0;
      mSpinConfiguration.Slot0.kI = 0;
      mSpinConfiguration.Slot0.kD = 0;
      mSpinConfiguration.HardwareLimitSwitch.ForwardLimitAutosetPositionValue = 0;
      mSpinConfiguration.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = true;
      mSpinConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      mSpinConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.degreesToRotations(10);
      mSpinConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.degreesToRotations(90);
      mSpinConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
      mIntakeMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.degreesToRotations(10);
      mIntakeMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      mIntakeMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.degreesToRadians(90);
      mIntakeMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
     }
     
  }
}
