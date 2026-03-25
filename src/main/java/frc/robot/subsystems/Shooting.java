// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class Shooting extends SubsystemBase {
  /** Creates a new Shooting. */
  TalonFX upperShooter;
  TalonFX lowerShooter;
  DutyCycleOut mCycleOut;
  TalonFX indexShooter;
  Trigger startBottomMotor;
  double goalSpeedForDutyCycleUpper;
  double goalSpeedForDutyCycleIndexer;
  

  public Shooting() {
    mCycleOut = new DutyCycleOut(0);
    upperShooter = new TalonFX(Constants.shooterMotorCon.upperShooter);
    lowerShooter = new TalonFX(Constants.shooterMotorCon.lowerShooter);
    indexShooter = new TalonFX(Constants.shooterMotorCon.indexer);
    upperShooter.getConfigurator().apply(Constants.shooterMotorCon.UPPER_MOTOR_CONFIG);
    lowerShooter.getConfigurator().apply(Constants.shooterMotorCon.LOWER_MOTOR_CONFIG);
    indexShooter.getConfigurator().apply(Constants.shooterMotorCon.INDEX_MOTOR_CONFIG);
    lowerShooter.setControl(new Follower(Constants.shooterMotorCon.upperShooter, MotorAlignmentValue.Opposed));
    goalSpeedForDutyCycleIndexer = Constants.shooterMotorCon.GOAL_SPEED_FOR_INDEX;
    goalSpeedForDutyCycleUpper = Constants.shooterMotorCon.GOAL_SPEED_FOR_UPPER;
    
    startBottomMotor = new Trigger(() -> checkMotorSpeed());

  }

  public boolean checkMotorSpeed() {

    if (upperShooter.getDutyCycle().getValueAsDouble() <= goalSpeedForDutyCycleUpper + goalSpeedForDutyCycleUpper * 0.05) {
      return true;
    }
    return false;
  }

  /**
   * A command to start shooting our balls
   * 
   * @return a sequential command group with our commands
   */
  public Command shootOut() {
    return new FunctionalCommand(
        () -> {
        },
        () -> upperShooter.setControl(mCycleOut.withOutput(-goalSpeedForDutyCycleUpper)),

        startBottomMotor -> {
          
            
            indexShooter.setControl(mCycleOut.withOutput(-goalSpeedForDutyCycleIndexer));
          
        },
        startBottomMotor,
        this);
  }

  /**
   * A command to stop shooting our balls
   * 
   * @return a sequential command group with our commands
   */
  public Command stopShooting() {
    return new SequentialCommandGroup(new InstantCommand(() -> upperShooter.setControl(mCycleOut.withOutput(0)))
        .alongWith(new InstantCommand(() -> indexShooter.setControl(mCycleOut.withOutput(0)))));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Upper shooter speed", upperShooter.getDutyCycle().getValueAsDouble());
    SmartDashboard.putNumber("Lower shooter value", lowerShooter.getDutyCycle().getValueAsDouble());
    SmartDashboard.putNumber("Index shooter speed", indexShooter.getDutyCycle().getValueAsDouble());
  }
}
