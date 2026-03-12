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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooting extends SubsystemBase {
  /** Creates a new Shooting. */
  TalonFX upperShooter;
  TalonFX lowerShooter;
  DutyCycleOut mCycleOut;
  TalonFX indexShooter;
  public Shooting() {
    mCycleOut = new DutyCycleOut(0);
    upperShooter = new TalonFX(Constants.shooterMotorCon.upperShooter);
    lowerShooter = new TalonFX(Constants.shooterMotorCon.lowerShooter);
    indexShooter = new TalonFX(Constants.shooterMotorCon.indexer);
    upperShooter.getConfigurator().apply(Constants.shooterMotorCon.UPPER_MOTOR_CONFIG);
    lowerShooter.getConfigurator().apply(Constants.shooterMotorCon.LOWER_MOTOR_CONFIG);
    indexShooter.getConfigurator().apply(Constants.shooterMotorCon.INDEX_MOTOR_CONFIG);
    lowerShooter.setControl(new Follower(Constants.shooterMotorCon.upperShooter, MotorAlignmentValue.Opposed));

  }
  private Command shootOut(){
    return new SequentialCommandGroup(new InstantCommand(()-> upperShooter.setControl(mCycleOut.withOutput(1))).until(()->upperShooter.getDutyCycle().getValueAsDouble() >= 0.8).andThen(new InstantCommand(()->indexShooter.setControl(mCycleOut.withOutput(-1)))));
  }
  private Command stopShooting(){
    return new SequentialCommandGroup(new InstantCommand(()->upperShooter.setControl(mCycleOut.withOutput(0))).alongWith(new InstantCommand(()->indexShooter.setControl(mCycleOut.withOutput(0)))));
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Upper shooter speed", upperShooter.getDutyCycle().getValueAsDouble());
    SmartDashboard.putNumber("Lower shooter value", lowerShooter.getDutyCycle().getValueAsDouble());
    SmartDashboard.putNumber("Index shooter speed", indexShooter.getDutyCycle().getValueAsDouble());
  }
}
