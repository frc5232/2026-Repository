// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.opencv.objdetect.RefineParameters;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private TalonFX constantShooter;
  private TalonFX shooter;
  private TalonFX constantShooter2;
  private double targetFlywheelRPS;
  private VelocityVoltage mRequest = new VelocityVoltage(0);
  public Shooter() {
    /**
     * Look into making it into a .withvelovity instead of voltage come testing time
     */
    /**
     * our bottom shooter
     */
    shooter = new TalonFX(0);
    //the first shooter motor that we spin up constantly
    constantShooter = new TalonFX(6);
    // the second shooter motor that we spin up constantly
    constantShooter2 = new TalonFX(1);
    
    constantShooter2.getConfigurator().apply(Constants.shooterMotorCon.CONSTANT_MOTOR2_CONFIG);
    constantShooter.getConfigurator().apply(Constants.shooterMotorCon.CONSTANT_MOTOR_CONFIG);
   // constantShooter.setControl(new Follower(constantShooter2.getDeviceID(), MotorAlignmentValue.Opposed));
   
  }
  /**
   * Our command to shoot using velocity voltage by telling it to speed up both the constant shooters in one
   * and also the regular shooter with all going to the same RPS
   */
  public Command shootOutWithVelocity(){
    targetFlywheelRPS = 90;
      // return new SequentialCommandGroup (new InstantCommand(()->speedUpVelocity(constantShooter2, targetFlywheelRPS))
      // .alongWith(new InstantCommand(()->speedUpVelocity(constantShooter, targetFlywheelRPS))
      // .alongWith(speedUpVelocity(shooter, targetFlywheelRPS))));
      return new SequentialCommandGroup(new InstantCommand(()-> speedUpVelocity(constantShooter, targetFlywheelRPS))
      .alongWith(new InstantCommand(()-> speedUpVelocity(constantShooter2, -targetFlywheelRPS)).alongWith(new InstantCommand(()->speedUpVelocity(shooter, -targetFlywheelRPS)))));
      
    
    

    
    
  }
  /**
   * Slows down both of the constant shooters to 5 rps and shooter regular to 5 rps aswll then once all are less then 10 it stops the shooter motor
   */
  public void stopShootingWithVelocity(){
    
    slowDownVelocity(shooter, 20);
    
    
    if(shooter.getVelocity().getValueAsDouble() <= 22) {
    stopSpinningMotorVelocity(shooter);}
    
    }

  /**
   * 
   * @param mTalonFX our talon to spin up
   * @param goalAmount our goal amount
   * 
   */
  private void speedUpVelocity(TalonFX mTalonFX, double goalAmount){
    mTalonFX.setControl(mRequest.withVelocity(goalAmount));
    }
  
  
  
  /**
   * 
   * @param mTalonFX our talonfx to slow down
   * @param amountToDerease amount to decrase it by every time its called
   */
  private void slowDownVelocity(TalonFX mTalonFX, double goalAmount){
    
    mTalonFX.setControl(mRequest.withVelocity(goalAmount));
    //).until(()-> mTalonFX.getVelocity().getValueAsDouble() <= goalAmount);
  }
  /**
   * 
   * @return a squental command group to slow it down by the values we want
   */
  public Command slowDownDutyCycle(){
    return new SequentialCommandGroup(new InstantCommand(()->slowDownCycle(shooter,0.1,0.1)),
    new InstantCommand(()->slowDownCycle(constantShooter, -0.1, -0.5)),
     new InstantCommand(()->  slowDownCycle(constantShooter2, 0.1, 0.5)));
    }
    
  /**
   * 
   * @param mFx Our talon fx we want to slow down
   * @param speed our speed to slow it down by every time its called
   * @param goalSpeed our slow speed we want to hit
   */
  private void slowDownCycle(TalonFX mFx,double speed,double goalSpeed){
    new InstantCommand(()->mFx.setControl(new DutyCycleOut(shooter.getDutyCycle().getValueAsDouble() - speed))).until(()-> goalSpeed > 0 ? mFx.getDutyCycle().getValueAsDouble() <= goalSpeed : mFx.getDutyCycle().getValueAsDouble() >= goalSpeed);
  }
  /**
   * 
   * @param mFx our talon fx to speed up
   * @param speed the speed to speed it up by
   */
  private void speedUpDutyCycle(TalonFX mFx, double speed){
    mFx.setControl(new DutyCycleOut(mFx.getDutyCycle().getValueAsDouble() + speed));
  }
  /**
   * 
   * @return a sequentail command group with all the values to go till
   */
  public Command shootWithDutyCycle(){
    return new SequentialCommandGroup(new InstantCommand(()->speedUpDutyCycle(shooter, -0.1)).until(()-> shooter.getDutyCycle().getValueAsDouble() <= -0.9)
    .alongWith(new InstantCommand(()-> speedUpDutyCycle(constantShooter, 0.1)).until(()-> constantShooter.getDutyCycle().getValueAsDouble()>= 0.9))
    .alongWith(new InstantCommand(()-> speedUpDutyCycle(constantShooter2, -0.1)).until(()-> constantShooter2.getDutyCycle().getValueAsDouble() <= -0.9)));
  }
  
  /**
   * 
   * @param mFx our talon fx we want to complete stop spinnng
   * only call when its already slowed down significantly or in emeergency situtations
   */
  private void stopSpinningMotorVelocity(TalonFX mFx){
   mFx.setControl(mRequest.withVelocity(0));
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
