// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private TalonFX constantShooter;
  private TalonFX shooter;
  private TalonFX constantShooter2;
  public Shooter() {
    /**
     * Look into making it into a .withvelovity instead of voltage come testing time
     */
    shooter = new TalonFX(0);
    constantShooter = new TalonFX(1);
    constantShooter2 = new TalonFX(6);
  }

  /**
   * 
   * @return a sequentail command group to spin up both motors until there spinning at 60 rps
   */
  public Command shootOutWithVelocity(){
    return new SequentialCommandGroup(new InstantCommand(()-> speedUpVelocity(constantShooter, 2))
    .until(()-> constantShooter.getVelocity().getValueAsDouble() >= 40).alongWith(new InstantCommand(()->
    speedUpVelocity(shooter, 4))).until(()-> shooter.getVelocity().getValueAsDouble() >=40).alongWith(
      new InstantCommand(()-> speedUpVelocity(constantShooter2,4 )).until(()->constantShooter2.getVelocity().getValueAsDouble() >= 40)
    ));
  }
  /**
   * slows down both of them by 2 rps until they are less then 10 which then i will call the stopvelocity on th non constant one
   */
  private void stopShootingWithVelocity(){
    new SequentialCommandGroup(new InstantCommand(()-> slowDownVelocity(constantShooter, 2))
    .until(()-> constantShooter.getVelocity().getValueAsDouble() <= 10).alongWith(new InstantCommand(()->
    slowDownVelocity(shooter, 2))).until(()->shooter.getVelocity().getValueAsDouble() <= 10)).alongWith(
      new InstantCommand(()-> slowDownVelocity(constantShooter2, 2)).until(()-> constantShooter2.getVelocity().getValueAsDouble() <=10));
    
    stopSpinningMotorVelocity(shooter);
    }

  /**
   * 
   * @param mTalonFX our talon to spin up
   * @param increaseAmount amount to increase by every time called
   * 
   */
  private void speedUpVelocity(TalonFX mTalonFX, double increaseAmount){
    mTalonFX.setControl(new VelocityVoltage(mTalonFX.getVelocity().getValueAsDouble() + increaseAmount));
  }
  
  /**
   * 
   * @param mTalonFX our talonfx to slow down
   * @param amountToDerease amount to decrase it by every time its called
   */
  private void slowDownVelocity(TalonFX mTalonFX, double amountToDerease){
    mTalonFX.setControl(new VelocityVoltage(mTalonFX.getVelocity().getValueAsDouble() - amountToDerease));
  }
  public Command shoot(){
    return new InstantCommand(()->{shooter.setControl(new DutyCycleOut(-0.5));constantShooter.setControl(new DutyCycleOut(0.5));    constantShooter2.setControl(new DutyCycleOut(-0.5));});
    
  }
  private void speedUpDutyCycle(TalonFX mFx, double speed){
    mFx.setControl(new DutyCycleOut(mFx.getDutyCycle().getValueAsDouble() + speed));
  }
  public Command shootWithDutyCycle(){
    return new SequentialCommandGroup(new InstantCommand(()->speedUpDutyCycle(shooter, -0.08)).until(()-> shooter.getDutyCycle().getValueAsDouble() <= -0.8)
    .alongWith(new InstantCommand(()-> {speedUpDutyCycle(constantShooter, 0.08);speedUpDutyCycle(constantShooter2, -0.05);}).until(()->constantShooter.getDutyCycle().getValueAsDouble() >= 0.8 && constantShooter2.getDutyCycle().getValueAsDouble() <= -0.8)));
  }
  
  /**
   * 
   * @param mFx our talon fx we want to complete stop spinnng
   * only call when its already slowed down significantly or in emeergency situtations
   */
  private void stopSpinningMotorVelocity(TalonFX mFx){
    mFx.setControl(new VelocityVoltage(0));
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
