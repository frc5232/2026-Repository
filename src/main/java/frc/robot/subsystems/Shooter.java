// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private TalonFX constantShooter;
  private TalonFX shooter;
  public Shooter() {
    /**
     * Look into making it into a .withvelovity instead of voltage come testing time
     */
  }
  
  /*
   * A sequential command group with chained instant commnads linked to sppeeding
   * up voltage until its greater then 3.8 for both of them with diffrent increase
   * amounts for both motors
   */
  private Command shootOutWithVoltage() {
    return new SequentialCommandGroup(new InstantCommand(() -> speedingUpVoltage(constantShooter, 0.2))
        .until(() -> constantShooter.getMotorVoltage().getValueAsDouble() >= 3.8).alongWith(
            new InstantCommand(() -> speedingUpVoltage(shooter, 0.4))
                .until(() -> shooter.getMotorVoltage().getValueAsDouble() >= 3.8)));
    

  }
  /**
   * 
   * @return a sequentail command group to spin up both motors until there spinning at 60 rps
   */
  private Command shootOutWithVelocity(){
    return new SequentialCommandGroup(new InstantCommand(()-> speedUpVelocity(constantShooter, 2))
    .until(()-> constantShooter.getVelocity().getValueAsDouble() >= 60).alongWith(new InstantCommand(()->
    speedUpVelocity(shooter, 4))).until(()-> shooter.getVelocity().getValueAsDouble() >=60));
  }
  /*
   * A sequential command group with chained instant commands linked up to our
   * slow down voltage method with will slow it down until its less then 0.5 for
   * both of them and then we call another method to stop them both
   */
  private void stopShootingWithVoltage() {
    new SequentialCommandGroup(new InstantCommand(() -> slowDownVoltage(constantShooter, 0.3))
        .until(() -> constantShooter.getMotorVoltage().getValueAsDouble() <= 0.5)
        .alongWith(new InstantCommand(() -> slowDownVoltage(shooter, 0.3))
            .until(() -> shooter.getMotorVoltage().getValueAsDouble() <= 0.5)));

    stopSpinningMotor(shooter);
  }
  /**
   * slows down both of them by 2 rps until they are less then 10 which then i will call the stopvelocity on th non constant one
   */
  private void stopShootingWithVelocity(){
    new SequentialCommandGroup(new InstantCommand(()-> slowDownVelocity(constantShooter, 2))
    .until(()-> constantShooter.getVelocity().getValueAsDouble() <= 10).alongWith(new InstantCommand(()->
    slowDownVelocity(shooter, 2))).until(()->shooter.getVelocity().getValueAsDouble() <= 10));
  }
  /**
   * 
   * @param mTalonFX       our motor to speed up voltage for
   * @param increaseAmount amount to increase it by each time its called
   * 
   */
  private void speedingUpVoltage(TalonFX mTalonFX, double increaseAmount) {
    mTalonFX.setVoltage(mTalonFX.getMotorVoltage().getValueAsDouble() + increaseAmount);
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
   * @param nmTalonFX      the talonfx to slow down by our decrease amount
   * @param decreaseAmount the amount to slow it down by
   */
  private void slowDownVoltage(TalonFX nmTalonFX, double decreaseAmount) {
    nmTalonFX.setVoltage(nmTalonFX.getMotorVoltage().getValueAsDouble() - decreaseAmount);
  }
  /**
   * 
   * @param mTalonFX our talonfx to slow down
   * @param amountToDerease amount to decrase it by every time its called
   */
  private void slowDownVelocity(TalonFX mTalonFX, double amountToDerease){
    mTalonFX.setControl(new VelocityVoltage(mTalonFX.getVelocity().getValueAsDouble() - amountToDerease));
  }
  /**
   * 
   * @param mFx our motor to stop
   *            use this to completely stop once its low enough
   */
  private void stopSpinningMotor(TalonFX mFx) {
    mFx.setVoltage(0);
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
