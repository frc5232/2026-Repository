// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private TalonFX feeder = new TalonFX(9);
  private TalonFX shooter = new TalonFX(8);
  public Shooter() {}
  private void shootOut(){
    feeder.setPosition(10);
    shooter.set(1);
  }
  private void stopShooting(){
    feeder.setPosition(0);
    shooter.set(0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
