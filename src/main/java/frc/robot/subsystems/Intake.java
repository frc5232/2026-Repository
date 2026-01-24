// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private TalonFX furthesTalonFX = new TalonFX(1);
  private TalonFX moveToFloorTalonFX = new TalonFX(2);
  private TalonFX closesTalonFX = new TalonFX(3);
  
  public Intake() {}
  private void intakeBalls(){
    moveToFloorTalonFX.setPosition(-90);
    furthesTalonFX.set(0.8);
    closesTalonFX.set(0.8);
  }
  private void stopIntake(){
    moveToFloorTalonFX.setPosition(0);
    furthesTalonFX.set(0);
    closesTalonFX.set(0);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
