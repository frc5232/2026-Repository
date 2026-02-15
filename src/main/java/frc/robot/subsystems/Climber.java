// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private TalonFX climbingTalon = new TalonFX(0);

  public Climber() {

  }

  private void climbUp() {
    // we will be ajusting this but this is most of it for the climb
    climbingTalon.setPosition(90);

  }

  private void climbDown() {
    climbingTalon.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
