// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private TalonFX climbingTalon = new TalonFX(0);
 
  public Climber() {

    climbingTalon.getConfigurator().apply(Constants.climberConstants.CLIMB_MOTOR_CONFIG.Slot0);
  }

  public InstantCommand climbUp() {
    // we will be ajusting this but this is most of it for the climb
    return new InstantCommand(()->
    climbingTalon.setControl(new PositionDutyCycle(climbingTalon.getPosition().getValueAsDouble() + 0.5)));

  }

  public InstantCommand climbDown() {
    return new InstantCommand(()->
    climbingTalon.setControl(new PositionDutyCycle(climbingTalon.getPosition().getValueAsDouble() - 0.5)));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
