// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private TalonFX feeder;
  private TalonFX shooter;
  private DutyCycleEncoder nCycleEncoder;
  public Shooter() {
    // feeder = new TalonFX(Constants.talonIntakeCon.INTAKE_MOTOR_ID);
    // shooter = new TalonFX(Constants.talonIntakeCon.SPIN_MOTOR_ID);
    // nCycleEncoder = new DutyCycleEncoder(0);
    
  }
  private void setArmPoseAtStart(){
    while(nCycleEncoder.get() != 0){
    feeder.setPosition(feeder.getPosition().getValueAsDouble() + 1);
  }
  }
  private void shootOut(){
    feeder.setPosition(Units.degreesToRotations(-90));
    shooter.setVoltage(4);
  }
  private void stopShooting(){
    feeder.setPosition(Units.degreesToRotations(90));
    shooter.setVoltage(0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
