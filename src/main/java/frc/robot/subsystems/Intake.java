// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;

//encoder
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

//import com.revrobotics.spark.SparkAbsoluteEncoder;
//import com.revrobotics.spark.SparkLowLevel.MotorType;
//import com.revrobotics.spark.config.AbsoluteEncoderConfig;
//import com.revrobotics.spark.config.AbsoluteEncoderConfigAccessor;
//import com.revrobotics.AbsoluteEncoder;
//import com.revrobotics.spark.*;;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  
  //private SparkMax max;
  public DutyCycleEncoder mEncoder = new DutyCycleEncoder(0, 180.0, 0.1);
  
  //private AbsoluteEncoder mAbsoluteEncoder;
  //private AbsoluteEncoderConfig mAbsoluteEncoderConfig;
  //private AbsoluteEncoderConfigAccessor mAbsoluteEncoderConfigAccessor;
  
  private TalonFX moveToFloorTalonFX;
  private TalonFX closesTalonFX;
  private TalonFXConfigurator mm;
  private TalonFXConfiguration mn;
  
  public Intake() {
    moveToFloorTalonFX = new TalonFX(Constants.talonIntakeCon.INTAKE_MOTOR_ID);
    //mEncoder.setIndexSource(0);
    
    //mEncoder.get();
    
    //mAbsoluteEncoderConfig.apply(AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoder);
    
    mn = Constants.talonIntakeCon.mSpinConfiguration;
    mm.apply(mn.Slot0);
    mm.apply(mn.SoftwareLimitSwitch);
    mm.apply(mn.HardwareLimitSwitch);
    mm.apply(mn.Voltage);
    closesTalonFX.getConfigurator().apply(mn.Voltage);
    closesTalonFX = new TalonFX(Constants.talonIntakeCon.SPIN_MOTOR_ID);
    moveToFloorTalonFX.getConfigurator().apply(Constants.talonIntakeCon.mSpinConfiguration);
  }
  public void intakeBalls(){
    
    moveToFloorTalonFX.setPosition(Units.degreesToRotations(-90));
    closesTalonFX.setVoltage(4);
  }
  public void stopIntake(){
    moveToFloorTalonFX.setPosition(Units.degreesToRotations(0));
    closesTalonFX.set(0);
  }
  public InstantCommand increasePositionBy1(){
    return new InstantCommand(()->moveToFloorTalonFX.setPosition(moveToFloorTalonFX.getPosition().getValueAsDouble() + 1));
    
  }
  public void decreasePositionBy1(){
    moveToFloorTalonFX.setPosition(moveToFloorTalonFX.getPosition().getValueAsDouble() - 1);
  }
  public void publishPosition(){
    SmartDashboard.putNumber("position", mEncoder.get());
  }
  private void startup(){
    /*
     * once i have robot ill call constants and make it so arm goes until its at absolute encoder position
     */
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("position of intake motor", moveToFloorTalonFX.getPosition().getValueAsDouble());
    
  }
}
