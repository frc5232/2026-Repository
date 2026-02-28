// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

//encoder
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  public DutyCycleEncoder absoluteEncoder;
  private MotionMagicExpoVoltage mRequest = new MotionMagicExpoVoltage(0);
  private TalonFX moveToFloorTalonFX;
  private TalonFX spinningMotor;
  private double encoderConstant;
  private double encoderDownPos;
  private boolean tryToBeAtDownPose = false;
  private boolean tryToBeAtStartPose = false;
  private double encoderPosition;
    public Intake() {
    moveToFloorTalonFX = new TalonFX(8);
    absoluteEncoder = new DutyCycleEncoder(0);
    
    moveToFloorTalonFX.getConfigurator().apply(Constants.talonIntakeCon.INTAKE_MOTOR_CONFIG);

    spinningMotor = new TalonFX(Constants.talonIntakeCon.SPIN_MOTOR_ID);
    spinningMotor.getConfigurator().apply(Constants.talonIntakeCon.SPIN_MOTOR_CONFIG);
    // our upper position
    encoderConstant = Constants.talonIntakeCon.ENCODER_STARTING_POSITION;
    // our down positon
    encoderDownPos  = 0.32;

  }
  private void updateEncoderPose(){
    encoderPosition = absoluteEncoder.get();
  }
  /**
   * our command to go to our down position using a set control until were at our
   * down positon
   */
  public void goToDownPositionCommand() {
       //tryToBeAtDownPose = true;
       double x =absoluteEncoder.get();
       if(!( x>= encoderDownPos && x < 0.8)){
       while(atDownPosition() == false){
        moveToFloorTalonFX.setControl(mRequest.withPosition(moveToFloorTalonFX.getPosition().getValueAsDouble() + 0.5));
     }
     }else{
      while (atDownPosition() == false) {
        moveToFloorTalonFX.setControl(mRequest.withPosition((moveToFloorTalonFX.getPosition().getValueAsDouble() - 0.5)));
       }
      
     }
      spinningMotor.setControl(new VelocityVoltage(75.99));
    tryToBeAtDownPose = true;
    tryToBeAtStartPose = false;
   
  }

  /*
   * This is our command to go to our starting pose by chaining our movement to
   * starting pose and saying to do it
   * until startingpose returns true
   */
  public void gotoStartPositonCommand() {
     spinningMotor.setControl(new VelocityVoltage(0));
     while(atStartingPosition() == false){
    moveToFloorTalonFX.setControl(new MotionMagicExpoVoltage(moveToFloorTalonFX.getPosition().getValueAsDouble() - 0.5));}
    
  
  }

  /**
   * this is just to publish our encoder position to our smartdashboard
   */
  public void publishPosition() {
    SmartDashboard.putNumber("position", absoluteEncoder.get());
  }

  /*
   * only returns true if encoder is greater then encoder constant and less then
   * encoder constant by 0.03
   */
  private boolean atStartingPosition() {
    double x = absoluteEncoder.get();
    if(x > 0.95 || x < 0.03){
      return true;
    }else{
      return false;
    }
  }

  /*
   * background for the .until for command to go to goal position
   * which works by saying if our encoder positon is greater then the ncoder down
   * position constant
   * - 0.02 it will return true otherwise it will return false
   */
  private boolean atDownPosition() {
    double x = absoluteEncoder.get();
    if(x <= encoderDownPos + 0.01 && x >= encoderDownPos - 0.01 ){
      return true;
    }else{
      return false;
    }
  }
  /**
   * Only call when its already at a low velocity voltage or in emergency
   */
  private void stopSpinMotor(){
    spinningMotor.setControl(new VelocityVoltage(0));
  }
  @Override
  public void periodic() {
    // publishPosition();
    
    
    // updateEncoderPose();
  }
}
