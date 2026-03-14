// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Consumer;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
//encoder
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  
  
  public DutyCycleEncoder absoluteEncoder;
  private MotionMagicExpoVoltage mRequest = new MotionMagicExpoVoltage(0);
  private DutyCycleOut mCycleOut = new DutyCycleOut(0);
  private TalonFX moveToFloorTalonFX;
  private TalonFX spinningMotor;
  private double encoderConstant;
  private double encoderDownPos;
  private double encoderPosition;
  private Trigger startSpinning;
  
  public Intake() {

    moveToFloorTalonFX = new TalonFX(8);

    absoluteEncoder = new DutyCycleEncoder(0);

    moveToFloorTalonFX.getConfigurator().apply(Constants.talonIntakeCon.INTAKE_MOTOR_CONFIG);

    spinningMotor = new TalonFX(Constants.talonIntakeCon.SPIN_MOTOR_ID);

    spinningMotor.getConfigurator().apply(Constants.talonIntakeCon.SPIN_MOTOR_CONFIG);

    encoderConstant = Constants.talonIntakeCon.ENCODER_STARTING_POSITION;

    encoderDownPos = Constants.talonIntakeCon.ENCODER_DOWN_POSITION;

    moveToFloorTalonFX.setPosition(Math.abs(absoluteEncoder.get()-0.79) * 8.57);
    //encoder total change is 0.7
    goToStartPosition();
    startSpinning = new Trigger(() -> goalPos());


  }
  public Command goToStartPosition(){
    return new InstantCommand(()->moveToFloorTalonFX.setControl(new MotionMagicExpoVoltage(0.3)));
  }
  /**
   * Position at bottom = 0.31 encoder
   * Position at middle = 0.62 encoder
   * Position at top = 0.93 encoder
   * 
   * 
   * 
   */
  public Command intakeDownCommand() {
    /*
     * Test deadline v Parrellel
     */
    return Commands.parallel(movingMotor(90), spinMotor(1));
  }

  public Command intakeUpCommand() {
    return Commands.parallel(spinMotor(0), movingMotor(-90));
  
  }

  private Command spinMotor(double dutyCycleAmount) {
    return new InstantCommand(() -> spinningMotor.setControl((mCycleOut.withOutput(dutyCycleAmount))));
  }

  private Command movingMotor(double goalAmount) {
    
    
    return new FunctionalCommand(
        () -> moveToFloorTalonFX.setControl(mRequest.withPosition(Units.rotationsToDegrees(-goalAmount))), () -> {
        },null, startSpinning, this);

  }

  public boolean goalPos() {
    if (absoluteEncoder.get() >= 0.3 && absoluteEncoder.get() <= 0.33) {
      return true;
    } else {
      return false;
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("encoder value",absoluteEncoder.get());
    SmartDashboard.putNumber("Position of Motor", moveToFloorTalonFX.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Spin motor speed",spinningMotor.getDutyCycle().getValueAsDouble());
   
  }

}
