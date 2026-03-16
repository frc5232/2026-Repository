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
  
  private double goalPositionToTarget;
  public DutyCycleEncoder absoluteEncoder;
  private MotionMagicExpoVoltage mRequest = new MotionMagicExpoVoltage(0);
  private DutyCycleOut mCycleOut = new DutyCycleOut(0);
  private TalonFX moveToFloorTalonFX;
  private TalonFX spinningMotor;
  private double encoderConstant;
  private double encoderDownPos;
  private Trigger startSpinning;
  private Consumer<Boolean> mConsumer;
  private boolean positionValue;
  public Intake() {

    moveToFloorTalonFX = new TalonFX(Constants.talonIntakeCon.INTAKE_MOTOR_ID);

    absoluteEncoder = new DutyCycleEncoder(Constants.talonIntakeCon.ENCODER_ID_CONSTANT);

    moveToFloorTalonFX.getConfigurator().apply(Constants.talonIntakeCon.INTAKE_MOTOR_CONFIG);

    spinningMotor = new TalonFX(Constants.talonIntakeCon.SPIN_MOTOR_ID);

    spinningMotor.getConfigurator().apply(Constants.talonIntakeCon.SPIN_MOTOR_CONFIG);

    encoderConstant = Constants.talonIntakeCon.ENCODER_STARTING_POSITION;

    encoderDownPos = Constants.talonIntakeCon.ENCODER_DOWN_POSITION;

    moveToFloorTalonFX.setPosition(Math.abs(absoluteEncoder.get()-encoderConstant) * 8.57);
    //encoder total change is 0.7
    goToStartPosition();
    goalPositionToTarget = encoderConstant;
    startSpinning = new Trigger(() -> goalPos());
    positionValue = goalPos() ? true : false;
    mConsumer = positionValue -> {
    
     if(positionValue){
        holdPosition();}
        else{

        }
      
      };
        

  }
  public Command goToStartPosition(){
    goalPositionToTarget = encoderConstant;
    return new InstantCommand(()->moveToFloorTalonFX.setControl(new MotionMagicExpoVoltage(0.3)));
  }
 
  public Command holdPosition(){
   return Commands.run(()->moveToFloorTalonFX.setControl(mRequest.withPosition(Units.degreesToRotations(goalPositionToTarget))),this);
  }
  /**
   * Position at bottom = 0.31 encoder
   * Position at middle = 0.62 encoder
   * Position at top = 0.93 encoder
   * 
   * 
   * 
   */

   /**
    * 
    * @return A command in parallel to run our motors
    */
  public Command intakeDownCommand() {
    /*
     * Test deadline v Parrellel
     */
    goalPositionToTarget = encoderDownPos;
    return Commands.parallel(movingMotor(5), spinMotor(0.32));
  }
  /**
   * parallel commands to run until they are both done
   * @return A command
   */
  public Command intakeUpCommand() {
    return Commands.parallel(spinMotor(0), movingMotor(-5));
  
  }
  /**
   * 
   * @param dutyCycleAmount our duty cycle amount we want the motor spinning by (Between 1 and -1)
   * @return A Instant Command to spin the motor
   */
  private Command spinMotor(double dutyCycleAmount) {
    return new InstantCommand(() -> spinningMotor.setControl((mCycleOut.withOutput(dutyCycleAmount))));
  }
  /**
   * 
   * @param goalAmount The goal amount in degrees you want the motor to move by
   * @return A functional command to move the motor with
   */
  private Command movingMotor(double goalAmount) {
    
    return new FunctionalCommand(
        () -> 
        moveToFloorTalonFX.setControl(mRequest.withPosition(Units.rotationsToDegrees(goalAmount))),
        ()->{},
        mConsumer,
        startSpinning, 
        this);

  }
  /**
   * 
   * @return A Boolean to say if were near our goal pos
   */
  public Boolean goalPos() {
    if (absoluteEncoder.get() <= goalPositionToTarget && absoluteEncoder.get() >= goalPositionToTarget - 0.03) {
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
