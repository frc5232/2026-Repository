package frc.robot;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SpinMotor {
    private static TalonFX spinMotorTalon;
    public SpinMotor(TalonFX spinMotor){
        spinMotorTalon = spinMotor;
    }
    public SpinMotor(){
        this(new TalonFX(0));
    }
    /**
     * 
     * @return the spin motor as a talonFx
     */
    public TalonFX getSpinMotor(){
        return spinMotorTalon;
    }
    /**
     * 
     * @param spinAmount the amount to spin to
     * @return a instant command to do
     */
    public Command spinMotorWithDutyCycle(double spinAmount){
        return new InstantCommand(()-> spinMotorTalon.setControl(new DutyCycleOut(spinAmount)));
    }
    /**
     * 
     * @param rpm the rotations per second to target
     * @return a instant command to do a veloity with that rps
     */
    public Command spinMotorWithVelocityVoltage(double rpm){
        return new InstantCommand(()-> spinMotorTalon.setControl(new VelocityVoltage(rpm)));
    }
    /**
     * 
     * @param spinAmountToIncrease the amount to increase by each time its called until it hits the spin goal
     * @param spinGoal the goal to target
     * @return a new instant command 
     */
    public Command spinMotor(double spinAmountToIncrease,double spinGoal){
        return new InstantCommand(()-> spinMotorTalon.setControl(new DutyCycleOut(spinMotorTalon.getDutyCycle().getValueAsDouble() + spinAmountToIncrease))).until(()-> spinGoal > 0 ? spinGoal >= spinMotorTalon.getDutyCycle().getValueAsDouble() : spinGoal <= spinMotorTalon.getDutyCycle().getValueAsDouble());
    }
    /**
     * 
     * @param slowDownAmount amount to slow down by until it hits the bottom amount to go to
     * @param bottomGoal the goal to go until
     * @return a instant command 
     */
    public Command slowDownMotor(double slowDownAmount,double bottomGoal){
        return new InstantCommand(()-> spinMotorTalon.setControl(new DutyCycleOut(spinMotorTalon.getDutyCycle().getValueAsDouble() - slowDownAmount)))
        .until(()-> bottomGoal > 0 ? bottomGoal <= spinMotorTalon.getDutyCycle().getValueAsDouble() : bottomGoal >= spinMotorTalon.getDutyCycle().getValueAsDouble());
    }
    public Command stopMotor(){
        return new InstantCommand(()-> spinMotorTalon.setControl(new DutyCycleOut(0)));
    }
}
