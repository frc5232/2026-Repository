package frc.robot;

import java.util.ArrayList;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;

public class Intake {
    private static ArrayList<SpinMotor> arrayOfIntakeSpinMotors;
    private static ArrayList<PivotMotor> arrayOfPivotMotors;
    private static ArrayList<String> arrayOfIntakeSpinMotorNames;
    private static ArrayList<Integer> arrayOfIntakeSpinMotorIds;
    private static ArrayList<String> arrayOfIntakePivotMotorNames;
    private static ArrayList<Integer> arrayOfIntakePivotMotorIds;

    private static Throwable addingThrowable = new Throwable("Wrong type of motor it should be TalonFX");

    public static void addASpinMotor(TalonFX spinMotor) throws Throwable {
        if (spinMotor.equals(new TalonFX(spinMotor.getDeviceID()))) {
            arrayOfIntakeSpinMotors.add(new SpinMotor(spinMotor));
            arrayOfIntakeSpinMotorNames.add(spinMotor.getDescription());
            arrayOfIntakeSpinMotorIds.add(spinMotor.getDeviceID());
        }
            throw addingThrowable;
        
    }

    public static void addMultiplePivotMotors(ArrayList<TalonFX> listOfPivotMotors) throws Throwable {

        for (int i = 0; i < listOfPivotMotors.size(); i++) {
            if (listOfPivotMotors.get(i).equals(new TalonFX(listOfPivotMotors.get(i).getDeviceID()))) {
                arrayOfPivotMotors.add(new PivotMotor(listOfPivotMotors.get(i)));
                arrayOfIntakePivotMotorIds.add(listOfPivotMotors.get(i).getDeviceID());
                arrayOfIntakePivotMotorNames.add(listOfPivotMotors.get(i).getDescription());
            } else {
                Throwable mThrowable = new Throwable("Wrong pivot motor type at " + i, addingThrowable);
                throw mThrowable;
            }

        }
    }

    public static void addMultipleSpinMotors(ArrayList<TalonFX> listOfSpinMotors) throws Throwable {
        for (int i = 0; i < listOfSpinMotors.size(); i++) {
            if (listOfSpinMotors.get(i).equals(new TalonFX(listOfSpinMotors.get(i).getDeviceID()))) {
                arrayOfIntakeSpinMotors.add(new SpinMotor(listOfSpinMotors.get(i)));
                arrayOfIntakeSpinMotorIds.add(listOfSpinMotors.get(i).getDeviceID());
                arrayOfIntakeSpinMotorNames.add(listOfSpinMotors.get(i).getDescription());
            } else {
                Throwable mThrowable = new Throwable("Wrong spin motor type at " + i, addingThrowable);
                throw mThrowable;
            }
        }
    }

    public static void addAPivotMotor(TalonFX pivotMotor) throws Throwable {
        if (pivotMotor.equals(new TalonFX(pivotMotor.getDeviceID()))) {
            arrayOfPivotMotors.add(new PivotMotor(pivotMotor));
            arrayOfIntakePivotMotorIds.add(pivotMotor.getDeviceID());
            arrayOfIntakePivotMotorNames.add(pivotMotor.getDescription());
        }
        throw addingThrowable;
    }

    public static TalonFX getAPivotMotor(int pivotMotorId) throws Throwable {
        Throwable eThrowable = new Throwable("Does not exist within object\n Error at method :" + checkWhichMethod(1));
        for (int i = 0; i < arrayOfPivotMotors.size(); i++) {
            if (arrayOfPivotMotors.get(i).getPivotMotor().getDeviceID() == pivotMotorId) {
                return arrayOfPivotMotors.get(i).getPivotMotor();
            }
        }
        throw eThrowable;
    }

    public static TalonFX getAPivotMotor(String name) throws Throwable {
        Throwable eThrowable = new Throwable("Does not exist within object\n Error at method :" + checkWhichMethod(1));
        for (int i = 0; i < arrayOfPivotMotors.size(); i++) {
            if (arrayOfPivotMotors.get(i).getPivotMotor().getDescription() == name) {
                return arrayOfPivotMotors.get(i).getPivotMotor();
            }
        }
        throw eThrowable;
    }

    public static TalonFX getASpinMotor(int spinMotorId) throws Throwable {
        Throwable eThrowable = new Throwable("Does not exist within object\n Error at method :" + checkWhichMethod(2));
        for (int i = 0; i < arrayOfIntakeSpinMotors.size(); i++) {
            if (arrayOfIntakeSpinMotors.get(i).getSpinMotor().getDeviceID() == spinMotorId) {
                return arrayOfIntakeSpinMotors.get(i).getSpinMotor();
            }
        }
        throw eThrowable;
    }

    public static TalonFX getASpinMotor(String name) throws Throwable {
        Throwable eThrowable = new Throwable("Does not exist within object\n Error at method :" + checkWhichMethod(2));
        for (int i = 0; i < arrayOfIntakeSpinMotors.size(); i++) {
            if (arrayOfIntakeSpinMotors.get(i).getSpinMotor().getDescription() == name) {
                return arrayOfIntakeSpinMotors.get(i).getSpinMotor();
            }
        }

        throw eThrowable;
    }

    public static ArrayList<TalonFX> getAllSpinMotors() {
        ArrayList<TalonFX> mList = new ArrayList<TalonFX>();
        for(int i = 0; i < arrayOfIntakeSpinMotors.size();i++){
            mList.add(arrayOfIntakeSpinMotors.get(i).getSpinMotor());
        }
        return mList;
    }

    public static ArrayList<TalonFX> getAllPivotMotors() {
        ArrayList<TalonFX> mList = new ArrayList<TalonFX>();
        for(int i = 0; i < arrayOfPivotMotors.size();i++){
            mList.add(arrayOfPivotMotors.get(i).getPivotMotor());
        }
        return mList;
    }

    public static void removePivotMotor(String name) {
        for (int i = 0; i < arrayOfPivotMotors.size(); i++) {
            if (arrayOfPivotMotors.get(i).getPivotMotor().getDescription() == name) {
                arrayOfPivotMotors.remove(i);
                arrayOfIntakePivotMotorIds.remove(i);
                arrayOfIntakePivotMotorIds.remove(i);
                break;
            }
        }
    }

    public static void removePivotMotor(int id) {
        for (int i = 0; i < arrayOfPivotMotors.size(); i++) {
            if (arrayOfPivotMotors.get(i).getPivotMotor().getDeviceID() == id) {
                arrayOfPivotMotors.remove(i);
                arrayOfIntakePivotMotorIds.remove(i);
                arrayOfIntakePivotMotorIds.remove(i);
                break;
            }
        }
    }

    public static void removeSpinMotor(int id) {
        for (int i = 0; i < arrayOfIntakeSpinMotors.size(); i++) {
            if (arrayOfIntakeSpinMotors.get(i).getSpinMotor().getDeviceID() == id) {
                arrayOfIntakeSpinMotorIds.remove(i);
                arrayOfIntakeSpinMotorNames.remove(i);
                arrayOfIntakeSpinMotors.remove(i);
                break;
            }
        }
    }

    public static void removeSpinMotor(String name) {
        for (int i = 0; i < arrayOfIntakeSpinMotors.size(); i++) {
            if (arrayOfIntakeSpinMotors.get(i).getSpinMotor().getDescription() == name) {
                arrayOfIntakeSpinMotorIds.remove(i);
                arrayOfIntakeSpinMotorNames.remove(i);
                arrayOfIntakeSpinMotors.remove(i);
                break;
            }
        }
    }

    private static String checkWhichMethod(int methodNumber) {
        switch (methodNumber) {
            case 1:
                return "getAPivotMotor()";
            case 2:
                return "getASpinMotor()";
            default:
                return "not found";
        }
    }

    public Intake(ArrayList<TalonFX> spinMotors, ArrayList<TalonFX> pivotMotors) {
        for(int i = 0; i <spinMotors.size(); i++){
            arrayOfIntakeSpinMotors.add(new SpinMotor(spinMotors.get(i)));
        }
         for(int i = 0; i <pivotMotors.size(); i++){
            arrayOfPivotMotors.add(new PivotMotor(pivotMotors.get(i)));
        }
    }

    public Intake() {
        this(new ArrayList<TalonFX>(), new ArrayList<TalonFX>());
    }

}
