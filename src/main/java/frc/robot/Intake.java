package frc.robot;

import java.util.ArrayList;

import com.ctre.phoenix6.hardware.TalonFX;

public class Intake {
    private static ArrayList<TalonFX> arrayOfIntakeSpinMotors;
    private static ArrayList<TalonFX> arrayOfPivotMotors;
    private static ArrayList<String> arrayOfIntakeSpinMotorNames;
    private static ArrayList<Integer> arrayOfIntakeSpinMotorIds;
    private static ArrayList<String> arrayOfIntakePivotMotorNames;
    private static ArrayList<Integer> arrayOfIntakePivotMotorIds;

    public static void addASpinMotor(TalonFX intakeMotor) {
        arrayOfIntakeSpinMotors.add(intakeMotor);
        arrayOfIntakeSpinMotorNames.add(intakeMotor.getDescription());
        arrayOfIntakeSpinMotorIds.add(intakeMotor.getDeviceID());
    }

    public static void addMultiplePivotMotors(ArrayList<TalonFX> listOfPivotMotors) {
        for (int i = 0; i < listOfPivotMotors.size(); i++) {

            arrayOfPivotMotors.add(listOfPivotMotors.get(i));
            arrayOfIntakePivotMotorIds.add(listOfPivotMotors.get(i).getDeviceID());
            arrayOfIntakePivotMotorNames.add(listOfPivotMotors.get(i).getDescription());

        }
    }

    public static void addMultipleSpinMotors(ArrayList<TalonFX> listOfSpinMotors) {
        for (int i = 0; i < listOfSpinMotors.size(); i++) {

            arrayOfIntakeSpinMotors.add(listOfSpinMotors.get(i));
            arrayOfIntakeSpinMotorIds.add(listOfSpinMotors.get(i).getDeviceID());
            arrayOfIntakeSpinMotorNames.add(listOfSpinMotors.get(i).getDescription());

        }
    }

    public static void addAPivotMotor(TalonFX pivotMotor) {
        arrayOfPivotMotors.add(pivotMotor);
        arrayOfIntakePivotMotorIds.add(pivotMotor.getDeviceID());
        arrayOfIntakePivotMotorNames.add(pivotMotor.getDescription());
    }

    public static TalonFX getAPivotMotor(int pivotMotorId) {
        for (int i = 0; i < arrayOfPivotMotors.size(); i++) {
            if (arrayOfPivotMotors.get(i).getDeviceID() == pivotMotorId) {
                return arrayOfPivotMotors.get(i);
            }
        }
        return null;
    }

    public static TalonFX getAPivotMotor(String name) {
        for (int i = 0; i < arrayOfPivotMotors.size(); i++) {
            if (arrayOfPivotMotors.get(i).getDescription() == name) {
                return arrayOfPivotMotors.get(i);
            }
        }
        return null;
    }

    public static TalonFX getASpinMotor(int spinMotorId) {
        for (int i = 0; i < arrayOfIntakeSpinMotors.size(); i++) {
            if (arrayOfIntakeSpinMotors.get(i).getDeviceID() == spinMotorId) {
                return arrayOfIntakeSpinMotors.get(i);
            }
        }
        return null;
    }

    public static TalonFX getASpinMotor(String name) {
        for (int i = 0; i < arrayOfIntakeSpinMotors.size(); i++) {
            if (arrayOfIntakeSpinMotors.get(i).getDescription() == name) {
                return arrayOfIntakeSpinMotors.get(i);
            }
        }
        return null;
    }

    public static ArrayList<TalonFX> getAllSpinMotors() {
        return arrayOfIntakeSpinMotors;
    }

    public static ArrayList<TalonFX> getAllPivotMotors() {
        return arrayOfPivotMotors;
    }

    public static void removePivotMotor(String name) {
        for (int i = 0; i < arrayOfPivotMotors.size(); i++) {
            if (arrayOfPivotMotors.get(i).getDescription() == name) {
                arrayOfPivotMotors.remove(i);
                arrayOfIntakePivotMotorIds.remove(i);
                arrayOfIntakePivotMotorIds.remove(i);
                break;
            }
        }
    }

    public static void removePivotMotor(int id) {
        for (int i = 0; i < arrayOfPivotMotors.size(); i++) {
            if (arrayOfPivotMotors.get(i).getDeviceID() == id) {
                arrayOfPivotMotors.remove(i);
                arrayOfIntakePivotMotorIds.remove(i);
                arrayOfIntakePivotMotorIds.remove(i);
            }
        }
    }

    public static void removeSpinMotor(int id) {
        for (int i = 0; i < arrayOfIntakeSpinMotors.size(); i++) {
            if (arrayOfIntakeSpinMotors.get(i).getDeviceID() == id) {
                arrayOfIntakeSpinMotorIds.remove(i);
                arrayOfIntakeSpinMotorNames.remove(i);
                arrayOfIntakeSpinMotors.remove(i);
            }
        }
    }

    public static void removeSpinMotor(String name) {
        for (int i = 0; i < arrayOfIntakeSpinMotors.size(); i++) {
            if (arrayOfIntakeSpinMotors.get(i).getDescription() == name) {
                arrayOfIntakeSpinMotorIds.remove(i);
                arrayOfIntakeSpinMotorNames.remove(i);
                arrayOfIntakeSpinMotors.remove(i);
                break;
            }
        }
    }

    public Intake(ArrayList<TalonFX> spinMotors, ArrayList<TalonFX> pivotMotors) {
        arrayOfIntakeSpinMotors = spinMotors;
        arrayOfPivotMotors = pivotMotors;
    }

    public Intake() {
        this(new ArrayList<TalonFX>(), new ArrayList<TalonFX>());
    }

}
