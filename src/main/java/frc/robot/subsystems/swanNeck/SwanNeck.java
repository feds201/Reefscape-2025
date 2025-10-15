// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swanNeck;

import java.util.Map;
import java.util.function.DoubleSupplier;

import org.dyn4j.geometry.Rotation;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.RobotMap;
import frc.robot.constants.RobotMap.CurrentLimiter;
import frc.robot.constants.RobotMap.IntakeMap;
import frc.robot.utils.SubsystemABS;
import frc.robot.utils.Subsystems;
 
public class SwanNeck extends SubsystemABS {
    /** Creates a new gooseNeck. */
   
    private TalonFX pivotMotor;
    private CANrange coralCanRange;
    // private CANcoder gooseNeckAngler;
    private DoubleSupplier coralCANrangeVal = ()-> 0.0;
    private DoubleSupplier swanNeckAngleValue = ()-> 0.0;
    public DoubleSupplier m_swanNeckPivotSpeed;
   
    private PIDController pid;
    private PIDController algaePid;
  

    public SwanNeck(Subsystems subsystem, String name) {
        super(subsystem, name);
       
        pivotMotor = new TalonFX(IntakeMap.SensorCanId.PIVOT_MOTOR);
            pivotMotor.getConfigurator().apply(CurrentLimiter.getCurrentLimitConfiguration(IntakeMap.PIVOT_MOTOR_CURRENT_LIMIT));
        coralCanRange = new CANrange(IntakeMap.SensorCanId.CORAL_CANRANGE);
        pid = IntakeMap.intakePid;
        algaePid = new PIDController(6.5, 0, 0);
        algaePid.setTolerance(.007);
        pid.setTolerance(.007);
    
        // gooseNeckAngler = new CANcoder(IntakeMap.SensorCanId.INTAKE_ENCODER);
        SmartDashboard.putNumber("L4 Positition", RobotMap.ElevatorMap.L4ROTATION);

        pivotMotor.getConfigurator().apply(IntakeMap.getBreakConfigurationGooseNeck());
        coralCANrangeVal = () -> coralCanRange.getDistance().getValueAsDouble();
        swanNeckAngleValue = () -> pivotMotor.getPosition().getValueAsDouble();

        tab.addNumber("coral CanRange Value", coralCANrangeVal);
        tab.addNumber("gooseNeck Angle", swanNeckAngleValue);
        tab.add("GooseNeck PID", pid).withWidget(BuiltInWidgets.kPIDController);

           GenericEntry swanNeckPivotSpeedSetter = tab.add("Swan Neck Pivot Speed", 0.0)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0, "max", .2))
                .getEntry();
                m_swanNeckPivotSpeed = () -> swanNeckPivotSpeedSetter.getDouble(0);

           
    }

    @Override
    public void periodic() {
        coralCANrangeVal = () -> coralCanRange.getDistance().getValueAsDouble();
        swanNeckAngleValue = () -> pivotMotor.getPosition().getValueAsDouble();
        SmartDashboard.putNumber("gooseneck pivot value", pivotMotor.getPosition().getValueAsDouble());
    }

    @Override
    public void simulationPeriodic() {
    }

    @Override
    public void setDefaultCmd() {

    }

    @Override
    public boolean isHealthy() {
        return true;

    }

    @Override
    public void Failsafe() {
      
        pivotMotor.disable();

    }

    public void setPivotSpeed(double speed) {
        pivotMotor.set(speed);
    }

    public void setPIDTarget(double target) {
        pid.setSetpoint(target);
    }

    public boolean pidAtSetpoint() {
        return pid.atSetpoint();
    }
//.355
//
    public void rotateSwanNeckPID() {
        double pidOutput = pid.calculate(getPivotPosition());
        if(pidOutput >= 0){
            pidOutput += IntakeMap.ks;
        } else {
            pidOutput -= IntakeMap.ks;
        }
        double output = pidOutput + (IntakeMap.kg * Math.cos((getPivotPosition() - .223) * 2 * Math.PI));
        
        setPivotSpeed(output);
    }

    public void setAlgaePIDTarget(double target) {
        algaePid.setSetpoint(target);
    }

    public boolean algaePidAtSetpoint() {
        return algaePid.atSetpoint();
    }

    public void rotateAlgaePID() {
        double pidOutput = pid.calculate(getPivotPosition());
        if(pidOutput >= 0){
            pidOutput += IntakeMap.ksAlgae;
        } else {
            pidOutput -= IntakeMap.ksAlgae;
        }
        double output = pidOutput - (IntakeMap.kgAlgae * Math.cos((getPivotPosition() - .223) * 2 * Math.PI));
        
        setPivotSpeed(output);
    }

 

    public void resetGooseNeckEncoder() {
        pivotMotor.setPosition(0);
    }

    public boolean getCoralLoaded(){
        return coralCanRange.getDistance().getValueAsDouble() <= IntakeMap.CANRANGETHRESHOLDVALUE;
    }

    public boolean getCoralLoadedOpposite(){
        return !(coralCanRange.getDistance().getValueAsDouble() <= IntakeMap.CANRANGETHRESHOLDVALUE);
    }

    public void zeroPivotPosition(){
        pivotMotor.setPosition(0);
    }

    public boolean positionGreaterThan(double pos){
        return getPivotPosition() > pos;
    }

    public double getPivotPosition() {
        return pivotMotor.getPosition().getValueAsDouble();
    }



    public void lockPivot() {
        pivotMotor.getConfigurator().apply(IntakeMap.getBreakConfigurationGooseNeck());
    }

    public void unlockPivot() {
        pivotMotor.getConfigurator().apply(IntakeMap.getCoastConfiguration());
    }

}
