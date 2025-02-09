// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swanNeck;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.RobotMap.CurrentLimiter;
import frc.robot.constants.RobotMap.IntakeMap;
import frc.robot.utils.SubsystemABS;
import frc.robot.utils.Subsystems;
 
public class SwanNeck extends SubsystemABS {
    /** Creates a new gooseNeck. */
    private TalonFX intakeMotor;
    private TalonFX pivotMotor;
    private CANrange coralCanRange;
    private CANrange algaeCanRange;
    private CANcoder gooseNeckAngler;
    private DoubleSupplier algaeCANrangeVal;
    private DoubleSupplier coralCANrangeVal;
    private DoubleSupplier swanNeckCANCoderValue;

    private Mechanism2d mech;
    private MechanismRoot2d root;
    private MechanismLigament2d pivotLigament;
    private MechanismLigament2d intakeLigament;
    
  

    public SwanNeck(Subsystems subsystem, String name) {
        super(subsystem, name);
        intakeMotor = new TalonFX(IntakeMap.SensorCanId.INTAKE_MOTOR);
            intakeMotor.getConfigurator().apply(CurrentLimiter.getCurrentLimitConfiguration(IntakeMap.INTAKE_MOTOR_CURRENT_LIMIT));
        pivotMotor = new TalonFX(IntakeMap.SensorCanId.PIVOT_MOTOR);
            pivotMotor.getConfigurator().apply(CurrentLimiter.getCurrentLimitConfiguration(IntakeMap.PIVOT_MOTOR_CURRENT_LIMIT));
        coralCanRange = new CANrange(IntakeMap.SensorCanId.CORAL_CANRANGE);
        algaeCanRange = new CANrange(IntakeMap.SensorCanId.ALGAE_CANRANGE);
        gooseNeckAngler = new CANcoder(IntakeMap.SensorCanId.INTAKE_ENCODER);
        

        pivotMotor.getConfigurator().apply(IntakeMap.getBreakConfiguration());

        algaeCANrangeVal = () -> algaeCanRange.getDistance().getValueAsDouble();
        coralCANrangeVal = () -> coralCanRange.getDistance().getValueAsDouble();
        swanNeckCANCoderValue = () -> gooseNeckAngler.getPosition().getValueAsDouble();
        tab.addNumber("algaeCanRange", algaeCANrangeVal);
        tab.addNumber("coralCanRange", coralCANrangeVal);
        tab.addNumber("gooseNeckAngler", swanNeckCANCoderValue);


        // change the pose3d to 45degree angle
        mech = new Mechanism2d(1,2);
        root = mech.getRoot("root", .5, .5);
        pivotLigament = new MechanismLigament2d("pivot", 0.5, 90);
        root.append(pivotLigament);
        SmartDashboard.putData("mech", mech);
    }

    @Override
    public void periodic() {

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
        intakeMotor.disable();
        pivotMotor.disable();

    }

    public void runPivotMotor(double speed) {
        pivotMotor.set(speed);
    }

    public double getPivotAngle() {
        return swanNeckCANCoderValue.getAsDouble();
    }

    public void lockPivot() {
        pivotMotor.getConfigurator().apply(IntakeMap.getBreakConfiguration());
    }

    public void unlockPivot() {
        pivotMotor.getConfigurator().apply(IntakeMap.getCoastConfiguration());
    }

}
