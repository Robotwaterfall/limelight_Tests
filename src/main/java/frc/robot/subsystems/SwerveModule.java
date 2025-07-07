package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;


public class SwerveModule {

    private final SparkMax driveMotor;
    private final SparkMax turningMotor;

    private final SparkMaxConfig driveMotorConfig = new SparkMaxConfig();
    private final SparkMaxConfig turningMotorConfig = new SparkMaxConfig();

    private final PIDController turningPidController;

    private final CANcoder absoluteEncoder;

    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
    int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed){

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new CANcoder(absoluteEncoderId);

        driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new SparkMax(turningMotorId, MotorType.kBrushless);

        driveMotorConfig.inverted(driveMotorReversed);
        turningMotorConfig.inverted(turningMotorReversed);

        driveMotorConfig.encoder.positionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveMotorConfig.encoder.velocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);

        turningMotorConfig.encoder.positionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        turningMotorConfig.encoder.velocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        turningPidController = new PIDController(ModuleConstants.kTurningControllerPValue, 0, 0);

        /**Tells the PID that the system is circular so that it doesnt have to go all
         *  the way around but it can simply reverse saving time
         **/
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        driveMotor.configure(driveMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        turningMotor.configure(turningMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);


    }

    public double getDrivePosition(){
        return driveMotor.getEncoder().getPosition();
    }

    public double getTurningPosition(){
        return turningMotor.getEncoder().getPosition();
    }

    public double getDriveVelocity(){
        return driveMotor.getEncoder().getVelocity();
    }

    public double getTurningVelocity(){
        return turningMotor.getEncoder().getVelocity();
    }


    public double getAbsoluteEncoderRad(){
        double angle = (absoluteEncoder.getAbsolutePosition().getValueAsDouble());
        
        angle *= 2.0 * Math.PI; 
        angle -= absoluteEncoderOffsetRad;

        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders(){
        driveMotor.getEncoder().setPosition(0);
        turningMotor.getEncoder().setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

     public SwerveModulePosition getSwerveModulePosition(){
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state){

        if(Math.abs (state.speedMetersPerSecond) < 0.001){

            stop();
            return;
        }




        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getDeviceID() + "] state", state.toString());
    }

    public void stop(){
        driveMotor.set(0);
        turningMotor.set(0);
    }


    
}
