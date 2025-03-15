package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

public class ClimbSubsystem implements Subsystem {
    private double peckOutput = 0.3;
    private double climbOutput = -0.3;
    private double stopClimbOut = 0.5;

    TalonFX climb = new TalonFX(Constants.ClimbConstants.ClimbId, Constants.CANivoreName);
    DCMotorSim climbSim;

    Servo climbStop = new Servo(Constants.ClimbConstants.ServoClimbStop);

    DutyCycleOut manualControlRequest = new DutyCycleOut(0);

    public ClimbSubsystem() {
        if(RobotBase.isSimulation()) {
            DCMotor motors = DCMotor.getKrakenX60(1);
            climbSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(motors, 0.1, 400), motors);
        }

    }
    
    public void setBrakeMode() {
        climb.getConfigurator().apply(
            new TalonFXConfiguration().withMotorOutput(
                new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
        );
    }
    public void setCoastMode() {
        climb.getConfigurator().apply(
            new TalonFXConfiguration().withMotorOutput(
                new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast))
        );
    }

    @Override
    public void simulationPeriodic() {
        TalonFXSimState simState = climb.getSimState();

        climbSim.setInputVoltage(simState.getMotorVoltage());

        climbSim.update(0.02);

        simState.setRawRotorPosition(climbSim.getAngularPosition());
        simState.setRotorVelocity(climbSim.getAngularVelocity());
    }

    private void manualDriveClimb(double output) {
        climb.setControl(manualControlRequest.withOutput(output));
    }

    private void manualDriveTalons(double output) {
        climb.setControl(manualControlRequest.withOutput(output));
    }

    private void manualDriveServo(double output) {
        climbStop.set(output);
    }


    public Command manualStopClimbCommand(DoubleSupplier climbStopInput) {
        return run(()-> {
            manualDriveServo(climbStopInput.getAsDouble());
        });
    }

    public Command manualClimbCommand(DoubleSupplier climbInput) {
        return run(()-> {
            manualDriveTalons(climbInput.getAsDouble());
        });
    }

    public Command peckCommand() {
        return run(()->{
            manualDriveTalons(peckOutput);
        });
    }

    public Command climbCommand() {
        return run(()->{
            manualDriveTalons(climbOutput);
        });
    }

    public Command neutralOutputCommand() {
        return run(()->{
            manualDriveTalons(0);
        });
    }

    public Command stopClimbCommand() {
        return run(()->{
            manualDriveServo(stopClimbOut);
        });
    }
    public Command initStopServo() {
        return run(()->{
            manualDriveServo(0);
        });
    }
}