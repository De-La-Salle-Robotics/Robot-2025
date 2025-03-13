package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

public class ClimbSubsystem implements Subsystem {
    private double peckOutput = 0.3;
    private double climbOutput = -0.3;

    TalonFX climb;
    DCMotorSim climbSim;

    DutyCycleOut manualControlRequest = new DutyCycleOut(0);

    public ClimbSubsystem() {
        climb = new TalonFX(Constants.ClimbConstants.ClimbId, Constants.CANivoreName);
        if(RobotBase.isSimulation()) {
            DCMotor motors = DCMotor.getKrakenX60(1);
            climbSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(motors, 0.1, 400), motors);
        }
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
}