package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotation;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.utils.Trigger;
import frc.robot.Constants;

public class CoralRollerSubsystem implements Subsystem{
    private double SuckOutput = -0.8;
    private double SpitOutput = 1;
    private double StopOutput = 0;
    private double EndEffectorRollersStopVelocity = 0.1;

    TalonFX coralEndEffectorRollers = new TalonFX(Constants.CoralEndEffectorConstants.RollerId, Constants.CANivoreName);

    DCMotorSim rollerSim;

    DutyCycleOut manualControlRequest = new DutyCycleOut(0);

    public CoralRollerSubsystem() {
        if(RobotBase.isSimulation()) {
            DCMotor rollerMotors = DCMotor.getKrakenX60(1);
            rollerSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(rollerMotors, 0.1, 1), rollerMotors);
        }
    }

    @Override
    public void simulationPeriodic() {
        TalonFXSimState rollerSimState = coralEndEffectorRollers.getSimState();

        rollerSim.setInputVoltage(rollerSimState.getMotorVoltage());

        rollerSim.update(0.02);

        rollerSimState.setRawRotorPosition(rollerSim.getAngularPosition());
        rollerSimState.setRotorVelocity(rollerSim.getAngularVelocity());
    }

    private void manualDriveCoralEndEffectorRollers(double output) {
        coralEndEffectorRollers.setControl(manualControlRequest.withOutput(output));
    }

    public Command manualCoralEndEffectorCommand(DoubleSupplier CoralEndEffectorInput) {
        return run(()-> {
            manualDriveCoralEndEffectorRollers(CoralEndEffectorInput.getAsDouble());
        });
    }

    public Command suckCommand() {
        return run(()->{
            manualDriveCoralEndEffectorRollers(SuckOutput);
        });
    }

    public Command spitCommand() {
        return run(()->{
            manualDriveCoralEndEffectorRollers(SpitOutput);
        });
    }

    public Command suckUntilHaveCoralCommand() {
        return run(()->manualDriveCoralEndEffectorRollers(SuckOutput)).until(new Trigger(()->{
            return coralEndEffectorRollers.getVelocity().getValueAsDouble()<EndEffectorRollersStopVelocity;
        }).debounce(1, DebounceType.kRising).asRisingEdge()).andThen(
        run(()->manualDriveCoralEndEffectorRollers(StopOutput))); 
    }
}
