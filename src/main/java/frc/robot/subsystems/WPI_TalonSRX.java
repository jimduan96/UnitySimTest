/**
 * WPI Compliant motor controller class.
 * WPILIB's object model requires many interfaces to be implemented to use
 * the various features.
 * This includes...
 * - Software PID loops running in the robot controller
 * - LiveWindow/Test mode features
 * - Motor Safety (auto-turn off of motor if Set stops getting called)
 * - Single Parameter set that assumes a simple motor controller.
 */
package frc.robot.subsystems; // wpk updated to allow it to be compiled locally

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.WPI_MotorSafetyImplem;
import com.ctre.phoenix.platform.DeviceType;
import com.ctre.phoenix.platform.PlatformJNI;
import com.ctre.phoenix.Logger;
import com.ctre.phoenix.ErrorCode;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_CallbackHelper;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.HALValue;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.wpilibj.simulation.CallbackStore;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.hal.simulation.SimValueCallback;
import edu.wpi.first.hal.simulation.SimulatorJNI;

import java.util.ArrayList;

/**
 * CTRE Talon SRX Motor Controller when used on CAN Bus.
 */
public class WPI_TalonSRX extends TalonSRX implements SpeedController, Sendable {

    private String _description;
    private double _speed;

    private SimDevice m_simMotor;
    private SimDouble m_simPercOut;
    private SimDouble m_simSupplyCurrent;
    private SimDouble m_simMotorCurrent;
    private SimDouble m_simVbat;

    private SimDevice m_simAnalogEnc;
    private SimBoolean m_simAnalogInit;
    private SimDouble m_simAnalogVoltage;

    private SimDevice m_simPulseWidthEnc;
    private SimBoolean m_simPulseWidthConnected;
    private SimDouble m_simPulseWidthPos;

    private SimDevice m_simQuadEnc;
    private SimDouble m_simQuadPos;
    private SimDouble m_simQuadVel;

    // container to hold callback stores produced by sim value regsitrations. Arraylist used since
    // these values are not referenced but need to be held to ensure they are not garbage collected.
    private ArrayList<CallbackStore> m_callbackStores;

    /**
     * The default motor safety timeout IF calling application enables the feature.
     */
    public static final double kDefaultSafetyExpiration = 0.1;

    /**
     * Late-constructed motor safety, which ensures feature is off unless calling
     * applications explicitly enables it.
     */
    private WPI_MotorSafetyImplem _motorSafety = null;
    private final Object _lockMotorSaf = new Object();
    private double _motSafeExpiration = kDefaultSafetyExpiration;

    /**
     * Constructor for motor controller
     * 
     * @param deviceNumber device ID of motor controller
     */
    public WPI_TalonSRX(int deviceNumber) {
        super(deviceNumber);
        HAL.report(tResourceType.kResourceType_CTRE_future2, deviceNumber + 1);
        _description = "Talon SRX " + deviceNumber;

        SendableRegistry.addLW(this, "Talon SRX ", deviceNumber);

        // wpk removed to instantiate variable specific versions
        // SimValueCallback callback = new OnValueChangedCallback();
        m_callbackStores = new ArrayList<CallbackStore>();

        m_simMotor = SimDevice.create("Talon SRX", deviceNumber);
        if (m_simMotor != null) {
            m_simPercOut = m_simMotor.createDouble("percentOutput", Direction.kOutput, 0);

            m_simSupplyCurrent = m_simMotor.createDouble("supplyCurrent", Direction.kInput, 0);
            m_simMotorCurrent = m_simMotor.createDouble("motorCurrent", Direction.kInput, 0);
            m_simVbat = m_simMotor.createDouble("busVoltage", Direction.kInput, 12.0);

            SimDeviceSim sim = new SimDeviceSim("Talon SRX");
            m_callbackStores.add(sim.registerValueChangedCallback(m_simSupplyCurrent,
                    new OnValueChangedCallback("CurrentSupply"), true));
            m_callbackStores.add(sim.registerValueChangedCallback(m_simMotorCurrent,
                    new OnValueChangedCallback("CurrentMotor"), true));
            m_callbackStores
                    .add(sim.registerValueChangedCallback(m_simVbat, new OnValueChangedCallback("BusVoltage"), true));
        }

        String base = "Talon SRX[" + deviceNumber + "]/";
        m_simAnalogEnc = SimDevice.create(base + "Analog In");
        if (m_simAnalogEnc != null) {
            m_simAnalogInit = m_simAnalogEnc.createBoolean("init", Direction.kOutput, true);
            m_simAnalogVoltage = m_simAnalogEnc.createDouble("voltage", Direction.kInput, 0);
            SimDeviceSim sim = new SimDeviceSim(base + "Analog In");
            m_callbackStores.add(sim.registerValueChangedCallback(m_simAnalogVoltage,
                    new OnValueChangedCallback("AnalogPos"), true));
        }

        m_simPulseWidthEnc = SimDevice.create(base + "DutyCycleInput");
        if (m_simPulseWidthEnc != null) {
            m_simPulseWidthConnected = m_simPulseWidthEnc.createBoolean("connected", Direction.kInput, true);
            m_simPulseWidthPos = m_simPulseWidthEnc.createDouble("position", Direction.kInput, 0);

            SimDeviceSim sim = new SimDeviceSim(base + "DutyCycleInput");
            m_callbackStores.add(sim.registerValueChangedCallback(m_simPulseWidthConnected,
                    new OnValueChangedCallback("PulseWidthConnected"), true));
            m_callbackStores.add(sim.registerValueChangedCallback(m_simPulseWidthPos,
                    new OnValueChangedCallback("PulseWidthPos"), true));
        }

        m_simQuadEnc = SimDevice.create(base + "Encoder");
        if (m_simQuadEnc != null) {
            m_simQuadPos = m_simQuadEnc.createDouble("position", Direction.kInput, 0);
            m_simQuadVel = m_simQuadEnc.createDouble("velocity", Direction.kInput, 0);
            SimDeviceSim sim = new SimDeviceSim(base + "Encoder");
            m_callbackStores.add(
                    sim.registerValueChangedCallback(m_simQuadPos, new OnValueChangedCallback("QuadEncPos"), true));
            m_callbackStores.add(
                    sim.registerValueChangedCallback(m_simQuadVel, new OnValueChangedCallback("QuadEncVel"), true));
        }

        SimulatorJNI.registerSimPeriodicBeforeCallback(() -> {
            // double value = 0;
            // int err = 0;

            this.m_simPercOut.set(this.getMotorOutputPercent());

            // the below lines are commented out because they were overwriting values
            // received through WebSockets. This should have
            // no impact on values set by a PhysicsSim class as used in the CTRE examples

            // only process items that are output. Inputs are all done through calllbacks
            // value = PlatformJNI.JNI_SimGetPhysicsValue(DeviceType.TalonSRX.value,
            // this.getDeviceID(), "BusVoltage");
            // err = PlatformJNI.JNI_SimGetLastError(DeviceType.TalonSRX.value,
            // this.getDeviceID());
            // if (err == 0) {
            // this.m_simVbat.set(value);
            // }
            // value = PlatformJNI.JNI_SimGetPhysicsValue(DeviceType.TalonSRX.value,
            // this.getDeviceID(), "CurrentSupply");
            // err = PlatformJNI.JNI_SimGetLastError(DeviceType.TalonSRX.value,
            // this.getDeviceID());
            // if (err == 0) {
            // this.m_simSupplyCurrent.set(value);
            // }
            // value = PlatformJNI.JNI_SimGetPhysicsValue(DeviceType.TalonSRX.value,
            // this.getDeviceID(), "CurrentMotor");
            // err = PlatformJNI.JNI_SimGetLastError(DeviceType.TalonSRX.value,
            // this.getDeviceID());
            // if (err == 0) {
            // this.m_simMotorCurrent.set(value);
            // }
            // value = PlatformJNI.JNI_SimGetPhysicsValue(DeviceType.TalonSRX.value,
            // this.getDeviceID(), "AnalogPos");
            // err = PlatformJNI.JNI_SimGetLastError(DeviceType.TalonSRX.value,
            // this.getDeviceID());
            // if (err == 0) {
            // this.m_simAnalogVoltage.set(value * 3.3 / 1023);
            // }
            // value = PlatformJNI.JNI_SimGetPhysicsValue(DeviceType.TalonSRX.value,
            // this.getDeviceID(),
            // "PulseWidthConnecteed");
            // err = PlatformJNI.JNI_SimGetLastError(DeviceType.TalonSRX.value,
            // this.getDeviceID());
            // if (err == 0) {
            // this.m_simPulseWidthConnected.set(value != 0);
            // }
            // value = PlatformJNI.JNI_SimGetPhysicsValue(DeviceType.TalonSRX.value,
            // this.getDeviceID(), "PulseWidthPos");
            // err = PlatformJNI.JNI_SimGetLastError(DeviceType.TalonSRX.value,
            // this.getDeviceID());
            // if (err == 0) {
            // this.m_simPulseWidthPos.set(value);
            // }
            // value = PlatformJNI.JNI_SimGetPhysicsValue(DeviceType.TalonSRX.value,
            // this.getDeviceID(), "QuadEncPos");
            // err = PlatformJNI.JNI_SimGetLastError(DeviceType.TalonSRX.value,
            // this.getDeviceID());
            // if (err == 0) {
            // this.m_simQuadPos.set(value);
            // }

            // value = PlatformJNI.JNI_SimGetPhysicsValue(DeviceType.TalonSRX.value,
            // this.getDeviceID(), "QuadEncVel");
            // err = PlatformJNI.JNI_SimGetLastError(DeviceType.TalonSRX.value,
            // this.getDeviceID());
            // if (err == 0) {
            // this.m_simQuadVel.set(value);
            // }

        });
    }

    // This was modified to accept the name of the CTRE JNI_Sim data value name
    // which is different than the
    // one used in the Websocket interface. This class could be eliminated if lambda
    // functions are used when
    // registering the callbacks, but the code is more compact and readable by using
    // this.
    private class OnValueChangedCallback implements SimValueCallback {
        private String m_simVar;

        public OnValueChangedCallback(String simVarName) {
            m_simVar = simVarName;
        }

        @Override
        public void callback(String name, int handle, boolean readonly, HALValue value) {
            PlatformJNI.JNI_SimSetPhysicsInput(DeviceType.TalonSRX.value, getDeviceID(), m_simVar,
                    WPI_CallbackHelper.getRawValue(value));
        }
    }

    // ------ set/get routines for WPILIB interfaces ------//
    /**
     * Common interface for setting the speed of a simple speed controller.
     *
     * @param speed The speed to set. Value should be between -1.0 and 1.0. Value is
     *              also saved for Get().
     */
    @Override
    public void set(double speed) {
        _speed = speed;
        set(ControlMode.PercentOutput, _speed);
        feed();
    }

    /**
     * Special write for PID, same functionality as calling set
     * 
     * @param output Output to send to motor
     */
    @Override
    public void pidWrite(double output) {
        set(output);
    }

    /**
     * Common interface for getting the current set speed of a speed controller.
     *
     * @return The current set speed. Value is between -1.0 and 1.0.
     */
    @Override
    public double get() {
        return _speed;
    }

    // ---------Intercept CTRE calls for motor safety ---------//
    /**
     * Sets the appropriate output on the talon, depending on the mode.
     * 
     * @param mode  The output mode to apply. In PercentOutput, the output is
     *              between -1.0 and 1.0, with 0.0 as stopped. In Current mode,
     *              output value is in amperes. In Velocity mode, output value is in
     *              position change / 100ms. In Position mode, output value is in
     *              encoder ticks or an analog value, depending on the sensor. In
     *              Follower mode, the output value is the integer device ID of the
     *              talon to duplicate.
     *
     * @param value The setpoint value, as described above.
     *
     *
     *              Standard Driving Example:
     *              _talonLeft.set(ControlMode.PercentOutput, leftJoy);
     *              _talonRght.set(ControlMode.PercentOutput, rghtJoy);
     */
    public void set(ControlMode mode, double value) {
        /* intercept the advanced Set and feed motor-safety */
        super.set(mode, value);
        feed();
    }

    /**
     * @deprecated use 4 parameter set
     * @param mode    Sets the appropriate output on the talon, depending on the
     *                mode.
     * @param demand0 The output value to apply. such as advanced feed forward
     *                and/or auxiliary close-looping in firmware. In PercentOutput,
     *                the output is between -1.0 and 1.0, with 0.0 as stopped. In
     *                Current mode, output value is in amperes. In Velocity mode,
     *                output value is in position change / 100ms. In Position mode,
     *                output value is in encoder ticks or an analog value, depending
     *                on the sensor. See In Follower mode, the output value is the
     *                integer device ID of the talon to duplicate.
     *
     * @param demand1 Supplemental value. This will also be control mode specific
     *                for future features.
     */
    @Deprecated
    public void set(ControlMode mode, double demand0, double demand1) {
        /* intercept the advanced Set and feed motor-safety */
        super.set(mode, demand0, demand1);
        feed();
    }

    /**
     * @param mode        Sets the appropriate output on the talon, depending on the
     *                    mode.
     * @param demand0     The output value to apply. such as advanced feed forward
     *                    and/or auxiliary close-looping in firmware. In
     *                    PercentOutput, the output is between -1.0 and 1.0, with
     *                    0.0 as stopped. In Current mode, output value is in
     *                    amperes. In Velocity mode, output value is in position
     *                    change / 100ms. In Position mode, output value is in
     *                    encoder ticks or an analog value, depending on the sensor.
     *                    See In Follower mode, the output value is the integer
     *                    device ID of the talon to duplicate.
     *
     * @param demand1Type The demand type for demand1. Neutral: Ignore demand1 and
     *                    apply no change to the demand0 output. AuxPID: Use demand1
     *                    to set the target for the auxiliary PID 1.
     *                    ArbitraryFeedForward: Use demand1 as an arbitrary additive
     *                    value to the demand0 output. In PercentOutput the demand0
     *                    output is the motor output, and in closed-loop modes the
     *                    demand0 output is the output of PID0.
     * @param demand1     Supplmental output value. Units match the set mode.
     *
     *
     *                    Arcade Drive Example:
     *                    _talonLeft.set(ControlMode.PercentOutput, joyForward,
     *                    DemandType.ArbitraryFeedForward, +joyTurn);
     *                    _talonRght.set(ControlMode.PercentOutput, joyForward,
     *                    DemandType.ArbitraryFeedForward, -joyTurn);
     *
     *                    Drive Straight Example: Note: Selected Sensor
     *                    Configuration is necessary for both PID0 and PID1.
     *                    _talonLeft.follow(_talonRght, FollwerType.AuxOutput1);
     *                    _talonRght.set(ControlMode.PercentOutput, joyForward,
     *                    DemandType.AuxPID, desiredRobotHeading);
     *
     *                    Drive Straight to a Distance Example: Note: Other
     *                    configurations (sensor selection, PID gains, etc.) need to
     *                    be set. _talonLeft.follow(_talonRght,
     *                    FollwerType.AuxOutput1);
     *                    _talonRght.set(ControlMode.MotionMagic, targetDistance,
     *                    DemandType.AuxPID, desiredRobotHeading);
     */
    public void set(ControlMode mode, double demand0, DemandType demand1Type, double demand1) {
        /* intercept the advanced Set and feed motor-safety */
        super.set(mode, demand0, demand1Type, demand1);
        feed();
    }

    /**
     * Sets the voltage output of the SpeedController. Compensates for the current
     * bus voltage to ensure that the desired voltage is output even if the battery
     * voltage is below 12V - highly useful when the voltage outputs are
     * "meaningful" (e.g. they come from a feedforward calculation).
     *
     * <p>
     * NOTE: This function *must* be called regularly in order for voltage
     * compensation to work properly - unlike the ordinary set function, it is not
     * "set it and forget it."
     *
     * @param outputVolts The voltage to output.
     */
    @Override
    public void setVoltage(double outputVolts) {
        if (super.isVoltageCompensationEnabled()) {
            com.ctre.phoenix.Logger.log(ErrorCode.DoubleVoltageCompensatingWPI, _description + ": setVoltage ");
        }
        set(outputVolts / RobotController.getBatteryVoltage());
    }

    // ----------------------- Invert routines -------------------//
    /**
     * Common interface for inverting direction of a speed controller.
     *
     * @param isInverted The state of inversion, true is inverted.
     */
    @Override
    public void setInverted(boolean isInverted) {
        super.setInverted(isInverted);
    }

    /**
     * Common interface for returning the inversion state of a speed controller.
     *
     * @return The state of inversion, true is inverted.
     */
    @Override
    public boolean getInverted() {
        return super.getInverted();
    }

    // ----------------------- turn-motor-off routines-------------------//
    /**
     * Common interface for disabling a motor.
     */
    @Override
    public void disable() {
        neutralOutput();
    }

    /**
     * Common interface to stop the motor until Set is called again.
     */
    @Override
    public void stopMotor() {
        neutralOutput();
    }

    // ---- essentially a copy of SendableBase -------//
    private String m_name = "";
    private String m_subsystem = "Ungrouped";

    /**
     * Free the resources used by this object.
     */
    public void free() {
        SendableRegistry.remove(this);
    }

    /**
     * @return name of object
     */
    @Override
    public final synchronized String getName() {
        return m_name;
    }

    /**
     * Sets the name of the object
     * 
     * @param name name of object
     */
    @Override
    public final synchronized void setName(String name) {
        m_name = name;
    }

    /**
     * Sets the name of the sensor with a channel number.
     *
     * @param moduleType A string that defines the module name in the label for the
     *                   value
     * @param channel    The channel number the device is plugged into
     */
    public final void setName(String moduleType, int channel) {
        setName(moduleType + "[" + channel + "]");
    }

    /**
     * Sets the name of the sensor with a module and channel number.
     *
     * @param moduleType   A string that defines the module name in the label for
     *                     the value
     * @param moduleNumber The number of the particular module type
     * @param channel      The channel number the device is plugged into (usually
     *                     PWM)
     */
    public final void setName(String moduleType, int moduleNumber, int channel) {
        setName(moduleType + "[" + moduleNumber + "," + channel + "]");
    }

    /**
     * @return subsystem name of this object
     */
    @Override
    public final synchronized String getSubsystem() {
        return m_subsystem;
    }

    /**
     * Sets the subsystem name of this object
     * 
     * @param subsystem
     */
    @Override
    public final synchronized void setSubsystem(String subsystem) {
        m_subsystem = subsystem;
    }

    /**
     * Add a child component.
     *
     * @param child child component
     */
    public final void addChild(Object child) {
        SendableRegistry.addChild(this, child);
    }

    /**
     * Initialize sendable
     * 
     * @param builder Base sendable to build on
     */
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Speed Controller");
        builder.setSafeState(this::stopMotor);
        builder.addDoubleProperty("Value", this::get, this::set);
    }

    /**
     * @return description of controller
     */
    public String getDescription() {
        return _description;
    }

    /* ----- Motor Safety ----- */
    /** caller must lock appropriately */
    private WPI_MotorSafetyImplem GetMotorSafety() {
        if (_motorSafety == null) {
            /* newly created MS object */
            _motorSafety = new WPI_MotorSafetyImplem(this, getDescription());
            /* apply the expiration timeout */
            _motorSafety.setExpiration(_motSafeExpiration);
        }
        return _motorSafety;
    }

    /**
     * Feed the motor safety object.
     *
     * <p>
     * Resets the timer on this object that is used to do the timeouts.
     */
    public void feed() {
        synchronized (_lockMotorSaf) {
            if (_motorSafety == null) {
                /* do nothing, MS features were never enabled */
            } else {
                GetMotorSafety().feed();
            }
        }
    }

    /**
     * Set the expiration time for the corresponding motor safety object.
     *
     * @param expirationTime The timeout value in seconds.
     */
    public void setExpiration(double expirationTime) {
        synchronized (_lockMotorSaf) {
            /* save the value for if/when we do create the MS object */
            _motSafeExpiration = expirationTime;
            /* apply it only if MS object exists */
            if (_motorSafety == null) {
                /* do nothing, MS features were never enabled */
            } else {
                /* this call will trigger creating a registered MS object */
                GetMotorSafety().setExpiration(_motSafeExpiration);
            }
        }
    }

    /**
     * Retrieve the timeout value for the corresponding motor safety object.
     *
     * @return the timeout value in seconds.
     */
    public double getExpiration() {
        synchronized (_lockMotorSaf) {
            /* return the intended expiration time */
            return _motSafeExpiration;
        }
    }

    /**
     * Determine of the motor is still operating or has timed out.
     *
     * @return a true value if the motor is still operating normally and hasn't
     *         timed out.
     */
    public boolean isAlive() {
        synchronized (_lockMotorSaf) {
            if (_motorSafety == null) {
                /* MC is alive - MS features were never enabled to neutral the MC. */
                return true;
            } else {
                return GetMotorSafety().isAlive();
            }
        }
    }

    /**
     * Enable/disable motor safety for this device.
     *
     * <p>
     * Turn on and off the motor safety option for this PWM object.
     *
     * @param enabled True if motor safety is enforced for this object
     */
    public void setSafetyEnabled(boolean enabled) {
        synchronized (_lockMotorSaf) {
            if (_motorSafety == null && !enabled) {
                /*
                 * Caller wants to disable MS, but MS features were nevere enabled, so it
                 * doesn't need to be disabled.
                 */
            } else {
                /* MS will be created if it does not exist */
                GetMotorSafety().setSafetyEnabled(enabled);
            }
        }
    }

    /**
     * Return the state of the motor safety enabled flag.
     *
     * <p>
     * Return if the motor safety is currently enabled for this device.
     *
     * @return True if motor safety is enforced for this device
     */
    public boolean isSafetyEnabled() {
        synchronized (_lockMotorSaf) {
            if (_motorSafety == null) {
                /* MS features were never enabled. */
                return false;
            } else {
                return GetMotorSafety().isSafetyEnabled();
            }
        }
    }

    private void timeoutFunc() {
        DriverStation ds = DriverStation.getInstance();
        if (ds.isDisabled() || ds.isTest()) {
            return;
        }

        DriverStation.reportError(getDescription() + "... Output not updated often enough.", false);

        stopMotor();
    }
}
