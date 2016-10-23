    Port of BetaFlight 3.0  Main PID Controller to C#
    Copyright (C) 2015  Bat Cave Games

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

public class BetaFlight : MonoBehaviour
{


    BetaFlight.pidProfile_s pidProfile = new BetaFlight.pidProfile_s();
    BetaFlight.rxConfig_t rxConfig = new BetaFlight.rxConfig_t();
    BetaFlight.rcControlsConfig_s rcControlsConfig = new BetaFlight.rcControlsConfig_s();

    BetaFlight.rollAndPitchTrims_t angleTrim = new BetaFlight.rollAndPitchTrims_t();
    BetaFlight.controlRateConfig_s controlRate = new BetaFlight.controlRateConfig_s();
    BetaFlight.escAndServoConfig_s escAndServoConfig = new BetaFlight.escAndServoConfig_s();




    /// <summary>
    /// 
    /// </summary>
    public void init()
    {

        resetPidProfile(pidProfile);
        resetRxConfig(rxConfig);
        resetRcControlsConfig(rcControlsConfig);
        resetControlRateConfig(controlRate);
        resetEscAndServoConfig(escAndServoConfig);

        for (int i=0; i<3; i++)
        {
            dtermFilterLpf[i] = new biquadFilter_t();
            dtermFilterNotch[i] = new biquadFilter_t();
            deltaFilter[i] = new pt1Filter_t();
        }

        generateThrottleCurve(controlRate, escAndServoConfig);

    }



        /// <summary>
        /// 
        /// </summary>
        /// <param name="pidProfile"></param>
        void initFilters(pidProfile_s pidProfile)
    {
        int axis;
        if (pidProfile.dterm_notch_hz != 0 && !dtermNotchInitialised) {
            float notchQ = filterGetNotchQ(pidProfile.dterm_notch_hz, pidProfile.dterm_notch_cutoff);
                for (axis = 0; axis < 3; axis++)
                {
                    biquadFilterInit(dtermFilterNotch[(int)axis], pidProfile.dterm_notch_hz, gyro.targetLooptime, notchQ, biquadFilterType_e.FILTER_NOTCH);
                }
        }

        if (pidProfile.dterm_filter_type == (uint)filterType_e.FILTER_BIQUAD) {
            if (pidProfile.dterm_lpf_hz != 0 && !dtermBiquadLpfInitialised) {
                for (axis = 0; axis < 3; axis++)
                {
                    biquadFilterInitLPF(dtermFilterLpf[(int)axis], pidProfile.dterm_lpf_hz, gyro.targetLooptime);
                }
            }
        }
    }

    /// <summary>
    /// 
    /// </summary>
    /// <param name="filter"></param>
    /// <param name="filterFreq"></param>
    /// <param name="refreshRate"></param>
    void biquadFilterInitLPF(biquadFilter_t filter, float filterFreq, uint refreshRate)
    {
        biquadFilterInit(filter, filterFreq, refreshRate, BIQUAD_Q, biquadFilterType_e.FILTER_LPF);
    }

    /// <summary>
    /// 
    /// </summary>
    /// <param name="centerFreq"></param>
    /// <param name="cutoff"></param>
    /// <returns></returns>
    float filterGetNotchQ(uint centerFreq, uint cutoff)
    {
        float octaves = Mathf.Log(((float)(centerFreq / (float)cutoff)), 2.0f) * 2.0f;
        return Mathf.Sqrt(Mathf.Pow(2, octaves)) / (Mathf.Pow(2, octaves) - 1);
    }
    
    /// <summary>
    /// 
    /// </summary>
    /// <param name="filter"></param>
    /// <param name="filterFreq"></param>
    /// <param name="refreshRate"></param>
    /// <param name="Q"></param>
    /// <param name="filterType"></param>
    void biquadFilterInit(biquadFilter_t filter, float filterFreq, uint refreshRate, float Q, biquadFilterType_e filterType)
    {
        // setup variables
        float sampleRate = 1 / ((float)refreshRate * 0.000001f);
        float omega = 2 * Mathf.PI * filterFreq / sampleRate;
        float sn = Mathf.Sin(omega);
        float cs = Mathf.Cos(omega);
        float alpha = sn / (2 * Q);

        float b0 = 0.0f, b1 = 0.0f, b2 = 0.0f, a0 = 0.0f, a1 = 0.0f, a2 = 0.0f;

        switch (filterType)
        {
            case biquadFilterType_e.FILTER_LPF:
                b0 = (1 - cs) / 2;
                b1 = 1 - cs;
                b2 = (1 - cs) / 2;
                a0 = 1 + alpha;
                a1 = -2 * cs;
                a2 = 1 - alpha;
                break;
            case biquadFilterType_e.FILTER_NOTCH:
                b0 = 1;
                b1 = -2 * cs;
                b2 = 1;
                a0 = 1 + alpha;
                a1 = -2 * cs;
                a2 = 1 - alpha;
                break;
        }

        // precompute the coefficients
        filter.b0 = b0 / a0;
        filter.b1 = b1 / a0;
        filter.b2 = b2 / a0;
        filter.a1 = a1 / a0;
        filter.a2 = a2 / a0;

        // zero initial samples
        filter.d1 = filter.d2 = 0;
    }

    /// <summary>
    /// 
    /// </summary>
    /// <param name="filter"></param>
    /// <param name="input"></param>
    /// <param name="f_cut"></param>
    /// <param name="dT"></param>
    /// <returns></returns>
    float pt1FilterApply4(pt1Filter_t filter, float input, uint f_cut, float dT)
    {
        // Pre calculate and store RC
        if (filter.RC == 0.0f)
        {
            filter.RC = 1.0f / (2.0f * Mathf.PI * f_cut);
            filter.dT = dT;
        }

        filter.state = filter.state + filter.dT / (filter.RC + filter.dT) * (input - filter.state);

        return filter.state;
    }
        
    /// <summary>
    /// Computes a biquad_t filter on a sample 
    /// </summary>
    /// <param name="filter"></param>
    /// <param name="input"></param>
    /// <returns></returns>
    float biquadFilterApply(biquadFilter_t filter, float input)
    {
        float result = filter.b0 * input + filter.d1;
        filter.d1 = filter.b1 * input - filter.a1 * result + filter.d2;
        filter.d2 = filter.b2 * input - filter.a2 * result;
        return result;
    }

    /// <summary>
    /// 
    /// </summary>
    /// <param name="mask"></param>
    /// <returns></returns>
    bool FLIGHT_MODE(flightModeFlags_e mask)
    {
            return (int)(flightModeFlags & mask) > 0 ? true : false; 
    }
    

    /// <summary>
    /// 
    /// </summary>
    /// <param name="axis"></param>
    /// <param name="midrc"></param>
    /// <returns></returns>
    int getRcStickDeflection(int axis, uint midrc)
    {
        return (int)Mathf.Min(Mathf.Abs(rcData[(int)axis] - midrc), 500);
    }

    /// <summary>
    /// 
    /// </summary>
    /// <returns></returns>
    float getdT()
    {
        return Time.fixedDeltaTime;
    }

    /// <summary>
    /// 
    /// </summary>
    /// <returns></returns>
    public bool isSuperExpoActive()
    {
        return true;
    }

    /// <summary>
    /// 
    /// </summary>
    /// <param name="pidProfile"></param>
    /// <param name="currentControlRateProfile"></param>
    /// <param name="axis"></param>
    /// <param name="rc"></param>
    /// <returns></returns>
    public float calculateSetpointRate(/* pidProfile_s pidProfile, controlRateConfig_s currentControlRateProfile, */ uint axis, int rc)
    {
        float angleRate;


        if (isSuperExpoActive())
        {
            rcInput[axis] = (axis == (int)rc_alias_e.YAW) ? (Mathf.Abs(rc) / (500.0f * (controlRate.rcYawRate8 / 100.0f))) : (Mathf.Abs(rc) / (500.0f * (controlRate.rcRate8 / 100.0f)));
            float rcFactor = 1.0f / (Mathf.Clamp(1.0f - (rcInput[axis] * (controlRate.rates[axis] / 100.0f)), 0.01f, 1.00f));

            angleRate = rcFactor * ((27 * rc) / 16.0f);
        }
        else {
            angleRate = (float)((controlRate.rates[axis] + 27) * rc) / 16.0f;
        }

        if (pidProfile.pidController == (int)pidControllerType_e.PID_CONTROLLER_LEGACY)
            return Mathf.Clamp(angleRate, -8190.0f, 8190.0f); // Rate limit protection
        else
            return Mathf.Clamp(angleRate / 4.1f, -1997.0f, 1997.0f); // Rate limit protection (deg/sec)
    }
         
    /// <summary>
    /// Betaflight pid controller, which will be maintained in the future with additional features specialised for current (mini) multirotor usage. Based on 2DOF reference design (matlab) 
    /// </summary>
    /// <param name="pidProfile"></param>
    /// <param name="max_angle_inclination"></param>
    /// <param name="angleTrim"></param>
    /// <param name="rxConfig"></param>
    public void pidBetaflight(/* pidProfile_s pidProfile, uint max_angle_inclination,
         rollAndPitchTrims_t angleTrim, rxConfig_t rxConfig */)
    {
        
        float errorRate = 0, rP = 0, rD = 0, PVRate = 0;
        float ITerm, PTerm, DTerm;
       
        float delta;
        int axis;
        float horizonLevelStrength = 1;

        float tpaFactor = PIDweight[(int)0] / 100.0f; // tpa is now float

        initFilters(pidProfile);

        if (FLIGHT_MODE(flightModeFlags_e.HORIZON_MODE)) {
          
            // Figure out the raw stick positions
        int stickPosAil = Mathf.Abs(getRcStickDeflection((int)flight_dynamics_index_t.FD_ROLL, rxConfig.midrc));
        int stickPosEle = Mathf.Abs(getRcStickDeflection((int)flight_dynamics_index_t.FD_PITCH, rxConfig.midrc));
        int mostDeflectedPos = Mathf.Max(stickPosAil, stickPosEle);
        // Progressively turn off the horizon self level strength as the stick is banged over
        horizonLevelStrength = (float)(500 - mostDeflectedPos) / 500;  // 1 at centre stick, 0 = max stick deflection
            if(pidProfile.D8[(int)(int)pidIndex_e.PIDLEVEL] == 0){
                horizonLevelStrength = 0;
            } else {
                horizonLevelStrength = Mathf.Clamp(((horizonLevelStrength - 1) * (100 / pidProfile.D8[(int)(int)pidIndex_e.PIDLEVEL])) + 1, 0, 1);
            }
        }

        // Yet Highly experimental and under test and development
        // Throttle coupled to Igain like inverted TPA // 50hz calculation (should cover all rx protocols)
        float kiThrottleGain = 1.0f;
        if (pidProfile.itermThrottleGain != 0) {
            uint maxLoopCount = 20000 / targetPidLooptime;
        
        float throttleItermGain = (float)pidProfile.itermThrottleGain * 0.001f;
            uint loopIncrement = 0;

            if (loopIncrement >= maxLoopCount) {
                kiThrottleGain = 1.0f + Mathf.Clamp((float)(Mathf.Abs(rcCommand[(int)(int)rc_alias_e.THROTTLE] - previousThrottle)) * throttleItermGain, 0.0f, 5.0f); // Limit to factor 5
                previousThrottle = rcCommand[(int)(int)rc_alias_e.THROTTLE];
                loopIncrement = 0;
            } else {
                loopIncrement++;
            }
        }

        // ----------PID controller----------
        for (axis = 0; axis< 3; axis++) {

           
            // static uint8_t configP[(int)3], configI[(int)3], configD[(int)3];

            // Prevent unnecessary computing and check for changed PIDs. No need for individual checks. Only pids is fine for now
            // Prepare all parameters for PID controller
            if ((pidProfile.P8[(int)axis] != configP[(int)axis]) || (pidProfile.I8[(int)axis] != configI[(int)axis]) || (pidProfile.D8[(int)axis] != configD[(int)axis])) {
                Kp[(int)axis] = PTERM_SCALE * pidProfile.P8[(int)axis];
                Ki[(int)axis] = ITERM_SCALE * pidProfile.I8[(int)axis];
                Kd[(int)axis] = DTERM_SCALE * pidProfile.D8[(int)axis];
                b[(int)axis] = pidProfile.ptermSetpointWeight / 100.0f;
                c[(int)axis] = pidProfile.dtermSetpointWeight / 100.0f;
                yawMaxVelocity = pidProfile.yawRateAccelLimit* 1000 * getdT();
                rollPitchMaxVelocity = pidProfile.rateAccelLimit* 1000 * getdT();

                configP[(int)axis] = pidProfile.P8[(int)axis];
                configI[(int)axis] = pidProfile.I8[(int)axis];
                configD[(int)axis] = pidProfile.D8[(int)axis];
            }

            // Limit abrupt yaw inputs / stops
            float maxVelocity = (axis == (int)rc_alias_e.YAW) ? yawMaxVelocity : rollPitchMaxVelocity;
            if (maxVelocity != 0.0f) {
                float currentVelocity = setpointRate[(int)axis] - previousSetpoint[(int)axis];
                if (Mathf.Abs(currentVelocity) > maxVelocity) {
                    setpointRate[(int)axis] = (currentVelocity > 0) ? previousSetpoint[(int)axis] + maxVelocity : previousSetpoint[(int)axis] - maxVelocity;
                }
                previousSetpoint[(int)axis] = setpointRate[(int)axis];
            }

            // Yaw control is GYRO based, direct sticks control is applied to rate PID
            if ((FLIGHT_MODE(flightModeFlags_e.ANGLE_MODE) || FLIGHT_MODE(flightModeFlags_e.HORIZON_MODE)) && axis != (int)rc_alias_e.YAW) {
                // calculate error angle and limit the angle to the max inclination
               

                float errorAngle = (Mathf.Clamp(2 * rcCommand[(int)axis], -((int)max_angle_inclination), +max_angle_inclination) - attitude.raw[(int)axis] + angleTrim.raw[(int)axis]) / 10.0f; // 16 bits is ok here
                if (FLIGHT_MODE(flightModeFlags_e.ANGLE_MODE)) {
                  
                    // ANGLE mode - control is angle based, so control loop is needed
                    setpointRate[(int)axis] = errorAngle* pidProfile.P8[(int)(int)pidIndex_e.PIDLEVEL] / 10.0f;
                } else {
                    // HORIZON mode - direct sticks control is applied to rate PID
                    // mix up angle error to desired AngleRate to add a little auto-level feel
                    setpointRate[(int)axis] = setpointRate[(int)axis] + (errorAngle* pidProfile.I8[(int)(int)pidIndex_e.PIDLEVEL] * horizonLevelStrength / 10.0f);
                   
                }
            }

            PVRate = gyroADCf[(int)axis]; // / 16.4f; // Process variable from gyro output in deg/sec

            // --------low-level gyro-based PID based on 2DOF PID controller. ----------
            //  ---------- 2-DOF PID controller with optional filter on derivative term. b = 1 and only c can be tuned (amount derivative on measurement or error).  ----------
            // Used in stand-alone mode for ACRO, controlled by higher level regulators in other modes
            // ----- calculate error / angle rates  ----------
            errorRate = setpointRate[(int)axis] - PVRate;       // r - y
            rP = b[(int)axis] * setpointRate[(int)axis] - PVRate;    // br - y

            // Slowly restore original setpoint with more stick input
            float diffRate = errorRate - rP;
    rP += diffRate * rcInput[(int)axis];

    // Reduce Hunting effect and jittering near setpoint. Limit multiple zero crossing within deadband and lower PID affect during low error amount
    float dynReduction = tpaFactor;
            if (pidProfile.toleranceBand != 0) {
                 float minReduction = (float)pidProfile.toleranceBandReduction / 100.0f;
  
                if (Mathf.Abs(errorRate) < pidProfile.toleranceBand) {
                    if (zeroCrossCount[(int)axis] != 0) {
                        if (currentErrorPolarity[(int)axis] == (uint)pidErrorPolarity_e.POSITIVE_ERROR) {
                            if (errorRate< 0 ) {
                                zeroCrossCount[(int)axis]--;
                                currentErrorPolarity[(int)axis] = (uint)pidErrorPolarity_e.NEGATIVE_ERROR;
                            }
                        } else {
                            if (errorRate > 0 ) {
                                zeroCrossCount[(int)axis]--;
                                currentErrorPolarity[(int)axis] = (uint)pidErrorPolarity_e.POSITIVE_ERROR;
                            }
                        }
                    } else {
                        dynReduction *= Mathf.Clamp(Mathf.Abs(errorRate) / pidProfile.toleranceBand, minReduction, 1.0f);
                    }
                } else {
                    zeroCrossCount[(int)axis] =  pidProfile.zeroCrossAllowanceCount;
                    currentErrorPolarity[(int)axis] = (errorRate > 0) ? (uint)pidErrorPolarity_e.POSITIVE_ERROR : (uint)pidErrorPolarity_e.NEGATIVE_ERROR;
                }
            }

            // -----calculate P component
            PTerm = Kp[(int)axis] * rP * dynReduction;

    // -----calculate I component.
    // Reduce strong Iterm accumulation during higher stick inputs
    float accumulationThreshold = (axis == (int)rc_alias_e.YAW) ? pidProfile.yawItermIgnoreRate : pidProfile.rollPitchItermIgnoreRate;
    float setpointRateScaler = Mathf.Clamp(1.0f - (1.5f * (Mathf.Abs(setpointRate[(int)axis]) / accumulationThreshold)), 0.0f, 1.0f);

    // Handle All windup Scenarios
    // limit maximum integrator value to prevent WindUp
    float itermScaler = setpointRateScaler * kiThrottleGain;

    errorGyroIf[(int)axis] = Mathf.Clamp(errorGyroIf[(int)axis] + Ki[(int)axis] * errorRate* getdT() * itermScaler, -250.0f, 250.0f);

            // I coefficient (I8) moved before integration to make limiting independent from PID settings
            ITerm = errorGyroIf[(int)axis];

            //-----calculate D-term (Yaw D not yet supported)
            if (axis == (int)rc_alias_e.YAW) {
                if (pidProfile.yaw_lpf_hz != 0)
                {
                    PTerm = pt1FilterApply4(yawFilter, PTerm, pidProfile.yaw_lpf_hz, getdT());
                }
                axisPID[(int)axis] = Mathf.RoundToInt(PTerm + ITerm);
                DTerm = 0.0f; // needed for blackbox

            } else {
                rD = c[(int)axis] * setpointRate[(int)axis] - PVRate;    // cr - y
                delta = rD - lastRateError[(int)axis];
                lastRateError[(int)axis] = rD;

                // Divide delta by targetLooptime to get differential (ie dr/dt)
                delta *= (1.0f / getdT());

                //  if (debugMode == DEBUG_DTERM_FILTER) debug[(int)axis] = Kd[(int)axis] * delta * dynReduction;

                // Filter delta
               
                if (dtermNotchInitialised) delta = biquadFilterApply(dtermFilterNotch[(int)axis], delta);

                if (pidProfile.dterm_lpf_hz != 0) {
                    if (dtermBiquadLpfInitialised) {
                        delta = biquadFilterApply(dtermFilterLpf[(int)axis], delta);
                    } else {
                        delta = pt1FilterApply4(deltaFilter[(int)axis], delta, pidProfile.dterm_lpf_hz, getdT());
                    }
                }
                
                DTerm = Kd[(int)axis] * delta * dynReduction;

                // -----calculate total PID output
                axisPID[(int)axis] = Mathf.Clamp(Mathf.RoundToInt(PTerm + ITerm + DTerm), -900, 900);
            }

            // Disable PID control at zero throttle
            if (!pidStabilisationEnabled) axisPID[(int)axis] = 0;

   /*
    #ifdef GTUNE
            if (FLIGHT_MODE(GTUNE_MODE) && ARMING_FLAG(ARMED)) {
                calculate_Gtune(axis);
            }
    #endif
    */
    /*
    #ifdef BLACKBOX
            axisPID_P[(int)axis] = PTerm;
            axisPID_I[(int)axis] = ITerm;
            axisPID_D[(int)axis] = DTerm;
    #endif
    */

        }
    }


    public enum rc_alias_e
    {
        ROLL = 0,
        PITCH,
        YAW,
        THROTTLE,
        AUX1,
        AUX2,
        AUX3,
        AUX4,
        AUX5,
        AUX6,
        AUX7,
        AUX8
    };

    public enum pIndx
    {
        PIDROLL,
        PIDPITCH,
        PIDYAW,
        PIDALT,
        PIDPOS,
        PIDPOSR,
        PIDNAVR,
        PIDLEVEL,
        PIDMAG,
        PIDVEL,
        PID_ITEM_COUNT
    };

     enum pidControllerType_e
     {
        PID_CONTROLLER_LEGACY = 0,           // Legacy PID controller. Old INT / Rewrite with 2.9 status. Fastest performance....least math. Will stay same in the future
        PID_CONTROLLER_BETAFLIGHT,           // Betaflight PID controller. Old luxfloat -> float evolution. More math added and maintained in the future
        PID_COUNT
    };

    public void resetControlRateConfig(controlRateConfig_s controlRateConfig)
    {
        controlRateConfig.rcRate8 = 100;
        controlRateConfig.rcYawRate8 = 100;
        controlRateConfig.rcExpo8 = 10;
        controlRateConfig.thrMid8 = 50;
        controlRateConfig.thrExpo8 = 0;
        controlRateConfig.dynThrPID = 20;
        controlRateConfig.rcYawExpo8 = 10;
        controlRateConfig.tpa_breakpoint = 1650;
        controlRateConfig.rates = new uint[FLIGHT_DYNAMICS_INDEX_COUNT];

        for (uint axis = 0; axis < FLIGHT_DYNAMICS_INDEX_COUNT; axis++)
        {
            controlRateConfig.rates[axis] = 70;
        }
    }

    public void resetRcControlsConfig(rxConfig_t rxConfig)
    {
        rxConfig.rssi_channel = 0;
        rxConfig.rssi_scale = 30; //RSSI_SCALE_DEFAULT;
        rxConfig.rssi_ppm_invert = 0;
        rxConfig.rcInterpolation = (uint)rcSmoothing_t.RC_SMOOTHING_AUTO;
        rxConfig.rcInterpolationInterval = 19;
        rxConfig.fpvCamAngleDegrees = 0;
        rxConfig.max_aux_channel = 6; // MAX_AUX_CHANNELS;
        rxConfig.airModeActivateThreshold = 1350;
    }

    public void resetRcControlsConfig(rcControlsConfig_s rcControlsConfig)
    {
        rcControlsConfig.deadband = 0;
        rcControlsConfig.yaw_deadband = 0;
        rcControlsConfig.alt_hold_deadband = 40;
        rcControlsConfig.alt_hold_fast_change = 1;
    }

    public void resetPidProfile(pidProfile_s pidProfile)
    {
        pidProfile.pidController = (int)pidControllerType_e.PID_CONTROLLER_BETAFLIGHT;

        pidProfile.P8[(int)rc_alias_e.ROLL] = 45;
        pidProfile.I8[(int)rc_alias_e.ROLL] = 40;
        pidProfile.D8[(int)rc_alias_e.ROLL] = 30;
        pidProfile.P8[(int)rc_alias_e.PITCH] = 60;
        pidProfile.I8[(int)rc_alias_e.PITCH] = 60;
        pidProfile.D8[(int)rc_alias_e.PITCH] = 25;
        pidProfile.P8[(int)rc_alias_e.YAW] = 80;
        pidProfile.I8[(int)rc_alias_e.YAW] = 45;
        pidProfile.D8[(int)rc_alias_e.YAW] = 20;
        pidProfile.P8[(int)pIndx.PIDALT] = 50;
        pidProfile.I8[(int)pIndx.PIDALT] = 0;
        pidProfile.D8[(int)pIndx.PIDALT] = 0;
        pidProfile.P8[(int)pIndx.PIDPOS] = 15;   // POSHOLD_P * 100;
        pidProfile.I8[(int)pIndx.PIDPOS] = 0;    // POSHOLD_I * 100;
        pidProfile.D8[(int)pIndx.PIDPOS] = 0;
        pidProfile.P8[(int)pIndx.PIDPOSR] = 34;  // POSHOLD_RATE_P * 10;
        pidProfile.I8[(int)pIndx.PIDPOSR] = 14;  // POSHOLD_RATE_I * 100;
        pidProfile.D8[(int)pIndx.PIDPOSR] = 53;  // POSHOLD_RATE_D * 1000;
        pidProfile.P8[(int)pIndx.PIDNAVR] = 25;  // NAV_P * 10;
        pidProfile.I8[(int)pIndx.PIDNAVR] = 33;  // NAV_I * 100;
        pidProfile.D8[(int)pIndx.PIDNAVR] = 83;  // NAV_D * 1000;
        pidProfile.P8[(int)pIndx.PIDLEVEL] = 50;
        pidProfile.I8[(int)pIndx.PIDLEVEL] = 50;
        pidProfile.D8[(int)pIndx.PIDLEVEL] = 100;
        pidProfile.P8[(int)pIndx.PIDMAG] = 40;
        pidProfile.P8[(int)pIndx.PIDVEL] = 55;
        pidProfile.I8[(int)pIndx.PIDVEL] = 55;
        pidProfile.D8[(int)pIndx.PIDVEL] = 75;

        pidProfile.yaw_p_limit = YAW_P_LIMIT_MAX;
        pidProfile.yaw_lpf_hz = 80;
        pidProfile.rollPitchItermIgnoreRate = 200;
        pidProfile.yawItermIgnoreRate = 50;
        pidProfile.dterm_filter_type = (uint)filterType_e.FILTER_BIQUAD;
        pidProfile.dterm_lpf_hz = 100;    // filtering ON by default
        pidProfile.dterm_notch_hz = 0;
        pidProfile.dterm_notch_cutoff = 150;
        pidProfile.deltaMethod = (int)pidDeltaType_e.DELTA_FROM_MEASUREMENT;
        pidProfile.vbatPidCompensation = 0;
        pidProfile.zeroThrottleStabilisation = (int)pidStabilisationState_e.PID_STABILISATION_OFF;

        // Betaflight PID controller parameters
        pidProfile.ptermSetpointWeight = 75;
        pidProfile.dtermSetpointWeight = 120;
        pidProfile.yawRateAccelLimit = 220;
        pidProfile.rateAccelLimit = 0;
        pidProfile.toleranceBand = 15;
        pidProfile.toleranceBandReduction = 40;
        pidProfile.zeroCrossAllowanceCount = 2;
        pidProfile.itermThrottleGain = 0;
/*
# ifdef GTUNE
        pidProfile.gtune_lolimP[(int)ROLL] = 10;          // [(int)0..200] Lower limit of ROLL P during G tune.
        pidProfile.gtune_lolimP[(int)PITCH] = 10;         // [(int)0..200] Lower limit of PITCH P during G tune.
        pidProfile.gtune_lolimP[(int)YAW] = 10;           // [(int)0..200] Lower limit of YAW P during G tune.
        pidProfile.gtune_hilimP[(int)ROLL] = 100;         // [(int)0..200] Higher limit of ROLL P during G tune. 0 Disables tuning for that axis.
        pidProfile.gtune_hilimP[(int)PITCH] = 100;        // [(int)0..200] Higher limit of PITCH P during G tune. 0 Disables tuning for that axis.
        pidProfile.gtune_hilimP[(int)YAW] = 100;          // [(int)0..200] Higher limit of YAW P during G tune. 0 Disables tuning for that axis.
        pidProfile.gtune_pwr = 0;                    // [(int)0..10] Strength of adjustment
        pidProfile.gtune_settle_time = 450;          // [(int)200..1000] Settle time in ms
        pidProfile.gtune_average_cycles = 16;        // [(int)8..128] Number of looptime cycles used for gyro average calculation
#endif
*/
    }

    public void resetEscAndServoConfig(escAndServoConfig_s escAndServoConfig)
    {
        escAndServoConfig.minthrottle = 1000;
        escAndServoConfig.maxthrottle = 2000;
        escAndServoConfig.mincommand = 1000;
        escAndServoConfig.servoCenterPulse = 1500;
    }

    public void generateThrottleCurve(controlRateConfig_s controlRateConfig, escAndServoConfig_s escAndServoConfig)
    {
        uint i;
        //  uint minThrottle = (feature(FEATURE_3D && IS_RC_MODE_ACTIVE(BOX3DDISABLESWITCH)) ? PWM_RANGE_MIN : escAndServoConfig->minthrottle);
        uint minThrottle = 1000;

        for (i = 0; i < THROTTLE_LOOKUP_LENGTH; i++)
        {
            int tmp = 10 * (int)i - (int)controlRateConfig.thrMid8;
            uint y = 1;
            if (tmp > 0)
                y = 100 - controlRateConfig.thrMid8;
            if (tmp < 0)
                y = controlRateConfig.thrMid8;
            lookupThrottleRC[i] = (int)(10 * controlRateConfig.thrMid8 + tmp * (100 - controlRateConfig.thrExpo8 + (int)controlRateConfig.thrExpo8 * (tmp * tmp) / (y * y)) / 10);
            lookupThrottleRC[i] = (int)(minThrottle + (int)(escAndServoConfig.maxthrottle - minThrottle) * lookupThrottleRC[i] / 1000); // [MINTHROTTLE;MAXTHROTTLE]
              
        }
    }

    int rcLookup(int tmp, uint expo, uint rate)
    {
        float tmpf = tmp / 100.0f;
        return (int)((2500.0f + (float)expo * (tmpf * tmpf - 25.0f)) * tmpf * (float)(rate) / 2500.0f);
    }

    int rcLookupThrottle(int tmp)
    {
        int tmp2 = tmp / 100;
        // [0;1000] -> expo -> [MINTHROTTLE;MAXTHROTTLE]
        return lookupThrottleRC[tmp2] + (tmp - tmp2 * 100) * (lookupThrottleRC[tmp2 + 1] - lookupThrottleRC[tmp2]) / 100;
    }

    public void resetRxConfig(rxConfig_t rxConfig)
    {

       rxConfig.sbus_inversion = 1;
       rxConfig.spektrum_sat_bind = 0;
       rxConfig.spektrum_sat_bind_autoreset = 1;
       rxConfig.midrc = 1500;
	    rxConfig.mincheck = 1000;
       rxConfig.maxcheck = 2000;
       rxConfig.rx_min_usec = 885;          // any of first 4 channels below this value will trigger rx loss detection
       rxConfig.rx_max_usec = 2115;
    }


    float throttleClosedTime = 0.0f;
  
    public void updateRcCommands(/*controlRateConfig_s controlRate, rxConfig_t rxConfig, rcControlsConfig_s rcControlsConfig */)
    {
        // PITCH & ROLL only dynamic PID adjustment,  depending on throttle value
        int prop;
        if (rcData[(int)rc_alias_e.THROTTLE] < controlRate.tpa_breakpoint)
        {
            prop = 100;
        }
        else {
            if (rcData[(int)rc_alias_e.THROTTLE] < 2000)
            {
                prop = 100 - (int)controlRate.dynThrPID * (rcData[(int)rc_alias_e.THROTTLE] - (int)controlRate.tpa_breakpoint) / (2000 - (int)controlRate.tpa_breakpoint);
            }
            else {
                prop = 100 - (int)controlRate.dynThrPID;
            }
        }

        for (int axis = 0; axis < 3; axis++)
        {
            // non coupled PID reduction scaler used in PID controller 1 and PID controller 2.
            PIDweight[axis] = (uint)prop;

            int tmp = Mathf.Min(Mathf.Abs(rcData[axis] - (int)rxConfig.midrc), 500);
            if (axis == (int)rc_alias_e.ROLL || axis == (int)rc_alias_e.PITCH)
            {
                if (tmp > rcControlsConfig.deadband)
                {
                    tmp -= (int)rcControlsConfig.deadband;
                }
                else {
                    tmp = 0;
                }
                rcCommand[axis] = rcLookup(tmp, controlRate.rcExpo8, controlRate.rcRate8);
            }
            else if (axis == (int)rc_alias_e.YAW)
            {
                if (tmp > rcControlsConfig.yaw_deadband)
                {
                    tmp -= (int)rcControlsConfig.yaw_deadband;
                }
                else {
                    tmp = 0;
                }
                rcCommand[axis] = rcLookup(tmp, controlRate.rcYawExpo8, controlRate.rcYawRate8) * -1 ;
            }
            if (rcData[axis] < rxConfig.midrc)
            {
                rcCommand[axis] = -rcCommand[axis];
            }
        }
       
        int temp = Mathf.Clamp(rcData[(int)rc_alias_e.THROTTLE], (int)rxConfig.mincheck, PWM_RANGE_MAX);
        temp = (int)(temp - rxConfig.mincheck) * PWM_RANGE_MIN / (PWM_RANGE_MAX - (int)rxConfig.mincheck);
        
        rcCommand[(int)rc_alias_e.THROTTLE] = rcLookupThrottle(temp);

       
        // experimental stop pid windup at low throttle for more than 3 seconds
        if(rcCommand[(int)rc_alias_e.THROTTLE] < 1100 
            && Mathf.Abs(rcCommand[(int)rc_alias_e.ROLL]) < 50
            && Mathf.Abs(rcCommand[(int)rc_alias_e.PITCH]) < 50
            && Mathf.Abs(rcCommand[(int)rc_alias_e.YAW]) < 50
            )
        {
            if(throttleClosedTime + 2.0f < Time.realtimeSinceStartup )
            {
                // set stab disabled
                pidStabilisationEnabled = false;
                pidResetErrorGyroState();
            }
        }
        else
        {
            throttleClosedTime = Time.realtimeSinceStartup;
            pidStabilisationEnabled = true;
        }

       
    }

    public void pidResetErrorGyroState()
    {
        for (int axis = 0; axis < 3; axis++)
        {
           // errorGyroI[axis] = 0;
            errorGyroIf[axis] = 0.0f;
        }
    }


   



    ///////////////////////////////////////////////////////////////////////////////////////////////////

    // PIDweight is a scale factor for PIDs which is derived from the throttle and TPA setting, and 100 = 100% scale means no PID reduction
    uint[] PIDweight = new uint[(int)3];
    bool dtermNotchInitialised, dtermBiquadLpfInitialised;
    flightModeFlags_e flightModeFlags; // = flightModeFlags_e.HORIZON_MODE;

    const int MAX_SUPPORTED_RC_CHANNEL_COUNT = 8;
    const int FLIGHT_DYNAMICS_INDEX_COUNT = 3;
    const int THROTTLE_LOOKUP_LENGTH = 12;
    const int PWM_RANGE_MIN = 1000;
    const int PWM_RANGE_MAX = 2000;
    const int PWM_RANGE_MIDDLE = 1500;

    int[] lookupThrottleRC = new int[THROTTLE_LOOKUP_LENGTH];    // lookup table for expo & mid THROTTLE


    public int[] rcData = new int[(int)MAX_SUPPORTED_RC_CHANNEL_COUNT]; // interval [(int)1000;2000]
    uint targetPidLooptime;
    int previousThrottle = 0;
    uint[] configP = new uint[(int)3];
    uint[] configI = new uint[(int)3];
    uint[] configD = new uint[(int)3];

    public int[] rcCommand = new int[(int)4];           // interval [(int)1000;2000] for THROTTLE and [(int)-500;+500] for ROLL/PITCH/YAW
    public float[] setpointRate = new float[(int)3];

    float[] lastRateError = new float[(int)2];
    float[] Kp = new float[(int)3];
    float[] Ki = new float[(int)3];
    float[] Kd = new float[(int)3];
    float[] b = new float[(int)3];
    float[] c = new float[(int)3];
    float[] previousSetpoint = new float[(int)3];
    float rollPitchMaxVelocity, yawMaxVelocity;
    public float[] gyroADCf = new float[(int)3];
    float[] rcInput = new float[(int)3];
    uint[] zeroCrossCount = new uint[(int)3];
    uint[] currentErrorPolarity = new uint[(int)3];
    float[] errorGyroIf = new float[(int)3];


    public int[] axisPID = new int[(int)3], axisPID_P = new int[(int)3], axisPID_I = new int[(int)3], axisPID_D = new int[(int)3];
    bool pidStabilisationEnabled = true;

    public const int GYRO_I_MAX = 256;         // Gyro I limiter
    public const int YAW_P_LIMIT_MIN = 100;              // Maximum value for yaw P limiter
    public const int YAW_P_LIMIT_MAX = 500;                 // Maximum value for yaw P limiter
    public const int YAW_JUMP_PREVENTION_LIMIT_LOW = 80;
    public const int YAW_JUMP_PREVENTION_LIMIT_HIGH = 400;

    public const int DYNAMIC_PTERM_STICK_THRESHOLD = 400;

    // Scaling factors for Pids for better tunable range in configurator for betaflight pid controller. The scaling is based on legacy pid controller or previous float
    public const float PTERM_SCALE = 0.032029f;
    public const float ITERM_SCALE = 0.244381f;
    public const float DTERM_SCALE = 0.000529f;

    public const int MAX_MAPPABLE_RX_INPUTS = 8;

    uint max_angle_inclination = 700; // 70 degrees

    class gyro_s
    {
        //sensorGyroInitFuncPtr init;                             // initialize function
        //sensorReadFuncPtr read;                                 // read 3 axis data function
        //sensorReadFuncPtr temperature;                          // read temperature if available
        //sensorInterruptFuncPtr intStatus;
        public float scale;                                            // scalefactor
        public uint targetLooptime;
    }
    gyro_s gyro = new gyro_s();

    /*** Filter stuff ***/

    const int DELTA_MAX_SAMPLES = 12;
    float BIQUAD_Q = 1.0f / Mathf.Sqrt(2.0f);     /* quality factor - butterworth*/

    class pt1Filter_t
    {
        public float state;
        public float RC;
        public float dT;
    };

    /* this holds the data required to update samples thru a filter */
    class biquadFilter_t
    {
        public float b0, b1, b2, a1, a2;
        public float d1, d2;
    };


    enum filterType_e
    {
        FILTER_PT1 = 0,
        FILTER_BIQUAD,
    };

    enum biquadFilterType_e
    {
        FILTER_LPF,
        FILTER_NOTCH
    };

    public class attitudeEulerAngles_t
    {
        public int[] raw = new int[(int)3];
    };

    public attitudeEulerAngles_t attitude = new attitudeEulerAngles_t(); // holds current attitude from the acc or mag ?
    pt1Filter_t[] deltaFilter = new pt1Filter_t[(int)3];
    pt1Filter_t yawFilter = new pt1Filter_t();
    biquadFilter_t[] dtermFilterLpf = new biquadFilter_t[(int)3];
    biquadFilter_t[] dtermFilterNotch = new biquadFilter_t[(int)3];

    public enum pidIndex_e
    {
        PIDROLL,
        PIDPITCH,
        PIDYAW,
        PIDALT,
        PIDPOS,
        PIDPOSR,
        PIDNAVR,
        PIDLEVEL,
        PIDMAG,
        PIDVEL,
        PID_ITEM_COUNT
    };
    /*
    public enum pidControllerType_e
    {
        PID_CONTROLLER_LEGACY = 0,           // Legacy PID controller. Old INT / Rewrite with 2.9 status. Fastest performance....least math. Will stay same in the future
        PID_CONTROLLER_BETAFLIGHT,           // Betaflight PID controller. Old luxfloat -> float evolution. More math added and maintained in the future
        PID_COUNT
    };
    */
    public enum pidDeltaType_e
    {
        DELTA_FROM_ERROR = 0,
        DELTA_FROM_MEASUREMENT
    };

    public enum pidSuperExpoYaw_e
    {
        SUPEREXPO_YAW_OFF = 0,
        SUPEREXPO_YAW_ON,
        SUPEREXPO_YAW_ALWAYS
    };

    public enum pidErrorPolarity_e
    {
        NEGATIVE_ERROR = 0,
        POSITIVE_ERROR
    };

    public enum pidStabilisationState_e
    {
        PID_STABILISATION_OFF = 0,
        PID_STABILISATION_ON
    };

    public enum flightModeFlags_e
    {
        ANGLE_MODE = (1 << 0),
        HORIZON_MODE = (1 << 1),
        MAG_MODE = (1 << 2),
        BARO_MODE = (1 << 3),
        GPS_HOME_MODE = (1 << 4),
        GPS_HOLD_MODE = (1 << 5),
        HEADFREE_MODE = (1 << 6),
        UNUSED_MODE = (1 << 7), // old autotune
        PASSTHRU_MODE = (1 << 8),
        SONAR_MODE = (1 << 9),
        FAILSAFE_MODE = (1 << 10),
        GTUNE_MODE = (1 << 11),
    };

    public enum flight_dynamics_index_t
    {
        FD_ROLL = 0,
        FD_PITCH,
        FD_YAW
    };
    /*
    public enum rc_alias_e
    {
        ROLL = 0,
        PITCH,
        YAW,
        THROTTLE,
        AUX1,
        AUX2,
        AUX3,
        AUX4,
        AUX5,
        AUX6,
        AUX7,
        AUX8
    };
    */
    public enum rcSmoothing_t
    {
        RC_SMOOTHING_OFF = 0,
        RC_SMOOTHING_DEFAULT,
        RC_SMOOTHING_AUTO,
        RC_SMOOTHING_MANUAL
    };

    public class controlRateConfig_s
    {
        public uint rcRate8;
        public uint rcYawRate8;
        public uint rcExpo8;
        public uint thrMid8;
        public uint thrExpo8;
        public uint[] rates;
        public uint dynThrPID;
        public uint rcYawExpo8;
        public uint tpa_breakpoint;                // Breakpoint where TPA is activated
    };

    public class pidProfile_s
    {
        public uint pidController;                  // 1 = rewrite betaflight evolved from http://www.multiwii.com/forum/viewtopic.php?f=8&t=3671, 2 = Betaflight PIDc (Evolved Luxfloat)

        public uint[] P8 = new uint[(int)pidIndex_e.PID_ITEM_COUNT];
        public uint[] I8 = new uint[(int)pidIndex_e.PID_ITEM_COUNT];
        public uint[] D8 = new uint[(int)pidIndex_e.PID_ITEM_COUNT];

        public uint dterm_filter_type;              // Filter selection for dterm
        public uint dterm_lpf_hz;                  // Delta Filter in hz
        public uint yaw_lpf_hz;                    // Additional yaw filter when yaw axis too noisy
        public uint dterm_notch_hz;                // Biquad dterm notch hz
        public uint dterm_notch_cutoff;            // Biquad dterm notch low cutoff
        public uint deltaMethod;                    // Alternative delta Calculation
        public uint rollPitchItermIgnoreRate;      // Experimental threshold for resetting iterm for pitch and roll on certain rates
        public uint yawItermIgnoreRate;            // Experimental threshold for resetting iterm for yaw on certain rates
        public uint yaw_p_limit;
        public uint dterm_average_count;            // Configurable delta count for dterm
        public uint vbatPidCompensation;            // Scale PIDsum to battery voltage
        public uint zeroThrottleStabilisation;      // Disable/Enable zero throttle stabilisation. Normally even without airmode P and D would be active.

        // Betaflight PID controller parameters
        public uint toleranceBand;                  // Error tolerance area where toleranceBandReduction is applied under certain circumstances
        public uint toleranceBandReduction;         // Lowest possible P and D reduction in percentage
        public uint zeroCrossAllowanceCount;        // Amount of bouncebacks within tolerance band allowed before reduction kicks in
        public uint itermThrottleGain;              // Throttle coupling to iterm. Quick throttle changes will bump iterm
        public uint ptermSetpointWeight;            // Setpoint weight for Pterm (lower means more PV tracking)
        public uint dtermSetpointWeight;            // Setpoint weight for Dterm (0= measurement, 1= full error, 1 > agressive derivative)
        public uint yawRateAccelLimit;             // yaw accel limiter for deg/sec/ms
        public uint rateAccelLimit;                // accel limiter roll/pitch deg/sec/ms
                                                   /*
                                                   # ifdef GTUNE
                                                           uint8_t gtune_lolimP[3];               // [0..200] Lower limit of P during G tune
                                                           uint8_t gtune_hilimP[3];               // [0..200] Higher limit of P during G tune. 0 Disables tuning for that axis.
                                                           uint8_t gtune_pwr;                     // [0..10] Strength of adjustment
                                                           uint16_t gtune_settle_time;             // [200..1000] Settle time in ms
                                                           uint8_t gtune_average_cycles;          // [8..128] Number of looptime cycles used for gyro average calculation
                                                   #endif
                                                   */
    };

    public class rollAndPitchTrims_t
    {
        public int[] raw = new int[2];
        //rollAndPitchTrims_t_def values;
    };

    public class rxConfig_t
    {
        public uint[] rcmap = new uint[MAX_MAPPABLE_RX_INPUTS];  // mapping of radio channels to internal RPYTA+ order
        public uint serialrx_provider;              // type of UART-based receiver (0 = spek 10, 1 = spek 11, 2 = sbus). Must be enabled by FEATURE_RX_SERIAL first.
        public uint sbus_inversion;                 // default sbus (Futaba, FrSKY) is inverted. Support for uninverted OpenLRS (and modified FrSKY) receivers.
        public uint spektrum_sat_bind;              // number of bind pulses for Spektrum satellite receivers
        public uint spektrum_sat_bind_autoreset;    // whenever we will reset (exit) binding mode after hard reboot
        public uint rssi_channel;
        public uint rssi_scale;
        public uint rssi_ppm_invert;
        public uint midrc;                         // Some radios have not a neutral point centered on 1500. can be changed here
        public uint mincheck;                      // minimum rc end
        public uint maxcheck;                      // maximum rc end
        public uint rcInterpolation;
        public uint rcInterpolationInterval;
        public uint fpvCamAngleDegrees;             // Camera angle to be scaled into rc commands
        public uint max_aux_channel;
        public uint airModeActivateThreshold;      // Throttle setpoint where airmode gets activated

        public uint rx_min_usec;
        public uint rx_max_usec;
        //rxFailsafeChannelConfiguration_t failsafe_channel_configurations[MAX_SUPPORTED_RC_CHANNEL_COUNT];

        //rxChannelRangeConfiguration_t channelRanges[NON_AUX_CHANNEL_COUNT];
    };

    public class escAndServoConfig_s
    {
        // PWM values, in milliseconds, common range is 1000-2000 (1 to 2ms)
        public uint minthrottle;                   // Set the minimum throttle command sent to the ESC (Electronic Speed Controller). This is the minimum value that allow motors to run at a idle speed.
        public uint maxthrottle;                   // This is the maximum value for the ESCs at full power this value can be increased up to 2000
        public uint mincommand;                    // This is the value for the ESCs when they are not armed. In some cases, this value must be lowered down to 900 for some specific ESCs
        public uint servoCenterPulse;              // This is the value for servos when they should be in the middle. e.g. 1500.
    };

    public class rcControlsConfig_s
    {
        public uint deadband;                       // introduce a deadband around the stick center for pitch and roll axis. Must be greater than zero.
        public uint yaw_deadband;                   // introduce a deadband around the stick center for yaw axis. Must be greater than zero.
        public uint alt_hold_deadband;              // defines the neutral zone of throttle stick during altitude hold, default setting is +/-40
        public uint alt_hold_fast_change;           // when disabled, turn off the althold when throttle stick is out of deadband defined with alt_hold_deadband; when enabled, altitude changes slowly proportional to stick movement
    };




}
