/*
    This file is part of Repetier-Firmware.

    Repetier-Firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Repetier-Firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Repetier-Firmware.  If not, see <http://www.gnu.org/licenses/>.

    This firmware is a nearly complete rewrite of the sprinter firmware
    by kliment (https://github.com/kliment/Sprinter)
    which based on Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.
*/

#include "Repetier.h"

uint8_t manageMonitor = 0; ///< Temp. we want to monitor with our host. 1+NUM_EXTRUDER is heated bed
unsigned int counterPeriodical = 0;
volatile uint8_t executePeriodical = 0;
uint8_t counter500ms = 5;

#if ANALOG_INPUTS > 0
const uint8 osAnalogInputChannels[] PROGMEM = ANALOG_INPUT_CHANNELS;
volatile uint osAnalogInputValues[ANALOG_INPUTS];
#endif

#ifdef USE_GENERIC_THERMISTORTABLE_1
short temptable_generic1[GENERIC_THERM_NUM_ENTRIES][2];
#endif
#ifdef USE_GENERIC_THERMISTORTABLE_2
short temptable_generic2[GENERIC_THERM_NUM_ENTRIES][2];
#endif
#ifdef USE_GENERIC_THERMISTORTABLE_3
short temptable_generic3[GENERIC_THERM_NUM_ENTRIES][2];
#endif
/** Makes updates to temperatures and heater state every call.

Is called every 100ms.
*/
static uint8_t extruderTempErrors = 0;
static uint8_t extrSecondFlag = 0;
void Extruder::manageTemperatures() {
    extrSecondFlag++;
    if(extrSecondFlag == 10)
        extrSecondFlag = 0;
    Com::writeToAll = true;
#if FEATURE_WATCHDOG
    HAL::pingWatchdog();
#endif // FEATURE_WATCHDOG
    uint8_t errorDetected = 0;

    bool newDefectFound = false;
    millis_t time = HAL::timeInMilliseconds(); // compare time for decouple tests
#if NUM_TEMPERATURE_LOOPS > 0
    for(uint8_t controller = 0; controller < NUM_TEMPERATURE_LOOPS; controller++) {
        TemperatureController *act = tempController[controller];
        // Get Temperature
        act->updateCurrentTemperature();
        // Handle automatic cooling of extruders
        if(controller < NUM_EXTRUDER) {
            if(act->currentTemperatureC < EXTRUDER_FAN_COOL_TEMP && act->targetTemperatureC < EXTRUDER_FAN_COOL_TEMP)
                extruder[controller].coolerPWM = 0;
            else
                extruder[controller].coolerPWM = extruder[controller].coolerSpeed;
        } // extruder controller
        // do skip temperature control while auto tuning is in progress
        if(controller == autotuneIndex) continue;

        if(controller == autotuneIndex)  // Ignore heater we are currently testing
            continue;

        // Check for obvious sensor errors
        if((act->currentTemperatureC < MIN_DEFECT_TEMPERATURE || act->currentTemperatureC > MAX_DEFECT_TEMPERATURE) &&
                act->targetTemperatureC > 0 /*is heating*/ &&
                (act->preheatTime() == 0 || act->preheatTime() >= MILLISECONDS_PREHEAT_TIME /*preheating time is over*/)) { // no temp sensor or short in sensor, disable heater
            errorDetected = 1;
            if(extruderTempErrors < 10)    // Ignore short temporary failures
                extruderTempErrors++;
            else {
                act->flags |= TEMPERATURE_CONTROLLER_FLAG_SENSDEFECT;
                if(!Printer::isAnyTempsensorDefect()) {
                    newDefectFound = true;
                    Printer::setAnyTempsensorDefect();
                    reportTempsensorError();
                    uid.showMessage(2);
                }
            }
        }

        else if(controller == HEATED_BED_INDEX && Extruder::getHeatedBedTemperature() > HEATED_BED_MAX_TEMP + 5) {
            errorDetected = 1;
            if(extruderTempErrors < 10)    // Ignore short temporary failures
                extruderTempErrors++;
            else {
                act->flags |= TEMPERATURE_CONTROLLER_FLAG_SENSDEFECT;
                Com::printErrorFLN(PSTR("Heated bed exceeded max temperature!"));
                if(!Printer::isAnyTempsensorDefect()) {
                    newDefectFound = true;
                    Printer::setAnyTempsensorDefect();
                    reportTempsensorError();
                    uid.showMessage(2);
                }
            }
        }

        if(Printer::isAnyTempsensorDefect()) continue;
        uint8_t on = act->currentTemperatureC >= act->targetTemperatureC ? LOW : HIGH;
        // Make a sound if alarm was set on reaching target temperature
        if(!on && act->isAlarm()) {
            //beep(50 * (controller + 1), 3);
            act->setAlarm(false);  //reset alarm
        }

        // Run test if heater and sensor are decoupled
        bool decoupleTestRequired = !errorDetected && act->decoupleTestPeriod > 0 && (time - act->lastDecoupleTest) > act->decoupleTestPeriod; // time enough for temperature change?
        if(decoupleTestRequired && act->isDecoupleFullOrHold() && Printer::isPowerOn()) { // Only test when powered
            if(act->isDecoupleFull()) { // Phase 1: Heating fully until target range is reached
                if(act->currentTemperatureC - act->lastDecoupleTemp < DECOUPLING_TEST_MIN_TEMP_RISE) { // failed test
                    extruderTempErrors++;
                    errorDetected = 1;
                    if(extruderTempErrors > 10) { // Ignore short temporary failures
                        act->flags |= TEMPERATURE_CONTROLLER_FLAG_SENSDECOUPLED;

                        if(!Printer::isAnyTempsensorDefect()) {
                            Printer::setAnyTempsensorDefect();
                            newDefectFound = true;
                            uid.showMessage(3);
                        }
                        UI_ERROR_P(PSTR("Heater decoupled"));
                        Com::printErrorFLN(PSTR("One heater seems decoupled from thermistor - disabling all for safety!"));
                        Com::printF(PSTR("Error:Temp. raised to slow. Rise = "), act->currentTemperatureC - act->lastDecoupleTemp);
                        Com::printF(PSTR(" after "), (int32_t)(time - act->lastDecoupleTest));
                        Com::printFLN(PSTR(" ms"));
                    }
                } else {
                    act->stopDecouple();
                    act->startFullDecouple(time);
                }
            } else { // Phase 2: Holding temperature inside a target corridor
                if(fabs(act->currentTemperatureC - act->targetTemperatureC) > DECOUPLING_TEST_MAX_HOLD_VARIANCE) { // failed test
                    extruderTempErrors++;
                    errorDetected = 1;
                    if(extruderTempErrors > 10) { // Ignore short temporary failures
                        act->flags |= TEMPERATURE_CONTROLLER_FLAG_SENSDECOUPLED;
                        if(!Printer::isAnyTempsensorDefect()) {
                            Printer::setAnyTempsensorDefect();
                            newDefectFound = true;
                            uid.showMessage(3);
                        }
                        UI_ERROR_P(PSTR("Heater decoupled"));
                        Com::printErrorFLN(PSTR("One heater seems decoupled from thermistor - disabling all for safety!"));
                        Com::printF(PSTR("Error:Could not hold temperature "), act->lastDecoupleTemp);
                        Com::printF(PSTR(" measured "), act->currentTemperatureC);
                        Com::printFLN(PSTR(" deg. C"));
                    }
                } else {
                    act->lastDecoupleTest = time - act->decoupleTestPeriod + 1000; // once running test every second
                }
            }
        }

        uint8_t output = 0;
        float error = act->targetTemperatureC - act->currentTemperatureC;
        if(act->targetTemperatureC < 20.0f) { // heating is off
            output = 0; // off is off, even if damping term wants a heat peak!
            act->stopDecouple();
        } else if(error > PID_CONTROL_RANGE) { // Phase 1: full heating until control range reached
            output = act->pidMax;
            act->startFullDecouple(time);
            act->tempIState = act->tempIStateLimitMin;
            if(act->heatManager == HTR_DEADTIME) {
                act->tempIStateLimitMax = act->pidDriveMax;
                act->tempIStateLimitMin = 0;
            }
        } else if(error < -PID_CONTROL_RANGE) // control range left upper side!
            output = 0;
        else { // control range handle by heat manager
            if(act->heatManager == HTR_PID) {
                act->startHoldDecouple(time);
                // Com::printF(PSTR(" CUR:"),act->currentTemperatureC); Com::printFLN(PSTR(" IST:"),(act->pidIGain * act->tempIState * 0.1),1);
                float pidTerm = act->pidPGain * error;
                act->tempIState = constrain(act->tempIState + error, act->tempIStateLimitMin, act->tempIStateLimitMax);
                pidTerm += act->pidIGain * act->tempIState * 0.1; // 0.1 = 10Hz
                // float dgain = act->pidDGain * (act->tempArray[act->tempPointer] - act->currentTemperatureC) * 3.333f;
                float dgain = act->pidDGain * (act->lastTemperatureC - act->temperatureC);
                pidTerm += dgain;
#if SCALE_PID_TO_MAX == 1
                pidTerm = (pidTerm * act->pidMax) * 0.0039215;
#endif // SCALE_PID_TO_MAX
                output = constrain((int)pidTerm, 0, act->pidMax);
            } else if(act->heatManager == HTR_DEADTIME) { // dead-time control
                act->startHoldDecouple(time);
                // output = (act->currentTemperatureC + act->tempIState * act->deadTime > act->targetTemperatureC ? 0 : act->pidDriveMax);
                float raising = (act->temperatureC - act->lastTemperatureC); // raising dT/dt from 0.5 seconds
                // act->tempIState = 0.25 * (3.0 * act->tempIState + raising); // damp raising
#ifndef SKIP_DEADTIME_ADJUSTMENT
                if(raising < 0 && act->tempIState >= 0) { // peak reached
                    if(error < -0.5)
                        act->tempIStateLimitMax = constrain(act->tempIStateLimitMax - 10, act->tempIStateLimitMin, act->pidDriveMax);
                    else
                        act->tempIStateLimitMax = constrain(act->tempIStateLimitMax + 10, act->tempIStateLimitMin, act->pidDriveMax);
                    // Com::printF(PSTR("Raise:"), raising);Com::printF(PSTR(" er:"),error,2);Com::printFLN(PSTR(" LimitMax:"),act->tempIStateLimitMax,0);
                } else if(raising > 0 && act->tempIState <= 0) { // bottom reached
                    if(error > 0.5)
                        act->tempIStateLimitMin = constrain(act->tempIStateLimitMin + 10, 0, act->tempIStateLimitMax - 20);
                    else
                        act->tempIStateLimitMin = constrain(act->tempIStateLimitMin - 10, 0, act->tempIStateLimitMax - 20);
                    // Com::printFLN(PSTR("LimitMin:"),act->tempIStateLimitMin,0);
                }
                // Com::printFLN(PSTR("Raise:"), raising);
#endif
                output = static_cast<uint8_t>(act->currentTemperatureC + raising * act->deadTime > act->targetTemperatureC ? act->tempIStateLimitMin : act->tempIStateLimitMax /* pidDriveMax */);
                act->tempIState = raising;
            } else // bang bang and slow bang bang
                if(act->heatManager == HTR_SLOWBANG) {  // Bang-bang with reduced change frequency to save relays life
                    if (time - act->lastTemperatureUpdate > HEATED_BED_SET_INTERVAL) {
                        output = (on ? act->pidMax : 0);
                        act->lastTemperatureUpdate = time;
                        if(on) act->startFullDecouple(time);
                        else act->stopDecouple();
                    } else continue;
                } else if(act->heatManager == HTR_OFF) { // Fast Bang-Bang fall back
                    output = (on ? act->pidMax : 0);
                    if(on) act->startFullDecouple(time);
                    else act->stopDecouple();
                }
        } // Temperature control
#ifdef MAXTEMP
        if(act->currentTemperatureC > MAXTEMP) // Force heater off if MAXTEMP is exceeded
            output = 0;
#endif // MAXTEMP
        pwm_pos[act->pwmIndex] = output; // set pwm signal
        if(extrSecondFlag == 0 /*|| (act->heatManager == HTR_DEADTIME && extrSecondFlag == 5)*/) {
            act->lastTemperatureC = act->temperatureC;
            act->temperatureC = act->currentTemperatureC;
        }

#if LED_PIN > -1
        if(act == &Extruder::current->tempControl)
            WRITE(LED_PIN, on);
#endif // LED_PIN
    } // for controller

    if(errorDetected == 0 && extruderTempErrors > 0)
        extruderTempErrors--;
    if(newDefectFound) {
        Com::printFLN(PSTR("Disabling all heaters due to detected sensor defect."));
        for(uint8_t i = 0; i < NUM_TEMPERATURE_LOOPS; i++) {
            tempController[i]->targetTemperatureC = 0;
            pwm_pos[tempController[i]->pwmIndex] = 0;
        }
#if defined(KILL_IF_SENSOR_DEFECT) && KILL_IF_SENSOR_DEFECT > 0
        if(!Printer::debugDryrun() && PrintLine::linesCount > 0) {  // kill printer if actually printing
			Printer::stopPrint();
            Printer::kill(false);
        }
#endif // KILL_IF_SENSOR_DEFECT
        Printer::debugSet(8); // Go into dry mode
        GCode::fatalError(PSTR("Heater/sensor error"));
    } // any sensor defect
#endif // NUM_TEMPERATURE_LOOPS

    // Report temperatures every second, so we do not need to send M105
    if(Printer::isAutoreportTemp()) {
        millis_t now = HAL::timeInMilliseconds();
        if(now - Printer::lastTempReport > 1000) {
            Printer::lastTempReport = now;
            Commands::printTemperatures();
        }
    }
}

void TemperatureController::waitForTargetTemperature() {
    if(targetTemperatureC < 30) return;
    if(Printer::debugDryrun()) return;
    bool oldReport = Printer::isAutoreportTemp();
    Printer::setAutoreportTemp(true);
    //millis_t time = HAL::timeInMilliseconds();
    while(true) {
        /*if( (HAL::timeInMilliseconds() - time) > 1000 )   //Print Temp Reading every 1 second while heating up.
        {
            Commands::printTemperatures();
            time = HAL::timeInMilliseconds();
        }*/
        Commands::checkForPeriodicalActions(true);
        GCode::keepAlive(WaitHeater);
        if(fabs(targetTemperatureC - currentTemperatureC) <= 1) {
            Printer::setAutoreportTemp(oldReport);
            return;
        }
    }
}

fast8_t TemperatureController::errorState() {
    if(isSensorDefect())
        return 1;
    if(isSensorDecoupled())
        return 2;
    return 0;
}
/* For pausing we negate target temperature, so negative value means paused extruder.
Since temp. is negative no heating will occur. */
void Extruder::pauseExtruders(bool bed) {

    disableAllExtruderMotors();
    for(fast8_t i = 0; i < NUM_EXTRUDER; i++) {
        if(extruder[i].tempControl.targetTemperatureC > 0) {
            extruder[i].tempControl.targetTemperatureC = -fabs(extruder[i].tempControl.targetTemperatureC);
            pwm_pos[extruder[i].tempControl.pwmIndex] = 0;
        }
    }

    if(bed) {
        heatedBedController.targetTemperatureC = -fabs(heatedBedController.targetTemperatureC);
        pwm_pos[heatedBedController.pwmIndex] = 0;
    }
}

void Extruder::unpauseExtruders(bool wait) {
    // activate temperatures
    for(fast8_t i = 0; i < NUM_EXTRUDER; i++) {
        if(extruder[i].tempControl.targetTemperatureC < 0)
            extruder[i].tempControl.targetTemperatureC = -extruder[i].tempControl.targetTemperatureC;
    }
    bool waitBed = false;
    if(heatedBedController.targetTemperatureC < 0) {
        heatedBedController.targetTemperatureC = -heatedBedController.targetTemperatureC;
        waitBed = true;
    }
    if(wait) {
        for(fast8_t i = 0; i < NUM_EXTRUDER; i++)
            extruder[i].tempControl.waitForTargetTemperature();
        if(waitBed) {
            heatedBedController.waitForTargetTemperature();
        }
    }
}

void TemperatureController::resetAllErrorStates() {
#if NUM_TEMPERATURE_LOOPS > 0
    for(int i = 0; i < NUM_TEMPERATURE_LOOPS; i++) {
        tempController[i]->removeErrorStates();
    }
#endif
    Printer::unsetAnyTempsensorDefect();
}

void Extruder::initHeatedBed() {
    heatedBedController.updateTempControlVars();
}

#if defined(USE_GENERIC_THERMISTORTABLE_1) || defined(USE_GENERIC_THERMISTORTABLE_2) || defined(USE_GENERIC_THERMISTORTABLE_3)
void createGenericTable(short table[GENERIC_THERM_NUM_ENTRIES][2], short minTemp, short maxTemp, float beta, float r0, float t0, float r1, float r2) {
    t0 += 273.15f;
    float rs, vs;
    if(r1 == 0) {
        rs = r2;
        vs = GENERIC_THERM_VREF;
    } else {
        vs = static_cast<float>((GENERIC_THERM_VREF * r1) / (r1 + r2));
        rs = (r2 * r1) / (r1 + r2);
    }
    float k = r0 * exp(-beta / t0);
    float delta = (maxTemp - minTemp) / (GENERIC_THERM_NUM_ENTRIES - 1.0f);
    for(uint8_t i = 0; i < GENERIC_THERM_NUM_ENTRIES; i++) {
#if FEATURE_WATCHDOG
        HAL::pingWatchdog();
#endif // FEATURE_WATCHDOG
        float t = maxTemp - i * delta;
        float r = exp(beta / (t + 272.65)) * k;
        float v = 4092 * r * vs / ((rs + r) * GENERIC_THERM_VREF);
        int adc = static_cast<int>(v);
        t *= 8;
        if(adc > 4092) adc = 4092;
        table[i][0] = (adc >> (ANALOG_REDUCE_BITS));
        table[i][1] = static_cast<int>(t);
#ifdef DEBUG_GENERIC
        Com::printF(PSTR("GenTemp:"), table[i][0]);
        Com::printFLN(PSTR(","), table[i][1]);
#endif
    }
}
#endif

/** \brief Initializes all extruder.

Updates the pin configuration needed for the extruder and activates extruder 0.
Starts a interrupt based analog input reader, which is used by simple thermistors
for temperature reading.
*/
void Extruder::initExtruder() {
    uint8_t i;
    Extruder::current = &extruder[0];
#ifdef USE_GENERIC_THERMISTORTABLE_1
    createGenericTable(temptable_generic1, GENERIC_THERM1_MIN_TEMP, GENERIC_THERM1_MAX_TEMP, GENERIC_THERM1_BETA, GENERIC_THERM1_R0, GENERIC_THERM1_T0, GENERIC_THERM1_R1, GENERIC_THERM1_R2);
#endif
#ifdef USE_GENERIC_THERMISTORTABLE_2
    createGenericTable(temptable_generic2, GENERIC_THERM2_MIN_TEMP, GENERIC_THERM2_MAX_TEMP, GENERIC_THERM2_BETA, GENERIC_THERM2_R0, GENERIC_THERM2_T0, GENERIC_THERM2_R1, GENERIC_THERM2_R2);
#endif
#ifdef USE_GENERIC_THERMISTORTABLE_3
    createGenericTable(temptable_generic3, GENERIC_THERM3_MIN_TEMP, GENERIC_THERM3_MAX_TEMP, GENERIC_THERM3_BETA, GENERIC_THERM3_R0, GENERIC_THERM3_T0, GENERIC_THERM3_R1, GENERIC_THERM3_R2);
#endif

#if defined(EXT0_STEP_PIN) && EXT0_STEP_PIN > -1
    SET_OUTPUT(EXT0_DIR_PIN);
    SET_OUTPUT(EXT0_STEP_PIN);
#if defined(EXT0_MIRROR_STEPPER) && EXT0_MIRROR_STEPPER
    SET_OUTPUT(EXT0_DIR2_PIN);
    SET_OUTPUT(EXT0_STEP2_PIN);
    SET_OUTPUT(EXT0_ENABLE2_PIN);
    WRITE(EXT0_ENABLE2_PIN, !EXT0_ENABLE_ON);
#endif
#endif

    for(i = 0; i < NUM_EXTRUDER; ++i) {
        Extruder *act = &extruder[i];
        if(act->enablePin > -1) {
            HAL::pinMode(act->enablePin, OUTPUT);
            HAL::digitalWrite(act->enablePin, !act->enableOn);
        }
        act->tempControl.lastTemperatureUpdate = HAL::timeInMilliseconds();
    }

    SET_OUTPUT(HEATED_BED_HEATER_PIN);
    WRITE(HEATED_BED_HEATER_PIN, HEATER_PINS_INVERTED);
    Extruder::initHeatedBed();
#if ANALOG_INPUTS > 0
    HAL::analogStart();
#endif
}

void TemperatureController::updateTempControlVars() {
    if(heatManager == HTR_PID && pidIGain != 0) { // prevent division by zero
        tempIStateLimitMax = (float)pidDriveMax * 10.0f / pidIGain;
        tempIStateLimitMin = (float)pidDriveMin * 10.0f / pidIGain;
    }
}

/** \brief Select extruder ext_num.

This function changes and initializes a new extruder. This is also called, after the eeprom values are changed.
*/
void Extruder::selectExtruderById(uint8_t extruderId) {
    float cx, cy, cz;
    Printer::realPosition(cx, cy, cz);

    Commands::waitUntilEndOfAllMoves();
    if(extruderId >= NUM_EXTRUDER)
        extruderId = 0;
    Extruder *current = extruder->current;
    Extruder *next = &extruder[extruderId];
    bool executeSelect = extruderId != current->id;


    Com::printFLN(PSTR("SelectExtruder:"), static_cast<int>(extruderId));

    float oldfeedrate = Printer::feedrate;
    current->extrudePosition = Printer::currentPositionSteps[E_AXIS];

    if(Printer::isHomedAll() && next->zOffset < current->zOffset) { // prevent extruder from hitting bed - move bed down a bit
        Printer::offsetZ = -next->zOffset * Printer::invAxisStepsPerMM[Z_AXIS];
        Printer::setNoDestinationCheck(true);
        Printer::moveToReal(IGNORE_COORDINATE, IGNORE_COORDINATE, IGNORE_COORDINATE, IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
        Printer::setNoDestinationCheck(false);
        Commands::waitUntilEndOfAllMoves();
        Printer::updateCurrentPosition(true);
    }


    Extruder::current = next;
    // --------------------- Now new extruder is active --------------------
#ifdef SEPERATE_EXTRUDER_POSITIONS
    // Use separate extruder positions only if being told. Slic3r e.g. creates a continuous extruder position increment
    Printer::currentPositionSteps[E_AXIS] = Extruder::current->extrudePosition;
#endif
    Printer::destinationSteps[E_AXIS] = Printer::currentPositionSteps[E_AXIS];
    Printer::axisStepsPerMM[E_AXIS] = Extruder::current->stepsPerMM;
    Printer::invAxisStepsPerMM[E_AXIS] = 1.0f / Printer::axisStepsPerMM[E_AXIS];
    Printer::maxFeedrate[E_AXIS] = Extruder::current->maxFeedrate;
//   max_start_speed_units_per_second[E_AXIS] = Extruder::current->maxStartFeedrate;
    Printer::maxAccelerationMMPerSquareSecond[E_AXIS] = Printer::maxTravelAccelerationMMPerSquareSecond[E_AXIS] = next->maxAcceleration;
    Printer::maxTravelAccelerationStepsPerSquareSecond[E_AXIS] =
        Printer::maxPrintAccelerationStepsPerSquareSecond[E_AXIS] = Printer::maxAccelerationMMPerSquareSecond[E_AXIS] * Printer::axisStepsPerMM[E_AXIS];
#if USE_ADVANCE
    Printer::maxExtruderSpeed = (ufast8_t)floor(HAL::maxExtruderTimerFrequency() / (Extruder::current->maxFeedrate * next->stepsPerMM));
    if(Printer::maxExtruderSpeed > 15) Printer::maxExtruderSpeed = 15;
    float fmax = ((float)HAL::maxExtruderTimerFrequency() / ((float)Printer::maxExtruderSpeed * Printer::axisStepsPerMM[E_AXIS])); // Limit feedrate to interrupt speed
    if(fmax < Printer::maxFeedrate[E_AXIS]) Printer::maxFeedrate[E_AXIS] = fmax;
#endif // USE_ADVANCE
    Extruder::current->tempControl.updateTempControlVars();
    Printer::offsetX = -next->xOffset * Printer::invAxisStepsPerMM[X_AXIS];
    Printer::offsetY = -next->yOffset * Printer::invAxisStepsPerMM[Y_AXIS];
    Printer::offsetZ = -next->zOffset * Printer::invAxisStepsPerMM[Z_AXIS];
    Commands::changeFlowrateMultiply(Printer::extrudeMultiply); // needed to adjust extrusionFactor to possibly different diameter
#if USE_ADVANCE
    HAL::resetExtruderDirection();
#endif // USE_ADVANCE

	if(Printer::isHomedAll()) {
		Printer::moveToReal(cx, cy, cz, IGNORE_COORDINATE, EXTRUDER_SWITCH_XY_SPEED);
	}

	Printer::feedrate = oldfeedrate;
	Printer::updateCurrentPosition(true);
}


void Extruder::setTemperatureForExtruder(float temperatureInCelsius, uint8_t extr, bool beep, bool wait) {
#if NUM_EXTRUDER > 0
    bool alloffs = true;
    for(uint8_t i = 0; i < NUM_EXTRUDER; i++)
        if(tempController[i]->targetTemperatureC > 15) alloffs = false;
#ifdef MAXTEMP
    if(temperatureInCelsius > MAXTEMP) temperatureInCelsius = MAXTEMP;
#endif
    if(temperatureInCelsius < 0) temperatureInCelsius = 0;
    TemperatureController *tc = tempController[extr];
        if(tc->sensorType == 0) temperatureInCelsius = 0;
        //if(temperatureInCelsius==tc->targetTemperatureC) return;
        if (temperatureInCelsius < MAX_ROOM_TEMPERATURE)
            tc->resetPreheatTime();
        else if (tc->targetTemperatureC == 0)
            tc->startPreheatTime();

        tc->setTargetTemperature(temperatureInCelsius);
        tc->updateTempControlVars();
        if(beep && temperatureInCelsius > MAX_ROOM_TEMPERATURE)
            tc->setAlarm(true);
        if(temperatureInCelsius >= EXTRUDER_FAN_COOL_TEMP) extruder[extr].coolerPWM = extruder[extr].coolerSpeed;
        Com::printF(PSTR("TargetExtr"), extr, 0);
        Com::printFLN(PSTR(":"), temperatureInCelsius, 0);
    if(wait && temperatureInCelsius > MAX_ROOM_TEMPERATURE
#if defined(SKIP_M109_IF_WITHIN) && SKIP_M109_IF_WITHIN > 0
            && !(abs(tc->currentTemperatureC - tc->targetTemperatureC) < (SKIP_M109_IF_WITHIN))// Already in range
#endif
      ) {
        Extruder *actExtruder = &extruder[extr];
        UI_STATUS_UPD_F(PSTR("Heating extruder"));
        bool dirRising = actExtruder->tempControl.targetTemperatureC > actExtruder->tempControl.currentTemperatureC;
        //millis_t printedTime = HAL::timeInMilliseconds();
        millis_t waituntil = 0;
#if RETRACT_DURING_HEATUP
        uint8_t retracted = 0;
#endif
        millis_t currentTime;
        millis_t maxWaitUntil = 0;
        bool oldAutoreport = Printer::isAutoreportTemp();
        Printer::setAutoreportTemp(true);
        do {
            previousMillisCmd = currentTime = HAL::timeInMilliseconds();
            /*if( (currentTime - printedTime) > 1000 )   //Print Temp Reading every 1 second while heating up.
            {
                Commands::printTemperatures();
                printedTime = currentTime;
            }*/
            Commands::checkForPeriodicalActions(true);
            GCode::keepAlive(WaitHeater);
            //gcode_read_serial();
#if RETRACT_DURING_HEATUP
            if (actExtruder == Extruder::current && actExtruder->waitRetractUnits > 0 && !retracted && dirRising && actExtruder->tempControl.currentTemperatureC > actExtruder->waitRetractTemperature) {
                PrintLine::moveRelativeDistanceInSteps(0, 0, 0, -actExtruder->waitRetractUnits * Printer::axisStepsPerMM[E_AXIS], actExtruder->maxFeedrate / 4, false, false);
                retracted = 1;
            }
#endif
            if(maxWaitUntil == 0) {
                if(dirRising ? actExtruder->tempControl.currentTemperatureC >= actExtruder->tempControl.targetTemperatureC - 5 : actExtruder->tempControl.currentTemperatureC <= actExtruder->tempControl.targetTemperatureC + 5) {
                    maxWaitUntil = currentTime + 120000L;
                }
            } else if((millis_t)(maxWaitUntil - currentTime) < 2000000000UL) {
                break;
            }
            if((waituntil == 0 &&
                    (dirRising ? actExtruder->tempControl.currentTemperatureC >= actExtruder->tempControl.targetTemperatureC - 1
                     : actExtruder->tempControl.currentTemperatureC <= actExtruder->tempControl.targetTemperatureC + 1))
#if defined(TEMP_HYSTERESIS) && TEMP_HYSTERESIS >= 1
                    || (waituntil != 0 && (abs(actExtruder->tempControl.currentTemperatureC - actExtruder->tempControl.targetTemperatureC)) > TEMP_HYSTERESIS)
#endif
              ) {
                waituntil = currentTime + 1000UL * (millis_t)actExtruder->watchPeriod; // now wait for temp. to stabilize
            }
        } while(waituntil == 0 || (waituntil != 0 && (millis_t)(waituntil - currentTime) < 2000000000UL));
        Printer::setAutoreportTemp(oldAutoreport);
#if RETRACT_DURING_HEATUP
        if (retracted && actExtruder == Extruder::current) {
            PrintLine::moveRelativeDistanceInSteps(0, 0, 0, actExtruder->waitRetractUnits * Printer::axisStepsPerMM[E_AXIS], actExtruder->maxFeedrate / 4, false, false);
        }
#endif
    }
    UI_CLEAR_STATUS;

    bool alloff = true;
    for(uint8_t i = 0; i < NUM_EXTRUDER; i++)
        if(tempController[i]->targetTemperatureC > 15) alloff = false;

    if(alloff && !alloffs) // All heaters are now switched off?
        EEPROM::updatePrinterUsage();

    if(alloffs && !alloff) { // heaters are turned on, start measuring printing time
        Printer::msecondsPrinting = HAL::timeInMilliseconds();
        Printer::filamentPrinted = 0;  // new print, new counter
        Printer::flag2 &= ~PRINTER_FLAG2_RESET_FILAMENT_USAGE;
    }
#endif
}

void Extruder::setHeatedBedTemperature(float temperatureInCelsius, bool beep) {
    if(temperatureInCelsius > HEATED_BED_MAX_TEMP) temperatureInCelsius = HEATED_BED_MAX_TEMP;
    if(temperatureInCelsius < 0) temperatureInCelsius = 0;
    if(heatedBedController.targetTemperatureC == temperatureInCelsius) return; // don't flood log with messages if killed
    heatedBedController.setTargetTemperature(temperatureInCelsius);
    if(beep && temperatureInCelsius > 30) heatedBedController.setAlarm(true);
    Com::printFLN(PSTR("TargetBed:"), heatedBedController.targetTemperatureC, 0);
    if(temperatureInCelsius > 15)
        pwm_pos[PWM_BOARD_FAN] = BOARD_FAN_SPEED;    // turn on the mainboard cooling fan
    else if(Printer::areAllSteppersDisabled())
        pwm_pos[PWM_BOARD_FAN] = BOARD_FAN_MIN_SPEED;      // turn off the mainboard cooling fan only if steppers disabled
}

float Extruder::getHeatedBedTemperature() {
    TemperatureController *c = tempController[HEATED_BED_INDEX];
    return c->currentTemperatureC;
}


/** \brief Sends the high-signal to the stepper for next extruder step.
Call this function only, if interrupts are disabled.
*/
void Extruder::step() {
    WRITE(EXT0_STEP_PIN, START_STEP_WITH_HIGH);
#if defined(EXT0_MIRROR_STEPPER) && EXT0_MIRROR_STEPPER
    WRITE(EXT0_STEP2_PIN, START_STEP_WITH_HIGH);
#endif
}
/** \brief Sets stepper signal to low for current extruder.

Call this function only, if interrupts are disabled.
*/


void Extruder::unstep() {
    WRITE(EXT0_STEP_PIN, !START_STEP_WITH_HIGH);
#if defined(EXT0_MIRROR_STEPPER) && EXT0_MIRROR_STEPPER
    WRITE(EXT0_STEP2_PIN, !START_STEP_WITH_HIGH);
#endif
}
/** \brief Activates the extruder stepper and sets the direction. */
void Extruder::setDirection(uint8_t dir) {
    if(dir) {
        WRITE(EXT0_DIR_PIN, !EXT0_INVERSE);
#if defined(EXT0_MIRROR_STEPPER) && EXT0_MIRROR_STEPPER
        WRITE(EXT0_DIR2_PIN, !EXT0_INVERSE2);
#endif
    } else {
        WRITE(EXT0_DIR_PIN, EXT0_INVERSE);
#if defined(EXT0_MIRROR_STEPPER) && EXT0_MIRROR_STEPPER
        WRITE(EXT0_DIR2_PIN, EXT0_INVERSE2);
#endif
    }
}

void Extruder::enable() {

#if EXT0_ENABLE_PIN > -1
    WRITE(EXT0_ENABLE_PIN, EXT0_ENABLE_ON );
#if defined(EXT0_MIRROR_STEPPER) && EXT0_MIRROR_STEPPER
    WRITE(EXT0_ENABLE2_PIN, EXT0_ENABLE_ON);
#endif
#endif
}



void Extruder::disableCurrentExtruderMotor() {
    switch(Extruder::current->id) {
#if defined(EXT0_MIRROR_STEPPER) && EXT0_MIRROR_STEPPER
    case 0:
        WRITE(EXT0_ENABLE2_PIN, !EXT0_ENABLE_ON);
        break;
#endif
    }
    if(Extruder::current->enablePin > -1)
        HAL::digitalWrite(Extruder::current->enablePin, !Extruder::current->enableOn);

}


void Extruder::disableAllExtruderMotors() {
    for(fast8_t i = 0; i < NUM_EXTRUDER; i++) {
        if(extruder[i].enablePin > -1)
            HAL::digitalWrite(extruder[i].enablePin, !extruder[i].enableOn);
    }
}


#define NUMTEMPS_1 28

// Epcos B57560G0107F000

const short temptable_1[NUMTEMPS_1][2] PROGMEM = {
    {0, 4000}, {92, 2400}, {105, 2320}, {121, 2240}, {140, 2160}, {162, 2080}, {189, 2000}, {222, 1920}, {261, 1840}, {308, 1760},
    {365, 1680}, {434, 1600}, {519, 1520}, {621, 1440}, {744, 1360}, {891, 1280}, {1067, 1200}, {1272, 1120},
    {1771, 960}, {2357, 800}, {2943, 640}, {3429, 480}, {3760, 320}, {3869, 240}, {3912, 200}, {3948, 160}, {4077, -160}, {4094, -440}
};
#define NUMTEMPS_2 21
const short temptable_2[NUMTEMPS_2][2] PROGMEM = {
    {1 * 4, 848 * 8}, {54 * 4, 275 * 8}, {107 * 4, 228 * 8}, {160 * 4, 202 * 8}, {213 * 4, 185 * 8}, {266 * 4, 171 * 8}, {319 * 4, 160 * 8}, {372 * 4, 150 * 8},
    {425 * 4, 141 * 8}, {478 * 4, 133 * 8}, {531 * 4, 125 * 8}, {584 * 4, 118 * 8}, {637 * 4, 110 * 8}, {690 * 4, 103 * 8}, {743 * 4, 95 * 8}, {796 * 4, 86 * 8},
    {849 * 4, 77 * 8}, {902 * 4, 65 * 8}, {955 * 4, 49 * 8}, {1008 * 4, 17 * 8}, {1020 * 4, 0 * 8} //safety
};

#define NUMTEMPS_3 28
const short temptable_3[NUMTEMPS_3][2] PROGMEM = {
    {1 * 4, 864 * 8}, {21 * 4, 300 * 8}, {25 * 4, 290 * 8}, {29 * 4, 280 * 8}, {33 * 4, 270 * 8}, {39 * 4, 260 * 8}, {46 * 4, 250 * 8}, {54 * 4, 240 * 8}, {64 * 4, 230 * 8}, {75 * 4, 220 * 8},
    {90 * 4, 210 * 8}, {107 * 4, 200 * 8}, {128 * 4, 190 * 8}, {154 * 4, 180 * 8}, {184 * 4, 170 * 8}, {221 * 4, 160 * 8}, {265 * 4, 150 * 8}, {316 * 4, 140 * 8}, {375 * 4, 130 * 8},
    {441 * 4, 120 * 8}, {513 * 4, 110 * 8}, {588 * 4, 100 * 8}, {734 * 4, 80 * 8}, {856 * 4, 60 * 8}, {938 * 4, 40 * 8}, {986 * 4, 20 * 8}, {1008 * 4, 0 * 8}, {1018 * 4, -20 * 8}
};

#define NUMTEMPS_4 20
const short temptable_4[NUMTEMPS_4][2] PROGMEM = {
    {1 * 4, 430 * 8}, {54 * 4, 137 * 8}, {107 * 4, 107 * 8}, {160 * 4, 91 * 8}, {213 * 4, 80 * 8}, {266 * 4, 71 * 8}, {319 * 4, 64 * 8}, {372 * 4, 57 * 8}, {425 * 4, 51 * 8},
    {478 * 4, 46 * 8}, {531 * 4, 41 * 8}, {584 * 4, 35 * 8}, {637 * 4, 30 * 8}, {690 * 4, 25 * 8}, {743 * 4, 20 * 8}, {796 * 4, 14 * 8}, {849 * 4, 7 * 8}, {902 * 4, 0 * 8},
    {955 * 4, -11 * 8}, {1008 * 4, -35 * 8}
};

// ATC 104GT

#define NUMTEMPS_8 34
const short temptable_8[NUMTEMPS_8][2] PROGMEM = {
    {0, 8000}, {69, 2400}, {79, 2320}, {92, 2240}, {107, 2160}, {125, 2080}, {146, 2000}, {172, 1920}, {204, 1840}, {244, 1760}, {291, 1680}, {350, 1600},
    {422, 1520}, {511, 1440}, {621, 1360}, {755, 1280}, {918, 1200}, {1114, 1120}, {1344, 1040}, {1608, 960}, {1902, 880}, {2216, 800}, {2539, 720},
    {2851, 640}, {3137, 560}, {3385, 480}, {3588, 400}, {3746, 320}, {3863, 240}, {3945, 160}, {4002, 80}, {4038, 0}, {4061, -80}, {4075, -160}
};
#define NUMTEMPS_9 67 // 100k Honeywell 135-104LAG-J01
const short temptable_9[NUMTEMPS_9][2] PROGMEM = {
    {1 * 4, 941 * 8}, {19 * 4, 362 * 8}, {37 * 4, 299 * 8}, //top rating 300C
    {55 * 4, 266 * 8}, {73 * 4, 245 * 8}, {91 * 4, 229 * 8}, {109 * 4, 216 * 8}, {127 * 4, 206 * 8}, {145 * 4, 197 * 8}, {163 * 4, 190 * 8}, {181 * 4, 183 * 8}, {199 * 4, 177 * 8},
    {217 * 4, 171 * 8}, {235 * 4, 166 * 8}, {253 * 4, 162 * 8}, {271 * 4, 157 * 8}, {289 * 4, 153 * 8}, {307 * 4, 149 * 8}, {325 * 4, 146 * 8}, {343 * 4, 142 * 8}, {361 * 4, 139 * 8},
    {379 * 4, 135 * 8}, {397 * 4, 132 * 8}, {415 * 4, 129 * 8}, {433 * 4, 126 * 8}, {451 * 4, 123 * 8}, {469 * 4, 121 * 8}, {487 * 4, 118 * 8}, {505 * 4, 115 * 8}, {523 * 4, 112 * 8},
    {541 * 4, 110 * 8}, {559 * 4, 107 * 8}, {577 * 4, 105 * 8}, {595 * 4, 102 * 8}, {613 * 4, 99 * 8}, {631 * 4, 97 * 8}, {649 * 4, 94 * 8}, {667 * 4, 92 * 8}, {685 * 4, 89 * 8},
    {703 * 4, 86 * 8}, {721 * 4, 84 * 8}, {739 * 4, 81 * 8}, {757 * 4, 78 * 8}, {775 * 4, 75 * 8}, {793 * 4, 72 * 8}, {811 * 4, 69 * 8}, {829 * 4, 66 * 8}, {847 * 4, 62 * 8},
    {865 * 4, 59 * 8}, {883 * 4, 55 * 8}, {901 * 4, 51 * 8}, {919 * 4, 46 * 8}, {937 * 4, 41 * 8},
    {955 * 4, 35 * 8}, {973 * 4, 27 * 8}, {991 * 4, 17 * 8}, {1009 * 4, 1 * 8}, {1023 * 4, 0} //to allow internal 0 degrees C
};
#define NUMTEMPS_10 20 // 100k 0603 SMD Vishay NTCS0603E3104FXT (4.7k pullup)
const short temptable_10[NUMTEMPS_10][2] PROGMEM = {
    {1 * 4, 704 * 8}, {54 * 4, 216 * 8}, {107 * 4, 175 * 8}, {160 * 4, 152 * 8}, {213 * 4, 137 * 8}, {266 * 4, 125 * 8}, {319 * 4, 115 * 8}, {372 * 4, 106 * 8}, {425 * 4, 99 * 8},
    {478 * 4, 91 * 8}, {531 * 4, 85 * 8}, {584 * 4, 78 * 8}, {637 * 4, 71 * 8}, {690 * 4, 65 * 8}, {743 * 4, 58 * 8}, {796 * 4, 50 * 8}, {849 * 4, 42 * 8}, {902 * 4, 31 * 8},
    {955 * 4, 17 * 8}, {1008 * 4, 0}
};
#define NUMTEMPS_11 31 // 100k GE Sensing AL03006-58.2K-97-G1 (4.7k pullup)
const short temptable_11[NUMTEMPS_11][2] PROGMEM = {
    {1 * 4, 936 * 8}, {36 * 4, 300 * 8}, {71 * 4, 246 * 8}, {106 * 4, 218 * 8}, {141 * 4, 199 * 8}, {176 * 4, 185 * 8}, {211 * 4, 173 * 8}, {246 * 4, 163 * 8}, {281 * 4, 155 * 8},
    {316 * 4, 147 * 8}, {351 * 4, 140 * 8}, {386 * 4, 134 * 8}, {421 * 4, 128 * 8}, {456 * 4, 122 * 8}, {491 * 4, 117 * 8}, {526 * 4, 112 * 8}, {561 * 4, 107 * 8}, {596 * 4, 102 * 8},
    {631 * 4, 97 * 8}, {666 * 4, 92 * 8}, {701 * 4, 87 * 8}, {736 * 4, 81 * 8}, {771 * 4, 76 * 8}, {806 * 4, 70 * 8}, {841 * 4, 63 * 8}, {876 * 4, 56 * 8}, {911 * 4, 48 * 8},
    {946 * 4, 38 * 8}, {981 * 4, 23 * 8}, {1005 * 4, 5 * 8}, {1016 * 4, 0}
};
#define NUMTEMPS_12 31 // 100k RS thermistor 198-961 (4.7k pullup)
const short temptable_12[NUMTEMPS_12][2] PROGMEM = {
    {1 * 4, 929 * 8}, {36 * 4, 299 * 8}, {71 * 4, 246 * 8}, {106 * 4, 217 * 8}, {141 * 4, 198 * 8}, {176 * 4, 184 * 8}, {211 * 4, 173 * 8}, {246 * 4, 163 * 8}, {281 * 4, 154 * 8}, {316 * 4, 147 * 8},
    {351 * 4, 140 * 8}, {386 * 4, 134 * 8}, {421 * 4, 128 * 8}, {456 * 4, 122 * 8}, {491 * 4, 117 * 8}, {526 * 4, 112 * 8}, {561 * 4, 107 * 8}, {596 * 4, 102 * 8}, {631 * 4, 97 * 8}, {666 * 4, 91 * 8},
    {701 * 4, 86 * 8}, {736 * 4, 81 * 8}, {771 * 4, 76 * 8}, {806 * 4, 70 * 8}, {841 * 4, 63 * 8}, {876 * 4, 56 * 8}, {911 * 4, 48 * 8}, {946 * 4, 38 * 8}, {981 * 4, 23 * 8}, {1005 * 4, 5 * 8}, {1016 * 4, 0 * 8}
};
#define NUMTEMPS_13 19
const short temptable_13[NUMTEMPS_13][2] PROGMEM = {
    {0, 0}, {908, 8}, {942, 10 * 8}, {982, 20 * 8}, {1015, 8 * 30}, {1048, 8 * 40}, {1080, 8 * 50}, {1113, 8 * 60}, {1146, 8 * 70}, {1178, 8 * 80}, {1211, 8 * 90}, {1276, 8 * 110}, {1318, 8 * 120}
    , {1670, 8 * 230}, {2455, 8 * 500}, {3445, 8 * 900}, {3666, 8 * 1000}, {3871, 8 * 1100}, {4095, 8 * 2000}
};
#define NUMTEMPS_14 46
const short temptable_14[NUMTEMPS_14][2] PROGMEM = {
    {1 * 4, 8 * 938}, {31 * 4, 8 * 314}, {41 * 4, 8 * 290}, {51 * 4, 8 * 272}, {61 * 4, 8 * 258}, {71 * 4, 8 * 247}, {81 * 4, 8 * 237}, {91 * 4, 8 * 229}, {101 * 4, 8 * 221}, {111 * 4, 8 * 215}, {121 * 4, 8 * 209},
    {131 * 4, 8 * 204}, {141 * 4, 8 * 199}, {151 * 4, 8 * 195}, {161 * 4, 8 * 190}, {171 * 4, 8 * 187}, {181 * 4, 8 * 183}, {191 * 4, 8 * 179}, {201 * 4, 8 * 176}, {221 * 4, 8 * 170}, {241 * 4, 8 * 165},
    {261 * 4, 8 * 160}, {281 * 4, 8 * 155}, {301 * 4, 8 * 150}, {331 * 4, 8 * 144}, {361 * 4, 8 * 139}, {391 * 4, 8 * 133}, {421 * 4, 8 * 128}, {451 * 4, 8 * 123}, {491 * 4, 8 * 117}, {531 * 4, 8 * 111},
    {571 * 4, 8 * 105}, {611 * 4, 8 * 100}, {681 * 4, 8 * 90}, {711 * 4, 8 * 85}, {811 * 4, 8 * 69}, {831 * 4, 8 * 65}, {881 * 4, 8 * 55},
    {901 * 4, 8 * 51},  {941 * 4, 8 * 39}, {971 * 4, 8 * 28}, {981 * 4, 8 * 23}, {991 * 4, 8 * 17}, {1001 * 4, 8 * 9}, {1021 * 4, 8 * -27}, {1023 * 4, 8 * -200}
};
#define NUMTEMPS_15 27 // DYZE DESIGN 500°C Thermistor
const short temptable_15[NUMTEMPS_15][2] PROGMEM = {
    { 18 * 4, 850 * 8 }, { 18 * 4, 500 * 8 }, { 22 * 4, 480 * 8 }, { 27 * 4, 460 * 8 }, { 33 * 4, 440 * 8 }, { 41 * 4, 420 * 8 }, { 52 * 4, 400 * 8 }, { 68 * 4, 380 * 8 }, { 86 * 4, 360 * 8 }, { 112 * 4, 340 * 8 },
    { 147 * 4, 320 * 8 }, { 194 * 4, 300 * 8 }, { 254 * 4, 280 * 8 }, { 330 * 4, 260 * 8 }, { 428 * 4, 240 * 8 }, { 533 * 4, 220 * 8 }, { 646 * 4, 200 * 8 }, { 754 * 4, 180 * 8 }, { 844 * 4, 160 * 8 },
    { 912 * 4, 140 * 8 }, { 959 * 4, 120 * 8 }, { 989 * 4, 100 * 8 }, { 1007 * 4, 80 * 8 }, { 1016 * 4, 60 * 8 }, { 1021 * 4, 30 * 8 }, { 4091, 25 * 8 }, { 4092, 20 * 8 }
};
// Contributed by Brandon Coates - May 2015
// B3 Innovations Pico 500c Thermistor
// B150/250 = 5300 K +/- 3%
// R 250 = 2.705k Ohms +/- 2.5%
#define NUMTEMPS_16 49
const short temptable_16[NUMTEMPS_16][2] PROGMEM = {
    {  37,  510 * 8 },
    {  44,  500 * 8 },
    {  51,  490 * 8 },
    {  58,  480 * 8 },
    {  67,  470 * 8 },
    {  77,  460 * 8 },
    {  88,  450 * 8 },
    {  99,  440 * 8 },
    {  112,  430 * 8 },
    {  126,  420 * 8 },
    {  144,  410 * 8 },
    {  163,  400 * 8 },
    {  185,  390 * 8 },
    {  210,  380 * 8 },
    {  240,  370 * 8 },
    {  273,  360 * 8 },
    {  311,  350 * 8 },
    {  355,  340 * 8 },
    {  408,  330 * 8 },
    {  470,  320 * 8 },
    {  542,  310 * 8 },
    {  623,  300 * 8 },
    {  720,  290 * 8 },
    {  832,  280 * 8 },
    {  961,  270 * 8 },
    {  1108,  260 * 8 },
    {  1276,  250 * 8 },
    {  1463,  240 * 8 },
    {  1676,  230 * 8 },
    {  1896,  220 * 8 },
    {  2133,  210 * 8 },
    {  2382,  200 * 8 },
    {  2635,  190 * 8 },
    {  2866,  180 * 8 },
    {  3092,  170 * 8 },
    {  3288,  160 * 8 },
    {  3468,  150 * 8 },
    {  3608,  140 * 8 },
    {  3727,  130 * 8 },
    {  3824,  120 * 8 },
    {  3898,  110 * 8 },
    {  3955,  100 * 8 },
    {  4029,  80 * 8 },
    {  4060,  65 * 8 },
    {  4073,  55 * 8 },
    {  4082,  45 * 8 },
    {  4088,  35 * 8 },
    {  4092,  25 * 8 },
    {  4095,   0 * 8 },
};
#if NUM_TEMPS_USERTHERMISTOR0 > 0
const short temptable_5[NUM_TEMPS_USERTHERMISTOR0][2] PROGMEM = USER_THERMISTORTABLE0 ;
#endif
#if NUM_TEMPS_USERTHERMISTOR1 > 0
const short temptable_6[NUM_TEMPS_USERTHERMISTOR1][2] PROGMEM = USER_THERMISTORTABLE1 ;
#endif
#if NUM_TEMPS_USERTHERMISTOR2 > 0
const short temptable_7[NUM_TEMPS_USERTHERMISTOR2][2] PROGMEM = USER_THERMISTORTABLE2 ;
#endif
const short * const temptables[16] PROGMEM = {(short int *)&temptable_1[0][0], (short int *)&temptable_2[0][0], (short int *)&temptable_3[0][0], (short int *)&temptable_4[0][0]
#if NUM_TEMPS_USERTHERMISTOR0 > 0
                                              , (short int *)&temptable_5[0][0]
#else
                                              , 0
#endif
#if NUM_TEMPS_USERTHERMISTOR1 > 0
                                              , (short int *)&temptable_6[0][0]
#else
                                              , 0
#endif
#if NUM_TEMPS_USERTHERMISTOR2 > 0
                                              , (short int *)&temptable_7[0][0]
#else
                                              , 0
#endif
                                              , (short int *)&temptable_8[0][0]
                                              , (short int *)&temptable_9[0][0]
                                              , (short int *)&temptable_10[0][0]
                                              , (short int *)&temptable_11[0][0]
                                              , (short int *)&temptable_12[0][0]
                                              , (short int *)&temptable_13[0][0]
                                              , (short int *)&temptable_14[0][0]
                                              , (short int *)&temptable_15[0][0]
                                              , (short int *)&temptable_16[0][0]
                                             };
const uint8_t temptables_num[16] PROGMEM = {NUMTEMPS_1, NUMTEMPS_2, NUMTEMPS_3, NUMTEMPS_4, NUM_TEMPS_USERTHERMISTOR0, NUM_TEMPS_USERTHERMISTOR1, NUM_TEMPS_USERTHERMISTOR2, NUMTEMPS_8,
                                            NUMTEMPS_9, NUMTEMPS_10, NUMTEMPS_11, NUMTEMPS_12, NUMTEMPS_13, NUMTEMPS_14, NUMTEMPS_15, NUMTEMPS_16
                                           };


void TemperatureController::updateCurrentTemperature() {
    uint8_t type = sensorType;
    // get raw temperature
    switch(type) {
    case 0:
        currentTemperature = 25;
        break;
#if ANALOG_INPUTS > 0
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
    case 8:
    case 9:
    case 10:
    case 11:
    case 12:
    case 14:
    case 15:
    case 16:
    case 97:
    case 98:
    case 99:
        currentTemperature = (1023 << (2 - ANALOG_REDUCE_BITS)) - (osAnalogInputValues[sensorPin] >> (ANALOG_REDUCE_BITS)); // Convert to 10 bit result
        break;
    case 13: // PT100 E3D
    case 50: // User defined PTC table
    case 51:
    case 52:
    case 60: // HEATER_USES_AD8495 (Delivers 5mV/degC)
    case 61: // HEATER_USES_AD8495 (Delivers 5mV/degC) 1.25v offset
    case 100: // AD595 / AD597
        currentTemperature = (osAnalogInputValues[sensorPin] >> (ANALOG_REDUCE_BITS));
        break;
#endif
    default:
        currentTemperature = 4095; // unknown method, return high value to switch heater off for safety
    }
    int currentTemperature = this->currentTemperature;
    switch(type) {
    case 0:
        currentTemperatureC = 25;
        break;
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
    case 8:
    case 9:
    case 10:
    case 11:
    case 12:
    case 14:
    case 15:
    case 16: {
        type--;
        uint8_t num = pgm_read_byte(&temptables_num[type]) << 1;
        uint8_t i = 2;
        const int16_t *temptable = (const int16_t *)pgm_read_word(&temptables[type]); //pgm_read_word_near(&temptables[type]);
        int16_t oldraw = pgm_read_word(&temptable[0]);
        int16_t oldtemp = pgm_read_word(&temptable[1]);
        int16_t newraw, newtemp = 0;
        currentTemperature = (1023 << (2 - ANALOG_REDUCE_BITS)) - currentTemperature;
        while(i < num) {
            newraw = pgm_read_word(&temptable[i++]);
            newtemp = pgm_read_word(&temptable[i++]);
            if (newraw > currentTemperature) {
                currentTemperatureC = TEMP_INT_TO_FLOAT(oldtemp + (float)(currentTemperature - oldraw) * (float)(newtemp - oldtemp) / (newraw - oldraw));
                return;
            }
            oldtemp = newtemp;
            oldraw = newraw;
        }
        // Overflow: Set to last value in the table
        currentTemperatureC = TEMP_INT_TO_FLOAT(newtemp);
    }
    break;
    case 13:
    case 50: // User defined PTC thermistor
    case 51:
    case 52: {
        if(type > 49)
            type -= 46;
        else
            type--;
        uint8_t num = pgm_read_byte(&temptables_num[type]) << 1;
        uint8_t i = 2;
        const int16_t *temptable = (const int16_t *)pgm_read_word(&temptables[type]); //pgm_read_word_near(&temptables[type]);
        int16_t oldraw = pgm_read_word(&temptable[0]);
        int16_t oldtemp = pgm_read_word(&temptable[1]);
        int16_t newraw, newtemp = 0;
        while(i < num) {
            newraw = pgm_read_word(&temptable[i++]);
            newtemp = pgm_read_word(&temptable[i++]);
            if (newraw > currentTemperature) {
                currentTemperatureC = TEMP_INT_TO_FLOAT(oldtemp + (float)(currentTemperature - oldraw) * (float)(newtemp - oldtemp) / (newraw - oldraw));
                return;
            }
            oldtemp = newtemp;
            oldraw = newraw;
        }
        // Overflow: Set to last value in the table
        currentTemperatureC = TEMP_INT_TO_FLOAT(newtemp);
        break;
    }
    case 60: // AD8495 (Delivers 5mV/degC vs the AD595's 10mV)
        currentTemperatureC = ((float)currentTemperature * 1000.0f / (1024 << (2 - ANALOG_REDUCE_BITS)));
        break;
    case 61: // AD8495 1.25V Vref offset (like Adafruit 8495 breakout board)
        currentTemperatureC = ((float)currentTemperature * 1000.0f / (1024 << (2 - ANALOG_REDUCE_BITS))) - 250.0f;
        break;
    case 62: // TMP36
        currentTemperatureC = ((float)currentTemperature * 500.0f / (1024 << (2 - ANALOG_REDUCE_BITS))) - 50.0f;
        break;
    case 100: // AD595 / AD597   10mV/°C
        //return (int)((long)raw_temp * 500/(1024<<(2-ANALOG_REDUCE_BITS)));
        currentTemperatureC = ((float)currentTemperature * 500.0f / (1024 << (2 - ANALOG_REDUCE_BITS)));
        break;
#if defined(USE_GENERIC_THERMISTORTABLE_1) || defined(USE_GENERIC_THERMISTORTABLE_2) || defined(USE_GENERIC_THERMISTORTABLE_3)
    case 97:
    case 98:
    case 99: {
        uint8_t i = 2;
        const short *temptable;
#ifdef USE_GENERIC_THERMISTORTABLE_1
        if(type == 97)
            temptable = (const short *)temptable_generic1;
#endif
#ifdef USE_GENERIC_THERMISTORTABLE_2
        if(type == 98)
            temptable = (const short *)temptable_generic2;
#endif
#ifdef USE_GENERIC_THERMISTORTABLE_3
        if(type == 99)
            temptable = (const short *)temptable_generic3;
#endif
        short oldraw = temptable[0];
        short oldtemp = temptable[1];
        short newraw, newtemp;
        currentTemperature = (1023 << (2 - ANALOG_REDUCE_BITS)) - currentTemperature;
        while(i < GENERIC_THERM_NUM_ENTRIES * 2) {
            newraw = temptable[i++];
            newtemp = temptable[i++];
            if (newraw > currentTemperature) {
                currentTemperatureC = TEMP_INT_TO_FLOAT(oldtemp + (float)(currentTemperature - oldraw) * (float)(newtemp - oldtemp) / (newraw - oldraw));
                return;
            }
            oldtemp = newtemp;
            oldraw = newraw;
        }
        // Overflow: Set to last value in the table
        currentTemperatureC = TEMP_INT_TO_FLOAT(newtemp);
        break;
    }
#endif
    }
}

void TemperatureController::setTargetTemperature(float target) {
    targetTemperatureC = target;
    stopDecouple();
}

uint8_t autotuneIndex = 255;
void Extruder::disableAllHeater() {
#if NUM_TEMPERATURE_LOOPS > 0
    for(uint8_t i = 0; i <= HEATED_BED_INDEX; i++) {
        TemperatureController *c = tempController[i];
        c->targetTemperatureC = 0;
        pwm_pos[c->pwmIndex] = 0;
    }
#endif
    autotuneIndex = 255;
}

void TemperatureController::autotunePID(float temp, uint8_t controllerId, int maxCycles, bool storeValues, int method) {
    if(method < 0) method = 0;
    if(method > 4) method = 4;
    float currentTemp;
    int cycles = 0;
    bool heating = true;

    uint32_t temp_millis = HAL::timeInMilliseconds();
    uint32_t t1 = temp_millis;
    uint32_t t2 = temp_millis;
    int32_t t_high = 0;
    int32_t t_low;

    int32_t bias = pidMax >> 1;
    int32_t d = pidMax >> 1;
    float Ku, Tu;
    float Kp = 0, Ki = 0, Kd = 0;
    float maxTemp = 20, minTemp = 20;
    if(maxCycles < 5)
        maxCycles = 5;
    if(maxCycles > 20)
        maxCycles = 20;
    Com::printInfoFLN(PSTR("PID Autotune start"));

    //Extruder::disableAllHeater(); // switch off all heaters.
    autotuneIndex = controllerId;
    pwm_pos[pwmIndex] = pidMax;
    if(controllerId < NUM_EXTRUDER) {
        extruder[controllerId].coolerPWM = extruder[controllerId].coolerSpeed;
        extruder[0].coolerPWM = extruder[0].coolerSpeed;
    }
    for(;;) {
#if FEATURE_WATCHDOG
        HAL::pingWatchdog();
#endif // FEATURE_WATCHDOG
        Commands::checkForPeriodicalActions(true); // update heaters etc.
        GCode::keepAlive(WaitHeater);
        updateCurrentTemperature();
        currentTemp = currentTemperatureC;
        millis_t time = HAL::timeInMilliseconds();
        maxTemp = RMath::max(maxTemp, currentTemp);
        minTemp = RMath::min(minTemp, currentTemp);
        if(heating == true && currentTemp > temp) { // switch heating -> off
            if(time - t2 > (controllerId < NUM_EXTRUDER ? 2500 : 1500)) {
                heating = false;
                pwm_pos[pwmIndex] = (bias - d);
                t1 = time;
                t_high = t1 - t2;
                maxTemp = temp;
            }
        }
        if(heating == false && currentTemp < temp) {
            if(time - t1 > (controllerId < NUM_EXTRUDER ? 5000 : 3000)) {
                heating = true;
                t2 = time;
                t_low = t2 - t1; // half wave length
                if(cycles > 0) {
                    bias += (d * (t_high - t_low)) / (t_low + t_high);
                    bias = constrain(bias, 20, pidMax - 20);
                    if(bias > pidMax / 2) d = pidMax - 1 - bias;
                    else d = bias;

                    Com::printF(PSTR(" bias: "), bias);
                    Com::printF(PSTR(" d: "), d);
                    Com::printF(PSTR(" min: "), minTemp);
                    Com::printFLN(PSTR(" max: "), maxTemp);
                    if(cycles > 2) {
                        // Parameter according Ziegler¡§CNichols method: http://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method
                        Ku = (4.0 * d) / (3.14159 * (maxTemp - minTemp));
                        Tu = static_cast<float>(t_low + t_high) / 1000.0;
                        Com::printF(PSTR(" Ku: "), Ku);
                        Com::printFLN(PSTR(" Tu: "), Tu);
                        if(method == 0) {
                            Kp = 0.6 * Ku;
                            Ki = Kp * 2.0 / Tu;
                            Kd = Kp * Tu * 0.125;
                            Com::printFLN(PSTR(" Classic PID"));
                        }
                        if(method == 1) {
                            Kp = 0.33 * Ku;
                            Ki = Kp * 2.0 / Tu;
                            Kd = Kp * Tu / 3.0;
                            Com::printFLN(PSTR(" Some Overshoot PID"));
                        }
                        if(method == 2) {
                            Kp = 0.2 * Ku;
                            Ki = Kp * 2.0 / Tu;
                            Kd = Kp * Tu / 3;
                            Com::printFLN(PSTR(" No Overshoot PID"));
                        }
                        if(method == 3) {
                            Kp = 0.7 * Ku;
                            Ki = Kp * 2.5 / Tu;
                            Kd = Kp * Tu * 3.0 / 20.0;
                            Com::printFLN(PSTR(" Pessen Integral Rule PID"));
                        }
						if(method == 4) { //Tyreus-Lyben
						   Kp = 0.4545f*Ku;      //1/2.2 KRkrit
			               Ki = Kp/Tu/2.2f;        //2.2 Tkrit
	                       Kd = Kp*Tu/6.3f;      //1/6.3 Tkrit[/code]
	                       Com::printFLN(PSTR(" Tyreus-Lyben PID"));
						}
                        Com::printFLN(PSTR(" Kp: "), Kp);
                        Com::printFLN(PSTR(" Ki: "), Ki);
                        Com::printFLN(PSTR(" Kd: "), Kd);
                    }
                }
                pwm_pos[pwmIndex] = (bias + d);
                cycles++;
                minTemp = temp;
            }
        }
        if(currentTemp > (temp + 40)) {
            Com::printErrorFLN(PSTR("PID Autotune failed! Temperature too high"));
            //Extruder::disableAllHeater();
            autotuneIndex = 255;
            return;
        }
        if(time - temp_millis > 1000) {
            temp_millis = time;
            Commands::printTemperatures();
        }
        if(((time - t1) + (time - t2)) > (10L * 60L * 1000L * 2L)) { // 20 Minutes
            Com::printErrorFLN(PSTR("PID Autotune failed! timeout"));
            //Extruder::disableAllHeater();
            autotuneIndex = 255;
            return;
        }
        if(cycles > maxCycles) {
            Com::printInfoFLN(PSTR("PID Autotune finished ! Place the Kp, Ki and Kd constants in the Configuration.h or EEPROM"));
            //Extruder::disableAllHeater();
            autotuneIndex = 255;
            if(storeValues) {
                pidPGain = Kp;
                pidIGain = Ki;
                pidDGain = Kd;
                heatManager = HTR_PID;
                EEPROM::storeDataIntoEEPROM();
            }
            return;
        }
        uid.mediumAction();
        uid.slowAction(true);
    } // loop
}

/** \brief Writes monitored temperatures.

This function is called every 250ms to write the monitored temperature. If monitoring is
disabled, the function is not called.
*/
void writeMonitor() {
    Commands::printTemperatures(false);
}

bool reportTempsensorError() {
#if NUM_TEMPERATURE_LOOPS > 0
    if(!Printer::isAnyTempsensorDefect()) return false;
    for(uint8_t i = 0; i < NUM_TEMPERATURE_LOOPS; i++) {
        if(i == HEATED_BED_INDEX) Com::printF(PSTR("heated bed"));
        else
            if(i < NUM_EXTRUDER) Com::printF(PSTR("extruder "), i);
            else Com::printF(PSTR("Other:"));
        TemperatureController *act = tempController[i];
        int temp = act->currentTemperatureC;
        if(temp < MIN_DEFECT_TEMPERATURE || temp > MAX_DEFECT_TEMPERATURE)
            Com::printF(PSTR(": temp sensor defect"));
        else
            Com::printF(PSTR(": working"));
        if(act->flags & TEMPERATURE_CONTROLLER_FLAG_SENSDEFECT)
            Com::printF(PSTR(" marked defect"));
        if(act->flags & TEMPERATURE_CONTROLLER_FLAG_SENSDECOUPLED)
            Com::printF(PSTR(" marked decoupled"));
        Com::println();
    }
    Com::printErrorFLN(PSTR("Printer set into dry run mode until restart!"));
    return true;
#else
    return false;
#endif
}


#if FEATURE_RETRACTION
void Extruder::retractDistance(float dist, bool extraLength) {
    float oldFeedrate = Printer::feedrate;
    int32_t distance = static_cast<int32_t>(dist * stepsPerMM / Printer::extrusionFactor);
    int32_t oldEPos = Printer::currentPositionSteps[E_AXIS];
    float speed = distance > 0 ? EEPROM_FLOAT(RETRACTION_SPEED) : EEPROM_FLOAT(RETRACTION_UNDO_SPEED);
    PrintLine::moveRelativeDistanceInSteps(0, 0, 0, -distance, RMath::max(speed, 1.f), false, false);
    Printer::currentPositionSteps[E_AXIS] = oldEPos; // restore previous extruder position
    Printer::feedrate = oldFeedrate;
}

void Extruder::retract(bool isRetract, bool isLong) {
    float oldFeedrate = Printer::feedrate;
    float distance = (isLong ? EEPROM_FLOAT( RETRACTION_LONG_LENGTH) : EEPROM_FLOAT(RETRACTION_LENGTH));
    float zLiftF = EEPROM_FLOAT(RETRACTION_Z_LIFT);
    int32_t zlift = static_cast<int32_t>(zLiftF * Printer::axisStepsPerMM[Z_AXIS]);
    if(isRetract && !isRetracted()) {
#ifdef EARLY_ZLIFT
        if(zlift > 0) {
            PrintLine::moveRelativeDistanceInStepsReal(0, 0, zlift, 0, Printer::maxFeedrate[Z_AXIS], false);
            Printer::coordinateOffset[Z_AXIS] -= zLiftF;
        }
#endif
        retractDistance(distance);
        setRetracted(true);
#ifndef EARLY_ZLIFT
        if(zlift > 0) {
            PrintLine::moveRelativeDistanceInStepsReal(0, 0, zlift, 0, Printer::maxFeedrate[Z_AXIS], false);
            Printer::coordinateOffset[Z_AXIS] -= zLiftF;
        }
#endif
    } else if(!isRetract && isRetracted()) {
        if(zlift > 0) {
            PrintLine::moveRelativeDistanceInStepsReal(0, 0, -zlift, 0, Printer::maxFeedrate[Z_AXIS], false);
            Printer::coordinateOffset[Z_AXIS] += zLiftF;
        }
        retractDistance(-distance - (isLong ? EEPROM_FLOAT(RETRACTION_UNDO_EXTRA_LONG_LENGTH) : EEPROM_FLOAT(RETRACTION_UNDO_EXTRA_LENGTH)), false);
        setRetracted(false);
    }
    Printer::feedrate = oldFeedrate;
}
#endif

Extruder *Extruder::current;

const char ext0_select_cmd[] PROGMEM = EXT0_SELECT_COMMANDS;
const char ext0_deselect_cmd[] PROGMEM = EXT0_DESELECT_COMMANDS;

Extruder extruder[NUM_EXTRUDER] = {
    {
        0, EXT0_X_OFFSET, EXT0_Y_OFFSET, EXT0_Z_OFFSET, EXT0_STEPS_PER_MM, EXT0_ENABLE_PIN, EXT0_ENABLE_ON,
        EXT0_MAX_FEEDRATE, EXT0_MAX_ACCELERATION, EXT0_MAX_START_FEEDRATE, 0, EXT0_WATCHPERIOD
        , EXT0_WAIT_RETRACT_TEMP, EXT0_WAIT_RETRACT_UNITS
#if USE_ADVANCE
#if ENABLE_QUADRATIC_ADVANCE
        , EXT0_ADVANCE_K
#endif
        , EXT0_ADVANCE_L, 0
#endif
        , {
            0, EXT0_TEMPSENSOR_TYPE, EXT0_SENSOR_INDEX, EXT0_HEAT_MANAGER, 0, 0, 0, 0, 0, 0
            , 0, EXT0_PID_INTEGRAL_DRIVE_MAX, EXT0_PID_INTEGRAL_DRIVE_MIN, EXT0_PID_PGAIN_OR_DEAD_TIME, EXT0_PID_I, EXT0_PID_D, EXT0_PID_MAX, 0, 0
            , 0, 0, 0, EXT0_DECOUPLE_TEST_PERIOD, 0, EXT0_PREHEAT_TEMP
        }
        , ext0_select_cmd, ext0_deselect_cmd, EXT0_EXTRUDER_COOLER_SPEED, 0, 0, 0
    }
};

TemperatureController heatedBedController = {
    PWM_HEATED_BED, HEATED_BED_SENSOR_TYPE, BED_SENSOR_INDEX, HEATED_BED_HEAT_MANAGER, 0, 0, 0, 0, 0, 0
    , 0, HEATED_BED_PID_INTEGRAL_DRIVE_MAX, HEATED_BED_PID_INTEGRAL_DRIVE_MIN, HEATED_BED_PID_PGAIN_OR_DEAD_TIME, HEATED_BED_PID_IGAIN, HEATED_BED_PID_DGAIN, HEATED_BED_PID_MAX, 0, 0
    , 0, 0, 0, HEATED_BED_DECOUPLE_TEST_PERIOD, 0, HEATED_BED_PREHEAT_TEMP
};




#if NUM_TEMPERATURE_LOOPS > 0
TemperatureController *tempController[NUM_TEMPERATURE_LOOPS] = {
    &extruder[0].tempControl , &heatedBedController
};
#endif
