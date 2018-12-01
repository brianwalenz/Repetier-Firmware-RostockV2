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

  Functions in this file are used to communicate using ascii or repetier protocol.
*/

#include "Repetier.h"
#include "HAL.h"

#include "rmath.h"



#define DISTORTION_CORRECTION_POINTS  5
/** Max. distortion value to enter. Used to prevent dangerous errors with big values. */
#define DISTORTION_LIMIT_TO 2
/* For delta printers you simply define the measured radius around origin */
#define DISTORTION_CORRECTION_R       80
/* For all others you define the correction rectangle by setting the min/max coordinates. Make sure the the probe can reach all points! */
#define DISTORTION_XMIN 10
#define DISTORTION_YMIN 10
#define DISTORTION_XMAX 190
#define DISTORTION_YMAX 190

/** Uses EEPROM instead of ram. Allows bigger matrix (up to 22x22) without any ram cost.
    Especially on arm based systems with cached EEPROM it is good, on AVR it has a small
    performance penalty.
*/
#define DISTORTION_PERMANENT          1
/** Correction computation is not a cheap operation and changes are only small. So it
    is not necessary to update it for every sub-line computed. For example lets take DELTA_SEGMENTS_PER_SECOND_PRINT = 150
    and fastest print speed 100 mm/s. So we have a maximum segment length of 100/150 = 0.66 mm.
    Now lats say our point field is 200 x 200 mm with 9 x 9 points. So between 2 points we have
    200 / (9-1) = 25 mm. So we need at least 25 / 0.66 = 37 lines to move to the next measuring
    point. So updating correction every 15 calls gives us at least 2 updates between the
    measured points.
    NOTE: Explicit z changes will always trigger an update!
*/
#define DISTORTION_UPDATE_FREQUENCY   15
/** z distortion degrades to 0 from this height on. You should start after the first layer to get
    best bonding with surface. */
#define DISTORTION_START_DEGRADE 0.5
/** z distortion correction gets down to 0 at this height. */
#define DISTORTION_END_HEIGHT 1.5
/** If your corners measurement points are not measurable with given radius, you can
    set this to 1. It then omits the outer measurement points allowing a larger correction area.*/
#define DISTORTION_EXTRAPOLATE_CORNERS 0




#if DISTORTION_CORRECTION

Distortion Printer::distortion;

void Printer::measureDistortion(void) {
  prepareForProbing();
#if defined(Z_PROBE_MIN_TEMPERATURE) && Z_PROBE_MIN_TEMPERATURE && Z_PROBE_REQUIRES_HEATING
  float actTemp[NUM_EXTRUDER];
  for(int i = 0; i < NUM_EXTRUDER; i++)
    actTemp[i] = extruder[i].tempControl.targetTemperatureC;
  Printer::moveToReal(IGNORE_COORDINATE, IGNORE_COORDINATE, RMath::max(EEPROM::zProbeHeight(), static_cast<float>(ZHOME_HEAT_HEIGHT)), IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
  Commands::waitUntilEndOfAllMoves();
#if ZHOME_HEAT_ALL
  for(int i = 0; i < NUM_EXTRUDER; i++) {
    Extruder::setTemperatureForExtruder(RMath::max(actTemp[i], static_cast<float>(ZPROBE_MIN_TEMPERATURE)), i, false, false);
  }
  for(int i = 0; i < NUM_EXTRUDER; i++) {
    if(extruder[i].tempControl.currentTemperatureC < ZPROBE_MIN_TEMPERATURE)
      Extruder::setTemperatureForExtruder(RMath::max(actTemp[i], static_cast<float>(ZPROBE_MIN_TEMPERATURE)), i, false, true);
  }
#else
  if(extruder[Extruder::current->id].tempControl.currentTemperatureC < ZPROBE_MIN_TEMPERATURE)
    Extruder::setTemperatureForExtruder(RMath::max(actTemp[Extruder::current->id], static_cast<float>(ZPROBE_MIN_TEMPERATURE)), Extruder::current->id, false, true);
#endif
#endif
  float oldFeedrate = Printer::feedrate;


  Printer::coordinateOffset[X_AXIS] = Printer::coordinateOffset[Y_AXIS] = Printer::coordinateOffset[Z_AXIS] = 0;

  if(!distortion.measure()) {
    commandQueue.fatalError(PSTR("G33 failed!"));
    return;
  }
  Printer::feedrate = oldFeedrate;
#if defined(Z_PROBE_MIN_TEMPERATURE) && Z_PROBE_MIN_TEMPERATURE && Z_PROBE_REQUIRES_HEATING
#if ZHOME_HEAT_ALL
  for(int i = 0; i < NUM_EXTRUDER; i++)
    Extruder::setTemperatureForExtruder(actTemp[i], i, false, false);
  for(int i = 0; i < NUM_EXTRUDER; i++)
    Extruder::setTemperatureForExtruder(actTemp[i], i, false, actTemp[i] > MAX_ROOM_TEMPERATURE);
#else
  Extruder::setTemperatureForExtruder(actTemp[Extruder::current->id], Extruder::current->id, false, actTemp[Extruder::current->id] > MAX_ROOM_TEMPERATURE);
#endif
#endif
}

Distortion::Distortion() {
}

void Distortion::init() {
  updateDerived();
#if !DISTORTION_PERMANENT
  resetCorrection();
#endif
  enabled = EEPROM::isZCorrectionEnabled();
  Com::printF(PSTR("zDistortionCorrection:"), (int)enabled);
  Com::printF(PSTR("\n"));
}

void Distortion::updateDerived() {
  step = (2 * Printer::axisStepsPerMM[Z_AXIS] * DISTORTION_CORRECTION_R) / (DISTORTION_CORRECTION_POINTS - 1.0f);
  radiusCorrectionSteps = DISTORTION_CORRECTION_R * Printer::axisStepsPerMM[Z_AXIS];

  zStart = DISTORTION_START_DEGRADE * Printer::axisStepsPerMM[Z_AXIS] + Printer::zMinSteps;
  zEnd = DISTORTION_END_HEIGHT * Printer::axisStepsPerMM[Z_AXIS] + Printer::zMinSteps;
}

void Distortion::enable(bool permanent) {
  enabled = true;
#if DISTORTION_PERMANENT
  if(permanent)
    EEPROM::setZCorrectionEnabled(enabled);
#endif
  Com::printF(PSTR("Z correction enabled\n"));
}

void Distortion::disable(bool permanent) {
  enabled = false;
#if DISTORTION_PERMANENT
  if(permanent)
    EEPROM::setZCorrectionEnabled(enabled);
#endif

  Printer::updateCurrentPosition(false);
  Com::printF(PSTR("Z correction disabled\n"));
}

void Distortion::reportStatus() {
  Com::printF(enabled ? PSTR("Z correction enabled\n") : PSTR("Z correction disabled\n"));
}

void Distortion::resetCorrection(void) {
  Com::printF(PSTR("INFO: Resetting Z correction\n"));
  for(int i = 0; i < DISTORTION_CORRECTION_POINTS * DISTORTION_CORRECTION_POINTS; i++)
    setMatrix(0, i);
}

int Distortion::matrixIndex(int8_t x, int8_t y) const {
  return static_cast<int>(y) * DISTORTION_CORRECTION_POINTS + x;
}

int32_t Distortion::getMatrix(int index) const {
#if DISTORTION_PERMANENT
  return EEPROM::getZCorrection(index);
#else
  return matrix[index];
#endif
}
void Distortion::setMatrix(int32_t val, int index) {
#if DISTORTION_PERMANENT
  EEPROM::setZCorrection(val, index);
#else
  matrix[index] = val;
#endif
}

bool Distortion::isCorner(int8_t i, int8_t j) const {
  return (i == 0 || i == DISTORTION_CORRECTION_POINTS - 1)
    && (j == 0 || j == DISTORTION_CORRECTION_POINTS - 1);
}

/**
   Extrapolates the changes from p1 to p2 to p3 which has the same distance as p1-p2.
*/
inline int32_t Distortion::extrapolatePoint(int8_t x1, int8_t y1, int8_t x2, int8_t y2) const {
  return 2 * getMatrix(matrixIndex(x2, y2)) - getMatrix(matrixIndex(x1, y1));
}

void Distortion::extrapolateCorner(int8_t x, int8_t y, int8_t dx, int8_t dy) {
  setMatrix((extrapolatePoint(x + 2 * dx, y, x + dx, y) + extrapolatePoint(x, y + 2 * dy, x, y + dy)) / 2.0,
            matrixIndex(x, y));
}

void Distortion::extrapolateCorners() {
  const int8_t m = DISTORTION_CORRECTION_POINTS - 1;
  extrapolateCorner(0, 0, 1, 1);
  extrapolateCorner(0, m, 1, -1);
  extrapolateCorner(m, 0, -1, 1);
  extrapolateCorner(m, m, -1, -1);
}

bool Distortion::measure(void) {
  int8_t ix, iy;

  disable(false);
	Printer::prepareForProbing();
  float z = RMath::max(EEPROM::zProbeBedDistance() + (EEPROM::zProbeHeight() > 0 ? EEPROM::zProbeHeight() : 0), static_cast<float>(ZHOME_HEAT_HEIGHT)); //EEPROM::zProbeBedDistance() + (EEPROM::zProbeHeight() > 0 ? EEPROM::zProbeHeight() : 0);
  Com::printF(PSTR("Reference Z for measurement:"), z, 3);
  Com::printF(PSTR("\n"));
  updateDerived();
  /*
    // It is not possible to go to the edges at the top, also users try
    // it often and wonder why the coordinate system is then wrong.
    // For that reason we ensure a correct behavior by code.
    Printer::homeAxis(true, true, true);
    Printer::moveTo(IGNORE_COORDINATE, IGNORE_COORDINATE, EEPROM::zProbeBedDistance() + (EEPROM::zProbeHeight() > 0 ? EEPROM::zProbeHeight() : 0), IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
  */
  //Com::printF(PSTR("radiusCorr:"), radiusCorrectionSteps);
  //Com::printF(PSTR("\n"));
  //Com::printF(PSTR("steps:"), step);
  //Com::printF(PSTR("\n"));
  int32_t zCorrection = 0;

  Printer::startProbing(true);
  Printer::moveToReal(IGNORE_COORDINATE, IGNORE_COORDINATE, z, IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
  for (iy = DISTORTION_CORRECTION_POINTS - 1; iy >= 0; iy--)
    for (ix = 0; ix < DISTORTION_CORRECTION_POINTS; ix++) {
#if DISTORTION_EXTRAPOLATE_CORNERS
      if (isCorner(ix, iy)) continue;
#endif
      float mtx = Printer::invAxisStepsPerMM[X_AXIS] * (ix * step - radiusCorrectionSteps);
      float mty = Printer::invAxisStepsPerMM[Y_AXIS] * (iy * step - radiusCorrectionSteps);
      //Com::printF(PSTR("mx "),mtx);
      //Com::printF(PSTR("my "),mty);
      //Com::printF(PSTR("ix "),(int)ix);
      //Com::printF(PSTR("iy "),(int)iy);
      //Com::printF(PSTR("\n"));

      Printer::moveToReal(mtx, mty, z, IGNORE_COORDINATE, EEPROM::zProbeXYSpeed());
      float zp = Printer::runZProbe(false, false, Z_PROBE_REPETITIONS);
#if defined(DISTORTION_LIMIT_TO) && DISTORTION_LIMIT_TO != 0
      if(zp == ILLEGAL_Z_PROBE || fabs(z - zp + zCorrection * Printer::invAxisStepsPerMM[Z_AXIS]) > DISTORTION_LIMIT_TO) {
#else
        if(zp == ILLEGAL_Z_PROBE) {
#endif
          Com::printF(PSTR("Stopping distortion measurement due to errors.\n"));
          Printer::finishProbing();
          return false;
        }
        setMatrix(floor(0.5f + Printer::axisStepsPerMM[Z_AXIS] * (z - zp)) + zCorrection,
                  matrixIndex(ix, iy));
      }
      Printer::finishProbing();
#if DISTORTION_EXTRAPOLATE_CORNERS
      extrapolateCorners();
#endif
      // make average center
      // Disabled since we can use grid measurement to get average plane if that is what we want.
      // Shifting z with each measuring is a pain and can result in unexpected behavior.
      /*
        float sum = 0;
        for(int k = 0;k < DISTORTION_CORRECTION_POINTS * DISTORTION_CORRECTION_POINTS; k++)
        sum += getMatrix(k);
        sum /= static_cast<float>(DISTORTION_CORRECTION_POINTS * DISTORTION_CORRECTION_POINTS);
        for(int k = 0;k < DISTORTION_CORRECTION_POINTS * DISTORTION_CORRECTION_POINTS; k++)
        setMatrix(getMatrix(k) - sum, k);
        Printer::zLength -= sum * Printer::invAxisStepsPerMM[Z_AXIS];
      */
      EEPROM::storeDataIntoEEPROM();
      // print matrix
      Com::printF(PSTR("INFO: Distortion correction matrix:\n"));
      for (iy = DISTORTION_CORRECTION_POINTS - 1; iy >= 0 ; iy--) {
        for(ix = 0; ix < DISTORTION_CORRECTION_POINTS; ix++)
          Com::printF(ix ? PSTR(", ") : PSTR(""), getMatrix(matrixIndex(ix, iy)));
        Com::printF(PSTR("\n"));
      }
      showMatrix();
      enable(true);
      return true;
      //Printer::homeAxis(false, false, true);
    }


  int32_t Distortion::correct(int32_t x, int32_t y, int32_t z) const {
    if (!enabled || Printer::isZProbingActive()) {
      return 0;
    }
    z += Printer::offsetZ * Printer::axisStepsPerMM[Z_AXIS] - Printer::zMinSteps;
    if (z > zEnd) {
      /* Com::printF(PSTR("NoCor z:"),z);
         Com::printF(PSTR(" zEnd:"),zEnd);
         Com::printF(PSTR(" en:"),(int)enabled);
         Com::printF(PSTR(" zp:"),(int)Printer::isZProbingActive());*/
      //Com::printF(PSTR("\n"));
      return 0;
    }
    x -= Printer::offsetX * Printer::axisStepsPerMM[X_AXIS]; // correct active tool offset
    y -= Printer::offsetY * Printer::axisStepsPerMM[Y_AXIS];
    if(false) {
      Com::printF(PSTR("correcting ("), x);
      Com::printF(PSTR(","), y);
    }
    x += radiusCorrectionSteps;
    y += radiusCorrectionSteps;
    int32_t fxFloor = (x - (x < 0 ? step - 1 : 0)) / step; // special case floor for negative integers!
    int32_t fyFloor = (y - (y < 0 ? step - 1 : 0)) / step;
    // indexes to the matrix

    // position between cells of matrix, range=0 to 1 - outside of the matrix the value will be outside this range and the value will be extrapolated

    int32_t fx = x - fxFloor * step; // Grid normalized coordinates
    int32_t fy = y - fyFloor * step;
    if (fxFloor < 0) {
      fxFloor = 0;
      fx = 0;
    } else if (fxFloor >= DISTORTION_CORRECTION_POINTS - 1) {
      fxFloor = DISTORTION_CORRECTION_POINTS - 2;
      fx = step;
    }
    if (fyFloor < 0) {
      fyFloor = 0;
      fy = 0;
    } else if (fyFloor >= DISTORTION_CORRECTION_POINTS - 1) {
      fyFloor = DISTORTION_CORRECTION_POINTS - 2;
      fy = step;
    }

    int32_t idx11 = matrixIndex(fxFloor, fyFloor);
    int32_t m11 = getMatrix(idx11), m12 = getMatrix(idx11 + 1);
    int32_t m21 = getMatrix(idx11 + DISTORTION_CORRECTION_POINTS);
    int32_t m22 = getMatrix(idx11 + DISTORTION_CORRECTION_POINTS + 1);
    int32_t zx1 = m11 + ((m12 - m11) * fx) / step;
    int32_t zx2 = m21 + ((m22 - m21) * fx) / step;
    int32_t correction_z = zx1 + ((zx2 - zx1) * fy) / step;
    /*if(z == Printer::zMinSteps) {
      Com::printF(PSTR("DT M11:"),m11);
      Com::printF(PSTR(" M12:"),m12);
      Com::printF(PSTR(" M21:"),m21);
      Com::printF(PSTR(" M22:"),m22);
      Com::printF(PSTR(" FX:"),fx);
      Com::printF(PSTR(" FY:"),fy);
      Com::printF(PSTR(" FFX:"),fxFloor);
      Com::printF(PSTR(" FFY:"),fyFloor);
      Com::printF(PSTR(" XP:"),x-radiusCorrectionSteps);
      Com::printF(PSTR(" Yp:"),y-radiusCorrectionSteps);
      Com::printF(PSTR(" STEP:"),step);
      Com::printF(PSTR(" ZCOR:"),correction_z);
      Com::printF(PSTR("\n"));
      }*/

    /* if(false) {
       Com::printF(PSTR(") by "), correction_z);
       Com::printF(PSTR(" ix= "), fxFloor);
       Com::printF(PSTR(" fx= "), (float)fx/(float)xCorrectionSteps,3);
       Com::printF(PSTR(" iy= "), fyFloor);
       Com::printF(PSTR(" fy= "), (float)fy/(float)yCorrectionSteps,3);
       Com::printF(PSTR("\n"));
       }*/
    if (z > zStart && z > Printer::zMinSteps)
      //All variables are type int. For calculation we need float values
      correction_z = (correction_z * static_cast<float>(zEnd - z) / (zEnd - zStart));
    /* if(correction_z > 20 || correction_z < -20) {
       Com::printF(PSTR("Corr. error too big:"),correction_z);
       Com::printF(PSTR("\n"));
       Com::printF(PSTR("fxf"),(int)fxFloor);
       Com::printF(PSTR(" fyf"),(int)fyFloor);
       Com::printF(PSTR(" fx"),fx);
       Com::printF(PSTR(" fy"),fy);
       Com::printF(PSTR(" x"),x);
       Com::printF(PSTR(" y"),y);
       Com::printF(PSTR("\n"));
       Com::printF(PSTR(" m11:"),m11);
       Com::printF(PSTR(" m12:"),m12);
       Com::printF(PSTR(" m21:"),m21);
       Com::printF(PSTR(" m22:"),m22);
       Com::printF(PSTR(" step:"),step);
       Com::printF(PSTR("\n"));
       correction_z = 0;
       }*/
    return correction_z;
  }

  void Distortion::set(float x, float y, float z) {
#if defined(DISTORTION_LIMIT_TO) && DISTORTION_LIMIT_TO != 0
    if(fabs(z) > DISTORTION_LIMIT_TO) {
      Com::printF(PSTR("WARNING: Max. distortion value exceeded - not setting this value.\n"));
      return;
    }
#endif

    int ix = (x * Printer::axisStepsPerMM[Z_AXIS] + radiusCorrectionSteps + step / 2) / step;
    int iy = (y * Printer::axisStepsPerMM[Z_AXIS] + radiusCorrectionSteps + step / 2) / step;

    if(ix < 0) ix = 0;
    if(iy < 0) iy = 0;
    if(ix >= DISTORTION_CORRECTION_POINTS - 1) ix = DISTORTION_CORRECTION_POINTS - 1;
    if(iy >= DISTORTION_CORRECTION_POINTS - 1) iy = DISTORTION_CORRECTION_POINTS - 1;
    int32_t idx = matrixIndex(ix, iy);
    setMatrix(z * Printer::axisStepsPerMM[Z_AXIS], idx);
  }

  void Distortion::showMatrix() {
    for(int ix = 0; ix < DISTORTION_CORRECTION_POINTS; ix++) {
      for(int iy = 0; iy < DISTORTION_CORRECTION_POINTS; iy++) {

        float x = (-radiusCorrectionSteps + ix * step) * Printer::invAxisStepsPerMM[Z_AXIS];
        float y = (-radiusCorrectionSteps + iy * step) * Printer::invAxisStepsPerMM[Z_AXIS];

        int32_t idx = matrixIndex(ix, iy);
        float z = getMatrix(idx) * Printer::invAxisStepsPerMM[Z_AXIS];
        Com::printF(PSTR("G33 X"), x, 2);
        Com::printF(PSTR(" Y"), y, 2);
        Com::printF(PSTR(" Z"), z, 3);
        Com::printF(PSTR("\n"));
      }
    }
  }

#endif // DISTORTION_CORRECTION
