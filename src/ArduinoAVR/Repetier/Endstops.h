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

*/

#ifndef _ENDSTOPS_H
#define _ENDSTOPS_H

#define ENDSTOP_X_MIN_ID 1
#define ENDSTOP_X_MAX_ID 2
#define ENDSTOP_Y_MIN_ID 4
#define ENDSTOP_Y_MAX_ID 8
#define ENDSTOP_Z_MIN_ID 16
#define ENDSTOP_Z_MAX_ID 32
#define ENDSTOP_Z_PROBE_ID 128

class Endstops {
    static flag8_t lastState;
    static flag8_t lastRead;
    static flag8_t accumulator;
public:
    static void update();
    static void report();
    static void setup();
    static INLINE bool anyXYZMax() {
        return (lastState & (ENDSTOP_X_MAX_ID | ENDSTOP_Y_MAX_ID | ENDSTOP_Z_MAX_ID)) != 0;
    }
    static INLINE bool anyXYZ() {
        return (lastState & (ENDSTOP_X_MAX_ID | ENDSTOP_Y_MAX_ID | ENDSTOP_Z_MAX_ID | ENDSTOP_X_MIN_ID | ENDSTOP_Y_MIN_ID | ENDSTOP_Z_MIN_ID )) != 0;
    }
    static INLINE bool anyEndstopHit() {
        return lastState != 0;
    }
    static INLINE void resetAccumulator() {
        accumulator = 0;
    }
    static INLINE void fillFromAccumulator() {
        lastState = accumulator;
    }
    static INLINE bool xMin() {
#if (X_MIN_PIN > -1) && MIN_HARDWARE_ENDSTOP_X
        return (lastState & ENDSTOP_X_MIN_ID) != 0;
#else
        return false;
#endif
    }
    static INLINE bool xMax() {
#if (X_MAX_PIN > -1) && MAX_HARDWARE_ENDSTOP_X
        return (lastState & ENDSTOP_X_MAX_ID) != 0;
#else
        return false;
#endif
    }
    static INLINE bool yMin() {
#if (Y_MIN_PIN > -1) && MIN_HARDWARE_ENDSTOP_Y
        return (lastState & ENDSTOP_Y_MIN_ID) != 0;
#else
        return false;
#endif
    }
    static INLINE bool yMax() {
#if (Y_MAX_PIN > -1) && MAX_HARDWARE_ENDSTOP_Y
        return (lastState & ENDSTOP_Y_MAX_ID) != 0;
#else
        return false;
#endif
    }
    static INLINE bool zMin() {
#if (Z_MIN_PIN > -1) && MIN_HARDWARE_ENDSTOP_Z
        return (lastState & ENDSTOP_Z_MIN_ID) != 0;
#else
        return false;
#endif
    }
    static INLINE bool zMax() {
#if (Z_MAX_PIN > -1) && MAX_HARDWARE_ENDSTOP_Z
        return (lastState & ENDSTOP_Z_MAX_ID) != 0;
#else
        return false;
#endif
    }
    static INLINE bool zProbe() {
#if FEATURE_Z_PROBE
        return (lastState & ENDSTOP_Z_PROBE_ID) != 0;
#else
        return false;
#endif
    }
};

#endif
