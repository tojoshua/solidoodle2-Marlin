/*
  ZWobble.cpp - A class that manages ZWobble
 
  Copyright (c) 2013 Francesco Santini
  Additional code by Tealvince (source: http://www.soliforum.com/topic/1165/firmware-z-wobble-compensation/)
 
 Grbl is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 Grbl is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "ZWobble.h"

#include "Marlin.h"
#include "planner.h"
#include "math.h"

// minimum distance within which two distances in mm are considered equal
#define TOLERANCE_MM 0.01
#define DISTANCE(_A,_B) abs( (_A) - (_B) )
#define EQUAL_WITHIN_TOLERANCE(_A,_B) (DISTANCE(_A,_B) < TOLERANCE_MM)
#define TWOPI 6.28318530718
#define ZACTUAL_IS_SCALED(_I) (zLut[_I][1] < 0)
#define ZACTUAL(_I) (zLut[_I][1] < 0 ? -zLut[_I][1] * m_scalingFactor : zLut[_I][1])
#define ZROD(_I) zLut[_I][0]
#define SET_ZACTUAL(_I,_V) zLut[_I][1] = _V
#define SET_ZROD(_I,_V) zLut[_I][0] = _V

//===========================================================================

ZWobble zwobble(0.0,0.0,0.0);

//===========================================================================

ZWobble::ZWobble( float _amplitude, float _period, float _phase ) : 
  m_consistent(false),
  lastZ(-1.0),
  lastZRod(-1.0),
  m_scalingFactor(1.0),
  m_sinusoidal(true)
{
  Set( _amplitude, _period, _phase);
}

//===========================================================================
void ZWobble::Set( float _amplitude, float _period, float _phase )
{
  setAmplitude(_amplitude);
  setPeriod(_period);
  setPhase(_phase);
}

boolean ZWobble::areParametersConsistent()
{
  if (!m_sinusoidal)
  {
    m_consistent = true; // parameters are always consistent if lut is not sinusoidal
    return true; // if the model is not sinusoidal, then don't check for consistency
  }
  if (m_puls <= 0 ||
      m_amplitude <= 0 ||
      m_amplitude*m_puls >= 1) // m_amplitude*m_puls must be less than 1 in order for the function to be invertible (otherwise it would mean that the wobble is so much that the axis goes back)
  {
    m_consistent = false; 
    return false; 
  }
  m_consistent = true;
  return true;
}

void ZWobble::setScaledSample(float zRod, float zScaledLength) {
  // We want to be able to correct scaling factor or set it before/after samples, so (ICK) store scaled samples as negative numbers  
  setSample(zRod, -zScaledLength);

  // Make sure we have a non-zero scaling factor
  if (!m_scalingFactor) {
    m_scalingFactor = 1.0;
  }

  // Find two scaled samples close to period
  float period = TWOPI/m_puls;
  int s1 = -1, s2 = -1;  
  
  for (int i=0; i<lutSize; i++) {
    if (ZACTUAL_IS_SCALED(i)) {
      s1 = s2;
      s2 = i;
      if (ZROD(s2) >= period) break;
    }
  }

  // Calculate scaling factor so zact[period] = zrod[period]
  if (s2 >= 0 && ZACTUAL(s2)) {
    // Case 1 - Only one sample
    if (s1 < 0) {
      m_scalingFactor *= ZROD(s2) / ZACTUAL(s2);
    }
    
    // Case 2 - Samples bracketing period (s1 - p - s2): average zact
    else if (ZROD(s2) > period) {
      float gap1 = period - ZROD(s1);
      float gap2 = ZROD(s2) - period;
      float zActPeriod = ZACTUAL(s1) + (ZACTUAL(s2) - ZACTUAL(s1)) * gap1 / (gap1 + gap2);
      
      m_scalingFactor *= period / zActPeriod;      
    }
    
    // Case 3 - Both samples before period (s1 - s2 - p): extrapolate zact
    else {
      float gap1 = ZROD(s2) - ZROD(s1);
      float gap2 = period - ZROD(s2);
      float zActPeriod = ZACTUAL(s2) + (ZACTUAL(s2) - ZACTUAL(s1)) * gap2 / gap1;
      
      m_scalingFactor *= period / zActPeriod;      
    }  
  }
}

void ZWobble::setScalingFactor(float zActualPerScaledLength) {
  m_scalingFactor = zActualPerScaledLength;
}

void ZWobble::setSample(float zRod, float zActual)
{
#ifdef ZWOBBLE_DEBUG
      SERIAL_PROTOCOLPGM("New sample Rod: ");
      SERIAL_PROTOCOL(zRod);
      SERIAL_PROTOCOLPGM(" Act: ");
      SERIAL_PROTOCOL(zActual);
      SERIAL_PROTOCOLLN("");
#endif
  if (m_puls <= 0)
  {
    SERIAL_PROTOCOLLN("You must define a period first (M97 W...)");
    return;
  }
  if (m_sinusoidal)
  {
    m_sinusoidal = false;
    calculateLut(); // initializes the LUT to linear
  }
  insertInLut(zRod, zActual);
}

void ZWobble::insertInLut(float zRod, float zActual)
{
  // check if the given zRod alread exists in LUT
  int i=0;
  
  for (i=0; i<lutSize; i++)
  {
    if (EQUAL_WITHIN_TOLERANCE(zRod, ZROD(i)))
    {
      // replace value
      SET_ZROD(i, zRod);
      SET_ZACTUAL(i, zActual);
      return;
    }
  }
  
  // ok the value does not exist: is there still space in LUT? Insert it
  if (lutSize < STEPS_IN_ZLUT)
  {
    int zPlace = -1;
    for (i=0; i < lutSize; i++)
    {
      if (ZROD(i) > zRod)
      {
	zPlace = i;
	break;
      }
    }
    
    // shift samples after zPlace
    for (i=lutSize; i > zPlace; i--)
    {
      SET_ZROD(i, ZROD(i-1));
      SET_ZACTUAL(i, ZACTUAL(i-1));
    }
    lutSize++; // increase lutSize
    
    // insert sample
    SET_ZROD(zPlace, zRod);
    SET_ZACTUAL(zPlace, zActual);
    return;
  } else
  {
    // lutSize == STEPS_IN_ZLUT: replace the closest point with the new sample
    int zPlace = 0;
    float dist = DISTANCE(zRod,ZROD(zPlace));
    for (int i = 1; i < lutSize; i++)
    {
      if (DISTANCE(zRod, ZROD(i)) < dist)
      {
	zPlace = i;
	dist = DISTANCE(zRod, ZROD(i));
      }
    }
    SET_ZROD(zPlace, zRod);
    SET_ZACTUAL(zPlace, zActual);
  }
}

void ZWobble::initLinearLut()
{
  float period = TWOPI/m_puls;
  lutSize = 2; // only 2 samples originally
  SET_ZROD(0, 0);
  SET_ZACTUAL(0, 0);
  SET_ZROD(1, period);
  SET_ZACTUAL(1, period);
}



// calculate the ZRod -> Zactual LUT using the model Zactual = Zrod + sin(w*Zrod) - this will actually only be used for one period
void ZWobble::calculateLut()
{
  lastZ = -1.0;
  lastZRod = -1.0; // reinitialize memorized Z values since we are changing the model
  if (!areParametersConsistent()) return;
  if (!m_sinusoidal)
  {
    initLinearLut();
    return; // if the model is not sinusoidal, initializes LUT to linear
  }
  lutSize = STEPS_IN_ZLUT;
  float period = TWOPI/m_puls;
  // divide the period in STEPS_IN_ZLUT steps
  float lutStep = period / STEPS_IN_ZLUT;
  for (int i=0; i<STEPS_IN_ZLUT; i++)
  {
    float zRod = lutStep*i;
    SET_ZROD(i, zRod);
    SET_ZACTUAL(i, zRod + m_amplitude*sin(m_puls*zRod));
  }
}

void ZWobble::setAmplitude( float _amplitude )
{
  m_amplitude = _amplitude;
  m_sinusoidal = true; // setAmplitude sets to sinusoidal by default
  calculateLut();
}
  
void ZWobble::setPeriod(float _period )
{
  if (_period <= 0) return;
  m_puls = TWOPI/_period;
  calculateLut();
}

void ZWobble::setPhase(float _phase )
{
  // poor man's modulo operation
  while (_phase > 0)
  {
    _phase -= 360;
  }
  while (_phase < 0)
  {
    _phase += 360;
  }
  
  // phase now will be between 0 and 360
  
  m_phase = (_phase*M_PI/180); // convert phase to radians
}

//===========================================================================
void ZWobble::ReportToSerial()
{
  if (!m_sinusoidal)
  {
    SERIAL_PROTOCOLPGM("Custom wobble function");
  } else
  {
    SERIAL_PROTOCOLPGM("ZWobble Amp (A) ");
    SERIAL_PROTOCOL(m_amplitude);
  }
  SERIAL_PROTOCOLPGM(" phase (P) ");
  SERIAL_PROTOCOL(m_phase); 
  SERIAL_PROTOCOLPGM(" period (W) ");
  SERIAL_PROTOCOL(TWOPI/m_puls);
  SERIAL_PROTOCOLPGM(" puls ");
  SERIAL_PROTOCOL(m_puls);
  if (!areParametersConsistent())
  {
      SERIAL_PROTOCOLPGM(" Warning! Inconsistent parameters!");
  }
  
  SERIAL_PROTOCOLLN("");
  
  if (!m_sinusoidal)
  {
    // print out the LUT
    for (int i=0; i<lutSize; i++)
    {
      SERIAL_PROTOCOLPGM("Rod: ");
      SERIAL_PROTOCOL(ZROD(i));
      SERIAL_PROTOCOLPGM(" Act: ");
      SERIAL_PROTOCOL(ZACTUAL(i));
      
      int delta = (ZACTUAL(i) - ZROD(i)) * 200 + 20;
      for (int j=0; j<delta; j++) {
        SERIAL_PROTOCOLPGM(" ");
      }
      SERIAL_PROTOCOLLN("  +");
    }
  }

}

float ZWobble::findInLut(float z)
{
  if (z>=ZACTUAL(lutSize-1))
  {
    return ZROD(lutSize-1);
  }
  
  if (z<=ZACTUAL(0))
  {
    return ZROD(0);
  }
  
  int i;
  for (i=0; i<lutSize; i++)
  {
    if (ZACTUAL(i) > z)
    {
      break;
    }
  }
  
  float invZDist = 1 / (ZACTUAL(i)-ZACTUAL(i-1)); // distance between Z steps
  
  float interpZ = (ZROD(i-1) * (ZACTUAL(i)-z) + ZROD(i) * (z - ZACTUAL(i-1))) * invZDist; // linear interpolation between neighboring Z values
  
  return interpZ;
}

// Find the Z value to be given to the "rod" in order to obtain the desired Z
float ZWobble::findZRod(float z)
{
  int nCycle = 0;
  float identicalZ = -m_phase/m_puls;
  
  // find the last point in which the two Z are identical: this happens every (2kPI-phase)/w
  while (identicalZ <= z)
  {
    identicalZ = (TWOPI*(++nCycle)-m_phase)/m_puls;
  } 
  
  // find Z again using the previous cycle
  identicalZ = (TWOPI*(nCycle-1)-m_phase)/m_puls;
  
  float deltaZa = z-identicalZ;
  
  // find deltaZRod by linear interpolation of the lookup table
  float deltaZrod = findInLut(deltaZa);
  return identicalZ + deltaZrod;
}

//===========================================================================
// insert a plan_buffer_line if required to handle any hysteresis
long ZWobble::InsertCorrection(const float targetZ)
{
  
  if (!m_consistent) return 0; // don't go through consistency checks all the time; just check one bool
  
  float originZ = (float)position[Z_AXIS]/axis_steps_per_unit[Z_AXIS];
  
  if (originZ < ZWOBBLE_MIN_Z || targetZ < ZWOBBLE_MIN_Z) return 0;

#ifdef ZWOBBLE_DEBUG
  SERIAL_PROTOCOLPGM("Origin: ");
  SERIAL_PROTOCOL(originZ); 
   
  SERIAL_PROTOCOLPGM(" Target: ");
  SERIAL_PROTOCOL(targetZ); 
#endif
  
  if (EQUAL_WITHIN_TOLERANCE(originZ, targetZ)) return 0; // if there is no Z move, do nothing

  float originZRod;

  // there is a high chance that the origin Z is the same as the last target Z: skip one iteration of the algorithm if possible
  if (originZ == lastZ)
    originZRod = lastZRod;
  else
    originZRod = findZRod(originZ);
  
#ifdef ZWOBBLE_DEBUG
  SERIAL_PROTOCOLPGM(" Origin rod: ");
  SERIAL_PROTOCOL(originZRod); 
#endif
  
  float targetZRod = findZRod(targetZ);
  

#ifdef ZWOBBLE_DEBUG
  SERIAL_PROTOCOLPGM(" Target Rod: ");
  SERIAL_PROTOCOL(targetZRod); 
#endif
  // difference in steps between the correct movement (originZRod->targetZRod) and the planned movement
  long stepDiff = lround((targetZRod-originZRod)*axis_steps_per_unit[Z_AXIS]) - (lround(targetZ*axis_steps_per_unit[Z_AXIS]) - position[Z_AXIS]);
  
#ifdef ZWOBBLE_DEBUG
  SERIAL_PROTOCOLPGM(" stepDiff: ");
  SERIAL_PROTOCOL(stepDiff); 
  SERIAL_PROTOCOLLN("");  
#endif
  lastZ = targetZ;
  lastZRod = targetZRod;
  
#ifdef ZWOBBLE_USE_FAKEPOS
  // don't adjust if target posizion is less than 0
  if (position[Z_AXIS] - stepDiff < 0)
    stepDiff = 0;
  else
    position[Z_AXIS] -= stepDiff;
#endif
  
  return stepDiff;
 
//  
 
}

