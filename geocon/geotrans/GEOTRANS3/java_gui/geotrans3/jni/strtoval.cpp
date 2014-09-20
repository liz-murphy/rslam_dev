// CLASSIFICATION: UNCLASSIFIED

/******************************************************************************
* Filename        : strtoval.cpp
*
* MODIFICATION HISTORY:
*
* DATE      NAME        DR#               DESCRIPTION
*
* 03/29/11  S. Gillis   BAEts28564        Fixed Windows memory crash
******************************************************************************/

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <math.h>
#include "strtoval.h"
#include "CoordinateConversionService.h"

#define Lat_String 1
#define Long_String 2

int leading_zeros = 0;
char Lat_Long_Sep = ' ';
Range Long_Range = _180_180;
Precision::Enum Lat_Long_Prec = Precision::tenthOfSecond;

void Show_Leading_Zeros(int lz)
{
  leading_zeros = lz;
}

void Set_Separator(char sep)
{
  Lat_Long_Sep = sep;
}

void Set_Long_Range(Range range)
{
  Long_Range = range;
}

char Get_Separator()
{
  return (Lat_Long_Sep);
}

Range Get_Long_Range()
{
  return (Long_Range);
}

void Set_Lat_Long_Precision(Precision::Enum precis)
{
  Lat_Long_Prec = precis;
}


SVC_Status String_to_Projection(const char *str, MSP::CCS::CoordinateType::Enum* val)
{
  SVC_Status error_Code = SVC_Success;
  /* Note: any name that is a substring of another name must come before that name */
  if (strstr("GEODETIC", str))
  {
    *val = CoordinateType::geodetic;
  }
  else if (strstr("GEOREF", str))
  {
    *val = CoordinateType::georef;
  }
  else if (strstr("GEOCENTRIC", str))
  {
    *val = CoordinateType::geocentric;
  }
  else if (strstr("LOCAL CARTESIAN", str))
  {
    *val = CoordinateType::localCartesian;
  }
  else if (strstr("MILITARY GRID REFERENCE SYSTEM (MGRS)", str))
  {
    *val = CoordinateType::militaryGridReferenceSystem;
  }
  else if (strstr("UNITED STATES NATIONAL GRID (USNG)", str))
  {
    *val = CoordinateType::usNationalGrid;
  }
  else if ((strstr("MERCATOR", str)) ||
           (strstr("MERCATOR (STANDARD PARALLEL)", str)))
  {
    *val = CoordinateType::mercatorStandardParallel;
  }
  else if (strstr("MERCATOR (SCALE FACTOR)", str))
  {
    *val = CoordinateType::mercatorScaleFactor;
  }
  else if (strstr("OBLIQUE MERCATOR", str))
  {
    *val = CoordinateType::obliqueMercator;
  }
  else if (strstr("TRANSVERSE MERCATOR", str))
  {
    *val = CoordinateType::transverseMercator;
  }
  else if (strstr("UNIVERSAL TRANSVERSE MERCATOR (UTM)", str))
  {
    *val = CoordinateType::universalTransverseMercator;
  }
  else if (strstr("STEREOGRAPHIC", str))
  {
    *val = CoordinateType::stereographic;
  }
  else if ((strstr("POLAR STEREOGRAPHIC", str)) ||
           (strstr("POLAR STEREOGRAPHIC (STANDARD PARALLEL)", str)))
  {
    *val = CoordinateType::polarStereographicStandardParallel;
  }
  else if (strstr("POLAR STEREOGRAPHIC (SCALE FACTOR)", str))
  {
    *val = CoordinateType::polarStereographicScaleFactor;
  }
  else if (strstr("UNIVERSAL POLAR STEREOGRAPHIC (UPS)", str))
  {
    *val = CoordinateType::universalPolarStereographic;
  }
  else if (strstr("ALBERS EQUAL AREA CONIC", str))
  {
    *val = CoordinateType::albersEqualAreaConic;
  }
  else if (strstr("AZIMUTHAL EQUIDISTANT", str))
  {
    *val = CoordinateType::azimuthalEquidistant;
  }
  else if (strstr("BONNE", str))
  {
    *val = CoordinateType::bonne;
  }
  else if (strstr("BRITISH NATIONAL GRID (BNG)", str))
  {
    *val = CoordinateType::britishNationalGrid;
  }
  else if (strstr("CASSINI", str))
  {
    *val = CoordinateType::cassini;
  }
  else if (strstr("ECKERT IV", str))
  {
    *val = CoordinateType::eckert4;
  }
  else if (strstr("ECKERT VI", str))
  {
    *val = CoordinateType::eckert6;
  }
  else if (strstr("EQUIDISTANT CYLINDRICAL", str))
  {
    *val = CoordinateType::equidistantCylindrical;
  }
  else if (strstr("GLOBAL AREA REFERENCE SYSTEM (GARS)", str))
  {
    *val = CoordinateType::globalAreaReferenceSystem;
  }
  else if (strstr("GNOMONIC", str))
  {
    *val = CoordinateType::gnomonic;
  }
  else if ((strstr("LAMBERT CONFORMAL CONIC", str)) ||
           (strstr("LAMBERT CONFORMAL CONIC (2 PARALLEL)", str)) ||
           (strstr("LAMBERT CONFORMAL CONIC (2 STANDARD PARALLEL)", str)))
  {
    *val = CoordinateType::lambertConformalConic2Parallels;
  }
  else if ((strstr("LAMBERT CONFORMAL CONIC (1 PARALLEL)", str)) ||
           (strstr("LAMBERT CONFORMAL CONIC (1 STANDARD PARALLEL)", str)))
  {
    *val = CoordinateType::lambertConformalConic1Parallel;
  }
  else if (strstr("MILLER CYLINDRICAL", str))
  {
    *val = CoordinateType::millerCylindrical;
  }
  else if (strstr("MOLLWEIDE", str))
  {
    *val = CoordinateType::mollweide;
  }
  else if ((strstr("NEY'S (MODIFIED LAMBERT CONFORMAL CONIC)", str)) ||
           (strstr("NEYS (MODIFIED LAMBERT CONFORMAL CONIC)", str)))
  {
    *val = CoordinateType::neys;
  }
  else if (strstr("NEW ZEALAND MAP GRID (NZMG)", str))
  {
    *val = CoordinateType::newZealandMapGrid;
  }
  else if (strstr("ORTHOGRAPHIC", str))
  {
    *val = CoordinateType::orthographic;
  }
  else if (strstr("POLYCONIC", str))
  {
    *val = CoordinateType::polyconic;
  }
  else if (strstr("SINUSOIDAL", str))
  {
    *val = CoordinateType::sinusoidal;
  }
  else if (strstr("CYLINDRICAL EQUAL AREA", str))
  {
    *val = CoordinateType::cylindricalEqualArea;
  }
  else if (strstr("TRANSVERSE CYLINDRICAL EQUAL AREA", str))
  {
    *val = CoordinateType::transverseCylindricalEqualArea;
  }
  else if (strstr("VAN DER GRINTEN", str))
  {
    *val = CoordinateType::vanDerGrinten;
  }
  else
  {
    error_Code = SVC_Invalid_Projection_String;
  }

  return error_Code;
}


SVC_Status Projection_to_String(const CoordinateType::Enum val, char str[32])
{
  SVC_Status error_Code = SVC_Success;

  switch (val)
  {
  case CoordinateType::geodetic:
    {
      strcpy(str, "Geodetic");
      break;
    }
  case CoordinateType::georef:
    {
      strcpy(str, "GEOREF");
      break;
    }
  case CoordinateType::geocentric:
    {
      strcpy(str, "Geocentric");
      break;
    }
  case CoordinateType::localCartesian:
    {
      strcpy(str, "Local Cartesian");
      break;
    }
  case CoordinateType::militaryGridReferenceSystem:
    {
      strcpy(str, "Military Grid Reference System (MGRS)");
      break;
    }
  case CoordinateType::usNationalGrid:
    {
      strcpy(str, "United States National Grid (USNG)");
      break;
    }
  case CoordinateType::universalTransverseMercator:
    {
      strcpy(str, "Universal Transverse Mercator (UTM)");
      break;
    }
  case CoordinateType::universalPolarStereographic:
    {
      strcpy(str, "Universal Polar Stereographic (UPS)");
      break;
    }
  case CoordinateType::albersEqualAreaConic:
    {
      strcpy(str, "Albers Equal Area Conic");
      break;
    }
  case CoordinateType::azimuthalEquidistant:
    {
      strcpy(str, "Azimuthal Equidistant");
      break;
    }
  case CoordinateType::britishNationalGrid:
    {
      strcpy(str, "British National Grid (BNG)");
      break;
    }
  case CoordinateType::bonne:
    {
      strcpy(str, "Bonne");
      break;
    }
  case CoordinateType::cassini:
    {
      strcpy(str, "Cassini");
      break;
    }
  case CoordinateType::cylindricalEqualArea:
    {
      strcpy(str, "Cylindrical Equal Area");
      break;
    }
  case CoordinateType::eckert4:
    {
      strcpy(str, "Eckert IV");
      break;
    }
  case CoordinateType::eckert6:
    {
      strcpy(str, "Eckert VI");
      break;
    }
  case CoordinateType::equidistantCylindrical:
    {
      strcpy(str, "Equidistant Cylindrical");
      break;
    }
  case CoordinateType::globalAreaReferenceSystem:
    {
      strcpy(str, "Global Area Reference System (GARS)");
      break;
    }
  case CoordinateType::gnomonic:
    {
      strcpy(str, "Gnomonic");
      break;
    }
  case CoordinateType::lambertConformalConic1Parallel:
    {
      strcpy(str, "Lambert Conformal Conic (1 Standard Parallel)");
      break;
    }
  case CoordinateType::lambertConformalConic2Parallels:
    {
      strcpy(str, "Lambert Conformal Conic (2 Standard Parallel)");
      break;
    }
  case CoordinateType::mercatorStandardParallel:
    {
      strcpy(str, "Mercator (Standard Parallel)");
      break;
    }
  case CoordinateType::mercatorScaleFactor:
    {
      strcpy(str, "Mercator (Scale Factor)");
      break;
    }
  case CoordinateType::millerCylindrical:
    {
      strcpy(str, "Miller Cylindrical");
      break;
    }
  case CoordinateType::mollweide:
    {
      strcpy(str, "Mollweide");
      break;
    }
  case CoordinateType::neys:
    {
      strcpy(str, "Ney's (Modified Lambert Conformal Conic)");
      break;
    }
  case CoordinateType::newZealandMapGrid:
    {
      strcpy(str, "New Zealand Map Grid (NZMG)");
      break;
    }
  case CoordinateType::obliqueMercator:
    {
      strcpy(str, "Oblique Mercator");
      break;
    }
  case CoordinateType::orthographic:
    {
      strcpy(str, "Orthographic");
      break;
    }
  case CoordinateType::polarStereographicStandardParallel:
    {
      strcpy(str, "Polar Stereographic (Standard Parallel)");
      break;
    }
  case CoordinateType::polarStereographicScaleFactor:
    {
      strcpy(str, "Polar Stereographic (Scale Factor)");
      break;
    }
  case CoordinateType::polyconic:
    {
      strcpy(str, "Polyconic");
      break;
    }
  case CoordinateType::sinusoidal:
    {
      strcpy(str, "Sinusoidal");
      break;
    }
  case CoordinateType::stereographic:
    {
      strcpy(str, "Stereographic");
      break;
    }
  case CoordinateType::transverseMercator:
    {
      strcpy(str, "Transverse Mercator");
      break;
    }
  case CoordinateType::transverseCylindricalEqualArea:
    {
      strcpy(str, "Transverse Cylindrical Equal Area");
      break;
    }
  case CoordinateType::vanDerGrinten:
    {
      strcpy(str, "Van der Grinten");
      break;
    }
  default:
    error_Code = SVC_Invalid_Projection_Value;
    break;
  }

  return error_Code;
}


double Round_Meter(const double Value)
/*
 *  The function Round Meter rounds the specified value, in meters, according to
 *  the current precision level.
 *  Value         : Value to be rounded                                 (input)
 */
{ /* Round_Meter */
  double avalue;
  double divisor = 1.0;
  double fraction;
  double ivalue;
  double result;
  long ival = 0;
  long sign = 1;
  switch (Lat_Long_Prec)
  {
  case Precision::degree:
    {
      divisor = 100000.0;
      break;
    }
  case Precision::tenMinute:
    {
      divisor = 10000.0;
      break;
    }
  case Precision::minute:
    {
      divisor = 1000.0;
      break;
    }
  case Precision::tenSecond:
    {
      divisor = 100.0;
      break;
    }
  case Precision::second:
    {
      divisor = 10.0;
      break;
    }
  case Precision::tenthOfSecond:
    {
      divisor = 1.0;
      break;
    }
  case Precision::hundrethOfSecond:
    {
      divisor = 0.1;
      break;
    }
  case Precision::thousandthOfSecond:
    {
      divisor = 0.01;
      break;
    }
  case Precision::tenThousandthOfSecond:
    {
      divisor = 0.001;
      break;
    }
  }
  if (Value < 0.0)
    sign = -1;
  avalue = fabs (Value / divisor);
  fraction = modf (avalue, &ivalue);
  ival = (long)(ivalue);
  if ((fraction > 0.5) || ((fraction == 0.5) && (ival%2 == 1)))
    ivalue++;
  result = (double)(ivalue * divisor * sign);
  return ( result );
} /* Round_Meter */


SVC_Status Meter_to_String(const double meters, char str[17])
{
  SVC_Status error_Code = SVC_Success;

  double meter_Value = Round_Meter(meters);

  if (Lat_Long_Prec > 4)
    if (sprintf(str, "%1.*lf",(int)(Lat_Long_Prec - 5), meter_Value) <= 0)
      error_Code = SVC_Meter_to_String;
    else
      error_Code = SVC_Success;
  else
    if (sprintf(str, "%1.0lf", meter_Value) <= 0)
      error_Code = SVC_Meter_to_String;
    else
      error_Code = SVC_Success;

  return error_Code;
}


SVC_Status Long_Meter_to_String(const double meters, char str[17])
{
  SVC_Status error_Code = SVC_Success;

  double meter_Value = Round_Meter(meters);

  if (Lat_Long_Prec > 4)
    if (sprintf(str, "%1.*lf",(int)(Lat_Long_Prec - 5), meter_Value) <= 0)
      error_Code = SVC_Meter_to_String;
    else
      error_Code = SVC_Success;
  else
    if (sprintf(str, "%1.0lf", meter_Value) <= 0)
      error_Code = SVC_Meter_to_String;
    else
      error_Code = SVC_Success;

  return error_Code;
}


SVC_Status String_to_Double(const char *str, double *val)
{
  char *placeholder = NULL;
  SVC_Status error_Code = SVC_Success;

  *val = 0;
  if (Valid_Number(str))
  {
    *val = strtod(str,&placeholder);
    return (error_Code);
  }
  else
    return (SVC_Error);
}


long Valid_Number(const char *str)
{
  int i = 0;
  long valid = 1;
  int length;
  int deci = 0;

  if (str)
  {
    length = strlen(str);
    if ((i<length) && ((str[i] == '-') || (str[i] == '+')))
      i ++;
    while (valid && (i < length))
    {
      if (!isdigit(str[i]))
      {
        if ((str[i] == '.') && !deci)
        {
          i++;
          deci = 1;
        }
        else
          valid = 0;
      }
      else
        i++;
    }
  }
  return (valid);
}


SVC_Status String_to_Long(const char *str, long *val)
{
  long return_Parameter = 0;
  char *placeholder = NULL;
  SVC_Status error_Code = SVC_Success;

  if (str && strlen(str))
  {
    return_Parameter = strtol(str, &placeholder, 10);
  }

  if ((return_Parameter == 0) && (placeholder == str))
  {
    error_Code = SVC_String_to_Long;
  }

  *val = return_Parameter;
  return error_Code;
}


long Valid_Coord(char *str, long Type)
{
  long Decimal = false;
  long Signed = false;
  long Separators = 0;
  long Valid = true;
  long Length;
  long Pos = 0;

  if (str != NULL)
  {
    Length = strlen(str);
    if ((Pos<Length) && ((str[Pos] == '-') || (str[Pos] == '+')))
    {
      Signed = true;
      Pos ++;
    }
    while ((Pos < Length) && Valid)
    {
      if (str[Pos] == '.')
      {
        if (Decimal)
          Valid = false;
        else
        {
          Decimal = true;
          Pos++;
        }
      }
      else if (isdigit(str[Pos]))
      {
        Pos++;
      }
      else if ((str[Pos] == ' ') || (str[Pos] == '/') || (str[Pos] == ':'))
      {
        if (Separators >= 3)
          Valid = false;
        else
        {
          Pos++;
          Separators++;
        }
      }
      else if (isalpha(str[Pos]))
      {
        str[Pos] = (char)toupper(str[Pos]);
        if ((((str[Pos] == 'N') || (str[Pos] == 'S')) && (Type == Lat_String))
            || (((str[Pos] == 'W') || (str[Pos] == 'E')) && (Type == Long_String)))
        {
          if (Signed)
            Valid = false;
          Pos++;
          if (Pos != Length)
            Valid = false;
        }
        else
          Valid = false;
      }
      else
        Valid = false;
    }
  }
  return (Valid);
}


SVC_Status String_to_Longitude(const char *str, double *val)
{
  SVC_Status error_Code = SVC_Success;

  double degrees = 0.0;
  double minutes = 0.0;
  double seconds = 0.0;
  long sign = 1;
  char *reference_Pointer = new char[strlen(str) + 1];
  char *parse_String;
  char *next_Str = 0;
  /* Longitudes may have the following format :

     PDDD/MM/SS.FFH
     PDDD/MM.FFFFH
     PDDD.FFFFFFH

     where these are defined as follows
     P = optional plus/minus
     D = degrees (up to three places)
     M = minutes (up to two places)
     S = seconds (up to two places)
     F = floating-point precision (up to 6 places)
     H = optional hemisphere (NSEW)
     / = separator character, one of ':' , '/' , ' '
  */
  if (strlen(str))
  {

    strcpy(reference_Pointer, str);
////////////////    reference_Pointer = strdup_(str);/////////////////////////////////
    parse_String = reference_Pointer;
    if (Valid_Coord(reference_Pointer,Long_String))
    {
      if (parse_String[0] == '-')
      {
        sign = -1;
      }

      next_Str = strtok(parse_String, ":/ ");

      if (next_Str != NULL)
      {
        degrees = atof(next_Str);
      }

      next_Str = strtok(NULL, ":/ ");

      if (next_Str != NULL)
      {
        minutes = atof(next_Str);
      }

      next_Str = strtok(NULL, ":/ ");

      if (next_Str != NULL)
      {
        seconds = atof(next_Str);
      }

      if ((strchr(str, 'N') != NULL) ||
          (strchr(str, 'S') != NULL))
      {
        error_Code = SVC_Inappropriate_Hemisphere;
      }

      if (((next_Str = strchr(const_cast<char*>(str), 'E')) != NULL)
          || ((next_Str = strchr(const_cast<char*>(str), 'e')) != NULL))
      {
        if (sign == -1)
        {
          error_Code = SVC_Sign_and_Hemisphere;
        }

        if (next_Str[1] != '\0')
        {
          error_Code = SVC_Misplaced_Hemisphere;
        }
      }

      if (((next_Str = strchr(const_cast<char*>(str), 'W')) != NULL)
          || ((next_Str = strchr(const_cast<char*>(str), 'w')) != NULL))
      {
        if (sign == -1)
        {
          error_Code = SVC_Sign_and_Hemisphere;
        }

        if (next_Str[1] == '\0')
        {
          sign = -1;
        }
        else
        {
          error_Code = SVC_Misplaced_Hemisphere;
        }
      }

      if (seconds >= 60 || seconds < 0)
      {
        error_Code = SVC_Seconds_out_of_Bounds;
      }

      if (minutes >= 60 || minutes < 0)
      {
        error_Code = SVC_Minutes_out_of_Bounds;
      }

      if ((degrees == -180 || degrees == 360) &&
          ((minutes != 0) || (seconds != 0)))
      {
        error_Code = SVC_Minutes_or_Seconds_Overflow;
      }

      /* Convert DMS to fractional degrees */
      *val = ( fabs(degrees) + (minutes / 60.0) + (seconds / 3600.0) ) * sign;

      /* Convert longitude to be between -180 and 180 */
      if (*val > 180)
        *val -= 360;

      if ((*val > 360) || (*val < -180))
      {
        error_Code = SVC_Degrees_out_of_Bounds;
      }

      if (error_Code != SVC_Success)
      {
        *val = 0;
      }

      delete [] reference_Pointer;
      ///////////////free(reference_Pointer);
    }
    else
      error_Code = SVC_Error;
  }
  return error_Code;
}/* String_to_Longitude */


SVC_Status Longitude_to_String(const double in_longitude, char str[17],
                               bool use_NSEW, bool use_Minutes, bool use_Seconds)
{/* Longitude_to_String */
  double degrees = 0.0;
  double longitude;
  char degrees_As_String[17];
  long starting_Index = 0;
  long ending_Index = 13;
  SVC_Status error_Code = SVC_Success;

  longitude = in_longitude;
  if ((longitude > -0.00000001) && (longitude < 0.00000001))
  {
    longitude = 0.0;
  }
  strcpy(degrees_As_String,"              ");
  switch (Long_Range)
  {
  case _180_180:
    {
      if (longitude > 180)
        degrees = fabs(longitude - 360);
      else
        degrees = fabs(longitude);
      break;
    }
  case _0_360:
    {
      if (longitude < 0)
        degrees = longitude + 360;
      else
        degrees = longitude;
      break;
    }
  }
  Degrees_to_String(degrees, &degrees_As_String[1], use_Minutes, use_Seconds, Long_String);
  ending_Index = strlen(&degrees_As_String[1]) + 1;
  starting_Index = 1;
  switch (Long_Range)
  {
  case _180_180:
    {
      if (use_NSEW)
      {
        if ((longitude > 180) || (longitude < 0))
          degrees_As_String[ending_Index] = 'W';
        else
          degrees_As_String[ending_Index] = 'E';
      }
      else
      {
        if ((longitude > 180) || (longitude < 0))
        {
          degrees_As_String[0] = '-';
          starting_Index = 0;
        }
      }
      break;
    }
  case _0_360:
    {
      if (use_NSEW)
        degrees_As_String[ending_Index] = 'E';
      break;
    }
  }
  degrees_As_String[ending_Index+1] = '\0';
  strcpy(str, &degrees_As_String[starting_Index]);
  return error_Code;
}


void Round_DMS(double *val, long place)
{
  double temp;
  double fraction;
  double int_temp;

  temp = *val * pow(10.0, (double)place);

  fraction = modf(temp, &int_temp);
  if (((temp - int_temp) > 0.5) ||
      (((temp - int_temp) == 0.5) && (fmod(int_temp,2.0) == 1.0)))
    *val = (int_temp + 1.0) / pow(10.0, (double)place);
  else
    *val = int_temp / pow(10.0, (double)place);
}


void Degrees_to_String(double degrees, char *str, bool use_Minutes, bool use_Seconds, long Type)
{
  double minutes = 0.0;
  double seconds = 0.0;
  long integer_Degrees = 0;
  long integer_Minutes = 0;
  long integer_Seconds = 0;
  int j;

  if ((!use_Minutes) || (Lat_Long_Prec == 0))
  { /* Decimal Degrees */
    Round_DMS(&degrees, Lat_Long_Prec);
    if(leading_zeros)
    {
      if(Type == Lat_String)
        if(fabs(degrees) < 10)
          sprintf(str,"0%1.*lf",(int)Lat_Long_Prec,degrees);
        else
          sprintf(str,"%1.*lf",(int)Lat_Long_Prec,degrees);
      else
        if(fabs(degrees) < 10)
          sprintf(str,"00%1.*lf",(int)Lat_Long_Prec,degrees);
        else if(fabs(degrees) < 100)
          sprintf(str,"0%1.*lf",(int)Lat_Long_Prec,degrees);
        else
          sprintf(str,"%1.*lf",(int)Lat_Long_Prec,degrees);
    }
    else
      sprintf(str,"%1.*lf",(int)Lat_Long_Prec,degrees);
  }
  else if ((use_Minutes && !use_Seconds) || (Lat_Long_Prec <= 2))
  { /* Degrees & Minutes */
    integer_Degrees = (long)degrees;
    minutes = (degrees - integer_Degrees) * 60.0;
    Round_DMS(&minutes, Lat_Long_Prec - 2);
    integer_Minutes = (long)minutes;
    if (integer_Minutes >= 60)
    {
      integer_Minutes -= 60;
      integer_Degrees += 1;
    }
    if (Lat_Long_Prec <= 2)
    {
      if(leading_zeros)
      {
        if(Type == Lat_String)
          if(fabs(degrees) < 10)
            j = sprintf(str,"0%ld%c",integer_Degrees,Lat_Long_Sep);
          else
            j = sprintf(str,"%ld%c",integer_Degrees,Lat_Long_Sep);
        else
          if(fabs(degrees) < 10)
            j = sprintf(str,"00%ld%c",integer_Degrees,Lat_Long_Sep);
          else if(fabs(degrees) < 100)
            j = sprintf(str,"0%ld%c",integer_Degrees,Lat_Long_Sep);
          else
            j = sprintf(str,"%ld%c",integer_Degrees,Lat_Long_Sep);

        if(integer_Minutes < 10)
          j += sprintf(str + j,"0%ld",integer_Minutes);
        else
          j += sprintf(str + j,"%ld",integer_Minutes);
      }
      else
        sprintf(str,"%ld%c%ld",integer_Degrees,Lat_Long_Sep,integer_Minutes);
    }
    else
    {
      if (minutes >= 60)
        minutes -= 60;
      if(leading_zeros)
      {
        if(Type == Lat_String)
          if(fabs(degrees) < 10)
            j = sprintf(str,"0%ld%c",integer_Degrees,Lat_Long_Sep);
          else
            j = sprintf(str,"%ld%c",integer_Degrees,Lat_Long_Sep);
        else
          if(fabs(degrees) < 10)
            j = sprintf(str,"00%ld%c",integer_Degrees,Lat_Long_Sep);
          else if(fabs(degrees) < 100)
            j = sprintf(str,"0%ld%c",integer_Degrees,Lat_Long_Sep);
          else
            j = sprintf(str,"%ld%c",integer_Degrees,Lat_Long_Sep);

        if(integer_Minutes < 10)
          j += sprintf(str + j,"0%1.*lf",(int)Lat_Long_Prec-2,minutes);
        else
          j += sprintf(str + j,"%1.*lf",(int)Lat_Long_Prec-2,minutes);
      }
      else
        sprintf(str,"%ld%c%1.*lf",integer_Degrees,Lat_Long_Sep,(int)Lat_Long_Prec-2,minutes);
    }
  }
  else
  { /* Degrees, Minutes, & Seconds */
    integer_Degrees = (long)degrees;
    minutes = (degrees - integer_Degrees) * 60.0;
    integer_Minutes = (long)minutes;
    seconds = (minutes - integer_Minutes) * 60.0;
    Round_DMS(&seconds, Lat_Long_Prec - 4);
    integer_Seconds = (long)seconds;
    if (integer_Seconds >= 60)
    {
      integer_Seconds -= 60;
      integer_Minutes += 1;
      if (integer_Minutes >= 60)
      {
        integer_Degrees += 1;
        integer_Minutes -= 60;
      }
    }

    if (Lat_Long_Prec <= 4)
    {
      if(leading_zeros)
      {
        if(Type == Lat_String)
          if(fabs(degrees) < 10)
            j = sprintf(str,"0%ld%c",integer_Degrees,Lat_Long_Sep);
          else
            j = sprintf(str,"%ld%c",integer_Degrees,Lat_Long_Sep);
        else
          if(fabs(degrees) < 10)
            j = sprintf(str,"00%ld%c",integer_Degrees,Lat_Long_Sep);
          else if(fabs(degrees) < 100)
            j = sprintf(str,"0%ld%c",integer_Degrees,Lat_Long_Sep);
          else
            j = sprintf(str,"%ld%c",integer_Degrees,Lat_Long_Sep);

        if(integer_Minutes < 10)
          j += sprintf(str + j,"0%ld%c",integer_Minutes,Lat_Long_Sep);
        else
          j += sprintf(str + j,"%ld%c",integer_Minutes,Lat_Long_Sep);
        if(integer_Seconds < 10)
          j += sprintf(str + j,"0%ld",integer_Seconds);
        else
          j += sprintf(str + j,"%ld",integer_Seconds);
      }
      else
        sprintf(str,"%ld%c%ld%c%ld",integer_Degrees,Lat_Long_Sep,integer_Minutes,Lat_Long_Sep,integer_Seconds);
    }
    else
    {
      if (seconds >= 60)
      {
        seconds -= 60;
      }
      if(leading_zeros)
      {
        if(Type == Lat_String)
          if(fabs(degrees) < 10)
            j = sprintf(str,"0%ld%c",integer_Degrees,Lat_Long_Sep);
          else
            j = sprintf(str,"%ld%c",integer_Degrees,Lat_Long_Sep);
        else
          if(fabs(degrees) < 10)
            j = sprintf(str,"00%ld%c",integer_Degrees,Lat_Long_Sep);
          else if(fabs(degrees) < 100)
            j = sprintf(str,"0%ld%c",integer_Degrees,Lat_Long_Sep);
          else
            j = sprintf(str,"%ld%c",integer_Degrees,Lat_Long_Sep);

        if(integer_Minutes < 10)
          j += sprintf(str + j,"0%ld%c",integer_Minutes,Lat_Long_Sep);
        else
          j += sprintf(str + j,"%ld%c",integer_Minutes,Lat_Long_Sep);
        if(integer_Seconds < 10)
          j += sprintf(str + j,"0%1.*lf",(int)Lat_Long_Prec-4,seconds);
        else
          j += sprintf(str + j,"%1.*lf",(int)Lat_Long_Prec-4,seconds);
      }
      else
        sprintf(str,"%ld%c%ld%c%1.*lf",integer_Degrees,Lat_Long_Sep,integer_Minutes,Lat_Long_Sep,(int)Lat_Long_Prec-4,seconds);
    }
  }
}


SVC_Status String_to_Latitude(const char *str, double *val)
{
  SVC_Status error_Code = SVC_Success;

  double degrees = 0.0;
  double minutes = 0.0;
  double seconds = 0.0;
  long sign = 1;
  char *reference_Pointer = new char[strlen(str) + 1];
  char *parse_String;
  char *next_Str = 0;
  /* Longitudes may have the following format :

     PDD/MM/SS.FFH
     PDD/MM.FFFFH
     PDD.FFFFFFH

     where these are defined as follows
     P = optional plus/minus
     D = degrees (up to two places)
     M = minutes (up to two places)
     S = seconds (up to two places)
     F = floating-point precision (up to 6 places)
     H = optional hemisphere (NSEW)
     / = separator character, one of / : sp
  */

  if (strlen(str))
  {
    strcpy(reference_Pointer, str);
///////////////////    reference_Pointer = strdup_(str);///////////////////////////////
    parse_String = reference_Pointer;
    if (Valid_Coord(reference_Pointer,Lat_String))
    {
      if (parse_String[0] == '-')
      {
        sign = -1;
      }

      next_Str = strtok(parse_String, ":/ ");

      if (next_Str != NULL)
      {
        degrees = atof(next_Str);
      }

      next_Str = strtok(NULL, ":/ ");

      if (next_Str != NULL)
      {
        minutes = atof(next_Str);
      }

      next_Str = strtok(NULL, ":/ ");

      if (next_Str != NULL)
      {
        seconds = atof(next_Str);
      }

      if ((strchr(str, 'W') != NULL) ||
          (strchr(str, 'E') != NULL))
      {
        error_Code = SVC_Inappropriate_Hemisphere;
      }

      if (((next_Str = strchr(const_cast<char*>(str), 'N')) != NULL)
          || ((next_Str = strchr(const_cast<char*>(str), 'n')) != NULL))
      {
        if (sign == -1)
        {
          error_Code = SVC_Sign_and_Hemisphere;
        }

        if (next_Str[1] != '\0')
        {
          error_Code = SVC_Misplaced_Hemisphere;
        }
      }

      if (((next_Str = strchr(const_cast<char*>(str), 'S')) != NULL)
          || ((next_Str = strchr(const_cast<char*>(str), 's')) != NULL))
      {
        if (sign == -1)
        {
          error_Code = SVC_Sign_and_Hemisphere;
        }

        if (next_Str[1] == '\0')
        {
          sign = -1;
        }
        else
        {
          error_Code = SVC_Misplaced_Hemisphere;
        }
      }

      if (seconds >= 60 || seconds < 0)
      {
        error_Code = SVC_Seconds_out_of_Bounds;
      }

      if (minutes >= 60 || minutes < 0)
      {
        error_Code = SVC_Minutes_out_of_Bounds;
      }

      if (degrees < -90 || degrees > 90)
      {
        error_Code = SVC_Degrees_out_of_Bounds;
      }

      if ((degrees == -90 || degrees == 90) &&
          ((minutes != 0) || (seconds != 0)))
      {
        error_Code = SVC_Minutes_or_Seconds_Overflow;
      }

      /* Convert DMS to fractional degrees */
      *val = (double)( fabs(degrees) + (minutes / 60) + (seconds / 3600) ) * sign;

      if (error_Code != SVC_Success)
      {
        *val = 0;
      }

      delete [] reference_Pointer;
      /////////////////////////free(reference_Pointer);
    }
    else
      error_Code = SVC_Error;
  }
  return error_Code;
}


SVC_Status Latitude_to_String(const double in_latitude, char str[17],
                              bool use_NSEW, bool use_Minutes, bool use_Seconds)
{
  double degrees = fabs(in_latitude);
  double latitude;
  char degrees_As_String[17];
  long starting_Index = 1;
  SVC_Status error_Code = SVC_Success;
  long ending_Index = 12;

  latitude = in_latitude;
  if ((latitude > -0.00000001) && (latitude < 0.00000001))
  {
    latitude = 0.0;
  }
  strcpy(degrees_As_String,"             ");
  Degrees_to_String(degrees, &degrees_As_String[1], use_Minutes, use_Seconds, Lat_String);
  ending_Index = strlen(&degrees_As_String[1]) + 1;

  if (use_NSEW)
  {
    if (latitude < 0)
    {
      degrees_As_String[ending_Index] = 'S';
    }
    else
    {
      degrees_As_String[ending_Index] = 'N';
    }
  }
  else
  {
    if (latitude < 0)
    {
      degrees_As_String[0] = '-';
      starting_Index = 0;
    }
  }
  degrees_As_String[ending_Index+1] = '\0';
  strcpy(str, &degrees_As_String[starting_Index]);
  return error_Code;
}


SVC_Status Zone_to_String(const long zone, char str[3])
{
  SVC_Status error_Code = SVC_Success;

  if (sprintf(str, "%2.0d", zone) <= 0)
  {
    error_Code = SVC_Zone_to_String;
  }

  return error_Code;
}


SVC_Status Ellipsoidal_Height_to_String(const double ellipsoidal_Height, char str[17])
{
  return Long_Meter_to_String(ellipsoidal_Height, str);
}


SVC_Status Scale_Factor_to_String(const double scale_Factor, char str[8])
{
  SVC_Status error_Code = SVC_Success;

  if (sprintf(str, "%.5f", scale_Factor) <= 0)
  {
    error_Code = SVC_Scale_Factor_to_String;
  }

  return error_Code;
}


void Error_Prefix
( const SourceOrTarget::Enum Direction,
 const CoordinateType::Enum System,
  char  *Separator,
  char  *String)
{
  char   *in_out;
  if (Direction == SourceOrTarget::source)
    in_out = "Input";
  else
    in_out = "Output";

  switch (System)
  {
  case CoordinateType::geocentric:
    {
      sprintf(String,"%s%s%s%s",in_out," Geocentric Coordinates:",Separator,Separator);
      break;
    }
  case CoordinateType::geodetic:
    {
      sprintf(String,"%s%s%s%s",in_out," Geodetic Coordinates:",Separator,Separator);
      break;
    }
  case CoordinateType::georef:
    {
      sprintf(String,"%s%s%s%s",in_out," GEOREF Coordinates:",Separator,Separator);
      break;
    }
  case CoordinateType::albersEqualAreaConic:
    {
      sprintf(String,"%s%s%s%s",in_out," Albers Equal Area Conic Projection:",Separator,Separator);
      break;
    }
  case CoordinateType::azimuthalEquidistant:
    {
      sprintf(String,"%s%s%s%s",in_out," Azimuthal Equidistant Projection:",Separator,Separator);
      break;
    }
  case CoordinateType::britishNationalGrid:
    {  
      sprintf(String,"%s%s%s%s",in_out," British National Grid Coordinates:",Separator,Separator);
      break;
    }
  case CoordinateType::bonne:
    {
      sprintf(String,"%s%s%s%s",in_out," Bonne Projection:",Separator,Separator);
      break;
    }
  case CoordinateType::cassini:
    {
      sprintf(String,"%s%s%s%s",in_out," Cassini Projection:",Separator,Separator);
      break;
    }
  case CoordinateType::cylindricalEqualArea:
    {
      sprintf(String,"%s%s%s%s",in_out," Cylindrical Equal Area Projection:",Separator,Separator);
      break;
    }
  case CoordinateType::eckert4:
    {
      sprintf(String,"%s%s%s%s",in_out," Eckert IV Projection:",Separator,Separator);
      break;
    }
  case CoordinateType::eckert6:
    {
      sprintf(String,"%s%s%s%s",in_out," Eckert VI Projection:",Separator,Separator);
      break;
    }
  case CoordinateType::equidistantCylindrical:
    {
      sprintf(String,"%s%s%s%s",in_out," Equidistant Cylindrical Projection:",Separator,Separator);
      break;
    }
  case CoordinateType::globalAreaReferenceSystem:
    {
      sprintf(String,"%s%s%s%s",in_out," GARS Coordinates:",Separator,Separator);
      break;
    }
  case CoordinateType::gnomonic:
    {
      sprintf(String,"%s%s%s%s",in_out," Gnomonic Projection:",Separator,Separator);
      break;
    }
  case CoordinateType::lambertConformalConic1Parallel:
    {
      sprintf(String,"%s%s%s%s",in_out," Lambert Conformal Conic (1 Standard Parallel) Projection:",Separator,Separator);
      break;
    }
  case CoordinateType::lambertConformalConic2Parallels:
    {
      sprintf(String,"%s%s%s%s",in_out," Lambert Conformal Conic (2 Standard Parallel) Projection:",Separator,Separator);
      break;
    }
  case CoordinateType::localCartesian:
    {
      sprintf(String,"%s%s%s%s",in_out," Local Cartesian Coordinates:",Separator,Separator);
      break;
    }
  case CoordinateType::mercatorStandardParallel:
    {
      sprintf(String,"%s%s%s%s",in_out," Mercator (Standard Parallel) Projection:",Separator,Separator);
      break;
    }
  case CoordinateType::mercatorScaleFactor:
    {
      sprintf(String,"%s%s%s%s",in_out," Mercator (Scale Factor) Projection:",Separator,Separator);
      break;
    }
  case CoordinateType::militaryGridReferenceSystem:
    {
      sprintf(String,"%s%s%s%s",in_out," MGRS Coordinates:",Separator,Separator);
      break;
    }
  case CoordinateType::millerCylindrical:
    {
      sprintf(String,"%s%s%s%s",in_out," Miller Cylindrical Projection:",Separator,Separator);
      break;
    }
  case CoordinateType::mollweide:
    {
      sprintf(String,"%s%s%s%s",in_out," Mollweide Projection:",Separator,Separator);
      break;
    }
  case CoordinateType::neys:
    {
      sprintf(String,"%s%s%s%s",in_out," Ney's Projection:",Separator,Separator);
      break;
    }
  case CoordinateType::newZealandMapGrid:
    {
      sprintf(String,"%s%s%s%s",in_out," New Zealand Map Grid Projection:",Separator,Separator);
      break;
    }
  case CoordinateType::obliqueMercator:
    {
      sprintf(String,"%s%s%s%s",in_out," Oblique Mercator Projection:",Separator,Separator);
      break;
    }
  case CoordinateType::orthographic:
    {
      sprintf(String,"%s%s%s%s",in_out," Orthographic Projection:",Separator,Separator);
      break;
    }
  case CoordinateType::polarStereographicStandardParallel:
    {
      sprintf(String,"%s%s%s%s",in_out," Polar Stereographic (Standard Parallel) Projection:",Separator,Separator);
      break;
    }
  case CoordinateType::polarStereographicScaleFactor:
    {
      sprintf(String,"%s%s%s%s",in_out," Polar Stereographic (Scale Factor) Projection:",Separator,Separator);
      break;
    }
  case CoordinateType::polyconic:
    {
      sprintf(String,"%s%s%s%s",in_out," Polyconic Projection:",Separator,Separator);
      break;
    }
  case CoordinateType::sinusoidal:
    {
      sprintf(String,"%s%s%s%s",in_out," Sinusoidal Projection:",Separator,Separator);
      break;
    }
  case CoordinateType::stereographic:
    {
      sprintf(String,"%s%s%s%s",in_out," Stereographic Projection:",Separator,Separator);
      break;
    }
  case CoordinateType::transverseCylindricalEqualArea:
    {
      sprintf(String,"%s%s%s%s",in_out," Transverse Cylindrical Equal Area Projection:",Separator,Separator);
      break;
    }
  case CoordinateType::transverseMercator:
    {
      sprintf(String,"%s%s%s%s",in_out," Transverse Mercator Projection:",Separator,Separator);
      break;
    }
  case CoordinateType::universalPolarStereographic:
    {
      sprintf(String,"%s%s%s%s",in_out," UPS Coordinates:",Separator,Separator);
      break;
    }
  case CoordinateType::usNationalGrid:
    {
      sprintf(String,"%s%s%s%s",in_out," USNG Coordinates:",Separator,Separator);
      break;
    }
  case CoordinateType::universalTransverseMercator:
    {
      sprintf(String,"%s%s%s%s",in_out," UTM Coordinates:",Separator,Separator);
      break;
    }
  case CoordinateType::vanDerGrinten:
    {
      sprintf(String,"%s%s%s%s",in_out," Van der Grinten Projection:",Separator,Separator);
      break;
    }
  } /* switch */
}

void Error_Append (char *New_String,
                   char *Separator,
                   char *String)
{
  strcat(String,New_String);
  strcat(String,Separator);
}

// CLASSIFICATION: UNCLASSIFIED
