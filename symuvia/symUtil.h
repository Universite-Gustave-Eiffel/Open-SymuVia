/* ----------------------------------------------------------------------
* OPEN-SYMUVIA (http://open-symuvia.ifsttar.fr). This file is part of OPEN-SYMUVIA.
*
* OPEN-SYMUVIA is an open-source traffic simulator.
*
* Copyright (C) - IFSTTAR, ENTPE - Ludovic Leclercq, Cécile Bécarie
*
* Open-SymuVia is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation; either version 3 of the License, or
* (at your option) any later version.
* 
* OPEN-SYMUVIA is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lessed General Public License for more details.
* 
* You should have received a copy of the GNU Lesser General Public License
* along with this program; if not, write to the Free Software Foundation,
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA or 
* see <http://ww.gnu.org/licenses/>
*
* For more information, please consult: <http://open-symuvia.ifsttar.fr> or 
* send an email to cecile.becarie@ifsttar.fr
*
* ----------------------------------------------------------------------*/

#pragma once
#ifndef symubruitH
#define symubruitH

#include <float.h>
#include <math.h>

// Position indÃ©finie (le vÃ©hicule vient d'Ãªtre crÃ©Ã© et n'est pas sur le rÃ©seau)
#define UNDEF_POSITION -DBL_MIN
// Instant indÃ©fini
#define UNDEF_INSTANT -DBL_MIN
// Stock indÃ©fini
#define UNDEF_STOCK -DBL_MIN

#define ZERO_DOUBLE         0.000001    // zero de type double
#define NONVAL_DOUBLE       9999        // UtilisÃ© pour ne pas traiter la valeur

namespace boost {
    namespace serialization {
        class access;
    }
}

//***************************************************************************//
//                          structure Point                                  //
//***************************************************************************//
struct Point
{
    Point(){dbX = 0.0;dbY = 0.0;dbZ = 0.0;}
    Point(double x, double y, double z){dbX = x;dbY = y;dbZ = z;}
    double DistanceTo(const Point& ptOtherPoint){return sqrt((dbX - ptOtherPoint.dbX)*(dbX - ptOtherPoint.dbX) + (dbY - ptOtherPoint.dbY)*(dbY - ptOtherPoint.dbY) + (dbZ - ptOtherPoint.dbZ)*(dbZ - ptOtherPoint.dbZ));}

    double      dbX;
    double      dbY;
    double      dbZ;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SÃ©rialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version);
};

#endif
