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

﻿#pragma once

#ifndef _SYMUMASTER_SYMUCORECONSTANTS_
#define _SYMUMASTER_SYMUCORECONSTANTS_

#include "SymuCoreExports.h"

namespace SymuCore {

    enum NodeType
    {
        NT_Undefined = 0,
        NT_InterLayerOrigin,
        NT_InterLayerDestination,
        NT_RoadExtremity,
        NT_RoadJunction,
        NT_Area,
        NT_SubArea,
        NT_PublicTransportStation,
        NT_TaxiStation,
        NT_Reservoir,
        NT_NetworksInterface,
    };

    enum PatternType     //define each possible pattern mode
    {
        PT_Undefined = 0,
        PT_Walk,
        PT_Cycling,
        PT_Driving,
        PT_PublicTransport,
        PT_Taxi
    };

    enum ServiceType     //define each possible Layers
    {
        ST_Undefined = 0,
        ST_All,
        ST_Driving,
        ST_PublicTransport,
        ST_Taxi
    };

    enum CostFunction     //define each possible cost function here
    {
        CF_Undefined = 0,
        CF_TravelTime,
        CF_Marginals
    };

    enum ShortestPathHeuristic // Heuristic for the shortest path calculations
    {
        SPH_NONE = 0,
        SPH_EUCLIDIAN = 1
    };

    class SYMUCORE_DLL_DEF SymuCoreConstants {

    };
}

#endif // _SYMUMASTER_SYMUCORECONSTANTS_
