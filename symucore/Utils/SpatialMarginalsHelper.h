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

#ifndef SYMUCORE_SPATIAL_MARGINALS_HELPER_H
#define SYMUCORE_SPATIAL_MARGINALS_HELPER_H

#include "SymuCoreExports.h"

#include "Utils/SymuCoreConstants.h"

#include <Graph/Model/ListTimeFrame.h>

#include <map>

#pragma warning(push)
#pragma warning(disable : 4251)

namespace SymuCore {

    class MacroType;
    class SubPopulation;
    class Cost;

    class SYMUCORE_DLL_DEF SpatialMarginalsHelper
    {
    public:
        void SetNbVehicles(size_t iPeriodIndex, double dbStartTime, double dbEndTime, SymuCore::MacroType * pMacroType, double dbNbVehicles, double dbNbVehiclesForAllMacroTypes,
            double dbTravelTimeForAllMacroTypes);

        void ComputeMarginal(const std::map<MacroType*, std::vector<std::pair<SymuCore::CostFunction, SubPopulation*> > > & listMacroType,
            const std::map<MacroType*, bool> & forbiddenMacroTypes,
            const SymuCore::ListTimeFrame<std::map<SymuCore::SubPopulation*, SymuCore::Cost> > & temporalCosts,
            double dbMaxMarginalsValue, double dbPenalisationRatio);

    private:
        // Pour le stockage des nombres de véhicules en mode marginals spatialisé pour calcul a posteriori
        SymuCore::ListTimeFrame<std::map<SymuCore::MacroType*, double>  > m_mapTemporalNbVehicles;
        // Stockage du temps de parcours pour tous les macro-types
        SymuCore::ListTimeFrame<double>                                   m_mapTravelTimesForAllMacroTypes;
    };

}

#pragma warning(pop)

#endif // SYMUCORE_SPATIAL_MARGINALS_HELPER_H


