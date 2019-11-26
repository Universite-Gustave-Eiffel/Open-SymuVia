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

#ifndef SYMUCORE_WALKINGPATTERN_H
#define SYMUCORE_WALKINGPATTERN_H

#include "SymuCoreExports.h"

#include "Graph/Model/Pattern.h"
#include "Graph/Model/Cost.h"

#include <map>

#pragma warning(push)
#pragma warning(disable : 4251)

namespace SymuCore {

class SYMUCORE_DLL_DEF WalkingPattern : public Pattern {

public:

    WalkingPattern();
    WalkingPattern(Link* pLink, bool bIsInitialPattern, bool bUseRealLength, double dbLength);
    virtual ~WalkingPattern();

    virtual void prepareTimeFrames(double startPeriodTime, double endPeriodTime, double travelTimesUpdatePeriod, const std::vector<SubPopulation*>& listSubPopulations, const std::map<SymuCore::SubPopulation*, SymuCore::CostFunction> & mapCostFunctions, int nbSimulationInstances, int iInstance);
    virtual Cost* getPatternCost(int iSimulationInstance, double t, SubPopulation* pSubPopulation);

    virtual std::string toString() const;

    virtual void fillMeasuredCostsForTravelTimesUpdatePeriod(int iTravelTimesUpdatePeriodIndex, const std::vector<SubPopulation*>& listSubPopulations, const std::map<SymuCore::SubPopulation*, SymuCore::CostFunction> & mapCostFunctions);

    std::map<SubPopulation*, Cost> & GetCostBySubPopulation();
    double getWalkLength() const;

    virtual void fillFromSecondaryInstance(Pattern * pFrom, int iInstance);

protected:

    bool m_bIsInitialPattern; // don't use the same walk speed and radius for initial and intermediate walk patterns
    bool m_bUseRealLength; // for zones, we don't use a real length value but the initial walk radius instead
    double m_dbLength;

    std::map<SubPopulation*, Cost> m_CostBySubPopulation;

};
}

#pragma warning(pop)

#endif // SYMUCORE_WALKINGPATTERN_H
