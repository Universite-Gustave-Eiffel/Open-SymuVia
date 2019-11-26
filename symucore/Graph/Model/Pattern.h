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

#ifndef SYMUCORE_PATTERN_H
#define SYMUCORE_PATTERN_H

#include "SymuCoreExports.h"

#include "Utils/SymuCoreConstants.h"
#include "ListTimeFrame.h"

#include <vector>
#include <map>

#pragma warning(push)
#pragma warning(disable : 4251)

namespace SymuCore {

class Link;
class VehicleType;
class Cost;
class SubPopulation;

class SYMUCORE_DLL_DEF Pattern {

public:

    Pattern();
    Pattern(Link* pLink, PatternType ePatternType);
    virtual ~Pattern();

    //getters
    Link *getLink() const;
    PatternType getPatternType() const;
    double getFuncClass() const;

    void setFuncClass(double dbFuncClass);

    virtual void prepareTimeFrames(double startPeriodTime, double endPeriodTime, double travelTimesUpdatePeriod, const std::vector<SubPopulation*>& listSubPopulations, const std::map<SymuCore::SubPopulation*, SymuCore::CostFunction> & mapCostFunctions, int nbSimulationInstances, int iInstance) = 0;
    virtual Cost* getPatternCost(int iSimulationInstance, double t, SubPopulation* pSubPopulation) = 0;

    virtual std::string toString() const = 0;
    int getUniqueID() const;

    virtual void fillMeasuredCostsForTravelTimesUpdatePeriod(int iTravelTimesUpdatePeriodIndex, const std::vector<SubPopulation*>& listSubPopulations, const std::map<SymuCore::SubPopulation*, SymuCore::CostFunction> & mapCostFunctions) = 0;
    
    // usefull for costs that can't be computed right away at the end of each travel time update period (ie : marginals in spatial mode)
    virtual void postProcessCosts(const std::vector<SubPopulation*>& listSubPopulations, const std::map<SymuCore::SubPopulation*, SymuCore::CostFunction> & mapCostFunctions) {}

    virtual void fillFromSecondaryInstance(Pattern * pFrom, int iInstance) = 0;

protected:

    static unsigned int     s_uniqueIDCounter;
    unsigned int            m_uniqueID; // unique identifier of the pattern

    Link*                   m_pLink; //pointer to the Link where the pattern apply
    PatternType             m_ePatternType; //Service mode chosen

    double                  m_dbFuncClass; // Functional class of the pattern (used for custom A* algorithm)
};
}

#pragma warning(pop)

#endif // SYMUCORE_PATTERN_H
