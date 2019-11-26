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

#include "DrivingPattern.h"
#include "Demand/MacroType.h"

#include <boost/make_shared.hpp>

using namespace SymuCore;

DrivingPattern::DrivingPattern() : Pattern()
{
    m_ePatternType = PT_Driving;
}

DrivingPattern::DrivingPattern(Link* pLink)
: Pattern(pLink, PT_Driving)
{
}

DrivingPattern::~DrivingPattern()
{
}

const std::vector<ListTimeFrame<std::map<SubPopulation*, Cost> > > & DrivingPattern::getTemporalCosts() const
{
    return m_mapTemporalCosts;
}

std::vector<ListTimeFrame<std::map<SubPopulation*, Cost> > > & DrivingPattern::getTemporalCosts()
{
    return m_mapTemporalCosts;
}

void DrivingPattern::prepareTimeFrames(double startPeriodTime, double endPeriodTime, double travelTimesUpdatePeriod, const std::vector<SubPopulation*>& listSubPopulations, const std::map<SymuCore::SubPopulation*, SymuCore::CostFunction> & mapCostFunctions, int nbSimulationInstances, int iInstance)
{
    m_mapTemporalCosts.clear();
    m_mapTemporalCosts.resize(iInstance == 0 ? nbSimulationInstances : 1);

    int nbTravelTimesUpdatePeriodsByPeriod = (int)ceil((endPeriodTime - startPeriodTime) / travelTimesUpdatePeriod);
    double currentTime = startPeriodTime;
    for (int iTravelTimesUpdatePeriod = 0; iTravelTimesUpdatePeriod < nbTravelTimesUpdatePeriodsByPeriod; iTravelTimesUpdatePeriod++)
    {
        double startTravelTimesUpdatePeriodTime = currentTime;
        double endTravelTimesUpdatePeriodTime = std::min<double>(startTravelTimesUpdatePeriodTime + travelTimesUpdatePeriod, endPeriodTime);

        ListTimeFrame<std::map<SubPopulation*, Cost>  > & mapTemporalCosts = m_mapTemporalCosts[0];
        mapTemporalCosts.addTimeFrame(startTravelTimesUpdatePeriodTime, endTravelTimesUpdatePeriodTime, boost::make_shared<std::map<SubPopulation*, Cost>>());
        fillMeasuredCostsForTravelTimesUpdatePeriod(iTravelTimesUpdatePeriod, listSubPopulations, mapCostFunctions);

        currentTime += travelTimesUpdatePeriod;
    }
}

Cost * DrivingPattern::getPatternCost(int iSimulationInstance, double t, SubPopulation *pSubPopulation)
{
    // remark : we suppose here that the map fo costs is always filled even if the macrotype can't use the pattern
    return &m_mapTemporalCosts[iSimulationInstance].getData(t)->at(pSubPopulation);
}

void DrivingPattern::fillFromSecondaryInstance(Pattern * pFrom, int iInstance)
{
    DrivingPattern * pOriginalPattern = (DrivingPattern*)pFrom;
    m_mapTemporalCosts[iInstance] = std::move(pOriginalPattern->m_mapTemporalCosts.front());
}


