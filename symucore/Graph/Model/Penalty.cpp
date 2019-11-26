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

#include "Penalty.h"

#include <boost/make_shared.hpp>

using namespace SymuCore;

Penalty::Penalty()
{
    m_pNode = NULL;
}

Penalty::Penalty(Node* pParentNode, const PatternsSwitch & patternsSwitch) : AbstractPenalty(pParentNode, patternsSwitch)
{

}

Penalty::~Penalty()
{

}

std::vector<ListTimeFrame<std::map<SubPopulation*, Cost> > > & Penalty::getTemporalCosts()
{
    return m_TemporalCosts;
}

const std::vector<ListTimeFrame<std::map<SubPopulation*, Cost> > > & Penalty::getTemporalCosts() const
{
    return m_TemporalCosts;
}

void Penalty::prepareTimeFrames(double startPeriodTime, double endPeriodTime, double travelTimesUpdatePeriod, const std::vector<SubPopulation*>& listSubPopulations, const std::map<SymuCore::SubPopulation*, SymuCore::CostFunction> & mapCostFunctions, int nbSimulationInstances, int iInstance)
{
    m_TemporalCosts.clear();
    m_TemporalCosts.resize(iInstance==0?nbSimulationInstances:1);
    
    int nbTravelTimesUpdatePeriodsByPeriod = (int)ceil((endPeriodTime - startPeriodTime) / travelTimesUpdatePeriod);
    double currentTime = startPeriodTime;
    for (int iTravelTimesUpdatePeriod = 0; iTravelTimesUpdatePeriod < nbTravelTimesUpdatePeriodsByPeriod; iTravelTimesUpdatePeriod++)
    {
        double startTravelTimesUpdatePeriodTime = currentTime;
        double endTravelTimesUpdatePeriodTime = std::min<double>(startTravelTimesUpdatePeriodTime + travelTimesUpdatePeriod, endPeriodTime);

        ListTimeFrame<std::map<SubPopulation*, Cost>  > & mapTemporalCosts = m_TemporalCosts[0];
        mapTemporalCosts.addTimeFrame(startTravelTimesUpdatePeriodTime, endTravelTimesUpdatePeriodTime, boost::make_shared<std::map<SubPopulation*, Cost>>());
        fillMeasuredCostsForTravelTimesUpdatePeriod(iTravelTimesUpdatePeriod, listSubPopulations, mapCostFunctions);

        currentTime += travelTimesUpdatePeriod;
    }
}

Cost * Penalty::getPenaltyCost(int iSimulationInstance, double t, SubPopulation *pSubPopulation)
{
    // remark : we suppose here that the map of costs is always filled for all subpopulations
    return &m_TemporalCosts[iSimulationInstance].getData(t)->at(pSubPopulation);
}

void Penalty::fillFromSecondaryInstance(AbstractPenalty * pFrom, int iInstance)
{
    Penalty * pOriginalPenalty = (Penalty*)pFrom;
    m_TemporalCosts[iInstance] = std::move(pOriginalPenalty->m_TemporalCosts.front());
}
