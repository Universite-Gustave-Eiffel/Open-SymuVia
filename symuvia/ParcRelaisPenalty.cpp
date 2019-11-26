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

ï»¿#include "stdafx.h"
#include "ParcRelaisPenalty.h"

#include "Parking.h"
#include "reseau.h"

#include <Graph/Model/Pattern.h>
#include <Demand/SubPopulation.h>
#include <Demand/Population.h>

using namespace SymuCore;

ParcRelaisPenalty::ParcRelaisPenalty(SymuCore::Node* pParentNode, const SymuCore::PatternsSwitch & patternsSwitch, Parking * pParking)
: SymuCore::DrivingPenalty(pParentNode, patternsSwitch)
{
    m_pParking = pParking;
}

ParcRelaisPenalty::~ParcRelaisPenalty()
{

}

void ParcRelaisPenalty::fillMeasuredCostsForTravelTimesUpdatePeriod(int iTravelTimesUpdatePeriodIndex, const std::vector<SubPopulation*>& listSubPopulations, const std::map<SymuCore::SubPopulation*, SymuCore::CostFunction> & mapCostFunctions)
{
    const TimeFrame<std::map<SubPopulation *, Cost> > & costVariation = getTemporalCosts().front().getTimeFrame((size_t)iTravelTimesUpdatePeriodIndex);

    // on regroupe les sous-population par macro-type :
    for (size_t iSubPop = 0; iSubPop < listSubPopulations.size(); iSubPop++)
    {
        SubPopulation * pSubPop = listSubPopulations[iSubPop];

        bool bCanEnter = false;
        MacroType * pMacroType = pSubPop->GetPopulation()->GetMacroType();
        if (pMacroType && m_pParking->IsAllowedVehiculeType(m_pParking->GetNetwork()->GetVehiculeTypeFromMacro(pMacroType)))
        {
            bCanEnter = true;

            // dans le cas de l'entrÃ©e dans le parking, on doit vÃ©rifier en plus si le parking est plein ou non.
            if (m_PatternsSwitch.getDownstreamPattern()->getPatternType() == PT_Walk && m_pParking->IsFull())
            {
                bCanEnter = false;
            }
        }

        Cost & cost = costVariation.getData()->operator[](pSubPop);

        double dbTravelTime = bCanEnter ? 0 : std::numeric_limits<double>::infinity();

        cost.setTravelTime(dbTravelTime);

        switch (mapCostFunctions.at(pSubPop))
        {
            // Pour ce type de pÃ©nalitÃ©, le marginal est le temps de parcours
        case SymuCore::CF_Marginals:
            cost.setUsedCostValue(dbTravelTime);
            cost.setOtherCostValue(CF_TravelTime, dbTravelTime);
            break;
        case SymuCore::CF_TravelTime:
            
            cost.setUsedCostValue(dbTravelTime);
            cost.setOtherCostValue(CF_Marginals, dbTravelTime);
            break;
        default:
            assert(false); // unimplemented cost function
            break;
        }
    }
}
