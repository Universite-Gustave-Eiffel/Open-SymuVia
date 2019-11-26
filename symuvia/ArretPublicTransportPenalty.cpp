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
#include "ArretPublicTransportPenalty.h"

#include "SymuViaPublicTransportPattern.h"
#include "usage/PublicTransportFleet.h"
#include "reseau.h"
#include "arret.h"
#include "tuyau.h"
#include "usage/PublicTransportLine.h"

#include <Graph/Model/Node.h>
#include <Graph/Model/Graph.h>
#include <Graph/Model/Penalty.h>

#include <Utils/TravelTimeUtils.h>

#include <boost/make_shared.hpp>

using namespace SymuCore;

ArretPublicTransportPenalty::ArretPublicTransportPenalty(SymuCore::Node* pParentNode, const SymuCore::PatternsSwitch & patternsSwitch, Reseau * pNetwork, Arret * pArret)
: SymuCore::Penalty(pParentNode, patternsSwitch)
{
    m_pNetwork = pNetwork;
    m_pArret = pArret;
}

ArretPublicTransportPenalty::~ArretPublicTransportPenalty()
{

}

void ArretPublicTransportPenalty::ComputeTravelTimeAndMarginalWhenTakinNewLine(SymuViaPublicTransportPattern* patternAvl, bool bComingFromAnotherLine, double & dbTravelTime, double & dbMarginal)
{
    TravelTimeUtils::ComputeTravelTimeAndMarginalAtBusStation(m_pNetwork->GetMeanBusExitTime(),
        patternAvl->GetSymuViaLine()->GetLastFrequency(m_pArret),
        m_pArret->getLastRatioMontee(patternAvl->GetSymuViaLine()),
        patternAvl->GetSymuViaLine()->GetLastStopDuration(m_pArret),
        bComingFromAnotherLine,
        dbTravelTime, dbMarginal); // sorties
}

void ArretPublicTransportPenalty::fillMeasuredCostsForTravelTimesUpdatePeriod(int iTravelTimesUpdatePeriodIndex, const std::vector<SubPopulation*>& listSubPopulations, const std::map<SymuCore::SubPopulation*, SymuCore::CostFunction> & mapCostFunctions)
{
    double dbTravelTime;
    double dbMarginal;

    SymuViaPublicTransportPattern* patternAmt = dynamic_cast<SymuViaPublicTransportPattern*>(m_PatternsSwitch.getUpstreamPattern());
    SymuViaPublicTransportPattern* patternAvl = dynamic_cast<SymuViaPublicTransportPattern*>(m_PatternsSwitch.getDownstreamPattern());
    if (patternAmt && patternAvl)
    {
        if (patternAmt->getLine() == patternAvl->getLine())
        {
            // Cas du passager qui reste et ne descend pas Ã  l'arrÃªt :
            dbTravelTime = patternAmt->GetSymuViaLine()->GetLastStopDuration(m_pArret);
            dbMarginal = dbTravelTime;
        }
        else
        {
            ComputeTravelTimeAndMarginalWhenTakinNewLine(patternAvl, true, dbTravelTime, dbMarginal);
        }
    }
    else if (patternAmt)
    {
        // Cas du pattern marche Ã  pieds en aval : facile, il n'y a qu'Ã  descendre du bus
        dbTravelTime = m_pNetwork->GetMeanBusExitTime();
        dbMarginal = dbTravelTime;
    }
    else if (patternAvl)
    {
        // Cas du pattern marche Ã  pieds en amont : idem que pour un changement de ligne sauf qu'on n'a pas de temps de descente
        ComputeTravelTimeAndMarginalWhenTakinNewLine(patternAvl, false, dbTravelTime, dbMarginal);
    }
    else
    {
        // Cas marche vers marche : aucune pÃ©nalitÃ©.
        // pour le SG, on interdit le transferts via marche Ã  pieds aux arrÃªts. Sinon, on aurait 
        // le problÃ¨me du nombre d'itinÃ©raires demandÃ©s Ã  gÃ©rer car doublons Ã  supprimer.
        if (m_pNetwork->IsWithPublicTransportGraph())
        {
            dbTravelTime = std::numeric_limits<double>::infinity();
            dbMarginal = std::numeric_limits<double>::infinity();
        }
        else
        {
            dbTravelTime = 0;
            dbMarginal = 0;
        }
    }

    std::map<SubPopulation*, Cost> * pMapCosts = getTemporalCosts().front().getData((size_t)iTravelTimesUpdatePeriodIndex);

    for (size_t iSubPop = 0; iSubPop < listSubPopulations.size(); iSubPop++)
    {
        SubPopulation * pSubPop = listSubPopulations.at(iSubPop);
        Cost & cost = pMapCosts->operator[](pSubPop);

        cost.setTravelTime(dbTravelTime);

        switch (mapCostFunctions.at(pSubPop))
        {
        case SymuCore::CF_Marginals:
            cost.setOtherCostValue(SymuCore::CF_TravelTime, dbTravelTime);
            cost.setUsedCostValue(dbMarginal);
            break;
        case SymuCore::CF_TravelTime:
            cost.setOtherCostValue(SymuCore::CF_Marginals, dbMarginal);
            cost.setUsedCostValue(dbTravelTime);
            break;
        default:
            assert(false); // unimplemented cost function
            break;
        }
    }
}
