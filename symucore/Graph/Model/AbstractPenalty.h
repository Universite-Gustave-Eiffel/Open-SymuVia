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

#ifndef SYMUCORE_AbstractPenalty_H
#define SYMUCORE_AbstractPenalty_H

#include "SymuCoreExports.h"

#include "ListTimeFrame.h"
#include "PatternsSwitch.h"
#include "Cost.h"

#include "Utils/SymuCoreConstants.h"

#include <map>

#pragma warning(push)
#pragma warning(disable : 4251)

namespace SymuCore {

class Pattern;
class Node;
class SubPopulation;

class SYMUCORE_DLL_DEF AbstractPenalty {

public:

    AbstractPenalty();
    AbstractPenalty(Node* pParentNode, const PatternsSwitch & patternsSwitch);
    virtual ~AbstractPenalty();

    //getters
    Node * getNode() const;
    const PatternsSwitch & getPatternsSwitch() const;

    virtual void prepareTimeFrames(double startPeriodTime, double endPeriodTime, double travelTimesUpdatePeriod, const std::vector<SubPopulation*>& listSubPopulations, const std::map<SymuCore::SubPopulation*, SymuCore::CostFunction> & mapCostFunctions, int nbSimulationInstances, int iInstance) = 0;
    virtual void fillMeasuredCostsForTravelTimesUpdatePeriod(int iTravelTimesUpdatePeriodIndex, const std::vector<SubPopulation*>& listSubPopulations, const std::map<SymuCore::SubPopulation*, SymuCore::CostFunction> & mapCostFunctions) = 0;

    // usefull for costs that can't be computed right away at the end of each travel time update period (ie : marginals in spatial mode)
    virtual void postProcessCosts(const std::vector<SubPopulation*>& listSubPopulations, const std::map<SymuCore::SubPopulation*, SymuCore::CostFunction> & mapCostFunctions) {}

    virtual Cost* getPenaltyCost(int iSimulationInstance, double t, SubPopulation* pSubPopulation) = 0;

    virtual void fillFromSecondaryInstance(AbstractPenalty * pFrom, int iInstance) = 0;

protected:
    Node*                   m_pNode; //parent Node
    PatternsSwitch          m_PatternsSwitch; // upstream/downstream pattern couple
};
}

#pragma warning(pop)

#endif // SYMUCORE_AbstractPenalty_H
