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
#include "VehicleTypePenalty.h"

#include "reseau.h"

#include <Demand/SubPopulation.h>
#include <Demand/Population.h>

using namespace SymuCore;

SymuCore::Cost VehicleTypePenalty::m_NullCost(0, 0);
SymuCore::Cost VehicleTypePenalty::m_InfCost(std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());

VehicleTypePenalty::VehicleTypePenalty(Node* pParentNode, const PatternsSwitch & patternsSwitch, Reseau * pNetwork, const std::set<TypeVehicule*> & lstALlowedVehicleTypes)
: SymuCore::AbstractPenalty(pParentNode, patternsSwitch),
m_pNetwork(pNetwork),
m_lstAllowedVehicleTypes(lstALlowedVehicleTypes)
{
}

VehicleTypePenalty::~VehicleTypePenalty()
{

}

Cost* VehicleTypePenalty::getPenaltyCost(int iSimulationInstance, double t, SymuCore::SubPopulation* pSubPopulation)
{
    SymuCore::MacroType * pMacroType = pSubPopulation->GetPopulation()->GetMacroType();
    TypeVehicule * pTypeVeh = m_pNetwork->GetVehiculeTypeFromMacro(pMacroType);
    if (m_lstAllowedVehicleTypes.find(pTypeVeh) != m_lstAllowedVehicleTypes.end())
    {
        return &m_NullCost;
    }
    else
    {
        return &m_InfCost;
    }
}

void VehicleTypePenalty::fillFromSecondaryInstance(AbstractPenalty * pFrom, int iInstance)
{
    //NO OP
}