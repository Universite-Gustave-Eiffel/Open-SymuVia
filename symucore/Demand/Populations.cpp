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

#include "Populations.h"

#include "Population.h"
#include "SubPopulation.h"
#include "Motive.h"
#include "MacroType.h"
#include "DefaultMacroType.h"

#include <algorithm>

using namespace SymuCore;

Populations::Populations()
{
}

Populations::~Populations()
{
    Clear();
}

void Populations::Clear()
{
    for (size_t i = 0; i < m_listPopulations.size(); i++)
    {
        delete m_listPopulations[i];
    }
    m_listPopulations.clear();
}

void Populations::DeepCopy(const Populations & source, MultiLayersGraph * pGraph)
{
    Clear();
    for (size_t iPop = 0; iPop < source.getListPopulations().size(); iPop++)
    {
        Population * newPop = new Population();
        newPop->DeepCopy(*source.getListPopulations()[iPop], pGraph);
        m_listPopulations.push_back(newPop);
    }
}

const std::vector<Population*> & Populations::getListPopulations() const
{
    return m_listPopulations;
}

std::vector<Population*> & Populations::getListPopulations()
{
    return m_listPopulations;
}

void Populations::getPopulationAndVehicleType(Population ** pPopulation, SubPopulation ** pSubPopulation, VehicleType ** pVehType, std::string strVehicleTypeName, std::string strMotive) const
{
    *pPopulation = NULL;
    *pSubPopulation = NULL;
    *pVehType = NULL;

    for (size_t i = 0; i < m_listPopulations.size(); i++)
    {
        Population * pPop = m_listPopulations[i];
        if (pPop->GetMacroType())
        {
            if (!strMotive.empty())
            {
                bool bMotiveFound = false;
                for (size_t iSubPop = 0; iSubPop < pPop->GetListSubPopulations().size(); iSubPop++)
                {
                    if (pPop->GetListSubPopulations()[iSubPop]->GetMotive()->GetStrName() == strMotive)
                    {
                        bMotiveFound = true;
                        *pSubPopulation = pPop->GetListSubPopulations()[iSubPop];
                        break;
                    }
                }

                if (!bMotiveFound)
                {
                    continue;
                }
            }

            if (strVehicleTypeName.empty())
            {
                *pPopulation = pPop;
                if (!pPop->GetMacroType()->getListVehicleTypes().empty())
                {
                    *pVehType = pPop->GetMacroType()->getListVehicleTypes().front();
                }
                break;
            }
            else
            {
                VehicleType * pVType = pPop->GetMacroType()->getVehicleType(strVehicleTypeName);
                if (pVType)
                {
                    *pPopulation = pPop;
                    *pVehType = pVType;
                    break;
                }
            }

            // to avoid returning a non null subpopulation on failure
            *pSubPopulation = NULL;
        }
    }
}

Population *Populations::getPopulation(const std::string &strPopulationName) const
{
    Population * pPop = NULL;
    for (size_t i = 0; i < m_listPopulations.size(); i++)
    {
        if (m_listPopulations[i]->GetStrName() == strPopulationName)
        {
            pPop = m_listPopulations[i];
            break;
        }
    }
    return pPop;
}

double Populations::getMaxIntermediateWalkRadius() const
{
    double dbMaxRadius;
    if (m_listPopulations.empty())
    {
        Population tempPop;
        dbMaxRadius = tempPop.GetIntermediateWalkRadius();
    }
    else
    {
        dbMaxRadius = -1;
        for (size_t i = 0; i < m_listPopulations.size(); i++)
        {
            double dbPopMaxRadius = m_listPopulations.at(i)->GetIntermediateWalkRadius();
            dbMaxRadius = std::max<double>(dbPopMaxRadius, dbMaxRadius);
        }
    }
    return dbMaxRadius;
}

double Populations::getMaxInitialWalkRadius() const
{
    double dbMaxRadius;
    if (m_listPopulations.empty())
    {
        Population tempPop;
        dbMaxRadius = tempPop.GetInitialWalkRadius();
    }
    else
    {
        dbMaxRadius = -1;
        for (size_t i = 0; i < m_listPopulations.size(); i++)
        {
            double dbPopMaxRadius = m_listPopulations.at(i)->GetInitialWalkRadius();
            dbMaxRadius = std::max<double>(dbPopMaxRadius, dbMaxRadius);
        }
    }
    return dbMaxRadius;
}

