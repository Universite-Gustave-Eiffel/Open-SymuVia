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

﻿#include "Cost.h"

#include <limits>

using namespace SymuCore;

Cost::Cost()
{
	m_dbCostValue = 0;
	m_dbTravelTime = 0;
}

Cost::Cost(double dCostValue, double dTravelTime)
{
	m_dbCostValue = dCostValue;
    m_dbTravelTime = dTravelTime;
}

Cost::~Cost()
{

}

void Cost::setUsedCostValue(double dCostValue)
{
    m_dbCostValue = dCostValue;
}

void Cost::setOtherCostValue(CostFunction eCostFunction, double dbValue)
{
    m_mapOtherCosts[eCostFunction] = dbValue;
}

double Cost::getOtherCostValue(CostFunction eCostFunction) const
{
    std::map<CostFunction, double>::const_iterator iter = m_mapOtherCosts.find(eCostFunction);
    if (iter != m_mapOtherCosts.end())
    {
        return iter->second;
    }
    else
    {
        return 0;
    }
}

double Cost::getCostValue(CostFunction eCostFunction) const
{
    std::map<CostFunction, double>::const_iterator iter = m_mapOtherCosts.find(eCostFunction);
    if (iter != m_mapOtherCosts.end())
    {
        return iter->second;
    }
    else
    {
        return m_dbCostValue;
    }
}

double Cost::getCostValue() const
{
	return m_dbCostValue;
}

void Cost::setTravelTime(double dTravelTime)
{
    m_dbTravelTime = dTravelTime;
}

double Cost::getTravelTime() const
{
    return m_dbTravelTime;
}

// OTK - TODO - Pourquoi pas un vrai opérateur + ?
void Cost::plus(Cost * otherCost)
{
    // rmq. we don't bother to process m_mapOtherCosts here since it is only use for now to output costs pattern per pattern or penalty per penalty.
    if(!otherCost)
    {
        m_dbCostValue = std::numeric_limits<double>::infinity();
        m_dbTravelTime = std::numeric_limits<double>::infinity();
    }else{
        m_dbCostValue += otherCost->getCostValue();
        m_dbTravelTime += otherCost->getTravelTime();
    }
}
