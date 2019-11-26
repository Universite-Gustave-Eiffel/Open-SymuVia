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

﻿#include "SubPopulation.h"

#include"Demand/Motive.h"
#include"Demand/Trip.h"
#include"Demand/Population.h"

using namespace SymuCore;

SubPopulation::SubPopulation()
{
    m_dbInitialWalkSpeed = -1;
    m_dbInitialWalkRadius = -1;
    m_pPopulation = NULL;
    m_pMotive = NULL;
}

SubPopulation::SubPopulation(const std::string& strName)
{
    m_strName = strName;
    m_dbInitialWalkSpeed = -1;
    m_dbInitialWalkRadius = -1;
    m_pPopulation = NULL;
    m_pMotive = NULL;
}

SubPopulation::~SubPopulation()
{
}

std::string SubPopulation::GetStrName() const
{
    return m_strName;
}

void SubPopulation::SetStrName(const std::string &strName)
{
    m_strName = strName;
}

Motive *SubPopulation::GetMotive() const
{
    return m_pMotive;
}

void SubPopulation::SetMotive(Motive *Motive)
{
    m_pMotive = Motive;
}

void SubPopulation::SetPopulation(Population *pPopulation)
{
    m_pPopulation = pPopulation;
}

Population *SubPopulation::GetPopulation() const
{
    return m_pPopulation;
}

void SubPopulation::SetInitialWalkSpeed(double dbWalkSpeed)
{
    m_dbInitialWalkSpeed = dbWalkSpeed;
}

double SubPopulation::GetInitialWalkSpeed() const
{
    if (m_dbInitialWalkSpeed == -1)
        return m_pPopulation->GetInitialWalkSpeed();

    return m_dbInitialWalkSpeed;
}

void SubPopulation::SetInitialWalkRadius(double dbWalkRadius)
{
    m_dbInitialWalkRadius = dbWalkRadius;
}

double SubPopulation::GetInitialWalkRadius() const
{
    if (m_dbInitialWalkRadius == -1)
        return m_pPopulation->GetInitialWalkRadius();

    return m_dbInitialWalkRadius;
}

const std::vector<Trip *> &SubPopulation::GetListUsers() const
{
    return m_listUsers;
}

std::vector<Trip *> &SubPopulation::GetListUsers()
{
    return m_listUsers;
}

void SubPopulation::SetListUsers(const std::vector<Trip *> &listUsers)
{
    m_listUsers = listUsers;
}
