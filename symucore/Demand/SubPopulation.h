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

﻿#pragma once

#ifndef SYMUCORE_SubPopulation_H
#define SYMUCORE_SubPopulation_H

#include "SymuCoreExports.h"

#include "Utils/SymuCoreConstants.h"

#include <vector>
#include <string>

#pragma warning(push)
#pragma warning(disable : 4251)

namespace SymuCore {

class Trip;
class Motive;
class Population;

class SYMUCORE_DLL_DEF SubPopulation{

public:

    SubPopulation();
    SubPopulation(const std::string& strName);
    virtual ~SubPopulation();

    const std::vector<Trip *> & GetListUsers() const;
    std::vector<Trip *> & GetListUsers();
    std::string GetStrName() const;
    Population *GetPopulation() const;
    Motive* GetMotive() const;
    double GetInitialWalkSpeed() const;
    double GetInitialWalkRadius() const;

    void SetListUsers(const std::vector<Trip *> &listUsers);
    void SetPopulation(Population *pPopulation);
    void SetStrName(const std::string &strName);
    void SetMotive(Motive *Motive);
    void SetInitialWalkSpeed(double dbWalkSpeed);
    void SetInitialWalkRadius(double dbWalkRadius);



protected:
    std::string                    m_strName; // name of this SubPopulation
    Population*                    m_pPopulation; //Population that own this SubPopulation
    std::vector<Trip*>             m_listUsers; //list of Users for this SubPopulation
    Motive*                        m_pMotive; //Motive for the SubPopulation
    double                         m_dbInitialWalkSpeed; //Initial walking speed (default defined in Population)
    double                         m_dbInitialWalkRadius;//Initial walk radius (default defined in Population)
};
}

#pragma warning(pop)

#endif // SYMUCORE_SubPopulation_H
