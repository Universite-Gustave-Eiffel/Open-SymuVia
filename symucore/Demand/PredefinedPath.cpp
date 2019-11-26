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

﻿#include "PredefinedPath.h"

using namespace SymuCore;


PredefinedPath::PredefinedPath()
{
    m_pJunction = NULL;
}

PredefinedPath::PredefinedPath(const std::vector<Pattern*>& listPattern, Node * pJunction, const std::string & name, double dbCoeff)
: m_Path(listPattern)
{
    m_pJunction = pJunction;
    m_strName = name;
    m_dbCoeff = dbCoeff;
}

PredefinedPath::~PredefinedPath()
{

}

double PredefinedPath::getCoeff() const
{
    return m_dbCoeff;
}

const Path & PredefinedPath::getPath() const
{
    return m_Path;
}

const std::string & PredefinedPath::getStrName() const
{
    return m_strName;
}

Node * PredefinedPath::getJunction() const
{
    return m_pJunction;
}
