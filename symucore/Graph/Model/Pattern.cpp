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

#include "Pattern.h"

using namespace SymuCore;

unsigned int Pattern::s_uniqueIDCounter = 0;

Pattern::Pattern() : m_uniqueID(s_uniqueIDCounter++), m_pLink(NULL), m_ePatternType(PT_Undefined), m_dbFuncClass(0.0)
{
    //Default constructor
}

Pattern::Pattern(Link* pLink, PatternType ePatternType) :
m_uniqueID(s_uniqueIDCounter++)
{
    m_pLink = pLink;
    m_ePatternType = ePatternType;
    m_dbFuncClass = 0.0;
}

Pattern::~Pattern()
{
    
}

Link* Pattern::getLink() const
{
    return m_pLink;
}

PatternType Pattern::getPatternType() const
{
    return m_ePatternType;
}

int Pattern::getUniqueID() const
{
    return m_uniqueID;
}

double Pattern::getFuncClass() const
{
    return m_dbFuncClass;
}

void Pattern::setFuncClass(double dbFuncClass)
{
    m_dbFuncClass = dbFuncClass;
}


