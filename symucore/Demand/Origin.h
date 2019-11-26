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

#ifndef SYMUCORE_ORIGIN_H
#define SYMUCORE_ORIGIN_H

#include <string>
#include <vector>
#include "Graph/Model/Node.h"

#include "SymuCoreExports.h"

#pragma warning(push)
#pragma warning(disable : 4251)

namespace SymuCore {

class Node;

class SYMUCORE_DLL_DEF Origin{

public:

    Origin();
    Origin(const std::string & strNodeName);
    virtual ~Origin();

    //getters
    Node *getNode() const;
    const std::string & getStrNodeName() const;
    Pattern * getPattern() const;

    //setters
    void setSelfNode(Node *pSelfNode);
    void setPatternAsOrigin(Pattern * pPattern);

    void computeCoordinates();


private:
    Node*                          m_pSelfNode; //Node tha represent the Origin
    std::string                    m_strNodeName; // Name of the Node
    Pattern*                       m_pPattern; // Target Pattern for the origin if the origin is a pattern and not a node
};
}

#pragma warning(pop)

#endif // SYMUCORE_ORIGIN_H
