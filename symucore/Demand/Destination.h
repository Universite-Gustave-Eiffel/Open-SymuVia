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

#ifndef SYMUCORE_DESTINATION_H
#define SYMUCORE_DESTINATION_H

#include "Graph/Model/Node.h"

#include "SymuCoreExports.h"

#include <string>
#include <vector>

#pragma warning(push)
#pragma warning(disable : 4251)

namespace SymuCore {

class Node;

class SYMUCORE_DLL_DEF Destination {

public:

    Destination();
    Destination(std::string strNodeName);
    Destination(Node* pSelfNode);
    virtual ~Destination();

    //getters
    Node *getNode() const;
    const std::string & getStrNodeName() const;
    Pattern * getPattern() const;

    //setters
    void setSelfNode(Node *pSelfNode);
    void setStrNodeName(const std::string &strNodeName);
    void setPatternAsDestination(Pattern * pDestinationPattern);

    void computeCoordinates();



private:
    Node*                          m_pSelfNode; //Node that represent the destination
    std::string                    m_strNodeName; // Name of the Node

    Pattern*                       m_pPattern; // Pattern as the destination if the destination is not a node.
};
}

#pragma warning(pop)

#endif // SYMUCORE_DESTINATION_H
