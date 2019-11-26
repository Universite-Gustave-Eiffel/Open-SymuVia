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

#ifndef SYMUCORE_LINK_H
#define SYMUCORE_LINK_H

#include "SymuCoreExports.h"

#include "Utils/SymuCoreConstants.h"

#include <vector>

#pragma warning(push)
#pragma warning(disable : 4251)

namespace SymuCore {

class Node;
class Graph;
class Pattern;

class SYMUCORE_DLL_DEF Link {

public:

    Link();
    Link(Node* pUpstreamNode, Node* pDownstreamNode);
    virtual ~Link();

    //getters
    Graph * getParent() const;
    Node* getUpstreamNode() const;
    Node* getDownstreamNode() const;
	const std::vector<Pattern*> & getListPatterns() const;
    std::vector<Pattern*> & getListPatterns();

    //setters
    void setParent(Graph * pGraph);
    void setUpstreamNode(Node *pUpstreamNode);
    void setDownstreamNode(Node *pDownstreamNode);

private:

    Graph*                                                      m_pParent;

    Node*                                                       m_pUpstreamNode; //pointer to the upstream Node
    Node*                                                       m_pDownstreamNode; //pointer to the downstream Node
	std::vector<Pattern*>                                       m_listPatterns; //all the patterns for this link

};
}

#pragma warning(pop)

#endif // SYMUCORE_LINK_H
