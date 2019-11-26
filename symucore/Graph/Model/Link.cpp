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

#include "Link.h"

#include "Pattern.h"

using namespace SymuCore;


Link::Link() : m_pParent(NULL), m_pUpstreamNode(NULL), m_pDownstreamNode(NULL)
{

}

Link::Link(Node * pUpstreamNode, Node * pDownstreamNode)
: m_pParent(NULL)
{
    m_pUpstreamNode=pUpstreamNode;
    m_pDownstreamNode=pDownstreamNode;
}

Link::~Link()
{
    for (size_t i = 0; i < m_listPatterns.size(); i++)
    {
        delete m_listPatterns[i];
    }
}

Graph * Link::getParent() const
{
    return m_pParent;
}

Node* Link::getUpstreamNode() const
{
    return m_pUpstreamNode;
}

Node* Link::getDownstreamNode() const
{
    return m_pDownstreamNode;
}

const std::vector<Pattern*> & Link::getListPatterns() const
{
    return m_listPatterns;
}

std::vector<Pattern*> & Link::getListPatterns()
{
    return m_listPatterns;
}

//setters

void Link::setParent(Graph * pGraph)
{
    m_pParent = pGraph;
}

void Link::setUpstreamNode(Node *pUpstreamNode)
{
    m_pUpstreamNode = pUpstreamNode;
}

void Link::setDownstreamNode(Node *pDownstreamNode)
{
    m_pDownstreamNode = pDownstreamNode;
}










