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

#include "Destination.h"

#include "Utils/Point.h"
#include "Graph/Model/Link.h"
#include "Graph/Model/Pattern.h"

#include <stddef.h>

using namespace SymuCore;

Destination::Destination()
{
    m_pSelfNode = NULL;
    m_pPattern = NULL;
}

Destination::Destination(std::string strNodeName)
{
    m_strNodeName = strNodeName;
    m_pSelfNode = NULL;
    m_pPattern = NULL;
}

Destination::Destination(Node *pSelfNode)
{
    m_pSelfNode = pSelfNode;
    m_strNodeName = pSelfNode->getStrName();
    m_pPattern = NULL;
}

Destination::~Destination()
{

}

const std::string & Destination::getStrNodeName() const
{
    return m_strNodeName;
}

void Destination::setStrNodeName(const std::string &strNodeName)
{
    m_strNodeName = strNodeName;
}

Node *Destination::getNode() const
{
    return m_pSelfNode;
}

void Destination::setSelfNode(Node *pSelfNode)
{
    m_pSelfNode = pSelfNode;
}

void Destination::setPatternAsDestination(Pattern * pDestinationPattern)
{
    m_pPattern = pDestinationPattern;
    m_pSelfNode = pDestinationPattern->getLink()->getDownstreamNode();
}

Pattern * Destination::getPattern() const
{
    return m_pPattern;
}

void Destination::computeCoordinates()
{
    int nbPoints = 0;
    Point pt;

    for (size_t iUpstreamLink = 0; iUpstreamLink < m_pSelfNode->getUpstreamLinks().size(); iUpstreamLink++)
    {
        Point * pUpstreamCoords = m_pSelfNode->getUpstreamLinks()[iUpstreamLink]->getUpstreamNode()->getCoordinates();
        if (pUpstreamCoords)
        {
            pt.setX(pt.getX() + pUpstreamCoords->getX());
            pt.setY(pt.getY() + pUpstreamCoords->getY());
            nbPoints++;
        }
    }

    if (nbPoints > 0)
    {
        pt.setX(pt.getX() / nbPoints);
        pt.setY(pt.getY() / nbPoints);
        m_pSelfNode->setCoordinates(&pt);
    }
    else
    {
        m_pSelfNode->setCoordinates(NULL);
    }
}
