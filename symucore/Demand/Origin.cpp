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

#include "Origin.h"

#include "Utils/Point.h"
#include "Graph/Model/Link.h"
#include "Graph/Model/Pattern.h"

#include <stddef.h>
#include <cassert>

using namespace SymuCore;

Origin::Origin()
{
    m_pSelfNode = NULL;
    m_pPattern = NULL;
}

Origin::Origin(const std::string & strNodeName)
{
    m_strNodeName = strNodeName;
    m_pSelfNode = NULL;
    m_pPattern = NULL;
}

Origin::~Origin()
{
}

const std::string & Origin::getStrNodeName() const
{
    return m_strNodeName;
}

Node *Origin::getNode() const
{
    return m_pSelfNode;
}

void Origin::setSelfNode(Node *pSelfNode)
{
    m_pSelfNode = pSelfNode;
}

void Origin::setPatternAsOrigin(Pattern * pPattern)
{
    m_pSelfNode = pPattern->getLink()->getUpstreamNode();
    m_pPattern = pPattern;
}

Pattern * Origin::getPattern() const
{
    return m_pPattern;
}

void Origin::computeCoordinates()
{
    int nbPoints = 0;
    Point pt;

    for (size_t iDownstreamLink = 0; iDownstreamLink < m_pSelfNode->getDownstreamLinks().size(); iDownstreamLink++)
    {
        Point * pDownstreamCoords = m_pSelfNode->getDownstreamLinks()[iDownstreamLink]->getDownstreamNode()->getCoordinates();
        if (pDownstreamCoords)
        {
            pt.setX(pt.getX() + pDownstreamCoords->getX());
            pt.setY(pt.getY() + pDownstreamCoords->getY());
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
