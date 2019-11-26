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

﻿#include "Graph.h"

#include "Node.h"
#include "Link.h"
#include "PublicTransport/PublicTransportLine.h"
#include "Demand/VehicleType.h"
#include "Demand/MacroType.h"
#include "Demand/DefaultMacroType.h"
#include "MultiLayersGraph.h"
#include "Demand/Destination.h"
#include "Demand/Origin.h"
#include "NullPattern.h"
#include "Penalty.h"
#include "NullPenalty.h"
#include "Driving/DrivingPattern.h"
#include "Utils/Point.h"

using namespace SymuCore;

Graph::Graph()
{
    m_pParent = NULL;
    m_eServiceType = ST_Undefined;
    m_bOwnODsAndMacroTypes = true;
}

Graph::Graph(ServiceType eServiceType)
{
    m_pParent = NULL;
    m_eServiceType = eServiceType;
    m_bOwnODsAndMacroTypes = true;
}

Graph::Graph(MultiLayersGraph * pParent, ServiceType eServiceType)
{
    m_pParent = pParent;
    m_eServiceType = eServiceType;
    m_bOwnODsAndMacroTypes = true;
}

Graph::~Graph()
{
    // if a parent graph is defined, it has ownership and will destroy the graph objets itself
    if (m_pParent == NULL)
    {
        for (size_t i = 0; i < m_listLinks.size(); i++)
        {
            delete m_listLinks[i];
        }
        for (size_t i = 0; i < m_listNodes.size(); i++)
        {
            delete m_listNodes[i];
        }
        if (m_bOwnODsAndMacroTypes)
        {
            for (size_t i = 0; i < m_listDestination.size(); i++)
            {
                delete m_listDestination[i];
            }
            for (size_t i = 0; i < m_listOrigin.size(); i++)
            {
                delete m_listOrigin[i];
            }
            for (size_t i = 0; i < m_listMacroTypes.size(); i++)
            {
                delete m_listMacroTypes[i];
            }
        }
        for (size_t i = 0; i < m_listTransportLines.size(); i++)
        {
            delete m_listTransportLines[i];
        }
    }
}

Node * Graph::CreateAndAddNode(const std::string & name, NodeType nodeType, const Point * pCoordinates)
{
    if (m_pParent)
    {
        Node * pNewNode = m_pParent->CreateAndAddNode(name, nodeType, pCoordinates);
        pNewNode->setParent(this);
        m_listNodes.push_back(pNewNode);
        return pNewNode;
    }
    else
    {
        Node * pNewNode = new Node(name, nodeType, pCoordinates);
        pNewNode->setParent(this);
        m_listNodes.push_back(pNewNode);
        return pNewNode;
    }
}

Destination * Graph::GetOrCreateDestination(const std::string & strNodeName)
{
    if (m_pParent)
    {
        return m_pParent->GetOrCreateDestination(strNodeName);
    }
    else
    {
        for(size_t iDest = 0; iDest < m_listDestination.size(); iDest++)
        {
			Destination * destination = m_listDestination.at(iDest);
            if (destination->getStrNodeName() == strNodeName)
            {
                //already inserted
                return destination;
            }
        }
        Destination * pResult = new Destination(strNodeName);
        m_listDestination.push_back(pResult);
        return pResult;
    }
}

Destination * Graph::GetDestinationIfExists(const std::string & strNodeName)
{
    if (m_pParent)
    {
        return m_pParent->GetDestinationIfExists(strNodeName);
    }
    else
    {
        for(size_t iDest = 0; iDest < m_listDestination.size(); iDest++)
        {
			Destination * destination = m_listDestination.at(iDest);
            if (destination->getStrNodeName() == strNodeName)
            {
                //already inserted
                return destination;
            }
        }
        return NULL;
    }
}

void Graph::CreateLinkToDestination(Node* pNode)
{
    if (m_pParent)
    {
        return m_pParent->CreateLinkToDestination(pNode);
    }
    else
    {
        for(size_t iDest = 0; iDest < m_listDestination.size(); iDest++)
        {
			Destination * destination = m_listDestination.at(iDest);
            if(destination->getStrNodeName() == pNode->getStrName())
            {
                Node * destNode;
                bool bUpdateCoordinates = false;
                if(destination->getNode() != NULL)
                {
                    destNode = destination->getNode();
                    bUpdateCoordinates = true;
                }
                else
                {
                    destNode = CreateAndAddNode(destination->getStrNodeName(), NT_InterLayerDestination, pNode->getCoordinates());
                    destination->setSelfNode(destNode);
                }

                //Construct a link
                Link* newLink = CreateAndAddLink(pNode, destNode);

                //Update coordinates if necessary
                if (bUpdateCoordinates)
                {
                    destination->computeCoordinates();
                }

                //Construct null pattern for the new link
                NullPattern* newPattern = new NullPattern(newLink);
                newLink->getListPatterns().push_back(newPattern);

                //Constuct associated new penalties for Node pNode :
                for (size_t iUpstreamLink = 0; iUpstreamLink < pNode->getUpstreamLinks().size(); iUpstreamLink++)
                {
                    Link * pUpstreamLink = pNode->getUpstreamLinks()[iUpstreamLink];
                    for (size_t iUpstreamPattern = 0; iUpstreamPattern < pUpstreamLink->getListPatterns().size(); iUpstreamPattern++)
                    {
                        Pattern * pUpstreamPattern = pUpstreamLink->getListPatterns()[iUpstreamPattern];
                        AbstractPenalty * pPenalty = new NullPenalty(pNode, PatternsSwitch(pUpstreamPattern, newPattern));
                        pNode->getMapPenalties()[pUpstreamPattern][newPattern] = pPenalty;
                    }
                }

                return;
            }
        }
    }
}

void Graph::RemoveDestination(Destination * pDestination)
{
    for (size_t iDestination = 0; iDestination < m_listDestination.size(); iDestination++)
    {
        if (m_listDestination[iDestination] == pDestination)
        {
            m_listDestination.erase(m_listDestination.begin() + iDestination);
            delete pDestination;
            break;
        }
    }
}

Origin * Graph::GetOrCreateOrigin(const std::string & strNodeName)
{
    if (m_pParent)
    {
        return m_pParent->GetOrCreateOrigin(strNodeName);
    }
    else
    {
		for(size_t iOrigin = 0; iOrigin < m_listOrigin.size(); iOrigin++)
        {
			Origin * origin = m_listOrigin.at(iOrigin);
            if(origin->getStrNodeName() == strNodeName)
            {
                //already inserted
                return origin;
            }
        }
        Origin * pResult = new Origin(strNodeName);
        m_listOrigin.push_back(pResult);
        return pResult;
    }
}

Origin * Graph::GetOriginIfExists(const std::string & strNodeName)
{
    if (m_pParent)
    {
        return m_pParent->GetOriginIfExists(strNodeName);
    }
    else
    {
        for(size_t iOrigin = 0; iOrigin < m_listOrigin.size(); iOrigin++)
        {
			Origin * origin = m_listOrigin.at(iOrigin);
            if (origin->getStrNodeName() == strNodeName)
            {
                //already inserted
                return origin;
            }
        }
        return NULL;
    }
}

void Graph::CreateLinkToOrigin(Node* pNode)
{
    if (m_pParent)
    {
        m_pParent->CreateLinkToOrigin(pNode);
    }
    else
    {
        for(size_t iOrigin = 0; iOrigin < m_listOrigin.size(); iOrigin++)
        {
			Origin * origin = m_listOrigin.at(iOrigin);
            if(origin->getStrNodeName() == pNode->getStrName())
            {
                Node * originNode;
                bool bUpdateCoordinates = false;
                if(origin->getNode() != NULL)
                {
                    originNode = origin->getNode();
                    bUpdateCoordinates = true;
                }
                else
                {
                    originNode = CreateAndAddNode(origin->getStrNodeName(), NT_InterLayerOrigin, pNode->getCoordinates());
                    origin->setSelfNode(originNode);
                }

                //Construct a link
                Link* newLink = CreateAndAddLink(originNode, pNode);

                //Update coordinates if necessary
                if (bUpdateCoordinates)
                {
                    origin->computeCoordinates();
                }

                //Construct a null Pattern for the new link
                NullPattern* newPattern = new NullPattern(newLink);
                newLink->getListPatterns().push_back(newPattern);

                //Constuct associated new penalties for Node pNode :
                for (size_t iDownstreamLink = 0; iDownstreamLink < pNode->getDownstreamLinks().size(); iDownstreamLink++)
                {
                    Link * pDownstreamLink = pNode->getDownstreamLinks()[iDownstreamLink];
                    for (size_t iDownstreamPattern = 0; iDownstreamPattern < pDownstreamLink->getListPatterns().size(); iDownstreamPattern++)
                    {
                        Pattern * pDownstreamPattern = pDownstreamLink->getListPatterns()[iDownstreamPattern];
                        AbstractPenalty * pPenalty = new NullPenalty(pNode, PatternsSwitch(newPattern, pDownstreamPattern));
                        pNode->getMapPenalties()[newPattern][pDownstreamPattern] = pPenalty;
                    }
                }

                return;
            }
        }
    }
}

void Graph::RemoveOrigin(Origin * pOrigin)
{
    for (size_t iOrigin = 0; iOrigin < m_listOrigin.size(); iOrigin++)
    {
        if (m_listOrigin[iOrigin] == pOrigin)
        {
            m_listOrigin.erase(m_listOrigin.begin() + iOrigin);
            delete pOrigin;
            break;
        }
    }
}

Link * Graph::CreateAndAddLink(Node * pUpstreamNode, Node * pDownstreamNode)
{
    if (m_pParent)
    {
        Link * pNewLink = m_pParent->CreateAndAddLink(pUpstreamNode, pDownstreamNode);
        pNewLink->setParent(this);
        m_listLinks.push_back(pNewLink);
        return pNewLink;
    }
    else
    {
        Link * pNewLink = new Link(pUpstreamNode, pDownstreamNode);
        pUpstreamNode->addDownstreamLink(pNewLink);
        pDownstreamNode->addUpstreamLink(pNewLink);
        pNewLink->setParent(this);
        m_listLinks.push_back(pNewLink);
        return pNewLink;
    }
}


void Graph::AddMacroType(MacroType * pMacroType)
{
    if (m_pParent)
    {
        m_pParent->AddMacroType(pMacroType);
    }else{
        m_listMacroTypes.push_back(pMacroType);
    }
}

void Graph::AddPublicTransportLine(PublicTransportLine *pPublicTransportLine)
{
    if (m_pParent)
    {
        m_pParent->AddPublicTransportLine(pPublicTransportLine);
    }else{
        m_listTransportLines.push_back(pPublicTransportLine);
    }
}

const std::vector<Link *>& Graph::GetListLinks() const
{
    return m_listLinks;
}

const std::vector<Node *>& Graph::GetListNodes() const
{
    return m_listNodes;
}

Node * Graph::GetNode(const std::string & strNodeName) const
{
    for (size_t iNode = 0; iNode < m_listNodes.size(); iNode++)
    {
        if (m_listNodes[iNode]->getStrName() == strNodeName)
        {
            return m_listNodes[iNode];
        }
    }
    return NULL;
}

const std::vector<PublicTransportLine *>& Graph::GetListTransportLines() const
{
    return m_listTransportLines;
}

const std::vector<Destination *>& Graph::GetListDestination() const
{
    return m_listDestination;
}

const std::vector<Origin *>& Graph::GetListOrigin() const
{
    return m_listOrigin;
}

void Graph::SetListDestination(const std::vector<Destination *> & destinations)
{
    m_listDestination = destinations;
    m_bOwnODsAndMacroTypes = false;
}

void Graph::SetListOrigin(const std::vector<Origin *> & origins)
{
    m_listOrigin = origins;
    m_bOwnODsAndMacroTypes = false;
}

void Graph::SetListMacroTypes(const std::vector<MacroType*> & macroTypes)
{
    m_listMacroTypes = macroTypes;
    m_bOwnODsAndMacroTypes = false;
}

const std::vector<MacroType *> &Graph::GetListMacroTypes() const
{
    // Macro types are only defined in the root graph
    if (m_pParent)
    {
        return m_pParent->GetListMacroTypes();
    }
    else
    {
        return m_listMacroTypes;
    }
}

std::vector<MacroType *> &Graph::GetListMacroTypes()
{
    // Macro types are only defined in the root graph
    if (m_pParent)
    {
        return m_pParent->GetListMacroTypes();
    }
    else
    {
        return m_listMacroTypes;
    }
}

ServiceType Graph::GetServiceType() const
{
    return m_eServiceType;
}

void Graph::GetListServiceType(std::vector<ServiceType> &listServiceType)
{
    MultiLayersGraph* pMultilayersGraph = dynamic_cast<MultiLayersGraph*>(this);
    if(pMultilayersGraph)
    {
        for(size_t iLayers=0; iLayers<pMultilayersGraph->GetListLayers().size(); iLayers++)
        {
            pMultilayersGraph->GetListLayers()[iLayers]->GetListServiceType(listServiceType);
        }
    }

    if(m_eServiceType != ST_Undefined)
    {
        listServiceType.push_back(m_eServiceType);
    }
}

void Graph::setParent(MultiLayersGraph * pParent)
{
    m_pParent = pParent;
}

MultiLayersGraph* Graph::getParent()
{
    return m_pParent;
}

bool Graph::hasChild(Node * pNode) const
{
    return pNode->getParent() == this;
}

