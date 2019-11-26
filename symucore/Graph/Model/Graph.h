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

#ifndef SYMUCORE_GRAPH_H
#define SYMUCORE_GRAPH_H

#include "SymuCoreExports.h"

#include "Utils/SymuCoreConstants.h"

#include <vector>
#include <string>

#pragma warning(push)
#pragma warning(disable : 4251)

namespace SymuCore {

class MultiLayersGraph;
class Link;
class Node;
class VehicleType;
class PublicTransportLine;
class Point;
class MacroType;
class Origin;
class Destination;
class Pattern;

class SYMUCORE_DLL_DEF Graph {

public:

    Graph();
    Graph(ServiceType eServiceType);
    Graph(MultiLayersGraph * pParent, ServiceType eServiceType);
    virtual ~Graph();

    Node * CreateAndAddNode(const std::string & name, NodeType nodeType, const Point * pCoordinates);

    Link * CreateAndAddLink(Node * pUpstreamNode, Node * pDownstreamNode);

    Destination * GetOrCreateDestination(const std::string & strNodeName);
    Destination * GetDestinationIfExists(const std::string & strNodeName);
    void CreateLinkToDestination(Node *pNode);
    void RemoveDestination(Destination * pDestination);

    Origin * GetOrCreateOrigin(const std::string & strNodeName);
    Origin * GetOriginIfExists(const std::string & strNodeName);
    void CreateLinkToOrigin(Node *pNode);
    void RemoveOrigin(Origin * pOrigin);

    void AddMacroType(MacroType * pMacroType);
    void AddPublicTransportLine(PublicTransportLine * pPublicTransportLine);

    const std::vector<Link *>& GetListLinks() const;
    const std::vector<Node *>& GetListNodes() const;
    const std::vector<Destination *>& GetListDestination() const;
    const std::vector<Origin *> &GetListOrigin() const;
    const std::vector<MacroType *> &GetListMacroTypes() const;
    std::vector<MacroType *> &GetListMacroTypes();
    const std::vector<PublicTransportLine *>& GetListTransportLines() const;
    ServiceType GetServiceType() const;
    void GetListServiceType(std::vector<ServiceType> &listServiceType);

    void SetListDestination(const std::vector<Destination *> & destinations);
    void SetListOrigin(const std::vector<Origin *> & origins);
    void SetListMacroTypes(const std::vector<MacroType*> & macroTypes);

    void setParent(MultiLayersGraph * pParent);
    MultiLayersGraph* getParent();

    virtual bool hasChild(Node * pNode) const;

    Node * GetNode(const std::string & strNodeName) const;


protected:

    MultiLayersGraph*                       m_pParent; // Parent graph if any

    std::vector<Link*>                      m_listLinks; //list of all the links for this graph all his sub-graph
    std::vector<Node*>                      m_listNodes; //list of all the nodes for this graph all his sub-graph
    std::vector<Destination*>               m_listDestination; //list of all the links for this graph all his sub-graph
    std::vector<Origin*>                    m_listOrigin; //list of all the nodes for this graph all his sub-graph
    ServiceType                             m_eServiceType; //Define the service proposed for this layer
    std::vector<MacroType*>                 m_listMacroTypes; //list of all the macro types for this graph all his sub-graph
    std::vector<PublicTransportLine*>       m_listTransportLines; //list of all public transport lines for this graph all his sub-graph

    bool                                    m_bOwnODsAndMacroTypes;

};
}

#pragma warning(pop)

#endif // SYMUCORE_GRAPH_H
