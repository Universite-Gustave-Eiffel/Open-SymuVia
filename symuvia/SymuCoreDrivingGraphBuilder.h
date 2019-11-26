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

#ifndef SymuCoreDrivingGraphBuilder_H
#define SymuCoreDrivingGraphBuilder_H

#include "tools.h"

#include <map>
#include <set>
#include <vector>

class Reseau;
class Connexion;
class Tuyau;
class TypeVehicule;
class ZoneDeTerminaison;
class SymuViaTripNode;
class SymuCoreGraphBuilder;
class CPlaque;

namespace SymuCore {
    class Graph;
    class MultiLayersGraph;
    class Link;
    class Node;
    class VehicleType;
    class Populations;
    class Pattern;
    class DrivingPattern;
    class AbstractPenalty;
    class Population;
    class Origin;
    class Destination;
    class Cost;
}

namespace boost {
    namespace posix_time {
        class ptime;
        class time_duration;
    }
}

class SymuCoreDrivingGraphBuilder
{
public:

    SymuCoreDrivingGraphBuilder();

	//! Constructeur
    SymuCoreDrivingGraphBuilder(Reseau* pReseau, bool bForSymuMaster);

	//! Destructeur
    virtual ~SymuCoreDrivingGraphBuilder();

	void SetParent(SymuCoreGraphBuilder * pParent);
    SymuCoreGraphBuilder * GetParent();

    virtual bool CreateDrivingLayer(SymuCore::MultiLayersGraph * pGraph, std::vector<SymuCore::Node*> & lstOrigins, std::vector<SymuCore::Node*> & lstDestinations);

    virtual void BuildPatternsAndPenalties(SymuCore::Graph * pDrivingLayer);

    const std::map<ZoneDeTerminaison*, SymuCore::Node*, LessConnexionPtr<ZoneDeTerminaison> > & GetMapZones() const;
    const std::map<CPlaque*, SymuCore::Node*, LessConnexionPtr<CPlaque> > & GetMapPlaques() const;

    const std::map<SymuCore::Node*, SymuViaTripNode*> & GetMapNodesToExternalConnexions() const;

    SymuCore::Node * GetNodeFromConnexion(Connexion * pCon) const;
    Connexion * GetConnexionFromNode(SymuCore::Node * pNode, bool bNodeInputConnexion) const;

    SymuCore::Link * GetLinkFromTuyau(Tuyau * pTuyau) const;
    
private:
    
    void BuildPatterns(SymuCore::Graph * pDrivingLayer);

    void BuildPenalties(SymuCore::Graph * pDrivingLayer);

    void BuildPatternsInZones(SymuCore::Graph * pDrivingLayer);

    void BuildPatternsInPlaques(SymuCore::Graph * pDrivingLayer);

    void BuildPenaltiesForZones(SymuCore::Graph * pDrivingLayer);

    void BuildPenaltiesForPlaques(SymuCore::Graph * pDrivingLayer);

private:

    struct CompareLink {
        bool operator()(const SymuCore::Link* l1, const SymuCore::Link* l2) const;
    };

protected:

    SymuCoreGraphBuilder*                                       m_pParent;
    Reseau*                                                     m_pNetwork;
    bool                                                        m_bForSymuMaster;

    std::map<Connexion*, SymuCore::Node*>                       m_mapInternalConnexionsToNode;
    std::map<SymuCore::Node*, Connexion*>                       m_mapNodesToInternalConnexions;
    std::map<SymuViaTripNode*, SymuCore::Node*>                 m_mapExternalConnexionsToNode;
    std::map<SymuCore::Node*, SymuViaTripNode*>                 m_mapNodesToExternalConnexions;
    std::map<SymuCore::Link*, std::set<Tuyau*, LessPtr<Tuyau> >, CompareLink>           m_mapLinkToTuyaux;
    std::map<ZoneDeTerminaison*, SymuCore::Node*, LessConnexionPtr<ZoneDeTerminaison> > m_mapZonesToNode;
    std::map<CPlaque*, SymuCore::Node*, LessConnexionPtr<CPlaque> >                     m_mapPlaquesToNodes;
};

#endif // SymuCoreDrivingGraphBuilder_H


