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

#ifndef SymuCoreGraphBuilder_H
#define SymuCoreGraphBuilder_H

#include "SymuCoreDrivingGraphBuilder.h"
#include "SymuCorePublicTransportGraphBuilder.h"

#include <map>
#include <set>

class Reseau;
class Connexion;
class Tuyau;
class TypeVehicule;
class ZoneDeTerminaison;
class SymuViaTripNode;

namespace SymuCore {
    class Graph;
    class MultiLayersGraph;
    class Link;
    class Node;
    class VehicleType;
    class MacroType;
    class Pattern;
    class DrivingPattern;
    class AbstractPenalty;
    class Populations;
}

namespace boost {
    namespace posix_time {
        class ptime;
        class time_duration;
    }
}

class SymuCoreGraphBuilder
{
public:

    SymuCoreGraphBuilder();

	//! Constructeur
    SymuCoreGraphBuilder(Reseau* pReseau, bool bForSymuMaster);

	//! Destructeur
    virtual ~SymuCoreGraphBuilder();

    virtual bool Build(SymuCore::MultiLayersGraph * pGraph, bool bIsPrimaryGraph);

    const SymuCoreDrivingGraphBuilder & GetDrivingGraphBuilder() const;
    const SymuCorePublicTransportGraphBuilder & GetPublicTransportGraphBuilder() const;

    SymuCore::MultiLayersGraph * GetGraph() const;

protected:

    Reseau*                                 m_pNetwork;

    SymuCore::MultiLayersGraph              *m_pGraph;

    // pour gestion des différences entre la génération du graphe pour SymuMaster et celui pour un usage interne par SymuVia
    bool                                    m_bForSymuMaster;

    SymuCoreDrivingGraphBuilder             m_DrivingGraphBuilder;

    SymuCorePublicTransportGraphBuilder     m_PublicTransportGraphBuilder;
};

#endif // SymuCoreGraphBuilder_H


