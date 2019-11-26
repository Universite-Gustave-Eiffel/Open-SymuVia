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
#ifndef SymuCoreManagerH
#define SymuCoreManagerH

#include "ReaderWriterLock.h"

#include <boost/thread/recursive_mutex.hpp>

#include <string>
#include <vector>
#include <map>
#include <deque>

namespace SymuCore {
    class MultiLayersGraph;
    class Node;
    class SubPopulation;
    class Origin;
    class Destination;
    class Pattern;
}

class SymuCoreGraphBuilder;
class SymuViaTripNode;
class Tuyau;
class Connexion;
class Reseau;
class TypeVehicule;
class TripNode;
class PublicTransportLine;
class Arret;

// Structure dÃ©crivant un itinÃ©raire rÃ©sultat
struct PathResult
{
    std::vector<Tuyau*> links;
    double dbCost;
    double dbPenalizedCost;
    double dbCommonalityFactor;
    bool bPredefined;
    std::string strName;
    Connexion* pJunction;
};

class SymuCoreManager
{
public:
    SymuCoreManager(Reseau * pNetwork);
    virtual ~SymuCoreManager();

    void ForceGraphRefresh();

    void UpdateCosts(const std::deque<TypeVehicule*> & lstTypes, bool bIsRecursiveCall);

    void Invalidate();

    void ComputePaths(TripNode* pOrigine, TripNode* pDestination, TypeVehicule * pTypeVeh, double dbInstant, int nbShortestPaths,
        int method, const std::vector<Tuyau*> & refPath, const std::vector<Tuyau*> & linksToAvoid,
        // Sorties
        std::vector<PathResult> & paths,
        std::map<std::vector<Tuyau*>, double> & MapFilteredItis, bool bRecursiveCall);

    void ComputePaths(TripNode* pOrigine, const std::vector<TripNode*> &lstDestinations, TypeVehicule * pTypeVeh, double dbInstant, int nbShortestPaths,
        int method, const std::vector<Tuyau*> & refPath, const std::vector<Tuyau*> & linksToAvoid,
        // Sorties
        std::vector<std::vector<PathResult> > & paths,
        std::vector<std::map<std::vector<Tuyau*>, double> > & MapFilteredItis, bool bRecursiveCall);

    void ComputePaths(const std::vector<TripNode*> lstOrigins, TripNode* pDestination, TypeVehicule * pTypeVeh, double dbInstant, int nbShortestPaths,
        int method, const std::vector<Tuyau*> & refPath, const std::vector<Tuyau*> & linksToAvoid,
        // Sorties
        std::vector<std::vector<PathResult> > & paths,
        std::vector<std::map<std::vector<Tuyau*>, double> > & MapFilteredItis, bool bRecursiveCall);
    
    void ComputePaths(TripNode* pOrigine, TripNode* pDestination, TypeVehicule * pTypeVeh, double dbInstant, int nbShortestPaths,
        // Sorties
        std::vector<PathResult> & paths,
        std::map<std::vector<Tuyau*>, double> & MapFilteredItis, bool bRecursiveCall);

    void ComputePaths(TripNode* pOrigine, const std::vector<TripNode*> lstDestinations, TypeVehicule * pTypeVeh, double dbInstant, int nbShortestPaths,
        // Sorties
        std::vector<std::vector<PathResult> > & paths,
        std::vector<std::map<std::vector<Tuyau*>, double> > & MapFilteredItis, bool bRecursiveCall);

    void ComputePaths(const std::vector<TripNode*> lstOrigins, TripNode* pDestination, TypeVehicule * pTypeVeh, double dbInstant, int nbShortestPaths,
        // Sorties
        std::vector<std::vector<PathResult> > & paths,
        std::vector<std::map<std::vector<Tuyau*>, double> > & MapFilteredItis, bool bRecursiveCall);

    void ComputePaths(TripNode* pOrigine, Connexion* pDestination, TypeVehicule * pTypeVeh, double dbInstant, int nbShortestPaths,
        int method, const std::vector<Tuyau*> & refPath, const std::vector<Tuyau*> & linksToAvoid,
        // Sorties
        std::vector<PathResult> & paths,
        std::map<std::vector<Tuyau*>, double> & MapFilteredItis, bool bRecursiveCall);

    void ComputePaths(const std::vector<TripNode*> lstOrigins, Connexion* pDestination, TypeVehicule * pTypeVeh, double dbInstant, int nbShortestPaths,
        int method, const std::vector<Tuyau*> & refPath, const std::vector<Tuyau*> & linksToAvoid,
        // Sorties
        std::vector<std::vector<PathResult> > & paths,
        std::vector<std::map<std::vector<Tuyau*>, double> > & MapFilteredItis, bool bRecursiveCall);

    void ComputePaths(TripNode* pOrigine, Connexion* pDestination, TypeVehicule * pTypeVeh, double dbInstant, int nbShortestPaths,
        // Sorties
        std::vector<PathResult> & paths,
        std::map<std::vector<Tuyau*>, double> & MapFilteredItis, bool bRecursiveCall);

    void ComputePaths(const std::vector<TripNode*> lstOrigins, Connexion* pDestination, TypeVehicule * pTypeVeh, double dbInstant, int nbShortestPaths,
        // Sorties
        std::vector<std::vector<PathResult> > & paths,
        std::vector<std::map<std::vector<Tuyau*>, double> > & MapFilteredItis, bool bRecursiveCall);

    void ComputePaths(Connexion* pOrigine, TripNode* pDestination, TypeVehicule * pTypeVeh, double dbInstant, int nbShortestPaths, Tuyau * pTuyauAmont,
        // Sorties
        std::vector<PathResult> & paths,
        std::map<std::vector<Tuyau*>, double> & MapFilteredItis, bool bRecursiveCall);

    void ComputePaths(Connexion* pOrigine, const std::vector<TripNode*> & lstDestinations, TypeVehicule * pTypeVeh, double dbInstant, int nbShortestPaths, Tuyau * pTuyauAmont,
        // Sorties
        std::vector<std::vector<PathResult> > & paths,
        std::vector<std::map<std::vector<Tuyau*>, double> > & MapFilteredItis, bool bRecursiveCall);

    void ComputePaths(const std::vector<std::pair<Connexion*, Tuyau*> > & lstOrigines, TripNode* pDestination, TypeVehicule * pTypeVeh, double dbInstant, int nbShortestPaths,
        // Sorties
        std::vector<std::vector<PathResult> > & paths,
        std::vector<std::map<std::vector<Tuyau*>, double> > & MapFilteredItis, bool bRecursiveCall);

    void ComputePaths(Connexion* pOrigine, TripNode* pDestination, TypeVehicule * pTypeVeh, double dbInstant, int nbShortestPaths, Tuyau * pTuyauAmont,
        int method, const std::vector<Tuyau*> & refPath, const std::vector<Tuyau*> & linksToAvoid,
        // Sorties
        std::vector<PathResult> & paths,
        std::map<std::vector<Tuyau*>, double> & MapFilteredItis, bool bRecursiveCall);

	void ComputePaths(Connexion* pOrigine, Connexion* pDestination, TypeVehicule * pTypeVeh, double dbInstant, int nbShortestPaths, Tuyau * pTuyauAmont,
		int method, const std::vector<Tuyau*> & refPath, const std::vector<Tuyau*> & linksToAvoid,
		// Sorties
		std::vector<PathResult> & paths,
		std::map<std::vector<Tuyau*>, double> & MapFilteredItis, bool bRecursiveCall);

    void ComputePaths(Connexion* pOrigine, const std::vector<TripNode*> & lstDestinations, TypeVehicule * pTypeVeh, double dbInstant, int nbShortestPaths, Tuyau * pTuyauAmont,
        int method, const std::vector<Tuyau*> & refPath, const std::vector<Tuyau*> & linksToAvoid,
        // Sorties
        std::vector<std::vector<PathResult> > & paths,
        std::vector<std::map<std::vector<Tuyau*>, double> > & MapFilteredItis, bool bRecursiveCall);

    double ComputeCost(TypeVehicule* pTypeVehicule, const std::vector<Tuyau*> & path, bool bIsRecursiveCall);

    //////////////////////////////////////////////////////////////////////////////////////
    // MÃ©thodes spÃ©cifiques aux calculs multimodaux (pour le simulation game par ex)
    //////////////////////////////////////////////////////////////////////////////////////
    void ComputePathsPublicTransport(Connexion* pOrigin, TripNode* pDestination, double dbInstant, int nbShortestPaths,
        // Sorties
        std::vector<std::pair<double, std::vector<std::string> > > & paths);

    void ComputePathsPublicTransport(Connexion* pOrigin, const std::vector<TripNode*> & lstDestinations, double dbInstant, int nbShortestPaths,
        // Sorties
        std::vector<std::vector<std::pair<double, std::vector<std::string> > > > & paths);

    void ComputePathsPublicTransport(Arret* pOrigin, PublicTransportLine* previousLine, PublicTransportLine* pNextLine, TripNode* pDEstination, double dbInstant, int nbShortestPaths,
        // Sorties
        std::vector<std::pair<double, std::vector<std::string> > > & paths);

private:

    void ClearSubPopulations();

    bool GraphExists();
    void Initialize();

    void CheckGraph(bool bIsRecursiveCall);

    void ComputePaths(const std::vector<SymuCore::Origin> & lstOrigins, const std::vector<SymuCore::Destination> & lstDestinations, TypeVehicule * pTypeVeh, double dbInstant, int nbShortestPaths,
        int method, const std::vector<Tuyau*> & refPath, const std::vector<Tuyau*> & linksToAvoid,
        // Sorties
        std::vector<std::vector<PathResult> > & paths,
        std::vector<std::map<std::vector<Tuyau*>, double> > & MapFilteredItis);

    void ComputePathsPublicTransport(SymuCore::Origin origin, const std::vector<SymuCore::Destination> & lstDestinations, const std::vector<SymuCore::Pattern*> & patternsToAvoid, double dbInstant, int nbShortestPaths,
        // Sorties
        std::vector<std::vector<std::pair<double, std::vector<std::string> > > > & paths);

    SymuCore::Node* GetNodeFromOrigin(TripNode * pOrigin);
    SymuCore::Node* GetNodeFromDestination(TripNode * pDest);

    void FillOriginFromTripNode(TripNode * pOrigin, SymuCore::Origin & origin);
    void FillDestinationFromTripNode(TripNode * pDest, SymuCore::Destination & destination);

private:

    // on gÃ¨re un graphe par type de vÃ©hicule (SymuScript ne sait pas gÃ©rer diffÃ©rents types de vÃ©hicules)
    SymuCore::MultiLayersGraph * m_pGraph;

    // flag indiquant si le rÃ©seau d'affectation a changÃ© et doit Ãªtre reconstruit si nÃ©cessaire
    bool m_bInvalid;

    // Objet rÃ©seau associÃ©
    Reseau *m_pNetwork;

    // Classe utilitaire pour la construction d'un graphe SymuCore
    SymuCoreGraphBuilder * m_pGraphBuilder;

    // Association entre types de vÃ©hicules et sous populations SymuCore
    std::map<TypeVehicule *, SymuCore::SubPopulation*> m_mapSubPopulations;

    // Sous population spÃ©cifique pour les demandes d'itinÃ©raires en transports en commun uniquement
    SymuCore::SubPopulation * m_pPublicTransportOnlySubPopulation;

    // Association entre les tuyaux et les Patterns SymuCore pour accÃ¨s rapide
    std::map<Tuyau*, SymuCore::Pattern*> m_MapTuyauxToPatterns;

    // Association entre les Nodes et les SymuViaTripNodes pour accÃ¨s rapide
    std::map<SymuCore::Node*, SymuViaTripNode*> m_MapSymuViaTripNodeByNode;

    // Objets de synchro pour interdire les opÃ©rations parallÃ¨les incompatibles
    ReaderWriterLock m_Lock;
    boost::recursive_mutex m_Mutex;
    
};

#endif // SymuCoreManagerH