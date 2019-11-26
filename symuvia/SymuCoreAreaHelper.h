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

#ifndef SymuCoreAreaHelper_H
#define SymuCoreAreaHelper_H

namespace SymuCore {
    class MacroType;
}

#include "tools.h"

#include <vector>
#include <map>
#include <set>

class Connexion;
class Tuyau;
class ZoneDeTerminaison;
class TypeVehicule;
class Reseau;

class SymuCoreAreaHelper
{
public:

    std::map<Tuyau*, std::pair<double, double> > &  GetLstLinks();

    void ComputeIncomingPaths(ZoneDeTerminaison * pZone, Connexion* pFromJunction, const std::vector<Tuyau*> & lstJunctionLinks, const std::vector<SymuCore::MacroType*> & listMacroTypes);
    void ComputeOutgoingPaths(ZoneDeTerminaison * pZone, Connexion* pToJunction, const std::vector<Tuyau*> & lstJunctionLinks, const std::vector<SymuCore::MacroType*> & listMacroTypes);

    bool isAcessibleFromConnexion(Connexion* pFromJunction) const;
    bool isAcessibleToConnexion(Connexion* pToJunction) const;

    bool isAcessibleFromConnexion(Connexion* pFromJunction, Tuyau * pUpstreamLink, bool bForSymuMaster, std::set<TypeVehicule*> & lstAllowedVehicleTypes) const;
    bool isAcessibleToConnexion(Connexion* pToJunction, Tuyau * pDownstreamLink, bool bForSymuMaster, std::set<TypeVehicule*> & lstAllowedVehicleTypes) const;

    bool isAcessibleFromConnexionWithDownstreamLink(Connexion* pFromJunction, Tuyau * pDownstreamLink) const;
    bool isAcessibleToConnexionWithUpstreamLink(Connexion* pToJunction, Tuyau * pUpstreamLink) const;

    bool isAccessibleFromArea(Connexion* pFromJunction, SymuCoreAreaHelper * pFromArea, bool bForSymuMaster, std::set<TypeVehicule*> & lstAllowedVehicleTypes) const;
    bool isAccessibleToArea(Connexion* pToJunction, SymuCoreAreaHelper * pToArea, bool bForSymuMaster, std::set<TypeVehicule*> & lstAllowedVehicleTypes) const;
    
    
    const std::map<Connexion*, std::map<Tuyau*, std::map<Tuyau*, std::map<SymuCore::MacroType*, std::pair<std::vector<Tuyau*>, double> > > > > & GetPrecomputedPaths(bool bZoneOrigine);

    double GetTravelTime(bool bIsOrigin, Connexion * pJunction, SymuCore::MacroType * pMacroType, ZoneDeTerminaison * pZone, double & dbMarginal, double & dbNbOfVehiclesForAllMacroTypes, double & dbTravelTimeForAllMacroTypes);

    void CalculTempsParcoursDepart(Reseau * pNetwork, ZoneDeTerminaison * pZone, double dbInstFinPeriode, SymuCore::MacroType * pMacroType, double dbPeriodDuration);
    void CalculTempsParcoursArrivee(Reseau * pNetwork, ZoneDeTerminaison * pZone, double dbInstFinPeriode, SymuCore::MacroType * pMacroType, double dbPeriodDuration);

    std::map<Connexion*, std::map<int, std::pair< TypeVehicule*, std::pair<std::pair<double, double>, double> > > > & GetMapDepartingVehicles();
    std::map<Connexion*, std::map<int, std::pair< TypeVehicule*, std::pair<std::pair<double, double>, double> > > > & GetMapArrivingVehicles();

    std::map<Connexion*, std::map<SymuCore::MacroType*, double > > & GetMapDepartingTT();
    std::map<Connexion*, std::map<SymuCore::MacroType*, double > > & GetMapDepartingMarginals();

    std::map<Connexion*, std::map<SymuCore::MacroType*, double > > & GetMapArrivingTT();
    std::map<Connexion*, std::map<SymuCore::MacroType*, double > > & GetMapArrivingMarginals();

    std::map<Connexion*, std::map<SymuCore::MacroType*, double> > & GetMapDepartingMeanNbVehicles();
    std::map<Connexion*, std::map<SymuCore::MacroType*, double> > & GetMapArrivingMeanNbVehicles();

    std::map<Connexion*, double> & GetMapDepartingTTForAllMacroTypes();
    std::map<Connexion*, double> & GetMapArrivingTTForAllMacroTypes();

    bool HasPath(Connexion * pJunction, SymuCore::MacroType * pMacroType, bool bIsOrigin);

private:

    void CalculTempsParcours(Reseau * pNetwork, ZoneDeTerminaison * pZone, double dbInstFinPeriode, SymuCore::MacroType * pMacroType, double dbPeriodDuration, bool bDeparting);

    double GetEmptyTravelTime(Connexion * pJunction, bool bIsOrigin, SymuCore::MacroType * pMacroType);

    double GetMeanLengthForPaths(Connexion * pJunction, SymuCore::MacroType * pMacroType, bool bIsOrigin);

    double GetMeanVehicleNumber(Connexion * pJunction, SymuCore::MacroType * pMacroType, bool bIsOrigin, double dbPeriodDuration);

    double ComputeSpatialTravelTime(double dbMeanSpeed, double dbEmptyMeanSpeed, double dbMeanLengthForPaths, double dbTotalMeanConcentration, bool bIsForbidden);

private:

    // Ensemble des tuyaux de l'aire (zone ou plaque), avec en valeur la position moyenne des départs sur le tuyau et la position moyenne des arrivées sur le tuyau
    std::map<Tuyau*, std::pair<double, double> > m_LstLinks;

    // Pour SymuMaster, on stocke les itinéraires possibles (avec leur longueur) à l'intérieur de la zone du (ou jusqu'au) point de jonction pour chaque point de jonction, chaque tuyau extérieur de la zone, chaque tuyau  de la zone, chaque tuyau de la zone et chaque macro-type:
    std::map<Connexion*, std::map<Tuyau*, std::map<Tuyau*, std::map<SymuCore::MacroType*, std::pair<std::vector<Tuyau*>, double> > > > > m_IncomingPaths; // Itinéraires du point de jonction au tuyau (de destination) de la zone
    std::map<Connexion*, std::map<Tuyau*, std::map<Tuyau*, std::map<SymuCore::MacroType*, std::pair<std::vector<Tuyau*>, double> > > > > m_OutgoingPaths; // Itinéraire du tuyau (d'origine) de la zone au point de jonction

    // Pour le suivi des temps de parcours des véhicules en zone
    std::map<Connexion*, std::map<int, std::pair< TypeVehicule*, std::pair<std::pair<double, double>, double> > > > m_mapDepartingVeh;
    std::map<Connexion*, std::map<int, std::pair< TypeVehicule*, std::pair<std::pair<double, double>, double> > > > m_mapArrivingVeh;

    // Stockage du résultat du calcul de temps de parcours réaliser moyen à utiliser pour l'affectation SymuMaster
    std::map<Connexion*, std::map<SymuCore::MacroType*, double > > m_LstDepartingTravelTimes;
    std::map<Connexion*, std::map<SymuCore::MacroType*, double > > m_LstDepartingMarginals;

    std::map<Connexion*, std::map<SymuCore::MacroType*, double > > m_LstArrivingTravelTimes;
    std::map<Connexion*, std::map<SymuCore::MacroType*, double > > m_LstArrivingMarginals;

    std::map<Connexion*, std::map<SymuCore::MacroType*, double> >  m_dbDepartingMeanNbVehicles;
    std::map<Connexion*, std::map<SymuCore::MacroType*, double> >  m_dbArrivingMeanNbVehicles;

    std::map<Connexion*, double>  m_dbDepartingTTForAllMacroTypes;
    std::map<Connexion*, double>  m_dbArrivingTTForAllMacroTypes;
};

#endif // SymuCoreAreaHelper_H


