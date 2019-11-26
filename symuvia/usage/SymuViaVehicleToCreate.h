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
#ifndef SymuViaVehicleToCreateH
#define SymuViaVehicleToCreateH

#include "VehicleToCreate.h"

#include <boost/shared_ptr.hpp>

#include <vector>

class SymuViaTripNode;
class TypeVehicule;
class Tuyau;
class CPlaque;
class Connexion;

// Définition des paramètres de création arbitraire d'un véhicule pour la flotte SymuVia
class SymuViaVehicleToCreate : public VehicleToCreate
{
public:
    SymuViaVehicleToCreate();
    SymuViaVehicleToCreate(int vehId, AbstractFleet * pFleet);
    virtual ~SymuViaVehicleToCreate();

    void SetOrigin(SymuViaTripNode * pOrigin);
    SymuViaTripNode * GetOrigin();

    void SetPlaqueOrigin(CPlaque * pPlaqueOrigin);
    CPlaque * GetPlaqueOrigin();

    void SetPlaqueDestination(CPlaque * pPlaqueDestination);
    CPlaque * GetPlaqueDestination();

    void SetType(TypeVehicule * pTypeVeh);
    TypeVehicule * GetType();

    void SetNumVoie(int nVoie);
    int GetNumVoie();

    void SetTimeFraction(double dbt);
    double GetTimeFraction();

    void SetDestination(SymuViaTripNode * pDest);
    SymuViaTripNode * GetDestination();

    void SetItinerary(boost::shared_ptr<std::vector<Tuyau*> > pIti);
    boost::shared_ptr<std::vector<Tuyau*> > GetItinerary();

    void SetJunction(Connexion * pJunction);
    Connexion * GetJunction();

    void SetExternalID(int externalID);
    int GetExternalID();

    void SetNextRouteID(const std::string & nextRouteID);
    const std::string & GetNextRouteID();

protected:
    SymuViaTripNode * m_pOrigine;
    CPlaque * m_pPlaqueOrigin;
    CPlaque * m_pPlaqueDestination;

    TypeVehicule * m_pTypeVeh;
    int m_nVoie;
    double m_dbt;
    SymuViaTripNode * m_pDest;
    boost::shared_ptr<std::vector<Tuyau*> > m_pItinerary;
    Connexion * m_pJunction;

    int m_ExternalID;

    std::string m_strNextRouteID;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Sérialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version);
};

#endif // SymuViaVehicleToCreateH