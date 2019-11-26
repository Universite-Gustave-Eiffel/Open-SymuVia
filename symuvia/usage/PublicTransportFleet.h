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
#ifndef PublicTransportFleetH
#define PublicTransportFleetH

#include "AbstractFleet.h"

class TypeVehicule;

class PublicTransportFleet : public AbstractFleet
{
public:
    PublicTransportFleet();
    PublicTransportFleet(Reseau * pNetwork);
    virtual ~PublicTransportFleet();

    // Instanciation de l'objet spécifique à la flotte contenant les paramètres d'un véhicule liés à celle-ci
    virtual AbstractFleetParameters * CreateFleetParameters();

    // récupération du type de véhicule associé à une ligne de bus
    TypeVehicule * GetTypeVehicule(Trip * pLine);

    virtual bool Load(XERCES_CPP_NAMESPACE::DOMNode * pXMLNetwork, Logger & loadingLogger);

    // Indique s'il faut sortir la charge courante du véhicule ou non
    virtual bool OutputVehicleLoad(Vehicule * pVehicle);

    // Construit le libellé du véhicule, utilisé par exemple pour les bus
    virtual std::string GetVehicleIdentifier(boost::shared_ptr<Vehicule> pVeh);

protected:
    // Paramétres des véhicules liés à un élément de calendrier
    std::map<Trip*, std::map<std::string, ScheduleParameters*> > m_MapScheduleParametersByTrip;

    // Indique si la durée des arrêts est dynamique ou non pour chaque ligne de bus
    std::map<Trip*, bool> m_MapDureeArretsDynamique;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Sérialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version);
};


#endif // PublicTransportFleetH