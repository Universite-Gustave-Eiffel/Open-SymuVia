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
#ifndef DeliveryFleetH
#define DeliveryFleetH

#include "../AbstractFleet.h"

class PointDeLivraison;

class DeliveryFleet : public AbstractFleet
{
public:
    DeliveryFleet();
    DeliveryFleet(Reseau * pNetwork);
    virtual ~DeliveryFleet();

    // Instanciation de l'objet spécifique à la flotte contenant les paramètres d'un véhicule liés à celle-ci
    virtual AbstractFleetParameters * CreateFleetParameters();

    virtual bool Load(XERCES_CPP_NAMESPACE::DOMNode * pXMLNetwork, Logger & loadingLogger);

    // Effectue les opérations à faire à l'activation d'un véhicule
    virtual void OnVehicleActivated(boost::shared_ptr<Vehicule> pVeh, double dbInstant);

    // Calcule la durée d'arrêt à un tripnode.
    virtual double GetStopDuration(boost::shared_ptr<Vehicule> pVehicle, TripNode * pTripNode, bool bIsRealStop);

    // Renvoie une map d'attributs à sortir de façon spécifique pour un véhicule dans le TripNode
    virtual std::map<std::string, std::string> GetOutputAttributes(Vehicule * pV, TripNode * pTripNode);

    // Indique s'il faut sortir la charge courante du véhicule ou non
    virtual bool OutputVehicleLoad(Vehicule * pVehicle) {return true;}

    // Ajoute un point de livraison à une tournée
    bool AddDeliveryPointToTrip(Trip * pTrip, PointDeLivraison * pTripNode, int positionIndex, Vehicule * pVehicle, int dechargement, int chargement);

    // Supprime un point de livraison d'une tournée
    bool RemoveDeliveryPointFromTrip(Trip * pTrip, int positionIndex, Vehicule * pVehicle);

    // Construit le libellé du véhicule, utilisé par exemple pour les bus
    virtual std::string GetVehicleIdentifier(boost::shared_ptr<Vehicule> pVeh);

protected:

    // Fonction utilitaire pour récupérer la valeur qui nous intéresse dans les maps membres de DeliveryFleet
    std::pair<int,int> GetLoadFromMap(const std::map<TripNode*, std::vector<std::pair<int,int> > > & mapLoads, Trip * pTrip, TripNode * pTripNode);

    // Fonction utilitaire pour déterminer l'index d'un tripleg à partir de l'index d'un point de livraison
    size_t GetTripLegIndexFromDeliveryPointIndex(Trip * pTrip, int positionIndex);

protected:

    // Quantités à charger et décharger par tournée et par point de livraison (il peut y en avoir plusieurs par point de livraison s'il apparaît plusieurs fois dans une tournée)
    std::map<Trip*, std::map<TripNode*, std::vector<std::pair<int,int> > > > m_MapLoads;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Sérialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version);
};


#endif // DeliveryFleetH