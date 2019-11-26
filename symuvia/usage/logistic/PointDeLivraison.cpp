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

#include "stdafx.h"
#include "PointDeLivraison.h"

PointDeLivraison::PointDeLivraison()
    : TripNode()
{
    // Les véhicules doivent s'arrêter au point de livraison.
    m_bStopNeeded = true;
    m_bListVehiclesInTripNode = true;

    m_nNbPlaces = INT_MAX;
}

PointDeLivraison::PointDeLivraison(const std::string & strID, Reseau * pNetwork)
    : TripNode(strID, pNetwork)
{
    // Les véhicules doivent s'arrêter au point de livraison.
    m_bStopNeeded = true;
    m_bListVehiclesInTripNode = true;

    m_nNbPlaces = INT_MAX;
}

PointDeLivraison::~PointDeLivraison()
{
}

void PointDeLivraison::SetNbPlaces(int nbPlaces)
{
    m_nNbPlaces = nbPlaces;
}

void PointDeLivraison::VehiculeEnter(boost::shared_ptr<Vehicule> pVehicle, VoieMicro * pDestinationEnterLane, double dbInstDestinationReached, double dbInstant, double dbTimeStep, bool bForcedOutside)
{
    // Si le point de livraison est complet, on laisse le véhicule immobile, sinon on le traite normalement
    if((int)m_LstStoppedVehicles.size() < m_nNbPlaces)
    {
        // Appel de la méthode de la classe mère
        TripNode::VehiculeEnter(pVehicle, pDestinationEnterLane, dbInstDestinationReached, dbInstant, dbTimeStep, bForcedOutside);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Sérialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template void PointDeLivraison::serialize(boost::archive::xml_woarchive & ar, const unsigned int version);
template void PointDeLivraison::serialize(boost::archive::xml_wiarchive & ar, const unsigned int version);

template<class Archive>
void PointDeLivraison::serialize(Archive & ar, const unsigned int version)
{
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(TripNode);

    ar & BOOST_SERIALIZATION_NVP(m_nNbPlaces);
}