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
#ifndef PublicTransportLineH
#define PublicTransportLineH

#include "Trip.h"

#include <map>

class Arret;

class PublicTransportLineSnapshot : public TripSnapshot {

public:
    PublicTransportLineSnapshot();
    virtual ~PublicTransportLineSnapshot();

    std::map<std::pair<Arret*, Arret*>, double> latestDrivingTime;
    std::map<Arret*, double> latestStopDuration;
    std::map<Arret*, double> latestFrequencyAtStop;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Sérialisation
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version);
};

class PublicTransportLine : public Trip
{
public:
    PublicTransportLine();
    virtual ~PublicTransportLine();

    void ComputeCosts();

    bool GetLastDrivingTimeBewteenStops(Arret * pUpstreamStop, Arret * pDownstreamStop, double & dbLastDrivingTime);
    double GetLastStopDuration(Arret * pStop);
    double GetLastFrequency(Arret * pStop);

    // méthode pour la sauvegarde et restitution de l'état des lignes de transport public (affectation dynamique convergente).
    virtual TripSnapshot * TakeSnapshot();
    virtual void Restore(Reseau * pNetwork, TripSnapshot * backup);

protected:

    std::map<std::pair<Arret*, Arret*>, double> m_LatestDrivingTime;
    std::map<Arret*, double> m_LatestStopDuration;
    std::map<Arret*, double> m_LatestFrequencyAtStop;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Sérialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version);
};


#endif // PublicTransportLineH