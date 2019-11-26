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
#ifndef TankSensorH
#define TankSensorH

#include "AbstractSensor.h"

#include <vector>
#include <set>
#include <map>


class Tuyau;
class SensorsManager;

class TankSensorData
{
public:
    int idVeh;
    double dbEntryTime;
    double dbExitTime;
    double dbTraveleldDistance;
    bool bCreatedInZone;
    bool bDestroyedInZone;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SÃ©rialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive& ar, const unsigned int version);
};

class TankSensorSnapshot : public AbstractSensorSnapshot
{
public:
    std::map<int, std::pair<std::pair<double, double>, bool> > vehiclesInsideZone;
    std::vector<TankSensorData> results;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SÃ©rialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive& ar, const unsigned int version);
};

class TankSensor : public AbstractSensor
{
public:
    TankSensor();
    TankSensor(Reseau * pNetwork,
        SensorsManager * pParentSensorsManager,
        const char* strNom,
        const std::vector<Tuyau*> & Tuyaux);
    virtual ~TankSensor();

    virtual std::string GetSensorPeriodXMLNodeName() const;

    virtual void CalculInfoCapteur(Reseau * pNetwork, double dbInstant, bool bNewPeriod, double dbInstNewPeriode, boost::shared_ptr<Vehicule> pVeh);

    virtual void AddMesoVehicle(double dbInstant, Vehicule * pVeh, Tuyau * pLink, Tuyau * pDownstreamLink, double dbLengthInLink = DBL_MAX);

    virtual void WriteDef(DocTrafic* pDocTrafic);
    virtual void Write(double dbInstant, Reseau * pReseau, double dbPeriodAgregation, double dbDebut, double dbFin, const std::deque<TraceDocTrafic* > & docTrafics, CSVOutputWriter * pCSVOutput);

    virtual void PrepareNextPeriod();

    virtual void VehicleDestroyed(Vehicule * pVeh);


    virtual AbstractSensorSnapshot * TakeSnapshot();
    virtual void Restore(Reseau * pNetwork, AbstractSensorSnapshot * backup);

protected:
    std::set<Tuyau*> m_Tuyaux;

    // VÃ©hicule dans la zone avec l'instant d'entrÃ©e et la distance parcourue au moment de l'entrÃ©e
    std::map<Vehicule*, std::pair<std::pair<double,double>, bool> > m_vehiclesInsideZone;

    // rÃ©sultats finaux
    std::vector<TankSensorData> m_results;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SÃ©rialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive& ar, const unsigned int version);
};

#endif // TankSensorH