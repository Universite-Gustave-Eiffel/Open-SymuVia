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
#ifndef EdieSensorH
#define EdieSensorH

#include "LongitudinalSensor.h"

#include <vector>
#include <set>
#include <map>

class MFDSensor;
class TypeVehicule;

/*===========================================================================================*/
/* Structure EdieSensorData                                                                 */
/*===========================================================================================*/
class EdieSensorData : public AbstractSensorSnapshot
{
public:
        std::vector<std::map<TypeVehicule*,double> > dbTotalTravelledDistance;		// Distance totale parcourue pour chaque voie et pour chaque type de vÃ©hicule
        std::vector<std::map<TypeVehicule*, double> > dbTotalTravelledTime;			// Temps total passÃ© pour chaque voie et pour chaque type de vÃ©hicule
		std::map<TypeVehicule*, int> nNumberOfRemovalVehicles;					// Nombre de vÃ©hicules supprimÃ© pour chaque type de vÃ©hicule 

        double GetTotalTravelledDistance() const;
        double GetTotalTravelledTime() const;
        double GetTravelledTimeForLane(size_t k) const;
        double GetTravelledDistanceForLane(size_t k) const;

		int	GetTotalNumberOfRemovalVehicles() const;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SÃ©rialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive& ar, const unsigned int version);
};


class EdieSensor : public LongitudinalSensor
{
public:
    enum ETuyauType
    {
        TT_Link = 1, // toncon classique
        TT_Aval = 2  // tronÃ§on interne Ã  la brique aval d'un tronÃ§on classique, constituant une partie de mouvement dont le tronÃ§on classique est le tronÃ§on amont
    };

public:
    EdieSensor();
    EdieSensor(const std::string & strNom, Tuyau * pTuyau, double startPos, double endPos, int tuyauType, MFDSensor * pParent);
    virtual ~EdieSensor();

    virtual void CalculInfoCapteur(Reseau * pNetwork, double dbInstant, bool bNewPeriod, double dbInstNewPeriode, boost::shared_ptr<Vehicule> pVeh);

    virtual void AddMesoVehicle(double dbInstant, Vehicule * pVeh, Tuyau * pLink, Tuyau * pDownstreamLink, double dbLengthInLink = DBL_MAX);

    virtual void WriteDef(DocTrafic* pDocTrafic);
    virtual void Write(double dbInstant, Reseau * pReseau, double dbPeriodAgregation, double dbDebut, double dbFin, const std::deque<TraceDocTrafic* > & docTrafics, CSVOutputWriter * pCSVOutput);

    virtual void PrepareNextPeriod();

    EdieSensorData & GetData();
    int GetTuyauType();

    virtual AbstractSensorSnapshot * TakeSnapshot();
    virtual void Restore(Reseau * pNetwork, AbstractSensorSnapshot * backup);

protected:

    int     eTuyauType;         // indique s'il s'agit d'un bout de capteur MFD sur un tronÃ§on interne ou un troncon intern aval
    MFDSensor *m_pParentMFD;    // Capteur MFD parent Ã©ventuel

	// Informations dynamiques du capteur agrÃ©gÃ©es sur une pÃ©riode finie (non glissante)
	EdieSensorData data;		

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SÃ©rialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive& ar, const unsigned int version);
};

#endif // EdieSensorH