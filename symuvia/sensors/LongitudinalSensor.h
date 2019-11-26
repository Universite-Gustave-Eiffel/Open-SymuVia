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
#ifndef LongitudinalSensorH
#define LongitudinalSensorH

#include "AbstractSensor.h"

#include <map>

class Tuyau;

/*===========================================================================================*/
/* Structure DataCapteurLongitudinal                                                         */
/*===========================================================================================*/
class LongitudinalSensorData : public AbstractSensorSnapshot
{
public:
    std::map<std::pair<int, int>, int>   mapNbChgtVoie;  // Map du nombre de véhicule ayant changé de voie dans la zone définie par le capteur
                                                         // détaillé couple de voie par couple de voie

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Sérialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive& ar, const unsigned int version);
};

class LongitudinalSensor : public AbstractSensor
{
public:
    LongitudinalSensor();
    LongitudinalSensor(const std::string & strName, Tuyau * pTuyau, double dbStartPos, double dbEndPos);
    virtual ~LongitudinalSensor();

    virtual std::string GetSensorPeriodXMLNodeName() const;

    void InitData();

    void AddChgtVoie(Tuyau * pTuyau, double dbPosition, int nVoieOrigine, int nVoieDestination);

    virtual void CalculInfoCapteur(Reseau * pNetwork, double dbInstant, bool bNewPeriod, double dbInstNewPeriode, boost::shared_ptr<Vehicule> pVeh);

    virtual void WriteDef(DocTrafic* pDocTrafic);
    virtual void Write(double dbInstant, Reseau * pReseau, double dbPeriodAgregation, double dbDebut, double dbFin, const std::deque<TraceDocTrafic* > & docTrafics, CSVOutputWriter * pCSVOutput);

    virtual void PrepareNextPeriod();

    double GetSensorLength();

    Tuyau * GetTuyau();
    double GetStartPosition();
    double GetEndPosition();

    virtual AbstractSensorSnapshot * TakeSnapshot();
    virtual void Restore(Reseau * pNetwork, AbstractSensorSnapshot * backup);

protected:

    Tuyau*      m_pTuyau;
    double      dbPositionDebut;    // Positionnement en m du début du capteur (à partir du début du tuyau
    double      dbPositionFin;      // Positionnement en m de la fin du capteur (à partir du début du tuyau)

	// Informations dynamiques du capteur agrégées sur une période finie (non glissante)
	LongitudinalSensorData	data;		

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Sérialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive& ar, const unsigned int version);
};

#endif // LongitudinalSensorH