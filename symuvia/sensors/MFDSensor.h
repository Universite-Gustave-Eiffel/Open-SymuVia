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
#ifndef MFDSensorH
#define MFDSensorH

#include "AbstractSensor.h"

#include <vector>

class EdieSensorData;
class EdieSensor;
class PonctualSensor;
class PonctualSensorSnapshot;
class Tuyau;
class SensorsManager;

class MFDSensorSnapshot : public AbstractSensorSnapshot
{
public:
    std::vector<EdieSensorData> edieData;
    std::vector<PonctualSensorSnapshot> ponctualData;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Sérialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive& ar, const unsigned int version);
};

class MFDSensor : public AbstractSensor
{
public:
    MFDSensor();
    MFDSensor(Reseau * pNetwork,
        SensorsManager * pParentSensorsManager,
        double dbPeriodeAgregation,
        const char* strNom,
        const std::vector<Tuyau*> & Tuyaux,
        const std::vector<double> & dbPosDebut,
        const std::vector<double> & dbPosFin,
        const std::vector<int> & eTuyauType,
        bool bIsMFD,
        bool bIsSimpleMFD);
    virtual ~MFDSensor();

    virtual std::string GetSensorPeriodXMLNodeName() const;

    virtual void CalculInfoCapteur(Reseau * pNetwork, double dbInstant, bool bNewPeriod, double dbInstNewPeriode, boost::shared_ptr<Vehicule> pVeh);

    virtual void AddMesoVehicle(double dbInstant, Vehicule * pVeh, Tuyau * pLink, Tuyau * pDownstreamLink, double dbLengthInLink = DBL_MAX);

    virtual void WriteDef(DocTrafic* pDocTrafic);
    virtual void Write(double dbInstant, Reseau * pReseau, double dbPeriodAgregation, double dbDebut, double dbFin, const std::deque<TraceDocTrafic* > & docTrafics, CSVOutputWriter * pCSVOutput);

    virtual void PrepareNextPeriod();

    const std::vector<EdieSensor*> &GetLstSensors();
    bool IsEdie();

    virtual AbstractSensorSnapshot * TakeSnapshot();
    virtual void Restore(Reseau * pNetwork, AbstractSensorSnapshot * backup);

protected:
    std::vector<EdieSensor*> lstSensors;
    std::vector<PonctualSensor*> lstPonctualSensors;
    std::vector<PonctualSensor*> lstNonStrictPonctualSensors;
    std::vector<PonctualSensor*> lstStrictPonctualSensors;
    bool m_bEnableStrictSensors;

    // flag indiquant si ce capteur MFD a été défini par l'utilisateur comme un capteur d'Edie 
    bool                     m_bIsEdie;

    // Longueurs géométriques calculées une fois pour toutes
    double                   m_bGeomLength1;
    double                   m_bGeomLength2;
    double                   m_bGeomLength3;

    // true si capteur MFD simple (on veut uniquement le temps passé et distance parcourue pour calcul vitesse moyenne spatialisée pour SymuMaster, sans créer de capteurs ponctuels par exemple)
    bool                     m_bSimpleMFD;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Sérialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive& ar, const unsigned int version);
};

#endif // MFDSensorH