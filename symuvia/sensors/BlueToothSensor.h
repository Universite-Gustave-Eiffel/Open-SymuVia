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
#ifndef BlueToothSensorH
#define BlueToothSensorH

#include "AbstractSensor.h"

#include <vector>
#include <map>

class PonctualSensor;
class PonctualSensorSnapshot;
class Connexion;
class Tuyau;
class SensorsManager;

struct BlueToothVehSensorData
{
    BlueToothVehSensorData();
    int nbVehs; // nombre de véhicules détectés pour un mouvement
    std::vector<int> nIDs;  // Identifiants des véhicules pris en compte pour le mouvement
    std::vector<double> dbInstEntrees; // Instants d'entrée des véhicules pris en compte pour le mouvement
    std::vector<double> dbInstSorties; // Instants de sortie des véhicules pris en compte pour le mouvement

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Sérialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive& ar, const unsigned int version);
};

/*===========================================================================================*/
/* Structure BlueToothSensorData                                                                 */
/*===========================================================================================*/
struct BlueToothSensorData
{
        std::map<std::string, std::map<std::string, BlueToothVehSensorData> > mapCoeffsDir; // nombre de véhicule détectés pour chaque mouvement

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Sérialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive& ar, const unsigned int version);
};


class BlueToothSensorSnapshot : public AbstractSensorSnapshot
{
public:

    virtual ~BlueToothSensorSnapshot();

	BlueToothSensorData data;  // Données du capteur bluetooth

    std::vector<PonctualSensorSnapshot*> lstCapteursAmontSnapshots;
    std::vector<PonctualSensorSnapshot*> lstCapteursAvalSnapshots;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Sérialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version);
};


class BlueToothSensor : public AbstractSensor
{
public:
    BlueToothSensor();
    BlueToothSensor(SensorsManager * pParentSensorsManager, const std::string & strNom, Connexion *pNode, const std::map<Tuyau*, double> &mapPositions);
    virtual ~BlueToothSensor();

    virtual std::string GetSensorPeriodXMLNodeName() const;

    virtual void CalculInfoCapteur(Reseau * pNetwork, double dbInstant, bool bNewPeriod, double dbInstNewPeriode, boost::shared_ptr<Vehicule> pVeh);

    virtual void AddMesoVehicle(double dbInstant, Vehicule * pVeh, Tuyau * pLink, Tuyau * pDownstreamLink, double dbLengthInLink = DBL_MAX);

    virtual void WriteDef(DocTrafic* pDocTrafic);
    virtual void Write(double dbInstant, Reseau * pReseau, double dbPeriodAgregation, double dbDebut, double dbFin, const std::deque<TraceDocTrafic* > & docTrafics, CSVOutputWriter * pCSVOutput);

    virtual void PrepareNextPeriod();
    
    virtual AbstractSensorSnapshot * TakeSnapshot();
    virtual void Restore(Reseau * pNetwork, AbstractSensorSnapshot * backup);

protected:
    Connexion*                   m_pNode;
    std::vector<PonctualSensor*> lstCapteursAmont;
    std::vector<PonctualSensor*> lstCapteursAval;

	// Informations dynamiques du capteur agrégées sur une période finie (non glissante)
	BlueToothSensorData    data;		

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Sérialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive& ar, const unsigned int version);
};

#endif // BlueToothSensorH