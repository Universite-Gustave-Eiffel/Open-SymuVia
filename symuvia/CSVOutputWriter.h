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
#ifndef csvoutputwriterH
#define csvoutputwriterH

#include "TimeUtil.h"
#pragma warning(disable: 4005)
#include "ogr_spatialref.h"
#pragma warning(default: 4005)
#include <fstream>
#include <deque>
#include <vector>

class Tuyau;
class PonctualSensor;
class Vehicule;
class SimulationSnapshot;

class CSVOutputWriter
{
public:
    CSVOutputWriter();
    CSVOutputWriter(const std::string& path, const std::string& suffix, const SDateTime& dateDebutSimu, OGRCoordinateTransformation *pCoordTransf,
                    bool bTrajectoriesOutput, bool bSensorsOutput);
    virtual ~CSVOutputWriter();

    // passage en mode d'Ã©criture dans fichiers temporaires (affectation dynamique)
    void doUseTempFiles(SimulationSnapshot * pSimulationSnapshot, size_t snapshotIdx);
    // validation de la derniÃ¨er pÃ©riode d'affectation et recopie des fichiers temporaires dans les fichiers finaux
    void converge(SimulationSnapshot * pSimulationSnapshot);

    // Ã©criture du fichier des tronÃ§ons
    void writeLinksFile(const std::deque<Tuyau*> &lstTuyaux);
    // Ã©criture du fichier des capteurs
    void writeSensorsFile(const std::vector<PonctualSensor*> &lstCapteurs);
    // Ã©criture d'une trajectoire
    void writeTrajectory(double dbInstant, Vehicule * pVehicule);
    // Ã©criture des donnÃ©es d'un capteur pour une pÃ©riode
    void writeSensorData(double dbInstant, const std::string& sID, int nNbVehFranchissant, double dbVitGlobal, const std::string& sVit);

private:
    std::string   m_Path;
    std::string   m_Suffix; // Pour gestion des rÃ©plications

    SDateTime     m_StartTime;

    // flux vers le fichier des trajectoires
    std::ofstream m_TrajsOFS;

    // flux vers le fichier des changements de tronÃ§on
    std::ofstream m_LinkChangesOFS;
    
    // flux vers le fichier des sorties capteur
    std::ofstream m_SensorsOFS;

    SimulationSnapshot * m_pCurrentSimulationSnapshot;

	// Transformation des coordonnÃ©es
	OGRCoordinateTransformation *m_pCoordTransf;

    bool m_bTrajectoriesOutput;
    bool m_bSensorsOutput;
};

#endif // csvoutputwriterH
