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
#ifndef traveltimsoutputmanagerH
#define traveltimsoutputmanagerH

#pragma warning(disable : 4003)
#include <rapidjson/document.h>
#pragma warning(default : 4003)

class Reseau;
class SensorsManager;
class Tuyau;
class AbstractSensorSnapshot;
class SimulationSnapshot;

#include <vector>
#include <map>

class TravelTimesOutputManagerSnapshot
{
public:
    int                                         nPeriode;
    std::map<Tuyau*, std::pair<int, double> >   mapTravelTimes;
    std::vector<AbstractSensorSnapshot*>        sensorsSnapshots;
};

class TravelTimesOutputManager
{
public:
    TravelTimesOutputManager();
    TravelTimesOutputManager(Reseau * pNetwork);
    virtual ~TravelTimesOutputManager();

    void Restore(TravelTimesOutputManagerSnapshot* pSnapshot);
    TravelTimesOutputManagerSnapshot * TakeSnapshot();


    void AddMeasuredTravelTime(Tuyau * pLink, double dbMeasuredTravelTime);

    void CalculInfoCapteurs(double dbInstant, bool bTerminate = false);

    // passage en mode d'écriture dans fichiers temporaires (affectation dynamique)
    void doUseTempFiles(SimulationSnapshot* pSimulationSnapshot);
    // validation de la dernière période d'affectation et recopie des fichiers temporaires dans les fichiers finaux
    void converge(SimulationSnapshot * pSimulationSnapshot);

private:
    Reseau*                                     m_pNetwork;

    SensorsManager*                             m_pSensorsManager;

    rapidjson::Document                         m_TravelTimes;

    SimulationSnapshot*                         m_pCurrentSimulationSnapshot;
    
    // variables de travail
    int                                         m_nPeriode;         // Numéro de la période courante
    std::map<Tuyau*, std::pair<int, double> >   m_mapTravelTimes;
};

#endif // traveltimsoutputmanagerH
