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
#include "CSVOutputWriter.h"

#include "tuyau.h"
#include "sensors/SensorsManager.h"
#include "sensors/PonctualSensor.h"
#include "SystemUtil.h"
#include "tools.h"
#include "vehicule.h"
#include "voie.h"
#include "SimulationSnapshot.h"

#define CSV_SEPARATOR ","
#define CSV_NEW_LINE std::endl

// prefixes des fichiers
#define CSV_EXTENSION ".csv"
#define LINKS_PREFIX "_links"
#define SENSORS_PREFIX "_sensors"
#define TRAJS_PREFIX "_trajs"
#define TRAJS_TEMP_PREFIX "_trajsTMP"
#define LINKCHANGES_PREFIX "_linkChanges"
#define LINKCHANGES_TEMP_PREFIX "_linkChangesTMP"
#define SENSORSDATA_PREFIX "_sensorsdata"
#define SENSORSDATA_TEMP_PREFIX "_sensorsdataTMP"

CSVOutputWriter::CSVOutputWriter()
{
	m_pCoordTransf = NULL;
    m_bTrajectoriesOutput = true;
    m_bSensorsOutput = true;

    m_pCurrentSimulationSnapshot = NULL;
}

CSVOutputWriter::CSVOutputWriter(const std::string& path, const std::string & suffix, const SDateTime& dateDebutSimu, OGRCoordinateTransformation *pCoordTransf,
                                 bool bTrajectoriesOutput, bool bSensorsOutput)
{
    m_Path = path;
    m_Suffix = suffix;
    m_StartTime = dateDebutSimu;

    m_pCoordTransf = pCoordTransf;

    m_bTrajectoriesOutput = bTrajectoriesOutput;
    m_bSensorsOutput = bSensorsOutput;

    m_pCurrentSimulationSnapshot = NULL;
}

CSVOutputWriter::~CSVOutputWriter()
{
    if(m_TrajsOFS.is_open())
    {
        m_TrajsOFS.close();
    }
    if(m_LinkChangesOFS.is_open())
    {
        m_LinkChangesOFS.close();
    }
    if(m_SensorsOFS.is_open())
    {
        m_SensorsOFS.close();
    }
}

// Ã©criture du fichier des tronÃ§ons
void CSVOutputWriter::writeLinksFile(const std::deque<Tuyau*> &lstTuyaux)
{
    if(lstTuyaux.size() > 0)
    {
        std::ofstream ofs((m_Path + LINKS_PREFIX + m_Suffix + CSV_EXTENSION).c_str(), std::ios::out);
        ofs.setf(std::ios::fixed);
		ofs << CSV_NEW_LINE;			// On saute la premiÃ¨re ligne

		 for(size_t i = 0; i < lstTuyaux.size(); i++)
		 {
			 Tuyau * pTuy = lstTuyaux[i];

			 if( pTuy->GetCnxAssAm() && pTuy->GetCnxAssAv() )
			 {
				 int nbPoints = (int)pTuy->GetLstPtsInternes().size()+2;

				 ofs << "(";

				// coordonnÃ©es des points
				for(int iPoint = 0; iPoint < nbPoints; iPoint++)
				{
					Point * pPoint;
					if(iPoint == 0)
					{
						pPoint = pTuy->GetExtAmont();
					}
					else if(iPoint == nbPoints-1)
					{
						pPoint = pTuy->GetExtAval();
					}
					else
					{
						pPoint = pTuy->GetLstPtsInternes()[iPoint-1];
					}

					if(m_pCoordTransf)
						m_pCoordTransf->Transform(1, &pPoint->dbX, &pPoint->dbY); 

					ofs << pPoint->dbX << " " << pPoint->dbY ;

					if(iPoint != nbPoints-1)
						ofs << "," ;
				}

				ofs << ")" << CSV_SEPARATOR;
				ofs << pTuy->GetCnxAssAm()->GetNumID() << CSV_SEPARATOR << pTuy->GetCnxAssAv()->GetNumID() << CSV_SEPARATOR ;
				ofs << pTuy->getNb_voies() << CSV_SEPARATOR ;
				ofs << pTuy->GetLabel() << CSV_SEPARATOR ;
				ofs << 0.00 << CSV_SEPARATOR << pTuy->GetLength()/1000.00 << CSV_SEPARATOR; 
				ofs << 50.00;		// TODO

				ofs << CSV_NEW_LINE;
			 }
        }

        ofs.close();
    }
}

// Ã©criture du fichier des capteurs
void CSVOutputWriter::writeSensorsFile(const std::vector<PonctualSensor*>& lstCapteurs)
{
    if(m_bSensorsOutput && lstCapteurs.size() > 0)
    {
        std::ofstream ofs((m_Path + SENSORS_PREFIX + m_Suffix + CSV_EXTENSION).c_str(), std::ios::out);
        ofs.setf(std::ios::fixed);

        for(size_t i = 0; i < lstCapteurs.size(); i++)
        {
            PonctualSensor * pCapteur = lstCapteurs[i];
            // Label et position XYZ
            ofs << pCapteur->GetUnifiedID();
			ofs << CSV_SEPARATOR << pCapteur->GetTuyau()->GetLabel();
			ofs << CSV_SEPARATOR << pCapteur->GetPosition()/1000;
			ofs << CSV_NEW_LINE;

          /*  Point pos;
            CalcCoordFromPos(pCapteur->pTuyau, pCapteur->dbPosition, pos);

			if(m_pCoordTransf)
				m_pCoordTransf->Transform(1, &pos.dbX, &pos.dbY); 

            ofs << CSV_SEPARATOR << pos.dbX << CSV_SEPARATOR << pos.dbY << CSV_SEPARATOR << pos.dbZ << CSV_NEW_LINE;*/
        }
        ofs.close();
    }
}

// Ã©criture d'une trajectoire
void CSVOutputWriter::writeTrajectory(double dbInstant, Vehicule * pVehicule)
{
    VoieMicro * pVoie = pVehicule->GetVoie(0);
    Tuyau * pTuyau = pVehicule->GetLink(0);

    // on ne sort pas les vÃ©hicules qui ne sont pas sur une voie et un tuyau bien dÃ©fini (sinon
    // on ne sait pas calculer les corrdonnÃ©es du vÃ©hciule, cf. Vehicule::CalculXYZ)
    if(!pVoie || !pTuyau)
        return;

    if(pVehicule->GetPos(0) <= UNDEF_POSITION && pTuyau->get_Type_amont()=='E')
		return;     // Pas de sortie rÃ©sultat pour les vÃ©hicules accumulÃ©s Ã  l'entrÃ©e

    std::ofstream *pOfsTrajs;
    if(m_pCurrentSimulationSnapshot)
    {
        pOfsTrajs = m_pCurrentSimulationSnapshot->m_pTrajsOFSTMP;
    }
    else
    {
        pOfsTrajs = &m_TrajsOFS;
    }

    if(!pOfsTrajs->is_open())
    {
        pOfsTrajs->open((m_Path + (m_pCurrentSimulationSnapshot ? TRAJS_TEMP_PREFIX : TRAJS_PREFIX) + m_Suffix + CSV_EXTENSION).c_str(), std::ios::out);
        pOfsTrajs->setf(std::ios::fixed);
    }

    double dbX, dbY, dbZ;
    pVehicule->CalculXYZ(dbX, dbY, dbZ);

	if(m_pCoordTransf)
		m_pCoordTransf->Transform(1, &dbX, &dbY); 

    int nSecs = (int)dbInstant;
    double ndbMS = dbInstant - nSecs;
    STimeSpan ts(nSecs);
    STime ti = m_StartTime.ToTime() + ts;

    std::ostringstream oss;
    oss << m_StartTime.ToDate().ToString() << " " << ti.ToString() << "." << std::setfill('0') << std::setw(3) << (int)(ndbMS*1000);

	*pOfsTrajs << pVehicule->GetID() << CSV_SEPARATOR << oss.str() << CSV_SEPARATOR << dbY << CSV_SEPARATOR << dbX << CSV_NEW_LINE;

 /*   *pOfsTrajs << oss.str()
        << CSV_SEPARATOR << pVehicule->GetID() << CSV_SEPARATOR << pVehicule->GetType()->GetLabel()
        << CSV_SEPARATOR << dbX << CSV_SEPARATOR << dbY << CSV_SEPARATOR << dbZ << CSV_SEPARATOR << pTuyau->GetLabel() << CSV_NEW_LINE;

    // on regarde s'il y a eu changement(s) de tronÃ§on sur un tronÃ§on non interne
    std::vector<Tuyau*> lstChangementsTuyaux;
    Tuyau * pPrevTuyau = pVehicule->GetLink(1);
    Tuyau * pNextTuyau = NULL;
    for(size_t i = 0; i < pVehicule->m_LstUsedLanes.size(); i++)
    {   
        pNextTuyau = (Tuyau*)pVehicule->m_LstUsedLanes[i]->GetParent();
        if(pNextTuyau != pPrevTuyau
            && !pNextTuyau->GetBriqueParente())
        {
            lstChangementsTuyaux.push_back((Tuyau*)pVehicule->m_LstUsedLanes[i]->GetParent());
        }
        pPrevTuyau = pNextTuyau;
    }
    if(pVehicule->GetLink(0) && pVehicule->GetLink(0) != pPrevTuyau)
    {
        lstChangementsTuyaux.push_back(pVehicule->GetLink(0));
    }
    // Ã©criture de chaque changement de tronÃ§on
    std::ofstream *pOfsChanges;
    if(m_pLinkChangesOFSTMP)
    {
        pOfsChanges = m_pLinkChangesOFSTMP;
    }
    else
    {
        pOfsChanges = &m_LinkChangesOFS;
    }
    for(size_t i = 0; i < lstChangementsTuyaux.size(); i++)
    {
        if(!pOfsChanges->is_open())
        {
            pOfsChanges->open(m_Path + (m_pLinkChangesOFSTMP?LINKCHANGES_TEMP_PREFIX:LINKCHANGES_PREFIX) + m_Suffix + CSV_EXTENSION, std::ios::out);
            pOfsChanges->setf(std::ios::fixed);            
        }
        *pOfsChanges << oss.str() << CSV_SEPARATOR << pVehicule->GetID() << CSV_SEPARATOR << lstChangementsTuyaux[i]->GetLabel() << CSV_NEW_LINE;
    }*/
}

// Ã©criture des donnÃ©es capteur pour une pÃ©riode
void CSVOutputWriter::writeSensorData(double dbInstant, const std::string& sID, int nNbVehFranchissant, double dbVitGlobal, const std::string& sVit)
{
    if(m_bSensorsOutput)
    {
        std::ofstream *pOfs;
        if(m_pCurrentSimulationSnapshot)
        {
            pOfs = m_pCurrentSimulationSnapshot->m_pSensorsOFSTMP;
        }
        else
        {
            pOfs = &m_SensorsOFS;
        }

        if(!pOfs->is_open())
        {
            pOfs->open((m_Path + (m_pCurrentSimulationSnapshot ? SENSORSDATA_TEMP_PREFIX : SENSORSDATA_PREFIX) + m_Suffix + CSV_EXTENSION).c_str(), std::ios::out);
            pOfs->setf(std::ios::fixed);
        }

        int nSecs = (int)dbInstant;
        double ndbMS = dbInstant - nSecs;
        STimeSpan ts(nSecs);
        STime ti = m_StartTime.ToTime() + ts;

	    if(dbVitGlobal<=0)
		    dbVitGlobal = 50;
	    else
		    dbVitGlobal= dbVitGlobal*3.6;

	    // CSV Outil de bruitage et d'extraction
        *pOfs << m_StartTime.ToDate().ToString() << " " << ti.ToString() << "." << std::setfill('0') << std::setw(3) << (int)(ndbMS*1000)
            << CSV_SEPARATOR << sID << CSV_SEPARATOR << nNbVehFranchissant << CSV_SEPARATOR << sVit << CSV_NEW_LINE;

	    // CSV Archipel
	    //*pOfs << SystemUtil::ToString(m_StartTime.ToDate().ToString()) << " " << SystemUtil::ToString(ti.ToString()) << "." << std::setfill('0') << std::setw(3) << (int)(ndbMS*1000);
	    //*pOfs << CSV_SEPARATOR << SystemUtil::ToString(sID) << CSV_SEPARATOR << "0" << CSV_SEPARATOR << dbVitGlobal << CSV_SEPARATOR << CSV_NEW_LINE;
    }
}

// passage en mode d'Ã©criture dans fichiers temporaires (affectation dynamique)
void CSVOutputWriter::doUseTempFiles(SimulationSnapshot * pSimulationSnapshot, size_t snapshotIdx)
{
    m_pCurrentSimulationSnapshot = pSimulationSnapshot;
    if(m_bTrajectoriesOutput)
    {
        assert(pSimulationSnapshot->m_pTrajsOFSTMP == NULL);
        pSimulationSnapshot->m_strTrajsOFSTMP = m_Path + TRAJS_TEMP_PREFIX + m_Suffix + (snapshotIdx == 0 ? "" : ("_" + SystemUtil::ToString(snapshotIdx))) + CSV_EXTENSION;
        pSimulationSnapshot->m_pTrajsOFSTMP = new std::ofstream(pSimulationSnapshot->m_strTrajsOFSTMP.c_str(), std::ios::out);
        
        //pSimulationSnapshot->m_strLinkChangesOFSTMP = m_Path + LINKCHANGES_TEMP_PREFIX;
        //m_pLinkChangesOFSTMP = new std::ofstream(m_strLinkChangesOFSTMP.c_str(), std::ios::out);
    }
    if(m_bSensorsOutput)
    {
        assert(pSimulationSnapshot->m_pSensorsOFSTMP == NULL);
        pSimulationSnapshot->m_strSensorsOFSTMP = m_Path + SENSORSDATA_TEMP_PREFIX + m_Suffix + (snapshotIdx == 0 ? "" : ("_" + SystemUtil::ToString(snapshotIdx))) + CSV_EXTENSION;
        pSimulationSnapshot->m_pSensorsOFSTMP = new std::ofstream(pSimulationSnapshot->m_strSensorsOFSTMP.c_str(), std::ios::out);
    }
}

// validation de la derniÃ¨re pÃ©riode d'affectation et recopie des fichiers temporaires dans les fichiers finaux
void CSVOutputWriter::converge(SimulationSnapshot * pSimulationSnapshot)
{
    // on recopie le contenu des fichiers temporaires dans les fichiers finaux
    if (pSimulationSnapshot->m_pTrajsOFSTMP)
    {
        pSimulationSnapshot->m_pTrajsOFSTMP->close();
        std::ifstream trajsIn((m_Path + TRAJS_TEMP_PREFIX + m_Suffix + CSV_EXTENSION).c_str(), std::ios::in);
        if(!m_TrajsOFS.is_open())
        {
            m_TrajsOFS.open((m_Path + TRAJS_PREFIX + m_Suffix + CSV_EXTENSION).c_str(), std::ios::out);
            m_TrajsOFS.setf(std::ios::fixed);
        }
        std::string strTmp;
        while(std::getline(trajsIn, strTmp))
        {
            m_TrajsOFS << strTmp << std::endl;
        }
        trajsIn.close();
    }
    if (pSimulationSnapshot->m_pLinkChangesOFSTMP)
    {
        pSimulationSnapshot->m_pLinkChangesOFSTMP->close();
        std::ifstream changesIn((m_Path + LINKCHANGES_TEMP_PREFIX + m_Suffix + CSV_EXTENSION).c_str(), std::ios::in);
        if(!m_LinkChangesOFS.is_open())
        {
            m_LinkChangesOFS.open((m_Path + LINKCHANGES_PREFIX + m_Suffix + CSV_EXTENSION).c_str(), std::ios::out);
            m_LinkChangesOFS.setf(std::ios::fixed);
        }
        std::string strTmp;
        while(std::getline(changesIn, strTmp))
        {
            m_LinkChangesOFS << strTmp << std::endl;
        }
        changesIn.close();
    }
    if (pSimulationSnapshot->m_pSensorsOFSTMP)
    {
        pSimulationSnapshot->m_pSensorsOFSTMP->close();
        std::ifstream sensorsIn((m_Path + SENSORSDATA_TEMP_PREFIX + m_Suffix + CSV_EXTENSION).c_str(), std::ios::in);
        if(!m_SensorsOFS.is_open())
        {
            m_SensorsOFS.open((m_Path + SENSORSDATA_PREFIX + m_Suffix + CSV_EXTENSION).c_str(), std::ios::out);
            m_SensorsOFS.setf(std::ios::fixed);
        }
        std::string strTmp;
        while(std::getline(sensorsIn, strTmp))
        {
            m_SensorsOFS << strTmp << std::endl;
        }
        sensorsIn.close();
    }
    m_pCurrentSimulationSnapshot = NULL;
}
