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

#include "DocTrafic.h"
#include "TimeUtil.h"

#pragma warning(disable : 4003)
#include <rapidjson/document.h>
#pragma warning(default : 4003)

class EVEDocTrafic;
class XMLDocTrafic;
class GMLDocTrafic;
class JSONDocTrafic;
class OGRCoordinateTransformation;
class Reseau;

namespace XERCES_CPP_NAMESPACE {
    class DOMDocument;
};

namespace eveShared {
    class TrafficState;
};

class TraceDocTrafic : public DocTrafic
{
private:
	EVEDocTrafic * m_pEveDocTrafic;
    XMLDocTrafic * m_pXMLDocTrafic;
    GMLDocTrafic * m_pGMLDocTrafic;
    JSONDocTrafic * m_pJSONDocTrafic;

    std::vector<DocTrafic*> m_LstDocTrafic;

    std::string m_strFileName;

public:
    TraceDocTrafic();
	TraceDocTrafic(Reseau * pNetwork, bool bEnableXMLDoc, bool bWriteXml, bool bDebug, bool bTraceStocks, bool bSortieLight, bool bSortieRegime, XERCES_CPP_NAMESPACE::DOMDocument * pXMLDocument,
        bool bEVEOutput, bool bGMLOutput, bool JSONOutput);
	virtual ~TraceDocTrafic();

public:

    XMLDocTrafic * GetXMLDocTrafic();
    EVEDocTrafic * GetEVEDocTrafic();
    GMLDocTrafic * GetGMLDocTrafic();
    JSONDocTrafic * GetJSONDocTrafic();

    std::string GetFileName();

	// Initialisation
	virtual void Init(const std::string & strFilename, const std::string & strVer, SDateTime dtDeb, SDateTime dtFin, double dbPerAgrCpts, XERCES_CPP_NAMESPACE::DOMDocument * xmlDocReseau, unsigned int uiInit,
        const std::string & simulationID, const std::string & traficID, const std::string & reseauID,
        const std::vector<Point> & enveloppe, TraceDocTrafic * pOriginalDocTrafic, OGRCoordinateTransformation *pCoordTransf, bool bHasAtLeastOneCDF, bool bHasSensor);

    // Post-initialisation
    virtual void PostInit();

    // Fin
	virtual void Terminate();

    // Suppression du fichier
    virtual void Remove();

	// Ajout d'un instant
	virtual void AddInstant(double dbInstant, double dbTimeStep, int nNbVeh);

    // Ajout d'un instant dans le doc XML (gestion particuliÃ¨re en mÃ©moire)
    virtual void AddInstantXML(double dbInstant, double dbTimeStep, int nNbVeh);

    // Ajout d'une pÃ©riode de surveillance des capteurs
    virtual void AddPeriodeCapteur(double dbDeb, double dbFin, const std::string& nodeName);

    virtual PCAPTEUR AddInfoCapteurLongitudinal(const std::string & sIdCpt, const std::string & sVoieOrigine, const std::string & sVoieDestination, const std::string & sCount);

    virtual PCAPTEUR AddInfoCapteurEdie(const std::string & sIdCpt, const std::string & sConcentrationCum, const std::string & sDebitCum, const std::string & sConcentration, const std::string & sDebit);

    virtual PCAPTEUR AddInfoCapteurMFD(const std::string & sIdCpt, bool bIncludeStrictData, const std::string & sGeomLength1, const std::string & sGeomLength2, const std::string & sGeomLength3,
        const std::string & sDistanceTotaleParcourue, const std::string & sDistanceTotaleParcourueStricte, const std::string & sTempsTotalPasse, const std::string & sTempsTotalPasseStrict,
        const std::string & sDebitSortie, const std::string &  sIntDebitSortie, const std::string &  sTransDebitSortie, const std::string & sDebitSortieStrict, const std::string & sLongueurDeplacement, const std::string & sLongueurDeplacementStricte,
        const std::string & sVitesseSpatiale, const std::string & sVitesseSpatialeStricte, const std::string & sConcentration, const std::string & sDebit);

    virtual PCAPTEUR AddInfoCapteurReservoir(const std::string & sIdCpt);

	virtual PCAPTEUR AddIndicateursCapteurGlobal(  double dbTravelledDistance, double dbTravelledTime, double dbVit, int nNbVeh);

    virtual PCAPTEUR AddInfoCapteurBlueTooth(const std::string & sIdCpt, const BlueToothSensorData& data);

    // Ajout d'une pÃ©riode de surveillance des capteurs
	virtual PCAPTEUR AddInfoCapteur(const std::string & sIdCpt, const std::string & sTypeVeh, const std::string & sVitGlob, const std::string & sNbCumGlob, const std::string & sVitVoie, const std::string & sNbCumVoie, const std::string & sFlow, bool bExtractVehicleList);

	virtual PCAPTEUR AddInfoCapteurGlobal( const std::string & sIdVeh, double TpsEcoulement, double DistanceParcourue);

    virtual void AddVehCpt(PCAPTEUR xmlNodeCpt, const std::string & sId, double dbInstPsg, int nVoie);

    virtual void AddTraverseeCpt(PCAPTEUR xmlNodeCpt, const std::string & vehId, const std::string & entryTime, const std::string & exitTime, const std::string & travelledDistance,
        const std::string & bCreatedInZone, const std::string & bDetroyedInZone);

    // Sauvegarde d'un instant
    virtual void SaveLastInstant();

    // Suppression du dernier instant en mÃ©moire DOM
    virtual void RemoveLastInstant();

	// Ajoute le contenu du document trafic de la source dans le document courant
	virtual void Add(DocTrafic * docTraficSrc);

    // Ajout d'une description d'une cellule de discrÃ©tisation
    virtual void AddCellule(int nID, const std::string & strLibelle, const std::string & strTuyau, double dbXam, double dbYam, double dbZam, double dbXav, double dbYav, double dbZav);

    // Ajout d'une description d'un tronÃ§on (dÃ©fini par l'utilisateur ou construit par Symubruit)
    virtual void AddTroncon(const std::string & strLibelle, Point * pPtAm, Point * pPtAv, int nVoie, const std::string & ssBrique, std::deque<Point*> &lstPtInterne, double dbLong, std::vector<double> dbLarg, double dbStockMax, double dbStockInitial, Tuyau * pTuyau);

    // Ajout d'une description d'une voie (dÃ©fini par l'utilisateur ou construit par Symubruit)
    virtual void AddVoie(Point * pPtAm, Point * pPtAv, std::deque<Point*> &lstPtInterne, double dbLong);

    // Ajout d'une description d'un capteur (de l'utilisateur ou crÃ©Ã© par Symubruit)
    virtual void AddDefCapteur(const std::string & sId, const std::string & sT, double dbPos, Point ptCoord);

    // Ajout d'une description d'un capteur longitudinal (de l'utilisateur)
    virtual void AddDefCapteurLongitudinal(const std::string & sId, const std::string & sT, double dbPosDebut, Point ptCoordDebut,
        double dbPosFin, Point ptCoordFin);

    // Ajout d'une description d'un capteur Edie (de l'utilisateur)
    virtual void AddDefCapteurEdie(const std::string & sId, const std::string & sT, double dbPosDebut, Point ptCoordDebut,
        double dbPosFin, Point ptCoordFin);

    // Ajout d'une description d'un capteur MFD (de l'utilisateur)
    virtual void AddDefCapteurMFD(const std::string & sId, bool bIncludeStrictData, const std::vector<Tuyau*>& lstTuyaux);

    // Ajout d'une description d'une entrÃ©e
    virtual void AddDefEntree(const std::string & sId, Point ptCoord);

    // Ajout d'une description d'une sortie
    virtual void AddDefSortie(const std::string & sId, Point ptCoord);

    // Ajout d'une description d'un CAF
    virtual void AddDefCAF(CarrefourAFeuxEx * pCAF);

    // Ajout d'une description d'un Giratoire
    virtual void AddDefGir(Giratoire * pGir);

    // Ajout d'une description d'un CDF
    virtual void AddDefCDF(ControleurDeFeux * pCDF);

    // Ajout d'une description d'un arrÃªt de bus
    virtual void AddArret(const std::string & strLibelle, int numVoie, char * strNomLigne);

    // Ajout de la description d'un vÃ©hicule
    virtual void AddVehicule(int nID, const std::string & strLibelle, const std::string & strType, const std::string & strGMLType, double dbKx, double dbVx, double dbW, const std::string & strEntree, const std::string & strSortie, const std::string &  strZoneEntree, const std::string &  strZoneSortie, const std::string & strRoute, double dbInstCreation, const std::string & sVoie, bool bAgressif, int iLane, const std::string & sLine, const std::vector<std::string> & initialPath, const std::deque<PlageAcceleration> & plagesAcc, const std::string & motifOrigine, const std::string & motifDestination, bool bAddToCreationNode);

    // ajout la description d'un vÃ©hicule sur sa crÃ©ation
    virtual void AddVehiculeToCreationNode(int nID, const std::string & strLibelle, const std::string & strType, double dbKx, double dbVx, double dbW, const std::string & strEntree, const std::string & strSortie, const std::string & strRoute, double dbInstCreation, const std::string & sVoie, bool bAgressif);

	// ajout la description d'un vÃ©hicule lors de la sortie d'un vÃ©hicule du rÃ©seau
	virtual void AddVehiculeToSortieNode(int nID, const std::string & strSortie, double dbInstSortie, double dbVit) {};

    // Ajout des sorties d'une brique de rÃ©gulation
    virtual void AddSimuRegulation(XERCES_CPP_NAMESPACE::DOMNode * pRestitutionNode);

    // Ajout de la description des donnÃ©es trafic d'une entrÃ©e
    virtual void AddSimuEntree(const std::string & sEntree, int nVeh);

    // Ajout de la description des donnÃ©es trafic d'un parking
    virtual void AddSimuParking(const std::string & sParking, int nStock, int nVehAttente);

    // Ajout de la description des donnÃ©es trafic d'un troncon de stationnement
    virtual void AddSimuTronconStationnement(const std::string & sTroncon, double dbLongueur);

	// Mise Ã  jour de l'instant d'entrÃ©e du vÃ©hicule (peut Ãªtre diffÃ©rent de l'instant de crÃ©ation
	virtual void UpdateInstEntreeVehicule(int nID, double dbInstEntree);

    // Mise Ã  jour de l'instant de sortie du vÃ©hicule
    virtual void UpdateInstSortieVehicule(int nID, double dbInstSortie, const std::string & sSortie, double dstParcourue, const std::vector<Tuyau*> & itinerary, const std::map<std::string, std::string> & additionalAttributes);

    // Suppression des vÃ©hicules mÃ©morisÃ©es Ã  partir d'un certain ID
    virtual void RemoveVehicules(int nFromID);
	
    // Ajout de l'Ã©tat des feux pour l'instant considÃ©rÃ©
	virtual void AddSimFeux(const std::string & sCtrlFeux, const std::string & sTE, const std::string & sTS, int bEtatFeu, int bPremierInstCycle, int bPrioritaire);

    // Ajout de l'Ã©tat des feux pour l'instant considÃ©rÃ© pour EVE
	virtual void AddSimFeuxEVE(const std::string & sCtrlFeux, const std::string & sTE, const std::string & sTS, int bEtatFeu, int bPremierInstCycle, int bPrioritaire);

    // Ajout des donnÃ©es complÃ¨te trafic d'une cellule de discrÃ©tisation pour l'instant considÃ©rÃ©
	virtual void AddCellSimu(int nID, double dbConc, double dbDebit, double dbVitAm, double dbAccAm, double bNAm, double dbVitAv, double dbAccAv, double dbNAv, const std::string & strLibelle, const std::string & strTuyau, double dbXam, double dbYam, double dbZam, double dbXav, double dbYav, double dbZav);

    // Ajout des donnÃ©es de la trajectoire d'un vÃ©hicule Ã  l'instant considÃ©rÃ©
	virtual void AddTrajectoire(int nID, Tuyau * pTuyau, const std::string & strTuyau, const std::string & strTuyauEx, const std::string & strNextTuyauEx, int nNumVoie, double dbAbs, double dbOrd, double dbZ, double dbAbsCur, double dbVit, double dbAcc, double dbDeltaN, const std::string & sTypeVehicule, double dbVitMax, double deLongueur, const std::string & sLib, int nIDVehLeader, int nCurrentLoad, bool bTypeChgtVoie, TypeChgtVoie eTypeChgtVoie,
        bool bVoieCible, int nVoieCible, bool bPi, double dbPi, bool bPhi, double dbPhi, bool bRand, double dbRand, bool bDriven, const std::string & strDriveState, bool bDepassement, bool bRegimeFluide, const std::map<std::string, std::string> & additionalAttributes);

    // Ajout des donnÃ©es du flux d'un vÃ©hicule Ã  l'instant considÃ©rÃ©
	virtual void AddStream(int nID, const std::string & strTuyau, const std::string & strTuyauEx);

    // Ajout des donnÃ©es d'un tuyau Ã  l'instant considÃ©rÃ©
    virtual void AddLink(const std::string &  strTuyau, double dbConcentration);

    // Obtention des informations du trafic
    eveShared::TrafficState * GetTrafficState();
    const rapidjson::Document & GetTrafficStateJSON();

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SÃ©rialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:

    static const rapidjson::Document s_EMPTY_JSON_DOC;

	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version);
};
