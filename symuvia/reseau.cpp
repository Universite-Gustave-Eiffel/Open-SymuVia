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
#include "reseau.h"

#include "frontiere.h"
#include "repartiteur.h"
#include "sortie.h"
#include "entree.h"
#include "XMLDocEmissionsAtmo.h"
#include "divergent.h"
#include "CarrefourAFeuxEx.h"
#include "ControleurDeFeux.h"
#include "MergingObject.h"
#include "TraceDocTrafic.h"
#include "TraceDocAcoustique.h"
#include "XMLReaderTrafic.h"
#include "RandManager.h"
#include "Parking.h"
#include "ZoneDeTerminaison.h"
#include "regulation/RegulationModule.h"
#include "regulation/PythonUtils.h"
#include "Affectation.h"
#include "DiagFonda.h"
#include "Xerces/XMLUtil.h"
#include "vehicule.h"
#include "tuyau.h"
#include "TuyauMeso.h"
#include "convergent.h"
#include "SystemUtil.h"
#include "XMLDocTrafic.h"
#include "EVEDocAcoustique.h"
#include "voie.h"
#include "loi_emission.h"
#include "XMLDocSirane.h"
#include "Segment.h"
#include "arret.h"
#include "CSVOutputWriter.h"
#include "carFollowing/AbstractCarFollowing.h"
#include "carFollowing/NewellContext.h"
#include "carFollowing/NewellCarFollowing.h"
#include "carFollowing/CarFollowingFactory.h"
#include "Logger.h"
#include "sensors/PonctualSensor.h"
#include "sensors/LongitudinalSensor.h"
#include "sensors/EdieSensor.h"
#include "sensors/MFDSensor.h"
#include "sensors/TankSensor.h"
#include "sensors/SensorsManager.h"
#include "SymuCoreManager.h"
#include "SQLNetworkExporter.h"
#include "GTFSExporter.h"
#include "TronconOrigine.h"
#include "TronconDestination.h"
#include "RepartitionTypeVehicule.h"
#include "Motif.h"
#include "usage/Trip.h"
#include "usage/TripLeg.h"
#include "usage/SymuViaFleet.h"
#include "usage/SymuViaFleetParameters.h"
#include "usage/PublicTransportFleet.h"
#include "usage/logistic/DeliveryFleet.h"
#include "usage/SymuViaVehicleToCreate.h"
#include "usage/PublicTransportLine.h"
#include "RepMotif.h"
#include "Plaque.h"
#include "TravelTimesOutputManager.h"
#include "ParkingParameters.h"
#ifdef USE_SYMUCORE
#include "SymuCoreGraphBuilder.h"
#include "SymuCoreDrivingTripsBuilder.h"
#include <Demand/SubPopulation.h>
#include <Demand/Population.h>
#include <Demand/Destination.h>
#include <Demand/MacroType.h>
#include <Demand/Origin.h>
#include <Graph/Model/MultiLayersGraph.h>
#include <Demand/VehicleType.h>
#include <Users/IUserHandler.h>
#endif // USE_SYMUCORE

#ifdef USE_SYMUCOM
#include "ITS/Stations/DerivedClass/Simulator.h"
#include "ITS/Stations/ITSStation.h"
#include "ITS/Applications/C_ITSApplication.h"
#include <Communication/Graph.h>
#include <Communication/CommunicationRunner.h>
#endif // USE_SYMUCOM

#pragma warning(disable : 4003)
#include <rapidjson/document.h>
#include <rapidjson/writer.h>
#pragma warning(default : 4003)

#include <xercesc/dom/DOMDocument.hpp>
#include <xercesc/dom/DOMElement.hpp>
#include <xercesc/dom/DOMNode.hpp>
#include <xercesc/dom/DOMNodeList.hpp>
#include <xercesc/dom/DOMException.hpp>
#include <xercesc/sax/SAXException.hpp>

#include <boost/thread/mutex.hpp>

#include <boost/make_shared.hpp>

#include <boost/serialization/deque.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/shared_ptr.hpp>

#ifdef USE_SYMUCORE
#include <boost/date_time/posix_time/posix_time_types.hpp>
#endif // USE_SYMUCORE

using namespace std;

XERCES_CPP_NAMESPACE_USE


// Version du formats des fichiers
const char* VERSION_FICHIER = "2.05";

#define BUFSIZE MAX_PATH

bool isSameTuyauPredicate(Tuyau * pTuyau, void * pArg)
{
    Tuyau * pTuyauRef = (Tuyau*)pArg;
    return pTuyauRef == pTuyau;
}

//---------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------
// Constructeur, destructeur et assimilÃ©
//---------------------------------------------------------------------------------------------

// Constructeur par dÃ©faut
Reseau::Reseau()
{
    g_FicDebug = new std::ofstream();
    FicTraficSAS = new std::ofstream();
    m_pLogger = NULL;

    m_pCarFollowingFactory = new CarFollowingFactory(this);
    m_pRegulationModule = new RegulationModule(this);
    m_pSymuScriptManager = new SymuCoreManager(this);

    m_pParkingParameters = new ParkingParameters();

    m_pPythonUtils = NULL;
    m_pRandManager = new RandManager();

    m_pMutex = new boost::mutex();

	m_bMeso = false;

    m_pXMLUtil = new XMLUtil();

    Initialize();
}

// constructeur
Reseau::Reseau(string sLogOutFile) :
m_sLogOutFile(sLogOutFile)
{
	g_FicDebug = new std::ofstream();
    FicTraficSAS = new std::ofstream();
    m_pLogger = NULL;

    m_pCarFollowingFactory = new CarFollowingFactory(this);
    m_pRegulationModule = new RegulationModule(this);
    m_pSymuScriptManager = new SymuCoreManager(this);

    m_pParkingParameters = new ParkingParameters();

    m_pPythonUtils = NULL;
    m_pRandManager = new RandManager();

    m_pMutex = new boost::mutex();

	m_bMeso = false;

    m_pXMLUtil = new XMLUtil();

    Initialize();
}

PythonUtils* Reseau::GetPythonUtils()
{
    // on fait en sorte de ne pas l'instancier si pas nÃ©cessaire pour ne pas rÃ©gresser par rapport au singleton, et 
    // ne pas voir de message d'avertissement s'il manque les scripts alors qu'on ne s'en sert pas.
    if (!m_pPythonUtils)
    {
        m_pPythonUtils = new PythonUtils();
        m_pPythonUtils->setLogger(m_pLogger);
    }
    return m_pPythonUtils;
}

// Initialise les membres du rÃ©seau
void Reseau::Initialize()
{
    m_nReplications = 1;
    m_sSuffixOutputFiles = "";

    m_dbInstSimu = 0;
    pas_de_temps = 0  ;
    hd = 0;
    md = 0;
    sd = 0;
    hf = 0;
    mf = 0;
    sf = 0;

    m_bSortieTraj = true;
    m_bDisableTrajectoriesOutput = false;
    m_bTraceRoute = false;
    m_bVGPDebug = false;
    m_bTraceStocks = false;
    m_bSortieRegime = false;
    m_bCSVOutput = false;
    m_bTravelTimesOutput = false;
    m_dbTravelTimesOutputPeriod = 0;
    m_bCSVTrajectories = false;
    m_bCSVSensors = false;
    m_bGMLOutput = false;

    //rmq. on initialise un logger sans fichier ici, afin d'Ã©viter de tester la NULLitÃ©  de m_pLogger Ã  chaque utilisation
    m_pLogger = new Logger("", true, Logger::Info);
    m_pLoadingLogger = NULL;

    Loi_emis = NULL;

    m_bFichierSAS = false;

    m_nLastVehicleID = 0;

    m_bChgtVoie = false;

    m_bDebug = false;
    m_bDebugOD = false;
    m_bDebugSAS = false;
    m_bChgtVoieDebug = false;

    m_bSortieLight = false;

    m_pGestionsCapteur = NULL;

    m_bCptDirectionnel = true;         // Par dÃ©faut, le comportement du flux ou des vÃ©hicules est
    m_bCptDestination = false;         // directionnel (Ã©vite la reprise des anciens fichiers oÃ¹
    m_bCptItineraire = false;          // uniquement le comportement directionnel existait)
    m_bAffectationDynamique = false;

    m_nLastIdSegment = 0;

    m_dbDstChgtVoie = 200;              // Par dÃ©faut

    m_dbDstChgtVoieForce = -1;
    m_dbPhiChgtVoieForce = 1;

    m_dbDiffDebutSimuData = 0;

    m_bLoiPoursuiteOld = false;         // Par dÃ©faut, nouvelle loi de poursuite

    m_bInitSimuTrafic = false;
    m_bInitSimuEmissions = false;

    m_strTitre = "";

    m_SymMode = Reseau::SYM_MODE_FULL;

    m_bXmlOutput = true;

    m_bProcDec = false;
    m_dbDecTaux = 0;

    m_uiSeed = 0;
    m_uiRandomSeed = 0;
    m_bPickedSeed = false;

    m_pModuleAffectation = NULL;

    m_bAcousCell = false;
    m_bAcousSrcs = false;

    m_XMLDocData = NULL;


    m_XmlDocAcoustique = NULL;
    m_XmlDocAtmo = NULL;
    m_XmlReaderTrafic = NULL;
    m_XmlDocSirane = NULL;
    m_CSVOutputWriter = NULL;
    m_pTravelTimesOutputManager = NULL;

    m_bSaveAffectation = false;

    m_bDriven = false;

    m_bDepassement = true;
    m_bTraversees = true;

    m_dbDebutPeriodeSirane = 0.0;
    m_bExtensionBarycentresSirane = false;

    m_bRelancerCalculRepartiteur = false;

    m_dbTetaLogit = 0.01;
    m_nMode = 1;
    m_dbWardropTolerance = 0.01;

    m_eShortestPathHeuristic = HEURISTIC_NONE;
    m_dbHeuristicGamma = 5.0;
    m_dbAStarBeta = 1.3;

    m_dbDijkstraAlpha = 0.05;
    m_dbDijkstraBeta = 20;
    m_dbDijkstraGamma = 20;

    m_bCommonalityFilter = false;
    m_dbCommonalityAlpha = 0;
    m_dbCommonalityBeta = 0.5;
    m_dbCommonalityGamma = 1.0;

    m_dbRightTurnPenalty = 1;
    m_dbNonPriorityPenalty = 5;
    m_dbAngleToutDroit = 30;
    m_dbRightTurnPenaltyFactor = 1.1;
    m_dbNonPriorityPenaltyFactor = 1.5;

    m_bTypeProfil = true;
    m_dbLag = 0;

    m_pCurrentProcessedNode = NULL;

    m_pOGRSpRefInput = NULL;
    m_pOGRSpRefOutput = NULL;
    m_pCoordTransf = NULL;

    m_nIncCnx = 1000;

    m_bLogFile = true;
    m_bLogFileLight = false;

    m_xmlDocTrafics.clear();
    m_LstItiChangeInstants.clear();

    m_LstFleets.push_back(new SymuViaFleet(this));
    m_LstFleets.push_back(new PublicTransportFleet(this));
    m_LstFleets.push_back(new DeliveryFleet(this));

    m_MicroVehicleTypes.clear();
    m_dbUpstreamMicroDistance = 0;
    m_dbDownstreamMicroDistance = 0;
    m_dbSidesMicroDistance = 0;

    m_dbChgtVoieGhostBevelLength = 0;
    m_nChgtVoieGhostDurationMin = 0;
    m_nChgtVoieGhostDurationMax = 0;

    m_nMesoNbVehChgtVoie = 1;

    m_bModeDepassementChgtDir = false;
    m_dbDistDepassementChgtDir = 20;


    m_dbCptCumCO2 = 0;
    m_dbCptCumNOx = 0;
    m_dbCptCumPM = 0;
#ifdef USE_SYMUCORE
    m_pSymuMasterUserHandler = NULL;
    m_pGraphBuilder = NULL;
    m_nMarginalsDeltaN = 0;
    m_bUseMarginalsMeanMethod = false;
    m_bUseMarginalsMedianMethod = false;
    m_nbVehiclesForTravelTimes = 0;
    m_nbPeriodsForTravelTimes = 0;
    m_bUseSpatialTTMethod = false;
    m_dbMinSpeedForTravelTime = 0;
    m_dbMaxMarginalsValue = 9999;
    m_bUseTravelTimesAsMarginalsInAreas = false;
    m_dbConcentrationRatioForFreeFlowCriterion = 0.5;
    m_bUseLastBusIfAnyForTravelTimes = true;
    m_bUseEmptyTravelTimesForBusLines = false;
    m_dbMeanBusExitTime = 4;
    m_nbStopsToConnectOutsideOfSubAreas = 1;
    m_dbMaxIntermediateWalkRadius = -1;
    m_dbMaxInitialWalkRadius = -1;
    m_dbMinLengthForMarginals = -1;
    m_dbWalkSpeed = -1;
    m_pVLMacroType = NULL;
    m_bComputeAllCosts = false;
#endif // USE_SYMUCORE
    m_bEstimateTrafficLightsWaitTime = true;


    m_bSymuMasterMode = false;
    m_bWithPublicTransportGraph = false;

    m_bUseMapRouteFromNodes = false;

#ifdef USE_SYMUCOM
    m_pSymucomSimulator = NULL;
#endif // USE_SYMUCOM
}

// destructeur
Reseau::~Reseau()
{
    if(IsInitSimuTrafic())
        FinSimuTrafic();

    FinSimuEmissions( IsSimuAcoustique(), IsSimuAir(), IsSimuSirane());

    vidange_listes() ;

    if(m_pModuleAffectation)
    {
        delete(m_pModuleAffectation);
        m_pModuleAffectation = NULL;
    }

    //if(Loi_emis)
      //  delete Loi_emis;      // efface la classe loi d'emission

    if(m_pGestionsCapteur)
    {
        delete m_pGestionsCapteur;
        m_pGestionsCapteur = NULL;
    }

    if(m_pCoordTransf)
    {
        delete m_pCoordTransf;
        m_pCoordTransf = NULL;
    }

    if(m_XMLDocData)
    {
        m_pXMLUtil->CleanLoadDocument(m_XMLDocData);
    }
    m_XMLDocData = NULL;

    std::vector<PlageTemporelle *>::iterator itRange;
    for( itRange = m_LstPlagesTemporelles.begin(); itRange!= m_LstPlagesTemporelles.end(); itRange++)
    {
        delete *itRange;
    }
    for( itRange = m_extractionRange.begin(); itRange!= m_extractionRange.end(); itRange++)
    {
        delete *itRange;
    }

    delete m_pSymuScriptManager;
    delete m_pCarFollowingFactory;
    delete m_pRegulationModule;

    delete m_pParkingParameters;

    m_mapFirstVehicles.clear(); // Pour que les vÃ©hicules correspondants soient dÃ©truits avant de dÃ©truire le fichier de log et pouvoir logger pendant leur destruction.

#ifdef USE_SYMUCORE
    if (m_pGraphBuilder)
    {
        delete m_pGraphBuilder;
    }
#endif // USE_SYMUCORE

#ifdef USE_SYMUCOM
    if (m_pSymucomSimulator)
    {
        delete m_pSymucomSimulator->GetCommunicationRunner();
    }
#endif // USE_SYMUCOM

    RemoveLogFicSimulation();

    if (m_pPythonUtils)
    {
        delete m_pPythonUtils;
    }
    delete m_pRandManager;

    delete g_FicDebug;
    delete FicTraficSAS;

    delete m_pMutex;

    delete m_pXMLUtil;
}

// Vidange des listes
void Reseau::vidange_listes(void)
{
    // Destruction de la liste des types de vÃ©hicule
    std::deque <TypeVehicule*>::iterator ItCurTypeVeh;
    for (ItCurTypeVeh=m_LstTypesVehicule.begin();ItCurTypeVeh!=m_LstTypesVehicule.end();++ItCurTypeVeh)
    {
        delete *(ItCurTypeVeh);
    }
    m_LstTypesVehicule.clear();
    //m_LstAssignmentVehicleType.clear();

    Liste_entree.clear();

    std::deque <MergingObject*>::iterator itMO;
    for(itMO = m_LstMergingObjects.begin(); itMO != m_LstMergingObjects.end(); ++itMO)
    {
        delete( (MergingObject*)(*itMO) );
    }
    m_LstMergingObjects.erase( m_LstMergingObjects.begin(), m_LstMergingObjects.end() );

  std::deque <Tuyau*>::iterator par;
  std::deque <Tuyau*>::iterator debut = m_LstTuyaux.begin();
  std::deque <Tuyau*>::iterator fin = m_LstTuyaux.end();

  std::deque <TuyauMacro*>::iterator parMac;
  std::deque <TuyauMacro*>::iterator debutMac;
  std::deque <TuyauMacro*>::iterator finMac;

  debutMac= m_LstTuyauxMacro.begin();
  finMac=   m_LstTuyauxMacro.end();
  for (parMac = debutMac;parMac!=finMac;++parMac)
  {
    delete (TuyauMacro*)(*parMac);
  }

  std::deque <TuyauMicro*>::iterator parMic;
  std::deque <TuyauMicro*>::iterator debutMic;
  std::deque <TuyauMicro*>::iterator finMic;

  debutMic= m_LstTuyauxMicro.begin();
  finMic=   m_LstTuyauxMicro.end();
  for (parMic = debutMic;parMic!=finMic;++parMic)
  {
    delete (*parMic);
  }

  m_LstTuyaux.erase(debut,fin);
  m_LstTuyauxMacro.erase(m_LstTuyauxMacro.begin(),m_LstTuyauxMacro.end() );
  m_LstTuyauxMicro.erase(m_LstTuyauxMicro.begin(),m_LstTuyauxMicro.end() );

    // Suppression des rÃ©partiteurs
    std::map<std::string, Repartiteur*>::iterator rep;
    std::map<std::string, Repartiteur*>::iterator rep_d = Liste_repartiteurs.begin();
    std::map<std::string, Repartiteur*>::iterator rep_f = Liste_repartiteurs.end();
    for (rep=rep_d; rep!=rep_f; ++rep)
    {
        delete (Repartiteur*)(rep->second);
    }
    Liste_repartiteurs.clear();

    // Suppression des giratoires
    std::deque <Giratoire*>::iterator par_g;
    std::deque <Giratoire*>::iterator debut_g;
    std::deque <Giratoire*>::iterator fin_g;
    debut_g= Liste_giratoires.begin();
    fin_g=   Liste_giratoires.end();
    for (par_g=debut_g;par_g!=fin_g;++par_g)
    {
        delete (Giratoire*)(*par_g);
    }
    Liste_giratoires.erase(  Liste_giratoires.begin(),Liste_giratoires.end());

    // Suppression des controleurs de feux
    std::deque <ControleurDeFeux*>::iterator par_cdf;
    std::deque <ControleurDeFeux*>::iterator debut_cdf;
    std::deque <ControleurDeFeux*>::iterator fin_cdf;
    debut_cdf= Liste_ControleursDeFeux.begin();
    fin_cdf=   Liste_ControleursDeFeux.end();
    for (par_cdf=debut_cdf;par_cdf!=fin_cdf;++par_cdf)
    {
        delete (ControleurDeFeux*)(*par_cdf);
    }
    Liste_ControleursDeFeux.erase(  Liste_ControleursDeFeux.begin(),Liste_ControleursDeFeux.end());

    // Suppression des convergents
    std::map<std::string, Convergent*>::iterator par_cvg;
    std::map<std::string, Convergent*>::iterator debut_cvg;
    std::map<std::string, Convergent*>::iterator fin_cvg;
    debut_cvg= Liste_convergents.begin();
    fin_cvg=   Liste_convergents.end();
    for (par_cvg=debut_cvg;par_cvg!=fin_cvg;++par_cvg)
    {
        delete (Convergent*)(par_cvg->second);
    }
    Liste_convergents.clear();

    // Suppression des divergents
    std::map<std::string, Divergent*>::iterator div;
    std::map<std::string, Divergent*>::iterator div_d = Liste_divergents.begin();
    std::map<std::string, Divergent*>::iterator div_f = Liste_divergents.end();
    for (div=div_d; div!=div_f; ++div)
    {
        delete (Divergent*)(div->second);
    }
    Liste_divergents.clear();


    Liste_sortie.clear();
    Liste_parkings.clear();
    Liste_zones.clear();

    // Suppression des carrefours Ã  feux
    std::deque <CarrefourAFeuxEx*>::iterator par_caf;
    std::deque <CarrefourAFeuxEx*>::iterator debut_caf;
    std::deque <CarrefourAFeuxEx*>::iterator fin_caf;
    debut_caf= Liste_carrefoursAFeux.begin();
    fin_caf=   Liste_carrefoursAFeux.end();
    for (par_caf=debut_caf;par_caf!=fin_caf;par_caf++)
    {
        delete (CarrefourAFeuxEx*)(*par_caf);
    }
    Liste_carrefoursAFeux.erase(  Liste_carrefoursAFeux.begin(),Liste_carrefoursAFeux.end());

  // Destruction de la liste des vÃ©hicules
  m_LstVehicles.clear();

  // Destruction de la liste initiale des vÃ©hicules
  m_LstInitVehicule.clear();

  // vidange de la liste des Oigines (sans delete car le delete est fait
  // dans la vidange de la liste des entrÃ©es, parkings et zones de terminaison)
  Liste_origines.clear();
  Liste_destinations.clear();

  // Vidange de la liste des vÃ©hicules Ã  crÃ©er
  for(size_t i = 0; i < m_VehiclesToCreate.size(); i++)
  {
      delete m_VehiclesToCreate[i];
  }
  m_VehiclesToCreate.clear();

  // Vidange de la liste des flottes
  for(size_t i = 0; i < m_LstFleets.size(); i++)
  {
    delete m_LstFleets[i];
  }
  m_LstFleets.clear();

  // Vidange de la liste des motifs
  for (size_t i = 0; i < m_LstMotifs.size(); i++)
  {
      delete m_LstMotifs[i];
  }
  m_LstMotifs.clear();

  for (std::map<Tuyau*, TronconOrigine*>::const_iterator iter = m_mapOriginLinksForCreatedVehicles.begin(); iter != m_mapOriginLinksForCreatedVehicles.end(); ++iter)
  {
      delete iter->second;
  }
  for (std::map<Tuyau*, TronconDestination*>::const_iterator iter = m_mapDestinationLinksForCreatedVehicles.begin(); iter != m_mapDestinationLinksForCreatedVehicles.end(); ++iter)
  {
      delete iter->second;
  }
}

//================================================================
    bool Reseau::Execution
//----------------------------------------------------------------
// Fonction  : Lancement de l'Ã©xecution
// Version du: 13/07/2007
// Historique: 13/07/2007 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//================================================================
(
)
{
    // Initialisation
    SetSymMode(Reseau::SYM_MODE_FULL);

    // Pour toutes les rÃ©plications
    for (int iReplication = 1; iReplication <= m_nReplications; iReplication++)
    {
        // Position de la graine si on fait plusieurs rÃ©plications (on ignore le paramÃ¨tre "seed" Ã©ventuellement dÃ©fini)
        if (m_nReplications > 1)
        {
            m_uiSeed = iReplication;
            std::ostringstream oss;
            oss << "_" << iReplication;
            m_sSuffixOutputFiles = oss.str();
        }

        // Simulation du trafic
        if (m_bSimuTrafic)
        {
            log() << "Traffic simulation initialization..." << std::endl;

            if (InitSimuTrafic(iReplication == m_nReplications))
            {
                log() << "Running traffic simulation..." << std::endl;

                bool bEnd = false;
                bool bEndStep;
                while (!bEnd)
                {
                    bEnd = SimuTrafic(bEndStep);

                    PostTimeStepSimulation();
                }

                if (IsCptItineraire())// && m_bSaveAffectation)
                {
                    m_pModuleAffectation->CloseSaveAffectation();
                }


                log() << std::endl << std::endl << "Traffic simulation termination..." << std::endl << std::endl;

                FinSimuTrafic();
            }
            else
            {
                return false;
            }
        }

        // Simulation des Ã©missions acoustiques et:ou atmosphÃ©riques
        if (m_bSimuAcoustique || m_bSimuAir || m_bSimuSirane)
        {
            log() << "Emissions simulation initialization..." << std::endl;

            if (InitSimuEmissions(m_bSimuAcoustique, m_bSimuAir, m_bSimuSirane))
            {
                log() << "Running emissions simulation..." << std::endl;

                while (!SimuEmissions(m_bSimuAcoustique, m_bSimuAir, m_bSimuSirane))
                {
                    if (m_XmlDocAcoustique)
                    {
                        m_XmlDocAcoustique->ReleaseLastInstant();
                    }
                }

                log() << "Emissions simulation termination..." << std::endl;

                FinSimuEmissions(m_bSimuAcoustique, m_bSimuAir, m_bSimuSirane);
            }
            else
            {
                return false;
            }
        }
    }

    log()<<"Execution finished"<<std::endl;

    return true;
}

//================================================================
    void Reseau::InitLogFicSimulation
//----------------------------------------------------------------
// Fonction  : Ouverture du fichier de log de la simulation
// Version du: 25/05/2011
// Historique: 25/05/2011 (O.Tonck - IPSIS)
//             CrÃ©ation
//================================================================
(
 )
{
    std::string fileName = m_sPrefixOutputFiles + "_Simulation" + m_sSuffixOutputFiles + ".log";
    if(m_pLogger && m_pLogger->getFileName().compare(fileName))
    {
        RemoveLogFicSimulation();
    }
    if( !m_pLogger )
    {
        m_pLogger = new Logger(fileName, m_bLogFile, m_bLogFileLight?Logger::Warning:Logger::Info);
        m_pLogger->SetNetwork(this);
        if (m_pPythonUtils)
        {
            m_pPythonUtils->setLogger(m_pLogger);
        }
    }
}

void Reseau::RemoveLogFicSimulation()
{
    if (m_pPythonUtils)
    {
        m_pPythonUtils->setLogger(NULL);
    }
    if(m_pLogger)
    {
        delete m_pLogger;
        m_pLogger = NULL;
    }
}

Logger& Reseau::log()
{
    return *m_pLogger;
}

Logger* Reseau::GetLogger()
{
    // Si pas de logger de simulation dÃ©jÃ  initialisÃ©, on utilise le logger du chargement pour ne pas rater de messages
    if (m_pLoadingLogger && (m_pLogger == NULL || m_pLogger->getFileName().empty()))
    {
        return m_pLoadingLogger;
    }
    else
    {
        return m_pLogger;
    }
}

//! PrÃ©pare les vecteurs Ã  passer au constructeur des capteurs MFD
void Reseau::PrepareLinksForMFD(const std::vector<Tuyau*> & zoneLinks, Logger * pLoadingLogger, const std::string & sensorName,
    std::vector<Tuyau*> & Tuyaux, std::vector<double> & dbPosDebut, std::vector<double> & dbPosFin, std::vector<int> & eTuyauType)
{
    // Ajout des tuyaux ajoutÃ©s explicitement
    for (size_t iLink = 0; iLink < zoneLinks.size(); iLink++)
    {
        Tuyau * pTuyau = zoneLinks[iLink];

        // dÃ©tection des Ã©ventuels tuyaux macros dans la liste
        if (pTuyau->IsMacro())
        {
            if (pLoadingLogger)
            {
                *pLoadingLogger << Logger::Warning << " WARNING : The sensor " << sensorName << " can't be placed on a macroscopic link : link "
                    << pTuyau->GetLabel() << " ignored..." << endl;
                *pLoadingLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
            }
        }
        else
        {
            Tuyaux.push_back(pTuyau);
            dbPosDebut.push_back(0.0);
            dbPosFin.push_back(pTuyau->GetLength());
            eTuyauType.push_back(EdieSensor::TT_Link);
        }
    }

    // ajout des tuyaux internes de la brique aval, dont le tuyau amont fait partie de la zone
    set<Tuyau *> tuyauxAval;
    for (size_t iLink = 0; iLink < zoneLinks.size(); iLink++)
    {
        Tuyau * pTuyAmont = zoneLinks[iLink];
        // si la connexion aval est une brique
        BriqueDeConnexion * pBriqueAval = pTuyAmont->GetBriqueAval();
        if (pBriqueAval != NULL)
        {
            // On ajoute les tuyaux internes de la brique aval, dont le tuyau amont est le tuyau courant pTuyAmont
            std::deque<Tuyau*>::iterator itTuyauAval;
            for (itTuyauAval = pBriqueAval->m_LstTuyAv.begin(); itTuyauAval != pBriqueAval->m_LstTuyAv.end(); itTuyauAval++)
            {
                set<Tuyau*> tuyauxAvalToAdd = pBriqueAval->GetAllTuyauxInternes(pTuyAmont, *itTuyauAval);
                tuyauxAval.insert(tuyauxAvalToAdd.begin(), tuyauxAvalToAdd.end());
            }
        }// si la connexion aval est une brique
    }
    set<Tuyau *>::iterator iterTuyIntSet;
    for (iterTuyIntSet = tuyauxAval.begin(); iterTuyIntSet != tuyauxAval.end(); iterTuyIntSet++)
    {
        Tuyaux.push_back(*iterTuyIntSet);
        dbPosDebut.push_back(0.0);
        dbPosFin.push_back((*iterTuyIntSet)->GetLength());
        eTuyauType.push_back(EdieSensor::TT_Aval);
    }
}

bool Reseau::InitSimuTraficMeso()
{
    m_dbInstSimuMeso= DBL_MAX;
    m_pCurrentProcessedNode= NULL;

    std::deque<Connexion* >::iterator itConn;
    std::deque<Tuyau *>::iterator itTuy;
    for( itConn = this->m_LstUserCnxs.begin(); itConn != this->m_LstUserCnxs.end(); itConn++)
    {
        Connexion *pConn = *itConn;

        ChangeMesoNodeEventTime(pConn, DBL_MAX, NULL);
        // Initialisation of Downstream Information
        for(itTuy = pConn->m_LstTuyAv.begin(); itTuy != pConn->m_LstTuyAv.end() ; ++itTuy)
        {
             if( dynamic_cast<CTuyauMeso* >( (*itTuy) ))
            {
                pConn->SetNextSupplyTime( (CTuyauMeso* ) (*itTuy), -DBL_MAX);
                pConn->SetRankNextIncomingVeh(*itTuy, 0);
            }

        }

        // Initialisation of Upstream Information
        for( itTuy =pConn->m_LstTuyAm.begin() ; itTuy != pConn->m_LstTuyAm.end(); itTuy++)
        {
            if( dynamic_cast<CTuyauMeso* >( (*itTuy) ))
            {
                pConn->ClearArrivals((CTuyauMeso* )(*itTuy) );
                pConn->SetNextArrival((CTuyauMeso* )(*itTuy), std::pair<double, Vehicule*>( DBL_MAX, nullptr) );
                pConn->SetRankNextIncomingVeh(*itTuy,0);
            }


        }
        // if the node is an entry
        if( pConn->m_LstTuyAm.size() ==0)
        {
            pConn->ClearArrivals(NULL);
            pConn->SetRankNextIncomingVeh(NULL,0);

        }
    }
    return true;

}
//================================================================
    bool Reseau::InitSimuTrafic
//----------------------------------------------------------------
// Fonction  : Initialisation de la simulation du trafic
// Version du: 13/07/2007
// Historique: 13/07/2007 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//================================================================
(
    bool bLastReplication
 )
{
    string          sOutputFile;
    bool            bOk;
    Tuyau           *pT;
    std::string	    ssBrique;
    double          dbPerAgrCpt;
    Point           ptCoord;
    std::string	    sId;
    Entree          *pE;
    Sortie          *pS;
    std::deque< TimeVariation<TraceDocTrafic> >::iterator itXmlDocTrafic;

    bOk = true;

    //initialisation du reseau
    InitLogFicSimulation();



    // RÃ©cupÃ©ration de la version de la dll
    string ssVer = SystemUtil::GetFileVersion();

    // Heure de dÃ©but et fin de la simulation
    SDateTime dtDeb;
    if(m_dtDateSimulation.GetYear() != 0)
    {
        dtDeb = m_dtDateSimulation;
    }
    else
    {
        dtDeb = SDateTime::Now();
    }
    SDateTime dtFin = dtDeb;
    dtDeb.m_hour = hd;
    dtDeb.m_minute = md;
    dtDeb.m_second = sd;
    dtFin.m_hour = hf;
    dtFin.m_minute = mf;
    dtFin.m_second = sf;

    // Initialisation des processus alÃ©atoires
    unsigned int uiInit;

    if( m_uiSeed == 0)
    {
        if(!m_bPickedSeed)
        {
            m_uiRandomSeed = (unsigned)time( NULL );
            m_bPickedSeed = true;
        }
        uiInit = m_uiRandomSeed;
    }
    else
        uiInit = m_uiSeed;
    m_pRandManager->mySrand(uiInit);
    log() << "seed : " << uiInit;

    if(m_pGestionsCapteur)   // PÃ©riode d'agrÃ©gation des capteurs
        dbPerAgrCpt = m_pGestionsCapteur->GetPeriodeAgregationPonctuels();
    else
        dbPerAgrCpt = 0;

    // Initialisation du document XML de trafic
    // ANOMALIE nÂ°80 : fuite mÃ©moire en cas de SymReset : m_XmlDocTrafic existe alors dÃ©jÃ 
    // pour chaque plage d'extraction on crÃ©Ã© un fichier de rÃ©sultat trafic
    for( itXmlDocTrafic = m_xmlDocTrafics.begin(); itXmlDocTrafic != m_xmlDocTrafics.end() ; ++itXmlDocTrafic)
    {
        // Initialisation des objets XML
        std::string ssFile = GetPrefixOutputFiles();
        std::string name = itXmlDocTrafic->m_pPlage->m_ID;

        name.erase(  std::remove( name.begin(),  name.end(), ':') ,name.end());

        ssFile = ssFile + "_" + name + "_traf" + GetSuffixOutputFiles() + ".xml";
        bool bEnableXML = (m_SymMode == Reseau::SYM_MODE_STEP_XML) || (m_SymMode == Reseau::SYM_MODE_FULL) || IsXmlOutput();
        TraceDocTrafic * docTrafic = new TraceDocTrafic(this, bEnableXML, IsXmlOutput(), m_bDebug, m_bTraceStocks, m_bSortieLight, m_bSortieRegime, bEnableXML?m_pXMLUtil->CreateXMLDocument(XS("OUT")):NULL,
            m_SymMode == Reseau::SYM_MODE_STEP_EVE, m_bGMLOutput, m_SymMode == Reseau::SYM_MODE_STEP_JSON);

        docTrafic->Init(ssFile, ssVer, dtDeb, dtFin, dbPerAgrCpt, m_XMLDocData, uiInit, m_SimulationID, m_TraficID, m_ReseauID, GetBoundingRect(), NULL, m_pCoordTransf, !Liste_ControleursDeFeux.empty(), GetGestionsCapteur());
        itXmlDocTrafic->m_pData.reset(docTrafic);
    } // rof each extraction range

    // Ajout des tronÃ§ons gÃ©rÃ©s par Symubruit (dont les info utiles pour l'outils d'analyse)
    for(int i=0; i<(int)m_LstTuyaux.size();i++)
    {
        pT = (Tuyau*)m_LstTuyaux[i];

        if(pT->GetBriqueParente())
            ssBrique = pT->GetBriqueParente()->GetID();
        else
            ssBrique = "";

        double dbStockMax = UNDEF_STOCK;
        if(m_bTraceStocks) {
            dbStockMax = pT->GetCapaciteStationnement();
        }

        for( itXmlDocTrafic = m_xmlDocTrafics.begin(); itXmlDocTrafic!= m_xmlDocTrafics.end(); itXmlDocTrafic++ )
        {
            itXmlDocTrafic->m_pData->AddTroncon(pT->GetLabel(), pT->GetExtAmont(), pT->GetExtAval(), pT->getNbVoiesDis(), ssBrique, pT->GetLstPtsInternes(), pT->GetLength(), pT->getLargeursVoies(), dbStockMax, pT->GetStockStationnement(), pT);
        }
        if(pT->getNbVoiesDis()>1)
        {
            for(int nV = 0; nV < pT->getNbVoiesDis(); nV++)
            {
                Voie *pV = pT->GetLstLanes()[nV];
                for( itXmlDocTrafic = m_xmlDocTrafics.begin(); itXmlDocTrafic!= m_xmlDocTrafics.end(); itXmlDocTrafic++ )
                {
                    itXmlDocTrafic->m_pData->AddVoie( pV->GetExtAmont(), pV->GetExtAval(), pV->GetLstPtsInternes(), pV->GetLength() );
                }
            }
        }
    }

    // Ajout des connexions (dont les info utiles pour l'outils d'analyse)
    for(size_t i=0; i<Liste_entree.size();i++)
    {
        pE = (Entree*)Liste_entree[i];
        if(pE->m_LstTuyAv.front())     // Sinon, non utilisÃ©
        {
            sId = pE->GetOutputID();
            CalcCoordFromPos( pE->m_LstTuyAv.front(), 0, ptCoord);
            for( itXmlDocTrafic = m_xmlDocTrafics.begin(); itXmlDocTrafic!= m_xmlDocTrafics.end(); itXmlDocTrafic++ )
            {
                itXmlDocTrafic->m_pData->AddDefEntree(sId, ptCoord);
            }
        }
    }
    for(size_t i=0; i<Liste_sortie.size();i++)
    {
        pS = (Sortie*)Liste_sortie[i];
        if(pS->GetNbElAmont()>0)
        {
            if(pS->m_LstTuyAm.front())     // Sinon, non utilisÃ©
            {
                sId = pS->GetInputID();
                CalcCoordFromPos( pS->m_LstTuyAm.front(), pS->m_LstTuyAm.front()->GetLength(), ptCoord);
                for( itXmlDocTrafic = m_xmlDocTrafics.begin(); itXmlDocTrafic!= m_xmlDocTrafics.end(); itXmlDocTrafic++ )
                {
                    itXmlDocTrafic->m_pData->AddDefSortie(sId, ptCoord);
                }
            }
        }
    }
    for(size_t i = 0; i < Liste_carrefoursAFeux.size(); i++)
    {
        for( itXmlDocTrafic = m_xmlDocTrafics.begin(); itXmlDocTrafic!= m_xmlDocTrafics.end(); itXmlDocTrafic++ )
        {
            itXmlDocTrafic->m_pData->AddDefCAF(Liste_carrefoursAFeux[i]);
        }
    }

    for(size_t i = 0; i < Liste_giratoires.size(); i++)
    {
        for( itXmlDocTrafic = m_xmlDocTrafics.begin(); itXmlDocTrafic!= m_xmlDocTrafics.end(); itXmlDocTrafic++ )
        {
            itXmlDocTrafic->m_pData->AddDefGir(Liste_giratoires[i]);
        }
    }
    for(size_t i = 0; i < Liste_ControleursDeFeux.size(); i++)
    {
        for( itXmlDocTrafic = m_xmlDocTrafics.begin(); itXmlDocTrafic!= m_xmlDocTrafics.end(); itXmlDocTrafic++ )
        {
            itXmlDocTrafic->m_pData->AddDefCDF(Liste_ControleursDeFeux[i]);
        }
    }


    // Ajout des capteurs gÃ©rÃ©s par Symubruit (dont les info utiles pour l'outils d'analyse)
    if(m_pGestionsCapteur)
    {
        std::vector<AbstractSensor*> allSensors = m_pGestionsCapteur->GetAllSensors();
        for( itXmlDocTrafic = m_xmlDocTrafics.begin(); itXmlDocTrafic!= m_xmlDocTrafics.end(); itXmlDocTrafic++ )
        {
            for(size_t i=0; i<allSensors.size();i++)
            {
                allSensors[i]->WriteDef(itXmlDocTrafic->m_pData.get());
            }
        }
    }

    std::deque <ConnectionPonctuel*>::iterator cc, cd, cf, cn, cci, cdi, cfi, ssc, ssd, ssf;
    std::deque <Sortie*>::iterator sc, sd, sf;
    std::deque<Tuyau*> LstTuyRes;

    //Ouverture des fichiers sorties
    if(m_bFichierSAS)
    {
        sOutputFile = m_sPrefixOutputFiles + "_SAS" + m_sSuffixOutputFiles + ".txt";
        FicTraficSAS->open(sOutputFile.c_str());
    }

  // Evolution nÂ°62 : ajout de la production de fichiers CSV
  // seulement si trace vaut true
  if(IsXmlOutput() && m_bCSVOutput)
  {
      m_CSVOutputWriter = new CSVOutputWriter(m_sPrefixOutputFiles, m_sSuffixOutputFiles, dtDeb, m_pCoordTransf, m_bCSVTrajectories, m_bCSVSensors);
      m_CSVOutputWriter->writeLinksFile(m_LstTuyaux);
      if(m_pGestionsCapteur)
      {
          std::vector<PonctualSensor*> ponctSensors;
          for(size_t i = 0; i < m_pGestionsCapteur->GetCapteursPonctuels().size(); i++)
          {
              ponctSensors.push_back((PonctualSensor*)m_pGestionsCapteur->GetCapteursPonctuels()[i]);
          }
          m_CSVOutputWriter->writeSensorsFile(ponctSensors);
      }
  }


  // ajout de la production du fichier JSON des temps de parcours
  if (m_bTravelTimesOutput)
  {
      m_pTravelTimesOutputManager = new TravelTimesOutputManager(this);
  }

  // Initialisation de l'instant calculÃ©
  m_dbInstSimu = 0;

  m_nInstSim = 0;

  // Initialisation de l'identifiant du premier vÃ©hicule
  m_nLastVehicleID = 0;

  // Initialisation du cumul du nombre de vÃ©hicules
  m_nNbVehCum = 0;

  //parcours des tuyaux
  std::deque <Tuyau*>::iterator tcourant;
  std::deque <Tuyau*>::iterator tdebut = m_LstTuyaux.begin();
  std::deque <Tuyau*>::iterator tfin = m_LstTuyaux.end();

  for (tcourant=tdebut;tcourant!=tfin;++tcourant)
   {
     if( !(*tcourant)->InitSimulation(false,false,"") )
     {
         log()<<std::endl<<" Problem detected for link "<<(*tcourant)->GetLabel();
         bOk = false;
     }
   }

    // Initialisation des connexions dÃ©finies par l'utilisateur
    std::deque<Connexion*>::iterator itCnx;
    for(itCnx = m_LstUserCnxs.begin(); itCnx != m_LstUserCnxs.end(); ++itCnx)
        (*itCnx)->InitSimuTrafic();

    //Parcours des convergents
    std::map<std::string, Convergent*>::iterator itCvg;
    for (itCvg=Liste_convergents.begin(); itCvg!=Liste_convergents.end(); ++itCvg)
    {
        itCvg->second->InitSimuTrafic();
    }

    //Parcours des repartiteurs
    std::map<std::string, Repartiteur*>::iterator rep;
    std::map<std::string, Repartiteur*>::iterator rep_d = Liste_repartiteurs.begin();
    std::map<std::string, Repartiteur*>::iterator rep_f = Liste_repartiteurs.end();

    for (rep=rep_d; rep!=rep_f; ++rep)
    {
        ((Repartiteur *)rep->second)->InitSimuTrafic();
    }

    // Initilisation des variables de simulation des CDF
    std::deque <ControleurDeFeux*>::iterator itCDF;
    for (itCDF=Liste_ControleursDeFeux.begin(); itCDF!=Liste_ControleursDeFeux.end(); ++itCDF)
    {
        *(*itCDF)->GetLstTrafficLightCycles() = *(*itCDF)->GetLstPlanDeFeuxInit();
        (*itCDF)->SetDureeRougeDegagement( (*itCDF)->GetDureeRougeDegagementInit() );
        (*itCDF)->SetDureeVertMin( (*itCDF)->GetDureeVertMinInit() );
    }

    // Initialisation des variables de simulation des diffÃ©rentes flottes
    for(size_t i = 0; i < m_LstFleets.size(); i++)
    {
        m_LstFleets[i]->InitSimuTrafic(m_xmlDocTrafics);
    }

    // Post Init des fichiers rÃ©sultats
    for( itXmlDocTrafic = m_xmlDocTrafics.begin(); itXmlDocTrafic!= m_xmlDocTrafics.end(); itXmlDocTrafic++ )
    {
        itXmlDocTrafic->m_pData->PostInit();
    }

    if( bOk)
        log()<<" ok"<<std::endl;
    else
        log()<<std::endl;

  //Ecrit l'entete dans les fichiers de sortie
    log()<<"Writing .txt files"<<std::endl;

//     FicTraficCel<<std::endl<<"*** Description de la simulation ***"<<std::endl<<std::endl;
     if(m_bFichierSAS)
     {
        *FicTraficSAS<<"*** Simulation description ***"<<std::endl<<std::endl;
     }



    // GÃ©nÃ©ration de noeuds du fichier de sortie XML
    if(m_pGestionsCapteur)
        m_pGestionsCapteur->Init();


    // Initialisation des variables de log
    if(m_bDebugOD)
    {
        std::deque <SymuViaTripNode*>::iterator origine;
        std::deque <SymuViaTripNode*>::iterator origine_d = Liste_origines.begin();
        std::deque <SymuViaTripNode*>::iterator origine_f = Liste_origines.end();
        for (origine=origine_d; origine!=origine_f; origine++)
        {
            if( !IsUsedODMatrix() )
            {
                std::deque <SymuViaTripNode*>::iterator sor;
                std::deque <SymuViaTripNode*>::iterator sor_d = Liste_destinations.begin();
                std::deque <SymuViaTripNode*>::iterator sor_f = Liste_destinations.end();
                for (sor= sor_d; sor!=sor_f; sor++)
                    (*origine)->AddDestinations( *sor );
            }
            (*origine)->InitLogMatriceOD( IsUsedODMatrix() );
        }

        // fichier de sortie des log qui s'enregistre dans le repertoire 'out'
        SDateTime	tNow;
        tNow = SDateTime::Now();

        if( m_bLogFile)
        {
            string sLogFile;
            sLogFile = m_sPrefixOutputFiles + "_Debug" + m_sSuffixOutputFiles + ".log";
            g_FicDebug->open(sLogFile.c_str());
            *g_FicDebug<<tNow.GetDay()<<"/"<<tNow.GetMonth()<<"/"<<tNow.GetYear()<<std::endl;  // inscrit l'heure et la date
            *g_FicDebug<<tNow.GetHour()<<":"<<tNow.GetMinute()<<":"<<tNow.GetSecond()<<std::endl<<std::endl;
        }
    }

    // Destruction de la liste des vÃ©hicules
    std::deque <boost::shared_ptr<Vehicule>>::iterator ItCurVeh;
	if(m_LstVehicles.size()>0)
		m_LstVehicles.clear();

    // rmq. : positionnÃ© Ã  true avant les calculs d'itiÃ©nraires car utilisÃ© pour savoir si les matrices OD ont Ã©tÃ© copiÃ©es des listes "Init" aux listes de travail....
    m_bInitSimuTrafic = true;

    // Calcul des itinÃ©raires
    if (m_pModuleAffectation != NULL && !IsSymuMasterMode())
    {
        m_pModuleAffectation->InitSaveAffectation(m_bSaveAffectation, this);
        m_pModuleAffectation->Run( this, m_dbInstSimu, 'P', false);
    }

    // Copie des vÃ©hicules de la liste initiale dans la liste dynamique des vÃ©hicules de la simulation
    for (ItCurVeh=m_LstInitVehicule.begin();ItCurVeh!=m_LstInitVehicule.end();ItCurVeh++)
    {
        boost::shared_ptr<Vehicule> pInitVeh = *ItCurVeh;
        boost::shared_ptr<Vehicule> pVeh = boost::make_shared<Vehicule>(this, pInitVeh->GetID(), 0, pInitVeh->GetType(), pas_de_temps);

        // Copie des caractÃ©ristiques
        pInitVeh->CopyTo(pVeh);
        GetSymuViaFleet()->OnVehicleActivated(pVeh, m_dbInstSimu);
        AddVehicule(pVeh);
        m_nLastVehicleID = max(m_nLastVehicleID, pVeh->GetID()+1);

        // Calcul des itinÃ©raires des vÃ©hicules dÃ©jÃ  prÃ©sent sur le rÃ©seau Ã  l'instant 0
        if(IsCptItineraire())
        {
            // Cas particulier origine == destination qui fait boucler Ã  l'infini dijkstra
            if(pVeh->GetLink(0)->GetCnxAssAv() == ((SymuViaTripNode*)pVeh->GetDestination())->GetInputConnexion())
            {
                std::vector<Tuyau*> iti;
                iti.push_back(pVeh->GetLink(0));
                pVeh->GetTrip()->ChangeRemainingPath(iti, pVeh.get());
            }
            else
            {
                // Calcul du plus court chemin
                std::vector<PathResult> newPaths;
                std::map<std::vector<Tuyau*>, double> placeHolder;
                GetSymuScript()->ComputePaths(pVeh->GetLink(0)->GetCnxAssAv(), (SymuViaTripNode*)pVeh->GetDestination(), pVeh->GetType(), m_dbInstSimu, 1, pVeh->GetLink(0), newPaths, placeHolder, false);

                if(newPaths.size() == 1)
                {
                    // Reconstruction de l'itinÃ©raire
                    std::vector<Tuyau*> newIti;
                    newIti.push_back(pVeh->GetLink(0));
                    newIti.insert(newIti.end(), newPaths.front().links.begin(),newPaths.front().links.end());
                    pVeh->GetTrip()->ChangeRemainingPath(newIti, pVeh.get());
                }
                else
                {
                    log() << "Failure computing path for vehicle ID=" << pVeh->GetID() << " on the network at the start of the simulation." << std::endl;
                }
            }
        }

        pVeh->SetVitRegTuyAct(pInitVeh->GetVitRegTuyAct() );

        // Ajout fichier trafic XML liste des vÃ©hicules
        std::vector<std::string> initialPath;
        for(size_t iTuy = 0; iTuy < pVeh->GetItineraire()->size(); iTuy++)
        {
            initialPath.push_back(pVeh->GetItineraire()->at(iTuy)->GetLabel());
        }

        //ETS  140929
        SymuViaFleetParameters * pFleetParams = dynamic_cast<SymuViaFleetParameters*>(pVeh->GetFleetParameters());
        AddVehiculeToDocTrafic(pVeh->GetID(), "", pVeh->GetType()->GetLabel(), pVeh->GetType()->GetGMLLabel(), pVeh->GetType()->GetKx(), pVeh->GetVitMax(), pVeh->GetType()->GetW(), "", pVeh->GetDestination()?pVeh->GetDestination()->GetInputID():"", "", "", pFleetParams?pFleetParams->GetRouteId():"", 0, "", pVeh->IsAgressif(), pVeh->GetVoie(0)->GetNum(), "", initialPath, pVeh->GetType()->GetLstPlagesAcceleration(), "", "");

        //GetXmlDocTrafic()->AddVehicule(pVeh->GetID(), "", ssType, pVeh->GetType()->GetKx(), pVeh->GetVitMax(), pVeh->GetType()->GetW(), "", pVeh->GetDestination()?pVeh->GetDestination()->GetDestinationID():"", 0,  "", pVeh->IsAgressif());
    }

    // TEST.................................
    //GenReseauCirculationFile("ReseauCirculation.xml");
    //......................................

    // DÃ©chargement du fichier d'entrÃ©e (sauf en mode d'affectation convergente)
    // Et sauf si plusieurs rÃ©plications ! on ne le ferme qu'Ã  la derniÃ¨re rÃ©plication
    if (m_XMLDocData && (!m_pModuleAffectation || !m_pModuleAffectation->IsConvergent()) && bLastReplication)
    {
        m_pXMLUtil->CleanLoadDocument(m_XMLDocData);
        m_XMLDocData = NULL;
    }

    // intialisations liÃ©es au mode meso
	if(m_bMeso)
		InitSimuTraficMeso();

    m_initTime = clock();

    if (m_strFileToLoadState != "") {
        SerializeUtil::Load(this, (char*)m_strFileToLoadState.c_str());
        log() << std::endl << "SymuVia loaded from " << m_strFileToLoadState << std::endl;
    }

    return true;
}


//================================================================
    bool Reseau::InitSimuEmissions
//----------------------------------------------------------------
// Fonction  : Initailsiation de la simulation acoustique
// Version du: 13/07/2007
// Historique: 13/07/2007 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//================================================================
(
    bool bAcoustic,
    bool bAir,
    bool bSirane,
    bool bOnlyCells /* = false */   // = vrai si uniquement les cellules acoustiques sont calculÃ©es
 )
{
    bool  bOk;
    bOk = true;

    InitLogFicSimulation();

    if(bAcoustic)
    {
        // Initialisation de l'objet LoiEmission
        Loi_emis= new Loi_emission(this);
        if(Loi_emis)
            if(!Loi_emis->LoadDatabase())
                return false;
    }

    // Initialisation des objets XML en entrÃ©e
    std::string ssFile = m_sPrefixOutputFiles;
    // remarque : seule la premiÃ¨re plage temporelle est traitÃ©e... Ã  reprendre si on veut traiter l'ensemble des plages ?
    if(m_extractionRange.size() > 1)
    {
        log()<< "The emissions simulation on multiple time frames in not implemented. Only the first time frame will be processed..."<<std::endl;
    }
    std::string name = (*m_extractionRange.begin())->m_ID;
    name.erase(  std::remove( name.begin(),  name.end(), ':') ,name.end());
    ssFile = ssFile + "_" + name + "_traf" + m_sSuffixOutputFiles + ".xml";

    m_XmlReaderTrafic = new XMLReaderTrafic(m_pLogger);

    std::string sTimeET;

    if ((m_SymMode == Reseau::SYM_MODE_FULL) && (!bOnlyCells))
    {
        log()<< "XML input file initialization..."<<std::endl;

        // VÃ©rification de l'existence du fichier XML de trafic
        if (!SystemUtil::FileExists(ssFile))
        {
            delete(Loi_emis);
            Loi_emis = NULL;
            log() << Logger::Error << "ERROR : The traffic XML file was not found ( "<< ssFile <<" )" <<std::endl;
            log() << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
            return false;
        }

        // Initialisation des curseurs de lecture du fichier XML de trafic
        if(!m_XmlReaderTrafic->Init(ssFile, sTimeET))
        {
            delete(Loi_emis);
            Loi_emis = NULL;
            return false;
        }
        log()<< "ok" << std::endl;
    }

    if(bAcoustic)
    {
        switch(m_SymMode)
        {
        case Reseau::SYM_MODE_FULL:
        case Reseau::SYM_MODE_STEP_XML:
            {
                // Initialisation des objets XML en sortie
                log()<< "Output XML acoustic file initialization...";
                std::string ssFileOut = m_sPrefixOutputFiles;

                if(!bOnlyCells)
                    ssFileOut += "_emi" + m_sSuffixOutputFiles + ".xml";
                else
                    ssFileOut += "_cell" + m_sSuffixOutputFiles + ".xml";

                std::string ssVersionDB = Loi_emis->GetVersion();

                // RÃ©cupÃ©ration de la version de la dll
                string ssVer = SystemUtil::GetFileVersion();

                STime dtDebut(hd, md, sd);

                XMLDocAcoustique * XmlDocAcoustique;
                m_XmlDocAcoustique = new XMLDocAcoustique(this, m_pXMLUtil->CreateXMLDocument(XS("SIMULATION")));
                XmlDocAcoustique = (XMLDocAcoustique *)m_XmlDocAcoustique;
                XmlDocAcoustique->setSave(m_bXmlOutput);

                SDate ssDate(m_dtDateSimulation.GetDay(), m_dtDateSimulation.GetMonth(), m_dtDateSimulation.GetYear());

                XmlDocAcoustique->Init(ssFileOut, dtDebut, ssVer, sTimeET, ssVersionDB, m_strTitre, ssDate.ToString());
            }
            break;
        case Reseau::SYM_MODE_STEP_EVE:
            if (IsXmlOutput())
            {
                // Initialisation des objets XML en sortie
                log()<< "Output XML acoustic file initialization...";
                std::string ssFileOut = m_sPrefixOutputFiles;

                if(!bOnlyCells)
                    ssFileOut += "_emi" + m_sSuffixOutputFiles + ".xml";
                else
                    ssFileOut += "_cell" + m_sSuffixOutputFiles + ".xml";

                std::string ssVersionDB = Loi_emis->GetVersion();

                // RÃ©cupÃ©ration de la version de la dll
                string ssVer = SystemUtil::GetFileVersion();

                STime dtDebut(hd, md, sd);

                SDate ssDate(m_dtDateSimulation.GetDay(), m_dtDateSimulation.GetMonth(), m_dtDateSimulation.GetYear());

                TraceDocAcoustique * tDocAcoustique;
                m_XmlDocAcoustique = new TraceDocAcoustique(this, m_pXMLUtil->CreateXMLDocument(XS("SIMULATION")));
                tDocAcoustique = (TraceDocAcoustique*)m_XmlDocAcoustique;
                tDocAcoustique->setSave(true);
                tDocAcoustique->Init(ssFileOut, dtDebut, ssVer, sTimeET, ssVersionDB, m_strTitre, ssDate.ToString());
            }
            else
            {
                EVEDocAcoustique * EveDocAcoustique;
                m_XmlDocAcoustique = new EVEDocAcoustique();
                EveDocAcoustique = (EVEDocAcoustique*)m_XmlDocAcoustique;
            }
            break;
            case Reseau::SYM_MODE_STEP_JSON:
                break;
        }

    }

    if(bAir)
    {
        // Initialisation des objets XML en sortie
        log()<< "Output atmospheric emissions XML file initialization...";
        std::string ssFileOut = m_sPrefixOutputFiles;

        ssFileOut += "_atm" + m_sSuffixOutputFiles + ".xml";

        // RÃ©cupÃ©ration de la version de la dll
        string ssVer = SystemUtil::GetFileVersion();

        STime dtDebut(hd, md, sd);

        m_XmlDocAtmo = new XMLDocEmissionsAtmo(this, m_pXMLUtil->CreateXMLDocument(XS("SIMULATION")));

        SDate ssDate(m_dtDateSimulation.GetDay(), m_dtDateSimulation.GetMonth(), m_dtDateSimulation.GetYear());

        m_XmlDocAtmo->Init(ssFileOut, dtDebut, ssVer, "", "", m_strTitre, ssDate.ToString().c_str());

        // Initialisation des compteurs
        m_dbCptCumCO2 = 0;
        m_dbCptCumNOx = 0;
        m_dbCptCumPM = 0;
    }

    if(bSirane)
    {
        // Initialisation des objets XML en sortie
        log()<< "Output Sirane XML file initialization...";
        std::string ssFileOut = m_sPrefixOutputFiles;

        ssFileOut += "_cit" + m_sSuffixOutputFiles + ".xml";

        // RÃ©cupÃ©ration de la version de la dll
        string ssVer = SystemUtil::GetFileVersion();

        STime dtDebut(hd, md, sd);

        m_XmlDocSirane = new XMLDocSirane(this, m_pXMLUtil->CreateXMLDocument(XS("FeatureCollection"), XS("ogr")));

        SDate ssDate(m_dtDateSimulation.GetDay(), m_dtDateSimulation.GetMonth(), m_dtDateSimulation.GetYear());

        m_XmlDocSirane->Init(ssFileOut, GetBoundingRect());
    }
    log()<< "ok" << std::endl;

    m_dbInstSimu = 0;
    m_dbInstSimuMeso= DBL_MAX;

    //parcours des tuyaux
    std::deque <Tuyau*>::iterator tcourant;
    std::deque <Tuyau*>::iterator tdebut = m_LstTuyaux.begin();
    std::deque <Tuyau*>::iterator tfin = m_LstTuyaux.end();

    for (tcourant=tdebut;tcourant!=tfin;tcourant++)
    {
     if( !(*tcourant)->InitSimulation(bAcoustic,bSirane,"") )
     {
         log()<<std::endl<<" Problem detected for link "<<(*tcourant)->GetLabel();
         bOk = false;
     }
    }

    if(bSirane)
    {
        //parcours des briques de connexion
        BriqueDeConnexion* pBrique;
        for(size_t briqueIdx = 0; briqueIdx < Liste_carrefoursAFeux.size(); briqueIdx++)
        {
            pBrique = Liste_carrefoursAFeux[briqueIdx];
            pBrique->InitSimulationSirane();
        }
        for(size_t briqueIdx = 0; briqueIdx < Liste_giratoires.size(); briqueIdx++)
        {
            pBrique = Liste_giratoires[briqueIdx];
            pBrique->InitSimulationSirane();
        }
    }


    if( bOk)
        log()<<" ok"<<std::endl;
    else
        log()<<std::endl;

    m_nInstSim = 0;

    m_nLastIdSegment = 0;

    // Remise Ã  zÃ©ro des variables de simulation
    std::deque <TuyauMacro*>::iterator ItCurTuy;
    for( ItCurTuy = m_LstTuyauxMacro.begin(); ItCurTuy != m_LstTuyauxMacro.end(); ItCurTuy++)
    {
       (*ItCurTuy)->InitVarSimu();
    }

    m_bInitSimuEmissions = true;

    return true;
}

//=================================================================
    bool Reseau::LoadTrafic
//----------------------------------------------------------------
// Fonction  : Charge les variables de trafic du pas de temps courant
// Remarque  :
// Version du: 13/07/2007
// Historique: 13/07/2007 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//=================================================================
(
 )
{
    int nIDVeh;
    int nIDCell;
    std::vector <boost::shared_ptr<Vehicule>>::iterator ItCurVeh;
    std::vector <boost::shared_ptr<Vehicule>>::iterator ItDebVeh;
    std::vector <boost::shared_ptr<Vehicule>>::iterator ItFinVeh;
    boost::shared_ptr<Vehicule>	pVehicule;
    std::deque <TypeVehicule*>::iterator ItCurTypeVeh;
    std::deque <TypeVehicule*>::iterator ItDebTypeVeh;
    std::deque <TypeVehicule*>::iterator ItFinTypeVeh;
    std::deque <TuyauMacro*>::iterator ItCurTuy;
    TypeVehicule	*pTypeVehicule;
    Tuyau			*pTuyau;
    Voie			*pVoie;
    std::string	    ssVal;

    // TRAITEMENT DES VEHICULES
    InitVehicules(0);    // MAJ de la liste des vÃ©hicules

    nIDVeh = m_XmlReaderTrafic->ReadNextTrajectoire(m_dbInstSimu, -1);

    // Lecture des donnÃ©es des vÃ©hicules
    m_XmlReaderTrafic->ReadVehicules();

    while( nIDVeh > -1)
    {
        // Recherche du vÃ©hicule dans la liste
        ItDebVeh = m_LstVehicles.begin();
        ItFinVeh = m_LstVehicles.end();
        pVehicule.reset();

        for (ItCurVeh=ItDebVeh;ItCurVeh!=ItFinVeh;ItCurVeh++)
        {
            if( (*ItCurVeh)->GetID() == nIDVeh)
            {
                pVehicule = (*ItCurVeh);
                break;
            }
        }
        // CrÃ©ation du vÃ©hicule
        if(!pVehicule)
        {
            // Attributs du vÃ©hicule
            VehTraficData & data = m_XmlReaderTrafic->GetAttrFromVehicule(SystemUtil::ToString(nIDVeh));
            // Recherche du type de vÃ©hicule
            std::string ssType = data.type;

            ItDebTypeVeh = m_LstTypesVehicule.begin();
            ItFinTypeVeh = m_LstTypesVehicule.end();
            pTypeVehicule = NULL;
            for (ItCurTypeVeh=ItDebTypeVeh;ItCurTypeVeh!=ItFinTypeVeh;ItCurTypeVeh++)
            {
                if( (*ItCurTypeVeh)->GetLabel() == ssType)
                {
                    pTypeVehicule = (*ItCurTypeVeh);
                    break;
                }
            }
            if(!pTypeVehicule)
            {
                nIDVeh = m_XmlReaderTrafic->ReadNextTrajectoire(m_dbInstSimu, nIDVeh);
                continue;
            }

            pVehicule = boost::make_shared<Vehicule>(this, nIDVeh, 0, pTypeVehicule, pas_de_temps);

            // Chargement des caractÃ©ristiques du vÃ©hicule
            Trip * pTrip = new Trip();
            char cOrigineType;
            pTrip->SetOrigin( this->GetOrigineFromID(data.entree, cOrigineType) );
            TripLeg * pTripLeg = new TripLeg();
            char cDestType;
            pTripLeg->SetDestination( this->GetDestinationFromID(data.sortie, cDestType) );
            pTrip->AddTripLeg(pTripLeg);
            pVehicule->SetTrip(pTrip);
            pVehicule->MoveToNextLeg();

            pVehicule->SetInstantCreation( SystemUtil::ToDouble(data.instC) );
            pVehicule->SetInstantEntree( SystemUtil::ToDouble(data.instE) );
            pVehicule->SetInstantSortie( SystemUtil::ToDouble(data.instS) );

            pVehicule->SetDstCumulee( SystemUtil::ToDouble(data.dstParcourue) );

            // Ajout Ã  la liste des vÃ©hicules
            GetSymuViaFleet()->OnVehicleActivated(pVehicule, pVehicule->GetInstantEntree());
            AddVehicule(pVehicule);
        }
        // MAJ des variables de simulation

        // Tuyau
        ssVal = m_XmlReaderTrafic->GetValueAttrFromTrajectoire("tron");
        pTuyau = GetLinkFromLabel( ssVal );

        if(!pTuyau)
        {
            nIDVeh = m_XmlReaderTrafic->ReadNextTrajectoire(m_dbInstSimu, nIDVeh);
            continue;
        }
            //break;		// Le vÃ©hicule n'est pas considÃ©rÃ©

        // Voie
        ssVal = m_XmlReaderTrafic->GetValueAttrFromTrajectoire("voie");
        if(pTuyau->getNb_voies() < atoi(ssVal.c_str()) )
            {
            nIDVeh = m_XmlReaderTrafic->ReadNextTrajectoire(m_dbInstSimu, nIDVeh);
            continue;
        }
            //break;		// Le vÃ©hicule n'est pas considÃ©rÃ©
        pVoie = pTuyau->GetLstLanes()[atoi(ssVal.c_str())-1];

        pVehicule->SetTuyau( (TuyauMicro*)pTuyau, 0);
        pVehicule->SetVoie( (VoieMicro*)pVoie);

        // Position
        pVehicule->SetPos( SystemUtil::ToDouble(m_XmlReaderTrafic->GetValueAttrFromTrajectoire("dst")) );
        // vitesse
        pVehicule->SetVit( SystemUtil::ToDouble(m_XmlReaderTrafic->GetValueAttrFromTrajectoire("vit")));
        // accÃ©lÃ©ration
        pVehicule->SetAcc( SystemUtil::ToDouble(m_XmlReaderTrafic->GetValueAttrFromTrajectoire("acc")));

        nIDVeh = m_XmlReaderTrafic->ReadNextTrajectoire(m_dbInstSimu, nIDVeh);

    } // Fin While

    // Suppression des vÃ©hicules sortis du rÃ©seau
    SupprimeVehicules(0);

    // TRAITEMENT DES CELLULES DES TUYAUX MACROSCOPIQUES
    SegmentMacro *pSegment;
    nIDCell = m_XmlReaderTrafic->ReadNextSimuCell(m_dbInstSimu, -1);
    while( nIDCell > -1)
    {
        pSegment = GetSegmentFromID( nIDCell );

        // DÃ©calage des variables de simulation
        if(!pSegment->getSegmentAmont())
            pSegment->GetFrontiereAmont()->DecalVarTrafic();
        pSegment->GetFrontiereAval()->DecalVarTrafic();

        // Copie des donnÃ©es de l'instant courant :

        // N
        if(!pSegment->getSegmentAmont())
            pSegment->GetFrontiereAmont()->SetN( atof( m_XmlReaderTrafic ->GetValueAttrFromSimuCell("NAm").c_str()));
        pSegment->GetFrontiereAval()->SetN( atof( m_XmlReaderTrafic ->GetValueAttrFromSimuCell("NAv").c_str()));

        // Vitesse
        if(!pSegment->getSegmentAmont())
            pSegment->GetFrontiereAmont()->SetVit( atof( m_XmlReaderTrafic ->GetValueAttrFromSimuCell("VitAm").c_str()));
        pSegment->GetFrontiereAval()->SetVit( atof( m_XmlReaderTrafic ->GetValueAttrFromSimuCell("VitAv").c_str()));

        // AccÃ©lÃ©ration
        if(!pSegment->getSegmentAmont())
            pSegment->GetFrontiereAmont()->SetAcc( atof( m_XmlReaderTrafic ->GetValueAttrFromSimuCell("AccAm").c_str()));
        pSegment->GetFrontiereAval()->SetAcc( atof( m_XmlReaderTrafic ->GetValueAttrFromSimuCell("AccAv").c_str()));

        nIDCell = m_XmlReaderTrafic->ReadNextSimuCell(m_dbInstSimu, nIDCell);
    }

    //delete ssVal;

    return true;
}


//=================================================================
void Reseau::SwitchMicroMeso()
//----------------------------------------------------------------
// Fonction  : Positionne la resolution des Ã©lÃ©ments du rÃ©seau
//             en fonction de la position des vÃ©hicules d'intÃ©rÃªt
// Remarque  :
// Version du: 26/07/2016
// Historique: 26/07/2016 (O.Tonck - Ipsis)
//             CrÃ©ation
//=================================================================
{
    // On dÃ©termine la liste des tronÃ§ons Ã  reprÃ©senter en micro (les autres sont en meso)
    std::vector <boost::shared_ptr<Vehicule>>::iterator ItCurVeh;
    std::vector <boost::shared_ptr<Vehicule>>::iterator ItDebVeh;
    std::vector <boost::shared_ptr<Vehicule>>::iterator ItFinVeh;

    ItDebVeh = m_LstVehicles.begin();
    ItFinVeh = m_LstVehicles.end();

    std::set<Tuyau*> lstLinksToMicro;

    std::vector<Tuyau*> * pIti = NULL;

    Tuyau * pCurrentTuyau = NULL;

    Tuyau * pLink = NULL;
    double dbDistance;

    for (ItCurVeh=ItDebVeh;ItCurVeh!=ItFinVeh;++ItCurVeh)
    {
        Vehicule * pVeh = (*ItCurVeh).get();
        if(std::find(m_MicroVehicleTypes.begin(), m_MicroVehicleTypes.end(), pVeh->GetType()) != m_MicroVehicleTypes.end())
        {
            // On prend le tuyau courant (ou le tuyau suivant si le tuyau courant est dans une brique de connexion)
            pCurrentTuyau = pVeh->GetCurrentTuyauMeso();
            if(pCurrentTuyau->GetBriqueParente())
            {
                pCurrentTuyau = pVeh->CalculNextTuyau(pCurrentTuyau, m_dbInstSimu);
            }
            if(pCurrentTuyau)
            {
                lstLinksToMicro.insert(pCurrentTuyau);

                // On ajoute les tuyaux amont et aval dans le respect de la distance amont et aval paramÃ©trÃ©e
                pIti = pVeh->GetItineraire();
                int currentTuyauIndex = pVeh->GetItineraireIndex(*pIti, isSameTuyauPredicate, pCurrentTuyau);

                // rmq. : on ignore les longueurs des tuyaux internes pour le calcul : Ã  affiner si nÃ©cessaire.
                // De mÃªme pour la position courante du vÃ©hicule sur le tronÃ§on courant.

                // On ajout les tuyaux sur les cÃ´tÃ©s du tuyau courant
                AppendSideLinks(pCurrentTuyau, lstLinksToMicro, true, true);

                // Tuyaux amont
                dbDistance = 0;
                for(int i = currentTuyauIndex-1; i >= 0 && dbDistance < m_dbUpstreamMicroDistance; i--)
                {
                    pLink = pIti->at(i);
                    lstLinksToMicro.insert(pLink);
                    dbDistance += pLink->GetLength();

                    AppendSideLinks(pLink, lstLinksToMicro, false, true);
                }

                // Tuyaux aval
                dbDistance = 0;
                for(int i = currentTuyauIndex+1; i < (int)pIti->size() && dbDistance < m_dbDownstreamMicroDistance; i++)
                {
                    pLink = pIti->at(i);
                    lstLinksToMicro.insert(pLink);
                    dbDistance += pLink->GetLength();

                    AppendSideLinks(pLink, lstLinksToMicro, true, false);
                }

            }
        }
    }

    // Passage en micro des tuyaux meso qui doivent l'Ãªtre
    CTuyauMeso * pTuyau;
    std::deque<CTuyauMeso*>::iterator iterMeso = m_LstTuyauxMeso.begin();
    while(iterMeso != m_LstTuyauxMeso.end())
    {
        pTuyau = *iterMeso;
        if(lstLinksToMicro.find(pTuyau) != lstLinksToMicro.end())
        {
            pTuyau->ToMicro();
            m_LstTuyauxMicro.push_back(pTuyau);
            iterMeso = m_LstTuyauxMeso.erase(iterMeso);
        }
        else
        {
            iterMeso++;
        }
    }

    // Passage en meso des tuyaux micro qui doivent l'Ãªtre
    std::deque<TuyauMicro*>::iterator iterMicro = m_LstTuyauxMicro.begin();
    while(iterMicro != m_LstTuyauxMicro.end())
    {
        pTuyau = dynamic_cast<CTuyauMeso*>(*iterMicro);
        if(pTuyau && lstLinksToMicro.find(pTuyau) == lstLinksToMicro.end())
        {
            pTuyau->ToMeso();
            m_LstTuyauxMeso.push_back(pTuyau);
            iterMicro = m_LstTuyauxMicro.erase(iterMicro);
        }
        else
        {
            iterMicro++;
        }
    }
}

void Reseau::AppendSideLinks(Tuyau * pLink, std::set<Tuyau*> & lstLinksToMicro,
                             bool bUpstream, bool bDownstream)
{
    Tuyau * pSideLink = NULL;
    Connexion * pCon = NULL;
    double dbSideDistance;

    if(m_dbSidesMicroDistance > 0)
    {
        // map des tuyaux sur les cÃ´tÃ©s et de la distance dÃ©jÃ  parcourue pour les atteindre
        std::map<Tuyau*, double> mapSideLinks;
        // ensemble des tuyaux dÃ©jÃ  traitÃ©s pour Ã©viter les boucles infinies
        std::map<Tuyau*, double> alreadyTreatedLinks;

        mapSideLinks[pLink] = 0;
        alreadyTreatedLinks[pLink] = 0;

        while(!mapSideLinks.empty())
        {
            pLink = mapSideLinks.begin()->first;
            dbSideDistance = mapSideLinks.begin()->second;

            if(dbSideDistance <= m_dbSidesMicroDistance)
            {
                if (bUpstream)
                {
                    pCon = pLink->GetCnxAssAm();
                    for (size_t iLink = 0; iLink < pCon->m_LstTuyAssAm.size(); iLink++)
                    {
                        pSideLink = pCon->m_LstTuyAssAm[iLink];
                        lstLinksToMicro.insert(pSideLink);

                        double dbWeight = dbSideDistance + pSideLink->GetLength();

                        if (dbWeight <= m_dbSidesMicroDistance)
                        {
                            std::map<Tuyau*, double>::const_iterator iterLink = alreadyTreatedLinks.find(pSideLink);
                            if (iterLink == alreadyTreatedLinks.end() || dbWeight < iterLink->second) // seconde condition pour gÃ©rer l'accÃ¨s Ã  un tuyau dÃ©jÃ  traitÃ© par un chein plus court
                            {
                                mapSideLinks[pSideLink] = dbWeight;
                                alreadyTreatedLinks[pSideLink] = dbWeight;
                            }
                        }
                    }
                    for (size_t iLink = 0; iLink < pCon->m_LstTuyAssAv.size(); iLink++)
                    {
                        pSideLink = pCon->m_LstTuyAssAv[iLink];
                        lstLinksToMicro.insert(pSideLink);

                        double dbWeight = dbSideDistance + pSideLink->GetLength();

                        if (dbWeight <= m_dbSidesMicroDistance)
                        {
                            std::map<Tuyau*, double>::const_iterator iterLink = alreadyTreatedLinks.find(pSideLink);
                            if (iterLink == alreadyTreatedLinks.end() || dbWeight < iterLink->second) // seconde condition pour gÃ©rer l'accÃ¨s Ã  un tuyau dÃ©jÃ  traitÃ© par un chein plus court
                            {
                                mapSideLinks[pSideLink] = dbWeight;
                                alreadyTreatedLinks[pSideLink] = dbWeight;
                            }
                        }
                    }
                }

                if (bDownstream)
                {
                    pCon = pLink->GetCnxAssAv();
                    for (size_t iLink = 0; iLink < pCon->m_LstTuyAssAm.size(); iLink++)
                    {
                        pSideLink = pCon->m_LstTuyAssAm[iLink];
                        lstLinksToMicro.insert(pSideLink);

                        double dbWeight = dbSideDistance + pSideLink->GetLength();

                        if (dbWeight <= m_dbSidesMicroDistance)
                        {
                            std::map<Tuyau*, double>::const_iterator iterLink = alreadyTreatedLinks.find(pSideLink);
                            if (iterLink == alreadyTreatedLinks.end() || dbWeight < iterLink->second) // seconde condition pour gÃ©rer l'accÃ¨s Ã  un tuyau dÃ©jÃ  traitÃ© par un chein plus court
                            {
                                mapSideLinks[pSideLink] = dbWeight;
                                alreadyTreatedLinks[pSideLink] = dbWeight;
                            }
                        }
                    }
                    for (size_t iLink = 0; iLink < pCon->m_LstTuyAssAv.size(); iLink++)
                    {
                        pSideLink = pCon->m_LstTuyAssAv[iLink];
                        lstLinksToMicro.insert(pSideLink);

                        double dbWeight = dbSideDistance + pSideLink->GetLength();

                        if (dbWeight <= m_dbSidesMicroDistance)
                        {
                            std::map<Tuyau*, double>::const_iterator iterLink = alreadyTreatedLinks.find(pSideLink);
                            if (iterLink == alreadyTreatedLinks.end() || dbWeight < iterLink->second) // seconde condition pour gÃ©rer l'accÃ¨s Ã  un tuyau dÃ©jÃ  traitÃ© par un chein plus court
                            {
                                mapSideLinks[pSideLink] = dbWeight;
                                alreadyTreatedLinks[pSideLink] = dbWeight;
                            }
                        }
                    }
                }
            }

            // Suppression de l'Ã©lÃ©ment traitÃ©
            mapSideLinks.erase(pLink);
        }
    }
}

//=================================================================
    bool Reseau::SimuTrafic
//----------------------------------------------------------------
// Fonction  : Lance le calcul du trafic au pas de temps courant
// Remarque  :
// Version du: 13/07/2007
// Historique: 13/07/2007 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//=================================================================
(
    bool & bStepCompleted
 )
{
    bStepCompleted = false;

	m_dbInstSimuMeso = DBL_MAX;

	if (m_bMeso)
	{
		m_pCurrentProcessedNode = m_mesoNodes.empty() ? NULL : m_mesoNodes.front();
		if (m_pCurrentProcessedNode)
		{
			m_dbInstSimuMeso = m_pCurrentProcessedNode->GetNextEventTime();
			// Si m_dbInstSimuMeso vaut DBL_MAX, ca peut venir d'un tronÃ§on meso
			// bloquÃ© car la tronÃ§on micro aval Ã©tait plein lors du dernier pas de temps meso.
			// Il faut donc appeler SimuTraficMeso dans ce cas pour mettre Ã  jour le supplytime du tronÃ§on anciennement bloquÃ©
			// et dÃ©bloquer la situation au prochain pas de temps.
			if (m_dbInstSimuMeso == DBL_MAX) {
				SimuTraficMeso();
				m_pCurrentProcessedNode = m_mesoNodes.empty() ? NULL : m_mesoNodes.front();
				if (m_pCurrentProcessedNode)
				{
					m_dbInstSimuMeso = m_pCurrentProcessedNode->GetNextEventTime();
				}
			}
		}
	}

    // gere le calcul du meso ou Micro/Macro
    if (m_dbInstSimu+ pas_de_temps <= m_dbInstSimuMeso+ 0.00001 )//m_pCurrentProcessedNode->GetNextEventTime() )
    {
        bStepCompleted = true;

        std::deque <TuyauMacro*>::iterator tmaccourant;
        std::deque <TuyauMacro*>::iterator tmacdebut;
        std::deque <TuyauMacro*>::iterator tmacfin;

        std::deque <CTuyauMeso*>::iterator tmescourant;
        std::deque <CTuyauMeso*>::iterator tmesdebut;
        std::deque <CTuyauMeso*>::iterator tmesfin;

        std::deque <TuyauMicro*>::iterator tmiccourant;
        std::deque <TuyauMicro*>::iterator tmicdebut;
        std::deque <TuyauMicro*>::iterator tmicfin;

        std::vector <boost::shared_ptr<Vehicule>>::iterator ItCurVeh;
        std::vector <boost::shared_ptr<Vehicule>>::iterator ItDebVeh;
        std::vector <boost::shared_ptr<Vehicule>>::iterator ItFinVeh;

        std::deque<TraceDocTrafic *>::iterator itDocTrafic;


        tmacdebut = m_LstTuyauxMacro.begin();
        tmacfin = m_LstTuyauxMacro.end();

        tmicdebut = m_LstTuyauxMicro.begin();
        tmicfin = m_LstTuyauxMicro.end();

        tmesdebut = m_LstTuyauxMeso.begin();
        tmesfin = m_LstTuyauxMeso.end();

        // Post-traitement de l'affectation si besoin (uniquement en dynamique et pas au premier pas de temps)
        if (m_pModuleAffectation && IsAffectationDynamique() && m_nInstSim != 0 && !IsSymuMasterMode())
        {
            m_pModuleAffectation->Run( this, m_dbInstSimu, 'P', false);
        }

        // MAJ des variables de gestion de l'instant
        m_dbInstSimu += pas_de_temps;
        m_nInstSim++;

        // recalcul des itinÃ©raires si les paramÃ¨tres ont changÃ© par rapport au pas de temps prÃ©cÃ©dent,
        // ou en cas de variation temporelle de voie rÃ©servÃ©e.
        if (m_pModuleAffectation && !IsSymuMasterMode() && m_LstItiChangeInstants.find(m_dbInstSimu) != m_LstItiChangeInstants.end())
        {
            m_pSymuScriptManager->ForceGraphRefresh();
        }

        // ecrit unqiuement si on est dans les bornes de restitution ou qu'on a pas de contrainte de restitution
   //     bool bDoExtractionFromBound = CanDoExtractionFromBound();

        // Debug
        //if( DoSortieTraj() || GetSymMode()==SYM_MODE_STEP_XML )

        //DWORD t1, t2, T1, T2;

        log() << std::endl << "Instant : "<<m_dbInstSimu ;

        //*m_pFicSimulation<<m_LstVehicule.size()<<" ";
        //T1 = GetTickCount();

        //FicTraficCel<<std::endl<<"Instant : "<<temps<<std::endl;s
            if(m_bFichierSAS)
                *FicTraficSAS<<std::endl<<"Instant : "<<m_dbInstSimu<<std::endl;

        // Assignation du document XML de trafic (temporaire dans le cas d'une affectation calculÃ©e convergente)
        std::deque<TraceDocTrafic *> xmlDocTrafics;

        if (m_pModuleAffectation && m_pModuleAffectation->GetCurrentDocTraficTmp())
        {
            xmlDocTrafics.push_back(m_pModuleAffectation->GetCurrentDocTraficTmp());
        }
        else
        {
             xmlDocTrafics = GetVariations<TraceDocTrafic>(m_dbInstSimu, &m_xmlDocTrafics, m_dbLag );
        }

        // Compteur temps 1
        //t1 = GetTickCount();
        //if( this->DoSortieTraj() ) // ETS 141030 dÃ©commente
        {
         // ajout l'instant pour les plage d'extactions demandÃ©es
            for( itDocTrafic = xmlDocTrafics.begin(); itDocTrafic != xmlDocTrafics.end(); itDocTrafic++)
            {

                (*itDocTrafic)->AddInstant(m_dbInstSimu, pas_de_temps, (int)m_LstVehicles.size() );


                // ajout la balise CREATION
                if( m_vehiclesToAdd[  (*itDocTrafic)].size()> 0 )
                {
                    if (DoSortieTraj())
                    {
                        std::deque<SVehicleToAdd>::const_iterator it = m_vehiclesToAdd[(*itDocTrafic)].begin();
                        while (it != m_vehiclesToAdd[*itDocTrafic].end())
                        {
                            // std::deque< DocTrafic *>

                            boost::shared_ptr<Vehicule> pVhe = GetVehiculeFromID(it->nID);
                            if (!pVhe){
                                it = m_vehiclesToAdd[*itDocTrafic].erase(it);
                            }
                            else{
                                (*itDocTrafic)->AddVehiculeToCreationNode(it->nID, it->strLibelle, it->strType,
                                    it->dbKx, it->dbVx, it->dbW, it->strEntree, it->strSortie, it->strRoute, m_dbInstSimu, it->sVoie, it->bAgressif);
                                it++;
                            }
                        }
                    }
                    m_vehiclesToAdd.clear();

                }
            }// rof each document Ã  extraire
        } // fi on trace les instants


        //t2 = GetTickCount();
        //*m_pFicSimulation << t2-t1 << " ";


        // Sondage des vitesses rÃ©glementaires : si elles ont Ã©voluÃ©es par rapport au pas de temps prÃ©cÃ©dent, un nouveau calcul d'itinÃ©raire doit Ãªtre effectuÃ©
        // ou un nouveau calcul d'affectation selon type de comportement des vÃ©hicules
        bool bRecalculItineraire = false;
        std::deque<TypeVehicule*>::iterator itTV;
        if (m_dbInstSimu > pas_de_temps && IsCptItineraire() && IsAffectationDynamique() && !IsSymuMasterMode())
        {
            for (tmiccourant=tmicdebut;tmiccourant!=tmicfin && bRecalculItineraire;++tmiccourant)
            {
                for(itTV = this->m_LstTypesVehicule.begin(); itTV != this->m_LstTypesVehicule.end() && bRecalculItineraire; ++itTV)
                {
                    // TODO - tracer dans MANTIS : pas de recalcul de l'itinÃ©raire en cas de SendSpeedLimit ou SendSpeedLimitPortion!
                    if((*tmiccourant)->GetCoutAVide(m_dbInstSimu, *itTV, false) != (*tmiccourant)->GetCoutAVide(m_dbInstSimu-this->pas_de_temps, *itTV, false))
                    {
                        bRecalculItineraire = true;
                    }
                }
            }
            for (tmescourant=tmesdebut;tmescourant!=tmesfin && bRecalculItineraire;++tmescourant)
            {
                for(itTV = this->m_LstTypesVehicule.begin(); itTV != this->m_LstTypesVehicule.end() && bRecalculItineraire; ++itTV)
                {

                    if((*tmescourant)->GetCoutAVide(m_dbInstSimu, *itTV, false)!= (*tmescourant)->GetCoutAVide(m_dbInstSimu-this->pas_de_temps, *itTV, false))
                    {
                        bRecalculItineraire = true;
                    }
                }
            }
            if(bRecalculItineraire)
                m_pModuleAffectation->Run( this, m_dbInstSimu, 'V', false);	// Lancement du module d'affectation
        }

        std::deque<SymuViaTripNode*>::iterator itOrigine;
        for( itOrigine = Liste_origines.begin(); itOrigine != Liste_origines.end(); itOrigine++)
        {
            (*itOrigine)->UpdateCritCreationVeh(m_dbInstSimu + m_dbDiffDebutSimuData, false);
        }

        for(int i=0; i<(int)Liste_carrefoursAFeux.size(); i++)	// CB 31/05/18: trÃ¨s chronophage et quasi aucun impact
        {
            Liste_carrefoursAFeux[i]->UpdatePrioriteTraversees(m_dbInstSimu+m_dbDiffDebutSimuData);        // MAJ des prioritÃ©s des traversÃ©es du CAF en fonction de la couleur des feux
            Liste_carrefoursAFeux[i]->UpdateConvergents(m_dbInstSimu+m_dbDiffDebutSimuData);               // MAJ des prioritÃ©s des convergents du CAF en fonction de la couleur des feux
        }
        m_bRelancerCalculRepartiteur = false;

        // Creation des vÃ©hicules crÃ©Ã©s avec la fonction SymCreateVehicle
        for(size_t iVeh = 0; iVeh < m_VehiclesToCreate.size(); iVeh++)
        {
            VehicleToCreate * pVehicleToCreate = m_VehiclesToCreate[iVeh];
            pVehicleToCreate->GetFleet()->ActivateVehicle(m_dbInstSimu, pVehicleToCreate);
            delete pVehicleToCreate;
        }
        m_VehiclesToCreate.clear();

        //t2 = GetTickCount();
        //*m_pFicSimulation << t2-t1 << " ";

        // Compteur temps 3
        //t1 = GetTickCount();

        // pour ne pas refaire des initialisations si dÃ©jÃ  faites Ã  cause du pilotage d'un vÃ©hicule
        if(!m_bDriven)
        {
            InitVehicules(m_dbInstSimu+m_dbDiffDebutSimuData);
        }
        //t2 = GetTickCount();
        //*m_pFicSimulation << t2-t1 << " ";

        prvGenerateVehicule();

        // Simulation traffic MICRO et Macro
        SimuTraficMicroMacro();


        // Mise Ã  jour des contrÃ´leurs de feux (notamment le mode de fonctionnement)

        // Compteur temps 13
        //t1 = GetTickCount();
        // rmq. : remplacer la fonctionnalitÃ© de VGP par une brique du module de rÃ©gulation ?
        for(int i=0; i<(int)Liste_ControleursDeFeux.size(); i++)
        {
            ((ControleurDeFeux*)Liste_ControleursDeFeux[i])->Update(m_dbInstSimu);
        }

        // appel au module de rÃ©gulation
        m_pRegulationModule->run();

        // Generation des sorties

        // generation des sorties associÃ©es aux briques de rÃ©gulation
        if (DoSortieTraj())
        {
             for( itDocTrafic = xmlDocTrafics.begin(); itDocTrafic != xmlDocTrafics.end(); itDocTrafic++)
             {
                 m_pRegulationModule->restitution(*itDocTrafic);
             }
        }

       // t2 = GetTickCount();
        //*m_pFicSimulation << t2-t1 << " ";

        // Mise Ã  jour du cumul du nombre de vÃ©hicules prÃ©sents sur le rÃ©seau Ã  chaque pas de temps
        m_nNbVehCum += (int)m_LstVehicles.size();


        // Sortie tronÃ§ons macroscopiques
        for (tmaccourant=tmacdebut;tmaccourant!=tmacfin;tmaccourant++)
        {
            (*tmaccourant)->TrafficOutput();
        }

        // Sortie tronÃ§ons microscopiques
        if(IsSortieTraj())
        {
            // Trajectoires des vÃ©hicules
            ItDebVeh = m_LstVehicles.begin();
            ItFinVeh = m_LstVehicles.end();

            for (ItCurVeh=ItDebVeh;ItCurVeh!=ItFinVeh;++ItCurVeh)
            {
                // Evolution nÂ°100 : restitution des types de vÃ©hicules voulus uniquement
                if(m_setTypesRestitution.find((*ItCurVeh)->GetType()) != m_setTypesRestitution.end())
                {
                   // if( bDoExtractionFromBound )
                    {
                         for( itDocTrafic = xmlDocTrafics.begin(); itDocTrafic != xmlDocTrafics.end(); itDocTrafic++)
                         {
                            (*ItCurVeh)->SortieTrafic( *itDocTrafic, m_bChgtVoieDebug, m_dbInstSimu );
                         }
                    }
                }
            }

            // SAS
            if(m_bFichierSAS)
            {
                for (tmiccourant=tmicdebut;tmiccourant!=tmicfin;tmiccourant++)
                {
                    (*tmiccourant)->TrafficOutput();
                }
            }
        }

        if(m_bCSVOutput && m_CSVOutputWriter && m_bCSVTrajectories)
        {
            ItDebVeh = m_LstVehicles.begin();
            ItFinVeh = m_LstVehicles.end();

            for (ItCurVeh=ItDebVeh;ItCurVeh!=ItFinVeh;ItCurVeh++)
            {
                m_CSVOutputWriter->writeTrajectory(m_dbInstSimu, (*ItCurVeh).get());
            }
        }

        // appel de la sortie Trafic pour toutes les flottes
        for( itDocTrafic = xmlDocTrafics.begin(); itDocTrafic != xmlDocTrafics.end(); itDocTrafic++)
        {
            for(size_t i = 0; i < m_LstFleets.size(); i++)
            {
                m_LstFleets[i]->SortieTrafic(*itDocTrafic);
            }
        }

        // Sortie des infos sur les tronÃ§ons (mÃ©so uniquement pour l'instant)
        for( itDocTrafic = xmlDocTrafics.begin(); itDocTrafic != xmlDocTrafics.end(); itDocTrafic++)
        {
            for(size_t iLink = 0; iLink < m_LstTuyauxMeso.size(); iLink++)
            {
                m_LstTuyauxMeso[iLink]->SortieTrafic(*itDocTrafic);
            }
        }

        // Sauvegarde de l'Ã©tat du feux Ã  la fin du pas de temps
        if (DoSortieTraj())
        {
            SaveControleurDeFeux(xmlDocTrafics);
        }


        // Ecriture du fichier XML de trafic
        if (DoSortieTraj() && m_bXmlOutput /*&& bDoExtractionFromBound*/)
        {
            for( itDocTrafic = xmlDocTrafics.begin(); itDocTrafic != xmlDocTrafics.end(); itDocTrafic++)
            {
                (*itDocTrafic)->SaveLastInstant();
            }
        }

        //SortieIndicateursInterDistance();

        // on met Ã  jour la rÃ©gle de vitesse rÃ©glementaire associÃ©e aux vÃ©hicules
        // pour prise en compte de l'entrÃ©e dans les portions de vitesse rÃ©glementaire spÃ©cifiques
        UpdateVitessesReg();


        // On passe Ã  l'instant suivant si la simulation complÃ¨te est gÃ©nÃ©rÃ©e ou si le calcul de l'acoustique (ou le calcul des Ã©missions) n'est pas demandÃ©
        //if((m_SymMode == Reseau::SYM_MODE_FULL) || !(IsSimuAcoustique() || IsSimuAir()) )
        //	m_dbInstSimu+=pas_de_temps;

        //T2 = GetTickCount();
        //*m_pFicSimulation << T2-T1 << std::endl;

        // MAJ de l'affectation si dernier instant calculÃ©
        // rmq. : on l'appelle aussi en affectation statique, en cas de sauvegarde des rÃ©sultats d'affectation
        if (IsCptItineraire() && !IsSymuMasterMode() && (IsAffectationDynamique() || m_pModuleAffectation->IsSaving()) && ((hd * 3600 + md * 60 + sd + m_dbInstSimu) >= (hf * 3600 + mf * 60 + sf)))
        {
            m_pModuleAffectation->Run( this, m_dbInstSimu, 'P', false);
        }

        // rÃ©initialisation pour le pas de temps suivant
        for(size_t i = 0; i < m_LstVehicles.size(); i++)
        {
            m_LstVehicles[i]->ResetDriveState();
        }
        m_bDriven = false;

        // gestion du switch des tronÃ§ons micro vers meso ou inversement pour le pas de temps suivant
        if(!m_MicroVehicleTypes.empty())
        {
            SwitchMicroMeso();
        }
    } //  fi (m_dbInstSimu < m_dbInstSimuMeso )
    else
    {
        SimuTraficMeso();

    } // sinon on calcul le mÃ©soscopic

    for (int i = 0; i < m_LstInstantsSauvegardes.size(); i++){
        if (m_LstInstantsSauvegardes[i] == m_dbInstSimu) {
            //Sauvegarde de l'Ã©tat du rÃ©seaux
            std::string currentTime = (m_dtDebutSimu + time_t(m_dbInstSimu)).ToString();
            currentTime.erase(std::remove(currentTime.begin(), currentTime.end(), ':'), currentTime.end());
            std::string strFilePath = m_sPrefixOutputFiles + "_" + currentTime + "_save.xml"; 
            SerializeUtil::Save(this, (char*)strFilePath.c_str());
            log() << std::endl << "Network state saved !";
            break;
        }
    }

    return ( (hd*3600+md*60+sd+m_dbInstSimu) >= (hf*3600+mf*60+sf) ) ;

}

//================================================================
    void Reseau::UpdateConvergents()
//----------------------------------------------------------------
// Fonction  : Met Ã  jour les convergents
//             rÃ©glementaire
// Version du: 16/01/2013
// Historique:
//================================================================
{
    // Mise Ã  jour des infos capteurs liÃ©s aux convergents
    std::vector <boost::shared_ptr<Vehicule>>::iterator itVeh;
    std::vector <boost::shared_ptr<Vehicule>>::iterator itDeb;
    std::vector <boost::shared_ptr<Vehicule>>::iterator itFin;

    itDeb = m_LstVehicles.begin();
    itFin = m_LstVehicles.end();

    Vehicule * pVeh;
    size_t nbSensors;
    size_t nbVoiesInternes;
    Tuyau *pTuyau0, *pTuyau1, *pTuyauIntermediaire;
    double dbPos0, dbPos1;
    size_t t, j;
    std::vector<PonctualSensor*> * pLstCapteurs;
    PonctualSensor* pCpt;
    Voie * pVoie;
    for(itVeh = itDeb; itVeh!=itFin; itVeh++)
    {
        pVeh = (*itVeh).get();

        dbPos0 = pVeh->GetPos(0);

        if(pVeh->GetVit(0)>0 && dbPos0 >= 0)    // Si vitesse nulle, le vÃ©hicule n'a pu franchir le capteur ! De la mÃªme faÃ§on, si il est hors du rÃ©seau !
        {
            pTuyau0 = pVeh->GetLink(0);
            pTuyau1 = pVeh->GetLink(1);
            dbPos1 = pVeh->GetPos(1);
            // 1 - Test des capteurs prÃ©sents sur le tuyau du dÃ©but du pas de temps
            if(pTuyau1)
            {
                pLstCapteurs = &pTuyau1->getListeCapteursConvergents();
                nbSensors = pLstCapteurs->size();
                for(j=0; j< nbSensors; j++)
                {
                    pCpt = pLstCapteurs->at(j);

                    // Le capteur est positionnÃ© en aval du vÃ©hicule au dÃ©but du pas de temps ?
                    if( dbPos1 < pCpt->GetPosition() )
                    {
                        // Le vÃ©hicule est-il positionnÃ© aprÃ¨s le capteur Ã  la fin du pas de temps
                        // ou a t'il changÃ© de tuyau ?
                        if( dbPos0 >= pCpt->GetPosition() || pTuyau1 != pTuyau0 )
                        {
                            pVoie = pVeh->GetVoie(1);     // Voie du vÃ©hicule
                            if(pVoie)
                            {
                                // IncrÃ©mentation du compteur des types
                                if (!m_bOffreCvgDeltaN)
                                    pCpt->GetNbTypeVehInst()[pVeh->GetType()][pVoie][pCpt->GetLastInd()]++;
                                else
                                {
                                    double deltaNToAdd = 1;
                                    if(dynamic_cast<NewellCarFollowing*>(pVeh->GetCarFollowing()))
                                    {
                                        deltaNToAdd = ((NewellContext*)pVeh->GetCarFollowing()->GetCurrentContext())->GetDeltaN();//pVeh->GetDeltaNFin();
                                    }
                                    else
                                    {
                                        // CrÃ©ation d'un context Newell virtuel pour calcul du deltaN
                                        NewellContext * pNewellContext = new NewellContext(this, pVeh, m_dbInstSimu, true);
                                        pNewellContext->SetContext(pVeh->GetCarFollowing()->GetCurrentContext()->GetLeaders(),
                                                                   pVeh->GetCarFollowing()->GetCurrentContext()->GetLeaderDistances(),
                                                                   pVeh->GetCarFollowing()->GetCurrentContext()->GetLstLanes(),
                                                                   0, NULL, true);
                                        deltaNToAdd = pNewellContext->GetDeltaN();
                                    }
                                    pCpt->GetNbTypeVehInst()[pVeh->GetType()][pVoie][pCpt->GetLastInd()] += deltaNToAdd;
                                }

                            }
                        }
                    }
                }
            }

            if( pTuyau1 != pTuyau0 )
            {
                // 2 - Test des capteurs prÃ©sents sur le tuyau de la fin du pas de temps si diffÃ©rent du dÃ©but du pas de temps
                if(pTuyau0)
                {
                    pLstCapteurs = &pTuyau0->getListeCapteursConvergents();
                    nbSensors = pLstCapteurs->size();
                    for(j=0; j< nbSensors; j++)
                    {
                        pCpt = pLstCapteurs->at(j);

                        // Le vÃ©hicule a t-il passÃ© le capteur ?
                        if( dbPos0 >= pCpt->GetPosition() )
                        {
                            pVoie = pVeh->GetVoie(0);     // Voie du vÃ©hicule
                            if(pVoie)
                            {
                                // IncrÃ©mentation du compteur des types
                                if (!m_bOffreCvgDeltaN)
                                    pCpt->GetNbTypeVehInst()[pVeh->GetType()][pVoie][pCpt->GetLastInd()]++;
                                else
                                {
                                    double deltaNToAdd = 1;
                                    if(dynamic_cast<NewellCarFollowing*>(pVeh->GetCarFollowing()))
                                    {
                                        deltaNToAdd = ((NewellContext*)pVeh->GetCarFollowing()->GetCurrentContext())->GetDeltaN();//pVeh->GetDeltaNFin();
                                    }
                                    else
                                    {
                                        // CrÃ©ation d'un context Newell virtuel pour calcul du deltaN
                                        NewellContext * pNewellContext = new NewellContext(this, pVeh, m_dbInstSimu, true);
                                        pNewellContext->SetContext(pVeh->GetCarFollowing()->GetCurrentContext()->GetLeaders(),
                                                                   pVeh->GetCarFollowing()->GetCurrentContext()->GetLeaderDistances(),
                                                                   pVeh->GetCarFollowing()->GetCurrentContext()->GetLstLanes(),
                                                                   0, NULL, true);
                                        deltaNToAdd = pNewellContext->GetDeltaN();
                                    }
                                    pCpt->GetNbTypeVehInst()[pVeh->GetType()][pVoie][pCpt->GetLastInd()] += deltaNToAdd;
                                }
                            }
                        }
                    }
                }

                // 3 - Test des capteurs prÃ©sents sur les tronÃ§on intermÃ©diaires
                nbVoiesInternes = pVeh->m_LstUsedLanes.size();
                for(t=0; t< nbVoiesInternes; t++)
                {
                    pTuyauIntermediaire = (Tuyau*)pVeh->m_LstUsedLanes[t]->GetParent();
                    pLstCapteurs = &pTuyauIntermediaire->getListeCapteursConvergents();
                    nbSensors = pLstCapteurs->size();
                    for(j=0; j< nbSensors; j++)
                    {
                        pCpt = pLstCapteurs->at(j);

                        pVoie = pVeh->m_LstUsedLanes[t];     // Voie du vÃ©hicule
                        if(pVoie)
                        {
                            // IncrÃ©mentation du compteur des types
                            if (!m_bOffreCvgDeltaN)
                                pCpt->GetNbTypeVehInst()[pVeh->GetType()][pVoie][pCpt->GetLastInd()]++;
                            else
                            {
                                double deltaNToAdd = 1;
                                if(dynamic_cast<NewellCarFollowing*>(pVeh->GetCarFollowing()))
                                {
                                    deltaNToAdd = ((NewellContext*)pVeh->GetCarFollowing()->GetCurrentContext())->GetDeltaN();//pVeh->GetDeltaNFin();
                                }
                                else
                                {
                                    // CrÃ©ation d'un context Newell virtuel pour calcul du deltaN
                                    NewellContext * pNewellContext = new NewellContext(this, pVeh, m_dbInstSimu, true);
                                    pNewellContext->SetContext(pVeh->GetCarFollowing()->GetCurrentContext()->GetLeaders(),
                                                                pVeh->GetCarFollowing()->GetCurrentContext()->GetLeaderDistances(),
                                                                pVeh->GetCarFollowing()->GetCurrentContext()->GetLstLanes(),
                                                                0, NULL, true);
                                    deltaNToAdd = pNewellContext->GetDeltaN();
                                }
                                pCpt->GetNbTypeVehInst()[pVeh->GetType()][pVoie][pCpt->GetLastInd()] += deltaNToAdd;
                            }
                        }
                    }
                }
            }
        }
    }
}


//================================================================
    void Reseau::UpdateVitessesReg()
//----------------------------------------------------------------
// Fonction  : Met Ã  jour la vitesse rÃ©glementaire Ã  utiliser pour
//             chaque vÃ©hicule au prochain pas de temps.
//             nÃ©cessaire pour gestion des portions de vitesse
//             rÃ©glementaire
// Version du: 16/05/2011
// Historique:
//================================================================
{
    std::vector <boost::shared_ptr<Vehicule>>::iterator ItCurVeh;
    std::vector <boost::shared_ptr<Vehicule>>::iterator ItDebVeh;
    std::vector <boost::shared_ptr<Vehicule>>::iterator ItFinVeh;

    ItDebVeh = m_LstVehicles.begin();
    ItFinVeh = m_LstVehicles.end();

    for (ItCurVeh=ItDebVeh;ItCurVeh!=ItFinVeh;ItCurVeh++)
    {
        (*ItCurVeh)->UpdateVitesseReg(m_dbInstSimu);
    }
}


//================================================================
    bool Reseau::SimuEmissions
//----------------------------------------------------------------
// Fonction  : Calcule les Ã©missions d'un pas de temps (acoustique/
//             atmosphÃ©rique)
// Version du: 02/04/2009
// Historique:
//================================================================
(
    bool bAcoustic,
    bool bAir,
    bool bSirane
)
{
    double dbValCO2, dbValNOx, dbValPM;
    double dbCumCO2, dbCumNOx, dbCumPM;
    std::deque <Tuyau*>::iterator tcourant;
    std::deque <Tuyau*>::iterator tdebut;
    std::deque <Tuyau*>::iterator tfin;

    std::vector <boost::shared_ptr<Vehicule>>::iterator ItCurVeh;
    std::vector <boost::shared_ptr<Vehicule>>::iterator ItDebVeh;
    std::vector <boost::shared_ptr<Vehicule>>::iterator ItFinVeh;

    SrcAcoustique srcAcous;

    // Initialisation du cumul des polluants pour l'instant considÃ©rÃ©
    dbCumCO2 = 0;
    dbCumNOx = 0;
    dbCumPM  = 0;

    // MAJ des variables de gestion de l'instant
    if (this->GetSymMode() == Reseau::SYM_MODE_FULL)
    {
        m_dbInstSimu+=pas_de_temps;
        m_nInstSim++;
    }

    if( bAcoustic && m_XmlDocAcoustique)
        m_XmlDocAcoustique->AddInstant(m_dbInstSimu, 0 );

    if( bAir && m_XmlDocAtmo)
        m_XmlDocAtmo->AddInstant(m_dbInstSimu, 0);

    log()<<std::endl<<"Instant : "<<m_dbInstSimu<<std::endl;

    m_bSimuTrafic = false;

     // Reconstitution des donnÃ©es du trafic Ã  partir du XML (fichier ou flux)
    if (m_SymMode == Reseau::SYM_MODE_FULL)
        LoadTrafic();

    if( bAcoustic )
    {
        // si on effectue le calcul acoustique pour la simulation
        // calcul de l'emission de bruit
        tdebut = m_LstTuyaux.begin();
        tfin = m_LstTuyaux.end();
        for (tcourant=tdebut;tcourant!=tfin;tcourant++)
        {
            // Initialisation des caractÃ©ristiques acoustiques
            (*tcourant)->InitAcousticVariables();
        }

        for (tcourant=tdebut;tcourant!=tfin;tcourant++)
        {  // c'est la dedans que l'ecriture dans les fichiers de sortie d'emission a lieu
            (*tcourant)->ComputeNoise(Loi_emis);
        }
    }

    // Calcul de l'Ã©mission des vÃ©hicules
    ItDebVeh = m_LstVehicles.begin();
    ItFinVeh = m_LstVehicles.end();

    for (ItCurVeh=ItDebVeh;ItCurVeh!=ItFinVeh;ItCurVeh++)
    {
        boost::shared_ptr<Vehicule> pV = (*ItCurVeh);

        if( bAcoustic )
        {
            for(int i=0; i<(int)pV->GetType()->GetLstSrcAcoustiques().size(); i++)
            {
                srcAcous = pV->GetType()->GetLstSrcAcoustiques()[i];

                if(pV->GetLink(0))
                {
                    pV->CalculEmission(Loi_emis, srcAcous.strIDDataBase.c_str(), pV->GetLink(0)->GetRevetement());

                    if( m_bAcousSrcs )	// Sortie par source
                    {
                        pV->SortieEmission();
                    }
                    if( m_bAcousCell )	// Sortie par cellule
                    {
                        // Ajout de la contribution du vÃ©hicule Ã  la cellule acoustique correspondante
                        Segment *pCellAcoustique;
                        pCellAcoustique = pV->GetCellAcoustique(srcAcous.dbPosition);

                        if(pCellAcoustique)
                        {
                            pCellAcoustique->AddPuissanceAcoustique(pV->GetPuissanceAcoustique());
                        }
                    }
                }
            }
        }

        if(bAir)
        {
            // Calcul des Ã©missions atmosphÃ©riques du vÃ©hicules
            pV->CalculEmissionAtmos(pas_de_temps, dbValCO2, dbValNOx, dbValPM);
            dbCumCO2 += dbValCO2;
            dbCumNOx += dbValNOx;
            dbCumPM  += dbValPM;

            // Compteurs cumulÃ©s
            m_dbCptCumCO2 += dbValCO2;
            m_dbCptCumNOx += dbValNOx;
            m_dbCptCumPM += dbValPM;

            // Sortie des valeurs pour chaque vÃ©hicule Ã  chaque pas de temps
            m_XmlDocAtmo->AddVehiculeInst( pV->GetID(),  dbValCO2, dbValNOx, dbValPM );
        }

        if(bSirane)
        {
            // Calcul pour le pas de temps courant du temps Ã©coulÃ© dans chaque cellule
            // et dans chaque plage de vitesse
            if(pV->GetLink(0))
            {
                // Ajout de la contribution du vÃ©hicule Ã  la cellule acoustique correspondante
                Segment *pCellSirane = pV->GetCellSirane();
                if(pCellSirane)
                {
                    pCellSirane->AddSpeedContribution(pV->GetType(), pV->GetVit(1), pV->GetVit(0), pas_de_temps);
                }
            }
        }
    }

    // Sortie des Ã©missions pour les tuyaux avec cellule acoustique
    if(bAcoustic)
    {
        for (tcourant=tdebut;tcourant!=tfin;tcourant++)
        {
            (*tcourant)->EmissionOutput();
        }

        // Ecriture du fichier XML d'acoustique
        m_XmlDocAcoustique->SaveLastInstant();
    }

    if(bAir)
    {
        // Sortie des valeurs cumulÃ©es du polluant pour l'instant
        m_XmlDocAtmo->Addval( dbCumCO2, dbCumNOx, dbCumPM, m_dbCptCumCO2, m_dbCptCumNOx, m_dbCptCumPM);

        m_XmlDocAtmo->SaveLastInstant();
    }

    if(bSirane)
    {
        // Si l'instant passÃ© clos la pÃ©riode d'agrÃ©gation, on sort les rÃ©sultats correspondants
        if(fmod(m_dbInstSimu, m_dbPeriodeAgregationSirane) < pas_de_temps)
        {
            // sortie des rÃ©sultats

            // nouveau noeud pÃ©riode
            m_XmlDocSirane->AddPeriod(m_dbDebutPeriodeSirane, m_dbInstSimu);
            m_dbDebutPeriodeSirane = m_dbInstSimu;

            // parcours des tuyaux
            tdebut = m_LstTuyaux.begin();
            tfin = m_LstTuyaux.end();
            for (tcourant=tdebut;tcourant!=tfin;tcourant++)
            {
                const std::vector<Segment*> & cells = ((TuyauMicro*)(*tcourant))->GetSiraneCells();
                for(size_t cellIdx = 0; cellIdx < cells.size(); cellIdx++)
                {
                    cells[cellIdx]->SortieSirane();
                    cells[cellIdx]->InitVariablesSirane();
                }
            }

            //parcours des briques de connexion
            BriqueDeConnexion* pBrique;
            for(size_t briqueIdx = 0; briqueIdx < Liste_carrefoursAFeux.size(); briqueIdx++)
            {
                pBrique = Liste_carrefoursAFeux[briqueIdx];
                pBrique->GetCellSirane()->SortieSirane();
                pBrique->GetCellSirane()->InitVariablesSirane();
            }
            for(size_t briqueIdx = 0; briqueIdx < Liste_giratoires.size(); briqueIdx++)
            {
                pBrique = Liste_giratoires[briqueIdx];
                pBrique->GetCellSirane()->SortieSirane();
                pBrique->GetCellSirane()->InitVariablesSirane();
            }

            // fermeture nouveau noeud pÃ©riode
            m_XmlDocSirane->ClosePeriod();

        }
    }

    return ( (hd*3600+md*60+sd+m_dbInstSimu) >= (hf*3600+mf*60+sf) ) ;

}


// Fin de la simulation du rÃ©seau
void Reseau::FinSimuTrafic()
{
    if(m_bDebugOD && m_bLogFile)
    {
        // Ecriture du nombre de vÃ©hicule crÃ©Ã©s par entrÃ©e

        *g_FicDebug<<std::endl<<"Number of created vehicles by input "<<std::endl;

        SymuViaTripNode* pEntree;
        std::deque <SymuViaTripNode*>::iterator ent;
        std::deque <SymuViaTripNode*>::iterator ent_d = Liste_origines.begin();
        std::deque <SymuViaTripNode*>::iterator ent_f = Liste_origines.end();
        if( m_bLogFile )
        {
            for (ent=ent_d; ent!=ent_f; ent++)
            {
                pEntree = (*ent);

                *g_FicDebug << std::endl << "Input " << pEntree->GetOutputID() << " :" << std::endl;
                for(int i=0; i<pEntree->GetNbVariations();i++)
                    *g_FicDebug<< "Variation " << i << " : " << pEntree->GetNbVehicule(i) << std::endl;
            }
        }

        /*
        //  Ecriture des compteurs des vÃ©hicules
        FicDebug<<std::endl<<"Compteur des vÃ©hicules par (origine, destination) "<<std::endl;
        rdebut = Liste_repartiteurs.begin();
        rfin = Liste_repartiteurs.end();
        Repartiteur *pRep;
        Tuyau*  TAmont;
        Tuyau*  TAval;
        int     nVAv, nVAm;
        int     nAmont, nAval;

        for (rcourant=rdebut;rcourant!=rfin;rcourant++)
        {
            pRep = (Repartiteur *) (*rcourant);
            if( pRep->IsMicro() )
            {
                FicDebug << std::endl << "RÃ©partiteur " << pRep->GetID() << " :" << std::endl;

              for( nAmont = 0; nAmont < pRep->getNbAmont(); nAmont++)
                {
                    TAmont = pRep->m_LstTuyAm[nAmont];
                    for( nVAm = 0; nVAm < TAmont->getNbVoiesDis(); nVAm++)
                    {
                        for( nAval = 0; nAval < pRep->GetNbElAval(); nAval++)
                        {
                            TAval = pRep->m_LstTuyAv[nAval];
                            for( nVAv = 0; nVAv < TAval->getNbVoiesDis(); nVAv++)
                            {
                                FicDebug << "Origine : "<< TAmont->GetLabel() << " voie " << (nVAm+1);
                                FicDebug << " - Destination : "<< TAval->GetLabel() << " voie " << (nVAv+1) << std::endl;
                                for(unsigned int i=0; i< pRep->GetLstRepartitions().size(); i++)
                                {
                                    FicDebug << pRep->GetNbVehRep(nAmont, nVAm, nAval, nVAv, i) << " ";
                                }
                                FicDebug << std::endl;
                            }
                        }
                    }
                }
            }
        }*/
        // Ecriture de la matrice OD
        //if(IsUsedODMatrix())
        {
            *g_FicDebug << "OD Matrix "<< std::endl;
             for (ent=ent_d; ent!=ent_f; ent++)
            {
                pEntree = (*ent);
                *g_FicDebug << "--- Input " << pEntree->GetInputID()<< " ---" << std::endl ;

                // Les sorties...
                for(int j=0; j<(int)pEntree->GetLstDestinations()->size();j++)
                    *g_FicDebug << pEntree->GetLstDestinations()->at(j)->GetInputID() << " ";
                *g_FicDebug << std::endl ;

                // Boucle sur les variations
                for( int i=0; i<(int)pEntree->GetLogMatriceODSize();i++)
                {
                    for(int j=0; j<(int)pEntree->GetLstDestinations()->size();j++)
                        *g_FicDebug << pEntree->GetLogMatriceOD(i,j) << " ";
                    *g_FicDebug << std::endl;
                }
                *g_FicDebug << std::endl ;
            }
        }
        g_FicDebug->close();
    }

    // On vide la liste des vÃ©hicules sur le rÃ©seau
    std::vector <boost::shared_ptr<Vehicule>>::iterator veh;
    for (veh=m_LstVehicles.begin(); veh!=m_LstVehicles.end(); veh++)
    {
        std::string sDest;
        TripNode * pDest = (*veh)->GetTrip()->GetFinalDestination();
        if (pDest)
        {
            if (pDest->GetInputPosition().IsLink())
            {
                sDest = pDest->GetInputPosition().GetLink()->GetCnxAssAm()->GetID();
            }
            else
            {
                sDest = pDest->GetInputID();
            }
        }
        else if (!(*veh)->GetTuyauxParcourus().empty() && (*veh)->GetTuyauxParcourus().back()->GetCnxAssAv())
        {
            sDest = (*veh)->GetTuyauxParcourus().back()->GetCnxAssAv()->GetID();
        }

        std::string sOrig;
        if((*veh)->GetOrigine() )
            sOrig = (*veh)->GetOrigine()->GetOutputID();

        if(GetXmlDocTrafic().size()>0)
        {
            std::map<std::string, std::string> additionalAttributes = (*veh)->GetFleet()->GetOutputAttributesAtEnd((*veh).get());

            std::deque<TraceDocTrafic*> doctrafics =  GetXmlDocTrafic();
            std::deque<TraceDocTrafic* >::iterator itDocTraf;
            for( itDocTraf = doctrafics.begin(); itDocTraf != doctrafics.end(); itDocTraf++)
            {
                (*itDocTraf)->UpdateInstSortieVehicule( (*veh)->GetID(), -1, sDest, (*veh)->GetDstCumulee(), (*veh)->GetTuyauxParcourus(), additionalAttributes);
            }
        }

        if(m_bSimuAir && m_XmlDocAtmo)
        {
            m_XmlDocAtmo->AddVehicule( (*veh)->GetID(), (*veh)->GetType()->GetLabel(), sOrig, (*veh)->GetInstantEntree(), sDest, (*veh)->GetExitInstant(),
                (*veh)->GetCumCO2(), (*veh)->GetCumNOx(), (*veh)->GetCumPM(), (*veh)->GetDstCumulee(), true );
        }
    }
    m_LstVehicles.clear();

    // On vide la liste des vÃ©hicules en attente aux origines
    std::deque <SymuViaTripNode*>::iterator itO;
    for(itO = Liste_origines.begin(); itO != Liste_origines.end(); ++itO)
    {
        (*itO)->FinSimuTrafic();
    }

    //  Parcours des repartiteurs
    std::map<std::string, Repartiteur*>::iterator rep;
    for (rep=Liste_repartiteurs.begin(); rep!=Liste_repartiteurs.end(); ++rep)
    {
        rep->second->FinSimuTrafic();
    }

    if(m_pGestionsCapteur)
        m_pGestionsCapteur->Terminate();

    //fermeture des fichiers de sortie
    std::deque< TimeVariation<TraceDocTrafic> >::iterator itXmlDocTrafic;
    for( itXmlDocTrafic = m_xmlDocTrafics.begin(); itXmlDocTrafic!= m_xmlDocTrafics.end(); itXmlDocTrafic++ )
    {

        if( itXmlDocTrafic->m_pData )
        {
            itXmlDocTrafic->m_pData->Terminate();
        }

    }


    if(m_bFichierSAS)
        FicTraficSAS->close();


    // Lancement de la procÃ©dure de correction de dÃ©cÃ©lÃ©ration si demandÃ©e
    if (DoSortieTraj() && (m_SymMode == Reseau::SYM_MODE_FULL) && m_bProcDec)
    {
        for( itXmlDocTrafic = m_xmlDocTrafics.begin(); itXmlDocTrafic!= m_xmlDocTrafics.end(); itXmlDocTrafic++ )
        {
            DecelerationProcedure(m_dbDecTaux, this, (itXmlDocTrafic->m_pData->GetXMLDocTrafic())->m_strFilename);
        }
    }
    //if(m_bTraceRoute)
        //TrajectoireProcedure(this, m_XmlDocTrafic->m_strFilename);


    // rmq. : on ne vide pas ici les xmlDocTrafics mais on les remet Ã  zero (sinon
    // Ã  la rÃ©plication suivante, pas de sortie : il faut Ãªtre cohÃ©rent entre ce qui est fait dans le chargement
    // et ce qui est fait dans le initSimuTrafic.
     //EraseListOfVariation(&m_xmlDocTrafics);
    for (size_t i = 0; i < m_xmlDocTrafics.size(); i++)
    {
        m_xmlDocTrafics[i].m_pData = boost::make_shared<TraceDocTrafic>();
    }

    if(m_CSVOutputWriter)
    {
        delete m_CSVOutputWriter;
        m_CSVOutputWriter = NULL;
    }
    if (m_pTravelTimesOutputManager)
    {
        delete m_pTravelTimesOutputManager;
        m_pTravelTimesOutputManager = NULL;
    }

#ifdef USE_SYMUCOM
    if(m_pSymucomSimulator)
        //rmk : Pour l'instant on affiche Tout ce qui correspond Ã  l'ecriture du fichier XML SymuCom seulement Ã  la fin de la simulation
        //Processus surement Ã  revoir pour pouvoir faire de l'affectation dynamique
        m_pSymucomSimulator->TerminateSymuComXML();
#endif // USE_SYMUCOM

    SDateTime tNow = SDateTime::Now();
    log()<< std::endl << "Cumulated number of vehicles on the network for all time steps " << m_nNbVehCum <<std::endl;
    log()<<tNow.GetHour()<<":"<<tNow.GetMinute()<<":"<<tNow.GetSecond()<<std::endl<<std::endl;

    log() << Logger::Info << "Traffic flow computation duration : " <<  ((float)(clock() - m_initTime))/CLOCKS_PER_SEC << " s" << std::endl << std::endl;

    // Remise Ã  faux du boolean d'initialisation de la simu trafic
    m_bInitSimuTrafic = false;

}


void Reseau::FinSimuEmissions(bool bAcoustic, bool bAir, bool bSirane)
{
    if(m_XmlReaderTrafic)
    {
        delete(m_XmlReaderTrafic);
        m_XmlReaderTrafic = NULL;
    }

     // On vide la liste des vÃ©hicules sur le rÃ©seau
    std::vector <boost::shared_ptr<Vehicule>>::iterator veh;
    for (veh=m_LstVehicles.begin(); veh!=m_LstVehicles.end(); veh++)
    {
        if(bAir && m_XmlDocAtmo)
        {
            std::string sOrig;
            if((*veh)->GetOrigine() )
                sOrig = (*veh)->GetOrigine()->GetOutputID();

            std::string sDest;
            if((*veh)->GetDestination() )
                sDest = (*veh)->GetDestination()->GetInputID();

            m_XmlDocAtmo->AddVehicule( (*veh)->GetID(), (*veh)->GetType()->GetLabel(), sOrig, (*veh)->GetInstantEntree(), sDest, (*veh)->GetExitInstant(),
                        (*veh)->GetCumCO2(), (*veh)->GetCumNOx(), (*veh)->GetCumPM(), (*veh)->GetDstCumulee(), true );
        }
    }
    m_LstVehicles.clear();


    // Fermeture documents XML de sortie

    if(bAcoustic)
    {
        if (m_XmlDocAcoustique != NULL)
        {
            m_XmlDocAcoustique->Terminate();
            delete(m_XmlDocAcoustique);
            m_XmlDocAcoustique = NULL;
        }
    }

    if(bAir)
    {
        if (m_XmlDocAtmo != NULL)
        {
            m_XmlDocAtmo->Terminate();
            delete(m_XmlDocAtmo);
            m_XmlDocAtmo = NULL;
        }
    }

    if(bSirane)
    {
        if (m_XmlDocSirane != NULL)
        {
            m_XmlDocSirane->Terminate();
            delete(m_XmlDocSirane);
            m_XmlDocSirane = NULL;
        }
    }

    SDateTime tNow = SDateTime::Now();
    log()<<tNow.GetHour()<<":"<<tNow.GetMinute()<<":"<<tNow.GetSecond()<<std::endl<<std::endl;

}


//---------------------------------------------------------------------------------------------
// Fonction de renvoi des variables de rÃ©seau
//---------------------------------------------------------------------------------------------

// renvoi du pas de temps
double Reseau::GetTimeStep(void)
{
  return pas_de_temps;
}

//================================================================
    bool Reseau::VerifieVersionFichier
//----------------------------------------------------------------
// Fonction  : VÃ©rifie la version du format de fichier
// Version du: 29/11/2004
// Historique: 29/11/2004 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//================================================================
(
 std::ifstream *pFic
)
{
    char            buff[512];
    std::streamoff  p;
    char            strVersion[16];

    pFic->clear();
    pFic->seekg(0);p = 0;  //remise au debut

    memset(strVersion, 0x0, sizeof(strVersion));

    while(!pFic->eof())
    {
        p=pFic->tellg();
        pFic->getline(buff,512);

        if (!strcmp(buff,"*** Version ***"))
        {
            do
            {
                p=pFic->tellg();
                pFic->getline(buff,512);
            }
            while(((buff[0]=='/' && buff[1]=='/') || !strcmp(buff,"")) && !pFic->eof());
            // on echappe les commentaires et les lignes vides

            pFic->seekg(p);   //on remonte une ligne
            *pFic>>strVersion;

            if (strcmp(strVersion, VERSION_FICHIER) == 0)
                return true;
            else
                return false;

        }// Fin du if (!strcmp(buff,"*** Version ***"))

    } //Fin du while(!Fic.eof())
    return false;
}

//=================================================================
    void Reseau::ComputeFirstVehicules
//----------------------------------------------------------------
// Fonction  : Calcule les premiers vÃ©hicules de chaque voie
//             au dÃ©but du pas de temps.
// Remarque  : Fait une fois pour toutes en dÃ©but de pas de temps.
//             permet d'utiliser ensuite la fonction
//             GetFirstVehicule.
// Version du: 14/12/2012
// Historique: 14/12/2012 (O.Tonck - Ipsis)
//             CrÃ©ation
//=================================================================
(
)
{
    boost::shared_ptr<Vehicule> pVehicule;
    Voie * pVoie;
    map<Voie*, boost::shared_ptr<Vehicule>>::iterator iter;
    size_t nbVeh = m_LstVehicles.size();

    m_mapFirstVehicles.clear();
    for(size_t i = 0; i < nbVeh; i++)
    {
        pVehicule = m_LstVehicles[i];

        // On ignore les vÃ©hicules guidÃ©s
        if(pVehicule->GetFleet() == GetSymuViaFleet()
            && !pVehicule->IsMeso()) // et on ignore les vÃ©hicules meso
        {
            pVoie = pVehicule->GetVoie(1);
            // rÃ©cupÃ©ration de l'actuel leader
            iter = m_mapFirstVehicles.find(pVoie);
            if(iter != m_mapFirstVehicles.end())
            {
                // on a dÃ©ja un leader. On regarde si ce n'est pas plutÃ´t notre vÃ©hicule courant
                if(pVehicule->GetPos(1) > iter->second->GetPos(1))
                {
                    m_mapFirstVehicles[pVoie] = pVehicule;
                }
            }
            else
            {
                m_mapFirstVehicles[pVoie] = pVehicule;
            }
        }
    }
}

//=================================================================
    boost::shared_ptr<Vehicule> Reseau::GetFirstVehicule
//----------------------------------------------------------------
// Fonction  : Retourne le premier vÃ©hicule si il existe d'une voie
//              Ã  l'instant t
// Remarque  : Le premier vÃ©hicule est celui qui est le plus proche
//             de la sortie. Les vÃ©hicules guidÃ©s ne sont pas
//             pris en compte
// Version du: 30/09/2008
// Historique: 18/07/2006 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//             30/09/2008 (C.BÃ©carie - Tinea)
//             Ajout du boolean pour prise en compte ou non des
//             vÃ©hicules guidÃ©s
//=================================================================
(
    VoieMicro*   pVoie
)
{
    boost::shared_ptr<Vehicule> pVehicule;
    std::set<boost::shared_ptr<Vehicule>, LessVehiculePtr<Vehicule> >::iterator ItCurVeh;
    std::set<boost::shared_ptr<Vehicule>, LessVehiculePtr<Vehicule> >::iterator ItDebVeh;
    std::set<boost::shared_ptr<Vehicule>, LessVehiculePtr<Vehicule> >::iterator ItFinVeh;
    double                      dbPos;

    if(pVoie)
    {

        ItDebVeh = pVoie->GetLstVehiculesAnt().begin();
        ItFinVeh = pVoie->GetLstVehiculesAnt().end();

        dbPos = 0;

        for (ItCurVeh=ItDebVeh;ItCurVeh!=ItFinVeh;++ItCurVeh)
        {
            if(dbPos <= (*ItCurVeh)->GetPos(1) )
            {
                pVehicule = (*ItCurVeh);
                dbPos = pVehicule->GetPos(1);
            }
        }
    }

    return pVehicule;
}

//=================================================================
    boost::shared_ptr<Vehicule> Reseau::GetFirstVehiculePrecalc
//----------------------------------------------------------------
// Fonction  : idem que GetFirstVehicule mais en utilisant
//             les donnÃ©es de ComputeFirstVehicules.
// Remarque  : Permet d'Ãªtre plus performance Ã  condition
//             que les vÃ©hicules ne bougent pas. idÃ©al pour le
//             calcul des offres des voies
// Version du: 14/12/2012
// Historique: 14/12/2012 (O.Tonck - Ipsis)
//             CrÃ©ation
//=================================================================
(
    Voie*   pVoie
)
{
    boost::shared_ptr<Vehicule> pResult;
    map<Voie*,boost::shared_ptr<Vehicule>>::iterator iter = m_mapFirstVehicles.find(pVoie);
    if(iter != m_mapFirstVehicles.end())
    {
        pResult = iter->second;
    }
    return pResult;
}

//=================================================================
    boost::shared_ptr<Vehicule>   Reseau::GetLastVehicule
//----------------------------------------------------------------
// Fonction  : Retourne le dernier vÃ©hicule si il existe d'une voie
// Remarque  : Le dernier vÃ©hicule est celui qui est le plus proche
//             de l'entrÃ©e. Les bus ne sont pas considÃ©rÃ©s car cette
//             fonction est utlisÃ©e pour des calculs du flux au niveau
//             des voies
// Version du: 18/07/2006
// Historique: 18/07/2006 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//=================================================================
(
    VoieMicro*  pVoie,
    Vehicule * pVehiculeRef
)
{
    boost::shared_ptr<Vehicule> pVehicule;
    std::set<boost::shared_ptr<Vehicule>, LessVehiculePtr<Vehicule> >::iterator ItCurVeh;
    std::set<boost::shared_ptr<Vehicule>, LessVehiculePtr<Vehicule> >::iterator ItDebVeh;
    std::set<boost::shared_ptr<Vehicule>, LessVehiculePtr<Vehicule> >::iterator ItFinVeh;
    double                      dbPos;

    if(pVoie)
    {
        ItDebVeh = pVoie->GetLstVehiculesAnt().begin();
        ItFinVeh = pVoie->GetLstVehiculesAnt().end();

        dbPos = pVoie->GetLength();

        for (ItCurVeh=ItDebVeh;ItCurVeh!=ItFinVeh;ItCurVeh++)
        {

            if(dbPos >= (*ItCurVeh)->GetPos(1) )
            {
                // Les bus ne sont pas pris en compte
                if( (*ItCurVeh)->GetFleet() == GetSymuViaFleet() && pVehiculeRef != ItCurVeh->get() )
                {
                    pVehicule = (*ItCurVeh);
                    dbPos = pVehicule->GetPos(1);
                }
            }
        }
    }

    return pVehicule;
}

//=================================================================
    boost::shared_ptr<Vehicule> Reseau::GetNearAmontVehicule
//----------------------------------------------------------------
// Fonction  : Retourne le plus proche vÃ©hicule (si il existe) d'une
//             position donnÃ©e (en amont) au dÃ©but du pas de temps
// Remarque  :
// Version du: 31/08/2011
// Historique: 31/08/2011 (O.Tonck - IPSIS)
//             Ajout de la possibilitÃ© d'exclure un vÃ©hicule de la
//             recherche
//             21/08/2006 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//=================================================================
(
    VoieMicro*   pVoie,
    double  dbPos,
    double	dbDstMax, /* = -1 */
    Vehicule * pVeh
)
{
    vector<boost::shared_ptr<Vehicule> > lstVehicules;
    Tuyau*                      pTuyau;
    std::set<boost::shared_ptr<Vehicule>, LessVehiculePtr<Vehicule> >::iterator ItCurVeh;
    std::set<boost::shared_ptr<Vehicule>, LessVehiculePtr<Vehicule> >::iterator ItDebVeh;
    std::set<boost::shared_ptr<Vehicule>, LessVehiculePtr<Vehicule> >::iterator ItFinVeh;
    double                      dbPosTmp;
    double						dbPosMax;

    if(pVoie)
    {

        ItDebVeh = pVoie->GetLstVehiculesAnt().begin();
        ItFinVeh = pVoie->GetLstVehiculesAnt().end();

        dbPosTmp = -99999;

        pTuyau = (Tuyau*)pVoie->GetParent();

        for (ItCurVeh=ItDebVeh;ItCurVeh!=ItFinVeh;ItCurVeh++)
        {
            if(dbPos > (*ItCurVeh)->GetPos(1) && (*ItCurVeh)->GetPos(1) >= dbPosTmp && !(*ItCurVeh)->IsOutside() )
            {
                // si le vÃ©hicule n'est pas exclu de la recherche
                if(!pVeh || (pVeh->GetID() != (*ItCurVeh)->GetID()))
                {
                    // Anomalie nÂ°105 : gestion du cas oÃ¹ plusieurs vÃ©hicules se trouvent au mÃªme endroit (plusieurs followers potentiels)
                    if((*ItCurVeh)->GetPos(1) == dbPosTmp)
                    {
                        lstVehicules.push_back(*ItCurVeh);
                    }
                    else
                    {
                        lstVehicules.resize(1);
                        lstVehicules[0] = *ItCurVeh;
                        dbPosTmp = (*ItCurVeh)->GetPos(1);
                    }
                }
            }
        }

        dbPosMax = dbPos;
        if( dbDstMax > 0 && dbPosMax > dbDstMax)	// Distance max Ã  surveiller atteinte ?
            return GetFirstVehicule(lstVehicules);

        // Si pas de vÃ©hicule trouvÃ© et si la voie prÃ©cÃ©dente est connue
        if(lstVehicules.empty() && ((VoieMicro*)pVoie)->GetPrevVoie() )
        {
            pVoie = ((VoieMicro*)pVoie)->GetPrevVoie();
            dbPos = pVoie->GetLength()+0.001;

            ItDebVeh = pVoie->GetLstVehiculesAnt().begin();
            ItFinVeh = pVoie->GetLstVehiculesAnt().end();

            for (ItCurVeh=ItDebVeh;ItCurVeh!=ItFinVeh;ItCurVeh++)
            {
                if(dbPos >= (*ItCurVeh)->GetPos(1) && (*ItCurVeh)->GetPos(1) >= dbPosTmp )
                {
                    // si le vÃ©hicule n'est pas exclu de la recherche
                    if(!pVeh || (pVeh->GetID() != (*ItCurVeh)->GetID()))
                    {
                        // Anomalie nÂ°105 : gestion du cas oÃ¹ plusieurs vÃ©hicules se trouvent au mÃªme endroit (plusieurs followers potentiels)
                        if((*ItCurVeh)->GetPos(1) == dbPosTmp)
                        {
                            lstVehicules.push_back(*ItCurVeh);
                        }
                        else
                        {
                            lstVehicules.resize(1);
                            lstVehicules[0] = *ItCurVeh;
                            dbPosTmp = (*ItCurVeh)->GetPos(1);
                        }
                    }
                }
            }
        }

        dbPosMax = dbPos;
        if( dbDstMax > 0 && dbPosMax > dbDstMax)	// Distance max Ã  surveiller atteinte ?
            return GetFirstVehicule(lstVehicules);

        // Si pas de vÃ©hicule trouvÃ© et si la connexion amont possÃ¨de un seul tronÃ§on amont avec une seule voie
        if(!pTuyau->getConnectionAmont())
            return boost::shared_ptr<Vehicule>();

        if(lstVehicules.empty() && pTuyau->getConnectionAmont()->GetNbElAmont() == 1 && pTuyau->getConnectionAmont()->m_LstTuyAm.front()->getNb_voies()==1)
        {
            pVoie = (VoieMicro*)(pTuyau->getConnectionAmont()->m_LstTuyAm.front()->GetLstLanes()[0]);
            dbPos = pVoie->GetLength()+0.001;

            ItDebVeh = pVoie->GetLstVehiculesAnt().begin();
            ItFinVeh = pVoie->GetLstVehiculesAnt().end();

            for (ItCurVeh=ItDebVeh;ItCurVeh!=ItFinVeh;ItCurVeh++)
            {
                if(dbPos >= (*ItCurVeh)->GetPos(1) && (*ItCurVeh)->GetPos(1) >= dbPosTmp )
                {
                    // si le vÃ©hicule n'est pas exclu de la recherche
                    if(!pVeh || (pVeh->GetID() != (*ItCurVeh)->GetID()))
                    {
                        // Anomalie nÂ°105 : gestion du cas oÃ¹ plusieurs vÃ©hicules se trouvent au mÃªme endroit (plusieurs followers potentiels)
                        if((*ItCurVeh)->GetPos(1) == dbPosTmp)
                        {
                            lstVehicules.push_back(*ItCurVeh);
                        }
                        else
                        {
                            lstVehicules.resize(1);
                            lstVehicules[0] = *ItCurVeh;
                            dbPosTmp = (*ItCurVeh)->GetPos(1);
                        }
                    }
                }
            }
        }
    }

    return GetFirstVehicule(lstVehicules);
}

//=================================================================
    boost::shared_ptr<Vehicule>   Reseau::GetNearAmontVehiculeEx
//----------------------------------------------------------------
// Fonction  : Retourne le plus proche vÃ©hicule (si il existe) d'une
//             position donnÃ©e (en amont) au dÃ©but du pas de temps
//             La recherche s'effectue jusqu'Ã  une distance donnÃ©e
// Remarque  : La mÃ©thode vÃ©rifie que le vÃ©hicule trouvÃ© doit bien
//             passÃ© par le tronÃ§on donnÃ©
//             ATTENTION, la mÃ©thode surveille uniquement le premier
//             tronÃ§on aval de chaque connexion
// Version du: 21/08/2006
// Historique: 21/08/2006 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//=================================================================
(
    Tuyau*  pTa,
    double  dbPos,
    Tuyau*  pTIti,  // TronÃ§on de l'itinÃ©raire
    double  dbDst,   // Distance maximale Ã  regarder
    int     nVoie /* = 0 */
)
{
    boost::shared_ptr<Vehicule> pVeh;
    double      dbP = dbPos;
    double      dbD = dbPos;
    Tuyau*      pT = pTa;

    while(!pVeh)
    {
        pVeh = GetNearAmontVehicule( (VoieMicro*)pT->GetLstLanes()[nVoie], dbP );

        if(pVeh)
            if(!pVeh->IsItineraire(pTIti))      // pas le bon itinÃ©raire
                pVeh.reset();

        // OTK - ajout test de non nullitÃ© de pT->getConnectionAmont() pour Ã©viter un plantage si rÃ©seau "mal" dÃ©fini : tronÃ§on aval Ã  une brique sans mouvement autorisÃ©,
        // donc pas de connection amont ponctuelle ici...
        if (!pVeh && dbDst > dbD && pT->getConnectionAmont() && pT->getConnectionAmont()->m_LstTuyAm.size()>0)    // Recherche sur le tronÃ§on aval si la distance max n'a pas Ã©tÃ© atteinte
        {
            // OTK - remarque : pas terrible de ne regarder que sur le premier tronÃ§on amont venu, si ?
            pT = pT->getConnectionAmont()->m_LstTuyAm.front();
            // Correction anomalie nÂ°110 : plantage si le tronÃ§on amont a moins de voies que nVoie
            if(pT && pT->getNbVoiesDis() > nVoie)
            {
                dbP = pT->GetLstLanes()[nVoie]->GetLength();
                dbD = dbD + pT->GetLstLanes()[nVoie]->GetLength();
            }
            else
                return boost::shared_ptr<Vehicule>();
        }
        else
            return pVeh;
    }

    return pVeh;
}


//=================================================================
    boost::shared_ptr<Vehicule> Reseau::GetNearAvalVehicule
//----------------------------------------------------------------
// Fonction  : Retourne le plus proche vÃ©hicule (si il existe) d'une
//             position donnÃ©e (en aval) au dÃ©but du pas de temps
// Remarque  :
// Version du: 31/08/2011
// Historique: 31/08/2011 (O.Tonck - IPSIS)
//             Ajout de la possibilitÃ© de l'exclusion d'un vÃ©hicule
//             de la recherche
//             21/08/2006 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//=================================================================
(
    VoieMicro*   pVoie,
    double  dbPos,
    Vehicule*   pVeh,
    bool    bIgnoreVehiculesenDepassement /* = false */
)
{
    vector<boost::shared_ptr<Vehicule> >  lstVehicules;
    std::set<boost::shared_ptr<Vehicule>, LessVehiculePtr<Vehicule> >::iterator ItCurVeh;
    std::set<boost::shared_ptr<Vehicule>, LessVehiculePtr<Vehicule> >::iterator ItDebVeh;
    std::set<boost::shared_ptr<Vehicule>, LessVehiculePtr<Vehicule> >::iterator ItFinVeh;
    double                      dbPosVeh;

    if(pVoie)
    {

        ItDebVeh = pVoie->GetLstVehiculesAnt().begin();
        ItFinVeh = pVoie->GetLstVehiculesAnt().end();

        dbPosVeh = pVoie->GetLength() + 0.00001;

        Vehicule * pCurVeh;

        for (ItCurVeh=ItDebVeh;ItCurVeh!=ItFinVeh;ItCurVeh++)
        {
            pCurVeh = (*ItCurVeh).get();

            if(pCurVeh->GetPos(1) >= dbPos && pCurVeh->GetPos(1) <= dbPosVeh)
            {
                // si le vÃ©hicule n'est pas exclu de la recherche
                if(!pVeh || (pVeh->GetID() != pCurVeh->GetID()))
                {
                    if( !pCurVeh->IsOutside() && (!bIgnoreVehiculesenDepassement || !pCurVeh->IsDepassement()))    // Le vÃ©hicule ne doit pas Ãªtre en retrait de la voie
                    {
                        // Anomalie nÂ°105 : gestion du cas oÃ¹ plusieurs vÃ©hicules se trouvent au mÃªme endroit (plusieurs leaders potentiels)
                        if(pCurVeh->GetPos(1) == dbPosVeh)
                        {
                            lstVehicules.push_back(*ItCurVeh);
                        }
                        else
                        {
                            lstVehicules.resize(1);
                            lstVehicules[0] = *ItCurVeh;
                            dbPosVeh = pCurVeh->GetPos(1);
                        }
                    }
                }
            }
        }
    }

    return GetLastVehicule(lstVehicules);
}

//=================================================================
    boost::shared_ptr<Vehicule> Reseau::GetPrevVehicule
//----------------------------------------------------------------
// Fonction  : Retourne le vÃ©hicule prÃ©cedent d'un vÃ©hicule sur une
//             voie
// Remarque  :
// Version du: 21/08/2006
// Historique: 21/08/2006 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//=================================================================
(
   Vehicule*                   pVehicule,
   bool                        bSearchNextVoie, /*=false*/
   bool                        bExcludeVehiculeDepasse /*=false*/
)
{
    std::vector<boost::shared_ptr<Vehicule> > lstVehicules;
    std::set<boost::shared_ptr<Vehicule>, LessVehiculePtr<Vehicule> >::iterator ItCurVeh;
    std::set<boost::shared_ptr<Vehicule>, LessVehiculePtr<Vehicule> >::iterator ItDebVeh;
    std::set<boost::shared_ptr<Vehicule>, LessVehiculePtr<Vehicule> >::iterator ItFinVeh;
    double                      dbPosVeh;
    VoieMicro*                  pVoie;
    Tuyau*                      pNextTuyau;
    Vehicule*                   pCurVeh;

    pNextTuyau = NULL;

    pVoie = pVehicule->GetVoie(1);
    if(!pVoie)
        return boost::shared_ptr<Vehicule>();

    int nVoie = pVoie->GetNum();  // Voie initiale :
    // utilisÃ©e dans le cas d'un convergent d'un giratoire Ã  plusieurs voies oÃ¹ la recherche
    // s'effectue Ã©galement en amont du convergent si aucun leader n'a Ã©tÃ© trouvÃ© (dans ce cas, on respecte
    // le fait que le vÃ©hicule qui arrive au point de conflit sur la voie de droite (respectivement gauche)
    // se dirige obligatoirement vers la voie externe (respectivement interne) de l'anneau du giratoire

    dbPosVeh = pVoie->GetLength()+1;

    ItDebVeh = pVoie->GetLstVehiculesAnt().begin();
    ItFinVeh = pVoie->GetLstVehiculesAnt().end();

    for (ItCurVeh=ItDebVeh;ItCurVeh!=ItFinVeh;ItCurVeh++)
    {
        pCurVeh = (*ItCurVeh).get();
        //if((*ItCurVeh)->GetPos(1) > pVehicule->GetPos(1) && (*ItCurVeh)->GetPos(1) < dbPosVeh )
        //if(pCurVeh->GetPos(1) > pVehicule->GetPos(1) && pCurVeh->GetPos(1) <= dbPosVeh && pCurVeh !=  pVehicule )  // 14/10/08
        if((pCurVeh->GetPos(1) > pVehicule->GetPos(1) || (pVehicule->GetPos(0) == UNDEF_POSITION && pCurVeh->GetPos(1) == pVehicule->GetPos(1)))
            && pCurVeh->GetPos(1) <= dbPosVeh && pCurVeh !=  pVehicule )  // 01/04/2015 - bug si un vÃ©hicule en attente Ã  une entrÃ©e est crÃ©Ã© : il ne voit pas pour leader le vÃ©hicule prÃªt qui a dÃ©marrÃ© avant lui
        {
            if( pCurVeh != pVehicule && !pCurVeh->IsOutside() // Si le vÃ©hicule est en retrait, il n'est pas pris en compte
                && (!bExcludeVehiculeDepasse || ((pCurVeh->IsDepassement() == pVehicule->IsDepassement()))))
            {
                // Anomalie nÂ°105 : gestion du cas oÃ¹ plusieurs vÃ©hicules se trouvent au mÃªme endroit (plusieurs leaders potentiels)
                if(pCurVeh->GetPos(1) == dbPosVeh)
                {
                    lstVehicules.push_back(*ItCurVeh);
                }
                else
                {
                    lstVehicules.resize(1);
                    lstVehicules[0] = *ItCurVeh;
                    dbPosVeh = pCurVeh->GetPos(1);
                }
            }
        }
    }


    if(lstVehicules.empty() && bSearchNextVoie && pVehicule->GetLink(1)->get_Type_aval() != 'S' && pVehicule->GetLink(1)->get_Type_aval() != 'P')
        pNextTuyau = pVehicule->GetLink(1);

    // On regarde la voie suivante dans le cas d'un convergent
    // OTK - 21/06/2013 - choses bizarres + bug constatÃ© ici lorsque la voie courante est un tuyau interne qui dÃ©bouche sur la voie de gauche d'un tronÃ§on a deux voies.
    // Dans ce cas, si un vÃ©hicule se trouve sur le tronÃ§on aval mais voie de droite, il sera retenu, alors qu'il ne devrait pas l'Ãªtre (il sera positionnÃ© comme leader
    // du vÃ©hicule courant alors que ces deux vÃ©hicules ne seront pas sur la mÃªme voie (cf. convergent.cpp, procÃ©dure d'insertion, on force le leader comme Ã©tant le rÃ©sultat de cette fonction).
    if(bSearchNextVoie && lstVehicules.empty() && pVehicule->GetLink(1)->get_Type_aval() == 'C')
    {
        pNextTuyau = pVehicule->GetLink(1)->getConnectionAval()->m_LstTuyAv.front();

        // OTK - 04/09/2012 - correction plantage si le tuyau aval du convergent a moins de voies que le numÃ©ro de la voie courante sur le troncon amont
        //pVoie = (VoieMicro*)pNextTuyau->GetLstLanes()[nVoie];
        // Anomalie nÂ°102 : utilisation de CalculNextVoie pour vÃ©iter les erreurs
        pVoie = (VoieMicro*)pVehicule->CalculNextVoie(pVoie, m_dbInstSimu);
        if(!pVoie)
        {
            pVoie = (VoieMicro*)pNextTuyau->GetLstLanes()[min(nVoie,(int)pNextTuyau->GetLstLanes().size()-1)];
        }

        dbPosVeh = pVoie->GetLength()+1;

        ItDebVeh = pVoie->GetLstVehiculesAnt().begin();
        ItFinVeh = pVoie->GetLstVehiculesAnt().end();

        // On rÃ©itÃ¨re l'algo de recherche
        for (ItCurVeh=ItDebVeh;ItCurVeh!=ItFinVeh;ItCurVeh++)
        {
            pCurVeh = (*ItCurVeh).get();

            if(pCurVeh->GetPos(1) <= dbPosVeh && pCurVeh->GetPos(1) >= 0)
            {
                if( pCurVeh != pVehicule
                    && (!bExcludeVehiculeDepasse || (pCurVeh->IsDepassement() == pVehicule->IsDepassement())))
                {
                    // Anomalie nÂ°105 : gestion du cas oÃ¹ plusieurs vÃ©hicules se trouvent au mÃªme endroit (plusieurs leaders potentiels)
                    if(pCurVeh->GetPos(1) == dbPosVeh)
                    {
                        lstVehicules.push_back(*ItCurVeh);
                    }
                    else
                    {
                        lstVehicules.resize(1);
                        lstVehicules[0] = *ItCurVeh;
                        dbPosVeh = pCurVeh->GetPos(1);
                    }
                }
            }
        }
    }

    // Encore une fois ou les autres connexions
    std::deque<Tuyau*>::iterator itT;
    if(bSearchNextVoie && lstVehicules.empty() && pNextTuyau && pNextTuyau->getConnectionAval())
    {


        for(itT = pNextTuyau->getConnectionAval()->m_LstTuyAv.begin(); itT != pNextTuyau->getConnectionAval()->m_LstTuyAv.end(); itT++)
        {
            Tuyau* pNextNextTuyau = (*itT);

            if( pVehicule->IsItineraire(pNextNextTuyau) ) // Ce tronÃ§on est sur l'itinÃ©raire du vÃ©hicule ?
            {
                // Anomalie nÂ°102 : utilisation de CalculNextVoie pour vÃ©iter les erreurs
                pVoie = (VoieMicro*)pVehicule->CalculNextVoie(pVoie, m_dbInstSimu);
                if(!pVoie)
                {
                    pVoie = (VoieMicro*)pNextNextTuyau->GetLstLanes()[min(nVoie,pNextNextTuyau->getNb_voies()-1)];    // Une seule voie pour les convergents
                }


                dbPosVeh = pVoie->GetLength()+1;

                ItDebVeh = pVoie->GetLstVehiculesAnt().begin();
                ItFinVeh = pVoie->GetLstVehiculesAnt().end();

                // On rÃ©itÃ¨re l'algo de recherche
                for (ItCurVeh=ItDebVeh;ItCurVeh!=ItFinVeh;ItCurVeh++)
                {
                    pCurVeh = (*ItCurVeh).get();

                    if(pCurVeh->GetPos(1) <= dbPosVeh && pCurVeh->GetPos(1) >= 0)
                    {
                        if( pCurVeh != pVehicule
                            && (!bExcludeVehiculeDepasse || (pCurVeh->IsDepassement() == pVehicule->IsDepassement())))
                        {
                            // Anomalie nÂ°105 : gestion du cas oÃ¹ plusieurs vÃ©hicules se trouvent au mÃªme endroit (plusieurs leaders potentiels)
                            if(pCurVeh->GetPos(1) == dbPosVeh)
                            {
                                lstVehicules.push_back(*ItCurVeh);
                            }
                            else
                            {
                                lstVehicules.resize(1);
                                lstVehicules[0] = *ItCurVeh;
                                dbPosVeh = pCurVeh->GetPos(1);
                            }
                        }
                    }
                }
            }
        }
    }

    // Anomalie nÂ°105 - si plusieurs leaders potentiels au mÃªme endroit, on renvoie celui pour qui le follower n'est pas dans la liste
    return GetLastVehicule(lstVehicules);
}

//=================================================================
    void    Reseau::MiseAJourVehicules
//----------------------------------------------------------------
// Fonction  : Met Ã  jour les variables de simulation des vÃ©hciules
// Remarque  :
// Version du: 18/07/2006
// Historique: 18/07/2006 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//=================================================================
(
    double dbInstant
)
{
    std::vector <boost::shared_ptr<Vehicule>>::iterator ItCurVeh;
    std::vector <boost::shared_ptr<Vehicule>>::iterator ItDebVeh;
    std::vector <boost::shared_ptr<Vehicule>>::iterator ItFinVeh;

    ItDebVeh = m_LstVehicles.begin();
    ItFinVeh = m_LstVehicles.end();


    // Calcul des caractÃ©ritiques du trafic vÃ©hicule par vÃ©hicule

    //*m_pFicSimulation << std::endl << "MAJ vÃ©hicules du rÃ©seau : " << std::endl;
    for (ItCurVeh=ItDebVeh;ItCurVeh!=ItFinVeh;ItCurVeh++)
    {
        if(!(*ItCurVeh)->IsMeso())
        {
            (*ItCurVeh)->CalculTraficEx(dbInstant);
        }
        else
        {
            // rÃ©initialisation de l'Ã©ventuelle voie dÃ©sirÃ©e pour les vÃ©hicules meso (sinon la voie dÃ©sirÃ©e
            // est conservÃ©e jusqu'au retour sur un tronÃ§on micro qui peut avoir moins de voie que l'index de la voie dÃ©sirÃ©e anciennement...)
            // rmq. : si le vÃ©hicule est hors voie on conserve la voie dÃ©sirÃ©e (sinon insertion impossible de bus si on passe ici avant insertion)
            if(!(*ItCurVeh)->IsOutside())
            {
                (*ItCurVeh)->ReinitVoieDesiree();
            }
        }
    }
    //*m_pFicSimulation << std::endl  << std::endl;
}

//=================================================================
    void    Reseau::FinCalculVehicules
//----------------------------------------------------------------
// Fonction  : Fin du calcul des vÃ©hicules
// Remarque  :
// Version du: 29/08/2008
// Historique: 29/08/2008 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//=================================================================
(
    double dbInst
)
{
    std::vector <boost::shared_ptr<Vehicule>>::iterator ItCurVeh;
    std::vector <boost::shared_ptr<Vehicule>>::iterator ItDebVeh;
    std::vector <boost::shared_ptr<Vehicule>>::iterator ItFinVeh;

    ItDebVeh = m_LstVehicles.begin();
    ItFinVeh = m_LstVehicles.end();

    for (ItCurVeh=ItDebVeh;ItCurVeh!=ItFinVeh;ItCurVeh++)
    {
        if((*ItCurVeh)->GetCurrentTuyauMeso() == NULL || ((*ItCurVeh)->GetCurrentTuyauMeso()->GetType() != Tuyau::TT_MESO))// tuyau courant n'est pas meso

        {
            (*ItCurVeh)->FinCalculTrafic(dbInst);
        }
    }
}

//=================================================================
    void    Reseau::InitVehicules
//----------------------------------------------------------------
// Fonction  : Met Ã  jour les variables de simulation des vÃ©hciules
// Remarque  :
// Version du: 18/07/2006
// Historique: 18/07/2006 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//=================================================================
(
    double dbInstant
)
{
    std::vector <boost::shared_ptr<Vehicule>>::iterator ItCurVeh;
    std::vector <boost::shared_ptr<Vehicule>>::iterator ItDebVeh;
    std::vector <boost::shared_ptr<Vehicule>>::iterator ItFinVeh;

    VoieMicro * pVoieMicro;
    TuyauMicro * pTuyMicro;
    CTuyauMeso * pTuyMeso;

    // nettoyage des listes de vÃ©hicules pour chaque voie du rÃ©seau (les listes seront actualisÃ©es
    // en y ajoutant chaque vÃ©hicule dans la boucle suivante)
    for(size_t iTuy = 0; iTuy < m_LstTuyauxMicro.size(); iTuy++)
    {
        pTuyMicro =  m_LstTuyauxMicro[iTuy];
        vector<Voie*> & lstVoies = pTuyMicro->GetLstLanes();
        for(size_t iVoie = 0; iVoie < lstVoies.size(); iVoie++)
        {
            ((VoieMicro*)lstVoies[iVoie])->GetLstVehiculesAnt().clear();
        }
    }
    // Il faut aussi le faire pour les tuyaux meso
    for(size_t iTuy = 0; iTuy < m_LstTuyauxMeso.size(); iTuy++)
    {
        pTuyMeso =  m_LstTuyauxMeso[iTuy];
        vector<Voie*> & lstVoies = pTuyMeso->GetLstLanes();
        for(size_t iVoie = 0; iVoie < lstVoies.size(); iVoie++)
        {
            ((VoieMicro*)lstVoies[iVoie])->GetLstVehiculesAnt().clear();
        }
    }


    ItDebVeh = m_LstVehicles.begin();
    ItFinVeh = m_LstVehicles.end();

    // Init des variables de simu pour le pas de temps
    for (ItCurVeh=ItDebVeh;ItCurVeh!=ItFinVeh;ItCurVeh++)
    {


        (*ItCurVeh)->DecalVarTrafic();

        // Ajout du vÃ©hicule Ã  la liste de la voie correspondante
        pVoieMicro = (*ItCurVeh)->GetVoie(1);
        if(pVoieMicro != NULL && !(*ItCurVeh)->IsMeso())
        {
            pVoieMicro->GetLstVehiculesAnt().insert(*ItCurVeh);
        }

        if( (*ItCurVeh)->GetLink(1) )
        {
            if(m_bSimuTrafic)
            {
                // Calcul et affectation du tuyau suivant si il n'a pas Ã©tÃ© encore calculÃ©
                if( (*ItCurVeh)->GetNextTuyau() == NULL)
                    (*ItCurVeh)->SetNextTuyau( (*ItCurVeh)->CalculNextTuyau((*ItCurVeh)->GetLink(1), dbInstant) );


                // Calcul et affectation des voies possibles du tuyau courant pour le vÃ©hicule en fonction du tuyau suivant
                (*ItCurVeh)->CalculVoiesPossibles(dbInstant);

                (*ItCurVeh)->CalculVoieDesiree(dbInstant);
            }
        }
        // Remise Ã  0 de l'instant d'entrÃ©e
        (*ItCurVeh)->SetInstantEntree(-1);
    }

    //*(Reseau::m_pFicSimulation) << std::endl;
}

//=================================================================
    void    Reseau::SupprimeVehicules
//----------------------------------------------------------------
// Fonction  : Supprime les vÃ©hicules qui sont sortis du rÃ©seau
// Remarque  :
// Version du: 18/07/2006
// Historique: 18/07/2006 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//=================================================================
(
    double dbInstant
)
{
    boost::shared_ptr<Vehicule>                         pVehicule;
    std::vector <boost::shared_ptr<Vehicule>>::iterator  ItCurVeh;
    std::vector <boost::shared_ptr<Vehicule>>::iterator  ItDebVeh;
    std::vector <boost::shared_ptr<Vehicule>>::iterator  ItFinVeh;

    std::vector <std::vector <boost::shared_ptr<Vehicule>>::iterator >  vehiculesToRemove;

    std::string sSortie;

    std::vector<boost::shared_ptr<Vehicule>> LstVehiculeNew;

    boost::lock_guard<boost::mutex> guard(*m_pMutex);

    ItDebVeh = m_LstVehicles.begin();
    ItFinVeh = m_LstVehicles.end();

    // Suppression des vehicules sortis
    for (ItCurVeh=ItDebVeh;ItCurVeh!=ItFinVeh;ItCurVeh++)
    {
        pVehicule = (*ItCurVeh);

        if( !pVehicule->GetLink(0) && pVehicule->GetCurrentTuyauMeso() == NULL )
        {
            std::string sOrig;
            if( pVehicule->GetOrigine() )
                sOrig = pVehicule->GetOrigine()->GetOutputID();

            std::string sDest;
            TripNode * pDest = pVehicule->GetTrip()->GetFinalDestination();
            if (pDest)
            {
                if (pDest->GetInputPosition().IsLink())
                {
                    sDest = pDest->GetInputPosition().GetLink()->GetCnxAssAm()->GetID();
                }
                else
                {
                    sDest = pDest->GetInputID();
                }
            }
            else if (!pVehicule->GetTuyauxParcourus().empty() && pVehicule->GetTuyauxParcourus().back()->GetCnxAssAv())
            {
                sDest = pVehicule->GetTuyauxParcourus().back()->GetCnxAssAv()->GetID();
            }

            if( pVehicule->IsForcedDriven() )
                sDest = pVehicule->GetLink(1)->GetLabel();

#ifdef USE_SYMUCORE
            if (m_pSymuMasterUserHandler && pVehicule->GetExternalID() != -1)
            {
                m_pSymuMasterUserHandler->OnPPathCompleted(pVehicule->GetExternalID(), fmod(pVehicule->GetExitInstant(), pas_de_temps));
            }
#endif

            // Ecriture de la sortie et de l'instant de sortie
            if(m_bSimuTrafic)
            {
                std::map<std::string, std::string> additionalAttributes = pVehicule->GetFleet()->GetOutputAttributesAtEnd(pVehicule.get());

                std::deque<TraceDocTrafic*> doctrafics = GetXmlDocTrafic();
                std::deque<TraceDocTrafic* >::iterator itDocTraf;
                for( itDocTraf = doctrafics.begin(); itDocTraf != doctrafics.end(); itDocTraf++)
                {
					(*itDocTraf)->AddVehiculeToSortieNode(pVehicule->GetID(), sDest, pVehicule->GetExitInstant(), pVehicule->GetVit(1));
                    (*itDocTraf)->UpdateInstSortieVehicule(pVehicule->GetID(), pVehicule->GetExitInstant(), sDest, pVehicule->GetDstCumulee(), pVehicule->GetTuyauxParcourus(), additionalAttributes);
                }
            }

            if(m_bSimuAir && m_XmlDocAtmo)
            {
                m_XmlDocAtmo->AddVehicule( pVehicule->GetID(), pVehicule->GetType()->GetLabel(), sOrig, pVehicule->GetInstantEntree(), sDest, pVehicule->GetExitInstant(),
                    pVehicule->GetCumCO2(), pVehicule->GetCumNOx(), pVehicule->GetCumPM(), pVehicule->GetDstCumulee(), true );
            }

            // Mise Ã  jour de la matrice OD du rÃ©seau
            if(m_bSimuTrafic && m_bDebugOD)
            {
                if(pVehicule->GetFleet() == GetSymuViaFleet() && !IsUsedODMatrix() && m_bSimuTrafic)
                {
                    SymuViaTripNode * pOrigine = dynamic_cast<SymuViaTripNode*>(pVehicule->GetOrigine());
                    SymuViaTripNode * pDestination = dynamic_cast<SymuViaTripNode*>(pVehicule->GetDestination());
                    pOrigine->MAJLogMatriceOD(false,pVehicule->GetHeureEntree()+m_dbDiffDebutSimuData,pDestination);
                }

                if(m_bLogFile && pVehicule->GetFleet() == GetPublicTransportFleet())
                {
                    // Trace de l'heure de sortie du bus
                    *g_FicDebug << "Bus " << pVehicule->GetID() << " " << pVehicule->GetTrip()->GetID() << " Created at " << pVehicule->GetHeureEntree() << " Inserted at " << pVehicule->GetInstInsertion() <<" Exit at "<< dbInstant << endl;
                }
            }

            if (IsCptItineraire())
            {
                TypeVehicule * pTV = (*ItCurVeh)->GetType();
                m_pModuleAffectation->IncreaseNBVehRecus(pTV, sOrig, sDest);
            }

            // Mesure du temps passÃ© en zone pour SymuMaster :
#ifdef USE_SYMUCORE
            if (IsSymuMasterMode())
            {
                ZoneDeTerminaison * pZoneDest = dynamic_cast<ZoneDeTerminaison*>(pVehicule->GetDestination());
                if (pZoneDest)
                {
                    SymuViaFleetParameters * pFleetParams = dynamic_cast<SymuViaFleetParameters*>(pVehicule->GetFleetParameters());
                    if (pFleetParams)
                    {
                        std::pair< TypeVehicule*, std::pair<std::pair<double, double>, double> > zoneTerminationTiming = std::make_pair(pVehicule->GetType(), std::make_pair(std::make_pair(pVehicule->GetExitInstant(), pVehicule->GetExitInstant() - pFleetParams->GetInstantEntreeZone()), pFleetParams->GetDistanceParcourue()));
                        pZoneDest->m_AreaHelper.GetMapArrivingVehicles()[pFleetParams->GetZoneDestinationInputJunction()][pVehicule->GetID()] = zoneTerminationTiming;
                        if (pFleetParams->GetPlaqueDestination())
                        {
                            pFleetParams->GetPlaqueDestination()->m_AreaHelper.GetMapArrivingVehicles()[pFleetParams->GetZoneDestinationInputJunction()][pVehicule->GetID()] = zoneTerminationTiming;
                        }
                    }
                }
            }
#endif // USE_SYMUCORE



            // Supprime des structure meso
     /*       std::deque<Tuyau *>::const_iterator itItineraire;
            for(itItineraire = (*ItCurVeh)->GetItineraire()->begin();
                itItineraire !=  (*ItCurVeh)->GetItineraire()->end(); itItineraire++)
            {

                // remove downstream passing time
               Connexion * pCon = (*itItineraire)->getConnectionAmont();

               if( !pCon)
               {
                   pCon = (*itItineraire)->GetBriqueAmont();
               }

               if(pCon)
               {
                  if( pCon->IsAnOrigine())
                  {
                     pCon->DecreaseRankNextIncommingVeh(NULL ) ;
                     pCon->RemoveFirstUpstreamPassingTime(NULL);
                  }
                  pCon->DecreaseRankNextOutgoingVeh((*itItineraire));
                  pCon->RemoveFirstDownStreamPassingTime((*itItineraire));
              //       pCon->DecreaseRankNextIncommingVeh( (*itItineraire) ) ;
              //     pCon->RemoveFirstUpstreamPassingTime( (*itItineraire));
               }
               // remove upstream passing time
               pCon =(*itItineraire)->getConnectionAval();
               if( !pCon)
               {
                   pCon = (*itItineraire)->GetBriqueAval();
               }
               if( pCon)
               {
                   if( pCon->IsADestination())
                  {
                     pCon->DecreaseRankNextOutgoingVeh(NULL);
                    pCon->RemoveFirstDownStreamPassingTime(NULL);
                  }
                   pCon->DecreaseRankNextIncommingVeh( (*itItineraire) ) ;
                   pCon->RemoveFirstUpstreamPassingTime( (*itItineraire));
           //         pCon->DecreaseRankNextOutgoingVeh((*itItineraire));
             //      pCon->RemoveFirstDownStreamPassingTime((*itItineraire));
               }

            }

            */

            vehiculesToRemove.push_back(ItCurVeh);
        }
    }

    //std::vector <std::vector <boost::shared_ptr<Vehicule>>::iterator>::iterator  ItDebVehit;
    //std::vector <std::vector <boost::shared_ptr<Vehicule>>::iterator>::iterator  ItFinVehit;
    //std::vector <std::vector <boost::shared_ptr<Vehicule>>::iterator>::iterator  ItCurVehit;
    //ItDebVehit = vehiculesToRemove.begin();
    //ItFinVehit = vehiculesToRemove.end();
    for (int i = (int) vehiculesToRemove.size()-1; i>=0; --i )
    {
        ItCurVeh = vehiculesToRemove[i];
        pVehicule = *ItCurVeh;
        pVehicule->GetFleet()->OnVehicleDestroyed(pVehicule);

        for (std::set<TankSensor*>::const_iterator iterTank = pVehicule->getReservoirs().begin(); iterTank != pVehicule->getReservoirs().end(); iterTank++)
        {
            (*iterTank)->VehicleDestroyed(pVehicule.get());
        }

        m_LstVehicles.erase(ItCurVeh);
    }
   /* ItDebVeh = m_LstVehicles.begin();
    ItFinVeh = m_LstVehicles.end();
    for (ItCurVeh=ItDebVeh;ItCurVeh!=ItFinVeh;ItCurVeh++)
    {
        pVehicule = (*ItCurVeh);
        if(pVehicule)
            LstVehiculeNew.push_back(pVehicule);
    }

    m_LstVehicles.clear();

    ItDebVeh = LstVehiculeNew.begin();
    ItFinVeh = LstVehiculeNew.end();
    for (ItCurVeh=ItDebVeh;ItCurVeh!=ItFinVeh;ItCurVeh++)
    {
        pVehicule = (*ItCurVeh);
        m_LstVehicles.push_back(pVehicule);
    }*/
}

//=================================================================
    void    Reseau::ChangementDeVoie
//----------------------------------------------------------------
// Fonction  : ProcÃ©dure de changement de voie des vÃ©hicules du rÃ©seau
// Remarque  :
// Version du: 12/09/2006
// Historique: 12/09/2006 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//=================================================================
(
    double dbInstant
)
{
    std::vector <boost::shared_ptr<Vehicule>>::iterator ItCurVeh;
    std::vector <boost::shared_ptr<Vehicule>>::iterator ItDebVeh;
    std::vector <boost::shared_ptr<Vehicule>>::iterator ItFinVeh;
    VoieMicro    *pVoie;


    ItDebVeh = m_LstVehicles.begin();
    ItFinVeh = m_LstVehicles.end();

    // Init des variables de simu du changement de voie
    for (ItCurVeh=ItDebVeh;ItCurVeh!=ItFinVeh;ItCurVeh++)
    {
        (*ItCurVeh)->InitChgtVoie();
    }

    // Le calcul s'effectue pour chaque vÃ©hicule
    // PremiÃ¨re passe pour changement de voie obligatoire (rÃ©duction de voie ou prise de direction)
    // Ceci est effectuÃ©e lors d'une premiÃ¨re passe afin de rendre prioritaire ce type de changement de voie
    // par rapport Ã  de la gÃ¨ne ou rabattement
    // Les vÃ©hicules pris en compte au cours de cette passe ne sont pas reconsidÃ©rÃ© dans la suite (calcul des autres types
    // de changement de voie)
    for (ItCurVeh=ItDebVeh;ItCurVeh!=ItFinVeh;ItCurVeh++)
    {

        pVoie = (*ItCurVeh)->GetVoie(1);
        if(pVoie && IsChgtVoie() )
        {
            // Changement de voie obligatoire
            if( (*ItCurVeh)->GetVoieDesiree() != -1)
            {
                bool bChtSuccess = (*ItCurVeh)->CalculChangementVoie( (*ItCurVeh)->GetVoieDesiree(), dbInstant+m_dbDiffDebutSimuData, m_pGestionsCapteur, true);
                (*ItCurVeh)->SetChtgVoie(true); // Pas d'autre changement de voie Ã  tester
                if(bChtSuccess)
                {
                    (*ItCurVeh)->ReinitVoieDesiree();
                }
            }
        }
    }

    // Le calcul s'effectue pour chaque vÃ©hicule
    for (ItCurVeh=ItDebVeh;ItCurVeh!=ItFinVeh;ItCurVeh++)
    {
        pVoie = (*ItCurVeh)->GetVoie(1);

        if(pVoie && IsChgtVoie() && !(*ItCurVeh)->IsChgtVoie() && (*ItCurVeh)->GetLink(1) )		// Le vÃ©hicule ne doit pas dÃ©jÃ  avoir dÃ©jÃ  essayÃ© de changer de voie pour rÃ©duction de voie ou pour direction
        {
            // Changement de voie pour gÃ¨ne
            int nV = (*ItCurVeh)->GetVoie(1)->GetNum();

            // Ordre de traitement des voies adjacentes
            int nVF, nVS;

            // Par dÃ©faut, voie Ã  gauche puis Ã  droite (m_nOrdreChgtVoieDiscr=1)
            nVF = nV+1;
            nVS = nV-1;

            if( m_nOrdreChgtVoieDiscr == 2 )
            {
                nVF = nV-1;
                nVS = nV+1;
            }

            if( m_nOrdreChgtVoieDiscr == 3 )	// Voie la plus favorable
            {
                double dbVit1 = (*ItCurVeh)->CalculVitesseVoieAdjacente( nV+1, dbInstant+m_dbDiffDebutSimuData );
                double dbVit2 = (*ItCurVeh)->CalculVitesseVoieAdjacente( nV-1, dbInstant+m_dbDiffDebutSimuData );

                if( dbVit1 <= dbVit2)
                {
                    nVF = nV+1;
                    nVS = nV-1;
                }
                else
                {
                    nVF = nV-1;
                    nVS = nV+1;
                }
            }

            if( nVF >= 0 && nVF < (*ItCurVeh)->GetLink(1)->getNbVoiesDis() )
            {
                if( (*ItCurVeh)->IsVoiePossible( nVF ) )   // la voie est-elle candidate ?
                    if( (*ItCurVeh)->CalculChangementVoie( nVF, dbInstant+m_dbDiffDebutSimuData, m_pGestionsCapteur) )
                        continue;
            }

            if( nVS >= 0 && nVS < (*ItCurVeh)->GetLink(1)->getNbVoiesDis() )
            {
                if( (*ItCurVeh)->IsVoiePossible( nVS ) )   // la voie est-elle candidate ?
                    if( (*ItCurVeh)->CalculChangementVoie( nVS, dbInstant+m_dbDiffDebutSimuData, m_pGestionsCapteur) )
                        continue;
            }

            // Dernier type de changement de voie : pour se rabattre
            if( (*ItCurVeh)->GetVoie(1)->GetNum() > 0 )
                if( (*ItCurVeh)->IsVoiePossible( (*ItCurVeh)->GetVoie(1)->GetNum()-1 ) )   // la voie est-elle candidate ?
                    if( (*ItCurVeh)->CalculChangementVoie( (*ItCurVeh)->GetVoie(1)->GetNum()-1, dbInstant+m_dbDiffDebutSimuData, m_pGestionsCapteur, false, true) )
                        continue;
        }
    }
}


//=================================================================
    void    Reseau::Depassement
//----------------------------------------------------------------
// Fonction  : ProcÃ©dure de dÃ©passement sur tronÃ§on opposÃ©
// Remarque  :
// Version du: 14/10/2011
// Historique: 14/10/2011 (O.Tonck - Ipsis)
//             CrÃ©ation
//=================================================================
(
    double dbInstant
)
{
    std::vector <boost::shared_ptr<Vehicule>>::iterator ItCurVeh;
    std::vector <boost::shared_ptr<Vehicule>>::iterator ItDebVeh;
    std::vector <boost::shared_ptr<Vehicule>>::iterator ItFinVeh;

    ItDebVeh = m_LstVehicles.begin();
    ItFinVeh = m_LstVehicles.end();

    for (ItCurVeh=ItDebVeh;ItCurVeh!=ItFinVeh;ItCurVeh++)
    {
        if( (*ItCurVeh)->GetLink(1) && (*ItCurVeh)->GetLink(1)->GetTuyauOppose() != NULL ) // Pour les vÃ©hicules crÃ©Ã©s avant le pas de temps courant
        {
            (*ItCurVeh)->CalculDepassement(dbInstant);
        }
    }
}

//=================================================================
    void    Reseau::AddVehicule
//----------------------------------------------------------------
// Fonction  : Ajout d'un vÃ©hicule Ã  la liste
// Remarque  :
// Version du: 10/11/2006
// Historique: 10/11/2006
//              La rÃ©allocation des vecteurs de stockage se fait ici
//             21/09/2006 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//=================================================================
(
    boost::shared_ptr<Vehicule> pVehicule
)
{
#ifdef USE_SYMUCOM
    if(m_pSymucomSimulator)
    {
        ITSStation* pConnectedStation = m_pSymucomSimulator->CreateDynamicStationFromPattern(pVehicule.get());
        pVehicule->SetConnectedVehicle(pConnectedStation);
    }
#endif // USE_SYMUCOM

    boost::lock_guard<boost::mutex> guard(*m_pMutex);
    m_LstVehicles.push_back(pVehicule);
}

SymuViaFleet* Reseau::GetSymuViaFleet()
{
    return (SymuViaFleet*)m_LstFleets.front();
}

PublicTransportFleet* Reseau::GetPublicTransportFleet()
{
    return (PublicTransportFleet*)m_LstFleets[1];
}

DeliveryFleet* Reseau::GetDeliveryFleet()
{
    return (DeliveryFleet*)m_LstFleets[2];
}

//=================================================================
    double Reseau::GetDistanceEntreVehicules
//----------------------------------------------------------------
// Fonction  : Calcule la distance entre 2 vÃ©hicules au dÃ©but
//             du pas de temps par dÃ©faut ou Ã  la fin du pas de temps
//             si prÃ©cisÃ©
// Remarque  : Les vÃ©hicules doivent Ãªtre sur le mÃªme tuyau ou
//             sur deux tuyaux consÃ©cutifs sinon on doit connaÃ®tre
//             leur itinÃ©raire
//             Retourne 9999 si le calcul est impossible
// Version du: 05/12/2006
// Historique: 05/12/2006 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//             15/09/08 (C.BÃ©carie - TINEA )
//             Prise en compte de la connaissance de l'itinÃ©raire
//=================================================================
(
    Vehicule*   pVeh1,
    Vehicule*   pVeh2,
    bool        bFin    /* = false */ // indique que la distance est
                        // calculÃ©e Ã  la fin du pas de temps
)
{
    double  dbDst;
    int     nTps;

    nTps = 1;
    if(bFin)
        nTps = 0;   // Fin du pas de temps

    if(!pVeh1)
        return 9999;

    if(!pVeh2)
        return 9999;

    // Les vÃ©hicules sont sur le mÃªme tuyau au moment qui nous intÃ©resse
    if( pVeh1->GetLink(nTps) == pVeh2->GetLink(nTps) )
        return abs(pVeh2->GetPos(nTps) - pVeh1->GetPos(nTps));

    // Le vÃ©hicule 1 vient d'entrer sur le rÃ©seau
    if(nTps==1 && !pVeh1->GetLink(nTps))
    {
        if(pVeh1->GetVoie(nTps) == pVeh2->GetVoie(nTps))
            return pVeh2->GetPos(nTps);
        else
        {
            if( ((Tuyau*)pVeh1->GetVoie(nTps)->GetParent())->getConnectionAval() == pVeh2->GetLink(nTps)->getConnectionAmont() )
                return pVeh1->GetVoie(nTps)->GetLength() + pVeh2->GetPos(nTps);
            else
                return 9999;
        }
    }

    // Le vÃ©hicule 2 vient d'entrer sur le rÃ©seau
    if(nTps==1 && !pVeh2->GetLink(nTps))
    {
        if(pVeh2->GetVoie(nTps) == pVeh1->GetVoie(nTps))
            return pVeh1->GetPos(nTps);
        else
        {
            if( pVeh1->GetLink(nTps) && pVeh2->GetVoie(nTps) )
            {
                if( ((Tuyau*)pVeh2->GetVoie(nTps)->GetParent())->getConnectionAval() == pVeh1->GetLink(nTps)->getConnectionAmont() )
                {
                    return pVeh2->GetVoie(nTps)->GetLength() + pVeh1->GetPos(nTps);
                }
            }
           // else
                return 9999;
        }
    }

    if( !pVeh1->GetLink(nTps))
    {
        //*m_pFicSimulation << pVeh1->GetID() << " " << nTps << std::endl;
            return 9999;
    }

    if( !pVeh2->GetLink(nTps))
    {
        //*m_pFicSimulation << pVeh2->GetID() << " " << nTps << std::endl;
            return 9999;
    }


    // Les vÃ©hicules sont sur deux tuyaux adjacents
    if( pVeh1->GetLink(nTps)->getConnectionAval() == pVeh2->GetLink(nTps)->getConnectionAmont() )
        return pVeh1->GetVoie(nTps)->GetLength() - pVeh1->GetPos(nTps) + pVeh2->GetPos(nTps);

    if( pVeh2->GetLink(nTps)->getConnectionAval() == pVeh1->GetLink(nTps)->getConnectionAmont() )
        return pVeh2->GetVoie(nTps)->GetLength() - pVeh2->GetPos(nTps) + pVeh1->GetPos(nTps);

    // Si on connaÃ®t l'itinÃ©raire du vÃ©hicule 1, on peut en dÃ©duire la distance (avec l'hypothÃ¨se vÃ©hicule 2 devant 1 !)
    if(pVeh1->GetItineraire())
    {
        const std::vector<Tuyau*> & itiEx = ComputeItineraireComplet(*pVeh1->GetItineraire());
        int iTuyIdx = pVeh1->GetItineraireIndex(itiEx, isSameTuyauPredicate, pVeh1->GetLink(nTps));
        if(iTuyIdx != -1)
        {
            dbDst = pVeh1->GetVoie(nTps)->GetLength() - pVeh1->GetPos(nTps);
            for(int j=iTuyIdx+1; j< (int)itiEx.size(); j++)
            {
                if( itiEx.at(j) == pVeh2->GetLink(nTps) )
                {
                    dbDst += pVeh2->GetPos(nTps);
                    return dbDst;
                }
                else
                {
                    dbDst += itiEx.at(j)->GetLength();
                }
            }
        }
    }

    return 9999;
}

//=================================================================
    Tuyau*	Reseau::GetLinkFromLabel
//----------------------------------------------------------------
// Fonction  : Retourne le tuyau dont le libellÃ© est passÃ© en
//             paramÃ¨tre
// Remarque  :
// Version du: 13/07/2007
// Historique: 13/07/2007 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//=================================================================
(
    const std::string & ssTuyau
)
{
    std::map<std::string, Tuyau*>::iterator iter = m_mapTuyaux.find(ssTuyau);
    if(iter != m_mapTuyaux.end())
    {
        return iter->second;
    }
    else
    {
        return NULL;
    }
}

//=================================================================
    TypeVehicule*	Reseau::GetVehicleTypeFromID
//----------------------------------------------------------------
// Fonction  : Retourne le type de vÃ©hicule dont l'identifiant est passÃ©
//             en paramÃ¨tre
// Remarque  :
// Version du: 12/10/2007
// Historique: 12/10/2007 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//=================================================================
(
    const std::string & ssID
)
{
    std::deque <TypeVehicule*>::iterator par;
    for( par = m_LstTypesVehicule.begin(); par  != m_LstTypesVehicule.end(); ++par)
    {
        if( (*par)->GetLabel() == ssID)
        {
            return (*par);
        }
    }
    return NULL;
}

//=================================================================
CMotif*	Reseau::GetMotifFromID
//----------------------------------------------------------------
// Fonction  : Retourne le motif de dÃ©placement dont l'identifiant
//             est passÃ© en paramÃ¨tre
// Remarque  :
// Version du: 24/10/2016
// Historique: 24/10/2016 (O.Tonck - Ipsis)
//             CrÃ©ation
//=================================================================
(
    const std::string & ssID
)
{
    std::deque <CMotif*>::iterator par;
    for (par = m_LstMotifs.begin(); par != m_LstMotifs.end(); ++par)
    {
        if ((*par)->GetID() == ssID)
        {
            return (*par);
        }
    }
    return NULL;
}

//=================================================================
Parking*	Reseau::GetParkingFromID
//----------------------------------------------------------------
// Fonction  : Retourne le parking dont l'identifiant
//             est passÃ© en paramÃ¨tre
// Remarque  :
// Version du: 26/04/2017
// Historique: 26/04/2017 (O.Tonck - Ipsis)
//             CrÃ©ation
//=================================================================
(
const std::string & ssID
)
{
    for (size_t iPark = 0; iPark < Liste_parkings.size(); iPark++)
    {
        if (Liste_parkings.at(iPark)->GetID() == ssID)
        {
            return Liste_parkings.at(iPark);
        }
    }
    return NULL;
}


//=================================================================
    ControleurDeFeux*	Reseau::GetTrafficLightControllerFromID
//----------------------------------------------------------------
// Fonction  : Retourne le contrÃ´leur de feux dont l'identifiant est passÃ©
//             en paramÃ¨tre
// Remarque  :
// Version du: 31/03/2009
// Historique: 31/03/2009 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//=================================================================
(
    const std::string & ssID
)
{
    std::deque <ControleurDeFeux*>::iterator par;
    for( par = Liste_ControleursDeFeux.begin(); par != Liste_ControleursDeFeux.end(); ++par)
    {
        if( (*par)->GetLabel() == ssID)
        {
            return (*par);
        }
    }
    return NULL;
}


//=================================================================
    BriqueDeConnexion*	Reseau::GetBrickFromID
//----------------------------------------------------------------
// Fonction  : Retourne la brique Ã  partir de l'ID
// Remarque  :
// Version du: 08/10/2007
// Historique: 08/10/2007 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//=================================================================
(
    const std::string & ssCnxID
)
{
    // Recherche dans la liste des giratoires
    for( size_t i=0; i<Liste_giratoires.size(); i++)
    {
        if( Liste_giratoires[i]->GetLabel() == ssCnxID)
        {
            return Liste_giratoires[i];
        }
    }

    // Recherche dans la liste des carrefours Ã  feux
    for( size_t i=0; i<Liste_carrefoursAFeux.size(); i++)
    {
        if( Liste_carrefoursAFeux[i]->GetID() == ssCnxID)
        {
            return Liste_carrefoursAFeux[i];
        }
    }

    return NULL;
}

//=================================================================
    ConnectionPonctuel*	Reseau::GetConnectionFromID
//----------------------------------------------------------------
// Fonction  : Retourne la connexion et son type Ã  partir de l'ID
// Version du: 04/05/2012
// Historique: 10/05/2012 (O. Tonck - IPSIS)
//             Ajout des connexions de type Parking
//             04/05/2012 (O. Tonck - IPSIS)
//             A partir de la version du 04/05/2012, cette fonction
//             ne renvoie plus les entrÃ©es et sorties (ambiguitÃ©
//             lorsqu'une extremitÃ© est Ã  la fois entrÃ©e et sortie
//             puisque ces deux objets ont le mÃªme ID). Il faut
//             passer par les nouvelles fonctions GetEntreeFromID
//             et GetSortieFromID
//             08/10/2007 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//=================================================================
(
    const std::string & ssCnxID,
    char            &cTypeCnx
)
{


    std::map<std::string, Repartiteur*>::iterator rep = Liste_repartiteurs.find(ssCnxID);
    if(rep != Liste_repartiteurs.end())
    {
        cTypeCnx = 'R';
        return rep->second;
    }

    std::map<std::string, Convergent*>::iterator cvg = Liste_convergents.find(ssCnxID);
    if(cvg != Liste_convergents.end())
    {
        cTypeCnx = 'C';
        return cvg->second;
    }



    std::map<std::string, Divergent*>::iterator div = Liste_divergents.find(ssCnxID);
    if(div != Liste_divergents.end())
    {
        cTypeCnx = 'D';
        return div->second;
    }

    cTypeCnx = '0';
    return NULL;
}


//=================================================================
    SymuViaTripNode*	Reseau::GetOrigineFromID
//----------------------------------------------------------------
// Fonction  : Retourne l'origine d'ID voulu
// Remarque  :
// Version du: 04/05/2012
// Historique: 04/05/2012 (O. Tonck - IPSIS)
//             CrÃ©ation
//=================================================================
(
    const std::string & ssCnxID,
    char            &cTypeCnx
)
{
    SymuViaTripNode * result = NULL;
    cTypeCnx = '0';

    std::deque <Entree*>::iterator ent;
    std::deque <Entree*>::iterator ent_d = Liste_entree.begin();
    std::deque <Entree*>::iterator ent_f = Liste_entree.end();
    for (ent=ent_d; ent!=ent_f; ++ent)
    {
        if( (*ent)->GetOutputID() == ssCnxID)
        {
            result = (*ent);
            cTypeCnx = 'E';
            break;
        }
    }

    std::deque <Parking*>::iterator pent;
    std::deque <Parking*>::iterator pent_d = Liste_parkings.begin();
    std::deque <Parking*>::iterator pent_f = Liste_parkings.end();
    for (pent=pent_d; pent!=pent_f; ++pent)
    {
        if( (*pent)->GetID() == ssCnxID && (*pent)->GetOutputConnexion())
        {
            result = (*pent);
            cTypeCnx = 'P';
            break;
        }
    }

    std::deque <ZoneDeTerminaison*>::iterator pzone;
    std::deque <ZoneDeTerminaison*>::iterator pzone_d = Liste_zones.begin();
    std::deque <ZoneDeTerminaison*>::iterator pzone_f = Liste_zones.end();
    for (pzone= pzone_d; pzone!=pzone_f; ++pzone)
    {
        if( (*pzone)->GetOutputID() == ssCnxID)
        {
            result = (*pzone);
            cTypeCnx = 'Z';
            break;
        }
    }

    return result;
}


//=================================================================
    SymuViaTripNode*	Reseau::GetDestinationFromID
//----------------------------------------------------------------
// Fonction  : Retourne la destination d'ID voulu
// Remarque  :
// Version du: 04/05/2012
// Historique: 04/05/2012 (O. Tonck - IPSIS)
//             CrÃ©ation
//=================================================================
(
    const std::string & ssCnxID,
    char            &cTypeCnx
)
{
    SymuViaTripNode * result = NULL;
    cTypeCnx = '0';

    std::deque <Sortie*>::iterator sor;
    std::deque <Sortie*>::iterator sor_d = Liste_sortie.begin();
    std::deque <Sortie*>::iterator sor_f = Liste_sortie.end();
    for (sor= sor_d; sor!=sor_f; ++sor)
    {
        if( (*sor)->GetInputID() == ssCnxID)
        {
            result = (*sor);
            cTypeCnx = 'S';
            break;
        }
    }

    std::deque <Parking*>::iterator psor;
    std::deque <Parking*>::iterator psor_d = Liste_parkings.begin();
    std::deque <Parking*>::iterator psor_f = Liste_parkings.end();
    for (psor= psor_d; psor!=psor_f; ++psor)
    {
        if( (*psor)->GetID() == ssCnxID)
        {
            result = (*psor);
            cTypeCnx = 'P';
            break;
        }
    }

    std::deque <ZoneDeTerminaison*>::iterator pzone;
    std::deque <ZoneDeTerminaison*>::iterator pzone_d = Liste_zones.begin();
    std::deque <ZoneDeTerminaison*>::iterator pzone_f = Liste_zones.end();
    for (pzone= pzone_d; pzone!=pzone_f; ++pzone)
    {
        if( (*pzone)->GetID() == ssCnxID)
        {
            result = (*pzone);
            cTypeCnx = 'Z';
            break;
        }
    }

    return result;
}


//=================================================================
    SegmentMacro* Reseau::GetSegmentFromID
//----------------------------------------------------------------
// Fonction  : Retourne le segment dont l'ID est passÃ© en paramÃ¨tre
// Remarque  :
// Version du: 13/07/2007
// Historique: 13/07/2007 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//=================================================================
(
    int nID
)
{
    std::deque <TuyauMacro*>::iterator ItCur;
    TuyauMacro   *pTuyau;
    VoieMacro    *pVoie;

    for(ItCur = m_LstTuyauxMacro.begin(); ItCur!=m_LstTuyauxMacro.end(); ItCur++)
    {
        pTuyau = (*ItCur);
        for(int i=0; i<pTuyau->getNbVoiesDis(); i++)
        {
            pVoie = (VoieMacro*)pTuyau->GetLstLanes()[i];
            for(int j=0; j<pTuyau->GetNbCell(); j++)
            {
                if( pVoie->GetSegment(j+1)->GetID() == nID )
                    return (pVoie->GetSegment(j+1));
            }
        }
    }
    return NULL;
}


//=================================================================
    bool Reseau::LoadReseauXML
//----------------------------------------------------------------
// Fonction  : Chargement des donnÃ©es au format XML
// Remarque  :
// Version du: ... (en constante Ã©volution !!!)
// Historique: 05/10/2007 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//             ...
//=================================================================
(
    const string & sXMLFile,
    const std::string & sScenario,      // identifiant du scÃ©nario Ã  charger
    const std::string & sOutDirectory   // rÃ©pertoire de sortie Ã  utiliser Ã  la place du rÃ©pertoire indiquÃ© dans le fichier XML d'entrÃ©e
)
{
    clock_t startLoadTime = clock();

    DOMNode	  *pXMLRoot;
    DOMNode   *pXMLNodePlagesTemporelles;
    DOMNode	  *pXMLReseau;
    DOMNode   *pXMLNode;
    DOMNode   *pXMLNodeTroncons;
    DOMNode   *pXMLChild;
    DOMNode   *pXMLPlages;
    DOMNode   *pXMLSrcAcoustiques;
    DOMNode	  *pXMLCapacites;

    std::string strTmp, strTmp2;
    double      dbTmp, dbTmp2;
    int         nTmp;
    char strID[256], strRevetement[256];
    char cType, cResolution;
    double dbLargeurVoie, dbVitReg, dbVitCat;
    Point ptExtAmont, ptExtAval;
    int nVoie, nCellAcoustique;
    double dbCellAcouLength;
    std::string strEltAmontID, strEltAvalID;
    double dbDuree;
    Repartiteur *pRepartiteur;
    Convergent  *pConvergent;
    double      dbLongueur;
    CarrefourAFeuxEx  *pCAF;
    BriqueDeConnexion   *pBrique;
    Entree      *pEntree;
    SDateTime	tNow;
    double      dbPmin , dbPmax;
    std::string sPath;
    std::string sVar;
    std::string sMicroVehicleType;

    // Initialisation
    strTmp = "";
    m_bDoVehicleListSensorExtraction = true;

    // Chargement du fichier de trafic   
    m_XMLDocData = m_pXMLUtil->LoadTrafic(sXMLFile);
   

    if(m_XMLDocData == NULL)
    {
        return false;
    }

    // Lecture de l'arbre

    // Noeud root
    pXMLRoot = m_XMLDocData->getDocumentElement();

    // RÃ©cupÃ©ration du noeud scenario pour voir quels paramÃ¨tres charger.
    DOMNode * pXMLScenarioNode = NULL;
    if(!sScenario.compare(""))
    {
        pXMLScenarioNode = m_pXMLUtil->SelectSingleNode("/ROOT_SYMUBRUIT/SCENARIOS/SCENARIO", m_XMLDocData);
    }
    else
    {
        pXMLScenarioNode = m_pXMLUtil->SelectSingleNode("/ROOT_SYMUBRUIT/SCENARIOS/SCENARIO[@id=\"" + sScenario + "\"]", m_XMLDocData);
    }

    GetXmlAttributeValue(pXMLScenarioNode, "replications", m_nReplications, m_pLogger);

    // Noeud paramÃ©trage de simulation
    GetXmlAttributeValue(pXMLScenarioNode, "simulation_id", strTmp, m_pLogger);
    m_SimulationID = strTmp;
    pXMLNode = m_pXMLUtil->SelectSingleNode("/ROOT_SYMUBRUIT/SIMULATIONS/SIMULATION[@id=\"" + strTmp + "\"]", m_XMLDocData);
    DOMNode * pXMLNodeSimulation = pXMLNode;

    // RÃ©pertoire de sortie
    size_t nIndex = sXMLFile.rfind(DIRECTORY_SEPARATOR);
    string sOutDir;
    if(sOutDirectory.empty())
    {
        sOutDir = sXMLFile.substr(0, nIndex);
        if(GetXmlAttributeValue(pXMLScenarioNode, "dirout", strTmp, m_pLogger) )
        {
            sOutDir = sOutDir + DIRECTORY_SEPARATOR + strTmp;// + "\\";
        }
        m_sOutputDir = sOutDir;
    }
    else
    {
        m_sOutputDir = sOutDirectory;
    }


    // CrÃ©ation du rÃ©pertoire si besoin
    bool bErrOutDir = false;
    try
    {
        if ( !SystemUtil::FolderExists(m_sOutputDir) )
        {
#ifdef WIN32
            ::CreateDirectory(SystemUtil::ToWString(m_sOutputDir).c_str(), NULL);
#else
            mkdir(m_sOutputDir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
#endif
        }
    }
    catch(...)
    {
        m_sOutputDir = sXMLFile.substr(0, nIndex);
        bErrOutDir = true;
    }

    // PrÃ©fixe de sortie
    GetXmlAttributeValue(pXMLScenarioNode, "prefout", strTmp, m_pLogger);
    m_sPrefixOutputFiles = m_sOutputDir + DIRECTORY_SEPARATOR + strTmp;

    // CrÃ©ation du fichier de trace de simulation
    tNow = SDateTime::Now();

    DOMNode* pRestitutionNode = m_pXMLUtil->SelectSingleNode("./RESTITUTION", pXMLNode->getOwnerDocument(), (DOMElement*)pXMLNode);
    if(pRestitutionNode)
    {
        // Logs
        GetXmlAttributeValue(pRestitutionNode, "sortie_logs", m_bLogFile, m_pLogger);
        GetXmlAttributeValue(pRestitutionNode, "sortie_logs_light", m_bLogFileLight, m_pLogger);
    }

    // CrÃ©ation du fichier de trace du chargement
    string sFileChargementLog = m_sPrefixOutputFiles + "_Loading" + m_sSuffixOutputFiles + ".log";
    Logger loadingLogger(sFileChargementLog, m_bLogFile, m_bLogFileLight?Logger::Warning:Logger::Info);
    m_pLoadingLogger = &loadingLogger;
    if (m_pPythonUtils)
    {
        m_pPythonUtils->setLogger(&loadingLogger);
    }
    if(bErrOutDir)
    {
        loadingLogger << Logger::Error << "ERROR : Failure creating the output folder !"<<std::endl<<std::endl;
        loadingLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
    }

    // DÃ©but
    SDateTime m_dt;
    GetXmlAttributeValue(pXMLNode, "debut", m_dt, &loadingLogger);
    m_dtDebutSimu = m_dt.ToTime();
    hd = m_dtDebutSimu.GetHour();
    md = m_dtDebutSimu.GetMinute();
    sd = m_dtDebutSimu.GetSecond();

    // Fin
    GetXmlAttributeValue(pXMLNode, "fin", m_dt, &loadingLogger);
    m_dtFinSimu = m_dt.ToTime();
    hf = m_dtFinSimu.GetHour();
    mf = m_dtFinSimu.GetMinute();
    sf = m_dtFinSimu.GetSecond();

    // VÃ©rification
    if( m_dtDebutSimu > m_dtFinSimu)
    {
        loadingLogger<<"The simulation start time (" << m_dtDebutSimu.ToString() <<") must be inferior to the simulation end time (" << m_dtFinSimu.ToString() <<") !"<<std::endl;
        return false;
    }

    // Lecture du systÃ¨me des coordonnÃ©es gÃ©orÃ©fÃ©rencÃ©es des donnÃ©es d'entrÃ©e
    int nEPSGinput;
    GetXmlAttributeValue(pXMLNode, "EPSGinput", nEPSGinput, &loadingLogger);

    // Lecture du noeud PLAGES_TEMPORELLES
    pXMLNodePlagesTemporelles = m_pXMLUtil->SelectSingleNode("/ROOT_SYMUBRUIT/PLAGES_TEMPORELLES", m_XMLDocData);

    // lecture du type de dÃ©finition des variantes temporelles
    m_bTypeProfil = true;
    if(pXMLNodePlagesTemporelles)
    {
        GetXmlAttributeValue(pXMLNodePlagesTemporelles, "type", strTmp, &loadingLogger);
        if( strTmp == "horaire" ) {
            m_bTypeProfil = false;
        }
    }

    if(m_bTypeProfil)
    {
        // Si type 'profil', chargement de l'offset
        m_dbLag = 0;
        if(pXMLNodePlagesTemporelles)
        {
            m_dbLag = LoadLagOfVariation(pXMLNodePlagesTemporelles, loadingLogger);
            if(m_dbLag == -1)
            {
                loadingLogger << Logger::Error << "ERROR: The start time of the time profile can't be greater than the simulation's start time" << std::endl;
                loadingLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                return false;
            }
        }
    }
    else
    {
        // Si type horaire, lecture des plages temporelles
        BuildPlagesTemporelles(pXMLNodePlagesTemporelles, loadingLogger);
    }
    // Fin de lectude du noeud PLAGES_TEMPORELLES

    // Version des donnÃ©es
    GetXmlAttributeValue(pXMLRoot, "version", strTmp, &loadingLogger);
    if (strTmp != VERSION_FICHIER)
    {
        loadingLogger<<"The data version is not up to date ! (" << strTmp << " instead of " << VERSION_FICHIER << ")" << std::endl;
        return false;
    }

    // SECTION SIMULATION

    // Sous-noeud RESTITUTION
    EraseListOfVariation(&m_xmlDocTrafics); // Nettoyage Ã©ventuel des docTrafic avant de charger les nouveaux
    if(pRestitutionNode)
    {

        // Debug
        GetXmlAttributeValue(pRestitutionNode, "debug", m_bDebug, &loadingLogger);
        GetXmlAttributeValue(pRestitutionNode, "debug_matrice_OD", m_bDebugOD, &loadingLogger);
        GetXmlAttributeValue(pRestitutionNode, "debug_SAS", m_bDebugSAS, &loadingLogger);

        // debug_chgt_voie
        GetXmlAttributeValue(pRestitutionNode, "chgt_voie_debug", m_bChgtVoieDebug, &loadingLogger);

        // Sortie des trajectoires
        GetXmlAttributeValue(pRestitutionNode, "sortie", strTmp, &loadingLogger);
        if( strTmp == "light" )
            m_bSortieLight = true;

        // Trajectoire
        GetXmlAttributeValue(pRestitutionNode, "trajectoires", m_bSortieTraj, &loadingLogger);

        // Trace route
        GetXmlAttributeValue(pRestitutionNode, "trace_route", m_bTraceRoute, &loadingLogger);

        // vgp_debug
        GetXmlAttributeValue(pRestitutionNode, "vgp_debug", m_bVGPDebug, &loadingLogger);

        // trace_stocks
        GetXmlAttributeValue(pRestitutionNode, "trace_stocks", m_bTraceStocks, &loadingLogger);

        // trace_regime
        GetXmlAttributeValue(pRestitutionNode, "trace_regime", m_bSortieRegime, &loadingLogger);

        // production fichiers CSV
        GetXmlAttributeValue(pRestitutionNode, "csv", m_bCSVOutput, &loadingLogger);

        // production sortie JSON des temps de parcours
        GetXmlAttributeValue(pRestitutionNode, "sortie_temps_parcours", m_bTravelTimesOutput, &loadingLogger);
        GetXmlAttributeValue(pRestitutionNode, "sortie_temps_parcours_periode", m_dbTravelTimesOutputPeriod, &loadingLogger);

        // type de production fichiers CSV
        if(m_bCSVOutput)
        {
            GetXmlAttributeValue(pRestitutionNode, "csv_type", strTmp, &loadingLogger);
            if(strTmp == "capteurs")
            {
                m_bCSVTrajectories = false;
                m_bCSVSensors = true;
            }
            else if(strTmp == "trajectoires")
            {
                m_bCSVTrajectories = true;
                m_bCSVSensors = false;
            }
            else
            {
                m_bCSVTrajectories = true;
                m_bCSVSensors = true;
            }
        }

        // production fichiers GML
        GetXmlAttributeValue(pRestitutionNode, "sortie_gml", m_bGMLOutput, &loadingLogger);

        // Lecture du systÃ¨me des coordonnÃ©es gÃ©orÃ©fÃ©rencÃ©es des donnÃ©es d'entrÃ©e
        int nEPSGoutput;
        GetXmlAttributeValue(pRestitutionNode, "EPSGoutput", nEPSGoutput, &loadingLogger);

        if( nEPSGinput>0 && nEPSGoutput>0 )
            CreateCoordinateTransformation(nEPSGinput, nEPSGoutput);

        // ETS 140615 : plage d'Ã©criture(extraction)

        DOMNode* pPlageExtractions = m_pXMLUtil->SelectSingleNode("./PLAGES_EXTRACTION", pXMLNode->getOwnerDocument(), (DOMElement*)pRestitutionNode);
        if( pPlageExtractions )
        {
            XMLSize_t counti = pPlageExtractions->getChildNodes()->getLength();
            for(XMLSize_t i=0; i<counti; i++)
            {

                DOMNode* pPlageExtraction = pPlageExtractions->getChildNodes()->item(i);
                if (pPlageExtraction->getNodeType() != DOMNode::ELEMENT_NODE) continue;
                STime tDebutExtraction;
                STime tFinExtraction;
                SDateTime dt;
                double dStartS;
                double dEndS;
                if( GetXmlAttributeValue(pPlageExtraction, "debut_extraction",dt, &loadingLogger) )
                {
                    dStartS = dt.ToTime().ToSecond();
                }
                else
                {
                    dStartS = m_dtDebutSimu.ToSecond();
                }
                dEndS = m_dtFinSimu.ToSecond();
                if( GetXmlAttributeValue(pPlageExtraction, "fin_extraction",dt, &loadingLogger) )
                {
                    if( dt.ToTime().ToSecond()>0)
                    {
                        dEndS = dt.ToTime().ToSecond();
                    }

                }

                PlageTemporelle::Set(dStartS  , dEndS , m_extractionRange  , m_dtDebutSimu.ToSecond() );

            }// rof each PLAGE_EXTRACTION
        } // fi pPlageExtractions
        else // on a pas de PLAGE d'EXTRACTION dÃ©fini -> on extrait sur toute la simulation
        {
            PlageTemporelle::Set( m_dtDebutSimu.ToSecond(), m_dtFinSimu.ToSecond(), m_extractionRange, m_dtDebutSimu.ToSecond() );
        }
        // dÃ©fini un document de sortie par plage d'extraction
        std::vector< PlageTemporelle *>::iterator itPlage;
        for( itPlage = m_extractionRange.begin(); itPlage != m_extractionRange.end(); itPlage++)
        {
            AddVariation((*itPlage), boost::shared_ptr<TraceDocTrafic>(), &m_xmlDocTrafics);

        }

        if( !GetXmlAttributeValue(pRestitutionNode, "vehicules_capteur_extraction", m_bDoVehicleListSensorExtraction, &loadingLogger) )
        {
            // cas par defaut
            m_bDoVehicleListSensorExtraction = true;
        }

        DOMNode* pSauvegardes = m_pXMLUtil->SelectSingleNode("./SAUVEGARDES_ETAT", pXMLNode->getOwnerDocument(), (DOMElement*)pRestitutionNode);
        if (pSauvegardes)
        {
            GetXmlAttributeValue(pSauvegardes, "allegement_fichier_sauvegarde", m_bLightSavingMode, &loadingLogger);

            int timeIntervalSave;
            if (GetXmlAttributeValue(pSauvegardes, "interval_temps_sauvegarde", timeIntervalSave, &loadingLogger))
            {
                double timeNextSave = m_dtDebutSimu.ToSecond() + timeIntervalSave;
                while (timeNextSave < m_dtFinSimu.ToSecond())
                {
                    m_LstInstantsSauvegardes.push_back(timeNextSave - m_dtDebutSimu.ToSecond());
                    timeNextSave += timeIntervalSave;
                }
            }

            XMLSize_t counti = pSauvegardes->getChildNodes()->getLength();
            for (XMLSize_t i = 0; i<counti; i++)
            {

                DOMNode* pSauvegarde = pSauvegardes->getChildNodes()->item(i);
                if (pSauvegarde->getNodeType() != DOMNode::ELEMENT_NODE) continue;
                SDateTime dt;
                if (GetXmlAttributeValue(pSauvegarde, "instant_sauvegarde", dt, &loadingLogger))
                {
                    if (dt.ToTime() >= m_dtDebutSimu && dt.ToTime() <= m_dtFinSimu)
                    {
                        m_LstInstantsSauvegardes.push_back(dt.ToTime().ToSecond() - m_dtDebutSimu.ToSecond());
                    }

                }
            }// rof each SAUVEGARDES_ETAT
        } // fi pSauvegardes
    }

    // Seed
    GetXmlAttributeValue(pXMLNode, "seed", m_uiSeed, &loadingLogger);

    // DÃ©but des donnÃ©es
    GetXmlAttributeValue(pXMLNode, "debut_data", m_dtDebutData, &loadingLogger);

    // VÃ©rification
    if( m_dtDebutData.ToTime() > m_dtDebutSimu)
    {
        //Chargement<<"L'heure de dÃ©marrage de la simulation (" << m_dtDebutSimu.Format("%H:%M:%S") <<") doit Ãªtre postÃ©rieure Ã  l'heure de dÃ©but des donnÃ©es (" << m_dtDebutSimu.Format("%H:%M:%S") <<") !"<<std::endl;
        //Chargement<<"L'heure de dÃ©marrage de la simulation (" << m_dtDebutSimu.ToString() <<") doit Ãªtre postÃ©rieure Ã  l'heure de dÃ©but des donnÃ©es (" << m_dtDebutSimu.ToString() <<") !"<<std::endl;
        return false;
    }

    m_dbDiffDebutSimuData = 0;
    if (m_dtDebutData != SDateTime())
        m_dbDiffDebutSimuData = (sd + md*60 + hd*3600) - (m_dtDebutData.GetSecond() + m_dtDebutData.GetMinute()*60 + m_dtDebutData.GetHour()*3600);

    // Calcul de la durÃ©e de la simulation
    m_dbDureeSimu = (hf*3600+mf*60+sf) - (hd*3600+md*60+sd);


    // Pas de temps
    GetXmlAttributeValue(pXMLNode, "pasdetemps", pas_de_temps, &loadingLogger);



    // Loi de poursuite utilisÃ©e
    m_bLoiPoursuiteOld = false;
    GetXmlAttributeValue(pXMLNode, "loipoursuite", m_sLoipoursuite, &loadingLogger);
    if( m_sLoipoursuite == "exacte" ) {
        m_bLoiPoursuiteOld = true;
    } else {
        loadingLogger << Logger::Warning << "WARNING : the 'loipoursuite' attribute is set to 'estimee'. The use of this mode is obsolete and can cause incorrect behavior and traffic dead locks." << std::endl;
        loadingLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
    }

    // Type de simulation
    m_bSimuAcoustique = false;
    m_bSimuTrafic = false;
    m_bSimuAir = false;
    m_bSimuSirane = false;

    // Simulation trafic ?
    GetXmlAttributeValue(pXMLNode, "simulation_trafic", m_bSimuTrafic, &loadingLogger);

    // Simulation acoustique ?
    GetXmlAttributeValue(pXMLNode, "simulation_acoustique", m_bSimuAcoustique, &loadingLogger);

    // Simulation atmosphÃ©rique ?
    GetXmlAttributeValue(pXMLNode, "simulation_atmospherique", m_bSimuAir, &loadingLogger);

    // Production des sorties nÃ©cesaires Ã  la simulation des polluants par Sirane ?
    GetXmlAttributeValue(pXMLNode, "simulation_sirane", m_bSimuSirane, &loadingLogger);


    // Type de sortie acoustique
    char cTmp;
    GetXmlAttributeValue(pXMLNode, "type_sortieacoustique", m_cTypeSortieAcoustique, &loadingLogger);
    if( m_cTypeSortieAcoustique == 'C' )
        m_bAcousCell = true;
    else
        m_bAcousSrcs = true;

    // Comportement du flux
    m_bCptDestination = false;
    m_bCptDirectionnel = false;
    m_bCptItineraire = false;
    m_bAffectationDynamique = false;
    GetXmlAttributeValue(pXMLNode, "comportementflux", strTmp, &loadingLogger);
    if( strTmp == "dir" )
        m_bCptDirectionnel = true;

    GetXmlAttributeValue(pXMLNode, "affectation_teta_logit", m_dbTetaLogit, &loadingLogger);				// Chargement du paramÃ¨tre teta du logit

    bool bDijkstra;
    std::string sKSP;
    GetXmlAttributeValue(pXMLNode, "affectation_calculKSP", sKSP, &loadingLogger);
    bDijkstra = ( sKSP == "Dijkstra" );

     // Nombre de plus court chemin Ã  prendre en compte lors du calcul des itinÃ©raires
    GetXmlAttributeValue(pXMLNode, "nombre_pluscourtchemin", m_nNbPlusCourtChemin, &loadingLogger);

    if( strTmp == "iti" )
    {
        m_bCptItineraire = true;

        if(pRestitutionNode)
            GetXmlAttributeValue(pRestitutionNode, "affectation_sortie", m_bSaveAffectation, &loadingLogger);	// Sortie d'affectation

        std::string sAffectationDyn;
        GetXmlAttributeValue(pXMLNode, "affectation", sAffectationDyn, &loadingLogger);
        m_bAffectationDynamique = sAffectationDyn == "dynamique";

        GetXmlAttributeValue(pXMLNode, "periode_affectation", nTmp, &loadingLogger);	// Chargement de la pÃ©riode d'affectation

        double dbVar;
        int nNbCnx;
        char cVar;

        std::string sAffectEqui;
        GetXmlAttributeValue(pXMLNode, "affectation_equilibre", sAffectEqui, &loadingLogger);				// Chargement du type d'affectation (pas d'Ã©quilibre, recherche d'Ã©quilibre sans mÃ©moire, recherche d'Ã©quilibre par la mÃ©thode des moyennes successives)

        int nAffectEqui = 1;
        if( sAffectEqui == "NE" )
            nAffectEqui = 1;
        else if ( sAffectEqui == "ENM" )
            nAffectEqui = 2;
        else if ( sAffectEqui == "MSA" )
            nAffectEqui = 3;

        cVar = 'R';
        GetXmlAttributeValue(pXMLNode, "affectation_type_temps_de_parcours", sVar, &loadingLogger);			// Chargement du type de calcul du temps de parcours
        if( sVar == "PN" )
            cVar = 'P';
        else if( sVar == "PL" )
            cVar = 'Q';

        GetXmlAttributeValue(pXMLNode, "affectation_variation_temps_de_parcours", dbVar, &loadingLogger);	// Chargement de la limite de variation du temps de parcours
        GetXmlAttributeValue(pXMLNode, "affectation_nb_connexion_amont", nNbCnx, &loadingLogger);			// Chargement du nombre de niveau en amont oÃ¹ le calcul des coeff. d'affectation doit Ãªtre relancÃ©

        double dbSeuil, dbSeuilTempsParcours;
        int nNbItMax;
        std::string sMode;

        // Mode d'affectation
        GetXmlAttributeValue(pXMLNode, "affectation_mode", sMode, &loadingLogger);			// Chargement du mode d'affectation

        if ( sMode == "wardrop" )
        {
            m_nMode = 2;
        }
        else
        {
            m_nMode = 1;
        }

        GetXmlAttributeValue(pXMLNode, "affectation_seuil_convergence", dbSeuil, &loadingLogger);							// Seuil de convergence
        GetXmlAttributeValue(pXMLNode, "affectation_seuil_tempsdeparcours", dbSeuilTempsParcours, &loadingLogger);			// Seuil des temps de parcours

        GetXmlAttributeValue(pXMLNode, "affectation_nb_it_max", nNbItMax, &loadingLogger);                 // Chargement du nombre max d'itÃ©ration

        bool bReroutageVehiculesExistants;
        double dbDstMinAvantReroutage;
        GetXmlAttributeValue(pXMLNode, "affectation_reroutage", bReroutageVehiculesExistants, &loadingLogger);   // Activation ou non du reroutage dynamique
        GetXmlAttributeValue(pXMLNode, "affectation_reroutage_dstmax", dbDstMinAvantReroutage, &loadingLogger);             // Distance min avant choix pour le reroutage dynamique
        GetXmlAttributeValue(pXMLNode, "affectation_wardrop_tolerance", m_dbWardropTolerance, &loadingLogger);                 // paramÃ©trage de la prise en compte des chemins dont le cout est infÃ©rieur Ã  cout_min*(1+tolerance)


        m_pModuleAffectation = new Affectation(nTmp, nAffectEqui, dbVar, nNbCnx, m_dbTetaLogit, cVar, m_nMode, dbSeuil, nNbItMax, bDijkstra, m_nNbPlusCourtChemin, dbSeuilTempsParcours, bReroutageVehiculesExistants, dbDstMinAvantReroutage, m_dbWardropTolerance);			// Instanciation du module de l'affectation calculÃ©e

    }
    if( strTmp == "des" )
    {
        m_bCptDestination = true;
    }

    // Titre
    strTmp = "";
    GetXmlAttributeValue(pXMLNode, "titre", strTmp, &loadingLogger);
    m_strTitre = strTmp;

    // Date
    GetXmlAttributeValue(pXMLNode, "date", m_dtDateSimulation, &loadingLogger);

    // ProcÃ©dure de correction de la dÃ©cÃ©lÃ©ration
    if(m_bSimuTrafic)
    {
        GetXmlAttributeValue(pXMLNode, "proc_deceleration", m_bProcDec, &loadingLogger);
        if(m_bProcDec)
            GetXmlAttributeValue(pXMLNode, "proc_dec_taux", m_dbDecTaux, &loadingLogger);   // Taux de dÃ©cÃ©lÃ©ration
    }

    // Type de calcul de l'offre des convergents
    GetXmlAttributeValue(pXMLNode, "offre_aval_convergent_deltaN", m_bOffreCvgDeltaN, &loadingLogger);

    GetXmlAttributeValue(pXMLNode, "calcul_tq_convergent", m_bCalculTqConvergent, &loadingLogger);

    char cDefaultResolution;
    double dbDefaultCellAcouLength;
    int nDefaultNbCellAcou;
    GetXmlAttributeValue(pXMLNode, "resolution", cDefaultResolution, &loadingLogger);
    GetXmlAttributeValue(pXMLNode, "longueur_cell_acoustique", dbDefaultCellAcouLength, &loadingLogger);
    GetXmlAttributeValue(pXMLNode, "nb_cell_acoustique", nDefaultNbCellAcou, &loadingLogger);

    // ParamÃ¨tres Sirane (pour CityDyne)
    GetXmlAttributeValue(pXMLNode, "nb_cell_sirane", m_nNbCellSirane, &loadingLogger);
    GetXmlAttributeValue(pXMLNode, "min_longueur_cell_sirane", m_dbMinLongueurCellSirane, &loadingLogger);
    GetXmlAttributeValue(pXMLNode, "periode_agregation_sirane", m_dbPeriodeAgregationSirane, &loadingLogger);
    GetXmlAttributeValue(pXMLNode, "extension_barycentre_sirane", m_bExtensionBarycentresSirane, &loadingLogger);
    if(m_bExtensionBarycentresSirane)
    {
        m_nNbCellSirane = 1;
    }


    // ParamÃ¨tres globaux de gestion des zones de terminaison
    double dbMaxRangeInZones = std::numeric_limits<double>::infinity();
    GetXmlAttributeValue(pXMLNode, "zone_distance_terminaison_max", dbMaxRangeInZones, &loadingLogger);

    GetXmlAttributeValue(pXMLNode, "affectation_heuristique", strTmp, &loadingLogger);
    if (!strTmp.compare("distanceEuclidienne"))
    {
        m_eShortestPathHeuristic = HEURISTIC_EUCLIDIAN;
    }
    else
    {
        m_eShortestPathHeuristic = HEURISTIC_NONE;
    }



    GetXmlAttributeValue(pXMLNode, "affectation_heuristique_gamma", m_dbHeuristicGamma, &loadingLogger);
    GetXmlAttributeValue(pXMLNode, "affectation_astar_beta", m_dbAStarBeta, &loadingLogger);
    GetXmlAttributeValue(pXMLNode, "affectation_alpha", m_dbDijkstraAlpha, &loadingLogger);
    GetXmlAttributeValue(pXMLNode, "affectation_beta", m_dbDijkstraBeta, &loadingLogger);
    GetXmlAttributeValue(pXMLNode, "affectation_gamma", m_dbDijkstraGamma, &loadingLogger);
    GetXmlAttributeValue(pXMLNode, "affectation_commonality_filter", m_bCommonalityFilter, &loadingLogger);
    GetXmlAttributeValue(pXMLNode, "affectation_commonality_alpha", m_dbCommonalityAlpha, &loadingLogger);
    GetXmlAttributeValue(pXMLNode, "affectation_commonality_beta", m_dbCommonalityBeta, &loadingLogger);
    GetXmlAttributeValue(pXMLNode, "affectation_commonality_gamma", m_dbCommonalityGamma, &loadingLogger);

    GetXmlAttributeValue(pXMLNode, "affectation_penalite_tourne_a_droite", m_dbRightTurnPenalty, &loadingLogger);
    GetXmlAttributeValue(pXMLNode, "affectation_penalite_non_prioritaire", m_dbNonPriorityPenalty, &loadingLogger);
    GetXmlAttributeValue(pXMLNode, "affectation_angle_tout_droit", m_dbAngleToutDroit, &loadingLogger);
    GetXmlAttributeValue(pXMLNode, "affectation_facteur_penalite_tourne_a_droite", m_dbRightTurnPenaltyFactor, &loadingLogger);
    GetXmlAttributeValue(pXMLNode, "affectation_facteur_penalite_tourne_a_gauche", m_dbNonPriorityPenaltyFactor, &loadingLogger);

    GetXmlAttributeValue(pXMLNode, "affectation_estimation_attente_feux", m_bEstimateTrafficLightsWaitTime, &loadingLogger);

    GetXmlAttributeValue(pXMLNode, "micro_types_vehicules", sMicroVehicleType, &loadingLogger);
    GetXmlAttributeValue(pXMLNode, "micro_distance_aval", m_dbDownstreamMicroDistance, &loadingLogger);
    GetXmlAttributeValue(pXMLNode, "micro_distance_amont", m_dbUpstreamMicroDistance, &loadingLogger);
    GetXmlAttributeValue(pXMLNode, "micro_distance_cotes", m_dbSidesMicroDistance, &loadingLogger);

    GetXmlAttributeValue(pXMLNode, "fichier_chargement_etat", m_strFileToLoadState, &loadingLogger);


    // SECTION TRAFIC
    // Noeud paramÃ©trage de trafic
    GetXmlAttributeValue(pXMLScenarioNode, "trafic_id", m_TraficID, &loadingLogger);

    DOMNode * pXMLNodeTrafic = m_pXMLUtil->SelectSingleNode("/ROOT_SYMUBRUIT/TRAFICS/TRAFIC[@id=\"" + m_TraficID + "\"]", m_XMLDocData);

    // AccÃ©lÃ©ration bornÃ©e
    GetXmlAttributeValue(pXMLNodeTrafic, "accbornee", m_bAccBornee, &loadingLogger);

    // Coeff. de relaxation du rÃ©seau
    GetXmlAttributeValue(pXMLNodeTrafic, "coeffrelax", m_dbRelaxation, &loadingLogger);

    // Changement de voie
    GetXmlAttributeValue(pXMLNodeTrafic, "chgtvoie", m_bChgtVoie, &loadingLogger);

    // DÃ©passement
    GetXmlAttributeValue(pXMLNodeTrafic, "depassement", m_bDepassement, &loadingLogger);

    // TraversÃ©es
    GetXmlAttributeValue(pXMLNodeTrafic, "traversees", m_bTraversees, &loadingLogger);

    // Ordre de traitement des voies dans le cas d'un changement de voie discrÃ©tionnaire
    GetXmlAttributeValue(pXMLNodeTrafic, "chgtvoie_discr_ordre", sVar, &loadingLogger);
    m_nOrdreChgtVoieDiscr = 1;	// Par dÃ©faut
    if( sVar == "DG")
        m_nOrdreChgtVoieDiscr = 2;
    if( sVar == "F")
        m_nOrdreChgtVoieDiscr = 3;

    // ProcÃ©dure d'agressivitÃ©
    GetXmlAttributeValue(pXMLNodeTrafic, "agressivite", m_bProcAgressivite, &loadingLogger);

    if(m_bChgtVoie)
    {
        // Alpha mandatory
        GetXmlAttributeValue(pXMLNodeTrafic, "chgtvoie_mandatory_falpha", m_dbFAlphaMandatory, &loadingLogger);
        //if( fabs(m_dbAlphaMandatory) <= 0.001 )
        //	m_dbAlphaMandatory = 1. / pas_de_temps;

        // Type de proba en mandatory
        GetXmlAttributeValue(pXMLNodeTrafic, "chgtvoie_mandatory_probabilite", m_cChgtVoieMandatoryProba, &loadingLogger);

        // Mode de changement de voie mandatory
        GetXmlAttributeValue(pXMLNodeTrafic, "chgtvoie_mandatory_mode", m_cChgtVoieMandatoryMode, &loadingLogger);

        // Distance avant la fin du tronÃ§on pour forcer un changement de voie
        GetXmlAttributeValue(pXMLNodeTrafic, "chgtvoie_dstfin", m_dbDstChgtVoie, &loadingLogger);

        GetXmlAttributeValue(pXMLNodeTrafic, "chgtvoie_dstfin_force", m_dbDstChgtVoieForce, &loadingLogger);
        GetXmlAttributeValue(pXMLNodeTrafic, "chgtvoie_dstfin_force_phi", m_dbPhiChgtVoieForce, &loadingLogger);

        // Gestion des ghosts
        GetXmlAttributeValue(pXMLNodeTrafic, "chgtvoie_ghost", m_bChgtVoieGhost, &loadingLogger);
        if( m_bChgtVoieGhost )
        {
            GetXmlAttributeValue(pXMLNodeTrafic, "chgtvoie_ghost_durationMin", m_nChgtVoieGhostDurationMin, &loadingLogger);
            GetXmlAttributeValue(pXMLNodeTrafic, "chgtvoie_ghost_durationMax", m_nChgtVoieGhostDurationMax, &loadingLogger);
            GetXmlAttributeValue(pXMLNodeTrafic, "chgtvoie_ghost_lenghtBevel", m_dbChgtVoieGhostBevelLength, &loadingLogger);
        }

    }

    GetXmlAttributeValue(pXMLNodeTrafic, "meso_nb_vehicules_chgt_voie", m_nMesoNbVehChgtVoie, &loadingLogger);

    GetXmlAttributeValue(pXMLNodeTrafic, "mode_depassement_chgt_direction", m_bModeDepassementChgtDir, &loadingLogger);
    GetXmlAttributeValue(pXMLNodeTrafic, "distance_depassement_chgt_direction", m_dbDistDepassementChgtDir, &loadingLogger);


    // SECTION TRAFIC / LOIS_DE_POURSUITE
    pXMLNode = m_pXMLUtil->SelectSingleNode("./LOIS_DE_POURSUITE", m_XMLDocData, (DOMElement*)pXMLNodeTrafic);
    if(pXMLNode)
    {
        // Lecture de la dÃ©finition des lois de poursuite
        XMLSize_t counti = pXMLNode->getChildNodes()->getLength();
        for(XMLSize_t i=0; i<counti;i++)
        {
            DOMNode *pXMLLoiDePoursuite = pXMLNode->getChildNodes()->item(i);
            if (pXMLLoiDePoursuite->getNodeType() != DOMNode::ELEMENT_NODE) continue;

            // Lecture des paramÃ¨tres et ajout de la loi de poursuite
            m_pCarFollowingFactory->addCarFollowing(pXMLLoiDePoursuite);
        }
    }

    // SECTION TRAFIC / TYPES_DE_VEHICULE
    pXMLNode = m_pXMLUtil->SelectSingleNode("./TYPES_DE_VEHICULE", m_XMLDocData, (DOMElement*)pXMLNodeTrafic);
    m_dbKxmin = 1;          // minimum des Kx
    m_dbMaxDebitMax = 0;    // maximum des dÃ©bit max

    XMLSize_t counti = pXMLNode->getChildNodes()->getLength();
    for(XMLSize_t i=0; i<counti;i++)
    {
        pXMLChild = pXMLNode->getChildNodes()->item(i);
        if (pXMLChild->getNodeType() != DOMNode::ELEMENT_NODE) continue;

        TypeVehicule *pTypeVeh;
        double dbW, dbKx, dbVx, dbAx;

        pTypeVeh = new TypeVehicule;

        // ID
        GetXmlAttributeValue(pXMLChild, "id", strTmp, &loadingLogger);
        pTypeVeh->SetLibelle( strTmp );

        // ID GML
        GetXmlAttributeValue(pXMLChild, "citygml_type", strTmp, &loadingLogger);
        pTypeVeh->SetGMLLibelle( strTmp );

        // DonnÃ©es du diagramme fondamental
        GetXmlAttributeValue(pXMLChild, "w", dbW, &loadingLogger);
        GetXmlAttributeValue(pXMLChild, "kx", dbKx, &loadingLogger);
        GetXmlAttributeValue(pXMLChild, "vx", dbVx, &loadingLogger);

        pTypeVeh->SetVxKxW(dbVx, dbKx, dbW);

        double dbVxDispersion, dbVxMin, dbVxMax;
        if(  GetXmlAttributeValue(pXMLChild, "vx_dispersion", dbVxDispersion, &loadingLogger)
          && GetXmlAttributeValue(pXMLChild, "vx_min", dbVxMin, &loadingLogger)
          && GetXmlAttributeValue(pXMLChild, "vx_max", dbVxMax, &loadingLogger) )
        {
            // VÃ©rification min et max
            if( dbVxMin > dbVxMax || dbVxMin > dbVx || dbVxMax < dbVx)
            {
                loadingLogger << Logger::Error << "ERROR: incorrect values for the Vx value of the vehicle type " << pTypeVeh->GetLabel() << "." << std::endl;
                loadingLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                return false;
            }

            // VÃ©rification de l'intervalle
            double dbVxMinNor = (dbVxMin - dbVx) / dbVxDispersion;
            double dbVxMaxNor = (dbVxMax - dbVx) / dbVxDispersion;

            dbPmin = CumulativeNormalDistribution(dbVxMinNor);
            dbPmax = CumulativeNormalDistribution(dbVxMaxNor);

            // Avertissement
            if( dbPmax - dbPmin < 0.9 )
            {
                std::string sTmp;
                double dbTmp = (dbPmax - dbPmin)*100;
                sTmp = SystemUtil::ToString(2, dbTmp);

                loadingLogger << Logger::Warning << "WARNING: the min and max thresholds of the free flow speed of the vehicle type " << pTypeVeh->GetLabel();
                loadingLogger << Logger::Warning << " must be set so that 90% of the samples are inside the defined range." << std::endl;
                loadingLogger << Logger::Warning << " ( currently " << sTmp.c_str() << " ). " << std::endl;
                loadingLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
            }

            // Distribution stochastique de la vitesse libre
            pTypeVeh->SetVxDistr(true);
            pTypeVeh->SetVxDisp(dbVxDispersion);
            pTypeVeh->SetVxMin(dbVxMin);
            pTypeVeh->SetVxMax(dbVxMax);
        }

        double dbWDispersion, dbWMin, dbWMax;
        if(  GetXmlAttributeValue(pXMLChild, "w_dispersion", dbWDispersion, &loadingLogger)
          && GetXmlAttributeValue(pXMLChild, "w_min", dbWMin, &loadingLogger)
          && GetXmlAttributeValue(pXMLChild, "w_max", dbWMax, &loadingLogger) )
        {
            // VÃ©rification min et max
            if( dbWMin > dbWMax || dbWMin > dbW || dbWMax < dbW)
            {
                loadingLogger << Logger::Error << "ERROR: incorrect values for the min/max W thresholds for vehicle type " << pTypeVeh->GetLabel() << "." << std::endl;
                loadingLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                return false;
            }

            // VÃ©rification de l'intervalle
            double dbWMinNor = (dbWMin - dbW) / dbWDispersion;
            double dbWMaxNor = (dbWMax - dbW) / dbWDispersion;

            dbPmin = CumulativeNormalDistribution(dbWMinNor);
            dbPmax = CumulativeNormalDistribution(dbWMaxNor);

            // Avertissement
            if( dbPmax - dbPmin < 0.9 )
            {
                std::string sTmp;
                double dbTmp = (dbPmax - dbPmin)*100;
                sTmp = SystemUtil::ToString(2, dbTmp);

                loadingLogger << Logger::Warning << "WARNING: the min/max thresholds for w speed for vehicle type " << pTypeVeh->GetLabel();
                loadingLogger << Logger::Warning << " must be set so that 90% of the samples are inside the defined range." << std::endl;
                loadingLogger << Logger::Warning << " ( currently " << sTmp << " ). " << std::endl;
                loadingLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
            }

            // Distribution stochastique de w
            pTypeVeh->SetWDistr(true);
            pTypeVeh->SetWDisp(dbWDispersion);
            pTypeVeh->SetWMin(dbWMin);
            pTypeVeh->SetWMax(dbWMax);
        }

        double dbInvKxDispersion, dbKxMin, dbKxMax;
        if(  GetXmlAttributeValue(pXMLChild, "inv_kx_dispersion", dbInvKxDispersion, &loadingLogger)
          && GetXmlAttributeValue(pXMLChild, "kx_min", dbKxMin, &loadingLogger)
          && GetXmlAttributeValue(pXMLChild, "kx_max", dbKxMax, &loadingLogger) )
        {
            // VÃ©rification min et max
            if( dbKxMin > dbKxMax || dbKxMin > dbKx || dbKxMax < dbKx)
            {
                loadingLogger << Logger::Error << "ERROR: incorrect values for the min/max Kx thresholds for vehicle type " << pTypeVeh->GetLabel() << "." << std::endl;
                loadingLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                return false;
            }

            // VÃ©rification de l'intervalle
            double dbInvKxMinNor = ( (1/dbKxMax) - (1/dbKx) ) / dbInvKxDispersion;
            double dbInvKxMaxNor = ( (1/dbKxMin) - (1/dbKx) ) / dbInvKxDispersion;

            dbPmin = CumulativeNormalDistribution(dbInvKxMinNor);
            dbPmax = CumulativeNormalDistribution(dbInvKxMaxNor);

            // Avertissement
            if( dbPmax - dbPmin < 0.9 )
            {
                std::string sTmp;
                double dbTmp = (dbPmax - dbPmin)*100;
                sTmp = SystemUtil::ToString(2, dbTmp);

                loadingLogger << Logger::Warning << "WARNING: the min/max thresholds of the max concentration Kx of vehicle type " << pTypeVeh->GetLabel();
                loadingLogger << Logger::Warning << " must be set so that 90% of the samples are inside the defined range." << std::endl;
                loadingLogger << Logger::Warning << " ( currently " << sTmp << " ). " << std::endl;
                loadingLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
            }

            // Distribution stochastique de l'inverse de Kx
            pTypeVeh->SetInvKxDistr(true);
            pTypeVeh->SetInvKxDisp(dbInvKxDispersion);
            pTypeVeh->SetKxMin(dbKxMin);
            pTypeVeh->SetKxMax(dbKxMax);

            if(dbKxMin < m_dbKxmin)       // Sert Ã  la recherche du leader (si ditribution des Kx, il faut bien prendre la borne min.)
                m_dbKxmin = dbKxMin;
        }

        // Espacement des vÃ©hicules Ã  l'arrÃªt
        GetXmlAttributeValue(pXMLChild, "espacement_arret", dbTmp, &loadingLogger);
        pTypeVeh->SetEspacementArret( dbTmp );

        // Sources acoustiques
        pXMLSrcAcoustiques = m_pXMLUtil->SelectSingleNode("./SOURCES_ACOUSTIQUES", pXMLChild->getOwnerDocument(), (DOMElement*)pXMLChild);
        if(!pXMLSrcAcoustiques)
        {
            if(m_bSimuAcoustique)
            {
                loadingLogger << Logger::Error << "ERROR: at least one acoustic source must be defined for vehicle type " << pTypeVeh->GetLabel() << "." << std::endl;
                loadingLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                return false;
            }
        }
        else
        {
            XMLSize_t countj = pXMLSrcAcoustiques->getChildNodes()->getLength();
            for(XMLSize_t j=0; j<countj;j++)
            {
                DOMNode * pXmlChild = pXMLSrcAcoustiques->getChildNodes()->item(j);
                if (pXmlChild->getNodeType() != DOMNode::ELEMENT_NODE) continue;

                GetXmlAttributeValue(pXmlChild, "position", dbTmp, &loadingLogger);
                GetXmlAttributeValue(pXmlChild, "id_DB_LoiEmission", strTmp, &loadingLogger);
                pTypeVeh->AddSrcAcoustiques( dbTmp, (char*)strTmp.c_str() );
            }
        }

        // DÃ©cÃ©lÃ©ration
        GetXmlAttributeValue(pXMLChild, "deceleration", dbTmp, &loadingLogger);
        pTypeVeh->SetDeceleration( dbTmp );

        // DÃ©cÃ©lÃ©ration Ã©cart type
        GetXmlAttributeValue(pXMLChild, "deceleration_ecart_type", dbTmp, &loadingLogger);
        pTypeVeh->SetDecelerationEcartType( dbTmp );

        // AccÃ©lÃ©ration
        pXMLPlages = m_pXMLUtil->SelectSingleNode("./ACCELERATION_PLAGES", pXMLChild->getOwnerDocument(), (DOMElement*)pXMLChild);
        if(!pXMLPlages)
        {
            GetXmlAttributeValue(pXMLChild, "ax", dbAx, &loadingLogger);    // l'accÃ©lÃ©ration est dÃ©finie de faÃ§on constante
            pTypeVeh->PushPlageAcc(dbAx, DBL_MAX);
        }
        else
        {
            double dbVitSup = 0;
            double dbVitSupAnt = 0;

            XMLSize_t countj = pXMLPlages->getChildNodes()->getLength();
            for(XMLSize_t j=0; j<countj;j++)
            {
                DOMNode * xmlChildj = pXMLPlages->getChildNodes()->item(j);
                if (xmlChildj->getNodeType() != DOMNode::ELEMENT_NODE) continue;

                GetXmlAttributeValue(xmlChildj, "ax", dbAx, &loadingLogger);
                GetXmlAttributeValue(xmlChildj, "vit_sup", strTmp, &loadingLogger);
                if(strTmp=="infini")
                    dbVitSup = DBL_MAX;
                else
                    GetXmlAttributeValue(xmlChildj, "vit_sup", dbVitSup, &loadingLogger);

                // VÃ©rification de l'ordre des plages par rapport Ã  la vitesse
                if(dbVitSup <= dbVitSupAnt)
                {
                    loadingLogger << Logger::Error << "ERROR: the acceleration ranges of the vehicle type must be ordered by the upper threshold of the speed";
                    loadingLogger << Logger::Error << " ( vehicle type : " << pTypeVeh->GetLabel() << ")." << std::endl;
                    loadingLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                    return false;
                }

                pTypeVeh->PushPlageAcc(dbAx, dbVitSup);
                dbVitSupAnt = dbVitSup;
            }
        }

        // VÃ©rification de la cohÃ©rence entre le pas de temps de la simulation et les donnÃ©es
        // du diagramme fondamental (le pas de temps calculÃ© Ã  partir du diagramme fondamental
        // du type de vÃ©hicule ( - 1 / w * Kx ) considÃ©rÃ© doit Ãªtre supÃ©rieur au pas de temps
        // de la simulation)
        // Si stochastique, le test utilise la borne sup

        if( pTypeVeh->IsInvKxDistr() )
            dbKx = dbKxMax;

        if( pTypeVeh->IsWDistr() )
            dbW = dbWMax;

        double dbPasTempsOpt = - 1. / ( dbW * dbKx);

        if( fabs(dbPasTempsOpt - pas_de_temps) > 0.0001 )
        {
          if( (dbPasTempsOpt - pas_de_temps) < 0 )
          {
            loadingLogger << Logger::Error << "ERROR: Mismatch between simulation time step and data from the fundamental diagram for vehicle type " << pTypeVeh->GetLabel()<<std::endl;
            loadingLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.

            delete pTypeVeh;
            return false;
          }

          // Correction des erreurs numÃ©riques du calcul du pas de temps
          dbPasTempsOpt = Round(dbPasTempsOpt, 0.0001);

          // On vÃ©rifie que le pas de temps optimal pour le type de vÃ©hicule considÃ©rÃ© est
          // bien un multiple du pas de temps de la simulation
          if( fmod(dbPasTempsOpt,pas_de_temps) > 0)
          {
              loadingLogger << Logger::Warning << "WARNING: numerical viscosity risk for vehicles of type " << pTypeVeh->GetLabel() <<std::endl;
              loadingLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
          }
        }

        // Chargement valeur d'agressivitÃ©
        GetXmlAttributeValue(pXMLChild, "agressivite", dbTmp, &loadingLogger);
        pTypeVeh->SetValAgr( dbTmp );

        m_LstTypesVehicule.push_back( pTypeVeh );

        if(pTypeVeh->GetKx() < m_dbKxmin)       // Sert Ã  la recherche du leader
            m_dbKxmin = pTypeVeh->GetKx();

        if(pTypeVeh->GetDebitMax() > m_dbMaxDebitMax)
            m_dbMaxDebitMax = pTypeVeh->GetDebitMax();

        // ParamÃ¨tres de changement de voie :
        double dbTau, dbPiRabattement;
        GetXmlAttributeValue(pXMLChild, "chgtvoie_tau", dbTau, &loadingLogger);					// Tau
        GetXmlAttributeValue(pXMLChild, "pi_rabattement", dbPiRabattement, &loadingLogger);		// Pi de rabattement

        pTypeVeh->SetTauChgtVoie(dbTau);
        pTypeVeh->SetPiRabattement(dbPiRabattement);

        // Vitesse latÃ©rale
        double dbVitLat;
        GetXmlAttributeValue(pXMLChild, "vitesse_laterale", dbVitLat, &loadingLogger);					// Vitesse latÃ©rale
        pTypeVeh->SetVitesseLaterale(dbVitLat);

        // ParamÃ¨tres de dÃ©passement sur tronÃ§on opposÃ©
        double dbTauDepassement;
        GetXmlAttributeValue(pXMLChild, "depassement_tau", dbTauDepassement, &loadingLogger);					// Tau dÃ©passement
        pTypeVeh->SetTauDepassement(dbTauDepassement);

        // TODO - JORGE - voir pour renommer le paramÃ¨tre d'agressivitÃ© des tronÃ§ons, pour lever l'ambiguitÃ© avec le nouveau paramÃ¨tre agressivitÃ© de Jorge ?
        double dbAgressiveProp;
        GetXmlAttributeValue(pXMLChild, "proportion_agressive", dbAgressiveProp, &loadingLogger);
        pTypeVeh->SetAgressiveProportion(dbAgressiveProp);
        double dbShyProp;
        GetXmlAttributeValue(pXMLChild, "proportion_timide", dbShyProp, &loadingLogger);
        pTypeVeh->SetShyProportion(dbShyProp);
        double dbAgressiveEtaT;
        GetXmlAttributeValue(pXMLChild, "eta_T_agressif", dbAgressiveEtaT, &loadingLogger);
        pTypeVeh->SetAgressiveEtaT(dbAgressiveEtaT);
        double dbShyEtatT;
        GetXmlAttributeValue(pXMLChild, "eta_T_timide", dbShyEtatT, &loadingLogger);
        pTypeVeh->SetShyEtaT(dbShyEtatT);
        double dbEps0;
        GetXmlAttributeValue(pXMLChild, "epsylon_0", dbEps0, &loadingLogger);
        pTypeVeh->SetEpsylon0(dbEps0);
        double dbEps1;
        GetXmlAttributeValue(pXMLChild, "epsylon_1", dbEps1, &loadingLogger);
        pTypeVeh->SetEpsylon1(dbEps1);

        // Lois de poursuite
        DOMNode * pCarFollowings = m_pXMLUtil->SelectSingleNode("./LOIS_DE_POURSUITE", pXMLChild->getOwnerDocument(), (DOMElement*)pXMLChild);
        if(pCarFollowings)
        {
            double dbSum = 0;
            XMLSize_t countj = pCarFollowings->getChildNodes()->getLength();
            for(XMLSize_t j=0; j<countj;j++)
            {
                DOMNode * pXmlCarFollowing = pCarFollowings->getChildNodes()->item(j);
                if (pXmlCarFollowing->getNodeType() != DOMNode::ELEMENT_NODE) continue;

                GetXmlAttributeValue(pXmlCarFollowing, "id", strTmp, &loadingLogger);
                GetXmlAttributeValue(pXmlCarFollowing, "coeff", dbTmp, &loadingLogger);
                dbSum += dbTmp;

				//On test pour voir si l'ID existe bien dans les loi de poursuites scriptÃ©es
				bool idExist = false;
				bool idScripted = false;

				if (strTmp == "Newell" || strTmp == "IDM" || strTmp == "Gipps"){
					idExist = true;
				}
				if (m_pCarFollowingFactory->GetScriptedCarFollowingFromID(strTmp) != NULL)
				{
					bool idExist = true;
					bool idScripted = true;
				}

				if (!idExist)
				{
					loadingLogger << Logger::Error << "ERROR : the following law " << strTmp << " does not correspond to any scripted following laws, nor Newell, IDM or Gipps" << std::endl;
					loadingLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
					return false;
				}

                pTypeVeh->GetScriptedLawsCoeffs()[strTmp] = dbTmp;

                // construction du dictionnaire correspondant aux paramÃ¨tres de la loi de poursuite
                DOMNode * pNodeParametres = m_pXMLUtil->SelectSingleNode("PARAMETRES", pXmlCarFollowing->getOwnerDocument(), (DOMElement*)pXmlCarFollowing);
                if(pNodeParametres)
                {
                    GetPythonUtils()->enterChildInterpreter();
                    GetPythonUtils()->buildDictFromNode(pNodeParametres, pTypeVeh->GetScriptedLawsParams()[strTmp]);
                    GetPythonUtils()->exitChildInterpreter();
                }
            }
            // VÃ©rification de la somme des coefficients
            if( (dbSum-1) > countj*std::numeric_limits<double>::epsilon())
            {
                loadingLogger << Logger::Error << "ERROR : the sum of the car following coefficients for vehicle type " << pTypeVeh->GetLabel() << " can't be greater than 1 " << std::endl;
                loadingLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                return false;
            }
        }

    }//for(XMLSize_t i=0; i<counti;i++)

    deque<std::string> sMicroVehTypes = SystemUtil::split(sMicroVehicleType, ' ');
    for(size_t iMVT = 0; iMVT < sMicroVehTypes.size(); iMVT++)
    {
        TypeVehicule * pTV = GetVehicleTypeFromID(sMicroVehTypes[iMVT]);
        if(pTV)
        {
            m_MicroVehicleTypes.push_back(pTV);
        }
        else
        {
            loadingLogger << Logger::Warning << " WARNING : the vehicle type " << sMicroVehTypes[iMVT] << " defined in micro_types_vehicules doesn't exist." << endl;
            loadingLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
        }
		m_bMeso = true;
    }

//	m_LstAssignmentVehicleType.push_back( this->m_LstTypesVehicule.front() );

    // Types de vÃ©hicules Ã  restituer
    if(GetXmlAttributeValue(pRestitutionNode, "types_vehicules", strTmp, &loadingLogger))
    {
        deque<string> split = SystemUtil::split(strTmp, ' ');
        for(size_t splitIdx = 0; splitIdx<split.size(); splitIdx++)
        {
            TypeVehicule * pTypeVeh = GetVehicleTypeFromID(split[splitIdx]);
            if(!pTypeVeh)
            {
                loadingLogger << Logger::Warning << " WARNING : the vehicle type " << split[splitIdx] << " defined for trajectories restitution doesn't exist." << endl;
                loadingLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
            }
            else
            {
                m_setTypesRestitution.insert(pTypeVeh);
            }
        }
    }
    else
    {
        // par dÃ©faut, on restitue tous les types de vÃ©hicules
        m_setTypesRestitution.insert(m_LstTypesVehicule.begin(), m_LstTypesVehicule.end());
    }

    // Pour exploitation des valeurs par dÃ©faut de la periode d'aggregation des capteurs, beta, mu, gamma etc...
    double dbDefaultTCapteurs, dbDefaultGamma, dbDefaultMu, dbDefaultBeta, dbDefaultBetaInt, dbDefaultPosCptAval, dbDefaultTi, dbDefaultTt;
    GetXmlAttributeValue(pXMLNodeTrafic, "PeriodeAgregationCapteurs", dbDefaultTCapteurs, &loadingLogger);
    GetXmlAttributeValue(pXMLNodeTrafic, "Gamma", dbDefaultGamma, &loadingLogger);
    GetXmlAttributeValue(pXMLNodeTrafic, "Mu", dbDefaultMu, &loadingLogger);
    GetXmlAttributeValue(pXMLNodeTrafic, "Beta", dbDefaultBeta, &loadingLogger);
    GetXmlAttributeValue(pXMLNodeTrafic, "BetaInt", dbDefaultBetaInt, &loadingLogger);
    GetXmlAttributeValue(pXMLNodeTrafic, "pos_cpt_Av", dbDefaultPosCptAval, &loadingLogger);
    GetXmlAttributeValue(pXMLNodeTrafic, "ti", dbDefaultTi, &loadingLogger);
    GetXmlAttributeValue(pXMLNodeTrafic, "tt", dbDefaultTt, &loadingLogger);

    // Section TRAFIC / PARAMETRAGE_STATIONNEMENT
    DOMNode * pParkingNode = m_pXMLUtil->SelectSingleNode("./PARAMETRAGE_STATIONNEMENT", pXMLNodeTrafic->getOwnerDocument(), (DOMElement*)pXMLNodeTrafic);
    if (pParkingNode)
    {
        if (!m_pParkingParameters->LoadFromXML(this, pParkingNode, NULL, &loadingLogger))
        {
            return false;
        }
    }
    else
    {
        // Pour ne pas obliger les gens Ã  rajouer le noeud PARAMETRAGE_STATIONNEMENT.
        // Dans ce cas, on crÃ©e une variation temporelle avec du rÃ©sidentiel Ã  100% (mode par dÃ©faut Ã©quivalent Ã  gestion_stationnement = false)
        boost::shared_ptr<ParkingParametersVariation> defaultParkVar = boost::make_shared<ParkingParametersVariation>();
        m_pParkingParameters->AddVariation(defaultParkVar, m_dbDureeSimu);
    }


    // SECTION RESEAU
    GetXmlAttributeValue(pXMLScenarioNode, "reseau_id", m_ReseauID, &loadingLogger);

    pXMLReseau = m_pXMLUtil->SelectSingleNode("/ROOT_SYMUBRUIT/RESEAUX/RESEAU[@id=\"" + m_ReseauID + "\"]", m_XMLDocData);

    // SECTION RESEAU / CONNEXIONS / EXTREMITES
    pXMLNode = m_pXMLUtil->SelectSingleNode("./CONNEXIONS/EXTREMITES", pXMLReseau->getOwnerDocument(), (DOMElement*)pXMLReseau);

    // map indiquant si une connexion a un tronÃ§on amont ou aval
    std::map<std::string, bool> mapHasTronconAval;
    std::map<std::string, bool> mapHasTronconAmont;
    pXMLNodeTroncons = m_pXMLUtil->SelectSingleNode("./TRONCONS", pXMLReseau->getOwnerDocument(), (DOMElement*)pXMLReseau);

    XMLSize_t countit = pXMLNodeTroncons->getChildNodes()->getLength();
    for (XMLSize_t i = 0; i<countit; i++)
    {
        pXMLChild = pXMLNodeTroncons->getChildNodes()->item(i);
        if (pXMLChild->getNodeType() != DOMNode::ELEMENT_NODE) continue;

        GetXmlAttributeValue(pXMLChild, "id_eltamont", strEltAmontID, &loadingLogger);      // ElÃ©ment amont
        GetXmlAttributeValue(pXMLChild, "id_eltaval", strEltAvalID, &loadingLogger);        // ElÃ©ment aval

        mapHasTronconAval[strEltAmontID] = true;
        mapHasTronconAmont[strEltAvalID] = true;
    }

    // Map pour accÃ¨s rapide au noeud de la section RESEAU d'une connexion dÃ©finie par l'utilisateur (pour accÃ©lÃ©rer le retour sur les mouvements autorisÃ©s)
    std::map<std::string, DOMNode*> mapUserConnections;


    counti = pXMLNode->getChildNodes()->getLength();
    for(XMLSize_t i=0; i<counti;i++)
    {
        DOMNode *pXMLChildi = pXMLNode->getChildNodes()->item(i);
        if (pXMLChildi->getNodeType() != DOMNode::ELEMENT_NODE) continue;

        // ID
        GetXmlAttributeValue(pXMLChildi, "id", strTmp, &loadingLogger);
        strcpy(strID, strTmp.c_str());

        // On regarde si on doit crÃ©er une entrÃ©e et une sortie ou uniquement l'un des deux
        // en fonction des Ã©ventuels tronÃ§ons amonts et avals liÃ©s Ã  l'extremitÃ© en cours de lecture
        bool bIsEntree = mapHasTronconAval.find(strTmp) != mapHasTronconAval.end();
        bool bIsSortie = mapHasTronconAmont.find(strTmp) != mapHasTronconAmont.end();

        if(bIsSortie)
        {
            Sortie* pSortie;
            pSortie = new Sortie(strID, this);

            // rÃ©cupÃ©ration des paramÃ¨tres trafic pour les sorties

            // Capacite
            pXMLCapacites = m_pXMLUtil->SelectSingleNode("./EXTREMITES/EXTREMITE[@id=\"" + strTmp + "\"]/CAPACITES", pXMLNodeTrafic->getOwnerDocument(), (DOMElement*)pXMLNodeTrafic);
            if(pXMLCapacites)
            {
                // Chargement du dÃ©calage temporel des variations
                pSortie->GetLstCapacites()->SetLag( m_dbLag );

                XMLSize_t countj = pXMLCapacites->getChildNodes()->getLength();
                for(XMLSize_t j=0; j<countj;j++) // La capacitÃ© Ã©volue au cours de la simulation
                {
                    DOMNode * xmlChildj = pXMLCapacites->getChildNodes()->item(j);
                    if (xmlChildj->getNodeType() != DOMNode::ELEMENT_NODE) continue;

                    vector<PlageTemporelle*> plages;

                    if(!GetXmlDuree(xmlChildj, this, dbDuree, plages, &loadingLogger))
                    {
                        return false;
                    }
                    boost::shared_ptr<tracked_double> ptrackeddbTmp =  boost::shared_ptr<tracked_double>(new tracked_double);
                    GetXmlAttributeValue(xmlChildj, "valeur", *ptrackeddbTmp, &loadingLogger);      // capacitÃ©
                    if(plages.size() != 0)
                    {
                        // Ajout de la variation pour chacune des plages temporelles dÃ©finies
                        for(size_t iPlage = 0; iPlage < plages.size(); iPlage++)
                        {
                            pSortie->GetLstCapacites()->AddVariation(plages[iPlage], ptrackeddbTmp );
                        }
                    }
                    else
                    {
                        pSortie->GetLstCapacites()->AddVariation(dbDuree, ptrackeddbTmp );
                    }
                }

                // vÃ©rifications de rigueur en mode plages temporelles : l'ensemble des plages doit couvrir toute la simulation !
                if(!pSortie->GetLstCapacites()->CheckPlagesTemporelles(m_dbDureeSimu))
                {
                    loadingLogger << Logger::Error << "ERROR : The time frames defined for capacities of output " << pSortie->GetInputID() << " don't cover the whole simulation duration !" << std::endl;
                    loadingLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                    return false;
                }
            }
            else     // Pas de restriction de capacitÃ© de sortie
            {
                boost::shared_ptr<tracked_double> pTrackeddbTmp = boost::make_shared<tracked_double>(NONVAL_DOUBLE);
                pSortie->GetLstCapacites()->SetLag( 0 );
                pSortie->GetLstCapacites()->AddVariation(m_dbDureeSimu, pTrackeddbTmp);
            }

            Liste_sortie.push_back(pSortie);
            this->m_LstUserCnxs.push_back(pSortie);
            Liste_destinations.push_back(pSortie);
            mapUserConnections[pSortie->GetInputID()] = pXMLChildi;
        }

        if(bIsEntree)
        {
            pEntree = new Entree(strID, this);

            Liste_entree.push_back(pEntree);
            Liste_origines.push_back(pEntree);
            this->m_LstUserCnxs.push_back(pEntree);
            mapUserConnections[strTmp] = pXMLChildi;

            // RÃ©cupÃ©ration des donnÃ©es de trafic
            pXMLChild = m_pXMLUtil->SelectSingleNode("./EXTREMITES/EXTREMITE[@id=\"" + strTmp + "\"]", pXMLNodeTrafic->getOwnerDocument(), (DOMElement*)pXMLNodeTrafic);

            if(!pXMLChild)
            {
                loadingLogger << Logger::Error << "ERROR: no traffic parameters definition for input " << strID << std::endl;
                loadingLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                return false;
            }

            // remarque : on charge le type de crÃ©ation des vÃ©hicules plus tard car il faut connaitre la liste des sorties
            if(!LoadDemandesAgressivite(pEntree, pXMLChild, m_dbLag, &loadingLogger ))
            {
                return false;
            }

            // Les rÃ©partitions par voie ainsi que les types de vÃ©hicule seront chargÃ©es plus tard
            // (on a besoin de connaitre l'Ã©lÃ©ment en aval)
        }
    }

    // SECTION RESEAU / CONNEXIONS / PARKINGS
    pXMLNode = m_pXMLUtil->SelectSingleNode("./CONNEXIONS/PARKINGS", pXMLReseau->getOwnerDocument(), (DOMElement*)pXMLReseau);

    if(pXMLNode)
    {
        counti = pXMLNode->getChildNodes()->getLength();
        for(XMLSize_t i=0; i<counti;i++)
        {
            DOMNode *pXMLChildi = pXMLNode->getChildNodes()->item(i);
            if (pXMLChildi->getNodeType() != DOMNode::ELEMENT_NODE) continue;

            // ID
            GetXmlAttributeValue(pXMLChildi, "id", strTmp, &loadingLogger);
            strcpy(strID, strTmp.c_str());

            // On regarde si on doit crÃ©er une entrÃ©e et une sortie ou uniquement l'un des deux
            // en fonction des Ã©ventuels tronÃ§ons amonts et avals liÃ©s Ã  l'extremitÃ© en cours de lecture
            pXMLNodeTroncons = m_pXMLUtil->SelectSingleNode("./TRONCONS", pXMLReseau->getOwnerDocument(), (DOMElement*)pXMLReseau);
            bool bIsEntree = false;
            bool bIsSortie = false;
            countit = pXMLNodeTroncons->getChildNodes()->getLength();
            for(XMLSize_t i=0; i<countit && (!bIsEntree || !bIsSortie);i++)
            {
                pXMLChild = pXMLNodeTroncons->getChildNodes()->item(i);
                if (pXMLChild->getNodeType() != DOMNode::ELEMENT_NODE) continue;

                GetXmlAttributeValue(pXMLChild, "id_eltamont", strEltAmontID, &loadingLogger);      // ElÃ©ment amont
                GetXmlAttributeValue(pXMLChild, "id_eltaval", strEltAvalID, &loadingLogger);        // ElÃ©ment aval

                if(!strEltAmontID.compare(strID))
                {
                    bIsEntree = true;
                }
                if(!strEltAvalID.compare(strID))
                {
                    bIsSortie = true;
                }
            }

            // RÃ©cupÃ©ration des donnÃ©es de trafic
            pXMLChild = m_pXMLUtil->SelectSingleNode("./PARKINGS/PARKING[@id=\"" + strTmp + "\"]", pXMLNodeTrafic->getOwnerDocument(), (DOMElement*)pXMLNodeTrafic);
            bool bParkingZone = false;
            if(!pXMLChild)
            {
                // Dans ce cas, on est peut-Ãªtre dans le cas d'un parking interne Ã  une zone :
                pXMLChild = m_pXMLUtil->SelectSingleNode("./ZONES_DE_TERMINAISON/ZONE_DE_TERMINAISON/PARKINGS/PARKING[@id=\"" + strTmp + "\"]", pXMLNodeTrafic->getOwnerDocument(), (DOMElement*)pXMLNodeTrafic);

                if(!pXMLChild)
                {
                    loadingLogger << Logger::Error << "ERROR: no traffic parameters definition for parking " << strID << std::endl;
                    loadingLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                    return false;
                }
                else
                {
                    bParkingZone = true;
                }
            }

            DOMNode * pNodeParkingParams = NULL;
            if(bParkingZone)
            {
                pNodeParkingParams = pXMLChild;
            }
            else
            {
                pNodeParkingParams = m_pXMLUtil->SelectSingleNode("./PARAMETRES", pXMLChild->getOwnerDocument(), (DOMElement*)pXMLChild);
            }

            int stockMax;
            GetXmlAttributeValue(pNodeParkingParams, "stock_max", stockMax, &loadingLogger);

            int stockInit;
            GetXmlAttributeValue(pNodeParkingParams, "stock_initial", stockInit, &loadingLogger);

            double dbInterTpsSortie;
            GetXmlAttributeValue(pNodeParkingParams, "inter_temps_sortie", dbInterTpsSortie, &loadingLogger);

            double dbInterTpsEntree;
            GetXmlAttributeValue(pNodeParkingParams, "inter_temps_entree", dbInterTpsEntree, &loadingLogger);

            bool bIsParcRelais;
            GetXmlAttributeValue(pNodeParkingParams, "parc_relais", bIsParcRelais, &loadingLogger);

            double dbParcRelaisMaxDistance;
            GetXmlAttributeValue(pNodeParkingParams, "parc_relais_rayon", dbParcRelaisMaxDistance, &loadingLogger);

            // Lecture des types de vÃ©hicules interdits...
            std::vector<TypeVehicule*> lstVehInterdits;
            DOMXPathResult * pXMLNodeList = m_pXMLUtil->SelectNodes("./TYPE_VEHICULE_INTERDIT", pNodeParkingParams->getOwnerDocument(), (DOMElement*)pNodeParkingParams);
            XMLSize_t countj = pXMLNodeList->getSnapshotLength();
            for(XMLSize_t j=0; j<countj; j++)
            {
                pXMLNodeList->snapshotItem(j);
                DOMNode * pNodeTypeVehInterdit = pXMLNodeList->getNodeValue();

                std::string typeID;
                GetXmlAttributeValue(pNodeTypeVehInterdit, "type", typeID, &loadingLogger);

                TypeVehicule * typeVehInterdit = GetVehicleTypeFromID(typeID);
                lstVehInterdits.push_back(typeVehInterdit);
            }
            pXMLNodeList->release();
            // Fin types de vÃ©hicules interdits

            Parking * pParking = new Parking(strID, this, stockMax, stockInit, dbInterTpsEntree, dbInterTpsSortie, lstVehInterdits, bIsParcRelais, dbParcRelaisMaxDistance);

            if(bIsEntree)
            {
                ConnectionPonctuel* pParkingEntree = new ConnectionPonctuel(strID, this);

                if(!bParkingZone)
                {
                    if(!LoadDemandesAgressivite(pParking, pXMLChild, m_dbLag, &loadingLogger))
                    {
                        return false;
                    }
                }

                Position parkingPos(pParkingEntree);
                pParking->SetOutputPosition(parkingPos);

                this->m_LstUserCnxs.push_back(pParkingEntree);
                mapUserConnections[strID] = pXMLChildi;
                this->Liste_origines.push_back(pParking);
            }

            if(bIsSortie)
            {
                ConnectionPonctuel* pParkingSortie = new ConnectionPonctuel(strID, this);

                Position parkingPos(pParkingSortie);
                pParking->SetInputPosition(parkingPos);

                this->m_LstUserCnxs.push_back(pParkingSortie);
                mapUserConnections[strID] = pXMLChildi;
                this->Liste_destinations.push_back(pParking);
            }

            Liste_parkings.push_back(pParking);
        }
    }


    // SECTION RESEAU / ZONES_DE_TERMINAISON
    pXMLNode = m_pXMLUtil->SelectSingleNode("./ZONES_DE_TERMINAISON", pXMLReseau->getOwnerDocument(), (DOMElement*)pXMLReseau);

    if(pXMLNode)
    {
        counti = pXMLNode->getChildNodes()->getLength();
        for(XMLSize_t i=0; i<counti;i++)
        {
            DOMNode *pXMLChildi = pXMLNode->getChildNodes()->item(i);
            if (pXMLChildi->getNodeType() != DOMNode::ELEMENT_NODE) continue;

            // ID
            GetXmlAttributeValue(pXMLChildi, "id", strTmp, &loadingLogger);
            strcpy(strID, strTmp.c_str());

            ZoneDeTerminaison* pZone = new ZoneDeTerminaison(strTmp, this);

            // RÃ©cupÃ©ration des donnÃ©es de trafic
            pXMLChildi = m_pXMLUtil->SelectSingleNode("./ZONES_DE_TERMINAISON/ZONE_DE_TERMINAISON[@id=\"" + strTmp + "\"]", pXMLNodeTrafic->getOwnerDocument(), (DOMElement*)pXMLNodeTrafic);

            if(!pXMLChildi)
            {
                loadingLogger << Logger::Error << "ERROR: no traffic parameters definition for termination zone " << strID << std::endl;
                loadingLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                return false;
            }

            if(!LoadDemandesAgressivite(pZone, pXMLChildi, m_dbLag, &loadingLogger))
            {
                return false;
            }

            Liste_zones.push_back(pZone);
        }
    }


    // SECTION RESEAU / CONNEXIONS / REPARTITEURS
    pXMLNode = m_pXMLUtil->SelectSingleNode("./CONNEXIONS/REPARTITEURS", pXMLReseau->getOwnerDocument(), (DOMElement*)pXMLReseau);

    if(pXMLNode)
    {
        XMLSize_t counti = pXMLNode->getChildNodes()->getLength();
        for(XMLSize_t i=0; i<counti;i++)
        {
            pXMLChild = pXMLNode->getChildNodes()->item(i);
            if (pXMLChild->getNodeType() != DOMNode::ELEMENT_NODE) continue;

            // ID
            GetXmlAttributeValue(pXMLChild, "id", strTmp, &loadingLogger);
            strcpy(strID, strTmp.c_str());

            mapUserConnections[strTmp] = pXMLChild;

            // rÃ©cupÃ©ration des paramÃ¨tres de simulation
            pXMLChild = m_pXMLUtil->SelectSingleNode("./ELEMENTS/ELEMENT[@id=\"" + strTmp + "\"]", pXMLNodeSimulation->getOwnerDocument(), (DOMElement*)pXMLNodeSimulation);

            // resolution
            if(!GetXmlAttributeValue(pXMLChild, "resolution", cType, &loadingLogger)) cType = cDefaultResolution;

            pRepartiteur = new Repartiteur(strID, cType, this);

            Liste_repartiteurs[strTmp] = pRepartiteur;
            this->m_LstUserCnxs.push_back(pRepartiteur);
        }
    }

    // SECTION RESEAU / CONNEXIONS / CONVERGENTS
    double dbGamma, dbMu, dbTcpt;
    pXMLNode = m_pXMLUtil->SelectSingleNode("./CONNEXIONS/CONVERGENTS", pXMLReseau->getOwnerDocument(), (DOMElement*)pXMLReseau);

    // Liste des noeuds correspondant aux convergents de type "insertion"
    std::vector<DOMNode*> Convergents_Insertion;

    if(pXMLNode)
    {
        XMLSize_t counti = pXMLNode->getChildNodes()->getLength();
        for(XMLSize_t i=0; i<counti;i++)
        {
            //pXMLChild = pXMLNode->ChildNodes[i];
            pXMLChild = pXMLNode->getChildNodes()->item(i);
            if (pXMLChild->getNodeType() != DOMNode::ELEMENT_NODE) continue;

            // cas particulier du convergent de type "insertion"
            GetXmlAttributeValue(pXMLChild, "type", strTmp, &loadingLogger);
            if(!strTmp.compare("insertion"))
            {
                // pas de crÃ©ation de convergent (il sera remplacÃ© par un troncon entre deux rÃ©partiteurs)
                Convergents_Insertion.push_back(pXMLChild);
            }
            else
            {
                // ID
                GetXmlAttributeValue(pXMLChild, "id", strTmp, &loadingLogger);
                strcpy(strID, strTmp.c_str());

                mapUserConnections[strTmp] = pXMLChild;

                // RÃ©cupÃ©ration des paramÃ¨tres trafic correspondants
                pXMLChild = m_pXMLUtil->SelectSingleNode("./CONNEXIONS_INTERNES/CONNEXION_INTERNE[@id=\"" + strTmp + "\"]", pXMLNodeTrafic->getOwnerDocument(), (DOMElement*)pXMLNodeTrafic);
                if(!GetXmlAttributeValue(pXMLChild, "Gamma", dbGamma, &loadingLogger)) dbGamma = dbDefaultGamma;
                if(!GetXmlAttributeValue(pXMLChild, "Mu", dbMu, &loadingLogger)) dbMu = dbDefaultMu;
                if(!GetXmlAttributeValue(pXMLChild, "PeriodeAgregationCapteurs", dbTcpt, &loadingLogger)) dbTcpt = dbDefaultTCapteurs;

                pConvergent = new Convergent(strID, dbTcpt, dbGamma, dbMu, this);
                Liste_convergents[strTmp] = pConvergent;
                this->m_LstUserCnxs.push_back(pConvergent);
            }
        }
    }

    // SECTION RESEAU / CONNEXIONS / GIRATOIRES
    pXMLNode = m_pXMLUtil->SelectSingleNode("./CONNEXIONS/GIRATOIRES", pXMLReseau->getOwnerDocument(), (DOMElement*)pXMLReseau);
    Giratoire*  pGiratoire;
    double dbVitMax, dbTAgr, dbBeta, dbBetaInt;
    bool bTraversees;
    int nm;

    if(pXMLNode)
    {
        XMLSize_t counti = pXMLNode->getChildNodes()->getLength();
        for(XMLSize_t i=0; i<counti;i++)
        {
            pXMLChild = pXMLNode->getChildNodes()->item(i);
            if (pXMLChild->getNodeType() != DOMNode::ELEMENT_NODE) continue;

            // ID
            GetXmlAttributeValue(pXMLChild, "id", strTmp, &loadingLogger);
            strcpy(strID, strTmp.c_str());
            dbVitMax = DBL_MAX;
            GetXmlAttributeValue(pXMLChild, "vit_max", dbVitMax, &loadingLogger);

            // rÃ©cupÃ©ration des attributs trafic associÃ©s
            DOMNode * pTraficParams = m_pXMLUtil->SelectSingleNode("./CONNEXIONS_INTERNES/CONNEXION_INTERNE[@id=\"" + strTmp + "\"]", pXMLNodeTrafic->getOwnerDocument(), (DOMElement*)pXMLNodeTrafic);

            if(!pTraficParams)
            {
                loadingLogger << Logger::Error << "ERROR: no traffic parameters definition for roundabout " << strID << std::endl;
                loadingLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                return false;
            }

            if(!GetXmlAttributeValue(pTraficParams, "PeriodeAgregationCapteurs", dbTAgr, &loadingLogger)) dbTAgr = dbDefaultTCapteurs;
            if(!GetXmlAttributeValue(pTraficParams, "Gamma", dbGamma, &loadingLogger)) dbGamma = dbDefaultGamma;
            if(!GetXmlAttributeValue(pTraficParams, "Mu", dbMu, &loadingLogger)) dbMu = dbDefaultMu;
            GetXmlAttributeValue(pXMLChild, "nb_voie", nVoie, &loadingLogger);
            GetXmlAttributeValue(pXMLChild, "LargeurVoie", dbLargeurVoie, &loadingLogger);
            GetXmlAttributeValue(pXMLChild, "Type", cType, &loadingLogger);
            GetXmlAttributeValue(pXMLChild, "m", nm, &loadingLogger);
            if(!GetXmlAttributeValue(pTraficParams, "Beta", dbBeta, &loadingLogger)) dbBeta = dbDefaultBeta;
            if(!GetXmlAttributeValue(pTraficParams, "BetaInt", dbBetaInt, &loadingLogger)) dbBetaInt = dbDefaultBetaInt;
            if(!GetXmlAttributeValue(pTraficParams, "traversees", bTraversees, &loadingLogger)) bTraversees = true;

            strTmp = "";
            if( !GetXmlAttributeValue(pXMLChild, "revetement", strTmp, &loadingLogger) && m_bSimuAcoustique)              // Revetement (obligatoire uniquement dans le cas d'une simulation acoustique)
            {
                loadingLogger << Logger::Error << "ERROR: the road surface of the roundabout " << strID << " is not defined, it is mandatory to acoustic simulation."  <<std::endl;
                loadingLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                return false;
            }
            else
                strcpy(strRevetement, strTmp.c_str());

            pGiratoire = new Giratoire(strID, dbVitMax, strRevetement, dbTAgr, dbGamma, dbMu, nVoie, dbLargeurVoie, cType, nm, dbBeta, dbBetaInt, bTraversees, this);

            // Chargement ZLevel
            int nZlevel;
            GetXmlAttributeValue(pXMLChild, "z_level_crossing", nZlevel, &loadingLogger);
            pGiratoire->SetZLevel(nZlevel);

            Liste_giratoires.push_back(pGiratoire);
            this->m_LstUserCnxs.push_back(pGiratoire);
            mapUserConnections[pGiratoire->GetLabel()] = pXMLChild;
        }
    }

    // SECTION RESEAU / CONNEXIONS / CARREFOUR A FEUX
    double  dbLong;
    pXMLNode = m_pXMLUtil->SelectSingleNode("./CONNEXIONS/CARREFOURSAFEUX", pXMLReseau->getOwnerDocument(), (DOMElement*)pXMLReseau);

    if(pXMLNode)
    {
        XMLSize_t counti = pXMLNode->getChildNodes()->getLength();
        for(XMLSize_t i=0; i<counti;i++)
        {
            pXMLChild = pXMLNode->getChildNodes()->item(i);
            if (pXMLChild->getNodeType() != DOMNode::ELEMENT_NODE) continue;

            // ID
            GetXmlAttributeValue(pXMLChild, "id", strTmp, &loadingLogger);
            strcpy(strID, strTmp.c_str());

            dbVitMax = DBL_MAX;
            GetXmlAttributeValue(pXMLChild, "vit_max", dbVitMax, &loadingLogger);

            // rÃ©cupÃ©ration des attributs trafic associÃ©s
            DOMNode * pTraficParams = m_pXMLUtil->SelectSingleNode("./CONNEXIONS_INTERNES/CONNEXION_INTERNE[@id=\"" + strTmp + "\"]", pXMLNodeTrafic->getOwnerDocument(), (DOMElement*)pXMLNodeTrafic);

            if(!GetXmlAttributeValue(pTraficParams, "PeriodeAgregationCapteurs", dbTAgr, &loadingLogger)) dbTAgr = dbDefaultTCapteurs;
            if(!GetXmlAttributeValue(pTraficParams, "Gamma", dbGamma, &loadingLogger)) dbGamma = dbDefaultGamma;
            if(!GetXmlAttributeValue(pTraficParams, "Mu", dbMu, &loadingLogger)) dbMu = dbDefaultMu;
            if(!GetXmlAttributeValue(pTraficParams, "traversees", bTraversees, &loadingLogger)) bTraversees = true;

            // rÃ©cupÃ©ration des attributs simulation associÃ©s
            DOMNode * pSimuParams = m_pXMLUtil->SelectSingleNode("./ELEMENTS/ELEMENT[@id=\"" + strTmp + "\"]", pXMLNodeSimulation->getOwnerDocument(), (DOMElement*)pXMLNodeSimulation);

            if(!GetXmlAttributeValue(pSimuParams, "longueur_cell_acoustique", dbLong, &loadingLogger))
                dbLong = dbDefaultCellAcouLength;     // Longueur dÃ©sirÃ©e des cellules acoustiques

            pCAF = new CarrefourAFeuxEx(strID, dbVitMax, dbTAgr, dbGamma, dbMu, dbLong, bTraversees, this);

            // Chargement ZLevel
            int nZlevel;
            GetXmlAttributeValue(pXMLChild, "z_level_crossing", nZlevel, &loadingLogger);
            pCAF->SetZLevel(nZlevel);

            Liste_carrefoursAFeux.push_back(pCAF);
            this->m_LstUserCnxs.push_back(pCAF);
            mapUserConnections[pCAF->GetID()] = pXMLChild;
        }
    }

    // SECTION RESEAU / TRONCONS

    // Pour opti : accÃ¨s prÃ©liminaire aux noeuds simulation et trafic des tronÃ§ons
    std::map<std::string, DOMNode*> mapTronconSimuNode;
    std::map<std::string, DOMNode*> mapTronconTrafNode;
    pXMLNode = m_pXMLUtil->SelectSingleNode("./TRONCONS", pXMLNodeTrafic->getOwnerDocument(), (DOMElement*)pXMLNodeTrafic);
    if (pXMLNode)
    {
        countit = pXMLNode->getChildNodes()->getLength();
        for (XMLSize_t i = 0; i < countit; i++)
        {
            pXMLChild = pXMLNode->getChildNodes()->item(i);
            if (pXMLChild->getNodeType() != DOMNode::ELEMENT_NODE) continue;

            GetXmlAttributeValue(pXMLChild, "id", strTmp, &loadingLogger);                      // ID

            mapTronconTrafNode[strTmp] = pXMLChild;
        }
    }
    pXMLNode = m_pXMLUtil->SelectSingleNode("./ELEMENTS", pXMLNodeSimulation->getOwnerDocument(), (DOMElement*)pXMLNodeSimulation);
    if (pXMLNode)
    {
        countit = pXMLNode->getChildNodes()->getLength();
        for (XMLSize_t i = 0; i < countit; i++)
        {
            pXMLChild = pXMLNode->getChildNodes()->item(i);
            if (pXMLChild->getNodeType() != DOMNode::ELEMENT_NODE) continue;

            GetXmlAttributeValue(pXMLChild, "id", strTmp, &loadingLogger);                      // ID

            mapTronconSimuNode[strTmp] = pXMLChild;
        }
    }


    std::map<std::string, DOMNode*> mapTronconReseauNode;
    pXMLNode = m_pXMLUtil->SelectSingleNode("./TRONCONS", pXMLReseau->getOwnerDocument(), (DOMElement*)pXMLReseau);

    std::vector<Repartiteur*> repartiteursInsertion;
    std::vector<Tuyau*> tronconsAvecOppose;   // liste des tuyaux ayant un troncon opposÃ©
    std::vector<std::string> tronconsOpposes; // liste des troncons opposÃ©s correspondants

    countit = pXMLNode->getChildNodes()->getLength();
    for(XMLSize_t i=0; i<countit;i++)
    {
        pXMLChild = pXMLNode->getChildNodes()->item(i);
        if (pXMLChild->getNodeType() != DOMNode::ELEMENT_NODE) continue;

        // Chargement des caractÃ©ristiques du tronÃ§ons
        GetXmlAttributeValue(pXMLChild, "id", strTmp, &loadingLogger);                      // ID
        strcpy(strID, strTmp.c_str());

        mapTronconReseauNode[strTmp] = pXMLChild;

        GetXmlAttributeValue(pXMLChild, "id_eltamont", strEltAmontID, &loadingLogger);      // ElÃ©ment amont
        GetXmlAttributeValue(pXMLChild, "id_eltaval", strEltAvalID, &loadingLogger);        // ElÃ©ment aval

        // pour rÃ©cupÃ©ration des paramÃ¨tres de simulation associÃ©s
        DOMNode * pSimuParams = mapTronconSimuNode[strTmp];

        if( strEltAmontID == strEltAvalID)
        {
            loadingLogger << Logger::Error << "ERROR: the upstream and downstream nodes of the link " << strID << " can't be identical"  <<std::endl;
            loadingLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
            return false;
        }

        strTmp = "";
        if( !GetXmlAttributeValue(pXMLChild, "revetement", strTmp, &loadingLogger) && m_bSimuAcoustique)              // Revetement (obligatoire uniquement dans le cas d'une simulation acoustique)
        {
            loadingLogger << Logger::Error << "ERROR: the road surface for link " << strID << " is undefined, and is mandatory for acoustic simulation"  <<std::endl;
            loadingLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
            return false;
        }
        else
            strcpy(strRevetement, strTmp.c_str());


        if(!GetXmlAttributeValue(pSimuParams, "resolution", cResolution, &loadingLogger)) cResolution = cDefaultResolution;     // resolution
        GetXmlAttributeValue(pXMLChild, "largeur_voie", dbLargeurVoie, &loadingLogger);     // Largeur d'une voie
        GetXmlAttributeValue(pXMLChild, "extremite_amont", ptExtAmont, &loadingLogger);           // ExtrÃ©mitÃ© amont
        GetXmlAttributeValue(pXMLChild, "extremite_aval", ptExtAval, &loadingLogger);             // ExtrÃ©mitÃ© aval
        GetXmlAttributeValue(pXMLChild, "longueur", dbLongueur, &loadingLogger);            // Longueur

        dbVitReg = DBL_MAX;
        GetXmlAttributeValue(pXMLChild, "vit_reg", dbVitReg, &loadingLogger);               // Vitesse rÃ©glementaire (par dÃ©faut)

        dbVitCat = DBL_MAX;
        GetXmlAttributeValue(pXMLChild, "vit_cat", dbVitCat, &loadingLogger);               // Vitesse rÃ©glementaire (par dÃ©faut)

        std::string roadLabel;
        GetXmlAttributeValue(pXMLChild, "nom_axe", roadLabel, &loadingLogger);               // Nom de l'axe / la route
        GetXmlAttributeValue(pXMLChild, "nb_voie", nVoie, &loadingLogger);                    // nombre de voie

        // Nombre ou longueur des cellules acoustiques du tronÃ§on
        if(pSimuParams)
        {
            if(!GetXmlAttributeValue(pSimuParams, "nb_cell_acoustique", nCellAcoustique, &loadingLogger)) nCellAcoustique = nDefaultNbCellAcou; // nCellAcoustique
            if(!GetXmlAttributeValue(pSimuParams, "longueur_cell_acoustique", dbCellAcouLength, &loadingLogger)) dbCellAcouLength = dbDefaultCellAcouLength;
        }
        else
        {
            nCellAcoustique = nDefaultNbCellAcou;
            dbCellAcouLength = dbDefaultCellAcouLength;
        }

        ConnectionPonctuel *pCnxAmont, *pCnxAval;
        char cTypeEltAmont, cTypeEltAval;

        pCnxAmont = GetConnectionFromID(strEltAmontID, cTypeEltAmont);
        if(!pCnxAmont)
        {
            SymuViaTripNode * pOrigin = GetOrigineFromID(strEltAmontID, cTypeEltAmont);
            if(pOrigin)
            {
                pCnxAmont = dynamic_cast<ConnectionPonctuel*>(pOrigin->GetOutputConnexion());
            }
        }
        pCnxAval = GetConnectionFromID(strEltAvalID, cTypeEltAval);
        if(!pCnxAval)
        {
            SymuViaTripNode * pDestination = GetDestinationFromID(strEltAvalID, cTypeEltAval);
            if(pDestination)
            {
                pCnxAval = dynamic_cast<ConnectionPonctuel*>(pDestination->GetInputConnexion());
            }
        }

        // **************************************************************************************
        // Cas particulier des tronÃ§on reliÃ©s Ã  un rÃ©partiteur crÃ©Ã© pour convergent d'insertion.
        // **************************************************************************************
        // on est obligÃ© de crÃ©er ici le rÃ©partiteur aval associÃ© Ã  l'insertion pour pouvoir crÃ©er le troncon courant qui en a besoin en amont
        if(pCnxAmont == NULL)
        {
            // tentative de rÃ©cupÃ©ration de la dÃ©fintion du convergent d'insertion dans le fichier XML
            DOMNode * XMLtmp = m_pXMLUtil->SelectSingleNode(("./CONNEXIONS/CONVERGENTS/CONVERGENT[@id=\"" + strEltAmontID + "\"]").c_str(), pXMLReseau->getOwnerDocument(), (DOMElement*)pXMLReseau );
            if(XMLtmp)
            {
                // encore faut-il que ce soit bien un convergent d'insertion
                GetXmlAttributeValue(XMLtmp, "type", strTmp, &loadingLogger);
                if(!strTmp.compare("insertion"))
                {
                    pRepartiteur = NULL;
                    std::string strIDRepInsAval = "R_" + strEltAmontID + "_INSERTION_AVAL";
                    for(size_t repInsIdx = 0; repInsIdx < repartiteursInsertion.size(); repInsIdx++)
                    {
                        if(!repartiteursInsertion[repInsIdx]->GetID().compare(strIDRepInsAval))
                        {
                            pRepartiteur = repartiteursInsertion[repInsIdx];
                        }
                    }
                    if(pRepartiteur == NULL)
                    {
                        // CrÃ©ation du rÃ©partiteur entre le tronÃ§on d'insertion (Ã  crÃ©er aprÃ¨s les troncons dÃ©finis par l'utilisateur)
                        // et le troncon aval dÃ©fini par l'utilisateur
                        pRepartiteur = new Repartiteur(strIDRepInsAval, 'H', this);
                        repartiteursInsertion.push_back(pRepartiteur);
                    }

                    // la vÃ©ritable connexion amont du tuyau est ce rÃ©partiteur.
                    pCnxAmont = pRepartiteur;
                    cTypeEltAmont = 'R';
                }
            }
        }
        // on est obligÃ© de crÃ©er ici le rÃ©partiteur amont associÃ© Ã  l'insertion pour pouvoir crÃ©er le troncon courant qui en a besoin en aval
        if(pCnxAval == NULL)
        {
            // tentative de rÃ©cupÃ©ration de la dÃ©fintion du convergent d'insertion dans le fichier XML
            DOMNode * XMLtmp = m_pXMLUtil->SelectSingleNode(("./CONNEXIONS/CONVERGENTS/CONVERGENT[@id=\"" + strEltAvalID + "\"]").c_str(), pXMLReseau->getOwnerDocument(), (DOMElement*)pXMLReseau );
            if(XMLtmp)
            {
                // encore faut-il que ce soit bien un convergent d'insertion
                GetXmlAttributeValue(XMLtmp, "type", strTmp, &loadingLogger);
                if(!strTmp.compare("insertion"))
                {
                    pRepartiteur = NULL;
                    std::string strIDRepInsAmont = "R_" + strEltAvalID + "_INSERTION_AMONT";
                    for(size_t repInsIdx = 0; repInsIdx < repartiteursInsertion.size(); repInsIdx++)
                    {
                        if(!repartiteursInsertion[repInsIdx]->GetID().compare(strIDRepInsAmont))
                        {
                            pRepartiteur = repartiteursInsertion[repInsIdx];
                        }
                    }
                    if(pRepartiteur == NULL)
                    {
                        // CrÃ©ation du rÃ©partiteur entre les tronÃ§ons amont du convergent d'insertion et le troncon d'insertion
                        pRepartiteur = new Repartiteur("R_" + strEltAvalID + "_INSERTION_AMONT", 'H', this);
                        repartiteursInsertion.push_back(pRepartiteur);
                    }

                    // la vÃ©ritable connexion amont du tuyau est ce rÃ©partiteur.
                    pCnxAval = pRepartiteur;
                    cTypeEltAval = 'R';
                }
            }
        }
        // ******************************************************************************************
        // Fin cas particulier des tronÃ§on reliÃ©s Ã  un rÃ©partiteur crÃ©Ã© pour convergent d'insertion.
        // ******************************************************************************************

        // rÃ©cupÃ©ration une fois pour toutes des noeuds fils du tronÃ§on pour Ã©viter les couteux SelectSingleNode pour chaque type de noeud fils
        DOMNode * XMLVoiesReservees = NULL;
        DOMNode * XMLPts = NULL;
        DOMNode * XMLVitRegs = NULL;
        DOMNode * XMLVoiesInterdites = NULL;
        DOMNode * XMLTerrePleins = NULL;
        DOMNode * XMLZonesDpssmtInterdit = NULL;
        DOMNode * XMLZLevelCrossings = NULL;
        XMLSize_t counttronconchilds = pXMLChild->getChildNodes()->getLength();
        for (XMLSize_t iTronconChild = 0; iTronconChild < counttronconchilds; iTronconChild++)
        {
            DOMNode * pXMLTronconChild = pXMLChild->getChildNodes()->item(iTronconChild);
            if (pXMLTronconChild->getNodeType() != DOMNode::ELEMENT_NODE) continue;

            std::string tronconChildNodeName = US(pXMLTronconChild->getNodeName());
            if (!tronconChildNodeName.compare("VOIES_RESERVEES"))
            {
                XMLVoiesReservees = pXMLTronconChild;
            }
            else if (!tronconChildNodeName.compare("POINTS_INTERNES"))
            {
                XMLPts = pXMLTronconChild;
            }
            else if (!tronconChildNodeName.compare("VOIES_INTERDITES"))
            {
                XMLVoiesInterdites = pXMLTronconChild;
            }
            else if (!tronconChildNodeName.compare("ZONES_DEPASSEMENT_INTERDIT"))
            {
                XMLZonesDpssmtInterdit = pXMLTronconChild;
            }
            else if (!tronconChildNodeName.compare("Z_LEVEL_CROSSINGS"))
            {
                XMLZLevelCrossings = pXMLTronconChild;
            }
			else if (!tronconChildNodeName.compare("VITESSES_REG"))
			{
				XMLVitRegs = pXMLTronconChild;
			}
        }

        // Construction du vecteur des largeurs des voies du troncon
        std::vector<double> largeursVoies;
        for(int iVoie = 0 ; iVoie < nVoie; iVoie++)
        {
            bool bFound = false;
            if(XMLVoiesReservees)
            {
                std::string numVoieStr = SystemUtil::ToString(iVoie+1);
                DOMXPathResult * pXMLNodeList = m_pXMLUtil->SelectNodes("./VOIE_RESERVEE", XMLVoiesReservees->getOwnerDocument(), (DOMElement*)XMLVoiesReservees);
                XMLSize_t countj = pXMLNodeList->getSnapshotLength();
                for(XMLSize_t j=0; j<countj; j++)
                {
                    pXMLNodeList->snapshotItem(j);
                    DOMNode * pNodeVoieReservee = pXMLNodeList->getNodeValue();
                    int num_voie;
                    GetXmlAttributeValue(pNodeVoieReservee, "num_voie", num_voie, &loadingLogger);
                    if(num_voie == (iVoie+1))
                    {
                        double largeurTmp;
                        if(GetXmlAttributeValue(pNodeVoieReservee, "largeur_voie", largeurTmp, &loadingLogger))
                        {
                            largeursVoies.push_back(largeurTmp);
                            bFound = true;
                            break;
                        }
                    }
                }
                pXMLNodeList->release();
            }

            if(!bFound)
            {
                largeursVoies.push_back(dbLargeurVoie);
            }
        }


        Tuyau* T;

        // RÃ©cupÃ©ration des paramÃ¨tres de trafic associÃ©s
        DOMNode * pTraficParams = mapTronconTrafNode[std::string(strID)];

        if( cResolution == 'M' )        // TronÃ§on macroscopique
        {
            // Un tronÃ§on macroscopique ne peut pas Ãªtre reliÃ© Ã  un giratoire
            if( !pCnxAmont || !pCnxAval )
            {
                loadingLogger << Logger::Error << "ERROR: Unknown or incorrect node type for the link " << strID <<std::endl;
                loadingLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                return false;
            }

            TuyauMacro *TMacro = new TuyauMacro
                    (this, strID,cTypeEltAmont,cTypeEltAval, pCnxAval, pCnxAmont, cResolution,
                      strRevetement, largeursVoies,
                      ptExtAmont.dbX, ptExtAmont.dbY, ptExtAval.dbX, ptExtAval.dbY,
                      ptExtAmont.dbZ, ptExtAval.dbZ,
                      0, nVoie,pas_de_temps, dbVitReg, dbVitCat, roadLabel);

            T = TMacro;

            m_LstTuyauxMacro.push_back(TMacro);
            m_LstTuyaux.push_back(TMacro);
            m_mapTuyaux[TMacro->GetLabel()] = TMacro;

            // Le type de vÃ©hicule de base est ajoutÃ© Ã  tous les tronÃ§ons
            T->AddTypeVeh(m_LstTypesVehicule[0]);

            TMacro->GetDiagFonda()->SetProperties( m_LstTypesVehicule.front()->GetW(), m_LstTypesVehicule.front()->GetKx(), m_LstTypesVehicule.front()->GetVx() );
        }
        else
        {
            if(cResolution == 'E')
            {
				m_bMeso = true;

                CTuyauMeso* TMeso = new CTuyauMeso
                        (this, strID,cTypeEltAmont,cTypeEltAval, pCnxAval, pCnxAmont, cResolution,
                          strRevetement, largeursVoies,
                          ptExtAmont.dbX, ptExtAmont.dbY, ptExtAval.dbX, ptExtAval.dbY,
                          ptExtAmont.dbZ, ptExtAval.dbZ,
                          nVoie,pas_de_temps, nCellAcoustique, dbVitReg, dbVitCat, dbCellAcouLength, roadLabel, Tuyau::TT_MESO);

                //Chargement << TMicro << " " << TMicro->GetLabel();

                T = TMeso;

                m_LstTuyauxMeso.push_back(TMeso);
                m_LstTuyaux.push_back(TMeso);
                m_mapTuyaux[TMeso->GetLabel()] = TMeso;

                T->SetID( (int)m_LstTuyauxMeso.size() );

                // Le type de vÃ©hicule de base est ajoutÃ© Ã  tous les tronÃ§ons
                T->AddTypeVeh(m_LstTypesVehicule[0]);

                if(m_bDebugSAS)
                    m_bFichierSAS = true;


                // Le tronÃ§on peut-il gÃ©rer des vÃ©hicules agressifs ?
                bool bTmp;
                if(pTraficParams)
                {
                    GetXmlAttributeValue(pTraficParams, "agressivite", bTmp, &loadingLogger);
                    TMeso->SetAgressif(bTmp);
                }

                GetXmlAttributeValue(pXMLChild, "chgt_voie_droite", bTmp, &loadingLogger);
                TMeso->SetChtVoieVersDroiteAutorise(bTmp);
            }// fi type meso
            else
            {
                TuyauMicro* TMicro = new CTuyauMeso
                        (this, strID,cTypeEltAmont,cTypeEltAval, pCnxAval, pCnxAmont, cResolution,
                          strRevetement, largeursVoies,
                          ptExtAmont.dbX, ptExtAmont.dbY, ptExtAval.dbX, ptExtAval.dbY,
                          ptExtAmont.dbZ, ptExtAval.dbZ,
                          nVoie,pas_de_temps, nCellAcoustique, dbVitReg, dbVitCat, dbCellAcouLength, roadLabel, Tuyau::TT_MICRO);

                //Chargement << TMicro << " " << TMicro->GetLabel();

                T = TMicro;

                m_LstTuyauxMicro.push_back(TMicro);
                m_LstTuyaux.push_back(TMicro);
                m_mapTuyaux[TMicro->GetLabel()] = TMicro;

                T->SetID( (int)m_LstTuyauxMicro.size() );

                // Le type de vÃ©hicule de base est ajoutÃ© Ã  tous les tronÃ§ons
                T->AddTypeVeh(m_LstTypesVehicule[0]);

                if(m_bDebugSAS)
                    m_bFichierSAS = true;


                // Le tronÃ§on peut-il gÃ©rer des vÃ©hicules agressifs ?
                bool bTmp;
                if(pTraficParams)
                {
                    GetXmlAttributeValue(pTraficParams, "agressivite", bTmp, &loadingLogger);
                    TMicro->SetAgressif(bTmp);
                }

                GetXmlAttributeValue(pXMLChild, "chgt_voie_droite", bTmp, &loadingLogger);
                TMicro->SetChtVoieVersDroiteAutorise(bTmp);
            }  // fi type micro
        }
        if(pTraficParams)
        {
            GetXmlAttributeValue(pTraficParams, "penalisation", dbTmp, &loadingLogger);
            T->SetPenalisation(dbTmp);

            int iTmp;
            if (GetXmlAttributeValue(pTraficParams, "func_class", iTmp, &loadingLogger))
            {
                T->SetFunctionalClass(iTmp);
            }
        }
        // MAJ des connexions
        if(pCnxAmont)
        {
            // On vÃ©rifie si l'entrÃ©e est dÃ©finie pour un unique tronÃ§on
            if( T->get_Type_amont() == 'E' && pCnxAmont->GetNbElAval() == 1)
            {
                loadingLogger << Logger::Error << "ERROR : an input can't lead to multiple links ( " << pCnxAmont->GetID() << " )." << std::endl;
                loadingLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                return false;
            }
            pCnxAmont->AddEltAval(T);
        }
        else
        {
            // Brique de giratoire ?
            pBrique = GetBrickFromID(strEltAmontID);
            T->SetBriqueAmont(pBrique);
            pBrique->AddEltAval(T);
        }

        if(pCnxAval)
            pCnxAval->AddEltAmont(T);
        else
        {
            // Brique de giratoire ?
            pBrique = GetBrickFromID(strEltAvalID);
            T->SetBriqueAval(pBrique);
            pBrique->AddEltAmont(T);
        }

        // Distance avant la fin du tronÃ§on pour forcer un changement de voie
        double dbDstChgtVoie;
        if(GetXmlAttributeValue(pXMLChild, "chgtvoie_dstfin", dbDstChgtVoie, &loadingLogger))
        {
            T->SetDstChgtVoie(dbDstChgtVoie);
        }

        // Distance avant la fin du tronÃ§on pour forcer encore plus un changement de voie
        double dbDstChgtVoieForce = m_dbDstChgtVoieForce;
        GetXmlAttributeValue(pXMLChild, "chgtvoie_dstfin_force", dbDstChgtVoieForce, &loadingLogger);
        T->SetDstChgtVoieForce(dbDstChgtVoieForce);

        // Phi Ã  appliquer dans la zone de changement de voie encore plus forcÃ©e
        double dbPhiChgtVoieForce = m_dbPhiChgtVoieForce;
        GetXmlAttributeValue(pXMLChild, "chgtvoie_dstfin_force_phi", dbPhiChgtVoieForce, &loadingLogger);
        T->SetPhiChgtVoieForce(dbPhiChgtVoieForce);

        // Lecture des points internes du tronÃ§on
        Point   pt;
        DOMNode * XMLPt;
        if(XMLPts)
        {
            XMLSize_t countj = XMLPts->getChildNodes()->getLength();
            for(XMLSize_t j=0; j<countj; j++)
            {
                XMLPt = XMLPts->getChildNodes()->item(j);
                if (XMLPt->getNodeType() != DOMNode::ELEMENT_NODE) continue;

                GetXmlAttributeValue(XMLPt, "coordonnees", pt, &loadingLogger);
                T->AddPtInterne(pt.dbX, pt.dbY, pt.dbZ);
            }
        }

        // TronÃ§on curviligne ?
        T->SetCurviligne(dbLongueur != 0);
        // Calcul de la longueur si non saisie et de la longueur de projection
        T->CalculeLongueur(dbLongueur);

        // Lecture des vitesses rÃ©glementaires associÃ©s aux types de vÃ©hicule
        if( cResolution != 'M' ) // Non utilisÃ© dans le cas macroscopique
        {
            if(XMLVitRegs)
            {
                T->m_LstVitReg.SetLag(m_dbLag);
                DOMNode * pVitRegNode = NULL;
                DOMXPathResult * pXMLNodeList = m_pXMLUtil->SelectNodes("./VITESSE_REG", pXMLChild->getOwnerDocument(), (DOMElement*)XMLVitRegs);
                XMLSize_t countj = pXMLNodeList->getSnapshotLength();
                for(XMLSize_t j=0; j<countj; j++)
                {
                    pXMLNodeList->snapshotItem(j);
                    pVitRegNode = pXMLNodeList->getNodeValue();

                    // rÃ©cupÃ©ration de la durÃ©e correspondante
                    vector<PlageTemporelle*> plages;
                    if(!GetXmlDuree(pVitRegNode,this, dbDuree, plages, &loadingLogger))
                    {
                        return false;
                    }

                    // structure contenant la description des vitesses rÃ©glementaire pour la variante temporelle concernÃ©e
                    boost::shared_ptr<VitRegDescription> vitRegDescr = boost::make_shared<VitRegDescription>();

                    // lecture de la dÃ©finition pour chaque type de vÃ©hicule
                    set<TypeVehicule*> alreadyTreatedTypes;
                    DOMNode * pVitRegTypeVehNode = NULL;
                    DOMXPathResult * pXMLNodeListTypeVeh = m_pXMLUtil->SelectNodes("./VITESSE_REG_PAR_TYPE", pVitRegNode->getOwnerDocument(), (DOMElement*)pVitRegNode);
                    XMLSize_t countTypesVeh  = pXMLNodeListTypeVeh->getSnapshotLength();
                    for(XMLSize_t iTV=0; iTV<countTypesVeh; iTV++)
                    {
                        pXMLNodeListTypeVeh->snapshotItem(iTV);
                        pVitRegTypeVehNode = pXMLNodeListTypeVeh->getNodeValue();

                        // rÃ©cupÃ©ration des types de vÃ©hicules concernÃ©s
                        set<TypeVehicule*> typesVeh;
                        string sType;
                        GetXmlAttributeValue(pVitRegTypeVehNode, "types_vehicules", sType, &loadingLogger);
                        if(sType.length() == 0)
                        {
                            // par dÃ©faut, on utilise donc tous les types existants
                            typesVeh.insert(m_LstTypesVehicule.begin(), m_LstTypesVehicule.end());
                        }
                        deque<string> split = SystemUtil::split(sType, ' ');
                        for(size_t splitIdx = 0; splitIdx<split.size(); splitIdx++)
                        {
                            TypeVehicule * pTV = GetVehicleTypeFromID(split[splitIdx]);
                            if(pTV)
                            {
                                if(alreadyTreatedTypes.find(pTV) != alreadyTreatedTypes.end())
                                {
                                    loadingLogger << Logger::Warning << " WARNING : the vehiccle type " << split[splitIdx] << " can't be defined multiple times in the nodes VITESSE_REG_PAR_TYPE of the link " << T->GetLabel() << "." << endl;
                                    loadingLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                                }
                                else
                                {
                                    typesVeh.insert(pTV);
                                    alreadyTreatedTypes.insert(pTV);
                                }
                            }
                            else
                            {
                                loadingLogger << Logger::Warning << " WARNING : the vehicle type " << split[splitIdx] << " defined for a speed limit of link " << T->GetLabel() << " doesn't exist." << endl;
                                loadingLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                            }
                        }

                        // rÃ©cupÃ©ration du paramÃ©trage de chaque voie pour ces types de vÃ©hicule...
                        DOMNode * pVitRegVoieNode = NULL;
                        DOMXPathResult * pXMLNodeListVoie = m_pXMLUtil->SelectNodes("./VITESSE_REG_PAR_VOIE", pVitRegTypeVehNode->getOwnerDocument(), (DOMElement*)pVitRegTypeVehNode);
                        XMLSize_t countVoies = pXMLNodeListVoie->getSnapshotLength();
                        for(XMLSize_t iVoie=0; iVoie<countVoies; iVoie++)
                        {
                            pXMLNodeListVoie->snapshotItem(iVoie);
                            pVitRegVoieNode = pXMLNodeListVoie->getNodeValue();

                            set<int> setVoies;
                            double dbVitesse, dbDebut, dbFin;
                            int nVoie;
                            GetXmlAttributeValue(pVitRegVoieNode, "vitesse", dbVitesse, &loadingLogger);		// valeur de la vitesse
                            GetXmlAttributeValue(pVitRegVoieNode, "position_debut", dbDebut, &loadingLogger);  // position curviligne du dÃ©but de la portion
                            GetXmlAttributeValue(pVitRegVoieNode, "position_fin", dbFin, &loadingLogger);		// position curviligne de la fin de la portion
                            if(!GetXmlAttributeValue(pVitRegVoieNode, "numvoie", nVoie, &loadingLogger))
                            {
                                // dans ce cas, on applique Ã  toute les voies
                                for(int jVoie = 0; jVoie < T->getNb_voies(); jVoie++)
                                {
                                    setVoies.insert(jVoie);
                                }
                            }
                            else
                            {
                                if(nVoie > T->getNb_voies())
                                {
                                    loadingLogger << Logger::Error << " ERROR : the lane number " << nVoie << " is greater than the link's lane number " << T->GetLabel() << std::endl;
                                    loadingLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                                    return false;
                                }
                                else
                                {
                                    setVoies.insert(nVoie-1);
                                }
                            }

                            // ajout de ces paramÃ¨tres Ã  la description pour chaque type de vÃ©hicule
                            set<TypeVehicule*>::iterator iterTV;
                            for(iterTV = typesVeh.begin(); iterTV != typesVeh.end(); iterTV++)
                            {
                                map<int, std::vector<VitRegPortion> > & mapVitRegType = vitRegDescr->vectPortions[*iterTV];
                                set<int>::iterator iterSetVoie;
                                for(iterSetVoie = setVoies.begin(); iterSetVoie != setVoies.end(); iterSetVoie++)
                                {
                                    VitRegPortion portion;
                                    portion.data = dbVitesse;
                                    portion.dbPosDebut = dbDebut;
                                    portion.dbPosFin = dbFin;
                                    mapVitRegType[*iterSetVoie].push_back(portion);
                                }
                            }
                        }
                        pXMLNodeListVoie->release();
                    }
                    pXMLNodeListTypeVeh->release();

                    // Ajout de la variante temporelle
                    if(plages.size() != 0)
                    {
                        for(size_t iPlage = 0; iPlage < plages.size(); iPlage++)
                        {
                            T->m_LstVitReg.AddVariation(plages[iPlage], vitRegDescr);
                        }
                    }
                    else
                    {
                        T->m_LstVitReg.AddVariation(dbDuree, vitRegDescr);
                    }
                }
                pXMLNodeList->release();



                // vÃ©rif de la couverture des plages temporelles
                if(!T->m_LstVitReg.CheckPlagesTemporelles(m_dbDureeSimu))
                {
                    loadingLogger << Logger::Error << "ERROR : the time frames defined for speed limit of link " << T->GetLabel() << " don't cover the whole simulation duration !" << std::endl;
                    loadingLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                    return false;
                }
            }
        }

        // Lecture des terre-pleins et voies rÃ©servÃ©es
        if(XMLVoiesReservees)
        {
            // Lecture des voies rÃ©servÃ©es
            DOMNode * pNodeVoieReservee;
            DOMXPathResult * pXMLNodeList = m_pXMLUtil->SelectNodes("./VOIE_RESERVEE", XMLVoiesReservees->getOwnerDocument(), (DOMElement*)XMLVoiesReservees);
            XMLSize_t countj = pXMLNodeList->getSnapshotLength();
            map<int, double> mapDureesVoiesReservees;
            map<int, vector<PlageTemporelle*> > plagesParVoie;
            for(XMLSize_t j=0; j<countj; j++)
            {
                pXMLNodeList->snapshotItem(j);
                pNodeVoieReservee = pXMLNodeList->getNodeValue();
                int num_voie;

                GetXmlAttributeValue(pNodeVoieReservee, "num_voie", num_voie, &loadingLogger);
                vector<PlageTemporelle*> plages;
                if(!GetXmlDuree(pNodeVoieReservee,this,dbDuree, plages, &loadingLogger))
                {
                    return false;
                }
                plagesParVoie[num_voie].insert(plagesParVoie[num_voie].end(), plages.begin(), plages.end());
                GetXmlAttributeValue(pNodeVoieReservee, "id_typesvehicules", strTmp, &loadingLogger);
                bool bTmp = true;
                GetXmlAttributeValue(pNodeVoieReservee, "active", bTmp, &loadingLogger);
                // construction de tous les types autres que ceux de la liste pour interdiction
                std::deque<std::string> split = SystemUtil::split(strTmp, ' ');
                // Evolution nÂ°20 : ajout d'un warning si un type de vÃ©hicule dÃ©fini ici n'existe pas
                for(size_t splitIdx = 0; splitIdx<split.size(); splitIdx++)
                {
                    if(!GetVehicleTypeFromID(split[splitIdx]))
                    {
                        loadingLogger << Logger::Warning << " WARNING : the vehicule type " << split[splitIdx] << " defined for a reserved lane on link " << T->GetLabel() << " doesn't exist." << endl;
                        loadingLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                    }
                }
                std::vector<TypeVehicule*> pVectTypesInterdits;
                for(size_t typeIdx = 0; typeIdx < m_LstTypesVehicule.size(); typeIdx++)
                {
                    bool isVehiculeAutorise = false;
                    for(size_t splitIdx = 0; splitIdx<split.size(); splitIdx++)
                    {
                        if(GetVehicleTypeFromID(split[splitIdx]) == m_LstTypesVehicule[typeIdx])
                        {
                            isVehiculeAutorise = true;
                        }
                    }
                    if(!isVehiculeAutorise)
                    {
                        pVectTypesInterdits.push_back(m_LstTypesVehicule[typeIdx]);
                    }
                }
                if(m_bTypeProfil)
                {
                    if(mapDureesVoiesReservees.find(num_voie-1) != mapDureesVoiesReservees.end())
                    {
                        mapDureesVoiesReservees[num_voie-1] += dbDuree;
                    }
                    else
                    {
                        mapDureesVoiesReservees[num_voie-1] = dbDuree;
                    }
                    T->AddVoieReserveeByTypeVeh(pVectTypesInterdits, mapDureesVoiesReservees[num_voie-1]-dbDuree, dbDuree, NULL, m_dbDureeSimu, num_voie-1, bTmp);
                    m_LstItiChangeInstants.insert(mapDureesVoiesReservees[num_voie-1]-dbDuree + GetTimeStep());
                }
                else
                {
                    if(plages.size() == 0)
                    {
                        // utilisation de la valeur par dÃ©faut dbDuree
                        T->AddVoieReserveeByTypeVeh(pVectTypesInterdits, 0, dbDuree, NULL, m_dbDureeSimu, num_voie-1, bTmp);
                    }
                    else
                    {
                        // Ajout de la variation pour chacune des plages temporelles dÃ©finies
                        for(size_t iPlage = 0; iPlage < plages.size(); iPlage++)
                        {
                            T->AddVoieReserveeByTypeVeh(pVectTypesInterdits, 0, 0, plages[iPlage], m_dbDureeSimu, num_voie-1, bTmp);
                            m_LstItiChangeInstants.insert(plages[iPlage]->m_Debut + GetTimeStep());
                        }
                    }
                }

            }
            pXMLNodeList->release();

            // vÃ©rif de la couverture des plages temporelles
            map<int, vector<PlageTemporelle*> >::iterator iter;
            for(iter = plagesParVoie.begin(); iter != plagesParVoie.end(); iter++)
            {
                if(iter->second.size() > 0 && !CheckPlagesTemporellesEx(m_dbDureeSimu, iter->second))
                {
                    loadingLogger << Logger::Error << "ERROR : The time frames defined for the reserved lanes of link " << T->GetLabel() << " don't cover the whole simulation duration for lane number " << iter->first << std::endl;
                    loadingLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                    return false;
                }
            }
        }

        // Lecture des voies interdites
        if(XMLVoiesInterdites)
        {
            // Lecture des voies interdites
            DOMNode * pNodeVoieInterdite;
            DOMXPathResult * pXMLNodeList = m_pXMLUtil->SelectNodes("./VOIE_INTERDITE", XMLVoiesInterdites->getOwnerDocument(), (DOMElement*)XMLVoiesInterdites);
            XMLSize_t countj = pXMLNodeList->getSnapshotLength();
            for(XMLSize_t j=0; j<countj; j++)
            {
                pXMLNodeList->snapshotItem(j);
                pNodeVoieInterdite = pXMLNodeList->getNodeValue();
                int num_voie;
                GetXmlAttributeValue(pNodeVoieInterdite, "num_voie", num_voie, &loadingLogger);
                GetXmlAttributeValue(pNodeVoieInterdite, "id_typesvehicules", strTmp, &loadingLogger);
                std::deque<std::string> split = SystemUtil::split(strTmp, ' ');
                std::vector<TypeVehicule*> lstTypesInterdits;
                for(size_t splitIdx = 0; splitIdx<split.size(); splitIdx++)
                {
                    TypeVehicule * pTV = GetVehicleTypeFromID(split[splitIdx]);
                    if(!pTV)
                    {
                        loadingLogger << Logger::Warning << " WARNING : the vehicle type " << split[splitIdx] << " defined for a forbidden lane of link " << T->GetLabel() << " doesn't exist." << endl;
                        loadingLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                    }
                    else
                    {
                        lstTypesInterdits.push_back(pTV);
                    }
                }
                T->AddVoieInterditeByTypeVeh(lstTypesInterdits, num_voie-1);
            }
            pXMLNodeList->release();
        }
        // test de l'attribut "exclusion_types_vehicules" du tronÃ§on (toutes voies interdites)
        if(GetXmlAttributeValue(pXMLChild, "exclusion_types_vehicules", strTmp, &loadingLogger))
        {
            std::deque<std::string> split = SystemUtil::split(strTmp, ' ');
            std::vector<TypeVehicule*> lstTypesInterdits;
            for(size_t splitIdx = 0; splitIdx<split.size(); splitIdx++)
            {
                TypeVehicule * pTV = GetVehicleTypeFromID(split[splitIdx]);
                if(!pTV)
                {
                    loadingLogger << Logger::Warning << " WARNING : the vehicle type " << split[splitIdx] << " defined in the attribute exclusion_types_vehicules of the link " << T->GetLabel() << " doesn't exist." << endl;
                    loadingLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                }
                else
                {
                    lstTypesInterdits.push_back(pTV);
                }
            }
            T->AddVoieInterditeByTypeVeh(lstTypesInterdits);
        }


        // Lecture des terre-pleins
        if(XMLTerrePleins)
        {
            DOMNode * pNodeTerrePlein;
            DOMXPathResult * pXMLNodeList = m_pXMLUtil->SelectNodes("./TERRE_PLEIN", XMLTerrePleins->getOwnerDocument(), (DOMElement*)XMLTerrePleins);
            XMLSize_t countj = pXMLNodeList->getSnapshotLength();
            for(XMLSize_t j=0; j<countj; j++)
            {
                pXMLNodeList->snapshotItem(j);
                pNodeTerrePlein = pXMLNodeList->getNodeValue();
                int num_voie;
                GetXmlAttributeValue(pNodeTerrePlein, "num_voie", num_voie, &loadingLogger);
                GetXmlAttributeValue(pNodeTerrePlein, "position_debut", dbTmp, &loadingLogger);
                GetXmlAttributeValue(pNodeTerrePlein, "position_fin", dbTmp2, &loadingLogger);
                dbTmp2=dbTmp2==numeric_limits<double>::infinity()?T->GetLength():dbTmp2;

                // VÃ©rifications de la position du terre-plein
                if( T->GetLength() < dbTmp)
                {
                    loadingLogger << Logger::Error << " ERROR : wrong median start position ( position_debut > link's length ) : median ignored..." << endl;
                    loadingLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                }
                else if(dbTmp2 <= dbTmp)
                {
                    loadingLogger << Logger::Error << " ERROR : wrong median end position ( position_fin <= position_debut ) : median ignored..." << endl;
                    loadingLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                }
                else
                {
                    if(T->GetLength() < dbTmp2)
                    {
                        loadingLogger << Logger::Warning << " WARNING : median end position greater than the link's length : the median's end is pulled back to the end of the link..." << endl;
                        loadingLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                        dbTmp2 = T->GetLength();
                    }
                }

                // lecture des diffÃ©rentes variantes temporelles
                vector<bool> actifs;
                vector<double> durees;
                DOMXPathResult * pXMLTPVarList = m_pXMLUtil->SelectNodes("./TERRE_PLEIN_VARIATION", pNodeTerrePlein->getOwnerDocument(), (DOMElement*)pNodeTerrePlein);
                XMLSize_t countk = pXMLTPVarList->getSnapshotLength();
                vector<PlageTemporelle*> tmpPlagesVect;
                vector<bool> tmpPlagesActivesVect;
                for(XMLSize_t k=0; k<countk; k++)
                {
                    pXMLTPVarList->snapshotItem(k);
                    DOMNode * pNodeTerrePleinVar = pXMLTPVarList->getNodeValue();

                    bool bActif = true;
                    GetXmlAttributeValue(pNodeTerrePleinVar, "actif", bActif, &loadingLogger);
                    vector<PlageTemporelle*> plages;
                    if(!GetXmlDuree(pNodeTerrePleinVar,this, dbDuree, plages, &loadingLogger))
                    {
                        return false;
                    }
                    if(plages.size() == 0)
                    {
                        durees.push_back(dbDuree);
                        actifs.push_back(bActif);
                    }
                    else
                    {
                        tmpPlagesVect.insert(tmpPlagesVect.end(), plages.begin(), plages.end());
                        tmpPlagesActivesVect.resize(tmpPlagesActivesVect.size() + plages.size(), bActif);
                    }
                }
                pXMLTPVarList->release();

                // vÃ©rification de la couverture des plages temporelles
                if(tmpPlagesVect.size() > 0)
                {
                    if(!CheckPlagesTemporellesEx(m_dbDureeSimu, tmpPlagesVect))
                    {
                        loadingLogger << Logger::Error << "ERROR : The time frames defined for a median on link " << T->GetLabel() << " don't cover the whole simulation duration!" << std::endl;
                        loadingLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                        return false;
                    }
                }

                // Evolution nÂ°92 : terre-plein actif pour toute la durÃ©e de la simulation si pas de variation dÃ©finie
                if(actifs.size() == 0 && tmpPlagesVect.size() == 0)
                {
                    durees.push_back(m_dbDureeSimu);
                    actifs.push_back(true);
                }

                T->AddTerrePlein(num_voie-1, dbTmp, dbTmp2, durees, actifs, tmpPlagesVect, tmpPlagesActivesVect);
            }
            pXMLNodeList->release();
        }

        // Lecture du troncon opposÃ© le cas Ã©chÃ©ant
        if(GetXmlAttributeValue(pXMLChild, "id_troncon_oppose", strTmp, &loadingLogger))
        {
            tronconsAvecOppose.push_back(T);
            tronconsOpposes.push_back(strTmp);
        }

        // Lecture des zones de dÃ©passement interdites
        if(XMLZonesDpssmtInterdit)
        {
            DOMNode * pNodeZoneDepassementInterdit;
            DOMXPathResult * pXMLNodeList = m_pXMLUtil->SelectNodes("./ZONE_DEPASSEMENT_INTERDIT", XMLZonesDpssmtInterdit->getOwnerDocument(), (DOMElement*)XMLZonesDpssmtInterdit);
            XMLSize_t countj = pXMLNodeList->getSnapshotLength();
            for(XMLSize_t j=0; j<countj; j++)
            {
                pXMLNodeList->snapshotItem(j);
                pNodeZoneDepassementInterdit = pXMLNodeList->getNodeValue();

                // dÃ©but de la zone
                GetXmlAttributeValue(pNodeZoneDepassementInterdit, "position_debut", dbTmp, &loadingLogger);

                // fin de la zone
                GetXmlAttributeValue(pNodeZoneDepassementInterdit, "position_fin", dbTmp2, &loadingLogger);
                dbTmp2=dbTmp2==numeric_limits<double>::infinity()?T->GetLength():dbTmp2;

                // VÃ©rifications de la position du terre-plein
                if( T->GetLength() < dbTmp)
                {
                    loadingLogger << Logger::Warning << " WARNING : wrong start position of the overtaking forbidding zone ( position_debut > link's length )" << endl;
                    loadingLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                }
                else if(dbTmp2 <= dbTmp)
                {
                    loadingLogger << Logger::Warning << " WARNING : wrong end position of the overtaking forbidding zone ( position_fin <= position_debut )" << endl;
                    loadingLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                }
                else
                {
                    if(T->GetLength() < dbTmp2)
                    {
                        loadingLogger << Logger::Warning << " WARNING : end position of the overtaking forbidding zone greater than the link's length : the end of the zone is pulled back to the link's length..." << endl;
                        loadingLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                        dbTmp2 = T->GetLength();
                    }
                }
                T->AddZoneDepassementInterdit(dbTmp, dbTmp2);
            }
            pXMLNodeList->release();
        }


        // Lecture de la dÃ©finition des zLevel
        if(XMLZLevelCrossings)
        {
            DOMNode * pNodeZLevelCrossing;
            DOMXPathResult * pXMLNodeList = m_pXMLUtil->SelectNodes("./Z_LEVEL_CROSSING", XMLZLevelCrossings->getOwnerDocument(), (DOMElement*)XMLZLevelCrossings);
            XMLSize_t countj = pXMLNodeList->getSnapshotLength();
            for(XMLSize_t j=0; j<countj; j++)
            {
                pXMLNodeList->snapshotItem(j);
                pNodeZLevelCrossing = pXMLNodeList->getNodeValue();

                GetXmlAttributeValue(pNodeZLevelCrossing, "start", dbTmp, &loadingLogger);
                GetXmlAttributeValue(pNodeZLevelCrossing, "end", dbTmp2, &loadingLogger);
                dbTmp2=dbTmp2==numeric_limits<double>::infinity()?T->GetLength():dbTmp2;
                GetXmlAttributeValue(pNodeZLevelCrossing, "zlevel", nTmp, &loadingLogger);

                // VÃ©rifications
                if( T->GetLength() < dbTmp)
                {
                    loadingLogger << Logger::Warning << " WARNING : wrong Z_LEVEL_CROSSING start position ( start > link's length )" << endl;
                    loadingLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                }
                else if(dbTmp2 <= dbTmp)
                {
                    loadingLogger << Logger::Warning << " WARNING : wrong Z_LEVEL_CROSSING end position ( start <= end )" << endl;
                    loadingLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                }
                else
                {
                    if(T->GetLength() < dbTmp2)
                    {
                        loadingLogger << Logger::Warning << " WARNING : Z_LEVEL_CROSSING end position greater than the link's length : the end is pulled back to the end of the link..." << endl;
                        loadingLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                        dbTmp2 = T->GetLength();
                    }
                }
                if(T->AddZoneZLevel(dbTmp, dbTmp2, nTmp))
                {
                    loadingLogger << Logger::Warning << " WARNING : multiple Z_LEVEL_CROSSING overlapping in the link" << T->GetLabel() << endl;
                    loadingLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                }
            }
            pXMLNodeList->release();
        }
    }

    // Association des tronÃ§ons opposÃ©s
    assert(tronconsAvecOppose.size() == tronconsOpposes.size());
    size_t nbTronconsOpposes = tronconsAvecOppose.size();
    for(size_t iTOppose = 0; iTOppose < nbTronconsOpposes; iTOppose++)
    {
        // rÃ©cupÃ©ration du troncon opposÃ©
        Tuyau * pTuyOppose = GetLinkFromLabel(tronconsOpposes[iTOppose]);
        if(!tronconsAvecOppose[iTOppose]->SetTuyauOppose(pTuyOppose, &loadingLogger))
        {
            return false;
        }
    }

    // ************************************************************************
    // Cas particulier des tronÃ§ons Ã  ajouter automatiquement pour l'insertion
    // ************************************************************************
    // l'ensemble des troncons utilisateur a Ã©tÃ© crÃ©Ã©. Il reste Ã  crÃ©er les Ã©ventuels tronÃ§ons d'insertion
    // associÃ©s aux convergents de type "insertion" ainsi que le rÃ©partiteur en amont de cse nouveaux tronÃ§ons
    for(size_t convInsIdx = 0; convInsIdx < Convergents_Insertion.size(); convInsIdx++)
    {
        // rÃ©cupÃ©ration de l'identifiant du convergent d'insertion
        std::string insertionID;
        GetXmlAttributeValue(Convergents_Insertion[convInsIdx], "id", insertionID, &loadingLogger);

        // RÃ©cupÃ©ration du rÃ©partiteur en amont du tronÃ§on d'insertion
        std::string strRepAmontID = "R_" + insertionID + "_INSERTION_AMONT";
        Repartiteur *pRepartiteurInsAmont = NULL;
        for(size_t repInsIdx = 0; repInsIdx < repartiteursInsertion.size(); repInsIdx++)
        {
            if(!repartiteursInsertion[repInsIdx]->GetID().compare(strRepAmontID))
            {
                pRepartiteurInsAmont = repartiteursInsertion[repInsIdx];
                break;
            }
        }
        assert(pRepartiteurInsAmont!=NULL);


        // RÃ©cupÃ©ration du rÃ©partiteur aval de l'insertion crÃ©Ã© plus haut :
        std::string strRepAvalID = "R_" + insertionID + "_INSERTION_AVAL";
        pRepartiteur = NULL;
        for(size_t repInsIdx = 0; repInsIdx < repartiteursInsertion.size(); repInsIdx++)
        {
            if(!repartiteursInsertion[repInsIdx]->GetID().compare(strRepAvalID))
            {
                pRepartiteur = repartiteursInsertion[repInsIdx];
                break;
            }
        }
        assert(pRepartiteur!=NULL);

        // CrÃ©ation du tronÃ§on d'insertion :

        // dÃ©termination des coordonnÃ©es du tronÃ§on Ã  crÃ©er :
        // rÃ©cupÃ©ration du tronÃ§on prioritaire pour connaÃ®tre les coordonnÃ©es amont du tuyau Ã  crÃ©er
        Tuyau * pTuyauAmontPrio = NULL;
        std::string tronconPrioID = GetTronconAmontPrio(Convergents_Insertion[convInsIdx], loadingLogger);
        pTuyauAmontPrio = GetLinkFromLabel(tronconPrioID);

        int nbVoiesInsDroite, nbVoiesInsGauche;
        GetXmlAttributeValue(Convergents_Insertion[convInsIdx], "nb_voies_insertion_droite", nbVoiesInsDroite, &loadingLogger);
        GetXmlAttributeValue(Convergents_Insertion[convInsIdx], "nb_voies_insertion_gauche", nbVoiesInsGauche, &loadingLogger);
        strcpy(strID, ("T_" + insertionID + "_INSERTION").c_str());

        // rÃ©cupÃ©ration du tuyau aval principal
        Tuyau * pTuyauAval = NULL;
        if(pRepartiteur->GetNbElAval() == 1)
        {
            // Cas convergent d'insertion classique, avec un unique tuyau aval
            pTuyauAval = pRepartiteur->m_LstTuyAv[0];
        }
        else
        {
            // EVOLUTION nÂ°70 - on utilise le tuyau aval "principal", dÃ©terminÃ© par l'attribut id_TAv
            std::string strIdTAv;
            GetXmlAttributeValue(Convergents_Insertion[convInsIdx], "id_TAv", strIdTAv, &loadingLogger);
            pTuyauAval = GetLinkFromLabel(strIdTAv);
        }

        int nbVoiesTuyauInsertion = nbVoiesInsDroite + nbVoiesInsGauche + pTuyauAval->getNb_voies();

        // nombre de cellules acoustiques
        int nbCellules;
        DOMNode * pSimuParams = m_pXMLUtil->SelectSingleNode("./ELEMENTS/ELEMENT[@id=\"" + insertionID + "\"]", Convergents_Insertion[convInsIdx]->getOwnerDocument(), (DOMElement*)pXMLNodeSimulation);
        if(!GetXmlAttributeValue(pSimuParams, "nb_cell_acoustique", nbCellules, &loadingLogger)) nbCellules = nDefaultNbCellAcou; // nCellAcoustique
        if(!GetXmlAttributeValue(pSimuParams, "longueur_cell_acoustique", dbCellAcouLength, &loadingLogger)) dbCellAcouLength = dbDefaultCellAcouLength;

        // crÃ©ation du tuyau compris entre le convergent et le rÃ©partiteur
        Tuyau * T;
        strcpy(strRevetement, pTuyauAval->GetRevetement().c_str());

        // on ajuste les coordonnÃ©es amont du tuyau aval en prenant en compte les nombre et la position des voies d'insertion :
        Point * ptAval2 = pTuyauAval->GetLstPtsInternes().size() == 0 ? pTuyauAval->GetExtAval() : pTuyauAval->GetLstPtsInternes()[0];
        Point ptAvalInsertion = CalculPositionTronconInsertion(*pTuyauAval->GetExtAmont(), *ptAval2, pTuyauAval->getLargeursVoies(), nbVoiesTuyauInsertion, nbVoiesInsDroite);

        std::vector<double> largeursVoies;
        for(int iVoie = 0; iVoie < nbVoiesTuyauInsertion; iVoie++)
        {
            if(iVoie < nbVoiesInsDroite)
            {
                largeursVoies.push_back(pTuyauAval->getLargeurVoie(0));
            }
            else if(iVoie >= nbVoiesInsDroite + pTuyauAval->getNb_voies())
            {
                largeursVoies.push_back(pTuyauAval->getLargeurVoie(pTuyauAval->getNb_voies()-1));
            }
            else
            {
                largeursVoies.push_back(pTuyauAval->getLargeurVoie(iVoie - nbVoiesInsDroite));
            }
        }
        if( pTuyauAval->IsMacro()) // TronÃ§on macroscopique
        {
            T = new TuyauMacro
                    (this, strID,'C','R', pRepartiteur, pRepartiteurInsAmont, 'M',
                    strRevetement, largeursVoies,
                    pTuyauAmontPrio->GetExtAval()->dbX, pTuyauAmontPrio->GetExtAval()->dbY,
                    ptAvalInsertion.dbX, ptAvalInsertion.dbY,
                    pTuyauAmontPrio->GetExtAval()->dbZ, pTuyauAval->GetExtAmont()->dbZ,
                    nbCellules, nbVoiesTuyauInsertion,pas_de_temps, pTuyauAval->GetVitesseMax(), DBL_MAX, pTuyauAval->GetRoadLabel());

            m_LstTuyauxMacro.push_back((TuyauMacro*)T);
            m_LstTuyaux.push_back(T);
            m_mapTuyaux[T->GetLabel()] = T;

            // Le type de vÃ©hicule de base est ajoutÃ© Ã  tous les tronÃ§ons
            T->AddTypeVeh(m_LstTypesVehicule[0]);

            ((TuyauMacro*)T)->GetDiagFonda()->SetProperties( m_LstTypesVehicule.front()->GetW(), m_LstTypesVehicule.front()->GetKx(), m_LstTypesVehicule.front()->GetVx() );
        }
        else
        {
            T = new CTuyauMeso
                    (this, strID,'C','R', pRepartiteur,pRepartiteurInsAmont, 'H',
                        strRevetement, largeursVoies,
                        pTuyauAmontPrio->GetExtAval()->dbX, pTuyauAmontPrio->GetExtAval()->dbY,
                        ptAvalInsertion.dbX, ptAvalInsertion.dbY,
                        pTuyauAmontPrio->GetExtAval()->dbZ, pTuyauAval->GetExtAmont()->dbZ,
                        nbVoiesTuyauInsertion,pas_de_temps, nbCellules, pTuyauAval->GetVitesseMax(), DBL_MAX, dbCellAcouLength, pTuyauAval->GetRoadLabel(), Tuyau::TT_MICRO);

            m_LstTuyauxMicro.push_back((TuyauMicro*)T);
            m_LstTuyaux.push_back(T);
            m_mapTuyaux[T->GetLabel()] = T;

            T->SetID( (int)m_LstTuyauxMicro.size() );

            // Le type de vÃ©hicule de base est ajoutÃ© Ã  tous les tronÃ§ons
            T->AddTypeVeh(m_LstTypesVehicule[0]);

            // Le tronÃ§on peut-il gÃ©rer des vÃ©hicules agressifs ?
            ((TuyauMicro*)T)->SetAgressif(((TuyauMicro*)pTuyauAval)->IsAgressif());
            // le troncon peut-il gÃ©rer les changements de voie
            ((TuyauMicro*)T)->SetChtVoieVersDroiteAutorise(((TuyauMicro*)pTuyauAval)->IsChtVoieVersDroiteAutorise());
        }

        // MAJ des connexions
        pRepartiteur->AddEltAmont(T);
        pRepartiteurInsAmont->AddEltAval(T);

        // TronÃ§on curviligne ?
        T->SetCurviligne(false);
        // Calcul de la longueur si non saisie et de la longueur de projection
        T->CalculeLongueur(0);
    }

    // **********************************************************************
    // Fin cas particulier des tronÃ§on en aval des convergents d'insertion.
    // **********************************************************************

    // Test de la longueur minimum des tuyaux pour le mode mÃ©so
    bool bMesoLengthOK = true;
    for(size_t iTuy = 0; iTuy < m_LstTuyaux.size(); iTuy++)
    {
        Tuyau * pLink =  m_LstTuyaux[iTuy];
        // pour chaque vÃ©hicule pouvant emprunter le tronÃ§on en mode mÃ©so :
        for(size_t iTV = 0; iTV < m_LstTypesVehicule.size(); iTV++)
        {
            // le vÃ©hicule est suivi en mode meso, on vÃ©rifie aussi qu'il peut Ãªtre amenÃ© Ã  circuler sur le tronÃ§on (pas interdit)
            if(!pLink->IsInterdit(m_LstTypesVehicule[iTV]))
            {
                // Le tronÃ§on est soit meso statiquement, soit potentiellement meso dynamiquement pour le type de vÃ©hicule considÃ©rÃ©
                if(pLink->GetType() == Tuyau::TT_MESO || (!m_MicroVehicleTypes.empty() && std::find(m_MicroVehicleTypes.begin(), m_MicroVehicleTypes.end(), m_LstTypesVehicule[iTV]) == m_MicroVehicleTypes.end()))
                {
                    if(pLink->GetLength() < 1.0 / m_LstTypesVehicule[iTV]->GetKx())
                    {
                        loadingLogger << Logger::Error << "ERROR : The length of the potentially mesoscopic link " << pLink->GetLabel() << " (" << pLink->GetLength() << "m) is lower than 1 / Kx (" << 1.0 / m_LstTypesVehicule[iTV]->GetKx() << "m) for the vehicle type " << m_LstTypesVehicule[iTV]->GetLabel() << std::endl;
                        loadingLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                        bMesoLengthOK = false;
                    }
                }
            }
        }
    }
    if(!bMesoLengthOK)
    {
        return false;
    }

    // Section RESEAU / ROUTES
    DOMNode * pRoutesNode;
    pRoutesNode = m_pXMLUtil->SelectSingleNode( "./ROUTES", pXMLReseau->getOwnerDocument(), (DOMElement*)pXMLReseau);
    if( pRoutesNode )
    {
        // parcours toutes les routes si le noeud existe
        XMLSize_t counti = pRoutesNode->getChildNodes()->getLength();
        for( XMLSize_t i =0; i < counti; ++i)
        {
            string strRouteId;

            DOMNode * pRouteNode;
            pRouteNode = pRoutesNode->getChildNodes()->item(i);
            if (pRouteNode->getNodeType() != DOMNode::ELEMENT_NODE) continue;

            GetXmlAttributeValue( pRouteNode, "id", strRouteId, &loadingLogger);

            // Obtient les troncons d'itineraires
            DOMNode * pXMLTroncons =  m_pXMLUtil->SelectSingleNode( "./TRONCONS_", pRouteNode->getOwnerDocument(), (DOMElement*)pRouteNode);
            std::vector<Tuyau *> routeLinks;
            if( pXMLTroncons )
            {
                XMLSize_t countj = pXMLTroncons->getChildNodes()->getLength();
                std::string strTronconId;
                for(XMLSize_t j=0; j<countj;j++)
                {
                    DOMNode * pTronconNode = pXMLTroncons->getChildNodes()->item(j);
                    GetXmlAttributeValue( pTronconNode, "id", strTronconId, &loadingLogger);
                    Tuyau *pLink = this->GetLinkFromLabel(strTronconId);
                    if( pLink )
                    {
                        routeLinks.push_back(pLink);
                    }
                } //rof each troncons
            }

            Connexion * pJunction = NULL;
            DOMNode * pXMLNoeud = m_pXMLUtil->SelectSingleNode("NOEUD", pRouteNode->getOwnerDocument(), (DOMElement*)pRouteNode);
            if (pXMLNoeud)
            {
                GetXmlAttributeValue(pXMLNoeud, "id", strTmp, &loadingLogger);
                pJunction = GetConnectionFromID(strTmp, cTmp);
                if (!pJunction)
                {
                    pJunction = GetBrickFromID(strTmp);
                }
            }

            // ajoute la route en mÃ©moire
            if (!strRouteId.empty())
            {
                m_routes[strRouteId] = std::make_pair(routeLinks, pJunction);
            }

        } // rof each routes

    }
    // fin section RESEAU / ROUTES

    // Retour sur les rÃ©partiteurs

    // SECTION RESEAU / CONNEXIONS / REPARTITEURS
    pXMLNode = m_pXMLUtil->SelectSingleNode("./CONNEXIONS/REPARTITEURS", pXMLReseau->getOwnerDocument(), (DOMElement*)pXMLReseau);

    if(pXMLNode)
    {
        XMLSize_t counti = pXMLNode->getChildNodes()->getLength();
        int ii = 0;
        for(XMLSize_t i=0; i<counti;i++)
        {
            pXMLChild = pXMLNode->getChildNodes()->item(i);
            if (pXMLChild->getNodeType() != DOMNode::ELEMENT_NODE) continue;

            GetXmlAttributeValue(pXMLChild, "id", strTmp, &loadingLogger);

            pRepartiteur = Liste_repartiteurs[strTmp];

            if( !(pRepartiteur->GetNbElAmont() > 0 && pRepartiteur->GetNbElAval() > 0) )
            {
                ii++;
                continue;      // rÃ©partiteur non utilisÃ©
            }

            // Calcul du nb max de voie pour le rÃ©partiteur
            int nNbmaxVoies = 1;
            for(int j=0; j < pRepartiteur->getNbAmont(); j++)
            {
                if( pRepartiteur->m_LstTuyAm[j]->GetLanesManagement() == 'D' && pRepartiteur->m_LstTuyAm[j]->getNb_voies() > nNbmaxVoies)
                    nNbmaxVoies = pRepartiteur->m_LstTuyAm[j]->getNb_voies();
            }
            for(int j=0; j < pRepartiteur->GetNbElAval(); j++)
            {
                if( pRepartiteur->m_LstTuyAv[j]->GetLanesManagement() == 'D' &&  pRepartiteur->m_LstTuyAv[j]->getNb_voies() > nNbmaxVoies)
                    nNbmaxVoies = pRepartiteur->m_LstTuyAv[j]->getNb_voies();
            }
            pRepartiteur->SetNbMaxVoies(nNbmaxVoies);

            pRepartiteur->calculBary();
            pRepartiteur->calculRayon();
            pRepartiteur->Constructeur_complet(pRepartiteur->GetNbElAmont(),pRepartiteur->GetNbElAval(),false);

            if( !pRepartiteur->Init(pXMLChild, &loadingLogger))
                return false;

            ii++;
        }

    } // Fin boucle sur les noeuds des rÃ©partiteurs


    // *******************************************************************************
    // Cas particulier des rÃ©partiteurs crÃ©Ã©s pour les convergent de type "insertion"
    // *******************************************************************************
    for(size_t repInsIdx = 0; repInsIdx < repartiteursInsertion.size(); repInsIdx++)
    {
        Repartiteur *pRepartiteur = repartiteursInsertion[repInsIdx];

        // Calcul du nb max de voie pour le rÃ©partiteur
        int nNbmaxVoies = 1;
        for(int j=0; j < pRepartiteur->getNbAmont(); j++)
        {
            if( pRepartiteur->m_LstTuyAm[j]->GetLanesManagement() == 'D' && pRepartiteur->m_LstTuyAm[j]->getNb_voies() > nNbmaxVoies)
                nNbmaxVoies = pRepartiteur->m_LstTuyAm[j]->getNb_voies();
        }
        for(int j=0; j < pRepartiteur->GetNbElAval(); j++)
        {
            if( pRepartiteur->m_LstTuyAv[j]->GetLanesManagement() == 'D' &&  pRepartiteur->m_LstTuyAv[j]->getNb_voies() > nNbmaxVoies)
                nNbmaxVoies = pRepartiteur->m_LstTuyAv[j]->getNb_voies();
        }
        pRepartiteur->SetNbMaxVoies(nNbmaxVoies);

        pRepartiteur->calculBary();
        pRepartiteur->calculRayon();
        pRepartiteur->Constructeur_complet(pRepartiteur->GetNbElAmont(),pRepartiteur->GetNbElAval(),false);


        // rÃ©partition : equi-repartie par dÃ©faut... ajouter la possibilitÃ© de dÃ©finir une rÃ©partition au niveau du convergent d'insertion ?
        if( !IsUsedODMatrix() )
        {
            for (size_t iTypeVeh = 0; iTypeVeh < m_LstTypesVehicule.size(); iTypeVeh++)
            {
                TypeVehicule * pTypeVeh = m_LstTypesVehicule.at(iTypeVeh);
                Tuyau *pTAmont;
                Tuyau *pTAval;
                int nNbVoiesAval;  // nombre de voie en aval.
                int nNbAmont = pRepartiteur->GetNbElAmont();
                int nNbAval = pRepartiteur->GetNbElAval();

                double** pLstCoeff = new double*[nNbAmont*nNbmaxVoies];
                int** pNbVeh = new int*[nNbAmont*nNbmaxVoies];

                for (int j = 0; j < nNbAmont; j++)
                {
                    for (int nVoieAmont = 0; nVoieAmont < nNbmaxVoies; nVoieAmont++)
                    {
                        pLstCoeff[j*nNbmaxVoies + nVoieAmont] = new double[nNbAval*nNbmaxVoies];
                        pNbVeh[j*nNbmaxVoies + nVoieAmont] = new int[nNbAval*nNbmaxVoies];

                        // Initialisation
                        for (int k = 0; k < nNbAval; k++)
                        for (int nVoieAval = 0; nVoieAval < nNbmaxVoies; nVoieAval++)
                            pNbVeh[j*nNbmaxVoies + nVoieAmont][k*nNbmaxVoies + nVoieAval] = 0;
                    }
                }

                for (int j = 0; j < pRepartiteur->GetNbElAmont(); j++)
                {
                    pTAmont = pRepartiteur->m_LstTuyAm[j];

                    for (int nVAm = 0; nVAm < pTAmont->getNbVoiesDis(); nVAm++)
                    {
                        nNbVoiesAval = 0;
                        for (int k = 0; k < pRepartiteur->GetNbElAval(); k++)
                        {
                            pTAval = pRepartiteur->m_LstTuyAv[k];
                            nNbVoiesAval += pTAval->getNbVoiesDis();
                        }

                        for (int k = 0; k < pRepartiteur->GetNbElAval(); k++)
                        {
                            pTAval = pRepartiteur->m_LstTuyAv[k];

                            for (int nVAv = 0; nVAv < pTAval->getNbVoiesDis(); nVAv++)
                            {
                                pLstCoeff[j*nNbmaxVoies + nVAm][k*nNbmaxVoies + nVAv] = 1.0 / nNbVoiesAval;
                            }
                        }
                    }
                }
                boost::shared_ptr<RepartitionFlux> pRF(new RepartitionFlux);
                pRF->nNbVeh = pNbVeh;
                pRF->pCoefficients = pLstCoeff;
                pRF->nNbVoiesAmont = nNbAmont*nNbmaxVoies;
                pRF->nNbVoiesAval = nNbAval*nNbmaxVoies;
                std::deque<TimeVariation<RepartitionFlux> >* & pLstRepartition = pRepartiteur->GetLstRepartition()[pTypeVeh];
                pLstRepartition = new std::deque<TimeVariation<RepartitionFlux> >();
                AddVariation(m_dbDureeSimu, pRF, pLstRepartition);
            }
        }

        // Ajout du rÃ©partiteur Ã  la liste
        Liste_repartiteurs[pRepartiteur->GetID()] = pRepartiteur;
        this->m_LstUserCnxs.push_back(pRepartiteur);
        mapUserConnections[pRepartiteur->GetID()] = NULL;
    }
    // *******************************************************************************
    // Fin cas particulier des rÃ©partiteurs crÃ©Ã©s pour convergent de type "insertion"
    // *******************************************************************************

   // DerniÃ¨res initialisations...
   std::deque <Tuyau*>::iterator curTuy;
   std::deque <Tuyau*>::iterator debTuy = m_LstTuyaux.begin();
   std::deque <Tuyau*>::iterator finTuy = m_LstTuyaux.end();

   TypeVehicule *pTV;
   double fDenominateur;
   int Nb_cel;
   double dbPasEspace;

   for (curTuy=debTuy;curTuy!=finTuy;curTuy++)
   {
        // Si tronÃ§on macroscopique, on peut calculer le nombre de segment
        if( (*curTuy)->IsMacro() )
        {
            TuyauMacro* TMacro = (TuyauMacro*)(*curTuy);
            pTV =  (*curTuy)->GetLstTypesVeh()[0];
            //Le programme calcule lui-mÃªme le nombre de segments
            fDenominateur =  pTV->GetVx()*pas_de_temps;
            {
                // DÃ©termination du nombre entier de cellules le plus proche de CFL
                Nb_cel=(int)floor((sqrtl(
                    ( (*curTuy)->GetAbsAmont() - (*curTuy)->GetAbsAval() )* ( (*curTuy)->GetAbsAmont() - (*curTuy)->GetAbsAval() ) +
                    ( (*curTuy)->GetOrdAmont() - (*curTuy)->GetOrdAval() )* ( (*curTuy)->GetOrdAmont() - (*curTuy)->GetOrdAval() ) +
                    ( (*curTuy)->GetHautAmont() - (*curTuy)->GetHautAval() )* ( (*curTuy)->GetHautAmont() - (*curTuy)->GetHautAval() )
                    ) )/(fDenominateur) );
            }

            if(Nb_cel>0)
                dbPasEspace=(sqrtl(
                    ( (*curTuy)->GetAbsAmont() - (*curTuy)->GetAbsAval())*( (*curTuy)->GetAbsAmont() - (*curTuy)->GetAbsAval()) +
                    ( (*curTuy)->GetOrdAmont() - (*curTuy)->GetOrdAval())*( (*curTuy)->GetOrdAmont() - (*curTuy)->GetOrdAval()) +
                    ( (*curTuy)->GetHautAmont() - (*curTuy)->GetHautAval())*( (*curTuy)->GetHautAmont() - (*curTuy)->GetHautAval())
                    ) ) /Nb_cel;

            (*curTuy)->SetPasEspace(dbPasEspace);
            (*curTuy)->SetNbCell(Nb_cel);

            // VÃ©rification de la condition sur la vitesse libre pour les tuyaux macroscopiques
            if(  fabs( (dbPasEspace / pas_de_temps) - pTV->GetVx() ) < ZERO_DOUBLE )
            {
                TMacro->SetCondVitAmont(TRUE);
            }
            else
            {
                if(  (dbPasEspace / pas_de_temps) <  pTV->GetVx() )
                {
                    loadingLogger << Logger::Error << "ERROR : The maximum speed value for link ";
                    loadingLogger << Logger::Error << TMacro->GetLabel();
                    loadingLogger << Logger::Error << " is lower than the space step over time step ratio"<<std::endl;
                    loadingLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                    return false;
                }
                else
                {
                    TMacro->SetCondVitAmont(FALSE);

                    // Calcul des coefficients de pondÃ©ration du calcul de N
                    TMacro->SetCoeffAmont(dbPasEspace, pas_de_temps, pTV->GetVx() );
                }
            }
        }

        //System::String ^ssT2 = gcnew System::String( (*curTuy)->GetLabel().c_str() );
        //std::string ssT2 = (*curTuy)->GetLabel();

        // La segmentation peut maintenant s'effectuer
        (*curTuy)->Segmentation();

        if( !(*curTuy)->IsMacro() )
        {
            DOMNode *pXMLVoiesReduites;
            DOMNode *pXMLTroncon;
            int nVoie;
            TuyauMicro* pCurTuyau = (TuyauMicro*)(*curTuy);

            pXMLTroncon = mapTronconReseauNode[pCurTuyau->GetLabel()];

            // Chargement des voies rÃ©duites
            if(pXMLTroncon)
            {
                // Chargement des voies rÃ©duites
                pXMLVoiesReduites = m_pXMLUtil->SelectSingleNode("./VOIES_REDUITES", pXMLTroncon->getOwnerDocument(), (DOMElement*)pXMLTroncon);

                if(pXMLVoiesReduites)
                {
                    XMLSize_t countj = pXMLVoiesReduites->getChildNodes()->getLength();
                    for(XMLSize_t j=0; j<countj; j++)
                    {
                        DOMNode* xmlChildj = pXMLVoiesReduites->getChildNodes()->item(j);
                        if (xmlChildj->getNodeType() != DOMNode::ELEMENT_NODE) continue;

                        GetXmlAttributeValue(xmlChildj, "numvoie", nVoie, &loadingLogger);

                        // VÃ©rification de la cohÃ©rence du numÃ©ro de voie rÃ©duite
                        if( nVoie > (*curTuy)->getNbVoiesDis() )
                        {
                            loadingLogger << Logger::Error << "ERROR : the lane number of the reduced lane is wrong ( link " << (*curTuy)->GetLabel() << ")";
                            loadingLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                            return false;
                        }

                        // rÃ©cupÃ©ration des exceptions de types de vÃ©hicules
                        std::vector<TypeVehicule*> lstExceptionTypesVeh;
                        if(GetXmlAttributeValue(xmlChildj, "exclusion_types_vehicules", strTmp, &loadingLogger))
                        {
                            std::deque<std::string> split = SystemUtil::split(strTmp, ' ');
                            for(size_t j=0; j< split.size(); j++)
                            {
                                TypeVehicule * pTV = GetVehicleTypeFromID(split.at(j));
                                if(pTV)
                                    lstExceptionTypesVeh.push_back(pTV);
                                else
                                {
                                    loadingLogger << Logger::Error << "ERROR : Wrong list of excluded vehicle types for the reduced lane definition on link ";
                                    loadingLogger << Logger::Error << (*curTuy)->GetLabel() << std::endl;
                                    loadingLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                                    return false;
                                }
                            }
                        }

                        ((VoieMicro*)(*curTuy)->GetLstLanes()[nVoie-1])->SetChgtVoieObligatoire(lstExceptionTypesVeh);

                        // Construction des merging objects dans le cas d'un gestion asynchrone des changements de voie 'mandatory'
                        if( this->m_cChgtVoieMandatoryMode == 'A' )
                        {
                            Tuyau *pTAv = pCurTuyau->getConnectionAval()->m_LstTuyAv[0];
                            Tuyau *pTAmP = pCurTuyau->getConnectionAmont()->m_LstTuyAm[0];
                            Tuyau *pTAmNP = pCurTuyau->getConnectionAmont()->m_LstTuyAm[1];

                            MergingObject *pMO = new MergingObject(this, pCurTuyau, nVoie-1, pTAv, pTAmP, pTAmNP, m_dbFAlphaMandatory);
                            m_LstMergingObjects.push_back( pMO );
                        }
                    }
                }
            }

            // Ajout des autres types de vÃ©hicule que le type de base
            for(int k=1; k<(int)m_LstTypesVehicule.size();k++)
                (*curTuy)->AddTypeVeh(m_LstTypesVehicule[k]);
        }
    }

   // Une fois que la subdivision par voie a Ã©tÃ© faite...

   // Retour sur les connexions afin de charger les mouvements autorises
   std::deque<Connexion*>::iterator itCnx;
   for(itCnx = this->m_LstUserCnxs.begin(); itCnx != this->m_LstUserCnxs.end(); itCnx++)
   {
       DOMNode * XMLtmp = mapUserConnections[(*itCnx)->GetID()];
       if (XMLtmp)
       {
           XMLtmp = m_pXMLUtil->SelectSingleNode("./MOUVEMENTS_AUTORISES", XMLtmp->getOwnerDocument(), (DOMElement*)XMLtmp);
           if (XMLtmp)
           {
               if (!LoadMouvementsAutorises((*itCnx), XMLtmp, &loadingLogger))
               {
                   loadingLogger << Logger::Error << "ERROR : wrong authorized movements definition for connection " << (*itCnx)->GetID() << ".";
                   loadingLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                   return false;
               }
           }
       }
   }

   // *******************************************************************************
   // Cas particulier des rÃ©partiteurs crÃ©Ã©s pour convergent de type "insertion"
   // *******************************************************************************
   for(size_t repInsIdx = 0; repInsIdx < repartiteursInsertion.size(); repInsIdx++)
    {
        Repartiteur *pRepartiteur = repartiteursInsertion[repInsIdx];

        // rÃ©cupÃ©ration du noeud du convergent associÃ©
        DOMNode *pConvInsNode = NULL;
        std::string convInsID = pRepartiteur->GetID().substr(2,pRepartiteur->GetID().find("_INSERTION_")-2);
        for(size_t convInsIdx = 0; convInsIdx < Convergents_Insertion.size(); convInsIdx++)
        {
            std::string insertionID;
            GetXmlAttributeValue(Convergents_Insertion[convInsIdx], "id", insertionID, &loadingLogger);

            if(!convInsID.compare(insertionID))
            {
                pConvInsNode = Convergents_Insertion[convInsIdx];
                break;
            }
        }

        assert(pConvInsNode!=NULL);

        // rÃ©cupÃ©ration du nb de voies d'insertion Ã  droite et Ã  gauche
        int nbVoiesInsDroite, nbVoiesInsGauche;
        GetXmlAttributeValue(pConvInsNode, "nb_voies_insertion_droite", nbVoiesInsDroite, &loadingLogger);
        GetXmlAttributeValue(pConvInsNode, "nb_voies_insertion_gauche", nbVoiesInsGauche, &loadingLogger);

        // EVOLUTION nÂ°70 - on a besoin de connaitre utilise le tuyau aval "principal", dÃ©terminÃ© par l'attribut id_TAv
        std::string strIdTAv;
        GetXmlAttributeValue(pConvInsNode, "id_TAv", strIdTAv, &loadingLogger);

        // on regarde s'il s'agit d'un rÃ©partiteur amont on aval de l'insertion
        std::string strTypeInsertion = pRepartiteur->GetID().substr(pRepartiteur->GetID().length()-4, 4);
        if(!strTypeInsertion.compare("AVAL"))
        {
            // crÃ©ation des mouvements autorisÃ©s aval de l'insertion
            pRepartiteur->CreateMouvementsAutorises(nbVoiesInsDroite, nbVoiesInsGauche, strIdTAv);
        }
        else
        {
            // cas du rÃ©partiteur amont de l'insertion : on utilise les mouvements autorisÃ©s dÃ©finis
            // pour le convergent de type insertion
            DOMNode * pXMLMvts = m_pXMLUtil->SelectSingleNode( "./MOUVEMENTS_AUTORISES", pXMLReseau->getOwnerDocument(), (DOMElement*)pConvInsNode);
            if(!pXMLMvts)
            {
                loadingLogger << Logger::Error << "ERROR : no defined authorized movement for insertion convergent " << convInsID << ".";
                loadingLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                return false;
            }

            // EVOLUTION nÂ°70 : dÃ©tection du cas convergent d'insertion "Ã©changeur"
            // rÃ©cupÃ©ration du rÃ©partiteur aval associÃ©
            bool bEchangeur = false;
            int nNbVoiesEch = 0;
            std::string repInsAvalID = pRepartiteur->GetID().substr(0, pRepartiteur->GetID().find("_AMONT")) + "_AVAL";
            for(size_t repInsIdx = 0; repInsIdx < repartiteursInsertion.size(); repInsIdx++)
            {
                if(!repartiteursInsertion[repInsIdx]->GetID().compare(repInsAvalID))
                {
                    if(repartiteursInsertion[repInsIdx]->m_LstTuyAv.size() > 1)
                    {
                        bEchangeur = true;
                        for(size_t iTuyAv = 0; iTuyAv < repartiteursInsertion[repInsIdx]->m_LstTuyAv.size(); iTuyAv++)
                        {
                            if(repartiteursInsertion[repInsIdx]->m_LstTuyAv[iTuyAv]->GetLabel().compare(strIdTAv))
                            {
                                nNbVoiesEch += repartiteursInsertion[repInsIdx]->m_LstTuyAv[iTuyAv]->getNb_voies();
                            }
                        }
                    }
                    break;
                }
            }

            pRepartiteur->LoadMouvementsAutorises(pXMLMvts, &loadingLogger, true, strIdTAv, nNbVoiesEch);

            // on ne peut appeler SetChgtVoieObligatoire qu'aprÃ¨s la segmentation, c'est pourquoi on le fait ici et non Ã  la crÃ©ation du
            // troncon d'insertion
            Tuyau * T = pRepartiteur->m_LstTuyAv[0];
            Tuyau * pTuyAval = T->getConnectionAval()->m_LstTuyAv[0];
            std::string strTAmonPrioID = GetTronconAmontPrio(pConvInsNode, loadingLogger);
            Tuyau * pTuyanAmontPrio = GetLinkFromLabel(strTAmonPrioID);

            for(int insertionIdx = 0; insertionIdx < nbVoiesInsDroite; insertionIdx++)
            {
                int insVoieIdx = insertionIdx;
                // EVOLUTION nÂ°70 - traitement particulier du cas "Ã©changeur"
                if(bEchangeur)
                {
                    // Pour Ã©viter que les vÃ©hicules qui restent sur le tuyau principal ne se rabattent sur la "voie d'insertion" Ã  droite
                    T->SetDstChgtVoie(T->GetLength()+50.0);
                    // boolÃ©en spÃ©cifique du mÃªme type que SetChgtVoieObligatoire Ã  positionner pour le cas Ã©changeur
                    ((VoieMicro*)T->GetLstLanes()[insVoieIdx])->SetChgtVoieObligatoireEch();
                }
                else
                {
                    ((VoieMicro*)T->GetLstLanes()[insVoieIdx])->SetChgtVoieObligatoire();
                }

                if( this->m_cChgtVoieMandatoryMode == 'A' )
                {
                    Tuyau * pTuyauAmontNonPrio = GetTronconAmontNonPrio(insVoieIdx, pRepartiteur);
                    MergingObject *pMO = new MergingObject(this, T, insVoieIdx, pTuyAval, pTuyanAmontPrio, pTuyauAmontNonPrio, m_dbFAlphaMandatory);
                    m_LstMergingObjects.push_back( pMO );
                }
            }
            for(int insertionIdx = 0; insertionIdx < nbVoiesInsGauche; insertionIdx++)
            {
                int insVoieIdx = T->getNb_voies()-insertionIdx-1;
                ((VoieMicro*)T->GetLstLanes()[insVoieIdx])->SetChgtVoieObligatoire();

                if( this->m_cChgtVoieMandatoryMode == 'A' )
                {
                    Tuyau * pTuyauAmontNonPrio = GetTronconAmontNonPrio(insVoieIdx, pRepartiteur);
                    MergingObject *pMO = new MergingObject(this, T, insVoieIdx, pTuyAval, pTuyanAmontPrio, pTuyauAmontNonPrio, m_dbFAlphaMandatory);
                    m_LstMergingObjects.push_back( pMO );
                }
            }
        }
    }
   // *******************************************************************************
   // Fin cas particulier des rÃ©partiteurs crÃ©Ã©s pour convergent de type "insertion"
   // *******************************************************************************

   // Chargement de l'affectation
   if( this->IsCptDestination() )
   {
        for(itCnx = this->m_LstUserCnxs.begin(); itCnx != this->m_LstUserCnxs.end(); itCnx++)
        {
            bool bAllType = false;
            const std::string & ssP = (*itCnx)->GetID();
            DOMNode * XMLtmp = NULL;
            XMLtmp = m_pXMLUtil->SelectSingleNode(("./CONNEXIONS_INTERNES/CONNEXION_INTERNE[@id=\"" + ssP + "\"]/AFFECTATIONS_TYPE_VEHICULE").c_str(), pXMLNodeTrafic->getOwnerDocument(), (DOMElement*)pXMLNodeTrafic);
            if(!XMLtmp)
            {
                XMLtmp = m_pXMLUtil->SelectSingleNode(("./CONNEXIONS_INTERNES/CONNEXION_INTERNE[@id=\"" + ssP + "\"]/AFFECTATIONS").c_str(), pXMLNodeTrafic->getOwnerDocument(), (DOMElement*)pXMLNodeTrafic);
                bAllType = true;		// MÃªme donnÃ©es d'affectation pour tous les types de vÃ©hicule
            }
            if(XMLtmp)
                (*itCnx)->LoadAffectation(XMLtmp, bAllType, &loadingLogger);
        }
   }

   // Retour sur les rÃ©partiteurs en mode directionnel, aprÃ¨s chargement des mouvements autorisÃ©s et segmentation des liens
   if (!IsUsedODMatrix())
   {
       pXMLNode = m_pXMLUtil->SelectSingleNode("./CONNEXIONS/REPARTITEURS", pXMLReseau->getOwnerDocument(), (DOMElement*)pXMLReseau);

       if (pXMLNode)
       {
           XMLSize_t counti = pXMLNode->getChildNodes()->getLength();
           int ii = 0;
           for (XMLSize_t i = 0; i<counti; i++)
           {
               pXMLChild = pXMLNode->getChildNodes()->item(i);
               if (pXMLChild->getNodeType() != DOMNode::ELEMENT_NODE) continue;

               GetXmlAttributeValue(pXMLChild, "id", strTmp, &loadingLogger);

               pRepartiteur = Liste_repartiteurs[strTmp];

               if (!(pRepartiteur->GetNbElAmont() > 0 && pRepartiteur->GetNbElAval() > 0))
               {
                   ii++;
                   continue;      // rÃ©partiteur non utilisÃ©
               }

               // RÃ©partition du flux
               if (!BuildRepartitionFlux(pRepartiteur, pXMLChild->getOwnerDocument(), &loadingLogger))
                   return false;

               ii++;
           }
       } // Fin boucle sur les noeuds des rÃ©partiteurs
   }

    // Retour sur les convergents
    // SECTION RESEAU / CONNEXIONS / CONVERGENTS
    double dbPos;
    Tuyau *pTAm, *pTAv;
    int nNiv;

    pXMLNode = m_pXMLUtil->SelectSingleNode("./CONNEXIONS/CONVERGENTS", pXMLReseau->getOwnerDocument(), (DOMElement*)pXMLReseau);

    if(pXMLNode)
    {
        XMLSize_t counti = pXMLNode->getChildNodes()->getLength();
        int ii = 0;
        for(XMLSize_t i=0; i<counti;i++)
        {
            pXMLChild = pXMLNode->getChildNodes()->item(i);
            if (pXMLChild->getNodeType() != DOMNode::ELEMENT_NODE) continue;

            // si convergent d'insertion, pas de convergent associÃ© dans Liste_convergents
            GetXmlAttributeValue(pXMLChild, "type", strTmp, &loadingLogger);
            if(!strTmp.compare("insertion"))
            {
                continue;
            }

            GetXmlAttributeValue(pXMLChild, "id", strTmp, &loadingLogger);

            pConvergent = Liste_convergents[strTmp];

            // TronÃ§on aval
            GetXmlAttributeValue(pXMLChild, "id_TAv", strTmp, &loadingLogger);
            pTAv = GetLinkFromLabel(strTmp);

            pConvergent->Init(pTAv);

            // rÃ©cupÃ©ration des paramÃ¨tres de trafic
            DOMNode * pXMLTraficParams = m_pXMLUtil->SelectSingleNode("./CONNEXIONS_INTERNES/CONNEXION_INTERNE[@id=\"" + pConvergent->GetID() + "\"]", pXMLNodeTrafic->getOwnerDocument(), (DOMElement*)pXMLNodeTrafic);
            // Positions du capteur aval
            if(!GetXmlAttributeValue(pXMLTraficParams, "pos_cpt_Av", dbPos, &loadingLogger)) dbPos = dbDefaultPosCptAval;

            // TronÃ§ons amont
            DOMNode * pXMLLstTrAm = m_pXMLUtil->SelectSingleNode("./TRONCONS_AMONT", pXMLChild->getOwnerDocument(), (DOMElement*)pXMLChild);
            DOMNode * pXMLTrAm;

            if(pXMLLstTrAm)
            {
                XMLSize_t countj = pXMLLstTrAm->getChildNodes()->getLength();
                for(XMLSize_t j=0; j<countj;j++)
                {
                    pXMLTrAm = pXMLLstTrAm->getChildNodes()->item(j);
                    if (pXMLTrAm->getNodeType() != DOMNode::ELEMENT_NODE) continue;

                    // id
                    GetXmlAttributeValue(pXMLTrAm, "id", strTmp, &loadingLogger);
                    pTAm = GetLinkFromLabel(strTmp);

                    // Positions du capteur
                    GetXmlAttributeValue(pXMLTrAm, "pos_cpt", dbPos, &loadingLogger);

                    // niveau de prioritÃ©
                    GetXmlAttributeValue(pXMLTrAm, "priorite", nNiv, &loadingLogger);

                    //pConvergent->AddTuyauAmont(pTAm, dbPos, nNiv);
                    for(int k=0; k<pTAm->getNb_voies();k++)
                        for(int l=0; l<pConvergent->m_LstTuyAv.front()->getNb_voies();l++)
                            pConvergent->AddMouvement( (VoieMicro*)pTAm->GetLstLanes()[k],  (VoieMicro*)pConvergent->m_LstTuyAv.front()->GetLstLanes()[l], nNiv);
                }
            }
            pConvergent->FinInit( pTAv, dbPos);

            if(!pXMLTraficParams)
            {
                loadingLogger << Logger::Error << "ERROR: no traffic parameters definition for convergent " << pConvergent->GetID() << std::endl;
                loadingLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                return false;
            }

            // Temps d'insertion
            DOMNode * pXMLLstTpsIns = m_pXMLUtil->SelectSingleNode("./LISTE_TEMPS_INSERTION", pXMLTraficParams->getOwnerDocument(), (DOMElement*)pXMLTraficParams);
            DOMNode * pXMLTpsIns;
            // pour tous les types de vÃ©hicule, on utilise le temps par dÃ©faut sauf si un temps d'insertion est dÃ©fini dans la liste
            for(size_t vehIdx = 0; vehIdx < m_LstTypesVehicule.size(); vehIdx++)
            {
                bool bNoDefault = false;
                if(pXMLLstTpsIns)
                {
                    XMLSize_t countj = pXMLLstTpsIns->getChildNodes()->getLength();
                    for(XMLSize_t j=0; j<countj;j++)
                    {
                        pXMLTpsIns = pXMLLstTpsIns->getChildNodes()->item(j);
                        if (pXMLTpsIns->getNodeType() != DOMNode::ELEMENT_NODE) continue;

                        // Type de vÃ©hicule
                        GetXmlAttributeValue(pXMLTpsIns, "id_typevehicule", strTmp, &loadingLogger);

                        if(!strTmp.compare(m_LstTypesVehicule[vehIdx]->GetLabel()))
                        {
                            bNoDefault = true;
                            // Temps d'insertion
                            GetXmlAttributeValue(pXMLTpsIns, "ti", dbTmp, &loadingLogger);
                            break;
                        }
                    }
                }

                if(!bNoDefault)
                {
                    dbTmp = dbDefaultTi;
                }
                pConvergent->AddTf(m_LstTypesVehicule[vehIdx], dbTmp);
            }

            ii++;
        }
    }


    // Retour sur les giratoires
    Tuyau *pT;

    // SECTION RESEAU / CONNEXIONS / GIRATOIRES
    pXMLNode = m_pXMLUtil->SelectSingleNode("./CONNEXIONS/GIRATOIRES", pXMLReseau->getOwnerDocument(), (DOMElement*)pXMLReseau);

    if(pXMLNode)
    {
        XMLSize_t counti = pXMLNode->getChildNodes()->getLength();
        int ii = 0;
        for(XMLSize_t i=0; i<counti;i++)
        {
            pXMLChild = pXMLNode->getChildNodes()->item(i);
            if (pXMLChild->getNodeType() != DOMNode::ELEMENT_NODE) continue;

            pGiratoire = (Giratoire*)Liste_giratoires[ii++];

            // TronÃ§ons
            GetXmlAttributeValue(pXMLChild, "troncons", strTmp, &loadingLogger);

            std::deque<std::string> split = SystemUtil::split(strTmp, ' ');

            for(size_t j=0; j< split.size(); j++)
            {
                pT = GetLinkFromLabel(split.at(j));
                if(pT)
                    pGiratoire->AddTuyauAmAv((TuyauMicro*)pT);
                else
                {
                    loadingLogger << Logger::Error << "ERROR : Wrong links list for the roundabout ";
                    loadingLogger << Logger::Error << pGiratoire->GetLabel() << std::endl;
                    loadingLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                    return false;
                }
            }

            // Pour rÃ©cupÃ©ration des paramÃ¨tres de trafic
            DOMNode * pXMLTraficParams = m_pXMLUtil->SelectSingleNode("./CONNEXIONS_INTERNES/CONNEXION_INTERNE[@id=\"" + pGiratoire->GetID() + "\"]", pXMLNodeTrafic->getOwnerDocument(), (DOMElement*)pXMLNodeTrafic);


            // Temps d'insertion
            DOMNode *pXMLLstTpsIns = m_pXMLUtil->SelectSingleNode("./LISTE_TEMPS_INSERTION", pXMLTraficParams->getOwnerDocument(), (DOMElement*)pXMLTraficParams);
            DOMNode *pXMLTpsIns;
            // pour tous les types de vÃ©hicule, on utilise le temps par dÃ©faut sauf si un temps d'insertion est dÃ©fini dans la liste
            for(size_t vehIdx = 0; vehIdx < m_LstTypesVehicule.size(); vehIdx++)
            {
                bool bNoDefault = false;
                if(pXMLLstTpsIns)
                {
                    XMLSize_t countj = pXMLLstTpsIns->getChildNodes()->getLength();
                    for(XMLSize_t j=0; j<countj;j++)
                    {
                        pXMLTpsIns = pXMLLstTpsIns->getChildNodes()->item(j);
                        if (pXMLTpsIns->getNodeType() != DOMNode::ELEMENT_NODE) continue;

                        // Type de vÃ©hicule
                        GetXmlAttributeValue(pXMLTpsIns, "id_typevehicule", strTmp, &loadingLogger);
                        if(!strTmp.compare(m_LstTypesVehicule[vehIdx]->GetLabel()))
                        {
                            bNoDefault = true;
                            // Temps d'insertion
                            GetXmlAttributeValue(pXMLTpsIns, "ti", dbTmp, &loadingLogger);
                            // Temps de traversÃ©e
                            if(pGiratoire->GetNbVoie() > 1)
                                GetXmlAttributeValue(pXMLTpsIns, "tt", dbTmp2, &loadingLogger);

                            break;
                        }
                    }
                }

                if(!bNoDefault)
                {
                    dbTmp = dbDefaultTi;
                    dbTmp2 = dbDefaultTt;
                }
                pGiratoire->AddTf(m_LstTypesVehicule[vehIdx], dbTmp, dbTmp2);
            }



            pGiratoire->Init();

            // Lecture des caractÃ©ristiques des tronÃ§ons internes
            DOMNode *pXMLNodeTIs;
            DOMNode *pXMLNodeTI;
            Tuyau* pTuyau;
            std::string strTAm, strTAv, strID;
            pXMLNodeTIs = m_pXMLUtil->SelectSingleNode("./TRONCONS_INTERNES", pXMLChild->getOwnerDocument(), (DOMElement*)pXMLChild);
            if(pXMLNodeTIs)
            {
                XMLSize_t countj = pXMLNodeTIs->getChildNodes()->getLength();
                for(XMLSize_t j=0; j<countj;j++)
                {
                    pXMLNodeTI = pXMLNodeTIs->getChildNodes()->item(j);
                    if (pXMLNodeTI->getNodeType() != DOMNode::ELEMENT_NODE) continue;

                    GetXmlAttributeValue(pXMLNodeTI, "troncon_amont", strTAm, &loadingLogger);
                    GetXmlAttributeValue(pXMLNodeTI, "troncon_aval", strTAv, &loadingLogger);
                    strTmp = strTAm  + strTAv;

                    GetXmlAttributeValue(pXMLNodeTI, "id", strID, &loadingLogger);

                    pTuyau = GetLinkFromLabel(strTmp);
                    if(pTuyau)
                    {
                        GetXmlAttributeValue(pXMLNodeTI, "extremite_amont", ptExtAmont, &loadingLogger);           // ExtrÃ©mitÃ© amont
                        GetXmlAttributeValue(pXMLNodeTI, "extremite_aval", ptExtAval, &loadingLogger);             // ExtrÃ©mitÃ© aval

                        pTuyau->SetExtAmont(ptExtAmont);
                        pTuyau->SetExtAval(ptExtAval);

                        char szID[128];
                        strcpy(szID, strID.c_str());
                        pTuyau->SetLabel(szID);

                        m_mapTuyaux[pTuyau->GetLabel()] = pTuyau;		// Ajout dans la map pour accÃ¨s optimisÃ© (les tronÃ§ons internes sont dÃ©jÃ  prÃ©sents mais sous un autre label)

                        // RÃ©cupÃ©ration du nb de cellules acoustiques dans les paramÃ¨tres de simulation
                        DOMNode * pXMLSimuParams = m_pXMLUtil->SelectSingleNode("./ELEMENTS/ELEMENT[@id=\"" + strID + "\"]", pXMLNodeSimulation->getOwnerDocument(), (DOMElement*)pXMLNodeSimulation);

                        if(pXMLSimuParams)
                            GetXmlAttributeValue(pXMLSimuParams, "nb_cell_acoustique", nTmp, &loadingLogger);
                        else
                            nTmp = 0;

                        TuyauMicro * pTuyMicro = dynamic_cast<TuyauMicro*>(pTuyau);
                        if(pTuyMicro)
                        {
                            pTuyMicro->SetNbCellAcoustique(nTmp);
                        }
                        pTuyau->SetNbCell(nTmp);

                        // Lecture des points internes du tronÃ§on
                        Point   pt;
                        DOMNode * XMLPts = m_pXMLUtil->SelectSingleNode("./POINTS_INTERNES", pXMLNodeTI->getOwnerDocument(), (DOMElement*)pXMLNodeTI);
                        if(XMLPts)
                        {
                            for(size_t k=0; k<XMLPts->getChildNodes()->getLength(); k++)
                            {
                                DOMNode * xmlChildk = XMLPts->getChildNodes()->item(k);
                                if (xmlChildk->getNodeType() != DOMNode::ELEMENT_NODE) continue;

                                GetXmlAttributeValue(xmlChildk, "coordonnees", pt, &loadingLogger);

                                pTuyau->AddPtInterne(pt.dbX, pt.dbY, pt.dbZ);
                            }
                        }

                        GetXmlAttributeValue(pXMLNodeTI, "longueur", dbLongueur, &loadingLogger);
                        // TronÃ§on curviligne ?
                        pTuyau->SetCurviligne(dbLongueur != 0);
                        // Calcul de la longueur si non saisie et de la longueur de projection
                        pTuyau->CalculeLongueur(dbLongueur);

                        pTuyau->Segmentation();

                    }
                    else
                    {
                        loadingLogger << Logger::Error << "ERROR : Wront list of internal links for roundabout ";
                        loadingLogger << Logger::Error << pGiratoire->GetLabel() << std::endl;
                        loadingLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                        return false;
                    }
                }
            }

            // Lecture des coefficients d'insertion manuels
            DOMNode * pXMLNodeCoefsInsGir = m_pXMLUtil->SelectSingleNode("./COEFFICIENTS_INSERTION", pXMLChild->getOwnerDocument(), (DOMElement*)pXMLChild);
            if(pXMLNodeCoefsInsGir)
            {
                XMLSize_t countj = pXMLNodeCoefsInsGir->getChildNodes()->getLength();
                for(XMLSize_t j=0; j<countj;j++)
                {
                    DOMNode * pXMLNodeCoefInsGir = pXMLNodeCoefsInsGir->getChildNodes()->item(j);
                    if (pXMLNodeCoefInsGir->getNodeType() != DOMNode::ELEMENT_NODE) continue;

                    // rÃ©cupÃ©ration des troncons entrÃ©e et sortie et de la voie d'entrÃ©e
                    int nVoieEntree;
                    GetXmlAttributeValue(pXMLNodeCoefInsGir, "num_voie_entree", nVoieEntree, &loadingLogger);

                    std::string strIDTmp;
                    GetXmlAttributeValue(pXMLNodeCoefInsGir, "troncon_entree", strIDTmp, &loadingLogger);
                    Tuyau * pTuyEntreeGir = GetLinkFromLabel(strIDTmp);

                    GetXmlAttributeValue(pXMLNodeCoefInsGir, "troncon_sortie", strIDTmp, &loadingLogger);
                    Tuyau * pTuySortieGir = GetLinkFromLabel(strIDTmp);

                    // rÃ©cupÃ©ration des coefficients
                    GetXmlAttributeValue(pXMLNodeCoefInsGir, "coeffs", strIDTmp, &loadingLogger);
                    std::deque<std::string> split = SystemUtil::split(strIDTmp, ' ');
                    if(split.size() != pGiratoire->GetNbVoie())
                    {
                        loadingLogger << Logger::Error << "ERROR : the insertion coefficients list for roundabout " << pGiratoire->GetID() << " is incorrect (number of coefficients different from the number of lanes ?)." << std::endl;
                        loadingLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                        return false;
                    }
                    std::vector<double> insGirCoeffs;
                    for(size_t j=0; j < split.size(); j++)
                    {
                        insGirCoeffs.push_back(SystemUtil::ToDouble(split[j]));
                    }

                    pGiratoire->AddCoefficientInsertion(pTuyEntreeGir, nVoieEntree-1, pTuySortieGir, insGirCoeffs);
                }
            }

            // Calcul du nb max de voie pour le GIR (pour les coefficients directionnels)
            int nNbmaxVoies = 1;
            for (size_t j = 0; j < pGiratoire->m_LstTuyAm.size(); j++)
            {
                if (pGiratoire->m_LstTuyAm[j]->GetLanesManagement() == 'D' && pGiratoire->m_LstTuyAm[j]->getNb_voies() > nNbmaxVoies)
                    nNbmaxVoies = pGiratoire->m_LstTuyAm[j]->getNb_voies();
            }
            for (size_t j = 0; j < pGiratoire->m_LstTuyAv.size(); j++)
            {
                if (pGiratoire->m_LstTuyAv[j]->GetLanesManagement() == 'D' &&  pGiratoire->m_LstTuyAv[j]->getNb_voies() > nNbmaxVoies)
                    nNbmaxVoies = pGiratoire->m_LstTuyAv[j]->getNb_voies();
            }
            pGiratoire->SetNbMaxVoies(nNbmaxVoies);

            // RÃ©partition du flux
            if (!IsUsedODMatrix())
            {
                if (!BuildRepartitionFlux(pGiratoire, pXMLChild->getOwnerDocument(), &loadingLogger))
                    return false;
            }
        }
    }

    // Retour sur les carrefours Ã  feux

    // SECTION RESEAU / CONNEXIONS / CARREFOURSAFEUX
    pXMLNode = m_pXMLUtil->SelectSingleNode( "./CONNEXIONS/CARREFOURSAFEUX", pXMLReseau->getOwnerDocument(), (DOMElement*)pXMLReseau);

    if(pXMLNode)
    {
        XMLSize_t counti = pXMLNode->getChildNodes()->getLength();
        int ii = 0;
        for(XMLSize_t i=0; i<counti;i++)
        {
            pXMLChild = pXMLNode->getChildNodes()->item(i);
            if (pXMLChild->getNodeType() != DOMNode::ELEMENT_NODE) continue;

            pCAF = (CarrefourAFeuxEx*)Liste_carrefoursAFeux[ii];

            DOMNode * pXMLTraficParams = m_pXMLUtil->SelectSingleNode("./CONNEXIONS_INTERNES/CONNEXION_INTERNE[@id=\"" + pCAF->GetID() + "\"]", pXMLNodeTrafic->getOwnerDocument(), (DOMElement*)pXMLNodeTrafic);

            // Temps d'insertion (traversÃ©e)
            DOMNode *pXMLLstTpsIns = NULL;
            if(pXMLTraficParams)
            {
                pXMLLstTpsIns = m_pXMLUtil->SelectSingleNode( "./LISTE_TEMPS_INSERTION", pXMLTraficParams->getOwnerDocument(), (DOMElement*)pXMLTraficParams);
            }
            DOMNode *pXMLTpsIns;
            // pour tous les types de vÃ©hicule, on utilise le temps par dÃ©faut sauf si un temps d'insertion est dÃ©fini dans la liste
            for(size_t vehIdx = 0; vehIdx < m_LstTypesVehicule.size(); vehIdx++)
            {
                bool bNoDefault = false;
                if(pXMLLstTpsIns)
                {
                    XMLSize_t countj = pXMLLstTpsIns->getChildNodes()->getLength();
                    for(XMLSize_t j=0; j<countj;j++)
                    {
                        pXMLTpsIns = pXMLLstTpsIns->getChildNodes()->item(j);
                        if (pXMLTpsIns->getNodeType() != DOMNode::ELEMENT_NODE) continue;

                        // Type de vÃ©hicule
                        GetXmlAttributeValue(pXMLTpsIns, "id_typevehicule", strTmp, &loadingLogger);
                        if(!strTmp.compare(m_LstTypesVehicule[vehIdx]->GetLabel()))
                        {
                            bNoDefault = true;
                            // Temps de traversÃ©e
                            GetXmlAttributeValue(pXMLTpsIns, "tt", dbTmp, &loadingLogger);
                            // Temps d'insertion (convergents)
                            GetXmlAttributeValue(pXMLTpsIns, "ti", dbTmp2, &loadingLogger);
                            break;
                        }
                    }
                }

                if(!bNoDefault)
                {
                    dbTmp = dbDefaultTt;
                    dbTmp2 = dbDefaultTi;
                }
                pCAF->AddTfTra(m_LstTypesVehicule[vehIdx], dbTmp);
                pCAF->AddTfCvg(m_LstTypesVehicule[vehIdx], dbTmp2);
            }

            if( !pCAF->Init(pXMLChild, &loadingLogger))
            {
                return false;
            }

            // Calcul du nb max de voie pour le CAF (pour les coefficients directionnels)
            int nNbmaxVoies = 1;
            for (size_t j = 0; j < pCAF->m_LstTuyAm.size(); j++)
            {
                if (pCAF->m_LstTuyAm[j]->GetLanesManagement() == 'D' && pCAF->m_LstTuyAm[j]->getNb_voies() > nNbmaxVoies)
                    nNbmaxVoies = pCAF->m_LstTuyAm[j]->getNb_voies();
            }
            for (size_t j = 0; j < pCAF->m_LstTuyAv.size(); j++)
            {
                if (pCAF->m_LstTuyAv[j]->GetLanesManagement() == 'D' &&  pCAF->m_LstTuyAv[j]->getNb_voies() > nNbmaxVoies)
                    nNbmaxVoies = pCAF->m_LstTuyAv[j]->getNb_voies();
            }
            pCAF->SetNbMaxVoies(nNbmaxVoies);

            // RÃ©partition du flux
            if (!IsUsedODMatrix())
            {
                if (!BuildRepartitionFlux(pCAF, pXMLChild->getOwnerDocument(), &loadingLogger))
                    return false;
            }

            ii++;
        }
    }

    // SECTION TRAFIC / MOTIFS
    pXMLNode = m_pXMLUtil->SelectSingleNode("./MOTIFS", pXMLNodeTrafic->getOwnerDocument(), (DOMElement*)pXMLNodeTrafic);

    if (pXMLNode)
    {
        // Lecture de la dÃ©finition des briques de rÃ©gulation
        XMLSize_t counti = pXMLNode->getChildNodes()->getLength();
        for (XMLSize_t i = 0; i<counti; i++)
        {
            DOMNode *pXMLMotif = pXMLNode->getChildNodes()->item(i);
            if (pXMLMotif->getNodeType() != DOMNode::ELEMENT_NODE) continue;

            GetXmlAttributeValue(pXMLMotif, "id", strTmp, &loadingLogger);

            // CrÃ©ation du motif et ajout Ã  la liste des motifs
            CMotif * pMotif = new CMotif(strTmp);

            GetXmlAttributeValue(pXMLMotif, "description", strTmp, &loadingLogger);
            pMotif->setDescription(strTmp);

            m_LstMotifs.push_back(pMotif);
        }
    }

    // Retour sur les entrÃ©es
    pXMLNode = m_pXMLUtil->SelectSingleNode( "./CONNEXIONS/EXTREMITES", pXMLReseau->getOwnerDocument(), (DOMElement*)pXMLReseau);

    counti = pXMLNode->getChildNodes()->getLength();
    for(XMLSize_t i=0; i<counti;i++)
    {
        pXMLChild = pXMLNode->getChildNodes()->item(i);
        if (pXMLChild->getNodeType() != DOMNode::ELEMENT_NODE) continue;

        GetXmlAttributeValue(pXMLChild, "id", strTmp, &loadingLogger);

        // On se dÃ©place vers le noeud des paramÃ¨tres de trafic de l'entrÃ©e
        pXMLChild = m_pXMLUtil->SelectSingleNode("./EXTREMITES/EXTREMITE[@id=\"" + strTmp + "\"]", pXMLNodeTrafic->getOwnerDocument(), (DOMElement*)pXMLNodeTrafic);

        char cOrigineType;
        SymuViaTripNode *pOrigine = GetOrigineFromID(strTmp, cOrigineType);

        if(pOrigine)
        {
            if(!LoadCreationRepartitions(pOrigine, pXMLChild, &loadingLogger))
            {
                return false;
            }
        }
    }

    // Retour sur les parkings
    pXMLNode = m_pXMLUtil->SelectSingleNode( "./CONNEXIONS/PARKINGS", pXMLReseau->getOwnerDocument(), (DOMElement*)pXMLReseau);

    if(pXMLNode)
    {
        counti = pXMLNode->getChildNodes()->getLength();
        for(XMLSize_t i=0; i<counti;i++)
        {
            pXMLChild = pXMLNode->getChildNodes()->item(i);
            if (pXMLChild->getNodeType() != DOMNode::ELEMENT_NODE) continue;

            GetXmlAttributeValue(pXMLChild, "id", strTmp, &loadingLogger);

            // On se dÃ©place vers le noeud des paramÃ¨tres de trafic de l'entrÃ©e
            pXMLChild = m_pXMLUtil->SelectSingleNode("./PARKINGS/PARKING[@id=\"" + strTmp + "\"]", pXMLNodeTrafic->getOwnerDocument(), (DOMElement*)pXMLNodeTrafic);

            if(pXMLChild)
            {
                char cOrigineType;
                SymuViaTripNode *pOrigine = GetOrigineFromID(strTmp, cOrigineType);

                if(pOrigine)
                {
                    if(!LoadCreationRepartitions(pOrigine, pXMLChild, &loadingLogger))
                    {
                        return false;
                    }
                }
            }
        }
    }

    // SECTION RESEAU / CONNEXIONS / ZONES_DE_TERMINAISON (fait Ã  la fin car on a besoin de connaitre les connexions et tronÃ§ons associÃ©s)
    pXMLNode = m_pXMLUtil->SelectSingleNode("./ZONES_DE_TERMINAISON", pXMLReseau->getOwnerDocument(), (DOMElement*)pXMLReseau);

    if(pXMLNode)
    {
        counti = pXMLNode->getChildNodes()->getLength();
        for(XMLSize_t i=0; i<counti;i++)
        {
            DOMNode *pXMLChildi = pXMLNode->getChildNodes()->item(i);
            if (pXMLChildi->getNodeType() != DOMNode::ELEMENT_NODE) continue;

            // ID
            GetXmlAttributeValue(pXMLChildi, "id", strTmp, &loadingLogger);

            ZoneDeTerminaison* pZone = NULL;
            for(size_t zoneIdx = 0; zoneIdx < Liste_zones.size() && pZone == NULL; zoneIdx++)
            {
                if(!Liste_zones[zoneIdx]->GetInputID().compare(strTmp))
                {
                    pZone = Liste_zones[zoneIdx];
                }
            }

            // RÃ©cupÃ©ration des donnÃ©es de trafic
            pXMLChildi = m_pXMLUtil->SelectSingleNode("./ZONES_DE_TERMINAISON/ZONE_DE_TERMINAISON[@id=\"" + strTmp + "\"]", pXMLNodeTrafic->getOwnerDocument(), (DOMElement*)pXMLNodeTrafic);

            // ParamÃ¨tres globaux de gestion des zones de terminaison
            double dbMaxRangeInZonesForThisZone = dbMaxRangeInZones;
            if (GetXmlAttributeValue(pXMLChildi, "zone_distance_terminaison_max", dbTmp, &loadingLogger))
            {
                dbMaxRangeInZonesForThisZone = dbTmp;
            }
            pZone->SetMaxDistanceToJunction(dbMaxRangeInZonesForThisZone);

            // RÃ©cupÃ©ration des paramÃ¨tres de stationnement
            DOMNode * pParkingForZone = m_pXMLUtil->SelectSingleNode("./PARAMETRAGE_STATIONNEMENT", pXMLChildi->getOwnerDocument(), (DOMElement*)pXMLChildi);
            if (pParkingForZone)
            {
                if (!pParkingNode)
                {
                    loadingLogger << Logger::Warning << "WARNING : the parking parameters defined for the zone " << pZone->GetID() << " are ignored because the TRAFIC/PARAMETRAGE_STATIONNEMENT node is missing" << std::endl;
                    loadingLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                }

                ParkingParameters * pZoneParkingParams = new ParkingParameters();
                pZone->SetParkingParameters(pZoneParkingParams, true);
                if (!pZoneParkingParams->LoadFromXML(this, pParkingForZone, m_pParkingParameters, &loadingLogger))
                {
                    return false;
                }
            }
            else
            {
                // Pas de paramÃ¨tres pour la zone : on pointe vers le jeu de paramÃ¨tres gÃ©nÃ©ral
                pZone->SetParkingParameters(m_pParkingParameters, false);
            }

            // RÃ©cupÃ©ration de la liste des tronÃ§ons constituant la zone
            std::vector<Tuyau*> lstTuyauxZone;
            pXMLNodeTroncons = m_pXMLUtil->SelectSingleNode("./TRONCONS", pXMLChildi->getOwnerDocument(), (DOMElement*)pXMLChildi);
            if(pXMLNodeTroncons)
            {
                XMLSize_t countit = pXMLNodeTroncons->getChildNodes()->getLength();
                for(XMLSize_t i=0; i<countit;i++)
                {
                    pXMLChild = pXMLNodeTroncons->getChildNodes()->item(i);
                    if (pXMLChild->getNodeType() != DOMNode::ELEMENT_NODE) continue;

                    GetXmlAttributeValue(pXMLChild, "id", strTmp, &loadingLogger); // Identifiant du tronÃ§on
                    Tuyau * pTuyau = GetLinkFromLabel(strTmp);
                    lstTuyauxZone.push_back(pTuyau);

                    GetXmlAttributeValue(pXMLChild, "capacite_stationnement", dbTmp, &loadingLogger); // CapacitÃ© de stationnement (stock max)
                    pTuyau->SetCapaciteStationnement(dbTmp);

                    GetXmlAttributeValue(pXMLChild, "stationnement_initial", dbTmp, &loadingLogger); // valeur de stationnement initial
                    pTuyau->IncStockStationnement(dbTmp);
                }
            }

            // RÃ©cupÃ©ration de la liste des plaques associÃ©es Ã  la zone
            DOMNode * pXMLNodePlaques = m_pXMLUtil->SelectSingleNode("./PLAQUES", pXMLChildi->getOwnerDocument(), (DOMElement*)pXMLChildi);
            if (pXMLNodePlaques)
            {
                XMLSize_t countit = pXMLNodePlaques->getChildNodes()->getLength();
                for (XMLSize_t it = 0; it<countit; it++)
                {
                    pXMLChild = pXMLNodePlaques->getChildNodes()->item(it);
                    if (pXMLChild->getNodeType() != DOMNode::ELEMENT_NODE) continue;

                    GetXmlAttributeValue(pXMLChild, "id", strTmp, &loadingLogger); // Identifiant de la plaque

                    CPlaque * pPlaque = new CPlaque(pZone, strTmp);

                    // RÃ©cupÃ©ration des paramÃ¨tres de stationnement
                    DOMNode * pParkingForPlaque = m_pXMLUtil->SelectSingleNode("./PARAMETRAGE_STATIONNEMENT", pXMLChildi->getOwnerDocument(), (DOMElement*)pXMLChild);
                    if (pParkingForPlaque)
                    {
                        if (!pParkingNode)
                        {
                            loadingLogger << Logger::Warning << "WARNING : the parking parameters defined for the plaque " << pPlaque->GetID() << " are ignored because the TRAFIC/PARAMETRAGE_STATIONNEMENT node is missing" << std::endl;
                            loadingLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                        }

                        ParkingParameters * pPlaqueParkingParams = new ParkingParameters();
                        pPlaque->SetParkingParameters(pPlaqueParkingParams, true);
                        if (!pPlaqueParkingParams->LoadFromXML(this, pParkingForPlaque, pZone->GetParkingParameters(), &loadingLogger))
                        {
                            return false;
                        }
                    }
                    else
                    {
                        // Pas de paramÃ¨tres pour la plaque : on pointe vers le jeu de paramÃ¨tres de la zone
                        pPlaque->SetParkingParameters(pZone->GetParkingParameters(), false);
                    }

                    // dÃ©codage des sections
                    DOMNode * pXMLNodeSections = m_pXMLUtil->SelectSingleNode("./SECTIONS", pXMLChild->getOwnerDocument(), (DOMElement*)pXMLChild);
                    if (pXMLNodeSections)
                    {
                        XMLSize_t countitSections = pXMLNodeSections->getChildNodes()->getLength();
                        for (XMLSize_t itSec = 0; itSec<countitSections; itSec++)
                        {
                            DOMNode * pXMLChildSection = pXMLNodeSections->getChildNodes()->item(itSec);
                            if (pXMLChildSection->getNodeType() != DOMNode::ELEMENT_NODE) continue;

                            GetXmlAttributeValue(pXMLChildSection, "troncon", strTmp, &loadingLogger);
                            Tuyau * pTuyau = GetLinkFromLabel(strTmp);

                            if (!GetSectionFromNode(pPlaque->GetID(), pXMLChildSection, pTuyau, dbTmp, dbTmp2, &loadingLogger))
                            {
                                return false;
                            }
                            else
                            {
                                CSection plaqueSection(pTuyau, dbTmp, dbTmp2);
                                pPlaque->AddSection(plaqueSection);
                            }
                        }
                    }

                    // dÃ©codage de la rÃ©partition des motifs par plaque
                    DOMNode * pXMLNodeRepMotifPlaques = m_pXMLUtil->SelectSingleNode("./REP_MOTIFS_PLAQUES", pXMLChild->getOwnerDocument(), (DOMElement*)pXMLChild);
                    if (pXMLNodeRepMotifPlaques)
                    {
                        XMLSize_t countitRepMotifPlaques = pXMLNodeRepMotifPlaques->getChildNodes()->getLength();
                        for (XMLSize_t itRepMotifPlaque = 0; itRepMotifPlaque<countitRepMotifPlaques; itRepMotifPlaque++)
                        {
                            DOMNode * pXMLChildRepMotifPlaque = pXMLNodeRepMotifPlaques->getChildNodes()->item(itRepMotifPlaque);
                            if (pXMLChildRepMotifPlaque->getNodeType() != DOMNode::ELEMENT_NODE) continue;

                            GetXmlAttributeValue(pXMLChildRepMotifPlaque, "motif", strTmp, &loadingLogger);
                            CMotif * pMotif = GetMotifFromID(strTmp);

                            GetXmlAttributeValue(pXMLChildRepMotifPlaque, "coeff", dbTmp, &loadingLogger);

                            pPlaque->SetMotifCoeff(pMotif, dbTmp);
                        }
                    }

                    // ajout de la plaque Ã  la zone
                    pZone->AddPlaque(pPlaque);
                } // Fin de la boucle sur les plaques
            }

            // RÃ©cupÃ©ration des parkings associÃ©s Ã  la zone
            DOMXPathResult * pXMLNodeListParkings = m_pXMLUtil->SelectNodes("./PARKINGS/PARKING", pXMLChildi->getOwnerDocument(), (DOMElement*)pXMLChildi);
            XMLSize_t countParkings = pXMLNodeListParkings->getSnapshotLength();
            for (XMLSize_t k = 0; k<countParkings; k++)
            {
                pXMLNodeListParkings->snapshotItem(k);
                pXMLChild = pXMLNodeListParkings->getNodeValue();

                GetXmlAttributeValue(pXMLChild, "id", strTmp, &loadingLogger);
                char cOrigineType;
                Parking * pParking = (Parking*)GetOrigineFromID(strTmp, cOrigineType);
                if (pParking)
                {
                    pParking->SetZoneParent(pZone);
                    pZone->AddParking(pParking);
                }
                else
                {
                    loadingLogger << Logger::Error << "ERROR : unable to find the parking " << strTmp << " referenced in the zone " << pZone->GetID() << std::endl;
                    loadingLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                    return false;
                }
            }
            pXMLNodeListParkings->release();

            pZone->SetZonePosition(lstTuyauxZone);

            Liste_origines.push_back(pZone);
            Liste_destinations.push_back(pZone);

            if(!LoadCreationRepartitions(pZone, pXMLChildi, &loadingLogger))
            {
                return false;
            }

            // vÃ©rification de la somme Ã  un des coefficients de chaque motif intervenant dans la zone de terminaison
            if (!pZone->CheckCoeffs(loadingLogger))
            {
                return false;
            }
        }
    }

    // Chargement du paramÃ©trage des diffÃ©rents types de flottes
    for(size_t iFleet = 0; iFleet < m_LstFleets.size(); iFleet++)
    {
        if(!m_LstFleets[iFleet]->Load(pXMLReseau, loadingLogger))
        {
            return false;
        }
    }

    // SECTION TRAFIC / PARAMETRAGE_CAPTEUR
    DOMNode *pXMLCapteurs;
    pXMLNode = m_pXMLUtil->SelectSingleNode( "./PARAMETRAGE_CAPTEURS", pXMLNodeTrafic->getOwnerDocument(), (DOMElement*)pXMLNodeTrafic);

    if(pXMLNode)
    {
        GetXmlAttributeValue(pXMLNode, "periodeagregation", dbTmp, &loadingLogger);
        GetXmlAttributeValue(pXMLNode, "t0", dbTmp2, &loadingLogger);

        // Evolution nÂ°90 : pÃ©riodes d'agrÃ©gation spÃ©cifiques aux diffÃ©rents types de capteur
        double dbAgregEdie, dbAgregMFD, dbAgregLong, dbAgregBT;
        double dbT0Edie, dbT0MFD, dbT0Long, dbT0BT, dbT0Reservoir;
        if(!GetXmlAttributeValue(pXMLNode, "periodeagregationEdie", dbAgregEdie, &loadingLogger)) dbAgregEdie = dbTmp;
        if(!GetXmlAttributeValue(pXMLNode, "t0Edie", dbT0Edie, &loadingLogger)) dbT0Edie = dbTmp2;
        if(!GetXmlAttributeValue(pXMLNode, "periodeagregationLongitudinale", dbAgregLong, &loadingLogger)) dbAgregLong = dbTmp;
        if(!GetXmlAttributeValue(pXMLNode, "t0Longitudinal", dbT0Long, &loadingLogger)) dbT0Long = dbTmp2;
        if(!GetXmlAttributeValue(pXMLNode, "periodeagregationMFD", dbAgregMFD, &loadingLogger)) dbAgregMFD = dbTmp;
        if(!GetXmlAttributeValue(pXMLNode, "t0MFD", dbT0MFD, &loadingLogger)) dbT0MFD = dbTmp2;
        if(!GetXmlAttributeValue(pXMLNode, "periodeagregationBlueTooth", dbAgregBT, &loadingLogger)) dbAgregBT = dbTmp;
        if(!GetXmlAttributeValue(pXMLNode, "t0BlueTooth", dbT0BT, &loadingLogger)) dbT0BT = dbTmp2;
        if (!GetXmlAttributeValue(pXMLNode, "t0Reservoir", dbT0Reservoir, &loadingLogger)) dbT0Reservoir = dbTmp2;

        m_pGestionsCapteur = new SensorsManagers(this, dbTmp, dbTmp2, dbAgregEdie, dbT0Edie, dbAgregLong, dbT0Long, dbAgregMFD, dbT0MFD, dbAgregBT, dbT0BT, dbT0Reservoir);
        pXMLCapteurs = m_pXMLUtil->SelectSingleNode( "./CAPTEURS", pXMLNode->getOwnerDocument(), (DOMElement*)pXMLNode);

        // Lecture de la dÃ©finition des capteurs
        XMLSize_t counti = pXMLCapteurs->getChildNodes()->getLength();
        for(XMLSize_t i=0; i<counti;i++)
        {
            DOMNode * xmlnode = pXMLCapteurs->getChildNodes()->item(i);
            if (xmlnode->getNodeType() != DOMNode::ELEMENT_NODE) continue;

            // lecture des attributs du type de base des capteurs
            GetXmlAttributeValue(xmlnode, "id", strTmp, &loadingLogger);
            strcpy(strID, strTmp.c_str());

            GetXmlAttributeValue(xmlnode, "troncon", strTmp, &loadingLogger);
            Tuyau * pTuyau = GetLinkFromLabel(strTmp);

            // lecture des attributs spÃ©cifique Ã  chaque type de capteur
            std::string capteurNodeName = US(xmlnode->getNodeName());
            if(!capteurNodeName.compare("CAPTEUR"))
            {
                GetXmlAttributeValue(xmlnode, "position", dbTmp, &loadingLogger);

                bool bTmp = false;
                GetXmlAttributeValue(xmlnode, "position_relative", bTmp, &loadingLogger);
                if(bTmp)
                {
                    dbTmp *= pTuyau->GetLength();
                }

                // Capteurs uniquement sur les tronÃ§ons de type microscopique
                if(pTuyau->IsMacro())
                {
                    loadingLogger << Logger::Error << " ERROR : The sensor " << strID << " can't be placed on a macroscopic link : sensor ignored..." << endl;
                    loadingLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                }
                else
                {
                    // VÃ©rification de la position
                    if( pTuyau->GetLength() < dbTmp)
                    {
                        loadingLogger << Logger::Error << " ERROR : wrong position of sensor " << strID << " ( position > link's length ) : ignored sensor..." << endl;
                        loadingLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                    }
                    else
                    {
                        // Ajout Ã  la liste
                        m_pGestionsCapteur->AddCapteurPonctuel(strID, pTuyau, dbTmp);
                    }
                }
            }
            else if(!capteurNodeName.compare("CAPTEUR_LONGITUDINAL") || !capteurNodeName.compare("CAPTEUR_EDIE"))
            {
                if (GetSectionFromNode(strID, xmlnode, pTuyau, dbTmp, dbTmp2, &loadingLogger))
                {
                    if (!capteurNodeName.compare("CAPTEUR_LONGITUDINAL"))
                    {
                        m_pGestionsCapteur->AddCapteurLongitudinal(strID, pTuyau, dbTmp, dbTmp2);
                    }
                    else if (!capteurNodeName.compare("CAPTEUR_EDIE"))
                    {
                        // avertissement lorsque la longueur du capteur est supÃ©rieure au minimum des 1/kx
                        if (m_LstTypesVehicule.size() > 0)
                        {
                            double dbMinKxInv = DBL_MAX;
                            std::deque<TypeVehicule*>::iterator typeVehIter;
                            std::deque<TypeVehicule*>::iterator typeVehEnd = m_LstTypesVehicule.end();
                            for (typeVehIter = m_LstTypesVehicule.begin(); typeVehIter != typeVehEnd; typeVehIter++)
                            {
                                dbMinKxInv = min(dbMinKxInv, 1.0 / (*typeVehIter)->GetKx());
                            }

                            if (dbTmp2 - dbTmp < dbMinKxInv)
                            {
                                loadingLogger << Logger::Warning << " WARNING : the length of the Edie sensor " << strID << " should not be lower than the minimum 1/Kx (Kx = Max concentration) of the vehicle types..." << endl;
                                loadingLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                            }
                        }
                        m_pGestionsCapteur->AddCapteurMFD(strID, std::vector<Tuyau*>(1, pTuyau), std::vector<double>(1, dbTmp), std::vector<double>(1, dbTmp2),
                            std::vector<int>(1, EdieSensor::TT_Link), false);
                    }
                    else
                    {
                        // cas impossible
                        assert(false);
                    }
                }
            }
            else if(!capteurNodeName.compare("CAPTEUR_MFD"))
            {
                std::vector<Tuyau*> mfdLinks;
                std::vector<double> endPositions;
                std::vector<double> startPositions;
                std::vector<int> linkTypes;

                // rÃ©cupÃ©ration de la liste complÃ¨te des tuyaux constituant le capteur
                set<Tuyau*> lstTuyauxCapteur;
                DOMXPathResult * pXMLNodeListTroncons = m_pXMLUtil->SelectNodes(".//TRONCON", xmlnode->getOwnerDocument(), (DOMElement*)xmlnode);
                XMLSize_t nbTuyauxCapteur = pXMLNodeListTroncons->getSnapshotLength();
                for(XMLSize_t iTuyCapteur = 0; iTuyCapteur<nbTuyauxCapteur; iTuyCapteur++)
                {
                    pXMLNodeListTroncons->snapshotItem(iTuyCapteur);
                    DOMNode * pNodeCapteurTroncon = pXMLNodeListTroncons->getNodeValue();

                    GetXmlAttributeValue(pNodeCapteurTroncon, "id", strTmp, &loadingLogger);
                    lstTuyauxCapteur.insert(GetLinkFromLabel(strTmp));
                }
                pXMLNodeListTroncons->release();

                PrepareLinksForMFD(std::vector<Tuyau*>(lstTuyauxCapteur.begin(), lstTuyauxCapteur.end()), &loadingLogger, strID, mfdLinks, startPositions, endPositions, linkTypes);

                m_pGestionsCapteur->AddCapteurMFD(strID, mfdLinks, startPositions, endPositions, linkTypes, true);
            }
            else if (!capteurNodeName.compare("CAPTEUR_RESERVOIR"))
            {
                // rÃ©cupÃ©ration de la liste complÃ¨te des tuyaux constituant le capteur
                set<Tuyau*> lstTuyauxCapteur;
                DOMXPathResult * pXMLNodeListTroncons = m_pXMLUtil->SelectNodes(".//TRONCON", xmlnode->getOwnerDocument(), (DOMElement*)xmlnode);
                XMLSize_t nbTuyauxCapteur = pXMLNodeListTroncons->getSnapshotLength();
                for (XMLSize_t iTuyCapteur = 0; iTuyCapteur<nbTuyauxCapteur; iTuyCapteur++)
                {
                    pXMLNodeListTroncons->snapshotItem(iTuyCapteur);
                    DOMNode * pNodeCapteurTroncon = pXMLNodeListTroncons->getNodeValue();

                    GetXmlAttributeValue(pNodeCapteurTroncon, "id", strTmp, &loadingLogger);
                    lstTuyauxCapteur.insert(GetLinkFromLabel(strTmp));
                }
                pXMLNodeListTroncons->release();

                m_pGestionsCapteur->AddCapteurReservoir(strID, std::vector<Tuyau*>(lstTuyauxCapteur.begin(), lstTuyauxCapteur.end()));
            }
            else if(!capteurNodeName.compare("CAPTEUR_BLUETOOTH"))
            {
                GetXmlAttributeValue(xmlnode, "connexion", strTmp, &loadingLogger);
                Connexion * pNode = GetBrickFromID(strTmp);
                if(!pNode)
                {
                    pNode = GetConnectionFromID(strTmp, cTmp);
                }
                if(!pNode)
                {
                    loadingLogger << Logger::Warning << " WARNING : the node " << strTmp << " linked to the bluetooth sensor " << strID << " doesn't exist" << endl;
                    loadingLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                }
                else
                {
                    std::map<Tuyau*, double> mapPositions;
                    DOMXPathResult * pXMLNodeList = m_pXMLUtil->SelectNodes("./POSITION", xmlnode->getOwnerDocument(), (DOMElement*)xmlnode);
                    XMLSize_t countj = pXMLNodeList->getSnapshotLength();
                    for(XMLSize_t j=0; j<countj; j++)
                    {
                        pXMLNodeList->snapshotItem(j);
                        DOMNode * pNodeCapteurPosition = pXMLNodeList->getNodeValue();

                        GetXmlAttributeValue(pNodeCapteurPosition, "troncon", strTmp, &loadingLogger);
                        pTuyau = GetLinkFromLabel(strTmp);

                        GetXmlAttributeValue(pNodeCapteurPosition, "position", dbTmp, &loadingLogger);

                        bool bTmp = false;
                        GetXmlAttributeValue(pNodeCapteurPosition, "position_relative", bTmp, &loadingLogger);
                        if(bTmp)
                        {
                            dbTmp *= pTuyau->GetLength();
                        }

                        // VÃ©rification de la position
                        if( pTuyau->GetLength() < dbTmp)
                        {
                            loadingLogger << Logger::Error << " ERROR : wrong position of sensor " << strID << " ( position > link's length ) : using default position..." << endl;
                            loadingLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                        }
                        else
                        {
                            // Ajout Ã  la liste
                            mapPositions[pTuyau] = dbTmp;
                        }
                    }
                    pXMLNodeList->release();
                    m_pGestionsCapteur->AddCapteurBlueTooth(strID, pNode, mapPositions);
                }
            }
            else
            {
                loadingLogger << Logger::Error << " ERROR : unknown sensor type " << strID << " : ignoring sensor..." << endl;
                loadingLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
            }
        }
    }

    // SECTION TRAFIC / REGULATIONS
    pXMLNode = m_pXMLUtil->SelectSingleNode( "./REGULATIONS", pXMLNodeTrafic->getOwnerDocument(), (DOMElement*)pXMLNodeTrafic);

    if(pXMLNode)
    {
        // Lecture de la dÃ©finition des briques de rÃ©gulation
        XMLSize_t counti = pXMLNode->getChildNodes()->getLength();
        for(XMLSize_t i=0; i<counti;i++)
        {
            DOMNode *pXMLRegulationBrique = pXMLNode->getChildNodes()->item(i);
            if (pXMLRegulationBrique->getNodeType() != DOMNode::ELEMENT_NODE) continue;

            // Lecture des paramÃ¨tres et ajout de la brique correspondante
            m_pRegulationModule->addBrique(pXMLRegulationBrique);
        }
    }

    // SECTION RESEAU / CONTROLEURS_DE_FEUX
    DOMXPathResult *pXMLNodeList;
    ControleurDeFeux *pCtrDeFeux;
    std::string sCnx;
    Repartiteur *pRep;
    pXMLNode = m_pXMLUtil->SelectSingleNode( "./CONTROLEURS_DE_FEUX", pXMLNodeTrafic->getOwnerDocument(), (DOMElement*)pXMLNodeTrafic);

    if(pXMLNode)
    {
        // Lecture de la dÃ©finition des contrÃ´leurs de feux
        XMLSize_t counti = pXMLNode->getChildNodes()->getLength();
        for(XMLSize_t i=0; i<counti;i++)
        {
            DOMNode *pXMLCtrlDeFeux = pXMLNode->getChildNodes()->item(i);
            if (pXMLCtrlDeFeux->getNodeType() != DOMNode::ELEMENT_NODE) continue;

            // Lecture de l'identifiant du contrÃ´leur
            GetXmlAttributeValue(pXMLCtrlDeFeux, "id", strTmp, &loadingLogger);
            strcpy(strID, strTmp.c_str());

            // On ignore les controleurs de feux sans plan de feu (ca plante ensuite en cours de simulation)
            DOMNode *pXMLPlansDeFeux = m_pXMLUtil->SelectSingleNode("./PLANS_DE_FEUX", pXMLCtrlDeFeux->getOwnerDocument(), (DOMElement*)pXMLCtrlDeFeux);
            if (pXMLPlansDeFeux && pXMLPlansDeFeux->getChildNodes()->getLength() > 0)
            {
                // Lecture de la durÃ©e de rouge de dÃ©gagement
                double dbDureeRougeDegagement;
                GetXmlAttributeValue(pXMLCtrlDeFeux, "duree_rouge_degagement", dbDureeRougeDegagement, &loadingLogger);

                // Lecture de la durÃ©e de vert minimum
                double dbDureeVertMin;
                GetXmlAttributeValue(pXMLCtrlDeFeux, "duree_vert_min", dbDureeVertMin, &loadingLogger);

                // CrÃ©ation du contrÃ´leur de feux
                pCtrDeFeux = new ControleurDeFeux(this, strID, dbDureeRougeDegagement, dbDureeVertMin);

                Liste_ControleursDeFeux.push_back(pCtrDeFeux);

                // Recherche des rÃ©partiteurs et/ou des carrefours Ã  feux qui ont pour controleur de feux celui-ci
                // RÃ©partiteurs:
                sPath = "CONNEXIONS/REPARTITEURS/REPARTITEUR[@controleur_de_feux=\"" + strTmp + "\"]";
                pXMLNodeList = m_pXMLUtil->SelectNodes(sPath, pXMLReseau->getOwnerDocument(), (DOMElement*)pXMLReseau);
                XMLSize_t countj = pXMLNodeList->getSnapshotLength();
                for (XMLSize_t j = 0; j < countj; j++)
                {
                    pXMLNodeList->snapshotItem(j);
                    GetXmlAttributeValue(pXMLNodeList->getNodeValue(), "id", sCnx, &loadingLogger);
                    pRep = (Repartiteur*)GetConnectionFromID(sCnx, cType);

                    if (pRep)
                        pRep->SetCtrlDeFeux(pCtrDeFeux);
                }
                pXMLNodeList->release();

                // Carrefours Ã  feux
                sPath = "CONNEXIONS/CARREFOURSAFEUX/CARREFOURAFEUX[@controleur_de_feux=\"" + strTmp + "\"]";
                pXMLNodeList = m_pXMLUtil->SelectNodes(sPath, pXMLReseau->getOwnerDocument(), (DOMElement*)pXMLReseau);
                countj = pXMLNodeList->getSnapshotLength();
                for (XMLSize_t j = 0; j < countj; j++)
                {
                    pXMLNodeList->snapshotItem(j);
                    GetXmlAttributeValue(pXMLNodeList->getNodeValue(), "id", sCnx, &loadingLogger);
                    pCAF = (CarrefourAFeuxEx*)GetBrickFromID(sCnx);

                    if (pCAF)
                        pCAF->SetCtrlDeFeux(pCtrDeFeux);
                }
                pXMLNodeList->release();

                // Lecture des plans de feux
                if (pXMLPlansDeFeux)
                {
                    XMLSize_t countj = pXMLPlansDeFeux->getChildNodes()->getLength();
                    for (XMLSize_t j = 0; j < countj; j++)
                    {
                        DOMNode * xmlnode = pXMLPlansDeFeux->getChildNodes()->item(j);
                        if (xmlnode->getNodeType() != DOMNode::ELEMENT_NODE) continue;

                        // ID
                        GetXmlAttributeValue(xmlnode, "id", strTmp, &loadingLogger);
                        strcpy(strID, strTmp.c_str());

                        // Heure de dÃ©but
                        STime dtDebut;
                        SDateTime dt;
                        GetXmlAttributeValue(xmlnode, "debut", dt, &loadingLogger);
                        dtDebut = dt.ToTime();

                        PlanDeFeux *pPlanDeFeux = new PlanDeFeux(strID, dtDebut);

                        // Ajout au contrÃ´leur de feux
                        pCtrDeFeux->GetLstPlanDeFeuxInit()->AddVariation(dtDebut, pPlanDeFeux);

                        // Chargement des donnÃ©es du plan de feux
                        LoadDataPlanDeFeux(pCtrDeFeux, pPlanDeFeux, xmlnode, &loadingLogger);
                    }
                }
                // Lecture des lignes guidÃ©es prioritaires
                DOMNode *pXMLLGPs = m_pXMLUtil->SelectSingleNode("./LIGNES_GUIDEES_PRIORITAIRES", pXMLCtrlDeFeux->getOwnerDocument(), (DOMElement*)pXMLCtrlDeFeux);
                if (pXMLLGPs)
                {
                    XMLSize_t countj = pXMLLGPs->getChildNodes()->getLength();
                    for (XMLSize_t j = 0; j < countj; j++)
                    {
                        LGP*         plgp = new LGP();
                        Trip        *pLigneBus = NULL;
                        std::string sT;

                        DOMNode *pXMLLGP = pXMLLGPs->getChildNodes()->item(j);
                        if (pXMLLGP->getNodeType() != DOMNode::ELEMENT_NODE) continue;

                        // Lecture des lignes de transport guidÃ© concernÃ©
                        GetXmlAttributeValue(pXMLLGP, "lignes", strTmp, &loadingLogger);
                        std::deque<std::string> split = SystemUtil::split(strTmp, ' ');
                        for (size_t k = 0; k < split.size(); k++)
                        {
                            strcpy(strID, split.at(k).c_str());
                            pLigneBus = GetPublicTransportFleet()->GetTrip(split.at(k));

                            if (!pLigneBus)
                            {
                                loadingLogger << Logger::Error << "ERROR : the public transport line " << strID << " defined as a prioritary line for traffic light controller " << pCtrDeFeux->GetLabel() << " doesn't exist." << endl;
                                loadingLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                                return FALSE;
                            }
                            plgp->LstLTG.push_back(pLigneBus);
                        }

                        // Lecture des caractÃ©ristiques du tronÃ§on amont
                        GetXmlAttributeValue(pXMLLGP, "position_capteur_amont", plgp->dbPosCptAm, &loadingLogger);
                        GetXmlAttributeValue(pXMLLGP, "troncon_capteur_amont", sT, &loadingLogger);
                        plgp->pTCptAm = GetLinkFromLabel(sT);
                        if (plgp->dbPosCptAm > plgp->pTCptAm->GetLength())
                        {
                            loadingLogger << Logger::Error << "ERROR : the upstream sensor's position of one of the prioritary lines of the traffic light controller " << pCtrDeFeux->GetLabel() << " on link " << plgp->pTCptAm->GetLabel() << " is incorrect." << endl;
                            loadingLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                            return FALSE;
                        }

                        // Lecture des caractÃ©ristiques de la zone d'entrÃ©e
                        GetXmlAttributeValue(pXMLLGP, "position_entree_zone", plgp->dbPosEntreeZone, &loadingLogger);
                        GetXmlAttributeValue(pXMLLGP, "troncon_entree_zone", sT, &loadingLogger);
                        plgp->pTEntreeZone = GetLinkFromLabel(sT);
                        if (plgp->dbPosEntreeZone > plgp->pTEntreeZone->GetLength())
                        {
                            loadingLogger << Logger::Error << "ERROR : the upstream sensor's position of one of the prioritary lines of the traffic light controller " << pCtrDeFeux->GetLabel() << " on link " << plgp->pTCptAm->GetLabel() << " is incorrect." << endl;
                            loadingLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                            return FALSE;
                        }

                        // Lecture des caractÃ©ristiques du tronÃ§on aval
                        GetXmlAttributeValue(pXMLLGP, "position_capteur_aval", plgp->dbPosCptAv, &loadingLogger);
                        GetXmlAttributeValue(pXMLLGP, "troncon_capteur_aval", sT, &loadingLogger);
                        plgp->pTCptAv = GetLinkFromLabel(sT);
                        if (plgp->dbPosCptAv > plgp->pTCptAv->GetLength())
                        {
                            loadingLogger << Logger::Error << "ERROR : the downstream sensor's position of one of the prioritary lines of the traffic light controller " << pCtrDeFeux->GetLabel() << " on link " << plgp->pTCptAv->GetLabel() << " is incorrect." << endl;
                            loadingLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                            return FALSE;
                        }

                        // Type de prioritÃ© (totale ou partagÃ©e)
                        GetXmlAttributeValue(pXMLLGP, "priorite_totale", plgp->bPrioriteTotale, &loadingLogger);

                        if (!plgp->bPrioriteTotale)
                        {
                            GetXmlAttributeValue(pXMLLGP, "sequence_partagee", plgp->nSeqPartagee, &loadingLogger);
                            plgp->nSeqPartagee = plgp->nSeqPartagee - 1;
                        }

                        pCtrDeFeux->AddLGP(plgp);
                    }
                }
            }
            else
            {
                loadingLogger << Logger::Warning << "WARNING : the traffic light controller " << strID << " has no traffic light plan and is ignored for the simulation" << endl;
                loadingLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
            }
        }
    }

    // Fin d'initialisation de divers objets...
    for(size_t i=0; i<Liste_giratoires.size(); i++)
        Liste_giratoires[i]->FinInit();

    for(size_t i=0; i<Liste_carrefoursAFeux.size(); i++)
        Liste_carrefoursAFeux[i]->FinInit(&loadingLogger);

    std::map<string, Repartiteur*>::const_iterator iterRep;
    for(iterRep = Liste_repartiteurs.begin(); iterRep != Liste_repartiteurs.end(); ++iterRep)
        iterRep->second->FinInit(&loadingLogger);

    // Discretisation Sirane Ã  la fin du chargement pour que les briqus soient complÃ©tement dÃ©finies (calcul des barycentres)
    if( IsSimuSirane() )
    {
        for(size_t iTuyau = 0; iTuyau < m_LstTuyaux.size(); iTuyau++)
        {
            Tuyau * pTuyau = m_LstTuyaux[iTuyau];
            if(pTuyau->GetBriqueParente() == NULL)
            {
                TuyauMicro * pTuyauMicro = dynamic_cast<TuyauMicro*>(pTuyau);
                if(pTuyauMicro)
                {
                    pTuyauMicro->DiscretisationSirane();
                }
            }
        }
    }

    // rmq. : pour Ã©viter un recalcul inutile de l'affectation au premier instant.
    m_LstItiChangeInstants.erase(GetTimeStep());

    // Chargement des vÃ©hicules dÃ©jÃ  prÃ©sent sur le rÃ©seau
    LoadInitVehs(m_pXMLUtil->SelectSingleNode( "./INIT", pXMLReseau->getOwnerDocument(), (DOMElement*)pXMLReseau), loadingLogger);

#ifdef USE_SYMUCOM
    DOMNode *pXMLNodeSymuCom = m_pXMLUtil->SelectSingleNode("/ROOT_SYMUBRUIT/SYMUCOM_ELEMENTS", m_XMLDocData);
    if (pXMLNodeSymuCom)
    {
        // Prepare simulation :
        SymuCom::Graph* pGraph = new SymuCom::Graph();
        SymuCom::CommunicationRunner* pCommunicationRunner = new SymuCom::CommunicationRunner(pGraph);
        SymuCom::Point placeHolderPoint;
        m_pSymucomSimulator = new Simulator(this, "Simulator", "Simulator", placeHolderPoint, pGraph, pCommunicationRunner, true, true, 10.0, 0.0, 0.0);

        std::string strFile = GetPrefixOutputFiles() + "_SymuCom.xml";
        m_pSymucomSimulator->InitSymuComXML(strFile);

        if (!m_pSymucomSimulator->LoadFromXML(m_XMLDocData, pXMLNodeSymuCom)){
            loadingLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
            return false;
        }
    }
#endif // USE_SYMUCOM

    loadingLogger<<"*** The network loaded successfully ***"<<std::endl;

    // ANOMALIE nÂ°80 : la fonction GenerateAssignmentNetwork Ã©tait appelÃ©e inutilement aprÃ¨s chaque reset, ce qui crÃ©ait des doublons
    // sources de bugs dans le rÃ©seau d'affectation. On procÃ¨de Ã  prÃ©sent Ã  sa crÃ©ation ici, dÃ¨s la fin du chargement du rÃ©seau physique

    // Si comportement de type itinÃ©raire, calcul des itinÃ©raires des vÃ©hicules
    // CrÃ©ation de la liste des connexions et des tronÃ§ons du graphe d'affectation
    // Le rÃ©seau crÃ©Ã© peut Ãªtre diffÃ©rent du rÃ©seau de dÃ©part notamment si il contient des CAF
    // Ceux-ci sont dÃ©crit de maniÃ¨re particuliÃ¨re
    // CrÃ©ation du rÃ©seau d'affectation

    GenerateAssignmentNetwork();

    // Initialisation des routes prÃ©dÃ©finies Ã  partir des noeuds
    pXMLNode = m_pXMLUtil->SelectSingleNode("./CONNEXIONS_INTERNES", pXMLNodeTrafic->getOwnerDocument(), (DOMElement*)pXMLNodeTrafic);

    if (pXMLNode)
    {
        // Lecture de la dÃ©finition des briques de rÃ©gulation
        XMLSize_t counti = pXMLNode->getChildNodes()->getLength();
        for (XMLSize_t i = 0; i<counti; i++)
        {
            DOMNode *pXMLConnexionInterne = pXMLNode->getChildNodes()->item(i);
            if (pXMLConnexionInterne->getNodeType() != DOMNode::ELEMENT_NODE) continue;

            GetXmlAttributeValue(pXMLConnexionInterne, "id", strTmp, &loadingLogger);

            Connexion * pConForPredefRoutes = GetConnectionFromID(strTmp, cTmp);
            if (!pConForPredefRoutes)
            {
                pConForPredefRoutes = GetBrickFromID(strTmp);
            }

            if (pConForPredefRoutes)
            {
                // Test du flag routes_predefinies
                bool bPredefinedRoutes = false;
                GetXmlAttributeValue(pXMLConnexionInterne, "routes_predefinies", bPredefinedRoutes, &loadingLogger);

                if (bPredefinedRoutes)
                {
                    m_bUseMapRouteFromNodes = true;
                    // Parcours de l'ensemble des routes pour rÃ©cupÃ©rer celles correspondantes Ã  ce noeud :
                    for (std::map<std::string, std::pair<std::vector<Tuyau *>, Connexion*> >::iterator iterRoutes = m_routes.begin(); iterRoutes != m_routes.end(); ++iterRoutes)
                    {
                        if (!iterRoutes->second.first.empty())
                        {
                            Tuyau * pFirstLink = iterRoutes->second.first.front();
                            if (pFirstLink->GetCnxAssAm() == pConForPredefRoutes)
                            {
                                // RÃ©cupÃ©ration de la ou des destinations
                                std::set<SymuViaTripNode*> availableDestinations;
                                Tuyau * pLastLink = iterRoutes->second.first.back();
                                SymuViaTripNode * pRouteDestination = GetDestinationFromID(pLastLink->GetCnxAssAv()->GetID(), cTmp);
                                if (pRouteDestination)
                                {
                                    availableDestinations.insert(pRouteDestination);
                                }
                                // a priori si le dernier tronÃ§on va vers une sortie, pas la peine de regarder les zones, sauf si on dÃ©cide (cf. commentaire
                                // plus bas) de considÃ©rer la route comme possible vers une zone si la route rentre dans la zone...
                                else
                                {
                                    // Il doit s'agir d'une ou plusieurs zones ...
                                    for (size_t iZone = 0; iZone < Liste_zones.size(); iZone++)
                                    {
                                        ZoneDeTerminaison * pZone = Liste_zones[iZone];
                                        // rmq. on pourrait utiliser Ã©galement les chemins qui arrivent dans la zone plutÃ´t qu'au bord de la zone...
                                        if (pZone->GetInputConnexion()->IsTuyauAmont(pLastLink))
                                        {
                                            availableDestinations.insert(pZone);
                                        }
                                    }
                                }

                                // Cas de la destination tuyau
                                m_mapRouteFromNodesToLink[pConForPredefRoutes][pLastLink].push_back(&iterRoutes->second.first);

                                // Cas de la destination noeud interne
                                if (!pRouteDestination)
                                {
                                    Connexion * pInternalConDest = GetConnectionFromID(pLastLink->GetCnxAssAv()->GetID(), cTmp);
                                    if (!pInternalConDest)
                                    {
                                        pInternalConDest = GetBrickFromID(pLastLink->GetCnxAssAv()->GetID());
                                    }
                                    if (pInternalConDest)
                                    {
                                        m_mapRouteFromNodesToInternalNode[pConForPredefRoutes][pInternalConDest].push_back(&iterRoutes->second.first);
                                    }
                                }

                                for (std::set<SymuViaTripNode*>::const_iterator iterDestForRoute = availableDestinations.begin(); iterDestForRoute != availableDestinations.end(); ++iterDestForRoute)
                                {
                                    m_mapRouteFromNodes[pConForPredefRoutes][*iterDestForRoute].push_back(&iterRoutes->second.first);
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    loadingLogger << Logger::Info << "Network loading duration : " <<  ((float)(clock() - startLoadTime))/CLOCKS_PER_SEC << " s" << std::endl << std::endl;

    return true;
}

//=================================================================
    bool  Reseau::HasVehicule
//----------------------------------------------------------------
// Fonction  : Retourne vrai si il y a au moins un vÃ©hicule sur la voie
// Remarque  :
// Version du: 06/07/2009
// Historique: 06/07/2009 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//=================================================================
(
    VoieMicro* pV
)
{
    std::vector<boost::shared_ptr<Vehicule>>::iterator itVeh;

    for(itVeh = m_LstVehicles.begin(); itVeh != m_LstVehicles.end(); itVeh++)
    {
        if( (*itVeh)->GetVoie(0) == pV )
            return true;
    }

    return false;
}

//=================================================================
    boost::shared_ptr<Vehicule>  Reseau::GetVehiculeAmont
//----------------------------------------------------------------
// Fonction  : Fonction rÃ©cursive de recherche du premier vÃ©hicule
//             sur un tronÃ§on et tous ses tronÃ§ons en amont
//             jusqu'Ã  une distance donnÃ©e
// Remarque  : Cette mÃ©thode est valable uniquement pour les tronÃ§ons Ã  une voie
// Version du: 22/11/2007
// Historique: 22/11/2007 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//=================================================================
(
    TuyauMicro* pTAv,       // TronÃ§on de dÃ©part sur lequel on commense la recherche
    double  dbDst,           // Distance maximale pour limiter la recherche en amont
    double  dbDstCum,        // Distance dÃ©jÃ  parcourue (=0 lors de l'appel initial)
    double  dbInstant
)
{
    boost::shared_ptr<Vehicule> pVeh;
    ConnectionPonctuel  *pCnxAmont;
    TuyauMicro  *pTAm;

    // Recherche sur le tronÃ§on initial
    pVeh = GetNearAmontVehicule( (VoieMicro*)pTAv->GetLstLanes()[0], pTAv->GetLength());
    if(pVeh)
        return pVeh;                    // un vÃ©hicule trouvÃ©

    if( dbDst < dbDstCum )   // Distance atteinte, pas de vÃ©hicule
        return boost::shared_ptr<Vehicule>();

    if( pTAv->get_Type_amont()=='E' || pTAv->get_Type_amont()=='S' || pTAv->get_Type_amont() == 'P' )
        return boost::shared_ptr<Vehicule>();                    // Pas d'autre Ã©lÃ©ment amont donc pas de vÃ©hicule trouvÃ©

    pCnxAmont = pTAv->getConnectionAmont();

    std::deque<Tuyau*>::iterator itT;
    for( itT = pCnxAmont->m_LstTuyAm.begin(); itT != pCnxAmont->m_LstTuyAm.end(); itT++)
    {
        pTAm = (TuyauMicro*)(*itT);
        pVeh = GetVehiculeAmont(pTAm, dbDst, dbDstCum + pTAv->GetLength(), dbInstant);

        if(pVeh)
            if(dbDst > dbDstCum + pTAv->GetLength() - pVeh->GetPos(1) )
                if(pVeh->CalculNextVoie( pVeh->GetVoie(1), dbInstant ) == pTAv->GetLstLanes()[0] ) // On vÃ©rifie que sa voie suivante est bien celle qui nous intÃ©resse
                    return pVeh;    // un vÃ©hicule trouvÃ© et il est bien dans la zone d'influence, on sort de la boucle et de la fonction rÃ©cursive
    }
    return boost::shared_ptr<Vehicule>();    // Aucun vÃ©hicule de trouvÃ©
}


//=================================================================
    double Reseau::GetMaxVitMax
//----------------------------------------------------------------
// Fonction  : Retourne le maximum des vitesses libres pour l'ensemble
//             des types de vÃ©hicule
// Remarque  :
// Version du: 22/05/2008
// Historique: 22/05/2008 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//=================================================================
(
)
{
    double dbMax;

    dbMax = 0;
    for(int i=0;i<(int)m_LstTypesVehicule.size();i++)
    {
        if( m_LstTypesVehicule[i]->GetVx() > dbMax)
            dbMax = m_LstTypesVehicule[i]->GetVx();

        if( m_LstTypesVehicule[i]->IsVxDistr() && m_LstTypesVehicule[i]->GetVxMax() > dbMax)
            dbMax = m_LstTypesVehicule[i]->GetVxMax();
    }
    return dbMax;
}

//=================================================================
    double Reseau::GetMinKv
//----------------------------------------------------------------
// Fonction  : Retourne le minimum sur tous les types de vÃ©hicule de Kv
// Remarque  :
// Version du: 23/11/2007
// Historique: 23/11/2007 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//=================================================================
(
)
{
    double dbMinKv, dbTmp;

    dbMinKv = 999;
    for(int i=0;i<(int)m_LstTypesVehicule.size();i++)
    {
        dbTmp = - m_LstTypesVehicule[i]->GetKx() * m_LstTypesVehicule[i]->GetW() / ( m_LstTypesVehicule[i]->GetVx() -  m_LstTypesVehicule[i]->GetW());

        if( dbTmp < dbMinKv)
            dbMinKv = dbTmp;
    }
    return dbMinKv;
}

//=================================================================
    double Reseau::GetMainKx
//----------------------------------------------------------------
// Fonction  : Retourne le Kx du type de vÃ©hicule principal (le
//             premier)
// Remarque  :
// Version du: 29/03/2017
// Historique: 29/03/2017 (O.Tonck - Ipsis)
//             CrÃ©ation
//=================================================================
(
)
{
    return m_LstTypesVehicule.empty() ? 0 : m_LstTypesVehicule.front()->GetKx();
}

//=================================================================
    Tuyau* Reseau::GetTuyauReliant
//----------------------------------------------------------------
// Fonction  : Retourne si il existe le tuyau permettant de relier
//             le tuyau amont au tuyau aval
// Remarque  :
// Version du: 11/03/2007
// Historique: 11/03/2007 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//=================================================================
(
    Tuyau *pTAm,
    Tuyau *pTAv
)
{
    if( pTAm->get_Type_aval() == 'E' || pTAm->get_Type_aval() == 'S' || pTAm->get_Type_aval() == 'P')
        return NULL;

    if( pTAv->get_Type_amont() == 'E' || pTAv->get_Type_amont() == 'S' || pTAm->get_Type_amont() == 'P')
        return NULL;

    std::deque<Tuyau*>::iterator itTav, itTam;
    for(itTav = pTAm->getConnectionAval()->m_LstTuyAv.begin(); itTav != pTAm->getConnectionAval()->m_LstTuyAv.end(); itTav++)
    {
        for(itTam = pTAv->getConnectionAmont()->m_LstTuyAm.begin(); itTam !=  pTAv->getConnectionAmont()->m_LstTuyAm.end(); itTam++)
        {
            if( (*itTav ) == (*itTam) )
                return (*itTav );
        }
    }
    return NULL;
}

//=================================================================
    bool Reseau::UpdateReseau
//----------------------------------------------------------------
// Fonction  : Chargement du document XML de modification
// Remarque  : Suppression des variantes chargÃ©es et chargement des
//			   nouvelles donnÃ©es
// Version du: 20/06/2008
// Historique: 20/06/2008 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//=================================================================
(
 const std::string & strXMLFile
)
{
    DOMNode				*pXMLRoot;
    DOMNode				*pXMLReseau;
    DOMNode				*pXMLTrafic;
    DOMNode             *pXMLExtremite;
    DOMNode             *pXMLRepartiteur;
    DOMNode             *pXMLDemandes;
    DOMNode             *pXMLCapacites;

    std::string         sXML;
    XERCES_CPP_NAMESPACE::DOMDocument    *pXMLDocUpdate;
    std::string		    sTmp;
    std::string         strTmp;
    double              dbDuree;
    char                cType;
    SymuViaTripNode     *pE;
    SymuViaTripNode     *pS;
    Repartiteur         *pR;
    double              dbTmp;
    double              dbTmp2;

    // Initialisation
    sXML = strXMLFile;

    // VÃ©rification de l'existence du fichier XML de modification
    if (!SystemUtil::FileExists(sXML))
    {
        std::string sMsg;
        sMsg = "The file ";
        sMsg += sXML + " doesn't exist !";
#ifdef WIN32
        ::MessageBox(NULL, SystemUtil::ToWString(sMsg).c_str(), L"SymuVia Error", 0);
#else
        std::cerr << sMsg << std::endl;
#endif
        return false;
    }

    // CrÃ©ation et validation du document XML
    std::string sSchema = SystemUtil::GetFilePath("update.xsd");
    std::string firstError;

    try
    {
        pXMLDocUpdate = m_pXMLUtil->LoadDocument(sXML, sSchema, NULL, firstError);
    }
    catch (const XMLException& e)
    {
        std::string s = US(e.getMessage());
#ifdef WIN32
        ::MessageBox(NULL,SystemUtil::ToWString("Error validating against the schema 'update.xsd' loading input file : " + s).c_str(), L"SymuVia - Network update", 0);
#else
        std::cerr << "Error validating against the schema 'update.xsd' loading input file : " << s << std::endl;
#endif

        return false;
    }
    catch (const SAXException& e)
    {
        std::string s = US(e.getMessage());
#ifdef WIN32
        ::MessageBox(NULL,SystemUtil::ToWString("Error validating against the schema 'update.xsd' loading input file : " + s).c_str(), L"SymuVia - Network update", 0);
#else
        std::cerr << "Error validating against the schema 'update.xsd' loading input file : " << s << std::endl;
#endif

        return false;
    }
    catch (const DOMException& e)
    {
        std::string s = US(e.getMessage());
#ifdef WIN32
        ::MessageBox(NULL,SystemUtil::ToWString("Error validating against the schema 'update.xsd' loading input file : " + s).c_str(), L"SymuVia - Network update", 0);
#else
        std::cerr << "Error validating against the schema 'update.xsd' loading input file : " << s << std::endl;
#endif

        return false;
    }
    catch (...)
    {
#ifdef WIN32
        ::MessageBox(NULL,L"Error validating against the schema 'update.xsd' loading input file", L"SymuVia - Network update", 0);
#else
        std::cerr << "Error validating against the schema 'update.xsd' loading input file" << std::endl;
#endif

        return false;
    }

    if (pXMLDocUpdate == NULL)
    {
#ifdef WIN32
        ::MessageBox(NULL,SystemUtil::ToWString("Error validating against the schema 'update.xsd' loading input file : " + firstError).c_str(), L"Network update", 0);
#else
        std::cerr << "Error validating against the schema 'update.xsd' loading input file : " + firstError << std::endl;
#endif

        return false;
    }

    // Lecture de l'arbre

    // Noeud root
    pXMLRoot = pXMLDocUpdate->getDocumentElement();

    // Noeud reseau
    pXMLReseau = m_pXMLUtil->SelectSingleNode("RESEAUX/RESEAU[@id=\"" + m_ReseauID + "\"]", pXMLDocUpdate, (DOMElement*)pXMLRoot);

    //----------------------
    // Noeud troncons
    //----------------------
    Tuyau	*pT;
    DOMNode *pXMLTroncons;
    DOMNode *pXMLTroncon;

    pXMLTroncons = m_pXMLUtil->SelectSingleNode( "TRONCONS", pXMLDocUpdate, (DOMElement*)pXMLReseau);
    if(pXMLTroncons)
    {
        XMLSize_t counti = pXMLTroncons->getChildNodes()->getLength();
        for(XMLSize_t i=0; i<counti;i++)
        {
            pXMLTroncon = pXMLTroncons->getChildNodes()->item(i);
            if (pXMLTroncon->getNodeType() != DOMNode::ELEMENT_NODE)
                continue;

            // id
            GetXmlAttributeValue(pXMLTroncon, "id", sTmp, m_pLogger);
            pT = this->GetLinkFromLabel( sTmp );

            if(pT)
            {
                if( pT->IsMicro() )
                {
                    bool bTmp;
                    GetXmlAttributeValue(pXMLTroncon, "chgt_voie_droite", bTmp, m_pLogger);
                    TuyauMicro* pTM = (TuyauMicro*)pT;
                    if( pTM->IsChtVoieVersDroiteAutorise()!=bTmp )
                        pTM->SetChtVoieVersDroiteAutorise(bTmp);

                    // Chargement des voies rÃ©duites
                    DOMNode *pXMLVoiesReduites = m_pXMLUtil->SelectSingleNode("VOIES_REDUITES", pXMLTroncon->getOwnerDocument(), (DOMElement*)pXMLTroncon);

                    if(pXMLVoiesReduites)
                    {
                        XMLSize_t countj = pXMLVoiesReduites->getChildNodes()->getLength();
                        for(XMLSize_t j=0; j<countj; j++)
                        {
                            DOMNode* xmlChildj = pXMLVoiesReduites->getChildNodes()->item(j);
                            if (xmlChildj->getNodeType() != DOMNode::ELEMENT_NODE) continue;

                            int nVoie;
                            GetXmlAttributeValue(xmlChildj, "numvoie", nVoie, m_pLogger);

                            // VÃ©rification de la cohÃ©rence du numÃ©ro de voie rÃ©duite
                            if( nVoie > pTM->getNbVoiesDis() )
                            {
                                return false;
                            }

                            // rÃ©cupÃ©ration des exceptions de types de vÃ©hicules
                            std::vector<TypeVehicule*> lstExceptionTypesVeh;
                            if(GetXmlAttributeValue(xmlChildj, "exclusion_types_vehicules", strTmp, m_pLogger))
                            {
                                std::deque<std::string> split = SystemUtil::split(strTmp, ' ');
                                for(size_t j=0; j< split.size(); j++)
                                {
                                    TypeVehicule * pTV = GetVehicleTypeFromID(split.at(j));
                                    if(pTV)
                                        lstExceptionTypesVeh.push_back(pTV);
                                    else
                                    {
                                        return false;
                                    }
                                }
                            }

                            bool bActive = true;
                            GetXmlAttributeValue(xmlChildj, "active", bActive, m_pLogger);
                            if(!bActive)
                            {
                                ((VoieMicro*)pTM->GetLstLanes()[nVoie-1])->SetNotChgtVoieObligatoire();
                            }
                            else
                            {
                                ((VoieMicro*)pTM->GetLstLanes()[nVoie-1])->SetChgtVoieObligatoire(lstExceptionTypesVeh);
                            }

                        }
                    }
                    else
                    {
                        for(int i=0; i<pTM->getNbVoiesDis(); i++)
                            ((VoieMicro*)pTM->GetLstLanes()[i])->SetNotChgtVoieObligatoire();
                    }
                }

                // Chargement des voies rÃ©servÃ©es et terre-pleins
                DOMNode * XMLVoiesReservees = m_pXMLUtil->SelectSingleNode("./VOIES_RESERVEES", pXMLTroncon->getOwnerDocument(), (DOMElement*)pXMLTroncon);
                if(XMLVoiesReservees)
                {
                    // Lecture des voies rÃ©servÃ©es
                    DOMNode * pNodeVoieReservee;
                    DOMXPathResult * pXMLNodeList = m_pXMLUtil->SelectNodes("./VOIE_RESERVEE", XMLVoiesReservees->getOwnerDocument(), (DOMElement*)XMLVoiesReservees);
                    XMLSize_t countj = pXMLNodeList->getSnapshotLength();
                    std::map<int, double> mapDureesVoiesReservees;
                    map<int, vector<PlageTemporelle*> > plagesParVoie;
                    for(XMLSize_t j=0; j<countj; j++)
                    {
                        pXMLNodeList->snapshotItem(j);
                        pNodeVoieReservee = pXMLNodeList->getNodeValue();
                        int num_voie;

                        GetXmlAttributeValue(pNodeVoieReservee, "num_voie", num_voie, m_pLogger);
                        vector<PlageTemporelle*> plages;
                        if(!GetXmlDuree(pNodeVoieReservee,this,dbDuree,plages, m_pLogger))
                        {
                            m_pXMLUtil->CleanLoadDocument(pXMLDocUpdate);
                            return false;
                        }
                        plagesParVoie[num_voie].insert(plagesParVoie[num_voie].end(), plages.begin(), plages.end());
                        GetXmlAttributeValue(pNodeVoieReservee, "id_typesvehicules", strTmp, m_pLogger);
                        bool bActive = true;
                        GetXmlAttributeValue(pNodeVoieReservee, "active", bActive, m_pLogger);
                        // construction de tous les types autres que ceux de la liste pour interdiction
                        std::deque<std::string> split = SystemUtil::split(strTmp, ' ');
                        std::vector<TypeVehicule*> pVectTypesInterdits;
                        for(size_t typeIdx = 0; typeIdx < m_LstTypesVehicule.size(); typeIdx++)
                        {
                            bool isVehiculeAutorise = false;
                            for(size_t splitIdx = 0; splitIdx<split.size(); splitIdx++)
                            {
                                if(GetVehicleTypeFromID(split[splitIdx]) == m_LstTypesVehicule[typeIdx])
                                {
                                    isVehiculeAutorise = true;
                                }
                            }
                            if(!isVehiculeAutorise)
                            {
                                pVectTypesInterdits.push_back(m_LstTypesVehicule[typeIdx]);
                            }
                        }
                        if(m_bTypeProfil)
                        {
                            if(mapDureesVoiesReservees.find(num_voie-1) != mapDureesVoiesReservees.end())
                            {
                                mapDureesVoiesReservees[num_voie-1] += dbDuree;
                            }
                            else
                            {
                                mapDureesVoiesReservees[num_voie-1] = dbDuree;
                            }
                            pT->AddVoieReserveeByTypeVeh(pVectTypesInterdits, mapDureesVoiesReservees[num_voie-1]-dbDuree, dbDuree, NULL, m_dbDureeSimu, num_voie-1, bActive);
                        }
                        else
                        {
                            if(plages.size() == 0)
                            {
                                // utilisation de la valeur par dÃ©faut dbDuree
                                pT->AddVoieReserveeByTypeVeh(pVectTypesInterdits, 0, dbDuree, NULL, m_dbDureeSimu, num_voie-1, bActive);
                            }
                            else
                            {
                                // Ajout de la variation pour chacune des plages temporelles dÃ©finies
                                for(size_t iPlage = 0; iPlage < plages.size(); iPlage++)
                                {
                                    pT->AddVoieReserveeByTypeVeh(pVectTypesInterdits, 0, 0, plages[iPlage], m_dbDureeSimu, num_voie-1, bActive);
                                }
                            }
                        }
                    }
                    pXMLNodeList->release();

                    // vÃ©rif de la couverture des plages temporelles
                    map<int, vector<PlageTemporelle*> >::iterator iter;
                    for(iter = plagesParVoie.begin(); iter != plagesParVoie.end(); iter++)
                    {
                        if(iter->second.size() > 0 && !CheckPlagesTemporellesEx(m_dbDureeSimu, iter->second))
                        {
                            log() << Logger::Error << "ERROR : The time frames defined for the reserved lanes of link " << pT->GetLabel() << " don't cover the home simulation duration for lane number " << iter->first << std::endl;
                            log() << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                            m_pXMLUtil->CleanLoadDocument(pXMLDocUpdate);
                            return false;
                        }
                    }
                }

                // Lecture des voies interdites
                DOMNode * XMLVoiesInterdites = m_pXMLUtil->SelectSingleNode("./VOIES_INTERDITES", pXMLTroncon->getOwnerDocument(), (DOMElement*)pXMLTroncon);
                if(XMLVoiesInterdites)
                {
                    // Lecture des voies interdites
                    DOMNode * pNodeVoieInterdite;
                    DOMXPathResult * pXMLNodeList = m_pXMLUtil->SelectNodes("./VOIE_INTERDITE", XMLVoiesInterdites->getOwnerDocument(), (DOMElement*)XMLVoiesInterdites);
                    XMLSize_t countj = pXMLNodeList->getSnapshotLength();
                    for(XMLSize_t j=0; j<countj; j++)
                    {
                        pXMLNodeList->snapshotItem(j);
                        pNodeVoieInterdite = pXMLNodeList->getNodeValue();
                        int num_voie;
                        GetXmlAttributeValue(pNodeVoieInterdite, "num_voie", num_voie, m_pLogger);
                        GetXmlAttributeValue(pNodeVoieInterdite, "id_typesvehicules", strTmp, m_pLogger);
                        std::deque<std::string> split = SystemUtil::split(strTmp, ' ');
                        std::vector<TypeVehicule*> lstTypesInterdits;
                        for(size_t splitIdx = 0; splitIdx<split.size(); splitIdx++)
                        {
                            TypeVehicule * pTV = GetVehicleTypeFromID(split[splitIdx]);
                            if(!pTV)
                            {
                                log() << Logger::Warning << " WARNING : the vehicle type " << split[splitIdx] << " defined for a forbidden lane of the link " << pT->GetLabel() << " doesn't exist." << endl;
                                log() << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                            }
                            else
                            {
                                lstTypesInterdits.push_back(pTV);
                            }
                        }
                        pT->AddVoieInterditeByTypeVeh(lstTypesInterdits, num_voie-1);
                    }
                    pXMLNodeList->release();
                }

                // Lecture des terre-pleins
                DOMNode * XMLTerrePleins = m_pXMLUtil->SelectSingleNode("./TERRE_PLEINS", pXMLTroncon->getOwnerDocument(), (DOMElement*)pXMLTroncon);
                if(XMLTerrePleins)
                {
                    DOMNode * pNodeTerrePlein;
                    DOMXPathResult * pXMLNodeList = m_pXMLUtil->SelectNodes("./TERRE_PLEIN", XMLTerrePleins->getOwnerDocument(), (DOMElement*)XMLTerrePleins);
                    XMLSize_t countj = pXMLNodeList->getSnapshotLength();
                    for(XMLSize_t j=0; j<countj; j++)
                    {
                        pXMLNodeList->snapshotItem(j);
                        pNodeTerrePlein = pXMLNodeList->getNodeValue();
                        int num_voie;
                        GetXmlAttributeValue(pNodeTerrePlein, "num_voie", num_voie, m_pLogger);
                        GetXmlAttributeValue(pNodeTerrePlein, "position_debut", dbTmp, m_pLogger);
                        GetXmlAttributeValue(pNodeTerrePlein, "position_fin", dbTmp2, m_pLogger);
                        dbTmp2=dbTmp2==numeric_limits<double>::infinity()?pT->GetLength():dbTmp2;

                        // VÃ©rifications de la position du terre-plein
                        if( pT->GetLength() < dbTmp || dbTmp2 <= dbTmp)
                        {
                            return false;
                        }

                        // lecture des diffÃ©rentes variantes temporelles
                        std::vector<bool> actifs;
                        std::vector<double> durees;
                        DOMXPathResult * pXMLTPVarList = m_pXMLUtil->SelectNodes("./TERRE_PLEIN_VARIATION", pNodeTerrePlein->getOwnerDocument(), (DOMElement*)pNodeTerrePlein);
                        XMLSize_t countk = pXMLTPVarList->getSnapshotLength();
                        std::vector<PlageTemporelle*> tmpPlagesVect;
                        std::vector<bool> tmpPlagesActivesVect;
                        for(XMLSize_t k=0; k<countk; k++)
                        {
                            pXMLTPVarList->snapshotItem(k);
                            DOMNode * pNodeTerrePleinVar = pXMLTPVarList->getNodeValue();

                            bool bActif = true;
                            GetXmlAttributeValue(pNodeTerrePleinVar, "actif", bActif, m_pLogger);
                            vector<PlageTemporelle*> plages;
                            if(!GetXmlDuree(pNodeTerrePleinVar,this, dbDuree, plages, m_pLogger))
                            {
                                m_pXMLUtil->CleanLoadDocument(pXMLDocUpdate);
                                return false;
                            }

                            if(plages.size() == 0)
                            {
                                durees.push_back(dbDuree);
                                actifs.push_back(bActif);
                            }
                            else
                            {
                                tmpPlagesVect.insert(tmpPlagesVect.end(), plages.begin(), plages.end());
                                tmpPlagesActivesVect.resize(tmpPlagesActivesVect.size() + plages.size(), bActif);
                            }
                        }
                        pXMLTPVarList->release();

                        // vÃ©rification de la couverture des plages temporelles
                        if(tmpPlagesVect.size() > 0)
                        {
                            if(!CheckPlagesTemporellesEx(m_dbDureeSimu, tmpPlagesVect))
                            {
                                log() << Logger::Error << "ERROR : The time frames defined for a median of link " << pT->GetLabel() << " don't cover the whole simulation duration!" << std::endl;
                                log() << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                                m_pXMLUtil->CleanLoadDocument(pXMLDocUpdate);
                                return false;
                            }
                        }

                        pT->AddTerrePlein(num_voie-1, dbTmp, dbTmp2, durees, actifs, tmpPlagesVect, tmpPlagesActivesVect);
                    }
                    pXMLNodeList->release();
                }
            }
        }
    }
    // Section RESEAU / ROUTES
    DOMNode * pRoutesNode;

    pRoutesNode = m_pXMLUtil->SelectSingleNode( "./ROUTES", pXMLDocUpdate, (DOMElement*)pXMLReseau);

    if( pRoutesNode )
    {
        // parcours toutes les routes si le noeud existe
        XMLSize_t counti = pRoutesNode->getChildNodes()->getLength();
        for( XMLSize_t i =0; i < counti; ++i)
        {
            string strRouteId;

            DOMNode * pRouteNode;
            pRouteNode = pRoutesNode->getChildNodes()->item(i);
            if (pRouteNode->getNodeType() != DOMNode::ELEMENT_NODE) continue;

            GetXmlAttributeValue( pRouteNode, "id", strRouteId, m_pLogger);
            // Obtient les troncons d'itineraires
            DOMNode * pXMLTroncons = m_pXMLUtil->SelectSingleNode("./TRONCONS_", pRouteNode->getOwnerDocument(), (DOMElement*)pRouteNode);
            std::vector<Tuyau *> routeLinks;
            if( pXMLTroncons)
            {
                XMLSize_t countj = pXMLTroncons->getChildNodes()->getLength();
                std::string strTronconId;
                for(XMLSize_t j=0; j<countj;j++)
                {
                    DOMNode * pTronconNode = pXMLTroncons->getChildNodes()->item(j);
                    GetXmlAttributeValue( pTronconNode, "id", strTronconId, m_pLogger);
                    Tuyau *pLink = this->GetLinkFromLabel(strTronconId);
                    if( pLink )
                    {
                        routeLinks.push_back(pLink);
                    }

                } //rof each troncons
            }

            Connexion * pJunction = NULL;
            DOMNode * pXMLNoeud = m_pXMLUtil->SelectSingleNode("./NOEUD", pRouteNode->getOwnerDocument(), (DOMElement*)pRouteNode);
            if (pXMLNoeud)
            {
                GetXmlAttributeValue(pXMLNoeud, "id", strTmp, m_pLogger);
                char cTmp;
                pJunction = GetConnectionFromID(strTmp, cTmp);
                if (!pJunction)
                {
                    pJunction = GetBrickFromID(strTmp);
                }
            }

            // ajoute la route en mÃ©moire
            if (!strRouteId.empty())
            {
                m_routes[strRouteId] = std::make_pair(routeLinks, pJunction);
            }

        } // rof each routes

    }
    // fin section RESEAU / ROUTES

    //----------------------
    // Noeud repartiteurs
    //----------------------
    Connexion *pCnx;
    DOMNode *pXMLCnxs;
    DOMNode    *pXMLCnx;
    DOMNode *pXMLMvts;

    pXMLCnxs = m_pXMLUtil->SelectSingleNode( "CONNEXIONS/REPARTITEURS", pXMLDocUpdate, (DOMElement*)pXMLReseau);
    if(pXMLCnxs)
    {
        XMLSize_t counti = pXMLCnxs->getChildNodes()->getLength();
        for(XMLSize_t i=0; i<counti;i++)
        {
            pXMLCnx = pXMLCnxs->getChildNodes()->item(i);
            if (pXMLCnx->getNodeType() != DOMNode::ELEMENT_NODE) continue;

            // id
            GetXmlAttributeValue(pXMLCnx, "id", sTmp, m_pLogger);
            pCnx = GetConnectionFromID( sTmp, cType );

            if(pCnx)
            {
                // Matrice des mouvements autorisÃ©s
                pXMLMvts = m_pXMLUtil->SelectSingleNode( "./MOUVEMENTS_AUTORISES", pXMLDocUpdate, (DOMElement*)pXMLCnx);
                if(pXMLMvts)
                {
                    pCnx->m_mapMvtAutorises.clear();
                    if( !LoadMouvementsAutorises( pCnx, pXMLMvts, m_pLogger))
                        return false;
                }
            }
        }
    }

    //----------------------
    // Noeud carrefoursafeux
    //----------------------
    pXMLCnxs = m_pXMLUtil->SelectSingleNode( "CONNEXIONS/CARREFOURSAFEUX", pXMLDocUpdate, (DOMElement*)pXMLReseau);
    if(pXMLCnxs)
    {
        XMLSize_t counti = pXMLCnxs->getChildNodes()->getLength();
        for(XMLSize_t i=0; i<counti;i++)
        {
            pXMLCnx = pXMLCnxs->getChildNodes()->item(i);
            if (pXMLCnx->getNodeType() != DOMNode::ELEMENT_NODE) continue;

            // id
            GetXmlAttributeValue(pXMLCnx, "id", sTmp, m_pLogger);
            CarrefourAFeuxEx * pCAF = (CarrefourAFeuxEx*)GetBrickFromID( sTmp );

            if(pCAF)
            {
                // Matrice des mouvements autorisÃ©s
                pXMLMvts = m_pXMLUtil->SelectSingleNode( "./MOUVEMENTS_AUTORISES", pXMLDocUpdate, (DOMElement*)pXMLCnx);
                if(pXMLMvts)
                {
                    pCAF->m_mapMvtAutorises.clear();
                    if(!LoadMouvementsAutorises( pCAF, pXMLMvts, m_pLogger))
                        return false;
                    pCAF->ReInit(pXMLCnx, m_pLogger);
                }
            }
        }
    }

    pXMLTrafic = m_pXMLUtil->SelectSingleNode("TRAFICS/TRAFIC[@id=\"" + m_TraficID + "\"]", pXMLDocUpdate, (DOMElement*)pXMLRoot);

    //-----------------------------------
    // Noeud extremitÃ©s et parkings
    //-----------------------------------
    // RÃ©cupÃ©ration du noeud TRAFIC/EXTREMITES et TRAFIC/PARKINGS
    DOMXPathResult * pXMLExtremites = m_pXMLUtil->SelectNodes( "EXTREMITES | PARKINGS | ZONES_DE_TERMINAISON", pXMLDocUpdate, (DOMElement*)pXMLTrafic);
    XMLSize_t countExtr = pXMLExtremites->getSnapshotLength();
    for(XMLSize_t iExtr=0; iExtr<countExtr; iExtr++)
    {
        pXMLExtremites->snapshotItem(iExtr);
        XMLSize_t counti = pXMLExtremites->getNodeValue()->getChildNodes()->getLength();
        for(XMLSize_t i=0; i<counti;i++)
        {
            pXMLExtremite = pXMLExtremites->getNodeValue()->getChildNodes()->item(i);
            if (pXMLExtremite->getNodeType() != DOMNode::ELEMENT_NODE) continue;

            // id
            GetXmlAttributeValue(pXMLExtremite, "id", sTmp, m_pLogger);

            char cOrigineType, cDestinationType;
            pE = GetOrigineFromID( sTmp, cOrigineType );   // Recherche de l'origine correspondante

            pS = GetDestinationFromID( sTmp, cDestinationType );   // Recherche de la destination correspondante

            if(!pE && !pS)     // l'extremitÃ© n'existe pas
            {
                log() << Logger::Error << "ERROR update : the extremity " << sTmp << " doesn't exist ! " << std::endl;
                log() << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                m_pXMLUtil->CleanLoadDocument(pXMLDocUpdate);
                return false;
            }


            if(pE)
            {
                TypeVehicule * pTypeVehicle = NULL;
                std::map<TypeVehicule *, DOMNode*> mapTypeFlux;
                DOMNode* pXMlFluxGlobal = m_pXMLUtil->SelectSingleNode("./FLUX_GLOBAL/FLUX", pXMLExtremite->getOwnerDocument(), (DOMElement*)pXMLExtremite);
                if( pXMlFluxGlobal )
                {
                    mapTypeFlux[NULL]= pXMlFluxGlobal;
                }
                else // obtient les flux par type si ils exsitent
                {
                     DOMNode* pXMlFluxParTypes = m_pXMLUtil->SelectSingleNode("./FLUX_TYPEVEHS", pXMLExtremite->getOwnerDocument(), (DOMElement*)pXMLExtremite);
                     if( pXMlFluxParTypes )
                    {
                         XMLSize_t nFluxType =  pXMlFluxParTypes->getChildNodes()->getLength();
                         // on parcours les flux pour tous les types de vÃ©hicule
                         for (XMLSize_t iFluxType = 0; iFluxType < nFluxType; ++iFluxType)
                         {
                             DOMNode *pFluxNodeParType = pXMlFluxParTypes->getChildNodes()->item(iFluxType);
                                // Obtient l'identifiant du Type de Vehicule
                             string strTypeVeh = "";
                             GetXmlAttributeValue(pFluxNodeParType, "id_typevehicule", strTypeVeh, m_pLogger);
                             if ( GetVehicleTypeFromID(strTypeVeh) )
                             {
                                 mapTypeFlux[GetVehicleTypeFromID(strTypeVeh)] =  m_pXMLUtil->SelectSingleNode("./FLUX", pXMLExtremite->getOwnerDocument(), (DOMElement*)pFluxNodeParType);
                             }
                          }
                     }
                }
                // on parcours les diffÃ©rents flux
                std::map<TypeVehicule *, DOMNode*>::iterator itFlux;
                for( itFlux = mapTypeFlux.begin(); itFlux != mapTypeFlux.end() ; itFlux++)
                {
                    pTypeVehicle = itFlux->first;
                    DOMNode * pXMLFuxNode = itFlux->second;
                    // -------------------
                    // DEMANDES
                    //--------------------
                    pXMLDemandes = m_pXMLUtil->SelectSingleNode("./DEMANDES", pXMLExtremite->getOwnerDocument(), (DOMElement*)pXMLFuxNode);

                    if(pXMLDemandes)
                    {
                        if(!(pE->IsTypeDemande() || pE->IsTypeDistribution()) )   // le type de crÃ©ation des vÃ©hicule n'est pas une demande
                        {
                            log() << Logger::Error << "ERROR update : the input " << sTmp << " doesn't have the right type of demand ! " << std::endl;
                            log() << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                            m_pXMLUtil->CleanLoadDocument(pXMLDocUpdate);
                            return false;
                        }

                        // Suppresion des palliers de demande Ã  partir du moment courant
                        if( pTypeVehicle == NULL ) // cas global on supprime tout
                        {
                           std::map<TypeVehicule*, ListOfTimeVariation<tracked_double>* > lstDemande = pE->GetLstDemande();
                           std::map<TypeVehicule*, ListOfTimeVariation<tracked_double>* >::iterator itDemande;
                           for( itDemande =  lstDemande.begin(); itDemande != lstDemande.end(); itDemande ++)
                           {
                               EraseVariation((*itDemande).second->GetLstTV(), m_dbInstSimu, m_dbLag);
                           }
                        }
                        else
                        {
                            EraseVariation( pE->GetLstDemande(pTypeVehicle)->GetLstTV(), m_dbInstSimu, m_dbLag);
                        }

                        // Mise Ã  jour des nouvelles demandes
                        XMLSize_t countj = pXMLDemandes->getChildNodes()->getLength();
                        for(XMLSize_t j=0; j<countj;j++)
                        {
                            DOMNode * xmlnode = pXMLDemandes->getChildNodes()->item(j);
                            if (xmlnode->getNodeType() != DOMNode::ELEMENT_NODE) continue;

                            // Chargement de la variante temporelle de la demande
                            vector<PlageTemporelle*> plages;
                            if(!GetXmlDuree(xmlnode,this,dbDuree, plages, m_pLogger))
                            {
                                m_pXMLUtil->CleanLoadDocument(pXMLDocUpdate);
                                return false;
                            }

                            boost::shared_ptr<tracked_double> pTrackedDouble = boost::make_shared<tracked_double>();
                            GetXmlAttributeValue(xmlnode, "niveau", *pTrackedDouble, m_pLogger);


                            if(plages.size() > 0)
                            {
                                for(size_t iPlage = 0; iPlage < plages.size(); iPlage++)
                                {
                                    AddVariation<>( plages[iPlage], pTrackedDouble, pE->GetLstDemande(pTypeVehicle)->GetLstTV());
                                }
                            }
                            else
                            {
                                AddVariation<>( dbDuree, pTrackedDouble, pE->GetLstDemande(pTypeVehicle)->GetLstTV());
                            }
                            if( pTypeVehicle == NULL ) // cas global on affecte Ã  tous les types
                            {
                                for( size_t iTypeV = 0; iTypeV < m_LstTypesVehicule.size() ; ++iTypeV)
                                {
                                    if(plages.size() > 0)
                                    {
                                        for(size_t iPlage = 0; iPlage < plages.size(); iPlage++)
                                        {
                                            AddVariation<>( plages[iPlage], pTrackedDouble, pE->GetLstDemande(m_LstTypesVehicule[i])->GetLstTV());
                                        }
                                    }
                                    else
                                    {
                                        AddVariation<>( dbDuree, pTrackedDouble, pE->GetLstDemande(m_LstTypesVehicule[i])->GetLstTV());
                                    }
                                }
                            }
                        }

                        // vÃ©rification de la couverture de l'ensemble de la simulation

                        vector<PlageTemporelle*> plages;
                        for(size_t iPlage = 0; iPlage < pE->GetLstDemande(pTypeVehicle)->GetLstTV()->size(); iPlage++)
                        {
                            PlageTemporelle * pPlage = pE->GetLstDemande(pTypeVehicle)->GetLstTV()->at(iPlage).m_pPlage;
                            if(pPlage)
                            {
                                plages.push_back(pPlage);
                            }
                        }
                        if(plages.size() > 0 && !CheckPlagesTemporellesEx(m_dbDureeSimu, plages))
                        {
                            log() << Logger::Error << "ERROR : The time frames defined for the demand of input " << pE->GetOutputID() << " don't cover the whole simulation duration ! " << std::endl;
                            log() << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                            m_pXMLUtil->CleanLoadDocument(pXMLDocUpdate);
                            return false;
                        }

                    }

                    // -------------------
                    // MATRICES_OD
                    //--------------------

                    {
                        if(IsUsedODMatrix())
                        {
                            // Matrices OD de l'entrÃ©e considÃ©rÃ©e
                            DOMNode *pXMLRepDestinations;
                            DOMNode *pXMLRepDestination;
                            VectDest*               pVectDest;
                            SymuViaTripNode*        pDestination;

                            pXMLRepDestinations = m_pXMLUtil->SelectSingleNode( "./REP_DESTINATIONS", pXMLDocUpdate, (DOMElement*)pXMLFuxNode);

                            if(pXMLRepDestinations)
                            {
                                XMLSize_t countj = pXMLRepDestinations->getChildNodes()->getLength();
                                for(XMLSize_t j=0; j<countj; j++)
                                {
                                    pXMLRepDestination = pXMLRepDestinations->getChildNodes()->item(j);
                                    if (pXMLRepDestination->getNodeType() != DOMNode::ELEMENT_NODE) continue;

                                    if( pTypeVehicle == NULL  ) // cas global on affecte  Ã  tous les types
                                    {
                                        std::map<TypeVehicule*, std::deque<TimeVariation<SimMatOrigDest> > >::iterator iterCoeffDest;
                                        for(iterCoeffDest = pE->GetLstCoeffDest().begin(); iterCoeffDest != pE->GetLstCoeffDest().end(); iterCoeffDest++)
                                        {
                                            EraseVariation(&iterCoeffDest->second, m_dbInstSimu, m_dbLag);
                                        }
                                     }
                                     else
                                     {
                                           EraseVariation(&pE->GetLstCoeffDest(pTypeVehicle ), m_dbInstSimu, m_dbLag);
                                     }

                                    // Duree
                                    vector<PlageTemporelle*> plages;
                                    if(!GetXmlDuree(pXMLRepDestination,this,dbDuree,plages, m_pLogger))
                                    {
                                        m_pXMLUtil->CleanLoadDocument(pXMLDocUpdate);
                                        return false;
                                    }

                                    boost::shared_ptr<SimMatOrigDest> pMatOrigDest = boost::make_shared<SimMatOrigDest>();

                                    // Destinations
                                    double dbSum = 0;
                                    XMLSize_t countk = pXMLRepDestination->getChildNodes()->getLength();
                                    for(XMLSize_t k=0; k<countk; k++)
                                    {
                                        DOMNode * xmlnode = pXMLRepDestination->getChildNodes()->item(k);
                                        if (xmlnode->getNodeType() != DOMNode::ELEMENT_NODE) continue;

                                        // Destination
                                        GetXmlAttributeValue(xmlnode, "sortie", sTmp, m_pLogger);
                                        char cDestinationType;
                                        pDestination = GetDestinationFromID(sTmp, cDestinationType);

                                        // Correction bug nÂ°39 : prÃ©vention du plantage si la destination n'est pas en aval d'un tronÃ§on
                                        if(!pDestination)
                                        {
                                            log() << Logger::Error << "ERROR : the destination " << sTmp << " in not correctly defined (the corresponding extremity might not be declared as the downstream node of a link ?)" << std::endl;
                                            log() << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                                            return false;
                                        }

                                        pE->AddDestinations(pDestination);

                                        // Coefficient d'affectation
                                        GetXmlAttributeValue(xmlnode, "coeffOD", dbTmp, m_pLogger);
                                        dbSum += dbTmp;

                                        pVectDest = new VectDest;
                                        pVectDest->pDest = pDestination;
                                        pVectDest->dbCoeff = dbTmp;
                                        pMatOrigDest->MatOrigDest.push_back(pVectDest);

                                        if(k == countk-1)
                                        {
                                            // VÃ©rification de la somme des coefficients
                                            if( fabs(dbSum-1) > countk*std::numeric_limits<double>::epsilon())
                                            {
                                                log() << Logger::Error << "ERROR update : the coefficients sum for all destinations of the input " << pE->GetOutputID() << " - variation " << j <<" of the element MATRICE_OD - " << " must be equal to 1" << std::endl;
                                                log() << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                                                m_pXMLUtil->CleanLoadDocument(pXMLDocUpdate);
                                                return false;
                                            }
                                        }
                                    }

                                    // Ajout de la matrice avec sa durÃ©e Ã  la liste des matrices de l'entrÃ©e

                                    if(plages.size() > 0)
                                    {
                                        for(size_t iPlage = 0; iPlage < plages.size(); iPlage++)
                                        {
                                            AddVariation<>( plages[iPlage], pMatOrigDest, &pE->GetLstCoeffDest(pTypeVehicle) );
                                        }
                                    }
                                    else
                                    {
                                        AddVariation<>( dbDuree, pMatOrigDest, &pE->GetLstCoeffDest(pTypeVehicle) );
                                    }
                                    if( pTypeVehicle == NULL  ) // cas global on affecte  Ã  tous les types
                                    {
                                        for( int iTypeV =0; iTypeV != m_LstTypesVehicule.size(); iTypeV++)
                                        {
                                            if(plages.size() > 0)
                                            {
                                                for(size_t iPlage = 0; iPlage < plages.size(); iPlage++)
                                                {
                                                    AddVariation<>( plages[iPlage], pMatOrigDest, &pE->GetLstCoeffDest(m_LstTypesVehicule[i]) );
                                                }
                                            }
                                            else
                                            {
                                                AddVariation<>( dbDuree, pMatOrigDest, &pE->GetLstCoeffDest(m_LstTypesVehicule[i]) );
                                            }
                                        }
                                    }

                                } // rof all matrices

                                // vÃ©rification de la couverture des plages temporelles poour chaque type de vÃ©hicule
                                for(size_t iTV = 0; iTV < m_LstTypesVehicule.size(); iTV++)
                                {
                                    vector<PlageTemporelle*> plages;
                                    for(size_t iPlage = 0; iPlage < pE->GetLstCoeffDest(m_LstTypesVehicule[iTV]).size(); iPlage++)
                                    {
                                        PlageTemporelle * pPlage = pE->GetLstCoeffDest(m_LstTypesVehicule[iTV])[iPlage].m_pPlage;
                                        if(pPlage)
                                        {
                                            plages.push_back(pPlage);
                                        }
                                    }
                                    if(plages.size() > 0 && !CheckPlagesTemporellesEx(m_dbDureeSimu, plages))
                                    {
                                        log() << Logger::Error << "ERROR : The time frames defined for the REP_DESTINATION node of the input " << pE->GetOutputID() << " don't cover the whole simulation duration for vehicle type " << m_LstTypesVehicule[iTV]->GetLabel() << std::endl;
                                        log() << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                                        m_pXMLUtil->CleanLoadDocument(pXMLDocUpdate);
                                        return false;
                                    }
                                }
                            }
                        }
                    } // fi node matrice od
                    //---------------------
                    // REPARTITIONS_VOIE
                    //---------------------
                    DOMNode *pXMLRepartitionsVoie = m_pXMLUtil->SelectSingleNode( "./REP_VOIES", pXMLDocUpdate, (DOMElement*)pXMLFuxNode);

                    if(pXMLRepartitionsVoie)
                    {

                        // Suppression des variantes chargÃ©es

                       EraseVariation(pE->GetLstRepVoie(pTypeVehicle), m_dbInstSimu, m_dbLag);
                       if( pTypeVehicle == NULL ) // cas global -> on supprime tous les types
                       {
                            for( int iTypeV =0; iTypeV != m_LstTypesVehicule.size(); iTypeV++)
                            {
                               EraseVariation(pE->GetLstRepVoie(m_LstTypesVehicule[i]), m_dbInstSimu, m_dbLag);
                            }
                       }

                        XMLSize_t countj = pXMLRepartitionsVoie->getChildNodes()->getLength();
                        for(XMLSize_t j=0; j<countj;j++)
                        {
                            DOMNode * xmlnode = pXMLRepartitionsVoie->getChildNodes()->item(j);
                            if (xmlnode->getNodeType() != DOMNode::ELEMENT_NODE) continue;

                            std::vector<double> pCoeff;
                            boost::shared_ptr<RepartitionEntree> pRE = boost::make_shared<RepartitionEntree>();
                            // DurÃ©e
                            vector<PlageTemporelle*> plages;
                            if(!GetXmlDuree(xmlnode,this,dbDuree,plages, m_pLogger))
                            {
                                m_pXMLUtil->CleanLoadDocument(pXMLDocUpdate);
                                return false;
                            }

                            // Valeurs
                            GetXmlAttributeValue(xmlnode, "coeffs", sTmp, m_pLogger);
                            std::deque<std::string> split = SystemUtil::split(sTmp, ' ');
                            if( split.size() !=  pE->GetOutputConnexion()->m_LstTuyAv.front()->getNbVoiesDis())
                            {
                                log() << Logger::Error << "ERROR update : the number of values of the 'valeurs' attribute of the REPARTITION_VOIE node (origin : " << pE->GetOutputID();
                                log() << Logger::Error << " - repartition : "<< j+1 <<  " ) is invalid (it must be the same as the number of lanes of the link " << pE->GetOutputConnexion()->m_LstTuyAv.front()->GetLabel() <<").";
                                log() << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                                m_pXMLUtil->CleanLoadDocument(pXMLDocUpdate);
                                return false;
                            }
                            double dbS = 0;
                            for(int k=0; k<pE->GetOutputConnexion()->m_LstTuyAv.front()->getNbVoiesDis();k++)
                            {
                                pCoeff.push_back(atof( split.at(k).c_str() ));
                                dbS += pCoeff[k];
                            }
                            if( fabs(dbS - 1) > 0.000001 )
                            {
                                log() << Logger::Error << "ERROR update : the values of attribute 'valeurs' of the REPARTITION_VOIE node (origin : " << pE->GetOutputID();
                                log() << Logger::Error << " - repartition : "<< j+1 <<  " ) are invalid (the sum must be 1) ";
                                log() << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                                m_pXMLUtil->CleanLoadDocument(pXMLDocUpdate);
                                return false;
                            }
                            pRE->pCoefficients = pCoeff;

                            if(plages.size() > 0)
                            {
                                for(size_t iPlage = 0; iPlage < plages.size(); iPlage++)
                                {
                                    AddVariation(plages[iPlage], pRE, pE->GetLstRepVoie(pTypeVehicle));
                                }
                            }
                            else
                            {
                                AddVariation(dbDuree, pRE, pE->GetLstRepVoie(pTypeVehicle));
                            }
                            if( pTypeVehicle == NULL )
                            { // affecte Ã  tous les types

                                 if(plages.size() > 0)
                                {
                                    for( int iTypeV =0; iTypeV != m_LstTypesVehicule.size(); iTypeV++)
                                    {
                                        for(size_t iPlage = 0; iPlage < plages.size(); iPlage++)
                                        {
                                            AddVariation(plages[iPlage], pRE, pE->GetLstRepVoie(m_LstTypesVehicule[i]));
                                        }
                                    }
                                }
                                else
                                {
                                    for( int iTypeV =0; iTypeV != m_LstTypesVehicule.size(); iTypeV++)
                                    {
                                        AddVariation(dbDuree, pRE, pE->GetLstRepVoie(m_LstTypesVehicule[i]));
                                    }
                                }
                            }
                        }

                        // vÃ©rification de la couverture des plages temporelles
                        vector<PlageTemporelle*> plages;
                        for(size_t iPlage = 0; iPlage < pE->GetLstRepVoie(pTypeVehicle)->size(); iPlage++)
                        {
                            PlageTemporelle * pPlage = pE->GetLstRepVoie(pTypeVehicle)->at(iPlage).m_pPlage;
                            if(pPlage)
                            {
                                plages.push_back(pPlage);
                            }
                        }
                        if(plages.size() > 0 && !CheckPlagesTemporellesEx(m_dbDureeSimu, plages))
                        {
                            log() << Logger::Error << "ERROR : The time frames defined for the input lanes repartition for input " << pE->GetOutputID() << " don't cover the whole simulation duration !" << std::endl;
                            log() << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                            m_pXMLUtil->CleanLoadDocument(pXMLDocUpdate);
                            return false;
                        }
                    }
                } // rof all flux node


                // ** Traitement de la rÃ©partition des vÃ©hicules
                int nTypes = (int)m_LstTypesVehicule.size();
                int nNbVoie;
                if(cOrigineType == 'Z')
                {
                    // cas de la zone : une seule liste de coeffs
                    nNbVoie = 1;
                }
                else
                {
                    pT = pE->GetOutputConnexion()->m_LstTuyAv.front();
                    nNbVoie = pT->getNbVoiesDis();
                }
                if( pXMlFluxGlobal ) // la rÃ©partition est dÃ©finie globalement
                {
                    // Chargement
                    // Traitement des rÃ©partition de typeVÃ©hicules uniquement pour les flux "globaux"
                    DOMNode *pXMLRepartitions = m_pXMLUtil->SelectSingleNode( "./REP_TYPEVEHICULES", pXMlFluxGlobal->getOwnerDocument(), (DOMElement*)pXMlFluxGlobal->getParentNode());

                    // Suppression des variantes chargÃ©es
                    pE->GetLstRepTypeVeh()->ClearFrom(m_dbInstSimu, m_dbLag);
                    if( pXMLRepartitions)
                    {
                        std::vector<TypeVehicule*> placeHolder;
                        if(!LoadRepTypeVehicules(pXMLRepartitions, pE->GetOutputID(), nNbVoie, placeHolder, pE->GetLstRepTypeVeh(), &log()))
                        {
                            return false;
                        }

                        // Ajout des types de vÃ©hicules au tronÃ§on
                        for(size_t iTuy = 0; iTuy < pE->GetOutputConnexion()->m_LstTuyAv.size(); iTuy++)
                        {
                            for(size_t k=1; k<m_LstTypesVehicule.size();k++)      // Ajout des autres types de vÃ©hicule que le type de base
                                pE->GetOutputConnexion()->m_LstTuyAv[iTuy]->AddTypeVeh(m_LstTypesVehicule[k]);
                        }
                    }
                    else // rien n'a Ã©tÃ© dÃ©fini (seul des vehicules de type de base seront gÃ©nÃ©rÃ©s)
                    {
                        int nNbVoie;
                        if(cOrigineType == 'Z')
                        {
                            // cas de la zone : une seule liste de coeffs
                            nNbVoie = 1;
                        }
                        else
                        {
                            pT = pE->GetOutputConnexion()->m_LstTuyAv.front();
                            nNbVoie = pT->getNbVoiesDis();
                        }

                        std::vector<std::vector<double> > coeffs(nTypes);

                        for(int j=0; j<nTypes; j++)
                        {
                            coeffs[j].resize(nNbVoie);
                            // Par defaut on ne sort que le premier type
                            for( int iCoeffWay = 0; iCoeffWay < nNbVoie; ++iCoeffWay)
                            {
                                coeffs[j][iCoeffWay]=(j==0)?(1.0):0;
                            }
                        }

                        pE->GetLstRepTypeVeh()->AddVariation(coeffs, m_dbDureeSimu);
                    } // fi else il n'y a pas de noeud rep_vehiculeparType
                } // fi on est dans un cas global
                  // Fin traitement REP_TYPEVEHICULES
                else
                { // sinon on n'est pas dans une distribution globale -> on dÃ©duit les rÃ©partitions des dÃ©bits par type de vÃ©hicules
                    double dSumDuree= 0;
                    // construit la liste des durÃ©es des demandes des diffÃ©rents type de vÃ©hicule
                    std::map<TypeVehicule *, ListOfTimeVariation<tracked_double>*> demandesInit = pE->GetLstDemandeInit();
                    std::map<TypeVehicule *, ListOfTimeVariation<tracked_double>*>::iterator itDemande;
                    std::list<double> durees;
                    std::list<double>::const_iterator itDuree;
                    bool bIsPlage = false;
                    for( itDemande = demandesInit.begin(); itDemande != demandesInit.end(); itDemande++)
                    {
                        size_t nDuree = itDemande->second->GetLstTV()->size();
                        dSumDuree = 0;
                        for( size_t iDuree = 0; iDuree< nDuree; ++iDuree )
                        {
                            if( itDemande->second->GetLstTV()->at(iDuree).m_pPlage == NULL)
                            {
                                dSumDuree += itDemande->second->GetLstTV()->at(iDuree).m_dbPeriod;
                                durees.push_back(dSumDuree);

                            }
                            else
                            {
                                durees.push_back(itDemande->second->GetLstTV()->at(iDuree).m_pPlage->m_Debut);
                                durees.push_back(itDemande->second->GetLstTV()->at(iDuree).m_pPlage->m_Fin);
                                bIsPlage = true;
                            }
                        }
                    }
                    // construit la lsite des durÃ©es des rÃ©partitions par voies des diffÃ©rentes type de vÃ©hicule
                    std::map<TypeVehicule*, std::deque<TimeVariation<RepartitionEntree> >* > repVoiesInit = pE->GetLstRepVoieInit();
                    std::map<TypeVehicule*, std::deque<TimeVariation<RepartitionEntree> >* >::iterator itRepVoie;
                    for( itRepVoie = repVoiesInit.begin(); itRepVoie != repVoiesInit.end(); itRepVoie++)
                    {
                        size_t nDuree = itRepVoie->second->size();
                        dSumDuree = 0;
                        for( size_t iDuree = 0; iDuree< nDuree; ++iDuree )
                        {
                            if( itRepVoie->second->at(iDuree).m_pPlage == NULL)
                            {
                                dSumDuree += itRepVoie->second->at(iDuree).m_dbPeriod;
                                durees.push_back(dSumDuree);
                            }
                            else
                            {
                                durees.push_back(itRepVoie->second->at(iDuree).m_pPlage->m_Debut);
                                durees.push_back(itRepVoie->second->at(iDuree).m_pPlage->m_Fin);
                                bIsPlage = true;
                            }
                        }
                    }

                    durees.sort();
                    durees.unique(); // need a sorted list

                    // On vire les instants nÃ©gatifs le cas Ã©chÃ©ant en les remplacant par 0 (sinon GetVariationEx
                    // sur un durÃ©e nÃ©gative renvoie null)
                    bool bAddStartSimulationTime = false;
                    while (!durees.empty() && durees.front() < 0)
                    {
                        durees.pop_front();
                        bAddStartSimulationTime = true;
                    }
                    if (bAddStartSimulationTime)
                    {
                        durees.push_front(0.0);
                    }

                    int nNbVoie;
                    if(cOrigineType == 'Z')
                    {
                        // cas de la zone : une seule liste de coeffs
                        nNbVoie = 1;
                    }
                    else
                    {
                        pT = pE->GetOutputConnexion()->m_LstTuyAv.front();
                        nNbVoie = pT->getNbVoiesDis();
                    }

                    // pour chaque duree on lui associe des repartition partype d'aprÃ©s les demandes
                    dSumDuree = 0;
                    std::list<double>::const_iterator itEnd = durees.end();
                    if( durees.size()>1 && bIsPlage )
                    {
                        itEnd--;
                    }
                    for( itDuree =durees.begin(); itDuree != itEnd; itDuree++)
                    {

                        std::vector<std::vector<double> > coeffs(nTypes);
                        std::vector<double> sumPerType;
                        for( int i = 0; i< nNbVoie; ++i)
                        {
                            sumPerType.push_back(0);
                        }
                        for( size_t iTypeVeh = 0; iTypeVeh < m_LstTypesVehicule.size() ;  ++iTypeVeh )
                        {
                            coeffs[iTypeVeh].resize(nNbVoie);
                            if( demandesInit.find(m_LstTypesVehicule[iTypeVeh] ) != demandesInit.end() &&
                                demandesInit.find(m_LstTypesVehicule[iTypeVeh] )->second->GetVariationEx(*itDuree) )
                            {
                                double dCoeff = *demandesInit.find(m_LstTypesVehicule[iTypeVeh] )->second->GetVariationEx(*itDuree);

                                // module par repartition par voie
                                RepartitionEntree* pREType = NULL;
                                if( repVoiesInit.find(m_LstTypesVehicule[iTypeVeh]) != repVoiesInit.end())
                                {
                                    pREType = GetVariation(*itDuree, repVoiesInit.find(m_LstTypesVehicule[iTypeVeh])->second, GetLag()); // le debit d'entrÃ©e est considÃ©rÃ© "globalement"
                                }
                                double dVoieCoeff = 1.0 / nNbVoie;

                                for( int iCoeffWay = 0; iCoeffWay < nNbVoie; ++iCoeffWay)
                                {
                                     if( pREType )
                                    {
                                        std::vector<double> pCoeffType = pREType->pCoefficients;
                                        dVoieCoeff = pCoeffType[iCoeffWay ];
                                    }

                                     coeffs[iTypeVeh][iCoeffWay]=dCoeff *dVoieCoeff;
                                     sumPerType[iCoeffWay] += dCoeff *dVoieCoeff;
                                }
                            }
                            else
                            {
                                for( int iCoeffWay = 0; iCoeffWay < nNbVoie; ++iCoeffWay)
                                {
                                     coeffs[iTypeVeh][iCoeffWay]=0;
                                }
                            }
                        }
                        // on norme les coefficients par voie
                        for( size_t iTypeVeh = 0; iTypeVeh < m_LstTypesVehicule.size() ;  ++iTypeVeh )
                        {
                            for( int iCoeffWay = 0; iCoeffWay < nNbVoie; ++iCoeffWay)
                            {
                               coeffs[iTypeVeh][iCoeffWay]= (sumPerType[iCoeffWay]!=0)? coeffs[iTypeVeh][iCoeffWay] /sumPerType[iCoeffWay] : 0 ;
                            }
                        }
                        // on ajoute la repartion ainsi crÃ©Ã©e
                        if( bIsPlage == false)
                        {
                            pE->GetLstRepTypeVeh()->AddVariation(coeffs, *itDuree - dSumDuree); // gestion "globale par type"
                        }
                        else
                        {
                            std::list<double>::const_iterator itDureeEnd  = itDuree;
                            itDureeEnd++;

                            PlageTemporelle *pPlage =  PlageTemporelle::Set( *itDuree + m_dtDebutSimu.ToSecond(),*itDureeEnd + m_dtDebutSimu.ToSecond(), m_LstPlagesTemporelles , m_dtDebutSimu.ToSecond() );
                            pE->GetLstRepTypeVeh()->AddVariation(coeffs, pPlage); // gestion "globale par type"
                        }

                        dSumDuree = *itDuree;
                    }

                }
                Sortie * pTypedS = dynamic_cast<Sortie*>(pS);
                if(pTypedS)
                {
                // -------------------
                // CAPACITES
                //--------------------
                   pXMLCapacites = m_pXMLUtil->SelectSingleNode( "./CAPACITES", pXMLDocUpdate, (DOMElement*)pXMLExtremite);

                    if(pXMLCapacites)
                    {
                        // Suppression des variantes chargÃ©es
                        pTypedS->GetLstCapacites()->RemoveVariations();

                        for(XMLSize_t j=0; j<pXMLCapacites->getChildNodes()->getLength();j++) // La capacitÃ© Ã©volue au cours de la simulation
                        {
                            DOMNode * xmlnode = pXMLCapacites->getChildNodes()->item(j);
                            if (xmlnode->getNodeType() != DOMNode::ELEMENT_NODE) continue;

                            vector<PlageTemporelle*> plages;
                            if(!GetXmlDuree(xmlnode,this,dbDuree,plages, m_pLogger))
                            {
                                m_pXMLUtil->CleanLoadDocument(pXMLDocUpdate);
                                return false;
                            }
                            boost::shared_ptr<tracked_double> pdbTmp = boost::make_shared<tracked_double>();
                            GetXmlAttributeValue(xmlnode, "valeur", *pdbTmp, m_pLogger);      // capacitÃ©
                            if(plages.size() > 0)
                            {
                                for(size_t iPlage = 0; iPlage < plages.size(); iPlage++)
                                {
                                    pTypedS->GetLstCapacites()->AddVariation(plages[iPlage], pdbTmp );
                                }
                            }
                            else
                            {
                                pTypedS->GetLstCapacites()->AddVariation(dbDuree, pdbTmp );
                            }
                        }

                        // vÃ©rification de la couverture des plages temporelles
                        if(!pTypedS->GetLstCapacites()->CheckPlagesTemporelles(m_dbDureeSimu))
                        {
                            log() << Logger::Error << "ERROR : The time frames defined for the capacity of the output " << pTypedS->GetInputID() << " don't cover the whole simulation duration !" << std::endl;
                            log() << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                            m_pXMLUtil->CleanLoadDocument(pXMLDocUpdate);
                            return false;
                        }

                        // Remise Ã  jour du prochain instant de sortie en fonction de la nouvelle capacitÃ©
                        double dbNextInstSortie = pS->GetNextEnterInstant( pTypedS->GetInputConnexion()->m_LstTuyAm.front()->getNbVoiesDis(), m_dbInstSimu, m_dbInstSimu, GetTimeStep(), std::string() );
                        for(int j=0; j<pTypedS->GetInputConnexion()->m_LstTuyAm.front()->getNbVoiesDis(); j++)
                            ((VoieMicro*)pTypedS->GetInputConnexion()->m_LstTuyAm.front()->GetLstLanes()[j])->GetNextInstSortie()[std::string()] = dbNextInstSortie;
                    }
                }
            } // fi PE
        }
    }

    //----------------------
    // Noeud CONNEXIONS_INTERNES
    //----------------------
    DOMNode * pXMLCnxInternes = m_pXMLUtil->SelectSingleNode( "CONNEXIONS_INTERNES", pXMLDocUpdate, (DOMElement*)pXMLTrafic);
    if(pXMLCnxInternes)
    {
        XMLSize_t counti = pXMLCnxInternes->getChildNodes()->getLength();
        for(XMLSize_t i=0; i<counti;i++)
        {
            pXMLRepartiteur = pXMLCnxInternes->getChildNodes()->item(i);
            if (pXMLRepartiteur->getNodeType() != DOMNode::ELEMENT_NODE)
                continue;

            // id
            GetXmlAttributeValue(pXMLRepartiteur, "id", sTmp, m_pLogger);

            pR = (Repartiteur*)GetConnectionFromID( sTmp, cType );   // Recherche de l'entrÃ© correspondante

            if(cType != 'R')     // le rÃ©partiteur n'existe pas
            {
                log() << Logger::Error << "ERROR update : the repartitor " << sTmp << " doesn't exist !" << std::endl;
                log() << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                m_pXMLUtil->CleanLoadDocument(pXMLDocUpdate);
                return false;
            }


            if( !IsUsedODMatrix() )
            {
                // Suppression des variantes de flux chargÃ©es
                for (std::map<TypeVehicule*, std::deque<TimeVariation<RepartitionFlux> >* >::iterator iterTypeVeh = pR->GetLstRepartition().begin(); iterTypeVeh != pR->GetLstRepartition().end(); ++iterTypeVeh)
                {
                    EraseVariation(iterTypeVeh->second, m_dbInstSimu, m_dbLag);
                    delete iterTypeVeh->second;
                }
                pR->GetLstRepartition().clear();

                // CrÃ©ation des nouvelles rÃ©partitions
                BuildRepartitionFlux(pR, pXMLDocUpdate, m_pLogger);
            }
        }
    }
    pXMLExtremites->release();

    //----------------------
    // Noeud CONTROLEURS_DE_FEUX
    //----------------------

    ControleurDeFeux *pCtrDeFeux;
    DOMNode *pXMLNode = m_pXMLUtil->SelectSingleNode( "CONTROLEURS_DE_FEUX", pXMLDocUpdate, (DOMElement*)pXMLTrafic);

    if(pXMLNode)
    {
        // Lecture de la dÃ©finition des contrÃ´leurs de feux
        XMLSize_t counti = pXMLNode->getChildNodes()->getLength();
        for(XMLSize_t i=0; i<counti;i++)
        {
            DOMNode * pXMLCtrlDeFeux = pXMLNode->getChildNodes()->item(i);
            if (pXMLCtrlDeFeux->getNodeType() != DOMNode::ELEMENT_NODE) continue;

            // Lecture de l'identifiant du contrÃ´leur
            GetXmlAttributeValue(pXMLCtrlDeFeux, "id", sTmp, m_pLogger);

            // Recherche du contrÃ´leur de feux
            pCtrDeFeux = GetTrafficLightControllerFromID(sTmp);

            if(!pCtrDeFeux)
                break;

            // Lecture de la durÃ©e de rouge de dÃ©gagement
            double dbDureeRougeDegagement;
            GetXmlAttributeValue(pXMLCtrlDeFeux, "duree_rouge_degagement", dbDureeRougeDegagement, m_pLogger);

            // Lecture de la durÃ©e de vert minimum
            double dbDureeVertMin;
            GetXmlAttributeValue(pXMLCtrlDeFeux, "duree_vert_min", dbDureeVertMin, m_pLogger);

            // MAJ des caractÃ©ristiques du CDF
            pCtrDeFeux->SetDureeRougeDegagement(dbDureeRougeDegagement);
            pCtrDeFeux->SetDureeVertMin(dbDureeVertMin);

            // MAJ des plans de feux
            DOMNode * pXMLPlansDeFeux = m_pXMLUtil->SelectSingleNode( "PLANS_DE_FEUX", pXMLDocUpdate, (DOMElement*)pXMLCtrlDeFeux);
            if(pXMLPlansDeFeux)
            {
                XMLSize_t countj = pXMLPlansDeFeux->getChildNodes()->getLength();
                for(XMLSize_t j=0; j<countj;j++)
                {
                    DOMNode * xmlnode = pXMLPlansDeFeux->getChildNodes()->item(j);
                    if (xmlnode->getNodeType() != DOMNode::ELEMENT_NODE) continue;

                    // ID
                    char strID[64];
                    GetXmlAttributeValue(xmlnode, "id", sTmp, m_pLogger);
                    strcpy(strID, sTmp.c_str());

                    // Heure de dÃ©but
                    STime dtDebut;
                    SDateTime dt;
                    GetXmlAttributeValue(xmlnode, "debut", dt, m_pLogger);
                    dtDebut = dt.ToTime();

                    // Recherche du plan de feux identifiÃ© par son ID
                    PlanDeFeux *pPlanDeFeux = NULL;
                    for(int nPDF =0; nPDF < (int)pCtrDeFeux->GetLstTrafficLightCycles()->GetLstTV()->size(); nPDF++ )
                        if( !pCtrDeFeux->GetLstTrafficLightCycles()->GetVariation(nPDF)->GetID().compare(strID))
                        {
                            pPlanDeFeux = pCtrDeFeux->GetLstTrafficLightCycles()->GetVariation(nPDF);
                            break;
                        }
                     if(!pPlanDeFeux)
                     {

                        pPlanDeFeux = new PlanDeFeux(strID, dtDebut);

                        // Ajout au contrÃ´leur de feux
                        pCtrDeFeux->GetLstTrafficLightCycles()->AddVariation( dtDebut, pPlanDeFeux);
                     }
                     else
                     {
                        pPlanDeFeux->m_tHrDeb = dtDebut;
                     }

                    pPlanDeFeux->RemoveAllSequences();
                    this->LoadDataPlanDeFeux( pCtrDeFeux, pPlanDeFeux, xmlnode, m_pLogger);
                }
            }
        }
    }

    if(pXMLDocUpdate) {
        m_pXMLUtil->CleanLoadDocument(pXMLDocUpdate);

        // En cas d'update, il faut relancer l'affectation (modifications possible des povuements autorisÃ©s, matrices OD, demandes,
        // etc...)
        m_pModuleAffectation->Run(this, m_dbInstSimu, 'R', false);
    }

    return true;
}

//=================================================================
    double Reseau::LoadLagOfVariation
//----------------------------------------------------------------
// Fonction  : Chargement du 'lag' des variations des donnÃ©es
// Remarque  :
// Version du: 21/07/2008
// Historique: 21/07/2008 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//=================================================================
(
    DOMNode					*pXmlNodeVar,
    Logger                  &logger
)
{
    SDateTime	  tDebutData;
    STimeSpan	  tsLag;

    if(pXmlNodeVar)
        GetXmlAttributeValue(pXmlNodeVar, "debut", tDebutData, &logger);

    if( m_dtDebutSimu < tDebutData.ToTime())
    {
        return -1;
    }

    //if(tDebutData > 0)
    if(tDebutData > SDateTime())
        tsLag =  m_dtDebutSimu - tDebutData.ToTime();
    else
        tsLag = 0;

    return (double)tsLag.GetTotalSeconds();
}

//=================================================================
    std::string Reseau::GetLibelleTronconEve
//----------------------------------------------------------------
// Fonction  : Retourne le libellÃ© d'une voie d'un tronÃ§on au
//             sens EVE
// Remarque  :
// Version du: 06/03/09
// Historique: 06/03/09 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//=================================================================
(
    Tuyau   *pTuyau,
    int     nVoie           // numÃ©rotÃ© Ã  partir de 0
)
{
    if(pTuyau->getNbVoiesDis() == 1)
        return pTuyau->GetLabel();

    return pTuyau->GetLabel() + "V" + SystemUtil::ToString(nVoie+1);
}

//=================================================================
eveShared::EveTroncon * Reseau::CreateEveTroncon
//----------------------------------------------------------------
// Fonction  : Retourne un tronÃ§on au sens d'EVE
//
// Remarque  :
// CrÃ©ation  : 23/08/10
//=================================================================
(
    const std::string &strID,
    const Point& PtAm,
    const Point& PtAv,
    const std::deque<Point*>& lstPtInterne,
    double dbLarg,
    double dbLong,
    const std::string& sSymTroncon,
    int nSymVoie,
    std::vector<ZoneZLevel> nzlevels,
    bool bTronconInterne
)
{
    eveShared::EveTroncon * pT = new eveShared::EveTroncon();
    pT->id = strID;
    pT->largeur = dbLarg;
    pT->longueur = dbLong;
    pT->sym_troncon = sSymTroncon;
    pT->interne = bTronconInterne;
    pT->sym_voie = nSymVoie;

    double * pCoord;

    pCoord = new double[3];
    pCoord[0] = PtAm.dbX;
    pCoord[1] = PtAm.dbY;
    pCoord[2] = PtAm.dbZ;
    pT->polyline.push_back(pCoord);

    std::deque<Point*>::const_iterator itBeg, itEnd, itPt;
    itBeg = lstPtInterne.begin();
    itEnd = lstPtInterne.end();
    for (itPt = itBeg; itPt != itEnd; itPt++)
    {
        pCoord = new double[3];
        pCoord[0] = (*itPt)->dbX;
        pCoord[1] = (*itPt)->dbY;
        pCoord[2] = (*itPt)->dbZ;
        pT->polyline.push_back(pCoord);
    }

    pCoord = new double[3];
    pCoord[0] = PtAv.dbX;
    pCoord[1] = PtAv.dbY;
    pCoord[2] = PtAv.dbZ;
    pT->polyline.push_back(pCoord);

    for(size_t i = 0; i < nzlevels.size(); i++)
    {
        eveShared::LevelCrossing * pLvlCrossing = new eveShared::LevelCrossing;
        pLvlCrossing->start = nzlevels[i].dbPosDebut;
        pLvlCrossing->end = nzlevels[i].dbPosFin;
        pLvlCrossing->zlevel = nzlevels[i].nZLevel;
        pT->LevelCrossings.push_back(pLvlCrossing);
    }


    return pT;
}

//=================================================================
    void Reseau::GenReseauCirculationFile
//----------------------------------------------------------------
// Fonction  : GÃ©nÃ©ration du 'rÃ©seau circulation' - intÃ©gration EVE
// Remarque  :
// Version du: 23/08/10
// Historique: 06/03/09 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//=================================================================
(
    eveShared::EveNetwork * &pNetwork	// Classe du rÃ©seau EVE
)
{
    if (pNetwork == NULL)
    {
        pNetwork = new eveShared::EveNetwork();
    }
    // TronÃ§ons EVE
    std::deque <Tuyau*>::iterator itTuy;
    std::deque <Tuyau*>::iterator itTuyD = m_LstTuyaux.begin();
    std::deque <Tuyau*>::iterator itTuyF = m_LstTuyaux.end();

    for(itTuy = itTuyD; itTuy != itTuyF; itTuy++)
    {
        Tuyau*  pT = (*itTuy);
        std::string sT = pT->GetLabel();
        eveShared::EveTroncon * pET = NULL;
        bool bTronconInterne = pT->GetBriqueParente() != NULL;

        if(pT->getNbVoiesDis() == 1)
        {
            if( !bTronconInterne )   // TronÃ§on utilisateur
                pET = CreateEveTroncon(sT, *pT->GetExtAmont(), *pT->GetExtAval(), pT->GetLstPtsInternes(), pT->getLargeurVoie(0), pT->GetLength(), sT, 1, pT->GetZLevelCrossings(), bTronconInterne );
            else
            {                               // TronÃ§on interne d'une brique
                std::string sCnx = pT->GetBriqueParente()->GetID();
                // on construit une zone de zLevel
                ZoneZLevel zoneZLvl;
                zoneZLvl.dbPosDebut = 0;
                zoneZLvl.dbPosFin = pT->GetLength();
                zoneZLvl.nZLevel = pT->GetBriqueParente()->GetZLevel();
                std::vector<ZoneZLevel> lstZonesZLvl;
                lstZonesZLvl.push_back(zoneZLvl);
                pET = CreateEveTroncon(sT, *pT->GetExtAmont(), *pT->GetExtAval(), pT->GetLstPtsInternes(), pT->getLargeurVoie(0), pT->GetLength(), sT, 1, lstZonesZLvl, bTronconInterne );
            }
            pNetwork->eveTroncons[sT] = pET;
        }
        else
        {
            for(int i=0; i<pT->getNb_voies(); i++)
            {
                Voie *pV = pT->GetLstLanes()[i];

                std::string sV = GetLibelleTronconEve(pT, i);

                pET = CreateEveTroncon(sV, *pV->GetExtAmont(), *pV->GetExtAval(), pV->GetLstPtsInternes(), pT->getLargeurVoie(i), pV->GetLength(), sT, i+1, pT->GetZLevelCrossings(), bTronconInterne);
                pNetwork->eveTroncons[sV] = pET;
            }
        }
    }
}

int Reseau::SendSignalPlan(const std::string & sCDF, const std::string & sSP)
{
    ControleurDeFeux    *pCDF = NULL;
    PlanDeFeux          *pPDF = NULL;

    pCDF = GetTrafficLightControllerFromID(sCDF);
    if(!pCDF)
        return -2;

    return pCDF->SendSignalPlan(sSP);

}

//=================================================================
    int Reseau::SendSpeedLimit
//----------------------------------------------------------------
// Fonction  : Pilotage de la vitesse rÃ©glementaire d'un tronÃ§on
// Remarque  :
// Version du: 10/11/09
// Historique: 10/11/09 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//=================================================================
(
    const std::string & sSection,			// identifiant du tronÃ§on
    const std::string & sVehType,			// identifiant du type de vÃ©hicule (si chaÃ®ne vide, le pilotage concerne tous les vÃ©hicules)
    int    numVoie,                         // numÃ©ro de voie concernÃ©e : -1 : toutes les voies
    double dbDebutPortion,                  // dÃ©but de la portion Ã  partir du dÃ©but du tronÃ§on (en m)
    double dbFinPortion,                    // fin de la portion Ã  partir du dÃ©but du tronÃ§on (en m)
    double dbSpeedLimit				        // vitesse rÃ©glementaire Ã  imposer
)
{
    Tuyau		*pT = NULL;
    TypeVehicule *pTV = NULL;
    int			nRes = 0;

    pT = GetLinkFromLabel( sSection );
    if(!pT)
    {
        nRes = -2;
    }

    if(dbSpeedLimit<0)
        nRes = -4;

    if( sVehType.size() != 0)
    {
        pTV = GetVehicleTypeFromID(sVehType);
        if( !pTV )
            nRes = -5;
    }

    if(nRes >= 0)		// Tout est OK
    {
        pT->SendSpeedLimit(m_dbInstSimu, pTV, numVoie, dbDebutPortion, dbFinPortion, dbSpeedLimit);
    }

    // ATTENTION, le rÃ©seau d'affectation devrait Ãªtre recalculÃ© ici !!!


    return nRes;
}


void Reseau::GenerateAssignmentNetwork()
{
    std::deque<Tuyau*>::iterator itT;
    BriqueDeConnexion *pBrique;

    m_dbSumLengthTrAssNet = 0;

    for(itT=m_LstTuyaux.begin(); itT!= m_LstTuyaux.end(); itT++)
    {
        if( (*itT)->GetType() != Tuyau::TT_MACRO)
            {


            if( !(*itT)->GetBriqueParente() )
                m_dbSumLengthTrAssNet += (*itT)->GetLength();

            pBrique = (*itT)->GetBriqueParente();			// Traitement des tronÃ§ons internes d'une brique de connection
            if( pBrique )
            {
                if( pBrique->GetType()=='C' || pBrique->GetType()=='G' ) 				// Traitement des briques de connection d'un CAF
                {
                    break;									// TronÃ§ons non pris en compte pour le rÃ©seau d'affectation
                }
                else
                {
                    (*itT)->SetCnxAssAm( (*itT)->getConnectionAmont() );
                    (*itT)->SetCnxAssAv( (*itT)->getConnectionAval() );

                    (*itT)->getConnectionAmont()->AddEltAvalAss( (*itT) );
                    (*itT)->getConnectionAval()->AddEltAmontAss( (*itT) );

                    break;
                }
            }

            pBrique = (*itT)->GetBriqueAval();				// Traitement des tronÃ§ons avec une brique de connection en aval
            if(pBrique)
            {
                if( pBrique->GetType()=='C' || pBrique->GetType()=='G')
                {
                    (*itT)->SetCnxAssAv( pBrique );
                    pBrique->AddEltAmontAss( (*itT) );
                }
                else
                {
                    (*itT)->SetCnxAssAv( (*itT)->getConnectionAval() );
                    (*itT)->getConnectionAval()->AddEltAmontAss( (*itT) );
                }
            }
            else
            {
                (*itT)->SetCnxAssAv( (*itT)->getConnectionAval() );
                (*itT)->getConnectionAval()->AddEltAmontAss( (*itT) );
            }

            pBrique = (*itT)->GetBriqueAmont();				// Traitement des tronÃ§ons avec une brique de connection en amont
            if(pBrique)
            {
                if( pBrique->GetType()=='C' || pBrique->GetType()=='G' )
                {
                    (*itT)->SetCnxAssAm( pBrique );
                    pBrique->AddEltAvalAss( (*itT) );
                }
                else
                {
                    (*itT)->SetCnxAssAm( (*itT)->getConnectionAmont() );
                    (*itT)->getConnectionAmont()->AddEltAvalAss( (*itT) );
                }
            }
            else
            {
                (*itT)->SetCnxAssAm( (*itT)->getConnectionAmont() );
                (*itT)->getConnectionAmont()->AddEltAvalAss( (*itT) );
            }
        }// process only if meso or micro tuyau
    }
}

//================================================================
    bool Reseau::CalculTraversee
//----------------------------------------------------------------
// Fonction  :
// Remarque  :
// Version du:
// Historique:
//================================================================
(
    Vehicule                *pVehEnAttente,         // VÃ©hicule candidat Ã  l'insertion
    std::vector<int>        &vehiculeIDs,           // Liste des identifiants des vÃ©hicules dÃ©jÃ  calculÃ©s (pour dÃ©tecter les boules infinies)
    PtConflitTraversee      *pPtCT,                 // Point de conflit traversÃ©e
    GrpPtsConflitTraversee  *pGrpPtsCT,             // Groupe d'appartenance du point de conflit
    double                  dbTf,
    double                  dbInstant,
    bool                    bDebutPasTemps,         // Calcul en utilisant les infos disponibles pour le vehicule en debut de pas de temps
    double                  dbTt                    // temps de traversÃ©e additionel
)
{
    boost::shared_ptr<Vehicule> pVehPrioritaire;
    double      dbDstVehPrio, dbDstVehAtt;
    double      dbtM, dbtm, dbtmAtt;

    // Calcul de la voie prioriaire
    VoieMicro *pTPrio, *pTNPrio;
    double dbDstPrio, dbDstNPrio;
    if( pPtCT->pT1 == pPtCT->pTProp )
    {
        pTPrio = pPtCT->pT1;
        pTNPrio = pPtCT->pT2;
        dbDstPrio = pPtCT->dbPos1;
        dbDstNPrio = pPtCT->dbPos2;
    }
    else
    {
        pTPrio = pPtCT->pT2;
        pTNPrio = pPtCT->pT1;
        dbDstPrio = pPtCT->dbPos2;
        dbDstNPrio = pPtCT->dbPos1;
    }

    // Recherche du vÃ©hicule sur le tronÃ§on prioritaire
    pVehPrioritaire = GetNearAmontVehiculeEx( (Tuyau*)pTPrio->GetParent(), dbDstPrio, (Tuyau*)pTPrio->GetParent(), GetTimeStep() * GetMaxVitMax(), pTPrio->GetNum() ); // recherche sur le tuyau prioritaire puis sur le tuyan en amont si la connexion est un divergent

    if(pVehPrioritaire) // Elimination des vÃ©hicules prioritaire qui ont Ã©tÃ© crÃ©Ã© au cours du pas de temps courant
        if(!pVehPrioritaire->GetLink(1))
            pVehPrioritaire.reset();

    // Si on n'a pas calculÃ© la nouvelle position du vÃ©hicule prioritaire et qu'on travaille en fin de pas de temps,
    // on appelle son calcul
    if(pVehPrioritaire && !bDebutPasTemps && !pVehPrioritaire->IsDejaCalcule())
    {
        if(std::find(vehiculeIDs.begin(), vehiculeIDs.end(), pVehPrioritaire->GetID()) == vehiculeIDs.end())
        {
            pVehPrioritaire->CalculTraficEx(dbInstant, vehiculeIDs);
        }
        else
        {
            // Dans ce cas (en principe trÃ¨s rare), impossible de dÃ©terminer s'il y a traversÃ©e : on ne gÃ¨re pas l'Ã©ventuelle traversÃ©e.
            log() << Logger::Warning << std::endl << "Potential crossing case ignored for vehicle " << pVehEnAttente->GetID() << " caused by a loop in the vehicles processing...";
            log() << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
            return true;
        }
    }

    // Calcul du temps prÃ©visionnel pour que le vÃ©hicule prioritaire atteigne le point de conflit
    if(pVehPrioritaire && (bDebutPasTemps || pVehPrioritaire->GetLink(0))) // CP du vÃ©hicule particulier prioritaire qui sort du rÃ©seau pendant le pas de temps : on l'ignore
    {
        // Calcul de la distance Ã  parcourir pour le vÃ©hicule prioritaire avant d'atteindre le point de conflit au dÃ©but du pas de temps
        if( pVehPrioritaire->GetLink(1) == pTPrio->GetParent() )    // Le vÃ©hicule prioritaire est dÃ©jÃ  sur le tronÃ§on prioritaire
            dbDstVehPrio = dbDstPrio - pVehPrioritaire->GetPos(1);
        else
            dbDstVehPrio = dbDstPrio + pVehPrioritaire->GetVoie(1)->GetLength() - pVehPrioritaire->GetPos(1);

        dbtM = pVehPrioritaire->GetTempsDistance( dbInstant, dbDstVehPrio, pas_de_temps, bDebutPasTemps );
    }
    // Calcul du temps prÃ©visonnel pour que le vÃ©hicule en attente atteigne le point de conflit

    // Calcul de la distance Ã  parcourir pour le vÃ©hicule en attente avant d'atteindre le point de conflit au dÃ©but du pas de temps
    if( pVehEnAttente->GetLink(1) == pTNPrio->GetParent() )    // Le vÃ©hicule en attente est dÃ©jÃ  sur le tronÃ§on du point d'attente
        dbDstVehAtt = dbDstNPrio - pVehEnAttente->GetPos(1);
    else
    {
        if(pVehEnAttente->GetLink(1))
        {
            dbDstVehAtt = dbDstNPrio + pVehEnAttente->GetLink(1)->GetLength() - pVehEnAttente->GetPos(1);
        }
        else
        {
            return true; // on ignore la traversÃ©e
        }
    }

    // Prise en compte de la longueur du vÃ©hicule
    dbDstVehAtt += pVehEnAttente->GetLength();

    dbtm = pVehEnAttente->GetTempsDistance( dbInstant, dbDstVehAtt, pas_de_temps, bDebutPasTemps ) + dbTt;

    if( pVehPrioritaire && (bDebutPasTemps || pVehPrioritaire->GetLink(0))) // CP du vÃ©hicule particulier prioritaire qui sort du rÃ©seau pendant le pas de temps : on l'ignore
    {
        if( dbtm < dbtM )   // Le vÃ©hicule passe
        {
            pVehPrioritaire.reset();
            return true;
        }
        else
        {
            return false;
        }
    }
    else // Pas de vÃ©hicule prioritaire gÃ©nant
    {
        // Calcul du temps nÃ©cessaire pour atteindre le point d'attente
        if( !pVehEnAttente->IsDejaPasse( (Tuyau*)pGrpPtsCT->pVPtAttente->GetParent(), pGrpPtsCT->dbPosPtAttente ) )
        {
             if( pVehEnAttente->GetVoie(1) == pGrpPtsCT->pVPtAttente )    // Le vÃ©hicule en attente est dÃ©jÃ  sur le tronÃ§on du point d'attente
                dbDstVehAtt = pGrpPtsCT->dbPosPtAttente - pVehEnAttente->GetPos(1);
            else
                dbDstVehAtt = pGrpPtsCT->dbPosPtAttente + pVehEnAttente->GetLink(1)->GetLength() - pVehEnAttente->GetPos(1);

             dbtmAtt = pVehEnAttente->GetTempsDistance( dbInstant, dbDstVehAtt, pas_de_temps, bDebutPasTemps );
        }
        else // le point d'attente a dÃ©jÃ  Ã©tÃ© passÃ©
        {
            return true;
        }

        // VÃ©rification du critÃ¨re sur le temps Ã©coulÃ© depuis la derniÃ¨re traversÃ©e
        if( dbInstant - GetTimeStep() + dbtmAtt - pGrpPtsCT->dbInstLastTraversee >= dbTf || dbInstant  - GetTimeStep() + dbtmAtt <= pGrpPtsCT->dbInstLastTraversee )
        {
            pGrpPtsCT->dbInstLastTraversee = dbInstant - GetTimeStep() + dbtmAtt;
            return true;
        }
        else
        {
           return false;
        }
    }
}

//================================================================
    void Reseau::ProcedureAgressivite
//----------------------------------------------------------------
// Fonction  : Appel de la procÃ©dure de gestion de l'agressivitÃ©
//			   des vÃ©hicules
// Remarque  :
// Version du:
// Historique:
//================================================================
(
    double dbInstant
)
{
    std::vector<boost::shared_ptr<Vehicule>>::iterator itVeh;

    for(itVeh = m_LstVehicles.begin(); itVeh != m_LstVehicles.end(); itVeh++)
    {
        if( (*itVeh)->IsAgressif() )
            (*itVeh)->GetCarFollowing()->CalculAgressivite(dbInstant);
    }
}
    ///<summary>
    /// Chargement d'une liste de vÃ©hicule initiale
    ///</summary>
    ///<param name="pXmlNodeInit">Noeud XML d'accÃ¨s Ã  la liste initiale des vÃ©hicules
    ///</param>
    ///<returns>void</returns>
    void Reseau::LoadInitVehs( DOMNode *pXmlNodeInit, Logger &logger)
    {
        DOMNode * pXmlNodeTrajs;
        int nID, nVoie;
        std::string sTr, sTV, sDestination;
        double dbPos, dbVit, dbAcc;
        SymuViaTripNode *pDestination;

        if(!pXmlNodeInit)	// Aucun vÃ©hicule initial
            return;

        pXmlNodeTrajs = m_pXMLUtil->SelectSingleNode( "./TRAJS", pXmlNodeInit->getOwnerDocument(), (DOMElement*)pXmlNodeInit);
        if(!pXmlNodeTrajs)		// Aucun vÃ©hicule initial
            return;

        XMLSize_t counti = pXmlNodeTrajs->getChildNodes()->getLength();
        for(XMLSize_t i=0; i<counti; i++)
        {
            DOMNode * xmlNode = pXmlNodeTrajs->getChildNodes()->item(i);
            if (xmlNode->getNodeType() != DOMNode::ELEMENT_NODE) continue;

            // Chargement id
            GetXmlAttributeValue(xmlNode, "id", nID, &logger);

            // Chargement tronÃ§on
            GetXmlAttributeValue(xmlNode, "tron", sTr, &logger);

            // Chargement numÃ©ro de voie
            GetXmlAttributeValue(xmlNode, "voie", nVoie, &logger);

            // Chargement position
            GetXmlAttributeValue(xmlNode, "dst", dbPos, &logger);

            // Chargement vitesse
            GetXmlAttributeValue(xmlNode, "vit", dbVit, &logger);

            // Chargement acc
            GetXmlAttributeValue(xmlNode, "acc", dbAcc, &logger);

            // Type de vÃ©hicule
            GetXmlAttributeValue(xmlNode, "type_veh", sTV, &logger);

            // Destination
            GetXmlAttributeValue(xmlNode, "destination", sDestination, &logger);

            // VÃ©rifications de l'intÃ©gritÃ© des donnÃ©es
            Tuyau* pT = GetLinkFromLabel(sTr);
            if(!pT || pT->IsMacro())
                break;

            if(pT->getNb_voies() < nVoie )
                break;

            TypeVehicule *pTV = this->GetVehicleTypeFromID(sTV);
            if(!pTV)
                break;

            if( pT->GetLength() < dbPos )
                break;

            if( IsCptDestination() || IsCptItineraire() )
            {
                if( sDestination.size() == 0)
                    break;
                char cDestinationType;
                pDestination = GetDestinationFromID(sDestination, cDestinationType);
            }

            // CrÃ©ation du vÃ©hicule
            boost::shared_ptr<Vehicule> pVehicule = boost::make_shared<Vehicule>(this, nID, 0, pTV, pas_de_temps);
            pVehicule->InitializeCarFollowing();

            // Ajout Ã  la liste des vÃ©hicules
            m_LstInitVehicule.push_back( pVehicule );

            // MAJ de ses caractÃ©ristiques
            pVehicule->SetTuyau((TuyauMicro*)pT, 0);
            pVehicule->SetVoie( (VoieMicro*)pT->GetLstLanes()[nVoie-1]);
            pVehicule->SetPos(dbPos);
            pVehicule->SetVit(dbVit);
            pVehicule->SetAcc(dbAcc);

            if( IsCptDestination() || IsCptItineraire() )
            {
                Trip * pTrip = new Trip();
                TripLeg * pTripLeg = new TripLeg();
                pTripLeg->SetDestination(pDestination);
                pTrip->AddTripLeg(pTripLeg);
                pVehicule->SetTrip(pTrip);
                pVehicule->MoveToNextLeg();
            }
        }
    }

    bool Reseau::LoadMouvementsAutorises(Connexion *pCnx, DOMNode *xmlNodeMvtsAutorises, Logger * pLogger)
    {
        return pCnx->LoadMouvementsAutorises(xmlNodeMvtsAutorises, pLogger);
    }

    boost::shared_ptr<Vehicule> Reseau::GetVehiculeFromID(int nID)
    {
        std::vector<boost::shared_ptr<Vehicule>>::iterator itV;

        for(itV = m_LstVehicles.begin(); itV != m_LstVehicles.end(); itV++)
            if( (*itV)->GetID() == nID )
                return (*itV);

        return boost::shared_ptr<Vehicule>();
    }

    boost::shared_ptr<Vehicule> Reseau::GetVehiculeFromIDThreadSafe(int nID)
    {
        boost::lock_guard<boost::mutex> guard(*m_pMutex);

        std::vector<boost::shared_ptr<Vehicule>>::iterator itV;

        for (itV = m_LstVehicles.begin(); itV != m_LstVehicles.end(); itV++)
        if ((*itV)->GetID() == nID)
            return (*itV);

        return boost::shared_ptr<Vehicule>();
    }

// Chargement des donnÃ©es d'un plan de feux Ã  partir du noeud XML
bool Reseau::LoadDataPlanDeFeux(ControleurDeFeux *pCtrDeFeux, PlanDeFeux *pPDF, DOMNode *pXMLPlansDeFeux, Logger *pChargement)
{
    double dbTmp;
    double dbDuree = 0;
    // Lecture des sÃ©quences du plan de feux
    DOMNode *pXMLSequences = m_pXMLUtil->SelectSingleNode("./SEQUENCES", pXMLPlansDeFeux->getOwnerDocument(), (DOMElement*)pXMLPlansDeFeux);

    XMLSize_t countk = pXMLSequences->getChildNodes()->getLength();
    for(XMLSize_t k=0; k<countk;k++)
    {
        DOMNode *pXMLSequence = pXMLSequences->getChildNodes()->item(k);
        if (pXMLSequence->getNodeType() != DOMNode::ELEMENT_NODE) continue;

        // DurÃ©e totale
        GetXmlAttributeValue(pXMLSequence, "duree_totale", dbTmp, pChargement);
        dbDuree += dbTmp;

        CDFSequence *pSequence = new CDFSequence(dbTmp, (int)k);

        // Lecture des signaux actifs
        DOMNode *pXMLSignaux = m_pXMLUtil->SelectSingleNode("./SIGNAUX_ACTIFS", pXMLSequence->getOwnerDocument(), (DOMElement*)pXMLSequence);
        //int countl = pXMLSignaux->ChildNodes->Count;
        XMLSize_t countl = pXMLSignaux->getChildNodes()->getLength();
        for(XMLSize_t l=0; l<countl;l++)
        {
            DOMNode *pXMLSignal = pXMLSignaux->getChildNodes()->item(l);
            if (pXMLSignal->getNodeType() != DOMNode::ELEMENT_NODE) continue;

            double dbDureeVert, dbDureeOrange, dbDureeRetardAllumage;
            std::string sTE, sTS;
            Tuyau *pTE, *pTS;

            // TronÃ§on d'entrÃ©e
            GetXmlAttributeValue(pXMLSignal, "troncon_entree", sTE, pChargement);
            pTE = GetLinkFromLabel(sTE);

            // TronÃ§on de sortie
            GetXmlAttributeValue(pXMLSignal, "troncon_sortie", sTS, pChargement);
            pTS = GetLinkFromLabel(sTS);

            // VÃ©rification de la validitÃ© du signal actif
            if(pTE == NULL || pTS == NULL)
            {
                *pChargement << Logger::Warning << "WARNING : the links " << sTE << " and " << sTS << " defined for an active signal don't exist. The signal is ignored.";
                *pChargement << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                continue;
            }

            // DurÃ©e du vert
            GetXmlAttributeValue(pXMLSignal, "duree_vert", dbDureeVert, pChargement);

            // DurÃ©e de l'orange
            GetXmlAttributeValue(pXMLSignal, "duree_orange", dbDureeOrange, pChargement);

            // Retard Ã  l'allumage
            GetXmlAttributeValue(pXMLSignal, "duree_retard_allumage", dbDureeRetardAllumage, pChargement);

            SignalActif *pSignal = new SignalActif(pTE, pTS, dbDureeVert, dbDureeOrange, dbDureeRetardAllumage);

            // Ajout du couple traitÃ©
            pCtrDeFeux->AddCoupleEntreeSortie(pTE, pTS);

            // Ajout Ã  la sÃ©quence
            pSequence->AddActiveSignal(pSignal);
        }

        // Ajout au plan de feux
        pPDF->AddSequence( pSequence );
    }

    return true;
}


// Chargement de la demande et agressivitÃ© d'une origine
bool Reseau::LoadDemandesAgressivite(SymuViaTripNode * pOrigine, DOMNode *pXMLTraficOrigine, double dbLag, Logger * pChargement)
{
    //double              dbTmp, dbDuree;
    std::string         sMsgErr;
    std::string         strTmp;
    std::deque<TypeVehicule * >::const_iterator itTV;
    // Evolution nÂ°79 : mode alternatif de saisie des matrices OD (par type de vÃ©hicule)
    DOMNode * pFluxGlobal = m_pXMLUtil->SelectSingleNode("./FLUX_GLOBAL", pXMLTraficOrigine->getOwnerDocument(), (DOMElement*)pXMLTraficOrigine);
    if( pFluxGlobal ) // traitement du flux globalement
    {
        // Demandes
        if( !LoadDemandeFromFluxNode(pOrigine, dbLag, pChargement, pFluxGlobal, NULL) )
        {
            return false;
        }

    }
  else
    {   //traitement individuel des flux par type de vÃ©hicule
         DOMNode * pFluxParType = m_pXMLUtil->SelectSingleNode("./FLUX_TYPEVEHS", pXMLTraficOrigine->getOwnerDocument(), (DOMElement*)pXMLTraficOrigine);
         if( pFluxParType )
         {
             XMLSize_t nFluxType =  pFluxParType->getChildNodes()->getLength();
             // on parcours les flux pour tous les types de vÃ©hicule
             for (XMLSize_t iFluxType = 0; iFluxType < nFluxType; ++iFluxType)
             {
                 DOMNode *pFluxNode = pFluxParType->getChildNodes()->item(iFluxType);
                 // Obtient l'identifiant du Type de Vehicule
                 string strTypeVeh = "";
                 GetXmlAttributeValue(pFluxNode, "id_typevehicule", strTypeVeh, pChargement);
                 if ( GetVehicleTypeFromID(strTypeVeh) )
                 {
                   if (!LoadDemandeFromFluxNode( pOrigine, dbLag, pChargement,
                                        pFluxNode, GetVehicleTypeFromID(strTypeVeh)))
                   {
                       return false;
                   }
                 }


             } //rof each type flux
         }

    }

    // AgressivitÃ©
    DOMNode *pXMLAgressivites = m_pXMLUtil->SelectSingleNode("./AGRESSIVITES", pXMLTraficOrigine->getOwnerDocument(), (DOMElement*)pXMLTraficOrigine);
    if(pXMLAgressivites)
    {
        XMLSize_t counti = pXMLAgressivites->getChildNodes()->getLength();
        for(XMLSize_t i=0; i<counti;i++)
        {
            DOMNode * xmlChildi = pXMLAgressivites->getChildNodes()->item(i);
            if (xmlChildi->getNodeType() != DOMNode::ELEMENT_NODE) continue;

            GetXmlAttributeValue(xmlChildi, "type_veh", strTmp, pChargement);
            TypeVehicule *pTV = GetVehicleTypeFromID(strTmp);

            if(pTV)
            {
                double dbTmp;
                GetXmlAttributeValue(xmlChildi, "taux", dbTmp, pChargement);
                pOrigine->AddAgressivite(pTV, dbTmp);
            }
        }
    }

    return true;
}

bool Reseau::LoadDemandeFromFluxNode(SymuViaTripNode * pOrigine, double dbLag, Logger * pChargement,
                            DOMNode * pFluxNode, TypeVehicule * pTypeVehicle)
{

    double   dbDuree;
  // Demandes
    DOMNode * pXMLDemandes = m_pXMLUtil->SelectSingleNode("./FLUX/DEMANDES", pFluxNode->getOwnerDocument(), (DOMElement*)pFluxNode);

    if(pXMLDemandes) // Utilisation de l'attribut demande (niveau de demande constant tout au long de la simulation)
    {
        // Chargement du dÃ©calage temporel des variations
        pOrigine->GetLstDemandeInit(pTypeVehicle)->SetLag( dbLag );

        XMLSize_t countj = pXMLDemandes->getChildNodes()->getLength();
        for(XMLSize_t j=0; j<countj;j++)
        {
            // Chargement de la variante temporelle de la demande
            DOMNode * xmlChildj = pXMLDemandes->getChildNodes()->item(j);
            if (xmlChildj->getNodeType() != DOMNode::ELEMENT_NODE) continue;

            vector<PlageTemporelle*> plages;
            if(!GetXmlDuree(xmlChildj,this,dbDuree,plages, pChargement))
            {
                return false;
            }

            boost::shared_ptr<tracked_double> pTrackedDouble = boost::make_shared<tracked_double>();
            GetXmlAttributeValue(xmlChildj, "niveau", *pTrackedDouble, pChargement);

            if(plages.size() > 0)
            {
                for(size_t iPlage = 0; iPlage < plages.size(); iPlage++)
                {
                    pOrigine->GetLstDemandeInit(pTypeVehicle)->AddVariation( plages[iPlage], pTrackedDouble );
                }
            }
            else
            {
               pOrigine->GetLstDemandeInit(pTypeVehicle)->AddVariation( dbDuree, pTrackedDouble );

            }
        }

        // vÃ©rification de la couverture des plages temporelles
        if(!pOrigine->GetLstDemandeInit(pTypeVehicle)->CheckPlagesTemporelles(m_dbDureeSimu))
        {
            *pChargement << Logger::Error << "ERROR : The time frames defined for the demand of the input " << pOrigine->GetOutputID() << " don't cover the whole simulation duration !" << std::endl;
            *pChargement << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
            return false;
        }

    }
    return true;
}

// Chargement d'un noeud REP_TYPEVEHICULES
bool Reseau::LoadRepTypeVehicules(XERCES_CPP_NAMESPACE::DOMNode *pXMLNode, const std::string & strElement, int nbCoeffs, const std::vector<TypeVehicule*> & lstTypes, RepartitionTypeVehicule * pRepTypesVeh, Logger * pChargement)
{
    std::vector<std::vector<double> > coeffs;
    double dbDuree;

    if(pXMLNode)
    {
        // Pour chaque variante temporelle ...
        XMLSize_t countj = pXMLNode->getChildNodes()->getLength();
        DOMNode * pXMLRepartition;
        for(XMLSize_t j=0; j<countj;j++)
        {
            pXMLRepartition = pXMLNode->getChildNodes()->item(j);
            if (pXMLRepartition->getNodeType() != DOMNode::ELEMENT_NODE) continue;

            coeffs.resize(m_LstTypesVehicule.size());

            for(size_t k=0; k<coeffs.size(); k++)
                coeffs[k].resize(nbCoeffs);

            // Duree
            vector<PlageTemporelle*> plages;
            if(!GetXmlDuree(pXMLRepartition,this,dbDuree,plages, pChargement))
            {
                return false;
            }

            // Liste des coefficients ordonnÃ©s
            std::string strTmp;
            GetXmlAttributeValue(pXMLRepartition, "coeffs", strTmp, pChargement);
            std::deque<std::string> split = SystemUtil::split(strTmp, ' ');
            if (split.size() != m_LstTypesVehicule.size())
            {
                *pChargement << Logger::Error << "ERROR : the number of values for the 'coeffs' attribute of the REP_TYPEVEHICULE node (element : " << strElement;
                *pChargement << Logger::Error << " - repartition : "<< j+1 << " ) is incorrect (must be equal to the number of vehicle types).";
                *pChargement << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                return false;
            }
            double dbS = 0;
            int nS = 0;
            for(size_t l=0; l<m_LstTypesVehicule.size();l++)
            {
                // dans le cas global on module le coeff de repartition des type de vehicule par le coeff de repartition des voies
                // ici on stock pour faire la modulation durant l'affectation aux variations temporelles
                for( int iCoeffWay = 0; iCoeffWay < nbCoeffs; ++iCoeffWay)
                {

                    coeffs[l][iCoeffWay] = atof( split.at(l).c_str() );
                    dbS += coeffs[l][iCoeffWay]/ nbCoeffs ;
                    nS++;
                }
            }

            // VÃ©rification que la somme des coeff. soit Ã©gale Ã  1
            if( fabs(dbS-1) > nS*std::numeric_limits<double>::epsilon() )
            {
                *pChargement << Logger::Error << "ERROR : the sum of the values of the 'valeurs' attribute of the REP_TYPEVEHICULE node (element : " << strElement;
                *pChargement << Logger::Error << " - repartition : "<< j+1 << " ) is incorrect (must be 1).";
                *pChargement << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                return false;
            }

            if(plages.size() > 0)
            {
                for(size_t iPlage = 0; iPlage < plages.size(); iPlage++)
                {
                    // crÃ©Ã©e une rÃ©partition par type par plage
                    pRepTypesVeh->AddVariation(coeffs, plages[iPlage]);
                }
            }
            else
            {
                // dans le cas global on module le coeff de repartition des type de vehicule par le coeff de repartition des voies
                pRepTypesVeh->AddVariation(coeffs, dbDuree);
            }
        }

        // VÃ©rification de la couverture temporelle
        vector<PlageTemporelle*> plages;
        for(size_t iPlage = 0; iPlage < pRepTypesVeh->GetLstCoefficients().size(); iPlage++)
        {
            PlageTemporelle * pPlage = pRepTypesVeh->GetLstCoefficients().at(iPlage).m_pPlage;
            if(pPlage)
            {
                plages.push_back(pPlage);
            }
        }
        if(plages.size() > 0 && !CheckPlagesTemporellesEx(m_dbDureeSimu, plages))
        {
            *pChargement << Logger::Error << "ERROR : The time frames defined for the vehicle types repartitions of element " << strElement << " don't cover the whole simulation duration !" << std::endl;
            *pChargement << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
            return false;
        }
    }
    else
    {
        // Si pas de noeud REP_TYPEVEHICULES, on construit une rÃ©partition uniforme avec la liste fournie.
        // Si liste vide, on prend le premier type de vÃ©hicule global avec un coeff 1

        coeffs.resize(m_LstTypesVehicule.size());

        for(size_t k=0; k<coeffs.size(); k++)
            coeffs[k].resize(nbCoeffs);

        for(size_t l=0; l<m_LstTypesVehicule.size();l++)
        {
            for( int iCoeffWay = 0; iCoeffWay < nbCoeffs; ++iCoeffWay)
            {
                double dbCoeff;
                // Cas d'aucune liste fournie : on met 1 pour le premier type de vÃ©hicule et 0 aux autres
                if(lstTypes.empty())
                {
                    dbCoeff = (l == 0 ? 1.0 : 0.0);
                }
                // sinon equirÃ©partition entre les types fournis
                else
                {
                    if(std::find(lstTypes.begin(), lstTypes.end(), m_LstTypesVehicule[l]) != lstTypes.end())
                    {
                        dbCoeff = 1.0 / (double)lstTypes.size();
                    }
                    else
                    {
                        dbCoeff = 0;
                    }
                }
                coeffs[l][iCoeffWay] = dbCoeff;
            }
        }

        pRepTypesVeh->AddVariation(coeffs, m_dbDureeSimu);
    }

    return true;
}

// Chargement des paramÃ¨tres de crÃ©ation et de rÃ©partition des vÃ©hicules crÃ©Ã©s
bool Reseau::LoadCreationRepartitions(SymuViaTripNode * pOrigine, DOMNode *pXMLTraficOrigine, Logger * pChargement)
{
    int              nVoie;
    std::string      strTmp;
    Tuyau           *pT;
    double          dbTmp;

    std::deque< TypeVehicule * >::const_iterator itTv;

    // Evolution nÂ°79 : mode alternatif de saisie des matrices OD (par type de vÃ©hicule)
     DOMNode * pFluxTypeVehs = m_pXMLUtil->SelectSingleNode("./FLUX_TYPEVEHS", pXMLTraficOrigine->getOwnerDocument(), (DOMElement*)pXMLTraficOrigine);
     DOMNode * pFluxGlobal = m_pXMLUtil->SelectSingleNode("./FLUX_GLOBAL", pXMLTraficOrigine->getOwnerDocument(), (DOMElement*)pXMLTraficOrigine);

     // Il ne peut y avoir un noeud global et un noeud type flux
     if( pFluxTypeVehs && pFluxGlobal)
     {
         *pChargement << Logger::Error << "ERROR : the " << pOrigine->GetOutputID() << " origin can't own both FLUX_TYPEVEHS and FLUX_GLOBAL nodes."<< std::endl;
         *pChargement << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
         return false;
     }
     // Creation des vÃ©hicules
//    if(!pFluxTypeVehs) ETS 141009 remove pkoi le garder?
    {
        // Mode de saisie classique

        GetXmlAttributeValue(pXMLTraficOrigine, "typeCreationVehicule", strTmp, pChargement);

        if(strTmp=="listeVehicules")
        {
            pOrigine->SetTypeDemande(false);
            pOrigine->SetTypeDistribution(false);

            // Assignation du pointeur sur le noeud de la liste
            DOMNode * pXMLLstVehicules = m_pXMLUtil->SelectSingleNode("./CREATION_VEHICULES", pXMLTraficOrigine->getOwnerDocument(), (DOMElement*)pXMLTraficOrigine);

            // Initialisation de la liste des destinations (pour construction des itinÃ©raires)
            XMLSize_t countj = pXMLLstVehicules->getChildNodes()->getLength();
            for(XMLSize_t j=0; j<countj; j++)
            {
                DOMNode * xmlChildj = pXMLLstVehicules->getChildNodes()->item(j);
                if (xmlChildj->getNodeType() != DOMNode::ELEMENT_NODE) continue;

                GetXmlAttributeValue(xmlChildj, "destination", strTmp, pChargement);
                char cDestType;
                SymuViaTripNode* pDst = GetDestinationFromID( strTmp, cDestType );
                if(pDst)
                {
                    GetXmlAttributeValue(xmlChildj, "typeVehicule", strTmp, pChargement);
                    TypeVehicule * pTV = GetVehicleTypeFromID(strTmp);
                    if(pTV)
                    {
                        GetXmlAttributeValue(xmlChildj, "instant", dbTmp, pChargement);
                        GetXmlAttributeValue(xmlChildj, "num_voie", nVoie, pChargement);
                        pOrigine->AddCreationVehicule(dbTmp, pTV, pDst, nVoie);
                    }
                    else
                    {
                        *pChargement << Logger::Error << "ERROR : the type of the vehicle to create at input " << pOrigine->GetOutputID() << " is invalid. "<< std::endl;
                        *pChargement << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                        return false;
                    }
                }
                else
                {
                    *pChargement << Logger::Error << "ERROR : the destination defined for the vehicle to create at input " << pOrigine->GetOutputID() << " is invalid. "<< std::endl;
                    *pChargement << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                    return false;
                }
            }
        }
        else if(strTmp=="distributionExponentielle")
        {
            pOrigine->SetTypeDemande(true);
            pOrigine->SetTypeDistribution(true);
        }
        // fi typeCreationVehicle
    } // fi !pRepParTypeNode
    // fin CREATION Vehicule


    if(pOrigine->IsTypeDemande())
    {
        // nombre de voies total
        int nbVoiesTotal = 0;
        for(size_t iTuy = 0; iTuy <  pOrigine->GetOutputConnexion()->m_LstTuyAv.size(); iTuy++)
        {
            pT = pOrigine->GetOutputConnexion()->m_LstTuyAv[iTuy];
            nbVoiesTotal += pT->getNbVoiesDis();
        }



        // VÃ©rification de la cohÃ©rence du dÃ©bit consigne par rapport Ã  Kx pour toutes les variantes temporelles
        ZoneDeTerminaison * pZone = dynamic_cast<ZoneDeTerminaison*>(pOrigine);
        // On ignore cette Ã©tape de validation si l'origine est une zone.
        if (!pZone)
        {
            if (pFluxGlobal)
            {
                for (int j = 0; j< (int)pOrigine->GetLstDemandeInit(NULL)->GetLstTV()->size(); j++)
                {

                    if (*pOrigine->GetLstDemandeInit(NULL)->GetLstTV()->at(j).m_pData / nbVoiesTotal > m_dbMaxDebitMax)
                    {
                        *pChargement << Logger::Error << "ERROR : the origin " << pOrigine->GetOutputID() << " has a demand value greater than the maximum fundamental diagram value." << std::endl;
                        *pChargement << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                        return false;
                    }
                }
            }
            else
            {
                if (pFluxTypeVehs)
                {
                    std::map<TypeVehicule*, ListOfTimeVariation<tracked_double>* > lstDemandeInit = pOrigine->GetLstDemandeInit();
                    std::map<TypeVehicule*, ListOfTimeVariation<tracked_double>* >::iterator itDemande;
                    size_t nLstTvCount = lstDemandeInit.begin()->second->GetLstTV()->size();

                    for (int j = 0; j< (int)nLstTvCount; j++)
                    {
                        double dataSum = 0;
                        for (itDemande = lstDemandeInit.begin(); itDemande != lstDemandeInit.end(); itDemande++)
                        {

                            dataSum += *itDemande->second->GetLstTV()->at(j).m_pData;

                        }// rof each type
                        if (dataSum / nbVoiesTotal > m_dbMaxDebitMax)
                        {
                            *pChargement << Logger::Error << "ERROR : the origin " << pOrigine->GetOutputID() << " has a demand value higher than the maximum flow of the fundamental diagram." << std::endl;
                            *pChargement << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                            return false;
                        }
                    }
                }
            }
        }
    }


    if(dynamic_cast<Entree*>(pOrigine) != NULL && pOrigine->GetOutputConnexion()->m_LstTuyAv.size() == 0)
    {
        *pChargement << Logger::Error << "ERROR : the input " << pOrigine->GetOutputID() << " is not connected to any link."<< std::endl;
        *pChargement << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
        return false;
    }

    if( pFluxGlobal)
    {
        DOMNode * pNodeFlux ;
        pNodeFlux = m_pXMLUtil->SelectSingleNode( "./FLUX", pFluxGlobal->getOwnerDocument(), (DOMElement*)pFluxGlobal);
        if( pNodeFlux )
        {
            if(!LoadFluxFromNode(pOrigine, pChargement, pNodeFlux,NULL))
            {
                return false;
            }
        }
        else
        {
            // pas de gestion de flux
            if(!LoadFluxFromNode(pOrigine, pChargement, NULL,NULL))
            {
                return false;
            }
        }
    }
    else
    {

        if( pFluxTypeVehs )
        {
            XMLSize_t nFluxType =  pFluxTypeVehs->getChildNodes()->getLength();
            // on parcours les flux pour tous les types de vÃ©hicule
            for (XMLSize_t iFluxType = 0; iFluxType < nFluxType; ++iFluxType)
            {
                DOMNode *pFluxNodeType = pFluxTypeVehs->getChildNodes()->item(iFluxType);
                // Obtient l'identifiant du Type de Vehicule
                string strTypeVeh = "";
                GetXmlAttributeValue(pFluxNodeType, "id_typevehicule", strTypeVeh, pChargement);

                if ( GetVehicleTypeFromID(strTypeVeh) )
                {
                    DOMNode * pNodeFlux ;
                    pNodeFlux = m_pXMLUtil->SelectSingleNode( "./FLUX", pFluxTypeVehs->getOwnerDocument(), (DOMElement*)pFluxNodeType);
                    if (!LoadFluxFromNode( pOrigine, pChargement,
                                    pNodeFlux, GetVehicleTypeFromID(strTypeVeh)))
                    {
                        return false;
                    }
                }

            } //rof each type flux
        }
        else
        {
            // pas de gestion de flux (should not be occur)
            if(!LoadFluxFromNode(pOrigine, pChargement, NULL,NULL))
            {
                return false;
            }
        }

    }
    // Gerer les type de repartition

    // RÃ©partition des types de vÃ©hicules
    int  nTypes = (int)m_LstTypesVehicule.size();

    // Nombre de listes de coefficients dÃ©finis (autant que de voies dans le cas ponctuel, une seule pour les zones)
    ZoneDeTerminaison * pZone = dynamic_cast<ZoneDeTerminaison*>(pOrigine);
    int nbCoeffs = pZone != NULL ? 1 : pOrigine->GetOutputConnexion()->m_LstTuyAv.front()->getNb_voies();
    if( pFluxGlobal )
    {
        // Chargement
        // Traitement des rÃ©partition de typeVÃ©hicules uniquement pour les flux "globaux"
        DOMNode *pXMLRepartitions = m_pXMLUtil->SelectSingleNode( "./REP_TYPEVEHICULES", pFluxGlobal->getOwnerDocument(), (DOMElement*)pFluxGlobal);

        if(pXMLRepartitions) // une rÃ©partition par type de vÃ©hicule a Ã©tÃ© dÃ©finie
        {                    // on est dans une gestion "globale du flux"

            // Chargement de la rÃ©partition des types de vÃ©hicules
            std::vector<TypeVehicule*> placeHolder;
            if(!LoadRepTypeVehicules(pXMLRepartitions, pOrigine->GetOutputID(), nbCoeffs, placeHolder, pOrigine->GetLstRepTypeVeh(), pChargement))
            {
                return false;
            }

            // Ajout des types de vÃ©hicules au tuyau
            for(size_t iTuy = 0; iTuy < pOrigine->GetOutputConnexion()->m_LstTuyAv.size(); iTuy++)
            {
                for(size_t k=1; k<m_LstTypesVehicule.size();k++)      // Ajout des autres types de vÃ©hicule que le type de base
                    pOrigine->GetOutputConnexion()->m_LstTuyAv[iTuy]->AddTypeVeh(m_LstTypesVehicule[k]);
            }
        }
        else // rien n'a Ã©tÃ© dÃ©fini (seul des vehicules de type de base seront gÃ©nÃ©rÃ©s)
        {
            int nNbVoie;
            if(pZone)
            {
                // cas de la zone : une seule liste de coeffs
                nNbVoie = 1;
            }
            else
            {
                pT = pOrigine->GetOutputConnexion()->m_LstTuyAv.front();
                nNbVoie = pT->getNbVoiesDis();
            }
            std::vector<std::vector<double> > coeffs(nTypes);

            for(int j=0; j<nTypes; j++)
            {
                coeffs[j].resize(nNbVoie);
                // Par defaut on ne sort que le premier type
                for( int iCoeffWay = 0; iCoeffWay < nNbVoie; ++iCoeffWay)
                {
                    coeffs[j][iCoeffWay]=(j==0)?(1.0):0;
                }
            }
            pOrigine->GetLstRepTypeVeh()->AddVariation(coeffs, m_dbDureeSimu); // gestion "globale du flux"
        } // fi else il n'y a pas de noeud rep_vehiculeparType
    } // fi on est dans un cas global
    // Fin traitement REP_TYPEVEHICULES
    else
    { // sinon on n'est pas dans une distribution globale -> on dÃ©duit les rÃ©partitions des dÃ©bits par type de vÃ©hicules
        double dSumDuree= 0;
        // construit la liste des durÃ©es des demandes des diffÃ©rents type de vÃ©hicule
        std::map<TypeVehicule *, ListOfTimeVariation<tracked_double>*> demandesInit = pOrigine->GetLstDemandeInit();
        std::map<TypeVehicule *, ListOfTimeVariation<tracked_double>*>::iterator itDemande;
        std::list<double> durees;
        std::list<double>::const_iterator itDuree;
        bool bIsPlage = false;
        for( itDemande = demandesInit.begin(); itDemande != demandesInit.end(); itDemande++)
        {
            size_t nDuree = itDemande->second->GetLstTV()->size();
            dSumDuree = 0;
            for( size_t iDuree = 0; iDuree< nDuree; ++iDuree )
            {
                if( itDemande->second->GetLstTV()->at(iDuree).m_pPlage == NULL)
                {
                    dSumDuree += itDemande->second->GetLstTV()->at(iDuree).m_dbPeriod;
                    durees.push_back(dSumDuree);

                }
                else
                {
                    durees.push_back(itDemande->second->GetLstTV()->at(iDuree).m_pPlage->m_Debut);
                    durees.push_back(itDemande->second->GetLstTV()->at(iDuree).m_pPlage->m_Fin);
                    bIsPlage = true;
                }
            }
        }
        // construit la liste des durÃ©es des rÃ©partitions par voies des diffÃ©rents types de vÃ©hicule
        std::map<TypeVehicule*, std::deque<TimeVariation<RepartitionEntree> >* > repVoiesInit = pOrigine->GetLstRepVoieInit();
        std::map<TypeVehicule*, std::deque<TimeVariation<RepartitionEntree> >* >::iterator itRepVoie;
        for( itRepVoie = repVoiesInit.begin(); itRepVoie != repVoiesInit.end(); itRepVoie++)
        {
            size_t nDuree = itRepVoie->second->size();
            dSumDuree = 0;
            for( size_t iDuree = 0; iDuree< nDuree; ++iDuree )
            {
                if( itRepVoie->second->at(iDuree).m_pPlage == NULL)
                {
                    dSumDuree += itRepVoie->second->at(iDuree).m_dbPeriod;
                    durees.push_back(dSumDuree);
                }
                else
                {
                    durees.push_back(itRepVoie->second->at(iDuree).m_pPlage->m_Debut);
                    durees.push_back(itRepVoie->second->at(iDuree).m_pPlage->m_Fin);
                    bIsPlage = true;
                }
            }
        }

        durees.sort();
        durees.unique(); // need a sorted list

        // On vire les instants nÃ©gatifs le cas Ã©chÃ©ant en les remplacant par 0 (sinon GetVariationEx
        // sur un durÃ©e nÃ©gative renvoie null)
        bool bAddStartSimulationTime = false;
        while (!durees.empty() && durees.front() < 0)
        {
            durees.pop_front();
            bAddStartSimulationTime = true;
        }
        if (bAddStartSimulationTime)
        {
            durees.push_front(0.0);
        }

        int nNbVoie;
        if(pZone)
        {
            // cas de la zone : une seule liste de coeffs
            nNbVoie = 1;
        }
        else
        {
            pT = pOrigine->GetOutputConnexion()->m_LstTuyAv.front();
            nNbVoie = pT->getNbVoiesDis();
        }

        // pour chaque duree on lui associe des repartition par type d'aprÃ¨s les demandes
        dSumDuree = 0;
        std::list<double>::const_iterator itEnd = durees.end();
        if( durees.size()>1 && bIsPlage )
        {
            itEnd--;
        }
        for( itDuree =durees.begin(); itDuree != itEnd; itDuree++)
        {

            std::vector<std::vector<double> > coeffs(nTypes);
            std::vector<double> sumPerType;
            for( int i = 0; i< nNbVoie; ++i)
            {
                sumPerType.push_back(0);
            }
            for( size_t iTypeVeh = 0; iTypeVeh < m_LstTypesVehicule.size() ;  ++iTypeVeh )
            {
                coeffs[iTypeVeh].resize(nNbVoie);
                if( demandesInit.find(m_LstTypesVehicule[iTypeVeh] ) != demandesInit.end() &&
                    demandesInit.find(m_LstTypesVehicule[iTypeVeh] )->second->GetVariationEx(*itDuree) )
                {
                    double dCoeff = *demandesInit.find(m_LstTypesVehicule[iTypeVeh] )->second->GetVariationEx(*itDuree);

                    // module par repartition par voie
                    RepartitionEntree* pREType = NULL;
                    if( repVoiesInit.find(m_LstTypesVehicule[iTypeVeh]) != repVoiesInit.end())
                    {
                        pREType = GetVariation(*itDuree, repVoiesInit.find(m_LstTypesVehicule[iTypeVeh])->second, GetLag()); // le debit d'entrÃ©e est considÃ©rÃ© "globalement"
                    }
                    double dVoieCoeff = 1.0 / nNbVoie;

                    for( int iCoeffWay = 0; iCoeffWay < nNbVoie; ++iCoeffWay)
                    {
                         if( pREType )
                        {
                            std::vector<double> pCoeffType = pREType->pCoefficients;
                            dVoieCoeff = pCoeffType[iCoeffWay ];
                        }

                         coeffs[iTypeVeh][iCoeffWay]=dCoeff *dVoieCoeff;
                         sumPerType[iCoeffWay] += dCoeff *dVoieCoeff;
                    }
                }
                else
                {
                    for( int iCoeffWay = 0; iCoeffWay < nNbVoie; ++iCoeffWay)
                    {
                         coeffs[iTypeVeh][iCoeffWay]=0;
                    }
                }
            }
            // on norme les coefficients par voie
            for( size_t iTypeVeh = 0; iTypeVeh < m_LstTypesVehicule.size() ;  ++iTypeVeh )
            {
                for( int iCoeffWay = 0; iCoeffWay < nNbVoie; ++iCoeffWay)
                {
                   coeffs[iTypeVeh][iCoeffWay]= (sumPerType[iCoeffWay]!=0)? coeffs[iTypeVeh][iCoeffWay] /sumPerType[iCoeffWay] : 0 ;
                }
            }
            // on ajoute la repartion ainsi crÃ©Ã©e
            if( bIsPlage == false)
            {
                pOrigine->GetLstRepTypeVeh()->AddVariation(coeffs, *itDuree - dSumDuree); // gestion "globale par type"
            }
            else
            {
                std::list<double>::const_iterator itDureeEnd  = itDuree;
                itDureeEnd++;


                PlageTemporelle *pPlage =  PlageTemporelle::Set( *itDuree + m_dtDebutSimu.ToSecond(),*itDureeEnd + m_dtDebutSimu.ToSecond(), m_LstPlagesTemporelles,m_dtDebutSimu.ToSecond() );
                pOrigine->GetLstRepTypeVeh()->AddVariation(coeffs, pPlage); // gestion "globale par type"
            }

            dSumDuree = *itDuree;
        }


     }
    return true;
}



bool Reseau::PostTimeStepSimulation()
{
    // Suppression du dernier instant en mÃ©moire calculÃ©
    std::deque<TraceDocTrafic*> docTrafics = GetXmlDocTrafic();
    if( docTrafics.size()>0 )
    {
        std::deque<TraceDocTrafic*>::iterator it;
        for( it = docTrafics.begin(); it != docTrafics.end() ; it++)
        {
            (*it)->RemoveLastInstant();
        }
    }

    return true;
}

std::deque<TraceDocTrafic*> Reseau::GetXmlDocTrafic() const
{
    std::deque<TraceDocTrafic*> docTrafic;
    if( IsCptItineraire() )
    {
        if (m_pModuleAffectation && m_pModuleAffectation->GetCurrentDocTraficTmp())
        {
            docTrafic.push_back(m_pModuleAffectation->GetCurrentDocTraficTmp());
            return docTrafic;
        }
    }

    std::deque<TimeVariation<TraceDocTrafic> >::const_iterator it;
    for( it = m_xmlDocTrafics.begin(); it != m_xmlDocTrafics.end(); it++)
    {
        docTrafic.push_back((*it).m_pData.get());
    }
    return docTrafic;
}

Voie* Reseau::GetVoieFromID(const std::string & sTuyauID, int NumVoie)
{
    Tuyau * pTuyau = GetLinkFromLabel(sTuyauID);
    if (pTuyau == NULL)
        return NULL;
    return pTuyau->GetVoie(NumVoie);
}

//=================================================================
    void    Reseau::SortieIndicateursInterDistance
//----------------------------------------------------------------
// Fonction  :
// Remarque  :
// Version du:
// Historique:
//
//=================================================================
(
)
{
    std::vector<boost::shared_ptr<Vehicule>>::iterator itV;
    int nCptVeh = 0;
    int nCptVehID6 = 0;
    int nCptVehID3 = 0;

    for(itV = m_LstVehicles.begin(); itV != m_LstVehicles.end(); ++itV)
    {
        boost::shared_ptr<Vehicule> pV = (*itV);
        boost::shared_ptr<Vehicule> pVLeader = pV->GetLeader();
        VoieMicro	 *pVoie0 = pV->GetVoie(0);
        double dbPos0 = pV->GetPos(0);

        // Si existence d'un ghost pour ce vÃ©hicule, c'est lui qui doit Ãªtre considÃ©rÃ©
        if( IsChgtVoieGhost() )
        {
            if( pV->GetGhostRemain() > 0 && pV->GetGhostVoie() )
            {
                pVoie0 = (VoieMicro*)pV->GetGhostVoie();	// Voie du ghost
                dbPos0 = pV->GetPos(0) * pVoie0->GetLength() / pV->GetVoie(0)->GetLength();	// Position du ghost

                pVLeader = this->GetNearAvalVehicule(pVoie0, dbPos0);
            }

            // Si le leader est ghostÃ©, il n'est pas pris en compte
            if( pVLeader && (pVLeader->GetGhostRemain() > 0 && pVLeader->GetGhostVoie()) )
            {
                pVLeader = pVLeader->GetLeader();
            }
        }

        nCptVeh++;
        if( pVLeader )
        {
            if( pVoie0 == pVLeader->GetVoie(0) )
            {
                if( pVLeader->GetPos(0) - dbPos0 <= 3 )
                    nCptVehID3++;
                if( pVLeader->GetPos(0) - dbPos0 <= 6 )
                    nCptVehID6++;
            }
        }
    }

    log() << " " << nCptVeh << " " << nCptVehID6 << " " << nCptVehID3 << std::endl;
}


//=================================================================
    std::string    Reseau::GetTronconAmontPrio
//----------------------------------------------------------------
// Fonction  : renvoie l'ID du troncon prioritaire du convergent
//             d'insertion
// Remarque  :
// Version du: 30/06/2011
// Historique: 30/06/2011 (O. TONCK - IPSIS)
//             CrÃ©ation
//=================================================================
(
    DOMNode * cvtInsertionNode, // noeud du convergent d'insertion
    Logger & logger
)
{
    DOMXPathResult* pTronconsAmontList = m_pXMLUtil->SelectNodes("TRONCONS_AMONT/TRONCON_AMONT", cvtInsertionNode->getOwnerDocument(), (DOMElement*)cvtInsertionNode);
    XMLSize_t countTronconsAmont = pTronconsAmontList->getSnapshotLength();
    int prioMin = INT_MAX;
    std::string tronconPrioID;
    for(XMLSize_t tronconAmontIdx=0; tronconAmontIdx<countTronconsAmont; tronconAmontIdx++)
    {
        pTronconsAmontList->snapshotItem(tronconAmontIdx);
        DOMNode *pTronconAmontNode = pTronconsAmontList->getNodeValue();
        std::string tronconID;
        GetXmlAttributeValue(pTronconAmontNode, "id", tronconID, &logger);
        int nPrioriteTroncon;
        GetXmlAttributeValue(pTronconAmontNode, "priorite", nPrioriteTroncon, &logger);

        if(nPrioriteTroncon <= prioMin)
        {
            tronconPrioID = tronconID;
            prioMin = nPrioriteTroncon;
        }
    }
    pTronconsAmontList->release();

    return tronconPrioID;
}

//=================================================================
    Tuyau*    Reseau::GetTronconAmontNonPrio
//----------------------------------------------------------------
// Fonction  : renvoie le tronÃ§on non prioritaire associÃ©e Ã  une
//             voie d'insertion Ã  partir du rÃ©partiteur amont du
//             troncon d'insertion
// Remarque  :
// Version du: 30/06/2011
// Historique: 30/06/2011 (O. TONCK - IPSIS)
//             CrÃ©ation
//=================================================================
(
    int nVoieInsertion,         // numÃ©ro de la voie d'insertion dans le troncon d'insertion
    Repartiteur * pRepartiteur  // rÃ©partiteur amont du troncon d'insertion
)
{
    // Le tuyau amont non prioritaire correspondant Ã  ce mergingobject est celui correspondant Ã  la voie insVoieIdx de T.
    // on le dÃ©termine grace Ã  la dÃ©finition des mouvements autorisÃ©s
    Tuyau * pTuyanAmontNonPrio = NULL;
    std::map<Voie*, std::map<Tuyau*, std::map<int, boost::shared_ptr<MouvementsSortie> >, LessPtr<Tuyau> >, LessPtr<Voie> >::iterator iterMvtAuto;
    std::map<Voie*, std::map<Tuyau*, std::map<int, boost::shared_ptr<MouvementsSortie> >, LessPtr<Tuyau> >, LessPtr<Voie> >::iterator endMvtAuto = pRepartiteur->m_mapMvtAutorises.end();
    for(iterMvtAuto = pRepartiteur->m_mapMvtAutorises.begin(); iterMvtAuto!=endMvtAuto; iterMvtAuto++)
    {
        // on a forcÃ©ment un unique tuyau aval
        assert(iterMvtAuto->second.size() == 1);

        // si une des voies aval de ces mouvements autorisÃ©s est la voie insVoieIdx, on a trouvÃ© la voie amont et donc le tuyau.
        const std::map<int, boost::shared_ptr<MouvementsSortie> > & mvtsSortie = (*iterMvtAuto->second.begin()).second;
        std::map<int, boost::shared_ptr<MouvementsSortie> >::const_iterator iterMvtSortie;
        std::map<int, boost::shared_ptr<MouvementsSortie> >::const_iterator endMvtSortie = mvtsSortie.end();
        for(iterMvtSortie = mvtsSortie.begin(); iterMvtSortie!=endMvtSortie; iterMvtSortie++)
        {
            if(iterMvtSortie->first == nVoieInsertion)
            {
                pTuyanAmontNonPrio = (Tuyau*)iterMvtAuto->first->GetParent();
            }
        }
    }

    return pTuyanAmontNonPrio;
}


//=================================================================
    bool    Reseau::BuildRepartitionFlux
//----------------------------------------------------------------
// Fonction  : Construit la rÃ©partition des flux associÃ©e Ã  un
//             rÃ©partiteur (possibilitÃ© de l'Ã©tendre aux autres
//             connexions)
// Remarque  :
// Version du: 18/07/2011
// Historique: 18/07/2011 (O. TONCK - IPSIS)
//             CrÃ©ation
//=================================================================
(
    Connexion * pConnexion,
    DOMDocument * pDocument,
    Logger *pChargement
)
{
    int nNbAmont = pConnexion->GetNbElAmont();
    int nNbAval = pConnexion->GetNbElAval();

    double** pLstCoeff;
    int** pNbVeh;

    // Chargement
    DOMXPathResult *pXMLRepartitionsLst;
    DOMNode *pXMLRepartitions;
    DOMNode *pXMLRepartition;
    DOMNode *pXMLTronconAmonts;
    DOMNode *pXMLTronconAmont;
    DOMNode *pXMLVoieAmonts;
    DOMNode *pXMLVoieAmont;
    DOMNode *pXMLTronconAvals;
    DOMNode *pXMLTronconAval;
    int nAmont, nAval;
    int nNumVoie;
    int nNbmaxVoies = pConnexion->GetNbMaxVoies();
    double dbDuree;
    std::string strTmp;

    bool bIsGir = dynamic_cast<Giratoire*>(pConnexion) != NULL;

    std::map<TypeVehicule*, std::deque<TimeVariation<RepartitionFlux> >* > mapRepartitions;

    // RÃ©cupÃ©ration des paramÃ¨tres trafic du rÃ©partiteur
    pXMLRepartitionsLst = m_pXMLUtil->SelectNodes("//TRAFICS/TRAFIC/CONNEXIONS_INTERNES/CONNEXION_INTERNE[@id=\"" + pConnexion->GetID() + "\"]/COEFFS_DIR", pDocument);
    XMLSize_t nbTypeVeh = pXMLRepartitionsLst->getSnapshotLength();
    for (XMLSize_t i = 0; i < nbTypeVeh; i++)
    {
        pXMLRepartitionsLst->snapshotItem(i);
        pXMLRepartitions = pXMLRepartitionsLst->getNodeValue();
        
        TypeVehicule * pTypeVeh = NULL;
        if (GetXmlAttributeValue(pXMLRepartitions, "type_veh", strTmp, pChargement))
        {
            pTypeVeh = GetVehicleTypeFromID(strTmp);
            if (!pTypeVeh)
            {
                *pChargement << Logger::Error << "ERROR : Unknown vehicle type '" << strTmp << "'" << std::endl;
                *pChargement << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                return false;
            }
        }

        std::deque<TimeVariation<RepartitionFlux> >* & repForType = mapRepartitions[pTypeVeh];
        repForType = new std::deque<TimeVariation<RepartitionFlux> >();

        XMLSize_t countj = pXMLRepartitions->getChildNodes()->getLength();
        int jj = 0;
        for(XMLSize_t j=0; j<countj;j++)
        {
            pXMLRepartition = pXMLRepartitions->getChildNodes()->item(j);
            if (pXMLRepartition->getNodeType() != DOMNode::ELEMENT_NODE) continue;

            pLstCoeff = new double*[nNbAmont*nNbmaxVoies];
            pNbVeh = new int*[nNbAmont*nNbmaxVoies];

            for(int l=0; l< nNbAmont; l++)
            {
                for(int nVoieAmont=0; nVoieAmont<  nNbmaxVoies; nVoieAmont++)
                {
                    pLstCoeff[l*nNbmaxVoies+nVoieAmont] = new double[nNbAval*nNbmaxVoies];
                    pNbVeh[l*nNbmaxVoies+nVoieAmont] = new int[nNbAval*nNbmaxVoies];

                    // Initialisation
                    for(int k=0; k< nNbAval; k++)
                    for (int nVoieAval = 0; nVoieAval < nNbmaxVoies; nVoieAval++) {
                        pLstCoeff[l*nNbmaxVoies + nVoieAmont][k*nNbmaxVoies + nVoieAval] = 0;
                        pNbVeh[l*nNbmaxVoies + nVoieAmont][k*nNbmaxVoies + nVoieAval] = 0;
                    }
                }
            }

            // DurÃ©e
            vector<PlageTemporelle*> plages;
            if(!GetXmlDuree(pXMLRepartition,this,dbDuree,plages, pChargement))
            {
                return false;
            }


            // Boucle sur les tronÃ§ons amont
            pXMLTronconAmonts = m_pXMLUtil->SelectSingleNode("./COEFFS_TRONCON_AMONT", pXMLRepartition->getOwnerDocument(), (DOMElement*)pXMLRepartition);
            XMLSize_t countk = pXMLTronconAmonts->getChildNodes()->getLength();
            for(XMLSize_t k=0; k<countk;k++)
            {
                pXMLTronconAmont = pXMLTronconAmonts->getChildNodes()->item(k);
                if (pXMLTronconAmont->getNodeType() != DOMNode::ELEMENT_NODE) continue;

                // id du tronÃ§on amont
                GetXmlAttributeValue(pXMLTronconAmont, "id_amont", strTmp, pChargement);
                Tuyau* pTAmont = GetLinkFromLabel(strTmp);
                nAmont = pConnexion->GetNoEltAmont(pTAmont);

                if(nAmont<0)
                {
                    *pChargement << Logger::Error << "ERROR : Wrong upstream link ( " << pTAmont->GetLabel() << " ) for the repartitor's flow repartition definition : " << pConnexion->GetID();
                    *pChargement << Logger::Error << " ( repartition  : "<< jj+1 <<" )" <<std::endl;
                    *pChargement << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                    return false;
                }

                // Boucle sur les voies amont
                pXMLVoieAmonts = m_pXMLUtil->SelectSingleNode("./COEFFS_VOIE_AMONT", pXMLTronconAmont->getOwnerDocument(), (DOMElement*)pXMLTronconAmont);
                XMLSize_t countl =pXMLVoieAmonts->getChildNodes()->getLength();
                bool bLaneByLaneModeDetermined = false;
                bool bLaneByLaneModeValue = true;
				std::map<size_t, std::map<int, double> > coeffsToReportOnOtherUpLane;
				std::map<size_t, std::map<int, int> > mapNbAccessibleDownLanes;
                for(XMLSize_t l=0; l<countl;l++)
                {
                    pXMLVoieAmont = pXMLVoieAmonts->getChildNodes()->item(l);
                    if (pXMLVoieAmont->getNodeType() != DOMNode::ELEMENT_NODE) continue;

                    // NumÃ©ro de voie
                    bool bIsForAllLanes = !GetXmlAttributeValue(pXMLVoieAmont, "numvoie", nNumVoie, pChargement);
                    if (!bLaneByLaneModeDetermined)
                    {
                        bLaneByLaneModeValue = bIsForAllLanes;
                        bLaneByLaneModeDetermined = true;
                    }
                    else if (bIsForAllLanes)
                    {
                        *pChargement << Logger::Error << "ERROR : all COEFF_VOIE_AMONT siblings must have the 'numvoie' attribute, or there must be just one COEFF_VOIE_AMONT node without the 'numvoie' attribute for connection " <<pConnexion->GetID() << std::endl;
                        *pChargement << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                        return false;
                    }

                    // VÃ©rification
                    if (!bIsForAllLanes && nNumVoie > pTAmont->getNbVoiesDis())
                    {
                        *pChargement << Logger::Error << "ERROR : Wrong lane number ('numvoie' of COEFF_VOIE_AMONT - repartitor : " << pConnexion->GetID();
                        *pChargement << Logger::Error << " - repartition : "<< jj+1 << " - upstream link " << pTAmont->GetLabel();
                        *pChargement << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                        return false;
                    }

                    std::vector<Voie*> lstLanes;
                    if (bIsForAllLanes)
                    {
                        lstLanes = pTAmont->GetLstLanes();
                    }
                    else
                    {
                        lstLanes.push_back(pTAmont->GetVoie(nNumVoie - 1));
                    }

                    for (size_t iUpLane = 0; iUpLane < lstLanes.size(); iUpLane++)
                    {
                        Voie * pVAm = lstLanes.at(iUpLane);

                        // Boucle sur les tronÃ§ons en aval
                        pXMLTronconAvals = m_pXMLUtil->SelectSingleNode("./COEFFS_TRONCON_AVAL", pXMLVoieAmont->getOwnerDocument(), (DOMElement*)pXMLVoieAmont);
                        XMLSize_t countm = pXMLTronconAvals->getChildNodes()->getLength();
                        for (XMLSize_t m = 0; m < countm; m++)
                        {
                            pXMLTronconAval = pXMLTronconAvals->getChildNodes()->item(m);
                            if (pXMLTronconAval->getNodeType() != DOMNode::ELEMENT_NODE) continue;

                            // id du tronÃ§on amont
                            GetXmlAttributeValue(pXMLTronconAval, "id_aval", strTmp, pChargement);
                            Tuyau* pTAval = GetLinkFromLabel(strTmp);
                            nAval = pConnexion->GetNoEltAval(pTAval);

                            if (nAval < 0)
                            {
                                *pChargement << Logger::Error << "ERROR : Invalid downtream link ( " << pTAval->GetLabel() << " ) when defining the flow repartition of the repartitor : " << pConnexion->GetID();
                                *pChargement << Logger::Error << " - repartition  : " << jj + 1 << " - upstream link " << pTAmont->GetLabel() << std::endl;
                                *pChargement << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                                return false;
                            }

                            // Liste des coefficients pour ce tronÃ§on aval
                            GetXmlAttributeValue(pXMLTronconAval, "coeffs", strTmp, pChargement);

                            // VÃ©rification du nombre de coeff
                            std::deque<std::string> split = SystemUtil::split(strTmp, ' ');
                            if (split.size() != pTAval->getNbVoiesDis() && split.size() != 1)
                            {
                                *pChargement << Logger::Error << "ERROR : the number of values of the attribute 'coeffs' of the COEFFS_TRONCON_AVAL node (repartitor : " << pConnexion->GetID();
                                *pChargement << Logger::Error << " - repartition : " << jj + 1 << " - upstream link : " << pTAmont->GetLabel() << " - upstream lane number : " << (iUpLane+1) << " - downstream link : " << pTAval->GetLabel();
                                *pChargement << Logger::Error << " ) is incorrect (must be unique for all lanes of downstream link or equal to the number of lanes of the link " << pTAval->GetLabel() << ").";
                                *pChargement << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                                return false;
                            }

                            bool bByLinkCoeffs = split.size() == 1;
                            if (bIsForAllLanes && bByLinkCoeffs && atof(split.at(0).c_str()) > 0)
                            {
                                if (!(bIsGir || pConnexion->IsMouvementAutorise(pTAmont, pTAval, pTypeVeh, NULL)))
                                {
                                    *pChargement << Logger::Error << "ERROR : the 'coeffs' attribute of the COEFFS_TRONCON_AVAL node contains a non-null value for the unauthorized movement from link " << pTAmont->GetLabel() << " to link " << pTAval->GetLabel() << std::endl;
                                    *pChargement << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                                    return false;
                                }
                            }
                            else if (bByLinkCoeffs && atof(split.at(0).c_str()) > 0)
                            {
                                if (!(bIsGir || pConnexion->IsMouvementAutorise(pVAm, pTAval, pTypeVeh, NULL)))
                                {
                                    *pChargement << Logger::Error << "ERROR : the 'coeffs' attribute of the COEFFS_TRONCON_AVAL node contains a non-null value for the unauthorized movement from lane " << (iUpLane + 1) << " of link " << pTAmont->GetLabel() << " to link " << pTAval->GetLabel() << std::endl;
                                    *pChargement << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                                    return false;
                                }
                            }


                            for (size_t n = 0; n<pTAval->GetLstLanes().size(); n++)
                            {
                                double dbTmp = bByLinkCoeffs ? atof(split.at(0).c_str()) : atof(split.at(n).c_str());

                                // VÃ©rification du mouvement autorisÃ©
                                Voie * pVAv = pTAval->GetVoie((int)n);
                                if (!bByLinkCoeffs && dbTmp > 0)
                                {
                                    if (bIsForAllLanes)
                                    {
                                        if (!(bIsGir || pConnexion->IsMouvementAutorise(pTAmont, pVAv, pTypeVeh, NULL)))
                                        {
                                            *pChargement << Logger::Error << "ERROR : the 'coeffs' attribute of the COEFFS_TRONCON_AVAL node contains a non-null value for the unauthorized movement from link " << pTAmont->GetLabel() << " to lane " << (n + 1) << " of link " << pTAval->GetLabel() << std::endl;
                                            *pChargement << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                                            return false;
                                        }
                                    }
                                    else
                                    {
                                        if (!(bIsGir || pConnexion->IsMouvementAutorise(pVAm, pVAv, pTypeVeh, NULL)))
                                        {
                                            *pChargement << Logger::Error << "ERROR : the 'coeffs' attribute of the COEFFS_TRONCON_AVAL node contains a non-null value for the unauthorized movement from lane " << (iUpLane + 1) << " of link " << pTAmont->GetLabel() << " to lane " << (n + 1) << " of link " << pTAval->GetLabel() << std::endl;
                                            *pChargement << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                                            return false;
                                        }
                                    }
                                }

                                if (bIsGir || pConnexion->IsMouvementAutorise(pVAm, pVAv, pTypeVeh, NULL))
                                {
									mapNbAccessibleDownLanes[iUpLane][nAval]++;
                                    pLstCoeff[nAmont*nNbmaxVoies + iUpLane][nAval*nNbmaxVoies + n] = dbTmp;
                                }
								else if(bIsForAllLanes && bByLinkCoeffs && !pConnexion->IsMouvementAutorise(pVAm, pTAval, pTypeVeh, NULL))
								{
									coeffsToReportOnOtherUpLane[iUpLane][nAval] = dbTmp;
								}
                            }
							if (bByLinkCoeffs && mapNbAccessibleDownLanes[iUpLane][nAval] > 0)
							{
								for (size_t n = 0; n < pTAval->GetLstLanes().size(); n++)
								{
									pLstCoeff[nAmont*nNbmaxVoies + iUpLane][nAval*nNbmaxVoies + n] /= mapNbAccessibleDownLanes[iUpLane][nAval];
								}
							}
                        }
                    }
                }

				if (!coeffsToReportOnOtherUpLane.empty())
				{
					for (std::map<size_t, std::map<int, double> >::const_iterator iterUpLane = coeffsToReportOnOtherUpLane.begin(); iterUpLane != coeffsToReportOnOtherUpLane.end(); ++iterUpLane)
					{
						for (std::map<int, double>::const_iterator iterDownLink = iterUpLane->second.begin(); iterDownLink != iterUpLane->second.end(); ++iterDownLink)
						{
							Tuyau * pTAval = pConnexion->m_LstTuyAv[iterDownLink->first];
							// Recherche des voies amont qui permettent d'aller sur le tronÃ§on aval : on y distribue le relicat de coefficient
							std::set<size_t> availableUpLanes;
							for (size_t iUpLane = 0; iUpLane < pTAmont->GetLstLanes().size(); iUpLane++)
							{
								if (iUpLane != iterUpLane->first)
								{
									if (pConnexion->IsMouvementAutorise(pTAmont->GetVoie((int)iUpLane), pTAval, pTypeVeh, NULL))
									{
										availableUpLanes.insert(iUpLane);
									}
								}
							}
							for (std::set<size_t>::const_iterator iterAvailableLanes = availableUpLanes.begin(); iterAvailableLanes != availableUpLanes.end(); ++iterAvailableLanes)
							{
								for (int nVoieAval = 0; nVoieAval < pTAval->getNbVoiesDis(); nVoieAval++)
								{
									pLstCoeff[nAmont*nNbmaxVoies + *iterAvailableLanes][iterDownLink->first*nNbmaxVoies + nVoieAval] += (iterDownLink->second / availableUpLanes.size()) / mapNbAccessibleDownLanes[*iterAvailableLanes][iterDownLink->first];
								}
							}
						}
					}

					// renormalisation Ã  1 de la somme des coefficients pour une voie amont
					for (size_t iUpLane = 0; iUpLane < pTAmont->GetLstLanes().size(); iUpLane++)
					{
						double dbSum = 0;
						for (int nLinkAval = 0; nLinkAval < nNbAval; nLinkAval++)
							for (int nVoieAval = 0; nVoieAval < nNbmaxVoies; nVoieAval++)
							{
								dbSum += pLstCoeff[nAmont*nNbmaxVoies + iUpLane][nLinkAval*nNbmaxVoies + nVoieAval];
							}
						if (dbSum > 0)
						{
							for (int nLinkAval = 0; nLinkAval < nNbAval; nLinkAval++)
								for (int nVoieAval = 0; nVoieAval < nNbmaxVoies; nVoieAval++)
								{
									pLstCoeff[nAmont*nNbmaxVoies + iUpLane][nLinkAval*nNbmaxVoies + nVoieAval] /= dbSum;
								}
						}
					}
				}
            }

            boost::shared_ptr<RepartitionFlux> pRF(new RepartitionFlux);
            pRF->nNbVeh = pNbVeh;
            pRF->pCoefficients = pLstCoeff;
            pRF->nNbVoiesAmont = nNbAmont*nNbmaxVoies;
            pRF->nNbVoiesAval = nNbAval*nNbmaxVoies;
            if(plages.size() > 0)
            {
                for(size_t iPlage = 0; iPlage < plages.size(); iPlage++)
                {
                    AddVariation(plages[iPlage], pRF, repForType);
                }
            }
            else
            {
                AddVariation(dbDuree, pRF, repForType);
            }
            jj++;
        }

        // vÃ©rification de la couverture temporelle
        vector<PlageTemporelle*> plages;
        for (size_t iPlage = 0; iPlage < repForType->size(); iPlage++)
        {
            PlageTemporelle * pPlage = repForType->at(iPlage).m_pPlage;
            if(pPlage)
            {
                plages.push_back(pPlage);
            }
        }
        if(plages.size() > 0 && !CheckPlagesTemporellesEx(m_dbDureeSimu, plages))
        {
            *pChargement << Logger::Error << "ERROR : The time frames defined for the directional coefficients of the node " << pConnexion->GetID() << " don't cover the whole simulation duration!" << std::endl;
            *pChargement << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
            return false;
        }
    }


    std::map<TypeVehicule*, std::deque<TimeVariation<RepartitionFlux> >* >::const_iterator iterNoTypeVeh = mapRepartitions.find(NULL);
    bool bHasDefaultRep = iterNoTypeVeh != mapRepartitions.end();
    for (size_t iTypeVeh = 0; iTypeVeh < m_LstTypesVehicule.size(); iTypeVeh++)
    {
        TypeVehicule * pTypeVeh = m_LstTypesVehicule.at(iTypeVeh);

        std::map<TypeVehicule*, std::deque<TimeVariation<RepartitionFlux> >* >::const_iterator iterTypeVeh = mapRepartitions.find(pTypeVeh);

        if (iterTypeVeh != mapRepartitions.end())
        {
            // Cas simple : une rÃ©partition a Ã©tÃ© dÃ©finie explicitement pour le type de vÃ©hicule : on l'utilise directement :
            std::deque<TimeVariation<RepartitionFlux> >* & repTypeVeh = pConnexion->GetLstRepartition()[pTypeVeh];
            repTypeVeh = iterTypeVeh->second;
        }
        else if (bHasDefaultRep)
        {
            // Une rÃ©partition a Ã©tÃ© dÃ©finie sans type de vÃ©hicule particulier :
            // On utilise celle-ci, en l'adaptant si nÃ©cessaire pour les mouvements autorisÃ©s possibles pour le type de vÃ©hicule :
            std::deque<TimeVariation<RepartitionFlux> >* & repTypeVeh = pConnexion->GetLstRepartition()[pTypeVeh];
            repTypeVeh = new std::deque<TimeVariation<RepartitionFlux> >();

            std::deque<TimeVariation<RepartitionFlux> >* pRepToAdapt = iterNoTypeVeh->second;
            for (size_t iRep = 0; iRep < pRepToAdapt->size(); iRep++)
            {
                TimeVariation<RepartitionFlux> adaptedTimeVariation = pRepToAdapt->at(iRep);

                boost::shared_ptr<RepartitionFlux> pRF(new RepartitionFlux);

                pRF->pCoefficients = new double*[nNbAmont*nNbmaxVoies];
                pRF->nNbVeh = new int*[nNbAmont*nNbmaxVoies];

                for (int j = 0; j < nNbAmont; j++)
                {
                    for (int nVoieAmont = 0; nVoieAmont < nNbmaxVoies; nVoieAmont++)
                    {
                        pRF->pCoefficients[j*nNbmaxVoies + nVoieAmont] = new double[nNbAval*nNbmaxVoies];
                        pRF->nNbVeh[j*nNbmaxVoies + nVoieAmont] = new int[nNbAval*nNbmaxVoies];

                        // Initialisation
                        for (int k = 0; k < nNbAval; k++)
                        for (int nVoieAval = 0; nVoieAval < nNbmaxVoies; nVoieAval++)
                        {
                            pRF->nNbVeh[j*nNbmaxVoies + nVoieAmont][k*nNbmaxVoies + nVoieAval] = 0;
                            pRF->pCoefficients[j*nNbmaxVoies + nVoieAmont][k*nNbmaxVoies + nVoieAval] = 0;
                        }

                    }
                }

                for (int j = 0; j < pConnexion->GetNbElAmont(); j++)
                {
                    Tuyau * pTAmont = pConnexion->m_LstTuyAm[j];

                    for (int nVAm = 0; nVAm < pTAmont->getNbVoiesDis(); nVAm++)
                    {
                        Voie * pVAm = pTAmont->GetVoie(nVAm);

                        double dbSum = 0;

                        for (int k = 0; k < pConnexion->GetNbElAval(); k++)
                        {
                            Tuyau * pTAval = pConnexion->m_LstTuyAv[k];

                            for (int nVAv = 0; nVAv < pTAval->getNbVoiesDis(); nVAv++)
                            {
                                Voie * pVAv = pTAval->GetVoie(nVAv);
                                if (bIsGir || pConnexion->IsMouvementAutorise(pVAm, pVAv, pTypeVeh, NULL))
                                {
                                    double dbTmp = adaptedTimeVariation.m_pData->pCoefficients[j*nNbmaxVoies + nVAm][k*nNbmaxVoies + nVAv];
                                    pRF->pCoefficients[j*nNbmaxVoies + nVAm][k*nNbmaxVoies + nVAv] = dbTmp;
                                    dbSum += dbTmp;
                                }
                            }
                        }

                        // Normalisation
                        if (dbSum > 0)
                        {
                            for (int k = 0; k < pConnexion->GetNbElAval(); k++)
                            {
                                Tuyau * pTAval = pConnexion->m_LstTuyAv[k];

                                for (int nVAv = 0; nVAv < pTAval->getNbVoiesDis(); nVAv++)
                                {
                                    pRF->pCoefficients[j*nNbmaxVoies + nVAm][k*nNbmaxVoies + nVAv] /= dbSum;
                                }
                            }
                        }
                    }
                }

                pRF->nNbVoiesAmont = adaptedTimeVariation.m_pData->nNbVoiesAmont;
                pRF->nNbVoiesAval = adaptedTimeVariation.m_pData->nNbVoiesAval;

                adaptedTimeVariation.m_pData = pRF;

                repTypeVeh->push_back(adaptedTimeVariation);
            }
        }
        else
        {
            // Utilisation d'une Ã©quirepartition sur les mouvements autorisÃ©s pour le type de vÃ©hicule.
            Tuyau *pTAmont;
            Tuyau *pTAval;
            int nNbVoiesAval;  // nombre de voie en aval

            pLstCoeff = new double*[nNbAmont*nNbmaxVoies];
            pNbVeh = new int*[nNbAmont*nNbmaxVoies];

            for (int j = 0; j < nNbAmont; j++)
            {
                for (int nVoieAmont = 0; nVoieAmont < nNbmaxVoies; nVoieAmont++)
                {
                    pLstCoeff[j*nNbmaxVoies + nVoieAmont] = new double[nNbAval*nNbmaxVoies];
                    pNbVeh[j*nNbmaxVoies + nVoieAmont] = new int[nNbAval*nNbmaxVoies];

                    // Initialisation
                    for (int k = 0; k < nNbAval; k++)
                    for (int nVoieAval = 0; nVoieAval < nNbmaxVoies; nVoieAval++)
                    {
                        pNbVeh[j*nNbmaxVoies + nVoieAmont][k*nNbmaxVoies + nVoieAval] = 0;
                        pLstCoeff[j*nNbmaxVoies + nVoieAmont][k*nNbmaxVoies + nVoieAval] = 0;
                    }

                }
            }

            for (int j = 0; j < pConnexion->GetNbElAmont(); j++)
            {
                pTAmont = pConnexion->m_LstTuyAm[j];

                for (int nVAm = 0; nVAm < pTAmont->getNbVoiesDis(); nVAm++)
                {
                    Voie * pVAm = pTAmont->GetVoie(nVAm);

                    nNbVoiesAval = 0;
                    for (int k = 0; k < pConnexion->GetNbElAval(); k++)
                    {
                        pTAval = pConnexion->m_LstTuyAv[k];

                        for (int nVAv = 0; nVAv < pTAval->getNbVoiesDis(); nVAv++)
                        {
                            Voie * pVAv = pTAval->GetVoie(nVAv);
                            if (bIsGir || pConnexion->IsMouvementAutorise(pVAm, pVAv, pTypeVeh, NULL))
                            {
                                pLstCoeff[j*nNbmaxVoies + nVAm][k*nNbmaxVoies + nVAv] = 1.0;
                                nNbVoiesAval++;
                            }
                        }
                    }

                    // Normalisation
                    if (nNbVoiesAval > 0)
                    {
                        for (int k = 0; k < pConnexion->GetNbElAval(); k++)
                        {
                            pTAval = pConnexion->m_LstTuyAv[k];

                            for (int nVAv = 0; nVAv < pTAval->getNbVoiesDis(); nVAv++)
                            {
                                pLstCoeff[j*nNbmaxVoies + nVAm][k*nNbmaxVoies + nVAv] /= nNbVoiesAval;
                            }
                        }
                    }
                }
            }
            boost::shared_ptr<RepartitionFlux> pRF(new RepartitionFlux);
            pRF->nNbVeh = pNbVeh;
            pRF->pCoefficients = pLstCoeff;
            pRF->nNbVoiesAmont = nNbAmont*nNbmaxVoies;
            pRF->nNbVoiesAval = nNbAval*nNbmaxVoies;
            std::deque<TimeVariation<RepartitionFlux> >* & repTypeVeh = pConnexion->GetLstRepartition()[pTypeVeh];
            repTypeVeh = new std::deque<TimeVariation<RepartitionFlux> >();
            AddVariation(m_dbDureeSimu, pRF, repTypeVeh);
        }
    }

    return true;
}


//=================================================================
    void    Reseau::BuildPlagesTemporelles
//----------------------------------------------------------------
// Fonction  : Construit la liste des plages temporelles dÃ©finies
//             dans le fichier scÃ©nario
// Remarque  :
// Version du: 05/09/2011
// Historique: 05/09/2011 (O. TONCK - IPSIS)
//             CrÃ©ation
//=================================================================
(
    DOMNode * pDOMNodePlagesTemporelles,
    Logger & loadingLogger
)
{
    std::vector<PlageTemporelle *>::iterator itRange;
    for( itRange = m_LstPlagesTemporelles.begin(); itRange!= m_LstPlagesTemporelles.end(); itRange++)
    {
        delete *itRange;
    }
    m_LstPlagesTemporelles.clear();

    DOMXPathResult * pPlages = m_pXMLUtil->SelectNodes("PLAGE_TEMPORELLE", pDOMNodePlagesTemporelles->getOwnerDocument(), (DOMElement*) pDOMNodePlagesTemporelles);

    XMLSize_t nbPlages = pPlages->getSnapshotLength();
    for(XMLSize_t i = 0; i < nbPlages; i++)
    {
        pPlages->snapshotItem(i);
        DOMNode * pPlage = pPlages->getNodeValue();

        std::string strTmp;
        GetXmlAttributeValue(pPlage, "id", strTmp, &loadingLogger);

        PlageTemporelle *plage = new PlageTemporelle();
        plage->m_ID = strTmp;

        SDateTime dateTime;
        GetXmlAttributeValue(pPlage, "debut", dateTime, &loadingLogger);
        STime time = dateTime.ToTime();
        int debutSimu = m_dtDebutSimu.GetSecond()+m_dtDebutSimu.GetMinute()*60+m_dtDebutSimu.GetHour()*3600;
        plage->m_Debut = time.GetSecond()+time.GetMinute()*60+time.GetHour()*3600-debutSimu;
        GetXmlAttributeValue(pPlage, "fin", dateTime, &loadingLogger);
        time = dateTime.ToTime();
        plage->m_Fin = time.GetSecond()+time.GetMinute()*60+time.GetHour()*3600-debutSimu;

        m_LstPlagesTemporelles.push_back(plage);
    }
    pPlages->release();
}

//=================================================================
    int	Reseau::CreateVehicle
//----------------------------------------------------------------
// Fonction  : CrÃ©ation d'un nouveau vÃ©hicule
// Remarque  :
// Version du:
// Historique:
//=================================================================
(
    const std::string & sType,		// Type du vÃ©hicule
    const std::string & sEntree,	// EntrÃ©e oÃ¹ est gÃ©nÃ©rÃ©e le vÃ©hicule
    const std::string & sSortie,	// Sortie vers laquelle se dirige le vÃ©hicule
    int nVoie,				// Voie de gÃ©nÃ©ration
    double dbt, 			// Instant exact de crÃ©ation du vÃ©hicule
    std::deque<string> * pIti, // ItinÃ©raire Ã©ventuel Ã  suivre par le vÃ©hicule
    const std::string & junctionName, // point de jonction si pIti vide
    int externalUserID,      // Identifiant externe Ã©ventuel
    std::string nextRouteId     // Identifiant de la route aval pour le vÃ©hicule (pour hybridation SymuMaster avec SimuRes
)
{
    TypeVehicule *pTV;
    SymuViaTripNode *pOrigine;
    SymuViaTripNode *pDestination;
    Connexion *pConOrigine, *pConDestination;
    Tuyau * pLink = NULL;
    CPlaque * pPlaqueOrigin = NULL, *pPlaqueDestination = NULL;
    char cTmp;

    // VÃ©rifications
    pTV = GetVehicleTypeFromID(sType);
    if(!pTV)
        return -2;

    boost::shared_ptr<std::vector<Tuyau*> > itinerary;

    // rÃ©cupÃ©ration des tronÃ§ons concernÃ©s si itinÃ©raire fourni
    Tuyau* pUpT = NULL;
    Tuyau* pDownT = NULL;
    if (pIti != NULL)
    {
        itinerary = boost::make_shared<std::vector<Tuyau*> >();
        for (size_t LinkIdx = 0; LinkIdx<pIti->size(); LinkIdx++)
        {
            pDownT = GetLinkFromLabel(pIti->at(LinkIdx));
            if (!pDownT)
                return -9;	// This link does not belong to the simulated network

            if (pUpT)
                if (!pDownT->GetCnxAssAm()->IsTuyauAmont(pUpT))
                    return -10;	// The route is wrong

            itinerary->push_back(pDownT);
            pUpT = pDownT;
        }
    }

    // RÃ©cupÃ©ration de l'origine
    pOrigine = GetOrigineFromID(sEntree, cTmp);
    if (!pOrigine)
    {
        pConOrigine = GetConnectionFromID(sEntree, cTmp);
        if (!pConOrigine)
        {
            pConOrigine = GetBrickFromID(sEntree);
        }
        if (pConOrigine && !itinerary->empty())
        {
            // Cas du tuyaux d'origine
            pLink = itinerary->front();

            if (pLink->GetCnxAssAm() != pConOrigine)
                return -11;

            std::map<Tuyau*, TronconOrigine*>::iterator iter = m_mapOriginLinksForCreatedVehicles.find(pLink);
            if (iter != m_mapOriginLinksForCreatedVehicles.end())
            {
                pOrigine = iter->second;
            }
            else
            {
                pOrigine = new TronconOrigine(pLink, NULL);
                m_mapOriginLinksForCreatedVehicles[pLink] = (TronconOrigine*)pOrigine;
            }
        }
        else
        {
            GetPlaqueFromID(sEntree, pOrigine, pPlaqueOrigin);
        }

        if (!pOrigine)
            return -3;
    }

    if (nVoie != -1
        && ((pLink && pLink->getNb_voies() <= nVoie)
        || (!pLink && pOrigine->GetOutputConnexion()->m_LstTuyAv.front()->getNb_voies() <= nVoie)))
    {
        return -5;
    }

    pLink = NULL;
    pDestination = GetDestinationFromID(sSortie, cTmp);
    if (!pDestination)
    {
        pConDestination = GetConnectionFromID(sSortie, cTmp);
        if (!pConDestination)
        {
            pConDestination = GetBrickFromID(sSortie);
        }
        if (!pConDestination)
        {
            // Pour SymuGame : gestion du cas oÃ¹ la destination est un tuyau
            Tuyau * pDestLink = GetLinkFromLabel(sSortie);
            if (pDestLink && !itinerary->empty() && itinerary->back() == pDestLink)
            {
                pConDestination = pDestLink->GetCnxAssAv();
            }
        }
        if (pConDestination && !itinerary->empty())
        {
            // Cas du tuyaux de destination
            pLink = itinerary->back();

            if (pLink->GetCnxAssAv() != pConDestination)
                return -12;

            std::map<Tuyau*, TronconDestination*>::iterator iter = m_mapDestinationLinksForCreatedVehicles.find(pLink);
            if (iter != m_mapDestinationLinksForCreatedVehicles.end())
            {
                pDestination = iter->second;
            }
            else
            {
                pDestination = new TronconDestination(pLink, pLink->GetLength());
                m_mapDestinationLinksForCreatedVehicles[pLink] = (TronconDestination*)pDestination;
            }
        }
        else
        {
            GetPlaqueFromID(sSortie, pDestination, pPlaqueDestination);
        }
        if (!pDestination)
        {
            return -6;
        }
    }

    Connexion * pJunction = NULL;
    if (!junctionName.empty())
    {
        pJunction = GetConnectionFromID(junctionName, cTmp);
        if (!pJunction)
        {
            pJunction = GetBrickFromID(junctionName);
            if (!pJunction)
            {
                return -13;
            }
        }
    }


    if( dbt <0 || dbt > this->GetTimeStep() )
        return -8;

    SymuViaVehicleToCreate * pVehicle = new SymuViaVehicleToCreate(IncLastIdVeh(), GetSymuViaFleet());

	//log() << Logger::Info << " Vehicle creation: " << pVehicle->GetVehicleID() << " type: " << pTV->GetLabel() << " origine: " << pOrigine->GetID() << endl;

    pVehicle->SetTimeFraction(dbt);
    pVehicle->SetNumVoie(nVoie);
    pVehicle->SetDestination(pDestination);
    pVehicle->SetOrigin(pOrigine);
    pVehicle->SetPlaqueOrigin(pPlaqueOrigin);
    pVehicle->SetPlaqueDestination(pPlaqueDestination);
    pVehicle->SetType(pTV);
    pVehicle->SetItinerary(itinerary);
    pVehicle->SetJunction(pJunction);
    pVehicle->SetExternalID(externalUserID);
    pVehicle->SetNextRouteID(nextRouteId);
    m_VehiclesToCreate.push_back(pVehicle);

    return pVehicle->GetVehicleID();
}

int Reseau::CreatePublicTransportUser(const std::string & startStop, const std::string & endStop, const std::string & lineName, double dbt, int externalUserID)
{
    // rï¿½cupï¿½ration de l'arrï¿½t de dï¿½part
    Arret * pFirstStop = dynamic_cast<Arret*>(GetPublicTransportFleet()->GetTripNode(startStop));
    if (!pFirstStop)
    {
        return -3;
    }

    Arret * pLastStop = dynamic_cast<Arret*>(GetPublicTransportFleet()->GetTripNode(endStop));
    if (!pLastStop)
    {
        return -4;
    }

    PublicTransportLine * pLine = dynamic_cast<PublicTransportLine*>(GetPublicTransportFleet()->GetTrip(lineName));
    if (!pLine)
    {
        return -4;
    }

    if (!pFirstStop->hasLine(pLine) || !pLastStop->hasLine(pLine))
    {
        return -5;
    }

    // idï¿½e de test supplï¿½mentaire : l'arrï¿½t pLastStop est aprï¿½s l'arrï¿½t pFirstStop sur la ligne pLine ...

    std::vector<Passenger> & waiters = pFirstStop->getRealStockRestant(pLine);
    waiters.push_back(Passenger(externalUserID, pLastStop));

    return 0;
}



void Reseau::GetPlaqueFromID(const std::string & plaqueId, SymuViaTripNode * & pParentZone, CPlaque * & pPlaque)
{
    pParentZone = NULL;
    ZoneDeTerminaison * pZone;
    for (size_t iZone = 0; iZone < Liste_zones.size(); iZone++)
    {
        pZone = Liste_zones[iZone];
        for (size_t iPlaque = 0; iPlaque < pZone->GetLstPlaques().size(); iPlaque++)
        {
            if (pZone->GetLstPlaques()[iPlaque]->GetID() == plaqueId)
            {
                pPlaque = pZone->GetLstPlaques()[iPlaque];
                pParentZone = pZone;
                return;
            }
        }
    }
}


//=================================================================
    int	Reseau::CreateVehicle
//----------------------------------------------------------------
// Fonction  : CrÃ©ation sur ordre d'un vÃ©hicule pour un trip et
//             une flotte donnÃ©s
// Remarque  :
// Version du:
// Historique:
//=================================================================
(
    Trip * pTrip,
    AbstractFleet * pFleet
)
{
    VehicleToCreate * pVehicle = new VehicleToCreate(IncLastIdVeh(), pFleet);
    pVehicle->SetTrip(pTrip);
    m_VehiclesToCreate.push_back(pVehicle);

    return pVehicle->GetVehicleID();
}

//=================================================================
    int	Reseau::AddDeliveryPoint
//----------------------------------------------------------------
// Fonction  : Ajout d'un point de livraison Ã  une tournÃ©e
//             existante
// Remarque  :
// Version du:
// Historique:
//=================================================================
(
    Trip * pTournee,
    int vehiculeId,
    PointDeLivraison * pPoint,
    int positionIndex,
    int dechargement,
    int chargement
)
{
    boost::shared_ptr<Vehicule> pVeh;

    // Choix du Trip Ã  modifier (peut s'agir d'un Trip associÃ© Ã  un vÃ©hicule ou non)
    Trip * pTripToChange;
    if(!pTournee)
    {
        pVeh = GetVehiculeFromID(vehiculeId);
        if(!pVeh)
        {
            return -4;
        }
        else
        {
            pTripToChange = pVeh->GetTrip();
            if(!pTripToChange)
            {
                return -5;
            }
        }
    }
    else
    {
        pTripToChange = pTournee;
    }

    bool bOk = GetDeliveryFleet()->AddDeliveryPointToTrip(pTripToChange, pPoint, positionIndex, pVeh.get(), dechargement, chargement);

    return bOk ? 0 : -6;
}


    //=================================================================
    int	Reseau::RemoveDeliveryPoint
//----------------------------------------------------------------
// Fonction  : Suppression d'un point de livraison Ã  une tournÃ©e
//             existante
// Remarque  :
// Version du:
// Historique:
//=================================================================
(
    Trip * pTournee,
    int vehiculeId,
    int positionIndex
)
{
    boost::shared_ptr<Vehicule> pVeh;

    // Choix du Trip Ã  modifier (peut s'agir d'un Trip associÃ© Ã  un vÃ©hicule ou non)
    Trip * pTripToChange;
    if(!pTournee)
    {
        pVeh = GetVehiculeFromID(vehiculeId);
        if(!pVeh)
        {
            return -4;
        }
        else
        {
            pTripToChange = pVeh->GetTrip();
            if(!pTripToChange)
            {
                return -5;
            }
        }
    }
    else
    {
        pTripToChange = pTournee;
    }

    bool bOk = GetDeliveryFleet()->RemoveDeliveryPointFromTrip(pTripToChange, positionIndex, pVeh.get());

    return bOk ? 0 : -6;
}


//=================================================================
    int	Reseau::DriveVehicle
//----------------------------------------------------------------
// Fonction  : Fonction de pilotage d'un vÃ©hicule
// Remarque  :
// Version du:
// Historique:
//=================================================================
(
    int nID,
    const std::string & sTroncon,
    int nVoie,
    double dbPos,
    bool bForce
)
{
    // VÃ©rifications
    boost::shared_ptr<Vehicule> pV;
    	    	
    pV = GetVehiculeFromID(nID);
    if( !pV )
        return -2;

    Tuyau* pT;
    pT = this->GetLinkFromLabel(sTroncon);
    if( !pT)
        return -3;

    if(pT->getNb_voies()<=nVoie)
        return -4;

    if (dbPos > pT->GetLength())
    {
        // for symugame : we have positions a little behind the link ??
        dbPos = pT->GetLength();
        //return -5;
    }

    return pV->Drive((TuyauMicro*)pT, nVoie, dbPos, bForce);
}

//=================================================================
    int	Reseau::DeleteVehicle
//----------------------------------------------------------------
// Fonction  : Suppression d'un vÃ©hicule
// Remarque  :
// Version du:
// Historique:
//=================================================================
(
    int nID
)
{
    // VÃ©rifications
    boost::shared_ptr<Vehicule> pV;
    pV = GetVehiculeFromID(nID);
    if( !pV )
        return-2;

    return pV->Delete();
}

//=================================================================
    int	Reseau::AlterRoute
//----------------------------------------------------------------
// Fonction  : Alter the route of a vehicle
// Remarque  :
// Version du: 15/09/2014
// Historique:
//=================================================================
(
    int nIdVeh,
    const std::deque<std::string> & dqLinks
)
{
    boost::shared_ptr<Vehicule> pV;
    std::vector<Tuyau*> newIti;
    Tuyau* pUpT = NULL;
    Tuyau* pDownT = NULL;

    // Checks
    pV = GetVehiculeFromID(nIdVeh);
	if (!pV)
	{
		log() << Logger::Error << "Reseau::AlterRoute -2" << std::endl;
		return -2;	// This vehicle is not simulated
	}

	if (dqLinks.size() == 0)
	{
		log() << Logger::Error << "Reseau::AlterRoute -3" << std::endl;
		return -3; // The route is empty
	}

    for(size_t LinkIdx = 0; LinkIdx<dqLinks.size(); LinkIdx++)
    {
        pDownT = GetLinkFromLabel(dqLinks[LinkIdx]);
		if (!pDownT)
		{
			log() << Logger::Error << "Reseau::AlterRoute -4" << std::endl;
			return -4;	// This link does not belong to the simulated network
		}

        if( pUpT )
			if (!pDownT->GetCnxAssAm()->IsTuyauAmont(pUpT))
			{
				log() << Logger::Error << "Reseau::AlterRoute -5" << std::endl;
				return -5;	// The route is wrong
			}

        newIti.push_back(pDownT);
        pUpT = pDownT;
    }

	int nRes = pV->AlterRoute(newIti);
	log() << Logger::Error << "Reseau::AlterRoute" << nRes << std::endl;

	return nRes;
}


//=================================================================
    std::vector<Point>	Reseau::GetBoundingRect()
//----------------------------------------------------------------
// Fonction  : Renvoie le rectangle englobant du rÃ©seau
// Remarque  : le premier point est le "topLeft", le second le
//             "bottomRight"
// Version du: 19/06/2012
// Historique: 19/06/2012 (O.Tonck - IPSIS)
//             CrÃ©ation
//=================================================================
{
    std::vector<Point> result;


    Point topLeft;
    topLeft.dbX = 0;
    topLeft.dbY = 0;
    topLeft.dbZ = 0;
    Point bottomRight;
    bottomRight.dbX = 0;
    bottomRight.dbY = 0;
    bottomRight.dbZ = 0;

    bool bAtleastOnePoint = false;

    // parcours des tronÃ§ons
    for(size_t i = 0; i < m_LstTuyaux.size(); i++)
    {
        std::deque<Point*> lstPoints = m_LstTuyaux[i]->GetLstPtsInternes();
        lstPoints.push_back(m_LstTuyaux[i]->GetExtAmont());
        lstPoints.push_back(m_LstTuyaux[i]->GetExtAval());
        for(size_t j = 0; j < lstPoints.size(); j++)
        {
            Point * pPt = lstPoints[j];
            if(!bAtleastOnePoint)
            {
                bAtleastOnePoint = true;
                topLeft.dbX = pPt->dbX;
                topLeft.dbY = pPt->dbY;
                topLeft.dbZ = pPt->dbZ;
                bottomRight.dbX = pPt->dbX;
                bottomRight.dbY = pPt->dbY;
                bottomRight.dbZ = pPt->dbZ;
            }
            else
            {
                topLeft.dbX = min(topLeft.dbX, pPt->dbX);
                topLeft.dbY = max(topLeft.dbY, pPt->dbY);
                topLeft.dbZ = max(topLeft.dbZ, pPt->dbZ);
                bottomRight.dbX = max(bottomRight.dbX, pPt->dbX);
                bottomRight.dbY = min(bottomRight.dbY, pPt->dbY);
                bottomRight.dbZ = min(bottomRight.dbZ, pPt->dbZ);
            }
        }
    }

    // Parcours des tronÃ§ons internes des giratoires
    for(size_t i = 0; i < Liste_giratoires.size(); i++)
    {
        const std::deque<TuyauMicro*> & lstTuyaux = Liste_giratoires[i]->GetTuyauxInternes();
        for(size_t k = 0; k < lstTuyaux.size(); k++)
        {
            std::deque<Point*> lstPoints = lstTuyaux[k]->GetLstPtsInternes();
            lstPoints.push_back(lstTuyaux[k]->GetExtAmont());
            lstPoints.push_back(lstTuyaux[k]->GetExtAval());
            for(size_t j = 0; j < lstPoints.size(); j++)
            {
                Point * pPt = lstPoints[j];
                if(!bAtleastOnePoint)
                {
                    bAtleastOnePoint = true;
                    topLeft.dbX = pPt->dbX;
                    topLeft.dbY = pPt->dbY;
                    topLeft.dbZ = pPt->dbZ;
                    bottomRight.dbX = pPt->dbX;
                    bottomRight.dbY = pPt->dbY;
                    bottomRight.dbZ = pPt->dbZ;
                }
                else
                {
                    topLeft.dbX = min(topLeft.dbX, pPt->dbX);
                    topLeft.dbY = max(topLeft.dbY, pPt->dbY);
                    topLeft.dbZ = max(topLeft.dbZ, pPt->dbZ);
                    bottomRight.dbX = max(bottomRight.dbX, pPt->dbX);
                    bottomRight.dbY = min(bottomRight.dbY, pPt->dbY);
                    bottomRight.dbZ = min(bottomRight.dbZ, pPt->dbZ);
                }
            }
        }
    }


    result.push_back(topLeft);
    result.push_back(bottomRight);

    return result;
}


// Construit la liste complÃ¨te des tuyaux (incluant les tronÃ§ons internes) correspondant Ã  l'itinÃ©raire spÃ©cifiÃ©
std::vector<Tuyau*> Reseau::ComputeItineraireComplet(const std::vector<Tuyau*> & itineraire)
{
    std::vector<Tuyau*> result;
    Tuyau * pLink;

    for(size_t i = 0; i < itineraire.size(); i++)
    {
        pLink = itineraire[i];
        if(i == 0)
        {
            result.push_back(pLink);
        }
        else
        {
            if(result.back()->GetBriqueAval() && result.back()->GetBriqueAval() == pLink->GetBriqueAmont())
            {
                // Cas oÃ¹ on doit rajouter les tuyaux internes
                std::vector<Tuyau*> dqTuyauxInternes;
                result.back()->GetBriqueAval()->GetTuyauxInternes(result.back(), pLink, dqTuyauxInternes);

                for(size_t j = 0; j < dqTuyauxInternes.size(); j++)
                {
                    result.push_back(dqTuyauxInternes[j]);
                }
            }
            result.push_back(pLink);
        }
    }

    return result;
}

// Supprime un convergent de la liste
void Reseau::RemoveConvergent(Convergent *pC)
{
    Liste_convergents.erase(pC->GetID());
}

// Renvoie le dernier (follower) vÃ©hicule de la liste (ils se trouvent tous Ã  la mÃªme position)
boost::shared_ptr<Vehicule> Reseau::GetLastVehicule(vector<boost::shared_ptr<Vehicule> > & vehicules)
{
    boost::shared_ptr<Vehicule> result;
    Vehicule * pCurVeh;
    if(vehicules.size() == 1)
    {
        result = vehicules[0];
    }
    else if(vehicules.size() > 1)
    {
        boost::shared_ptr<Vehicule>  pVehleader;
        for(size_t iVeh = 0; iVeh < vehicules.size() && !result; iVeh++)
        {
            pCurVeh = vehicules[iVeh].get();
            bool bFollowerFound = false;
            for(size_t iVeh2 = 0; iVeh2 < vehicules.size() && !bFollowerFound; iVeh2++)
            {
                if(iVeh != iVeh2)
                {
                    pVehleader = vehicules[iVeh2]->GetLeader();
                    if(pVehleader && pVehleader->GetID() == pCurVeh->GetID())
                    {
                        bFollowerFound = true;
                    }
                }
            }
            // Si pas de follower trouvÃ©, on sÃ©lectionne ce vÃ©hicule
            if(!bFollowerFound)
            {
                result = vehicules[iVeh];
            }
        }

        // Normalement, on ne devrait jamais Ãªtre dans ce cas, mais sinon, on renvoie le dernier vÃ©hicule de la liste (le dernier
        // et pas un autre pour Ãªtre conforme au rÃ©sultat qui aurait Ã©tÃ© rendu avant la correction de l'anomalie 105)
        if(!result)
        {
            result = vehicules.back();
        }
    }
    return result;
}

// Renvoie le premier (leader) vÃ©hicule de la liste (ils se trouvent tous Ã  la mÃªme position)
boost::shared_ptr<Vehicule> Reseau::GetFirstVehicule(vector<boost::shared_ptr<Vehicule> > & vehicules)
{
    boost::shared_ptr<Vehicule> result, pLeader;
    if(vehicules.size() == 1)
    {
        result = vehicules[0];
    }
    else if(vehicules.size() > 1)
    {
        for(size_t iVeh = 0; iVeh < vehicules.size() && !result; iVeh++)
        {
            pLeader = vehicules[iVeh]->GetLeader();
            if(!pLeader)
            {
                result = vehicules[iVeh];
            }
            else
            {
                bool bLeaderFound = false;
                for(size_t iVeh2 = 0; iVeh2 < vehicules.size() && !bLeaderFound; iVeh2++)
                {
                    if(iVeh != iVeh2 && pLeader->GetID() == vehicules[iVeh2]->GetID())
                    {
                        bLeaderFound = true;
                    }
                }

                // Si pas de leader trouvÃ©, on sÃ©lectionne ce vÃ©hicule
                if(!bLeaderFound)
                {
                    result = vehicules[iVeh];
                }
            }
        }

        // Normalement, on ne devrait jamais Ãªtre dans ce cas, mais sinon, on renvoie le dernier vÃ©hicule de la liste (le dernier
        // et pas un autre pour Ãªtre conforme au rÃ©sultat qui aurait Ã©tÃ© rendu avant la correction de l'anomalie 105)
        if(!result)
        {
            result = vehicules.back();
        }
    }
    return result;
}

void Reseau::RestoreSeed(unsigned int randCount)
{
    if(m_uiSeed == 0)
    {
        assert(m_bPickedSeed); // la dÃ©sÃ©rialisation si la graine n'a pas encore Ã©tÃ© tirÃ©e n'a pas de sens
        m_pRandManager->mySrand(m_uiRandomSeed, randCount);
    }
    else
    {
        m_pRandManager->mySrand(m_uiSeed, randCount);
    }
}

bool Reseau::prvGenerateVehicule()
{
    for(size_t iFleet = 0; iFleet < m_LstFleets.size(); iFleet++)
    {
        m_LstFleets[iFleet]->ActivateVehicles(m_dbInstSimu+m_dbDiffDebutSimuData, GetTimeStep());
    }
    return true;
}

void Reseau::SimuTraficMicroMacro()
{
    std::deque <TuyauMacro*>::iterator tmaccourant;
    std::deque <TuyauMacro*>::iterator tmacdebut;
    std::deque <TuyauMacro*>::iterator tmacfin;

    std::deque <TuyauMicro*>::iterator tmiccourant;
    std::deque <TuyauMicro*>::iterator tmicdebut;
    std::deque <TuyauMicro*>::iterator tmicfin;

    std::deque <Tuyau*>::iterator tcourant;
    std::deque <Tuyau*>::iterator tdebut;
    std::deque <Tuyau*>::iterator tfin;

    tdebut = m_LstTuyaux.begin();
    tfin = m_LstTuyaux.end();

    tmacdebut = m_LstTuyauxMacro.begin();
    tmacfin = m_LstTuyauxMacro.end();

    tmicdebut = m_LstTuyauxMicro.begin();
    tmicfin = m_LstTuyauxMicro.end();

    // --- ProcÃ©dure de dÃ©passement --- //
    if(m_bDepassement)
        Depassement(m_dbInstSimu+m_dbDiffDebutSimuData);

    // --- ProcÃ©dure de changement de voie --- //
    if(m_bChgtVoie)
        ChangementDeVoie(m_dbInstSimu+m_dbDiffDebutSimuData);

    // Calcul du leader de chaque voie.
    ComputeFirstVehicules();

    // --- Calcul des caractÃ©ristques du trafic pour chaque tuyau macroscopique
    for (tmaccourant=tmacdebut;tmaccourant!=tmacfin;++tmaccourant)
    {
        (*tmaccourant)->ComputeTraffic(m_dbInstSimu+m_dbDiffDebutSimuData);
    }
     // Compteur temps 4
    //t1 = GetTickCount();

    // Parcours des tuyaux
    for (tcourant=tdebut;tcourant!=tfin;++tcourant)
    {
        if(  (*tcourant)->GetType() != Tuyau::TT_MESO)
        {
            (*tcourant)->CalculEcoulementConnections(m_dbInstSimu+m_dbDiffDebutSimuData); //
        }
    }

    // Parcours des repartiteurs
    std::map<std::string, Repartiteur*>::iterator rep;
    std::map<std::string, Repartiteur*>::iterator rep_d = Liste_repartiteurs.begin();
    std::map<std::string, Repartiteur*>::iterator rep_f = Liste_repartiteurs.end();
    for (rep=rep_d; rep!=rep_f; ++rep)
    {
        rep->second->CalculEcoulementConnections(m_dbInstSimu+m_dbDiffDebutSimuData);
    }

    for (tcourant=tdebut;tcourant!=tfin;++tcourant)
    {
        if(  (*tcourant)->GetType() != Tuyau::TT_MESO)
        {

            (*tcourant)->FinCalculEcoulementConnections(m_dbInstSimu+m_dbDiffDebutSimuData);
        }
    }
     // --- MAJ des SAS d'entrÃ©e sortie des tuyaux microscopiques
    for (tmiccourant=tmicdebut;tmiccourant!=tmicfin;++tmiccourant)
    {
        (*tmiccourant)->ComputeTraffic(m_dbInstSimu+m_dbDiffDebutSimuData);
    }

    // On relance les calculs du rÃ©partiteur si besoin (modÃ©lisation microscopique)
    if(m_bRelancerCalculRepartiteur)
    {
        for (tcourant=tdebut;tcourant!=tfin;++tcourant)
        {
            if(  (*tcourant)->GetType() != Tuyau::TT_MESO)
            {
                (*tcourant)->CalculEcoulementConnections(m_dbInstSimu+m_dbDiffDebutSimuData);
            }
        }

         for (rep=rep_d; rep!=rep_f; ++rep)
             rep->second->CalculEcoulementConnections(m_dbInstSimu+m_dbDiffDebutSimuData);

        for (tcourant=tdebut;tcourant!=tfin;++tcourant)
        {
            if(  (*tcourant)->GetType() != Tuyau::TT_MESO)
            {
                (*tcourant)->FinCalculEcoulementConnections(m_dbInstSimu+m_dbDiffDebutSimuData);
            }
        }

        for (tmiccourant=tmicdebut;tmiccourant!=tmicfin;++tmiccourant)
            (*tmiccourant)->EndComputeTraffic(m_dbInstSimu+m_dbDiffDebutSimuData);
    }
     // --- Calcul des autres caractÃ©ristiques du trafic pour chaque tuyau
    for (tmaccourant=tmacdebut;tmaccourant!=tmacfin;++tmaccourant)
    {
        (*tmaccourant)->ComputeTrafficEx(m_dbInstSimu+m_dbDiffDebutSimuData);
    }
    //t2 = GetTickCount();
    //*m_pFicSimulation << t2-t1 << " ";

    // Mise Ã  jour des SAS

    // Compteur temps 5
    //t1 = GetTickCount();
     for (rep=rep_d; rep!=rep_f; ++rep)
    {
        rep->second->MiseAJourSAS(m_dbInstSimu+m_dbDiffDebutSimuData);
    }
    //t2 = GetTickCount();
    //*m_pFicSimulation << t2-t1 << " ";

         // --- MAJ de la position des vÃ©hicule existant sur le rÃ©seau

    // Compteur temps 6
    //t1 = GetTickCount();

    // Nettoyage si besoin pour le mode de dÃ©passement pour changement de direction pour Charlotte
    if (m_bModeDepassementChgtDir)
    {
        for (tmiccourant = tmicdebut; tmiccourant != tmicfin; ++tmiccourant)
        {
            (*tmiccourant)->m_mapVehiclesForDirChangeOvertakeMode.clear();
        }
    }

    MiseAJourVehicules(m_dbInstSimu);

      //t2 = GetTickCount();
    //*m_pFicSimulation << t2-t1 << " ";

    // Reprise des calculs des trajectoires pour le vÃ©hicule passant un tuyau avec une traversÃ©e
    //if(this->m_bDebug)
        //*m_pFicSimulation << "Tentative insertion vÃ©hicule en attente : ";
    std::deque<SymuViaTripNode*>::iterator itOrigine;
    for( itOrigine = Liste_origines.begin(); itOrigine != Liste_origines.end(); ++itOrigine)
    {
        (*itOrigine)->InsertionVehEnAttente(m_dbInstSimu + m_dbDiffDebutSimuData);
    }
    //*m_pFicSimulation << std::endl;

        // Compteur temps 7
    //t1 = GetTickCount();

    //t2 = GetTickCount();
    //*m_pFicSimulation << t2-t1 << " ";

    // Reprise des calculs des trajectoires pour le vÃ©hicule passant un convergent

    // Compteur m_dbInstSimu 8
    //t1 = GetTickCount();
    std::map<std::string, Convergent*>::iterator itConv;
    for (itConv = Liste_convergents.begin(); itConv != Liste_convergents.end(); ++itConv)
    {
        itConv->second->CalculInsertion(m_dbInstSimu+m_dbDiffDebutSimuData);
        itConv->second->ClearLstInsVeh();
    }
    //t2 = GetTickCount();
    //*m_pFicSimulation << t2-t1 << " ";
 // Fin calcul des trajectoires des vÃ©hicules

    // ProcÃ©dure d'agressivitÃ©
    if(m_bProcAgressivite)
        ProcedureAgressivite(m_dbInstSimu);

    // Mode anticongestion Charlotte
    if (m_bModeDepassementChgtDir)
    {
        ProcedureDepassementChgtDirection();
    }

#ifdef USE_SYMUCOM
    if(m_pSymucomSimulator)
    {
        //Simulation des systÃ¨mes connÃ©ctÃ©s
        boost::posix_time::ptime dtActualTime = GetSimulationStartTime() + boost::posix_time::microseconds((int)(m_dbInstSimu * 1000000.0));
        // 1 : on met a jour les capteurs
        for(size_t i=0; i<m_pSymucomSimulator->GetStations().size(); i++)
        {
            ITSStation* pStation = m_pSymucomSimulator->GetStations()[i];
            pStation->UpdateSensorsData(dtActualTime, pas_de_temps * 1000.0);
        }

        // 2 : execution des applications qui ont des actions dÃ©finies sans messages de paramÃ¨tres
        for(size_t i=0; i<m_pSymucomSimulator->GetApplications().size(); i++)
        {
            m_pSymucomSimulator->GetApplications()[i]->RunApplication(NULL, m_pSymucomSimulator);
        }

        // 3 : Ã©xecution de chacune des stations
        for(size_t i=0; i<m_pSymucomSimulator->GetStations().size(); i++)
        {
            ITSStation* pStation = m_pSymucomSimulator->GetStations()[i];
            pStation->Run(dtActualTime, pas_de_temps);
        }

        // 4 : execution de l'Ã©coulement des messages par symucom
        m_pSymucomSimulator->GetCommunicationRunner()->Run((int)(pas_de_temps * 1000.0));

        // 5 : Modification des variable de chacun des vÃ©hicules
        m_pSymucomSimulator->ArbitrateVehicleChange();
    }
#endif // USE_SYMUCOM

    // Compteur temps 9
    //t1 = GetTickCount();
    FinCalculVehicules(m_dbInstSimu);
    //t2 = GetTickCount();
    //*m_pFicSimulation << t2-t1 << " ";

    // Gestion des capteurs
    if(m_pGestionsCapteur)
        m_pGestionsCapteur->CalculInfoCapteurs(m_dbInstSimu);

#ifdef USE_SYMUCORE
    if (IsSymuMasterMode())
    {
        for (size_t iVeh = 0; iVeh < m_LstVehicles.size(); iVeh++)
        {
            boost::shared_ptr<Vehicule> pVeh = m_LstVehicles[iVeh];

            // RÃ©cupÃ©ration des capteurs potentiellement impactÃ©s par le vÃ©hicule :
            std::set<AbstractSensor*> potentiallyImpactedSensors = pVeh->GetPotentiallyImpactedSensors(NULL);
            // Traitement des capteurs potentiellement impactÃ©s
            for (std::set<AbstractSensor*>::iterator iterSensor = potentiallyImpactedSensors.begin(); iterSensor != potentiallyImpactedSensors.end(); ++iterSensor)
            {
                (*iterSensor)->CalculInfoCapteur(this, m_dbInstSimu, false, -1, pVeh);
            }
        }
    }
#endif // USE_SYMUCORE


    if (m_pTravelTimesOutputManager)
        m_pTravelTimesOutputManager->CalculInfoCapteurs(m_dbInstSimu);

    // Correction de l'accÃ©lÃ©ration pour les vÃ©hicules sortant d'un giratoire

    // Compteur temps 10
    //t1 = GetTickCount();
    std::map<std::string, Divergent*>::iterator div;
    std::map<std::string, Divergent*>::iterator div_d = Liste_divergents.begin();
    std::map<std::string, Divergent*>::iterator div_f = Liste_divergents.end();
    for (div=div_d; div!=div_f; ++div)
        div->second->CorrigeAcceleration(m_dbInstSimu);
    //t2 = GetTickCount();
    //*m_pFicSimulation << t2-t1 << " ";

    // Nettoyage des listes des connections de vÃ©hicules susceptible de s'insÃ©rer
    for (div=div_d; div!=div_f; ++div)
        div->second->ClearLstInsVeh();

    for (itConv = Liste_convergents.begin(); itConv != Liste_convergents.end(); ++itConv)
        itConv->second->ClearLstInsVeh();

    for (rep=rep_d; rep!=rep_f; ++rep)
        rep->second->ClearLstInsVeh();

      // Compteur temps 11
    //t1 = GetTickCount();
    SupprimeVehicules(m_dbInstSimu);
    //t2 = GetTickCount();
    //*m_pFicSimulation << t2-t1 << " ";

    // Mise Ã  jour des capteurs des convergents

    // Compteur temps 12
    //t1 = GetTickCount();
    // prÃ©paration de la structure de rÃ©sultats pour les donnÃ©es du pas de temps courant
    for (itConv = Liste_convergents.begin(); itConv != Liste_convergents.end(); ++itConv)
        itConv->second->UpdateCapteurs();

      //t2 = GetTickCount();
    //*m_pFicSimulation << t2-t1 << " ";

    // --- ProcÃ©dure de changement de voie mandatory ---

    // MAJ du nombre de vÃ©hicules sur les voies rÃ©duites
    for (tmiccourant=tmicdebut;tmiccourant!=tmicfin;tmiccourant++)
    {
        TuyauMicro *pT = (*tmiccourant);
        for( int nV = 0; nV !=pT->getNbVoiesDis(); nV++)
            if( ((VoieMicro*)pT->GetLstLanes()[nV])->IsChgtVoieObligatoire() )
                ((VoieMicro*)pT->GetLstLanes()[nV])->UpdateNbVeh();
    }


    // Mode asynchrone de gestion des merging objects
    if( this->m_cChgtVoieMandatoryMode == 'A' )
    {
        // MAJ des capteurs des merging objects et calcul des probabilitÃ©s
        std::deque<MergingObject*>::iterator itMO;
        for(itMO = this->m_LstMergingObjects.begin(); itMO != m_LstMergingObjects.end(); itMO++)
        {
            (*itMO)->UpdateInfoCapteurs();
        }
    }


    // traitements des capteurs non utilisateur (convergents et mergingobjects)
    UpdateConvergents();

        // Mode asynchrone de gestion des merging objects
    if( this->m_cChgtVoieMandatoryMode == 'A' )
    {
        // MAJ des capteurs des merging objects et calcul des probabilitÃ©s
        std::deque<MergingObject*>::iterator itMO;
        for(itMO = this->m_LstMergingObjects.begin(); itMO != m_LstMergingObjects.end(); itMO++)
        {
            (*itMO)->ProbabilityCalculation(m_dbInstSimu);
        }
    }
}

void Reseau::ProcedureDepassementChgtDirection()
{
    std::deque <TuyauMicro*>::iterator tmiccourant;
    std::deque <TuyauMicro*>::iterator tmicdebut;
    std::deque <TuyauMicro*>::iterator tmicfin;

    tmicdebut = m_LstTuyauxMicro.begin();
    tmicfin = m_LstTuyauxMicro.end();

    for (tmiccourant = tmicdebut; tmiccourant != tmicfin; ++tmiccourant)
    {
        TuyauMicro * pTuy = *tmiccourant;

        if (pTuy->getNb_voies() == 1)
        {
            const std::map<Tuyau*, Vehicule*> & mapCandidates = pTuy->m_mapVehiclesForDirChangeOvertakeMode;

            // tri des vÃ©hicules du plus proche de la fin du tronÃ§on au moins proche de la fin :
            std::map<double, std::vector<std::map<Tuyau*, Vehicule*>::const_iterator > > orderedCandidates;
            for (std::map<Tuyau*, Vehicule*>::const_iterator iter = mapCandidates.begin(); iter != mapCandidates.end(); ++iter)
            {
                orderedCandidates[pTuy->GetLength() - iter->second->GetPos(0)].push_back(iter);
            }

            // traitement des candidates par ordre de prioritÃ© :
            bool bCandidateFound = false;
            for (std::map<double, std::vector<std::map<Tuyau*, Vehicule*>::const_iterator > >::const_iterator iter = orderedCandidates.begin(); iter != orderedCandidates.end() && !bCandidateFound; ++iter)
            {
                for (size_t iCandidate = 0; iCandidate < iter->second.size() && !bCandidateFound; iCandidate++)
                {
                    const std::map<Tuyau*, Vehicule*>::const_iterator & iter2 = iter->second.at(iCandidate);
                    Tuyau * pTargetLink = iter2->first;

                    // s'il y a un feu rouge, on passe Ã  la suite.
                    ControleurDeFeux * pCDF = pTuy->GetCnxAssAv()->GetCtrlDeFeux();
                    if (pCDF)
                    {
                        if (pCDF->GetCouleurFeux(m_dbInstSimu, pTuy, pTargetLink) != FEU_VERT)
                        {
                            continue;
                        }
                    }

                    Vehicule * pCandidate = iter2->second;

                    assert(pCandidate->GetLink(0) == pTuy); // si le vehicule est candidat, il appartient forcÃ©ment au tuyau traitÃ©, sinon problÃ¨me (si ca arrive, Ã  comprendre. au pire, ce devrait Ãªtre rare et on pourrait remplacer l'assertion par un if ?)

                    bool bIsOk = true;
                    bool bHasAtLeastOneLeader = false;
                    Vehicule * pLeader = pCandidate->GetLeader().get();

                    // on remonte la chaÃ®ne des leaders jusqu'Ã  changer de tronÃ§on ou que le tuyau suivant du leader soit le mÃªme que pTargetLink !
                    while (bIsOk && pLeader && pLeader->GetLink(0) == pTuy)
                    {
                        // Si on trouve un leader qui va vers le tuyau pTargetLink, on rejÃ¨te le candidat.
                        Tuyau * pLeaderTargetLink = pLeader->CalculNextTuyau(pTuy, m_dbInstSimu);
                        if (pLeaderTargetLink == pTargetLink)
                        {
                            bIsOk = false;
                        }
                        else
                        {
                            // Passage au leader suivant
                            bHasAtLeastOneLeader = true;
                            pLeader = pLeader->GetLeader().get();
                        }
                    }

                    // Si aucun leader sur le mÃªme tuyau, c'est que ca avance : on ne fait rien !
                    if (bIsOk && bHasAtLeastOneLeader)
                    {
                        Tuyau * pNextLink;
                        if (pTuy->GetBriqueAval())
                        {
                            // Dans le cas d'une brique, on positionne le vÃ©hicule sur le premier tronÃ§on du mouvement interne du mouvement qui va bien :

                            // rÃ©cupÃ©ration d'un numÃ©ro de voie autorisÃ© 
                            int iTargetLane = 0;
                            for (int iOutlane = 0; iOutlane < (int)pTargetLink->GetLstLanes().size(); iOutlane++)
                            {
                                if (pTuy->GetBriqueAval()->IsMouvementAutorise(pTuy, pTargetLink->GetLstLanes().at(iOutlane), pCandidate->GetType(), &pCandidate->GetSousType()))
                                {
                                    iTargetLane = iOutlane;
                                    break;
                                }
                            }
                            
                            std::vector<Tuyau*> internalLinks;
                            pTuy->GetBriqueAval()->GetTuyauxInternes(pTuy, 0, pTargetLink, iTargetLane, internalLinks);

                            assert(!internalLinks.empty());

                            pNextLink = internalLinks.front();

                            pCandidate->SetVoie((VoieMicro*)pNextLink->GetLstLanes().front()); // Une seule voie car tuyau interne

                        }
                        else
                        {
                            pNextLink = pTargetLink;

                            pCandidate->SetVoie((VoieMicro*)pCandidate->CalculNextVoie(pTuy->GetLstLanes().front(), m_dbInstSimu));
                            assert(pCandidate->GetVoie(0));
                        }

                        pCandidate->SetTuyau(pNextLink, 0);

                        pCandidate->SetPos(0);

                        double dbEffectiveSpeed = iter->first / pas_de_temps;
                        double dbFinalSpeed = std::max<double>(pCandidate->GetVit(1), std::min<double>(dbEffectiveSpeed, pCandidate->GetVitMax()));
                        pCandidate->SetVit(dbFinalSpeed);

                        double dbEffectiveAcc = (dbFinalSpeed - pCandidate->GetVit(1)) / pas_de_temps;
                        pCandidate->SetAcc(std::min<double>(pCandidate->GetAccMax(dbFinalSpeed), std::max<double>(pCandidate->GetAcc(1), dbEffectiveAcc)));

                        // On sort de la boucle
                        bCandidateFound = true;
                    }
                }
            }
        }
    }
}

// Fonction de calcul du numÃ©ro de voie d'un vÃ©hicule qui sort du mode mÃ©so
int Reseau::ComputeAvailableLaneNumber(Vehicule * pVeh, TuyauMicro * pLink, Tuyau * pPrevLink, double dbWishedPos, std::map<int, double> * mapLastVehPosOnEachLane, double & dbAvailablePos)
{
    // On tire alÃ©atoirement une voie sur laquelle il reste la place de s'insÃ©rer, parmis les voies permettant
    // de tourner vers la bonne direction le cas Ã©chÃ©ant.
    // Si aucune voie possible : on tire une voie quand mÃªme, avec warning.

    Connexion *pCon = NULL;
    if (pPrevLink)
    {
        pCon = pLink->GetCnxAssAm();
        assert(pCon == pPrevLink->GetCnxAssAv());
    }

    double dbMinSpacing = 1.0 / pVeh->GetDiagFonda()->GetKMax();

    // Pour vÃ©rification de l'Ã©ligibilitÃ© de la voie en regarde du changement de destination Ã©ventuel :
    Tuyau * pNextTuy = NULL;
    if (!pPrevLink)
    {
        if (dbWishedPos >= pLink->GetLength() - pLink->GetDstChgtVoie(GetDstChgtVoie()))
        {
            size_t iItiIndex = pVeh->GetItineraireIndex(isSameTuyauPredicate, pLink);
            if (iItiIndex != -1 && pVeh->GetItineraire()->size() > iItiIndex + 1)
            {
                pNextTuy = pVeh->GetItineraire()->at(iItiIndex + 1);
            }
        }
    }

    // Ensemble des voies atteignables du tronÃ§on si un tronÃ§on prÃ©cÃ©dent est dÃ©fini (il doit y avoir un mouvement autorisÃ©
    // conduisant vers la voie pour qu'elle soit possible)
    std::set<int> possibleLanes;
    std::set<int> preferedLanes;
    for (int iLane = 0; iLane < pLink->getNb_voies(); iLane++)
    {
        // Si pas de connexion ou giratoire, on ne peut pas se baser sur les mouvements autorisÃ©s.
        if (!pCon || dynamic_cast<Giratoire*>(pCon) || pCon->IsMouvementAutorise(pPrevLink, pLink->GetLstLanes()[iLane], pVeh->GetType(), NULL))
        {
            // VÃ©rification de l'Ã©ligibilitÃ© de la voie au changment de voie pour destination
            if (!pNextTuy || dynamic_cast<Giratoire*>(pNextTuy->GetCnxAssAm()) || pNextTuy->GetCnxAssAm()->IsMouvementAutorise(pLink->GetLstLanes()[iLane], pNextTuy, pVeh->GetType(), NULL))
            {
                preferedLanes.insert(iLane);
            }
            possibleLanes.insert(iLane);
        }
    }

    // Si pas de mapLastVehPosOnEachLane, on la construit pour ensuite pouvoir travailler gÃ©nÃ©riquement
    bool bOwnMap = false;
    if (!mapLastVehPosOnEachLane)
    {
        mapLastVehPosOnEachLane = new std::map<int, double>();
        bOwnMap = true;
        for (std::set<int>::iterator iter = possibleLanes.begin(); iter != possibleLanes.end(); ++iter)
        {
            boost::shared_ptr<Vehicule> pLastVehicle = GetLastVehicule((VoieMicro*)pLink->GetLstLanes()[*iter], pVeh);
            if (pLastVehicle) {
                (*mapLastVehPosOnEachLane)[*iter] = pLastVehicle->GetPos(1);
            }
        }
    }

    // Constitution de la liste des voies pour lesquels on a la place de s'insÃ©rer Ã  la position voulue en respectant le 1/kx :
    std::set<int> availableLanes;
    for (std::set<int>::iterator iter = possibleLanes.begin(); iter != possibleLanes.end(); ++iter)
    {
        std::map<int, double>::const_iterator iterMap = mapLastVehPosOnEachLane->find(*iter);
        if (iterMap != mapLastVehPosOnEachLane->end())
        {
            // La voie n'est dispo que si le dernier vÃ©hicule est suffisemment loin devant (respect du 1/kx)
            if (iterMap->second >= dbWishedPos + dbMinSpacing)
            {
                availableLanes.insert(*iter);
            }
        }
        else
        {
            // Pas de vÃ©hicule dÃ©jÃ  prÃ©sent sur la voie : la voie est dispo
            availableLanes.insert(*iter);
        }
    }

    std::set<int> availablePreferedLanes;
    for (std::set<int>::iterator iter = availableLanes.begin(); iter != availableLanes.end(); ++iter)
    {
        if (preferedLanes.find(*iter) != preferedLanes.end())
        {
            availablePreferedLanes.insert(*iter);
        }
    }

    std::set<int> randomLanes;
    if (!availablePreferedLanes.empty())
    {
        randomLanes = availablePreferedLanes;
    }
    else if (!availableLanes.empty())
    {
        randomLanes = availableLanes;
    }
    else
    {
        randomLanes = possibleLanes;
    }

    // tirage :
    double dbRand = m_pRandManager->myRand() / (double)MAXIMUM_RANDOM_NUMBER;
    int indexVoieMicro = (int)ceil(dbRand * (double)randomLanes.size()) - 1;
    std::set<int>::iterator iter = randomLanes.begin();
    for (int i = 0; i < indexVoieMicro; i++)
    {
        iter++;
    }
    int iVoieMicro = *iter;

    // positionnement de dbAvailablePos (voir les diffÃ©rents cas de figure possibles ci-dessous :)
    std::map<int, double>::const_iterator iterMap = mapLastVehPosOnEachLane->find(iVoieMicro);
    if (iterMap != mapLastVehPosOnEachLane->end())
    {
        // Le vÃ©hicule peut ne pas atteindre la position souhaitÃ©e :
        dbAvailablePos = std::max<double>(0.0, std::min<double>(dbWishedPos, iterMap->second - dbMinSpacing));
    }
    else
    {
        // Pas de vÃ©hicule dÃ©jÃ  prÃ©sent sur la voie : le vÃ©hicule peut prendre la position dÃ©sirÃ©e de toute faÃ§on
        dbAvailablePos = dbWishedPos;
    }


    if (bOwnMap) {
        delete mapLastVehPosOnEachLane;
    }

    return iVoieMicro;
}

void Reseau::SimuTraficMeso()
{
        // Assignation du document XML de trafic (temporaire dans le cas d'une affectation calculÃ©e convergente)

    std::deque<SymuViaTripNode*>::iterator itOrigine;



    // Sondage des vitesses rÃ©glementaires : si elles ont Ã©voluÃ©es par rapport au pas de temps prÃ©cÃ©dent, un nouveau calcul d'itinÃ©raire doit Ãªtre effectuÃ©
    // ou un nouveau calcul d'affectation selon type de comportement des vÃ©hicules

    std::deque<TypeVehicule*>::iterator itTV;

    std::deque <CTuyauMeso*>::iterator tmeccourant;
    std::deque <CTuyauMeso*>::iterator tmecdebut;
    std::deque <CTuyauMeso*>::iterator tmecfin;
    std::deque<CMesoNode *>::iterator itNode;
    std::deque<Tuyau*>::iterator itTuyau;

    // INITIALISATION OF NODE TO BE UPDATED
    m_nodeToUpdate.clear();

    // I- UPDATE OF THE NEXT SUPPLY TIME ON DOWNSTREAM BOUNDARIES OF NODES DUE TO POSTPONED CALCULATION
    std::deque<Connexion *>::iterator itConn;
    for( itConn = m_LstUserCnxs.begin(); itConn != m_LstUserCnxs.end(); ++itConn)
    {
        for( itTuyau =(*itConn)->m_LstTuyAv.begin();
            itTuyau != (*itConn)->m_LstTuyAv.end();
            itTuyau++)
        {
            Tuyau *pTuyauBase = *itTuyau;

            // Postponed calculation
            if( (*itConn)->GetNextSupplyTime(pTuyauBase ) == DBL_MAX)
            {
                // Cas meso
                if(pTuyauBase->GetType() == Tuyau::TT_MESO)
                {
                    CTuyauMeso * pTuyauMeso = (CTuyauMeso *)(*itTuyau);
                    if (((CTuyauMeso *)(*itTuyau))->GetCurrentVehiculeAt(0))
                    {
                        //Constraint on capacity at the beginning of the link

                        Vehicule *pLastExistingVehicule = pTuyauMeso->GetCurrentVehiculeAt(0);
                        double dLastExistingTime = pTuyauMeso->GetDownstreamPassingTime((*itConn)->GetRankNextOutgoingVehicule(pTuyauMeso) - 1);

                        double dQx = fabs(pLastExistingVehicule->GetType()->GetW()) *pLastExistingVehicule->GetType()->GetVx()*(pLastExistingVehicule->GetType()->GetKx()) /
                            (fabs(pLastExistingVehicule->GetType()->GetW()) + pLastExistingVehicule->GetType()->GetVx());
                        double dNextSupplyTimeCapacity = dLastExistingTime + 1.0 / (pTuyauMeso->getNb_voies()* dQx);

                        //Constraint coming from downstream boundary (end of the outgoing link) l 620
                        double dNextSupplyTimeDownstream = -DBL_MAX;
                        double dNextVehToEnterRank = (*itConn)->GetRankNextOutgoingVehicule(pTuyauMeso);
                        double dTuyauMeanK = pTuyauMeso->GetMeanK();

                        double dDownstreamRefVehRank = dNextVehToEnterRank - pTuyauMeso->getNb_voies() * pTuyauMeso->GetLength() * dTuyauMeanK;
                        if (dDownstreamRefVehRank > 1)
                        {
                            int iRefVehRankInteger = (int)floor(dDownstreamRefVehRank);
                            double dRefVehRankResidue = dDownstreamRefVehRank - iRefVehRankInteger;
                            Connexion *pConnDownstream = pTuyauMeso->GetCnxAssAv();
                            std::deque<double> listPassingTime = pConnDownstream->GetUpstreamPassingTimes(pTuyauMeso);// (CTuyauMeso *)pLastExistingVehicule->GetPreviousTuyau() );

                            double dDownstreamRefTime = DBL_MAX;

                            if (iRefVehRankInteger >= (int)listPassingTime.size())
                            {
                                // We miss one vehicle to properly estimate the passing time of
                                // the reference vehicle at the downstream boundary. This
                                // calculation is then postpone to the next global event time
                                //dDownstreamRefTime = DBL_MAX;
                                dNextSupplyTimeDownstream = DBL_MAX;
                            }
                            else
                            {
                                dDownstreamRefTime = listPassingTime.at(iRefVehRankInteger - 1) + dRefVehRankResidue*(listPassingTime.at(iRefVehRankInteger) - listPassingTime.at(iRefVehRankInteger - 1));
                                if (pTuyauMeso->GetMeanW() != 0.0)
                                {
                                    dNextSupplyTimeDownstream = dDownstreamRefTime + pTuyauMeso->GetLength() / fabs(pTuyauMeso->GetMeanW());
                                    if ((*itConn)->IsAnOrigine() && (*itConn)->GetNextArrival(NULL).second == NULL)
                                    {
                                        (*itConn)->SetNextArrival(NULL, (*itConn)->GetMinArrival(NULL));
                                    }
                                }
                                else
                                {
                                    dNextSupplyTimeDownstream = DBL_MAX;
                                }
                            }


                        } // fi dDownstreamRefVehRank>=0

                        (*itConn)->SetNextSupplyTime(pTuyauMeso, max(dNextSupplyTimeCapacity, dNextSupplyTimeDownstream));
                    }
                    else
                    {
                        // Pas de vÃ©hicule sur le tronÃ§on meso : cas du tronÃ§on micro qui est passÃ© mÃ©so :
                        // On autorise le passage direct
                        (*itConn)->SetNextSupplyTime(pTuyauMeso, -DBL_MAX);
                    }
                    m_nodeToUpdate.push_back(*itConn);
                } // cas aval mesoscopique
                else // sinon cas microscopique
                {
                    if(pTuyauBase->GetType() == Tuyau::TT_MICRO)
                    {

                        TuyauMicro * pDownStreamMicro = (TuyauMicro *) pTuyauBase;
                        double dNextSupplyTimeCapacity = DBL_MAX;
                        int iVoie;

                        for( iVoie = 0; iVoie<pDownStreamMicro->getNb_voies(); ++iVoie)
                        {
                            // On ne prend en compte ques les voies accessibles directement
                            bool bAccessible = false;
                            for (size_t iAm = 0; iAm < (*itConn)->m_LstTuyAm.size(); iAm++)
                            {
                                if (dynamic_cast<Giratoire*>(*itConn) || (*itConn)->IsMouvementAutorise((*itConn)->m_LstTuyAm[iAm], pDownStreamMicro->GetVoie(iVoie), NULL, NULL))
                                {
                                    bAccessible = true;
                                    break;
                                }
                            }
                            if (bAccessible)
                            {
                                boost::shared_ptr<Vehicule> pNextArringVeh = GetLastVehicule((VoieMicro*)pDownStreamMicro->GetVoie(iVoie));

                                if (pNextArringVeh.get())
                                {

                                    boost::shared_ptr<Vehicule> pLastWayVehicule = pNextArringVeh->GetLeader();
                                    double dPosLeader = DBL_MAX;
                                    if (pLastWayVehicule.get() &&
                                        pDownStreamMicro->m_mapVeh.find(pNextArringVeh->GetID()) != pDownStreamMicro->m_mapVeh.end())
                                    {
                                        dPosLeader = GetDistanceEntreVehicules(pNextArringVeh.get(), pLastWayVehicule.get(), true);
                                    }

                                    double dQ = fabs(pNextArringVeh->GetType()->GetW())*(pNextArringVeh->GetType()->GetKx() - 1.0 /
                                        max(1.0 / pNextArringVeh->GetDiagFonda()->GetKCritique(), dPosLeader));


                                    if (!(1.0 / dQ < pNextArringVeh->GetType()->GetKx() || pNextArringVeh->GetPos(0) < 1.0 / pNextArringVeh->GetType()->GetKx()))
                                    {
                                        double dLastEnter;
                                        if (pDownStreamMicro->m_mapVeh.find(pNextArringVeh->GetID()) != pDownStreamMicro->m_mapVeh.end())
                                        {
                                            dLastEnter = pDownStreamMicro->m_mapVeh.find(pNextArringVeh->GetID())->second.second.first;
                                        }
                                        else
                                        {
                                            // le temps d'entrÃ©e du vÃ©hicule a pu Ãªtre ratÃ©... si pas notÃ© non plus, renvoie DBL_MAX,
                                            // ce qui ramÃ¨ne au cas dNextSupplyTimeCapacity = DBL_MAX; ....
                                            dLastEnter = pNextArringVeh->GetTimeCode(pDownStreamMicro, Vehicule::TC_INCOMMING);
                                        }
                                        // rmq. : problÃ¨me de blocage ici si m_dbInstSimuMeso vaut DBL_MAX : on ne pourra jamais dÃ©bloquer quelquesoit le nouveau supply time calculÃ© !
                                        //dNextSupplyTimeCapacity = max( dNextSupplyTimeCapacity,  max( max(m_dbInstSimu - pas_de_temps, m_dbInstSimuMeso),  dLastEnter +1.0/dQ ));
                                        double dNextSupplyTimeCapacityForLane = max(m_dbInstSimu - pas_de_temps, dLastEnter + 1.0 / dQ);
                                        if (m_dbInstSimuMeso != DBL_MAX)
                                        {
                                            dNextSupplyTimeCapacityForLane = max(dNextSupplyTimeCapacityForLane, m_dbInstSimuMeso);
                                        }
                                        dNextSupplyTimeCapacity = min(dNextSupplyTimeCapacity, dNextSupplyTimeCapacityForLane);
                                    }
                                    // Pas de else : on n'amÃ©liore pas le dNextSupplyTimeCapacity ici...
                                }
                                else
                                {
                                    // Pas de vÃ©hicule gÃ©nant sur cette voie
                                    dNextSupplyTimeCapacity = -DBL_MAX;
                                    break;
                                }
                            }


                        }

                        (*itConn)->SetNextSupplyTime(pDownStreamMicro, dNextSupplyTimeCapacity);
                        m_nodeToUpdate.push_back(*itConn);
                    }
                } // fi cas microscopique

            } // fiPostponed calculation
        }// for each outgoingLink
    } // rof each connection node
    // l 653


    // l 656
    CTuyauMeso * pDownStreamLink = NULL;
    CTuyauMeso * pUpstreamLink = NULL;
    // Ibis- UPDATE OF THE VEHICLE TIME CODE (Times when they cross nodes)


    pUpstreamLink = (CTuyauMeso * )m_pCurrentProcessedNode->GetNextEventIncommingLink();

    // obtient le vehicule courant
    std::pair<double, Vehicule*>  arrivingVehicle;

    arrivingVehicle = m_pCurrentProcessedNode->GetNextArrival(pUpstreamLink) ;

    if (arrivingVehicle.second != NULL && m_dbInstSimuMeso != DBL_MAX)
    {
        Tuyau * pDownStreamLinkBase = NULL;
        if (pUpstreamLink)
        {
            pDownStreamLinkBase = arrivingVehicle.second->CalculNextTuyau(pUpstreamLink, arrivingVehicle.first);
        }
        else
        {
            if (arrivingVehicle.second->GetItineraire()->size() > 0)
            {
                pDownStreamLinkBase = (CTuyauMeso *)arrivingVehicle.second->GetItineraire()->front();
            }
            else
            { // sinon on n'a pas d'itineraire -> on voyage uniquement sur le tuyau 0 aval de la connexion

                pDownStreamLinkBase = (CTuyauMeso *)((Connexion *)m_pCurrentProcessedNode)->m_LstTuyAv[0];
            }
        }
        pDownStreamLink = (CTuyauMeso *)pDownStreamLinkBase;


        double dNextArrivalTimeRecord = arrivingVehicle.first;

        double dW = fabs(arrivingVehicle.second->GetType()->GetW());
        double dU = arrivingVehicle.second->GetType()->GetVx();
        double dK = arrivingVehicle.second->GetType()->GetKx();

        // Constraint on capacity at the beginning of the link
        double dQx = dW*dU*dK / (dW + dU);

        // Vehicle is withdrawn from the Incoming link l680
        std::list< std::pair<double, Vehicule*> > listArrivalTime = m_pCurrentProcessedNode->GetArrivalTimes(pUpstreamLink);
        int sizetrt = (int)listArrivalTime.size();
        listArrivalTime.remove(arrivingVehicle);

        assert(listArrivalTime.size() == sizetrt-1);

        // To respect the FIFO rule (especially at diverge), all the arrival time
        // lower than the Temp_NextArrivalTime are updated to the
        // GlobalNextEvent.Time (that corresponds to the exit time of the last
        // vehicule)
        std::list< std::pair<double, Vehicule*> >::iterator itArrivalTime;
        std::pair<double, Vehicule*> nextArrival(DBL_MAX, nullptr);
        double dArrivalMin = DBL_MAX;
        for (itArrivalTime = listArrivalTime.begin(); itArrivalTime != listArrivalTime.end(); itArrivalTime++)
        {
            itArrivalTime->first = std::max<double>(itArrivalTime->first, m_dbInstSimuMeso + 1.0 / dQx);
            // Par cohÃ©rence avec le tri de arrivals (par id de vÃ©hicule si temps identique, pour les entrÃ©es, sinon problÃ¨me de vÃ©hicule qui ne s'insÃ¨rent pas dans l'ordre)
            if (dArrivalMin == DBL_MAX || (dArrivalMin > itArrivalTime->first || (dArrivalMin == itArrivalTime->first && nextArrival.second->GetID() > itArrivalTime->second->GetID())))
            {
                dArrivalMin = itArrivalTime->first;
                nextArrival = *itArrivalTime;
            }
        }

        m_pCurrentProcessedNode->ReplaceArrivalTimes(pUpstreamLink, listArrivalTime);

        if (listArrivalTime.empty() == false)
        {
            m_pCurrentProcessedNode->SetNextArrival(pUpstreamLink, nextArrival);//m_pCurrentProcessedNode->GetMinArrival(pUpstreamLink));
        }
        else
        { //No more vehicle left on the incoming link l 700
            m_pCurrentProcessedNode->SetNextArrival(pUpstreamLink, std::pair<double, Vehicule*>(DBL_MAX, nullptr));
        }

        // l 703
        Vehicule *pNextIncommingVehicule = arrivingVehicle.second;

        if (m_pCurrentProcessedNode->IsAnOrigine() == false && // si le noeud n'est pas une entree
            pUpstreamLink != NULL && pUpstreamLink->GetType() == Tuyau::TT_MESO && // ou le troncon n'est pas micro
            pNextIncommingVehicule->GetCurrentTuyauMeso() && pNextIncommingVehicule->GetCurrentTuyauMeso()->GetType() == Tuyau::TT_MESO)
        {
            CTuyauMeso * pCurrentTuyauVeh = (CTuyauMeso *)pNextIncommingVehicule->GetCurrentTuyauMeso();
            pNextIncommingVehicule->SetTimeCode(pCurrentTuyauVeh, Vehicule::TC_OUTGOING, m_dbInstSimuMeso);

            // Update of the vehicle list in links + current w and so and travel
            // time
            // List of. vehicles
            // 2- remove vehicle from the arrival list
            if (pCurrentTuyauVeh->GetType() == Tuyau::TT_MESO)
            {
                pCurrentTuyauVeh->RemoveCurrentVehicule(pNextIncommingVehicule);

                // 4- Update Wm and Km on Incomming link and outGoingLink
                pCurrentTuyauVeh->ComputeMeanW();
                pCurrentTuyauVeh->ComputeMeanK();
            }
        } // fi le noeud est une entrÃ©e l727

        m_pCurrentProcessedNode->AddUpstreamPassingTime(pUpstreamLink, m_dbInstSimuMeso);
        m_pCurrentProcessedNode->IncreaseRankNextIncommingVeh(pUpstreamLink);

        m_nodeToUpdate.push_back(m_pCurrentProcessedNode);

        Vehicule * pArrivingVehicle = arrivingVehicle.second;
        bool bInserted = true;
        // Particular Case the Node is an entry. Then Next Vehicle is directly
        // updated l 737
        if (m_pCurrentProcessedNode->IsAnOrigine())
        {
            bInserted = ((Entree*)m_pCurrentProcessedNode)->InsertionVehEnAttenteMeso(m_dbInstSimuMeso, pArrivingVehicle);

            if (!bInserted)
            {
                m_pCurrentProcessedNode->AddArrival(NULL, arrivingVehicle.first, arrivingVehicle.second, false);
            }
        }
        else
        {  // cas d'une insertion depuis un tuyau micro
            if (pUpstreamLink->GetType() == Tuyau::TT_MICRO)
            {

                pArrivingVehicle->SetTuyauMeso(pDownStreamLink, m_dbInstSimuMeso);
                pArrivingVehicle->SetTuyau((TuyauMicro *)pDownStreamLink, m_dbInstSimuMeso);
                pArrivingVehicle->SetTuyauAnt((TuyauMicro *)pUpstreamLink);
                pArrivingVehicle->SetVoieAnt((VoieMicro *)pUpstreamLink->GetVoie(0));
                pArrivingVehicle->SetVoie((VoieMicro *)pDownStreamLink->GetVoie(0));
            }
        }

        // II - Link model

        // OTK - remarque : c'est pas un problÃ¨me de ne tester l'insertion qu'ici ? alors qu'on a dÃ©jÃ  fait les opÃ©rations prÃ©cÃ©dentes du genre AddUpstreamPassingTime
        // et IncreaseRankNextIncommingVeh ???
        if (bInserted)
        {

            // 3- Vehicle is inserted in the outgoing link
            pArrivingVehicle->SetTuyauMeso(pDownStreamLink, m_dbInstSimuMeso);
            pArrivingVehicle->SetTimeCode(pDownStreamLink, Vehicule::TC_INCOMMING, m_dbInstSimuMeso);

            if (pUpstreamLink)
            {
                pArrivingVehicle->AddTuyauParcouru(pUpstreamLink, m_dbInstSimuMeso);
            }


            if (pDownStreamLink) // if the node is not an exit
            {
                if (pDownStreamLink->GetType() == Tuyau::TT_MESO)
                {
                    // Udpdate of the vehicle list in links
                    pDownStreamLink->AddCurrentVehicule(pArrivingVehicle);
                    // 4- Update Wm and Km on Incomming link and outGoingLink
                    pDownStreamLink->ComputeMeanW();
                    pDownStreamLink->ComputeMeanK();
                }
                else // sinon le troncon de sortie est un micro ou macro
                {
                    pArrivingVehicle->SetPos(0.0);

                    // Ajoute le vÃ©hicule sur un tuyau de type micro
                    TuyauMicro * pDownStreamMicro = (TuyauMicro *)pDownStreamLinkBase;

                    double dbTmp;
                    int nVoieMicro = ComputeAvailableLaneNumber(pArrivingVehicle, pDownStreamMicro, pUpstreamLink, pArrivingVehicle->GetPos(0), NULL, dbTmp);

                    double dNextSupplyTimeCapacity = -DBL_MAX;
                    boost::shared_ptr<Vehicule> pLastWayVehicule = GetLastVehicule((VoieMicro*)pDownStreamMicro->GetVoie(nVoieMicro), pArrivingVehicle);
                    if (pLastWayVehicule.get()
                        && pLastWayVehicule.get()->GetPos(0) == 0.0) // le troncon suivant est congestionnÃ©
                    {
                        pArrivingVehicle->SetVit(min(pArrivingVehicle->GetVitMax(), pLastWayVehicule.get()->GetVit(0)));
                        pArrivingVehicle->SetVitAnt(min(pArrivingVehicle->GetVitMax(), pDownStreamLinkBase->GetVitReg()));
                    }
                    else
                    {
                        pArrivingVehicle->SetVit(min(pArrivingVehicle->GetVitMax(), pDownStreamLinkBase->GetVitReg()));
                    }
                    pArrivingVehicle->SetTuyau(pDownStreamMicro, m_dbInstSimuMeso);

                    pArrivingVehicle->SetVoie((VoieMicro*)pDownStreamLinkBase->GetVoie(nVoieMicro));

                    pArrivingVehicle->SetVoieAnt((VoieMicro*)pUpstreamLink->GetVoie(nVoieMicro));
                    pArrivingVehicle->SetTuyauAnt((TuyauMicro*)pUpstreamLink);

                    pArrivingVehicle->SetInstantCreation(m_dbInstSimuMeso - m_dbInstSimu);

                    pArrivingVehicle->FinCalculTrafic(m_dbInstSimuMeso);

                    pArrivingVehicle->SetTuyau(pDownStreamMicro, m_dbInstSimuMeso);

                    pArrivingVehicle->SetTuyauAnt((TuyauMicro *)pDownStreamLinkBase);
                    pArrivingVehicle->SetVoieAnt((VoieMicro*)pDownStreamLinkBase->GetVoie(nVoieMicro));
                    pArrivingVehicle->SetTuyauMeso(NULL, m_dbInstSimuMeso);
                }

            }
            // on passe le tuyau en fin de pas de temps Ã  NULL dans ce cas pour
            // provoquer la suppression du vÃ©hicule (sinon plantage au prochaine CalcultraficEx)
            else
            {
                pArrivingVehicle->SetTuyau(NULL, m_dbInstSimuMeso);
            }


            m_pCurrentProcessedNode->AddDownstreamPassingTime(pDownStreamLink, m_dbInstSimuMeso);

            //   m_pCurrentProcessedNode->AddOutGoingVehicule(pDownStreamLink, pArrivingVehicle); // ligne 807
            m_pCurrentProcessedNode->IncreaseRankNextOutgoingVeh(pDownStreamLink);
        } // fi isInserted

        //6- UPDATES OF SUPPLY TIMES AT THE DOWNSTREAM BOUNDARIES OF NODES
        // Calcule temps de traversÃ© du noeud courant
        // Si le noeud n'est pas une sortie
        if (!pDownStreamLink || pDownStreamLink->GetType() == Tuyau::TT_MESO)
        {
            if (pDownStreamLink)
            {
                // Constraint on capacity at the beginning of the link
                double dNextSupplyTimeCapacity = m_dbInstSimuMeso + 1.0 / (pDownStreamLink->getNb_voies() * dQx);

                // Constraint coming from downstream boundary (end of the outgoing link)
                double dNextSupplyTimeDownstream = -DBL_MAX;

                int iVehicleRAnk = m_pCurrentProcessedNode->GetRankNextOutgoingVehicule(pDownStreamLink);
                double dlLinkMeanK = pDownStreamLink->GetMeanK();

                double iVehicleRankDownStream = iVehicleRAnk - pDownStreamLink->getNb_voies() *pDownStreamLink->GetLength()*dlLinkMeanK;

                if (iVehicleRankDownStream > 1)
                {

                    int iRefVehRank = (int)floor(iVehicleRankDownStream);
                    double dRefRankResidue = iVehicleRankDownStream - iRefVehRank;

                    Connexion *pConnDownStream = pDownStreamLink->GetCnxAssAv();

                    std::deque<double> incommingPassingTime = pConnDownStream->GetUpstreamPassingTimes(pDownStreamLink);
                    //pUpstreamLink);
                    if (incommingPassingTime.size() > 0)
                    {
                        double dDownStreamRefTime = DBL_MAX;
                        if (iRefVehRank < (int)incommingPassingTime.size())
                        {
                            dDownStreamRefTime = incommingPassingTime.at(iRefVehRank - 1) + dRefRankResidue *(incommingPassingTime.at(iRefVehRank) - incommingPassingTime.at(iRefVehRank - 1)); //l 840
                            if (pDownStreamLink->GetMeanW() != 0.0)
                            {
                                dNextSupplyTimeDownstream = dDownStreamRefTime + pDownStreamLink->GetLength() / fabs(pDownStreamLink->GetMeanW());
                            }
                            else
                            {
                                dNextSupplyTimeDownstream = DBL_MAX;
                            }

                        }
                        else
                        {
                            dNextSupplyTimeDownstream = DBL_MAX;
                            //else iRefVehRank +1 > incommingPassingTime.size()
                            // We miss one vehicle to properly estimate the passing time of
                            // the reference vehicle at the downstream boundary. This
                            //calculation is then postpone to the next global event time
                        } // fi iRefVehRank +1 > incommingPassingTime.size()
                    } // fi incommingPassingTime.size()>0
                }// fi iVehicleRankDownStream>0

                m_pCurrentProcessedNode->SetNextSupplyTime(pDownStreamLink, max(dNextSupplyTimeCapacity, dNextSupplyTimeDownstream));
            }
            else
            { // else le noeud est une sortie, only the constraint on the local capacity is considered

                Sortie *pExit = dynamic_cast<Sortie*>(m_pCurrentProcessedNode);

                if (pExit)
                {
                    // Pour traitement du vÃ©hicule par la sortie (et donc renseignement du temps de sortie du rÃ©seau pour les vÃ©hicules meso...)
                    pExit->VehiculeEnter(pNextIncommingVehicule->shared_from_this(),
                        (VoieMicro*)pUpstreamLink->GetLstLanes()[0], // rmq. : approximation ici, la gestion des restrictions de capacitÃ©s sont rÃ©parties par voie et on ne gÃ¨re pas de numÃ©ro de voie en mode mÃ©so
                        m_dbInstSimuMeso,
                        m_dbInstSimuMeso,
                        GetTimeStep());

                    double dNextSupplyTimeCapacity = 0;


                    tracked_double* pCapavituValue = pExit->GetLstCapacites()->GetVariationEx(m_dbInstSimuMeso);
                    if (pCapavituValue == NULL)
                    {
                        assert(false);
                    }
                    else
                    {
                        // should not occurt
                        if (pCapavituValue->t > 0)
                        {
                            dNextSupplyTimeCapacity = m_dbInstSimuMeso + 1.0 / pCapavituValue->t;
                        }
                        else
                        {
                            // next supply time is after end simulation
                            dNextSupplyTimeCapacity = m_dbDureeSimu + 1;
                        }
                    }
                    m_pCurrentProcessedNode->SetNextSupplyTime(pDownStreamLink, dNextSupplyTimeCapacity);
                }

            } // fi else node is an exit
        } //pDownStreamLink->GetType() == Tuyau::TT_MESO
        else
        {
            // GÃ©re le supply time depuis un troncon micro
            if (m_pCurrentProcessedNode->IsADestination() == false)
            {
                TuyauMicro * pDownStreamMicro = (TuyauMicro *)pDownStreamLinkBase;

                double dNextSupplyTimeCapacity = -DBL_MAX;
                boost::shared_ptr<Vehicule> pLastWayVehicule;
                double dbMaxPos = -DBL_MAX;
                int iVoie = 0;
                for (int iLane = 0; iLane < pDownStreamMicro->getNb_voies(); iLane++)
                {
                    // On ne regarde que les voies accessibles depuis le tronÃ§on amont s'il est prÃ©cisÃ©
                    if (!pUpstreamLink || dynamic_cast<Giratoire*>(pDownStreamMicro->GetCnxAssAm()) || pDownStreamMicro->GetCnxAssAm()->IsMouvementAutorise(pUpstreamLink, pDownStreamLink->GetLstLanes()[iLane], arrivingVehicle.second->GetType(), NULL))
                    {
                        boost::shared_ptr<Vehicule> pLastWayVehiculeCandidate = GetLastVehicule((VoieMicro*)pDownStreamMicro->GetVoie(iLane), pArrivingVehicle);
                        if (!pLastWayVehiculeCandidate)
                        {
                            // On a une voie accessible sans vÃ©hicule dessus : c'est bon pour un supply time immÃ©diat, on ne va pas plus loin.
                            pLastWayVehicule.reset();
                            break;
                        }
                        else if (pLastWayVehiculeCandidate->GetPos(0) > dbMaxPos)
                        {
                            dbMaxPos = pLastWayVehiculeCandidate->GetPos(0);
                            pLastWayVehicule = pLastWayVehiculeCandidate;
                            iVoie = iLane;
                        }
                    }
                }

                if (pLastWayVehicule.get())
                {
                    double dPosLeader = DBL_MAX;
                    dPosLeader = pLastWayVehicule->GetPos(0);

                    //    double dLastEnter = pDownStreamMicro->m_mapVeh.find(pArrivingVehicle->GetID())->second.second.first;
                    double dQ = 0;
                    //     if( dPosLeader-pArrivingVehicle->GetPos(0)>0.001)
                    {
                        dQ = fabs(pArrivingVehicle->GetType()->GetW())*(pArrivingVehicle->GetType()->GetKx() - 1.0 /
                            max(1.0 / pArrivingVehicle->GetDiagFonda()->GetKCritique(), fabs(dPosLeader - pArrivingVehicle->GetPos(0))));
                    }

                    if (dPosLeader - pArrivingVehicle->GetPos(0) < 1.0 / pArrivingVehicle->GetType()->GetKx() || 1.0 / dQ < pArrivingVehicle->GetType()->GetKx()
                                     || max(1.0/pArrivingVehicle->GetDiagFonda()->GetKCritique(), fabs(dPosLeader-pArrivingVehicle->GetPos(0))  ) <=  1.0/pArrivingVehicle->GetDiagFonda()->GetKCritique()) // regime congestionnÃ©
                    {
                        dNextSupplyTimeCapacity = DBL_MAX;
                    }
                    else
                    {

                        dNextSupplyTimeCapacity = max(dNextSupplyTimeCapacity, m_dbInstSimuMeso + 1.0 / dQ);
                    }
                }

                m_pCurrentProcessedNode->SetNextSupplyTime(pDownStreamLinkBase,
                    max(dNextSupplyTimeCapacity, m_pCurrentProcessedNode->GetNextSupplyTime(pDownStreamLinkBase)));

            }
        }

        // Prise en compte du vÃ©hicule s'il quitte un tronÃ§on meso par les capteurs sur ce tronÃ§on
        if (m_pGestionsCapteur && pUpstreamLink && pUpstreamLink->GetType() == Tuyau::TT_MESO && pNextIncommingVehicule->GetTimeCode(pUpstreamLink, Vehicule::TC_INCOMMING) != DBL_MAX)
        {
            m_pGestionsCapteur->AddMesoVehicle(m_dbInstSimuMeso, pNextIncommingVehicule, pUpstreamLink, pDownStreamLink);
        }

        // UPDATES OF ARRIVAL TIMES AT UPSTREAM BOUNDARIES OF NODES
        // compute temps de traversÃ© du troncon courant( aval du noeud courant) l 873
        // Si le noeud n'est pas une sortie
        if (pDownStreamLink && bInserted)
        {
            Connexion *pConnDownStream = pDownStreamLink->GetBriqueAval();

            if (!pConnDownStream)
            {
                pConnDownStream = pDownStreamLink->GetCnxAssAv();
            }

            if (pDownStreamLink->GetType() == Tuyau::TT_MESO)
            {
                double dVehiculeArrivalTime = m_dbInstSimuMeso + pDownStreamLink->GetLength() / min(pDownStreamLink->GetVitReg(), pArrivingVehicle->GetType()->GetVx());;

                // prise en compte la longueur dans la brique amont en plus du tuyau aval (sinon on ignore complÃ¨tement la taille des briques) :
                // rmq. : on nÃ©glige l'impact de l'accÃ©lÃ©ration bornÃ©e ici.
                BriqueDeConnexion * pBriqueUpStream = pDownStreamLink->GetBriqueAmont();
                if (pBriqueUpStream)
                {
                    int iItiIndex = pArrivingVehicle->GetItineraireIndex(isSameTuyauPredicate, pDownStreamLink);
                    if (iItiIndex != -1 && iItiIndex-1 >= 0)
                    {
                        Tuyau * pPrevTuy = pArrivingVehicle->GetItineraire()->at(iItiIndex-1);
                        std::vector<Tuyau*> lstTuyauxInternes;
                        pBriqueUpStream->GetTuyauxInternes(pPrevTuy, pDownStreamLink, lstTuyauxInternes);
                        for (size_t iTuyInt = 0; iTuyInt < lstTuyauxInternes.size(); iTuyInt++)
                        {
                            Tuyau * pTuyInt = lstTuyauxInternes[iTuyInt];
                            dVehiculeArrivalTime += pTuyInt->GetLength() / (min(pTuyInt->GetVitReg(), pArrivingVehicle->GetType()->GetVx()));
                        }
                    }
                }

                std::list<std::pair<double, Vehicule *> > arrivalTimes = pConnDownStream->GetArrivalTimes(pDownStreamLink);

                // gestion des arrÃªts :
                // - ajout des temps d'arrÃªts Ã©ventuels
                // - passage aux triplegs suivant pour chaque arrÃªt !
                // remarque : en mode mixte, si on repasse en micro, les bus ne vont pas s'arrÃªter aux arrÃªts (mais
                // comme la durÃ©e de traversÃ©e du tronÃ§on prend en compte la durÃ©e des arrÃªts, le vÃ©hicule devrait rÃ©apparaÃ®tre
                // avant sa position rÃ©elle (du coup c'est un peu faux car il fera la fin du tronÃ§on Ã  vitesse max et pas la vitesse moyenne en comptant les arrÃªts)
                // remarque : le temps d'arrÃªt ne prend en compte que le temps d'arrÃªt, sans compter une Ã©ventuelle dÃ©cÃ©lÃ©ration puis rÃ©accÃ©lÃ©ration du vÃ©hicule...
                while (pArrivingVehicle->GetTrip()->GetCurrentDestination()->GetInputPosition().GetLink() == pDownStreamLink)
                {
                    TripNode * pStop = pArrivingVehicle->GetTrip()->GetCurrentDestination();
                    double dbStopTime = pArrivingVehicle->GetFleet()->GetStopDuration(pArrivingVehicle->shared_from_this(), pStop, true);
                    dVehiculeArrivalTime += dbStopTime;

                    // IF pour Ã©viter une assertion (normalement en mode micro pour lequel a Ã©tÃ© conÃ§u le mÃ©canisme des Trips,
                    // on ne peut pas faire de MoveToNextLeg alors qu'on est dÃ©jÃ  sur le dernier leg...
                    if (pArrivingVehicle->GetTrip()->GetCurrentLegIndex() < (int)pArrivingVehicle->GetTrip()->GetLegs().size() - 1)
                    {
                        pArrivingVehicle->MoveToNextLeg();
                    }
                    else
                    {
                        break;
                    }
                }

                // Impacts of heteorgeneous vehicles, i.e. if the link has more than one
                // lane vehicle are allowed to overtake (in free-flow) without
                // Penalty. If the link only has one lane, the new arrival time can not be
                // lower than maximum existing one.
                pConnDownStream->AddArrival(pDownStreamLink, dVehiculeArrivalTime, pArrivingVehicle, pDownStreamLink->getNb_voies() == 1 && arrivalTimes.empty() == false);
                //pConnDownStream->AddArrivingVehicule( pDownStreamLink,pArrivingVehicle );
                pConnDownStream->SetNextArrival(pDownStreamLink, pConnDownStream->GetMinArrival(pDownStreamLink));
                // l903 should be replaced   pConnDownStream->SetNextArrivalTime(pDownStreamLink,pDownStreamLink->GetMinArrivalTime() );
                m_nodeToUpdate.push_back(pConnDownStream);
            }

        }
    }

    // III- NODE MODEL
    // EVENT TIMES ARE UPDATED FOR ALL NODES ThAT HAVE BEEN MARKED
    std::map<Tuyau*, double> mapUpstreamSupplyTime;
    for(itNode= m_nodeToUpdate.begin(); itNode != m_nodeToUpdate.end(); itNode++ )
    {
        Connexion * pCon = (Connexion *)(*itNode);
        ControleurDeFeux * pCtrlFeux = pCon->GetCtrlDeFeux();

        std::map<Tuyau*, std::map< std::pair<double, Tuyau*>, Vehicule*, LessPairPtrIDRef< std::pair<double, Tuyau*> > >, LessPtr<Tuyau> > mapNextLocalEvents;

        size_t i;
        for( i = 0; i< std::max<size_t>( ((pCon)->m_LstTuyAv.size()),1); ++i) //The use of max directly encompasses Exit Nodes
        {
            Tuyau * pOutgoingLink = NULL;
            if(i< (pCon)->m_LstTuyAv.size() )
            {
                pOutgoingLink = pCon->m_LstTuyAv.at(i);
            }

            double dNextSupplyTime= pCon->GetNextSupplyTime(pOutgoingLink);

            // The next supply time is refined by incoming links to account for
            // red traffic signals
            size_t k;
            mapUpstreamSupplyTime.clear();
            for( k = 0; k < std::max<size_t>(1, pCon->m_LstTuyAm.size() ); ++k)
            {
                Tuyau *pIncommingLink = NULL;

                if( k< pCon->m_LstTuyAm.size())
                {
                   pIncommingLink = pCon->m_LstTuyAm.at(k);
                }


                mapUpstreamSupplyTime[pIncommingLink] = dNextSupplyTime;

                // pour ne pas boucler inutilement (GetInstantVertSuivant est trÃ¨s couteux),
                // on regarde d'abord si le mouvement est autorisÃ©
                if(pIncommingLink && pOutgoingLink && !pCon->IsMouvementAutorise(pIncommingLink, pOutgoingLink, NULL, NULL))
                {
                    mapUpstreamSupplyTime[pIncommingLink] = std::max<double>(dNextSupplyTime, m_dbDureeSimu);
                }
                else
                {
                    // si il y a un signal
                    if( pCtrlFeux != NULL &&
                        dNextSupplyTime < m_dbDureeSimu)
                    {
                        double dRefTime = dNextSupplyTime;
                        if (dRefTime == -DBL_MAX)
                        {
                            dRefTime = m_dbInstSimu;
                        }
                        eCouleurFeux eLightColor = pCtrlFeux->GetCouleurFeux(dRefTime, pIncommingLink, pOutgoingLink);
                        // si le signal sera rouge Ã  l'instant de l'Ã©vÃ¨nement
                        if (eLightColor == FEU_ROUGE)
                        {
                            double dTimeNexGreen = pCtrlFeux->GetInstantVertSuivant(dRefTime, m_dbDureeSimu, pIncommingLink, pOutgoingLink);
                            mapUpstreamSupplyTime[pIncommingLink] = max(dNextSupplyTime, dTimeNexGreen);

                        } // fi le signal est rouge
                        else
                        {
                            // le feu est vert Ã  l'instant d'offre : on applique la formule LWR des instants de passage au feu suite Ã  un passage rouge -> vert
                            // pour respecter l'accÃ©lÃ©ration bornÃ©e des vÃ©hicules :
                            double dbStartGreenCycleTime = pCtrlFeux->GetInstantDebutVert(dRefTime, pIncommingLink, pOutgoingLink);

                            // numÃ©ro du vÃ©hicule depuis le feu vert
                            int nbVehiclesSinceGreen = 0;
                            std::deque<double> listPreviousPassingTime = pCon->GetUpstreamPassingTimes(pIncommingLink);
                            for (std::deque<double>::reverse_iterator ritPassingTimes = listPreviousPassingTime.rbegin(); ritPassingTimes != listPreviousPassingTime.rend(); ++ritPassingTimes)
                            {
                                if (*ritPassingTimes >= dbStartGreenCycleTime)
                                {
                                    nbVehiclesSinceGreen++;
                                }
                                else if (*ritPassingTimes < dbStartGreenCycleTime)
                                {
                                    break;
                                }
                            }

                            double dbAcceleration = m_LstTypesVehicule.front()->GetAccMax(0);
                            double dbKMax = m_LstTypesVehicule.front()->GetKx() * pIncommingLink->getNb_voies();
                            double dbW = m_LstTypesVehicule.front()->GetW();
                            double dbNextPassingTime = -2.0 * dbAcceleration * dbAcceleration * ((double)nbVehiclesSinceGreen / (dbW * dbKMax) - dbStartGreenCycleTime);
                            dbNextPassingTime += 2.0 * dbAcceleration * sqrt(2.0 * dbAcceleration * nbVehiclesSinceGreen / dbKMax);
                            dbNextPassingTime /= 2.0 * dbAcceleration * dbAcceleration;

                            mapUpstreamSupplyTime[pIncommingLink] = max(dNextSupplyTime, dbNextPassingTime);
                        }
                    } // fi il y a un signal
                }
            }// rof each incommink link of the node

            // Search for the candidate vehicle (that wants to take this outgoinglink)
            std::vector<Vehicule *> vehCandidates;
            std::vector<Tuyau *> incommingLinksCandidates;
            std::vector<double> nextArrivalTimeCandidate;

            //The use of max directly encompasses entry Nodes
            for( k = 0; k < std::max<size_t>(1, pCon->m_LstTuyAm.size() ); ++k)
            {
                Tuyau *pIncommingLink = NULL;
                if(pCon->m_LstTuyAm.size()>0)
                {
                    pIncommingLink = pCon->m_LstTuyAm.at(k);
                }
                // on permet Ã  un vÃ©hicule qui est arrivÃ© aprÃ¨s d'Ãªtre candidat dans le cas oÃ¹ le vÃ©hicule qui le prÃ©cÃ¨de
                // ne le gÃ¨ne pas
                std::set<Tuyau*> tryedOutputLinks;
                std::list< std::pair<double, Vehicule* > > arrivalsForLink = pCon->GetArrivalTimes(pIncommingLink);
                int nbTries = 0;
                int nbTriesMax = pIncommingLink ? (1 + m_nMesoNbVehChgtVoie * (pIncommingLink->getNb_voies()-1)) : 1;
                for (std::list< std::pair<double, Vehicule* > >::const_iterator iterArrival = arrivalsForLink.begin(); iterArrival != arrivalsForLink.end() && nbTries < nbTriesMax; ++iterArrival)
                {
                    std::pair<double, Vehicule* > nextArrivalCand = *iterArrival;
                    Tuyau *pTuyauNextLink = NULL;
                    if (pCon->IsAnOrigine()) // si c'est une origine le tuyau courant est le tuyau suivant (sinon s'il est Ã  null il sera detruit)
                    {
                        if (nextArrivalCand.second->GetCurrentTuyauMeso())
                        {
                            pTuyauNextLink = nextArrivalCand.second->GetCurrentTuyauMeso();
                        }
                        else
                        {
                            if (nextArrivalCand.second->GetItineraire()->size() > 0)
                            {
                                pTuyauNextLink = nextArrivalCand.second->GetItineraire()->front();
                            }
                        }
                    }
                    else
                    {
                        pTuyauNextLink = nextArrivalCand.second->CalculNextTuyau(pIncommingLink/*nextArrivalCand.second->GetCurrentTuyauMeso()*/, nextArrivalCand.first);
                        nextArrivalCand.second->SetNextTuyau(pTuyauNextLink);
                    }

                    // si le prochain lien est le lien de sortie actuellement traitÃ© (ou si le vÃ©hicule est sur son dernier tuyau et sort)
                    if ((pTuyauNextLink == pOutgoingLink || pTuyauNextLink == NULL)
                        && (tryedOutputLinks.find(pOutgoingLink) == tryedOutputLinks.end())) // Ajout de la condition que le tuyau aval doit Ãªtre diffÃ©rent des tuyaux aval dÃ©jÃ  testÃ©s, sinon on doublerait le vÃ©hicule prÃ©cÃ©dent

                    {
                        // Soit il s'agit du premier vÃ©hicule de la pile, soit le vÃ©hicule est sur une voie lui permettant de doubler le premier vÃ©hicule de la pile :
                        if (nbTries == 0 || pCon->HasReservedLaneForMove(pIncommingLink, pOutgoingLink))
                        {
                            vehCandidates.push_back(nextArrivalCand.second);
                            incommingLinksCandidates.push_back(pIncommingLink);
                            nextArrivalTimeCandidate.push_back(nextArrivalCand.first);

                            // On a trouvÃ© un candidat : on arrÃªte de remonter la liste des arrivÃ©es
                            break;
                        }
                    }
                    nbTries++;
                    tryedOutputLinks.insert(pTuyauNextLink);
                }

            } // rof looking for the next arriving vehicules

            // Prise en compte des vÃ©hicules qui arrivent de l'interieur de la brique le cas Ã©chÃ©ant (cas hybridation dynamique)
            std::map<Tuyau*, std::pair<double, Vehicule* > >::const_iterator iterInternArrival;
            for(iterInternArrival = pCon->getNextArrivals().begin(); iterInternArrival != pCon->getNextArrivals().end(); ++iterInternArrival)
            {
                if (iterInternArrival->first && iterInternArrival->first->GetBriqueParente())
                {
                    assert(iterInternArrival->first->GetBriqueParente() == pCon);

                    if (iterInternArrival->second.second
                        && iterInternArrival->second.first != DBL_MAX)
                    {
                        Tuyau *pTuyauNextLink = iterInternArrival->second.second->CalculNextTuyau(iterInternArrival->first, iterInternArrival->second.first);

                        if (pTuyauNextLink)
                        {
                            if (pTuyauNextLink == pOutgoingLink)
                            {
                                vehCandidates.push_back(iterInternArrival->second.second);
                                incommingLinksCandidates.push_back(iterInternArrival->first);
                                nextArrivalTimeCandidate.push_back(iterInternArrival->second.first);

                                mapUpstreamSupplyTime[iterInternArrival->first] = pCon->GetNextSupplyTime(pOutgoingLink);
                            }
                        }
                    }
                }
            }

             // Only links that have a candidate are considered

            std::vector<double> nextSupplyTimeIncommingLink;
            std::vector<double> nextSupplyTimeILUnique;
            int iIncomming;
            for(iIncomming = 0; iIncomming < (int)incommingLinksCandidates.size(); ++iIncomming)
            {
                nextSupplyTimeILUnique.push_back(mapUpstreamSupplyTime[incommingLinksCandidates.at(iIncomming)]);
            }
            nextSupplyTimeIncommingLink = nextSupplyTimeILUnique;
            std::vector<double>::iterator itDbl;


            itDbl =std::unique(nextSupplyTimeILUnique.begin(), nextSupplyTimeILUnique.end());
            nextSupplyTimeILUnique.resize(std::distance(nextSupplyTimeILUnique.begin(), itDbl));
            std::sort(nextSupplyTimeILUnique.begin(), nextSupplyTimeILUnique.end());

            int nDiffSupplyTime = (int)nextSupplyTimeILUnique.size();

            // update the next events
            // Gestion des affectations
            if( vehCandidates.size() >0)
            {
                // Principle of the algorithm :
                // For the considered exit, several next supply time are possible
                // due to traffic signals on the incoming links. Possible Next
                // Supply Time are explored from the lowest to the highest. At each
                // step, we select the incoming links with a Next Supply Time lower
                // or equal to the currently investigated supply time. Then, we look
                // if there is at least an incoming links that is congeted. In that
                // case, the loop end : the next supply time is the current next
                // supply time and the choice of the incoming link is realized by
                // applying the algorithm related to congested situations. If all
                // incoming links are in free-flow, we determine if at least vehicle
                // arrive before the following next supply time, then next local
                // event is defined by this arrival. Otherwise, we explore the
                // following next supply time (loop in the algorithm). If we have reached
                // the highest next supply time, we apply the free-flow repartition to all
                // the incoming link candidates.
                Tuyau *pSelectedIncomingLink = NULL;
                int iCaseId= 1;
                bool bStopLoop = false;
                double nextEvent= DBL_MAX ;
                Vehicule * pSelectedVehicle = NULL;
                while( !bStopLoop &&
                    iCaseId <= nDiffSupplyTime)
                {
                  double dNextSupplyTimeMin=  nextSupplyTimeILUnique.at(iCaseId-1);
                  int iTimeOccCount = (int)std::count(nextArrivalTimeCandidate.begin(), nextArrivalTimeCandidate.end(), dNextSupplyTimeMin);
                  std::vector<Vehicule* > VehCandidates2;
                  std::vector<Tuyau *> incommingLinksCandidates2;
                  std::vector<double > nextArrivalTimeCandidate2;
                  std::vector<double> affectationCoeff;

                  std::vector<double>::iterator itNextArrivalFind = nextArrivalTimeCandidate.begin();

                  double dSumAffectationCoeff = 0;
                  std::vector<int>  IdsLessEqualId = FindLessEqual<double>(nextSupplyTimeIncommingLink, dNextSupplyTimeMin);
                  size_t iIdsLessEqual;
                  for( iIdsLessEqual = 0; iIdsLessEqual< IdsLessEqualId.size() ; ++iIdsLessEqual )
                  {
                      int iNextArrivalFind = IdsLessEqualId.at(iIdsLessEqual);
                      VehCandidates2.push_back( vehCandidates.at( iNextArrivalFind ) ) ;
                      incommingLinksCandidates2.push_back( incommingLinksCandidates.at(iNextArrivalFind ));
                      nextArrivalTimeCandidate2.push_back( nextArrivalTimeCandidate.at(iNextArrivalFind ));
                    // Selection of the O / D coefficients
                     // get affectation coeff


                    double affectationOD = 1;//mapCoeffs[pOutgoingLink];
                    dSumAffectationCoeff +=affectationOD;
                    affectationCoeff.push_back( affectationOD);
                  }

                  // normalize coeffs
                  size_t iLoop;
                  for( iLoop=0; iLoop< affectationCoeff.size() ; ++iLoop )
                  {
                      double dTmpCoeff = affectationCoeff.at(iLoop)/dSumAffectationCoeff;
                      affectationCoeff[iLoop] = dTmpCoeff;
                  }

                  // Search in the subselection of links for Candidates that arrives before they can leave
                  std::vector<int>  id3 = FindLessEqual<double>( nextArrivalTimeCandidate2, dNextSupplyTimeMin);

                  // At least on incoming road is congested l 1053
                  if( id3.empty() == false )
                  {
                      nextEvent = dNextSupplyTimeMin;
                      std::vector<Tuyau *> incomingLinksCandidateCongested;
                      std::vector<double> alphaODCandidateCongested;
                      std::vector<Vehicule*> vehicleCongested;
                      double dSumAlphaOdCandidate =0;
                      for( iLoop = 0; iLoop <id3.size() ; ++iLoop)
                      {
                          dSumAlphaOdCandidate += affectationCoeff.at(id3.at(iLoop ) );
                      }
                      for( iLoop =0; iLoop< id3.size(); ++iLoop)
                      {
                          incomingLinksCandidateCongested.push_back(incommingLinksCandidates2.at(id3.at(iLoop ) ));
                          alphaODCandidateCongested.push_back(affectationCoeff.at(id3.at(iLoop ) ) /dSumAlphaOdCandidate );
                          vehicleCongested.push_back(VehCandidates2.at(id3.at(iLoop)));
                      }
                      //TODO RAND l 1085:
                  //    Temp_SelectedIncomingLink=Temp_ListIncomingLinksCandidateCongested(min(find(cumsum(Temp_AlphaOD_CandidateCongestedNormalized) >= rand)));
                      int iSelectedCandidate = int(
                          floor((fabs(m_pRandManager->myRand() - 1.0) / double(MAXIMUM_RANDOM_NUMBER))* (incomingLinksCandidateCongested.size())));// std::distance(nextArrivalTimeCandidate.begin(), itDbl) );
                      pSelectedIncomingLink = incomingLinksCandidateCongested.at(iSelectedCandidate);
                      pSelectedVehicle = vehicleCongested.at(iSelectedCandidate);
                      bStopLoop = true;
                  }
                  else // no road is congested
                  {
                       if( nextArrivalTimeCandidate2.size()>0)
                       {
                             double timeMinArrival =*(std::min_element(nextArrivalTimeCandidate2.begin(), nextArrivalTimeCandidate2.end()));

                            // all the possible next supply time have been investigated and the exit is still in free-flow
                            if(iCaseId == nDiffSupplyTime ||
                                timeMinArrival < nextSupplyTimeILUnique[iCaseId] )
                            { // All road are in free flow
                                nextEvent = timeMinArrival;

                                itDbl = find( nextArrivalTimeCandidate2.begin(), nextArrivalTimeCandidate2.end(), nextEvent);

                                size_t iSelectedCandidate = std::distance(nextArrivalTimeCandidate2.begin(), itDbl);
                                pSelectedIncomingLink = incommingLinksCandidates2.at(iSelectedCandidate);
                                pSelectedVehicle = VehCandidates2.at(iSelectedCandidate);

                                //TODO RAND l 1073
                                bStopLoop = true;
                            }
                            else
                            {
                                    iCaseId++;
                            }
                       }
                       else
                       {
                           iCaseId++;
                       }

                  }

                }// end loop !bStopLoop && caseId <= nDiffSupplyTime

                mapNextLocalEvents[pOutgoingLink][std::make_pair(nextEvent, pSelectedIncomingLink)] = pSelectedVehicle;
            }

        }// rof each outgoing link of the node

        // l 1114

        std::pair<std::pair<double, Tuyau *>, Vehicule*> minLocalEventTime(std::make_pair(DBL_MAX, nullptr), nullptr);
        for (std::map<Tuyau*, std::map< std::pair<double, Tuyau*>, Vehicule*, LessPairPtrIDRef< std::pair<double, Tuyau*> > >, LessPtr<Tuyau> >::const_iterator iterOutgoingLink
            = mapNextLocalEvents.begin(); iterOutgoingLink != mapNextLocalEvents.end(); ++iterOutgoingLink)
        {
            if (!iterOutgoingLink->second.empty() && iterOutgoingLink->second.begin()->first.first < minLocalEventTime.first.first)
            {
                minLocalEventTime = std::make_pair(iterOutgoingLink->second.begin()->first, iterOutgoingLink->second.begin()->second);
            }
        }
        // On ne permet pas de revenir en arriÃ¨re :
        if (minLocalEventTime.first.first < m_dbInstSimuMeso)
        {
            minLocalEventTime.first.first = m_dbInstSimuMeso;
        }
        ChangeMesoNodeEventTime(pCon, minLocalEventTime.first.first, minLocalEventTime.first.second);
        // On met aussi Ã  jour l'instant dans la liste des arrivals
        std::list< std::pair<double, Vehicule*> > listArrivals = pCon->GetArrivalTimes(minLocalEventTime.first.second);
        for (std::list< std::pair<double, Vehicule* > >::iterator iterArrival = listArrivals.begin(); iterArrival != listArrivals.end(); ++iterArrival)
        {
            if (iterArrival->second == minLocalEventTime.second)
            {
                iterArrival->first = minLocalEventTime.first.first;
                break;
            }
        }
        pCon->ReplaceArrivalTimes(minLocalEventTime.first.second, listArrivals);

        // On met Ã  jour le prochain arrival qui a pu changer car on ne prend pas nÃ©cessairement le premier vÃ©hicule dans la file (pour gestion
        // intelligente des mouvements non interbloquants aux feux)
        pCon->SetNextArrival(minLocalEventTime.first.second, std::make_pair(minLocalEventTime.first.first, minLocalEventTime.second));
    } // rof each node to update

    // DATA RECORDING l 1118
    m_recordEventTimes.push_back(m_dbInstSimuMeso);

    // GlOBAL UPDATE OF THE EVENT TIMES
    if(m_mesoNodes.size()>0 )
    {

        m_pCurrentProcessedNode = m_mesoNodes.front();
        if(m_pCurrentProcessedNode->GetNextEventTime() == DBL_MAX)
        {
            m_pCurrentProcessedNode= NULL;
        }

    }
    else
    {
       assert(false);
    }

    SupprimeVehicules(m_dbInstSimuMeso);


}

bool CMesoNodeLess(const CMesoNode* a, const CMesoNode*b)
{
    return a->GetNextEventTime() < b->GetNextEventTime();
}

void Reseau::ChangeMesoNodeEventTime(CMesoNode * pNode, double dNewTime, Tuyau * pTuyauOutgoing)
{
    assert(pNode);
    if( pNode)
    {
        // Ajout du noeud s'il n'est pas dÃ©jÃ  dans la liste
        if(std::find(m_mesoNodes.begin(), m_mesoNodes.end(), pNode) == m_mesoNodes.end())
        {
            m_mesoNodes.push_back(pNode);
        }

        pNode->SetGlobalNextEvent(dNewTime,pTuyauOutgoing);

        // Tri des Ã©lÃ©ments en fonction du temps du prochain Ã©vÃ¨nement
        std::sort(m_mesoNodes.begin(), m_mesoNodes.end(), CMesoNodeLess);
    }

}

void Reseau::SaveControleurDeFeux(const std::deque<TraceDocTrafic *> &xmlDocTrafics)
{
    eCouleurFeux eEtat;
    bool bPremierInstCycle;
    bool bPrioritaire;
    std::deque<TraceDocTrafic* >::const_iterator itDocTraf;
      for(int i=0; i<(int)Liste_ControleursDeFeux.size(); i++)
    {
        ControleurDeFeux *pCF;
        pCF = (ControleurDeFeux*)Liste_ControleursDeFeux[i];

        const std::string & sCF = pCF->GetLabel();

        // Par couple d'entrÃ©e sortie
        for(size_t j=0; j < pCF->GetLstCoupleEntreeSortie()->size(); j++)
        {
            CoupleEntreeSortie cplES = (*pCF->GetLstCoupleEntreeSortie())[j];

            eEtat = pCF->GetStatutCouleurFeux(m_dbInstSimu, cplES.pEntree, cplES.pSortie, &bPremierInstCycle, NULL, &bPrioritaire);

            const std::string & sTE = cplES.pEntree->GetLabel();
            const std::string & sTS = cplES.pSortie->GetLabel();

            for( itDocTraf = xmlDocTrafics.begin(); itDocTraf != xmlDocTrafics.end(); ++itDocTraf)
            {
                (*itDocTraf)->AddSimFeux( sCF, sTE, sTS, eEtat, bPremierInstCycle, bPrioritaire);
            }

            if(m_SymMode == Reseau::SYM_MODE_STEP_EVE)
            {
                for(int k=0; k< cplES.pEntree->getNb_voies(); k++)
                {
                    std::string sTE = GetLibelleTronconEve(cplES.pEntree, k);

                    CarrefourAFeuxEx *pCAF;
                    pCAF = (CarrefourAFeuxEx*)cplES.pEntree->GetBriqueAval();

                    if(pCAF)
                    {
                        bool bAuthorized = false;
                        for(size_t typeVeh = 0; typeVeh < m_LstTypesVehicule.size() && !bAuthorized; typeVeh++)
                        {
                            bAuthorized = pCAF->IsMouvementAutorise(cplES.pEntree->GetVoie(k), cplES.pSortie, m_LstTypesVehicule[typeVeh], NULL);
                        }
                        if( bAuthorized )
                        {
                            for(int l=0; l< cplES.pSortie->getNb_voies(); l++)
                            {
                                std::string sTS = GetLibelleTronconEve(cplES.pSortie, l);
                                for( itDocTraf = xmlDocTrafics.begin(); itDocTraf != xmlDocTrafics.end(); ++itDocTraf)
                                {
                                    (*itDocTraf)->AddSimFeuxEVE( sCF, sTE, sTS, eEtat, bPremierInstCycle, bPrioritaire);
                                }
                            }
                        }
                    }
                }
            }

            /**m_pFicSimulation << "CDF : " << pCF->GetLabel() << " - origine : " << pCF->GetLstCoupleEntreeSortie()[j].pEntree->GetLabel() << " - destination : " <<  pCF->GetLstCoupleEntreeSortie()[j].pSortie->GetLabel();
            if(eEtat == FEU_VERT)
            {
                *m_pFicSimulation << " - feux : 1 " << std::endl;
            }
            else
                *m_pFicSimulation << " - feux : 0 " << std::endl; */
        }
    }
}

bool Reseau::CreateCoordinateTransformation(int sEPSGinput, int sEPSGoutput)
{
    OGRErr ogrErrI, ogErrO;

    m_pOGRSpRefInput = new OGRSpatialReference();
    ogrErrI = m_pOGRSpRefInput->importFromEPSG(sEPSGinput);

    m_pOGRSpRefOutput = new OGRSpatialReference();
    ogErrO = m_pOGRSpRefOutput->importFromEPSG(sEPSGoutput);

    m_pCoordTransf = NULL;
    if( ogrErrI==0 &&  ogErrO==0)
        m_pCoordTransf = OGRCreateCoordinateTransformation( m_pOGRSpRefInput, m_pOGRSpRefOutput );

    if(m_pCoordTransf)
        return true;
    else
        return false;
}

SymuViaTripNode* Reseau::GetOriginSymuViaTripNode(Connexion * pConnexion)
{
    SymuViaTripNode * pResult = NULL;
    for(size_t i = 0 ; i < Liste_origines.size() && !pResult; i++)
    {
        if(Liste_origines[i]->GetOutputConnexion() == pConnexion)
        {
            pResult = Liste_origines[i];
        }
    }
    return pResult;
}

SymuViaTripNode* Reseau::GetDestinationSymuViaTripNode(Connexion * pConnexion)
{
    SymuViaTripNode * pResult = NULL;
    for(size_t i = 0 ; i < Liste_destinations.size() && !pResult; i++)
    {
        if(Liste_destinations[i]->GetInputConnexion() == pConnexion)
        {
            pResult = Liste_destinations[i];
        }
    }
    return pResult;
}

std::string Reseau::GetParkingInfos()
{
    std::vector<TripNode*> lstDestinations;
    for (size_t iDest = 0; iDest < Liste_destinations.size(); iDest++)
    {
        SymuViaTripNode* pDest = Liste_destinations.at(iDest);

        // Le SG ne gÃ¨re que les destinations ponctuelles
        if (!pDest->GetInputPosition().IsZone())
        {
            lstDestinations.push_back(pDest);
        }
    }

    rapidjson::Document JsonParkings(rapidjson::kArrayType);
    rapidjson::Document::AllocatorType& allocator = JsonParkings.GetAllocator();

    m_mapAvailableParkingsForDestination.clear();

    for (size_t iParking = 0; iParking < Liste_parkings.size(); iParking++)
    {
        Parking * pParking = Liste_parkings.at(iParking);

        // Si le vÃ©hicule ne peut pas rentrer dans le parking (uniquement reliÃ© Ã  une entrÃ©e ponctuelle),
        // il ne nous intÃ©resse pas pour le SG.
        if (pParking->GetInputConnexion())
        {
            Connexion * pOutputConnexion = pParking->GetOutputConnexion();
            if (!pOutputConnexion)
            {
                // Si ce n'est qu'un parking sans sortie, on prend pour connexion origine du calcul d'itinÃ©raire le noeud amont
                // de l'entrÃ©e du parking...
                pOutputConnexion = pParking->GetInputConnexion()->m_LstTuyAssAm.front()->GetCnxAssAm();
            }
            std::vector<std::vector<std::pair<double, std::vector<std::string> > > > paths;
            m_pSymuScriptManager->ComputePathsPublicTransport(pOutputConnexion, lstDestinations, 0, 1, paths);


            rapidjson::Value destinations(rapidjson::kArrayType);
            for (size_t iDest = 0; iDest < paths.size(); iDest++)
            {
                const std::string & destId = lstDestinations[iDest]->GetID();
                const std::vector<std::pair<double, std::vector<std::string> > > & pathForDest = paths[iDest];
                if (!pathForDest.empty())
                {

                    rapidjson::Value destination(rapidjson::kObjectType);
                    rapidjson::Value strValue(rapidjson::kStringType);
                    strValue.SetString(destId.c_str(), (int)destId.length(), allocator);
                    destination.AddMember("id", strValue, allocator);

                    const std::vector<std::string> & path = pathForDest.front().second;
                    std::string concatenatedPath;
                    for (size_t iStep = 0; iStep < path.size(); iStep++)
                    {
                        if (iStep != 0)
                        {
                            concatenatedPath.append(" ");
                        }
                        concatenatedPath.append(path.at(iStep));
                    }
                    rapidjson::Value strPathValue(rapidjson::kStringType);
                    strPathValue.SetString(concatenatedPath.c_str(), (int)concatenatedPath.length(), allocator);
                    destination.AddMember("path", strPathValue, allocator);

                    destinations.PushBack(destination, allocator);

                    m_mapAvailableParkingsForDestination[lstDestinations[iDest]].insert(pParking);
                }
            }

            if (destinations.Size() > 0)
            {
                rapidjson::Value parking(rapidjson::kObjectType);

                rapidjson::Value strValue(rapidjson::kStringType);
                strValue.SetString(pParking->GetID().c_str(), (int)pParking->GetID().length(), allocator);
                parking.AddMember("id", strValue, allocator);

                rapidjson::Value position(rapidjson::kObjectType);
                position.AddMember("coordX", pParking->getCoordonnees().dbX, allocator);
                position.AddMember("coordY", pParking->getCoordonnees().dbY, allocator);
                parking.AddMember("position", position, allocator);

                parking.AddMember("destinations", destinations, allocator);

                JsonParkings.PushBack(parking, allocator);
            }

        }
    }

    rapidjson::StringBuffer buffer;
    buffer.Clear();
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    JsonParkings.Accept(writer);

    return buffer.GetString();
}

std::string Reseau::GetNetwork(bool bUseJSON)
{
    XMLDocTrafic * pXmlDocTrafic = NULL;
    rapidjson::Document JsonNetwork(rapidjson::kObjectType);
    rapidjson::Document::AllocatorType& allocator = JsonNetwork.GetAllocator();
    std::string sXmlFlow;

    if(!bUseJSON)
    {
        pXmlDocTrafic = new XMLDocTrafic(this, m_pXMLUtil->CreateXMLDocument(XS("OUT")));
        pXmlDocTrafic->Init("tmp.xml", "", SDateTime::Now(), SDateTime::Now(), 0, m_XMLDocData, 0, m_SimulationID, m_TraficID, m_ReseauID, m_pCoordTransf);
    }


    std::deque<Tuyau*>::iterator itBeg, itEnd, itTuyau;
    itBeg = m_LstTuyaux.begin();
    itEnd = m_LstTuyaux.end();

    rapidjson::Value links(rapidjson::kArrayType);

    for (itTuyau = itBeg; itTuyau != itEnd; itTuyau++)
    {
        Point *ptAm = new Point();
        Point *ptAv = new Point();

        ptAm->dbX = (*itTuyau)->GetExtAmont()->dbX;
        ptAm->dbY = (*itTuyau)->GetExtAmont()->dbY;

        ptAv->dbX = (*itTuyau)->GetExtAval()->dbX;
        ptAv->dbY = (*itTuyau)->GetExtAval()->dbY;

        if( m_pCoordTransf )
        {
            m_pCoordTransf->Transform(1, &ptAm->dbX, &ptAm->dbY);
            m_pCoordTransf->Transform(1, &ptAv->dbX, &ptAv->dbY);
        }

        std::deque<Point*>::iterator itPt;
        std::deque<Point*> LstPtsIntTransf;
        for( itPt = (*itTuyau)->GetLstPtsInternes().begin(); itPt!=(*itTuyau)->GetLstPtsInternes().end(); itPt++)
        {
            Point *pt = new Point();
            pt->dbX = (*itPt)->dbX;
            pt->dbY = (*itPt)->dbY;

            if( m_pCoordTransf )
                m_pCoordTransf->Transform(1, &pt->dbX, &pt->dbY);

            LstPtsIntTransf.push_back(pt);
        }

        if(bUseJSON)
        {
            rapidjson::Value link(rapidjson::kObjectType);
            rapidjson::Value strValue(rapidjson::kStringType);
            strValue.SetString((*itTuyau)->GetLabel().c_str(), (int)(*itTuyau)->GetLabel().length(), allocator);
            link.AddMember("id", strValue, allocator);
            rapidjson::Value laneWidth(rapidjson::kArrayType);
            for(size_t iLane = 0; iLane < (*itTuyau)->getLargeursVoies().size(); iLane++)
            {
                laneWidth.PushBack((*itTuyau)->getLargeursVoies()[iLane], allocator);
            }
            link.AddMember("lane width", laneWidth, allocator);
            link.AddMember("lane number", (*itTuyau)->getNb_voies(), allocator);
            double dbVitReg = (*itTuyau)->GetVitReg();
            // Si on a une vitesse reg dÃ©finie pour les VL, on utilise plutÃ´t celle-ci...
            if((*itTuyau)->m_LstVitReg.GetVariationEx(0))
            {
                VitRegDescription * pVitReg = (*itTuyau)->m_LstVitReg.GetVariationEx(0);
                TypeVehicule * pTV = GetVehicleTypeFromID("VL");
                if(pTV && pVitReg->vectPortions.find(pTV) != pVitReg->vectPortions.end())
                {
                    const std::map<int, std::vector<VitRegPortion> > & mapVitReg = pVitReg->vectPortions[pTV];
                    std::map<int, std::vector<VitRegPortion> >::const_iterator iter;
                    for(iter = mapVitReg.begin(); iter != mapVitReg.end(); ++iter)
                    {
                        if(!iter->second.empty())
                        {
                            dbVitReg = iter->second.front().data;
                            break;
                        }
                    }
                }
            }
            if(dbVitReg < DBL_MAX)
            {
                link.AddMember("speed limit", dbVitReg, allocator);
            }
            if(!(*itTuyau)->GetRoadLabel().empty())
            {
                rapidjson::Value strValue(rapidjson::kStringType);
                strValue.SetString((*itTuyau)->GetRoadLabel().c_str(), (int)(*itTuyau)->GetRoadLabel().length(), allocator);
                link.AddMember("road", strValue, allocator);
            }
            if((*itTuyau)->GetBriqueParente())
            {
                rapidjson::Value strValue(rapidjson::kStringType);
                strValue.SetString((*itTuyau)->GetBriqueParente()->GetID().c_str(), (int)(*itTuyau)->GetBriqueParente()->GetID().length(), allocator);
                link.AddMember("parent node", strValue, allocator);
            }

            // Voies rÃ©servÃ©es
            bool bHasExclusiveLane = false;
            rapidjson::Value exculiveLanes(rapidjson::kArrayType);
            for(int iLane = 0; iLane < (*itTuyau)->getNb_voies(); iLane++)
            {
                std::vector<TypeVehicule*> authorizedVehicles;
                for(size_t iTV = 0; iTV < m_LstTypesVehicule.size(); iTV++)
                {
                    if(!(*itTuyau)->IsVoieInterdite(m_LstTypesVehicule[iTV], iLane))
                    {
                        authorizedVehicles.push_back(m_LstTypesVehicule[iTV]);
                    }
                }
                rapidjson::Value exculiveLane(rapidjson::kArrayType);
                if(authorizedVehicles.size() != m_LstTypesVehicule.size())
                {
                    for(size_t i = 0; i < authorizedVehicles.size(); i++)
                    {
                        bHasExclusiveLane = true;
                        const std::string & typeLabel = authorizedVehicles[i]->GetLabel();
                        rapidjson::Value strValue(rapidjson::kStringType);
                        strValue.SetString(typeLabel.c_str(), (int)typeLabel.length(), allocator);
                        exculiveLane.PushBack(strValue, allocator);
                    }
                }
                exculiveLanes.PushBack(exculiveLane, allocator);
            }
            if(bHasExclusiveLane)
            {
                link.AddMember("exclusive lane", exculiveLanes, allocator);
            }

            rapidjson::Value geometry(rapidjson::kObjectType);
            geometry.AddMember("type", "LineString", allocator);
            rapidjson::Value coordinates(rapidjson::kArrayType);
            rapidjson::Value pointAm(rapidjson::kArrayType);
            pointAm.PushBack(ptAm->dbX, allocator);
            pointAm.PushBack(ptAm->dbY, allocator);
            coordinates.PushBack(pointAm, allocator);
            for( itPt = LstPtsIntTransf.begin(); itPt!=LstPtsIntTransf.end(); itPt++)
            {
                rapidjson::Value point(rapidjson::kArrayType);
                point.PushBack((*itPt)->dbX, allocator);
                point.PushBack((*itPt)->dbY, allocator);
                coordinates.PushBack(point, allocator);
            }
            rapidjson::Value pointAv(rapidjson::kArrayType);
            pointAv.PushBack(ptAv->dbX, allocator);
            pointAv.PushBack(ptAv->dbY, allocator);
            coordinates.PushBack(pointAv, allocator);
            geometry.AddMember("coordinates", coordinates, allocator);
            link.AddMember("geometry", geometry, allocator);

            links.PushBack(link, allocator);
        }
        else
        {
            pXmlDocTrafic->AddTroncon((*itTuyau)->GetLabel(), ptAm, ptAv, (*itTuyau)->getNb_voies(), (*itTuyau)->GetBriqueParente()?(*itTuyau)->GetBriqueParente()->GetID():"", LstPtsIntTransf, (*itTuyau)->GetLength(), (*itTuyau)->getLargeursVoies(), 0,0, *itTuyau);
        }

        delete ptAm;
        delete ptAv;

        for( itPt = LstPtsIntTransf.begin(); itPt!=LstPtsIntTransf.end(); itPt++)
        {
            delete (*itPt);
        }
        LstPtsIntTransf.clear();
    }

    if(bUseJSON)
    {
        if (!links.Empty())
        {
            JsonNetwork.AddMember("links", links, allocator);
        }

        // Ajout des parkings
        rapidjson::Value parkings(rapidjson::kArrayType);
        for(size_t iParking = 0; iParking < Liste_parkings.size(); iParking++)
        {
            rapidjson::Value parking(rapidjson::kObjectType);

            std::string parkingLabel;
            Point ptCoord;
            ptCoord.dbX = 0;
            ptCoord.dbY = 0;
            ptCoord.dbZ = 0;
            int nbPts = 0;

            if(Liste_parkings[iParking]->GetOutputConnexion())
            {
                parkingLabel = Liste_parkings[iParking]->GetOutputID();
                Point ptCoordEntree;
                CalcCoordFromPos( Liste_parkings[iParking]->GetOutputConnexion()->m_LstTuyAv.front(), 0, ptCoordEntree);

                ptCoord.dbX += ptCoordEntree.dbX;
                ptCoord.dbY += ptCoordEntree.dbY;
                ptCoord.dbZ += ptCoordEntree.dbZ;
                nbPts++;
            }

            if(Liste_parkings[iParking]->GetInputConnexion())
            {
                parkingLabel = Liste_parkings[iParking]->GetInputID();
                Point ptCoordSortie;
                CalcCoordFromPos( Liste_parkings[iParking]->GetInputConnexion()->m_LstTuyAm.front(),
                                  Liste_parkings[iParking]->GetInputConnexion()->m_LstTuyAm.front()->GetLength(),
                                  ptCoordSortie);

                ptCoord.dbX += ptCoordSortie.dbX;
                ptCoord.dbY += ptCoordSortie.dbY;
                ptCoord.dbZ += ptCoordSortie.dbZ;
                nbPts++;
            }

            ptCoord.dbX /= nbPts;
            ptCoord.dbY /= nbPts;
            ptCoord.dbZ /= nbPts;

            rapidjson::Value strValue(rapidjson::kStringType);
            strValue.SetString(parkingLabel.c_str(), (int)parkingLabel.length(), allocator);
            parking.AddMember("id", strValue, allocator);
            rapidjson::Value position(rapidjson::kObjectType);
            position.AddMember("type", "Point", allocator);
            rapidjson::Value coordinates(rapidjson::kArrayType);
            coordinates.PushBack(ptCoord.dbX, allocator);
            coordinates.PushBack(ptCoord.dbY, allocator);
            position.AddMember("coodinates", coordinates, allocator);
            parking.AddMember("position", position, allocator);

            parkings.PushBack(parking, allocator);
        }
        if (!parkings.Empty())
        {
            JsonNetwork.AddMember("parkings", parkings, allocator);
        }

        // Ajouts des transports guidÃ©s
        rapidjson::Value publicTransports(rapidjson::kObjectType);
        PublicTransportFleet * pPublicTransportFleet = GetPublicTransportFleet();
        const std::vector<Trip*> & lstLines = pPublicTransportFleet->GetTrips();
        bool bHasPublicTransports = false;
        rapidjson::Value lines(rapidjson::kArrayType);
        for(size_t iLine = 0; iLine < lstLines.size(); iLine++)
        {
            rapidjson::Value line(rapidjson::kObjectType);
            const std::string & linkeStr = lstLines[iLine]->GetID();
            rapidjson::Value strValueId(rapidjson::kStringType);
            strValueId.SetString(linkeStr.c_str(), (int)linkeStr.length(), allocator);
            line.AddMember("id", strValueId, allocator);
            rapidjson::Value strValueType(rapidjson::kStringType);
            const std::string & typeLabel = pPublicTransportFleet->GetTypeVehicule(lstLines[iLine])->GetLabel();
            strValueType.SetString(typeLabel.c_str(), (int)typeLabel.length(), allocator);
            line.AddMember("type", strValueType, allocator);
            std::vector<Tuyau*> * pFullPath = lstLines[iLine]->GetFullPath();
            rapidjson::Value lineLinks(rapidjson::kArrayType);
            for(size_t iLink = 0; iLink < pFullPath->size(); iLink++)
            {
                rapidjson::Value strValue(rapidjson::kStringType);
                const std::string & linkStr = pFullPath->at(iLink)->GetLabel();
                strValue.SetString(linkStr.c_str(), (int)linkStr.length(), allocator);
                lineLinks.PushBack(strValue, allocator);
            }
            if (!lineLinks.Empty())
            {
                line.AddMember("links", lineLinks, allocator);
            }
            lines.PushBack(line, allocator);
            bHasPublicTransports = true;
        }
        if (!lines.Empty())
        {
            publicTransports.AddMember("lines", lines, allocator);
        }
        rapidjson::Value stops(rapidjson::kArrayType);
        const std::vector<TripNode*> lstStops = pPublicTransportFleet->GetTripNodes();
        for(size_t iStop = 0; iStop < lstStops.size(); iStop++)
        {
            rapidjson::Value stop(rapidjson::kObjectType);
            Arret * pStop = (Arret*)lstStops[iStop];
            rapidjson::Value strValue(rapidjson::kStringType);
            strValue.SetString(pStop->GetID().c_str(), (int)pStop->GetID().length(), allocator);
            stop.AddMember("id", strValue, allocator);
            rapidjson::Value strValueLink(rapidjson::kStringType);
            const std::string & stopLinkLabel = pStop->GetInputPosition().GetLink()->GetLabel();
            strValueLink.SetString(stopLinkLabel.c_str(), (int)stopLinkLabel.length(), allocator);
            stop.AddMember("link", strValueLink, allocator);
            rapidjson::Value stoplines(rapidjson::kArrayType);
            for(size_t iLine = 0; iLine < pStop->getLstLTPs().size(); iLine++)
            {
                const std::string & stopLineStr = pStop->getLstLTPs()[iLine]->GetID();
                rapidjson::Value strValueStop(rapidjson::kStringType);
                strValueStop.SetString(stopLineStr.c_str(), (int)stopLineStr.length(), allocator);
                stoplines.PushBack(strValueStop, allocator);
            }
            if (!stoplines.Empty())
            {
                stop.AddMember("lines", stoplines, allocator);
            }
            stop.AddMember("out of lane", pStop->GetInputPosition().IsOutside(), allocator);
            Point ptCoord;
            CalcCoordFromPos( pStop->GetInputPosition().GetLink(), pStop->GetInputPosition().GetPosition(), ptCoord);
            rapidjson::Value position(rapidjson::kObjectType);
            position.AddMember("type", "Point", allocator);
            rapidjson::Value coordinates(rapidjson::kArrayType);
            coordinates.PushBack(ptCoord.dbX, allocator);
            coordinates.PushBack(ptCoord.dbY, allocator);
            position.AddMember("coordinates", coordinates, allocator);
            stop.AddMember("position", position, allocator);
            stops.PushBack(stop, allocator);
            bHasPublicTransports = true;
        }
        if(bHasPublicTransports)
        {
            publicTransports.AddMember("stops", stops, allocator);
            JsonNetwork.AddMember("public transport", publicTransports, allocator);
        }

        // Ajout des feux
        rapidjson::Value trafficlights(rapidjson::kArrayType);
        for(size_t iCDF = 0; iCDF < Liste_ControleursDeFeux.size(); iCDF++)
        {
            rapidjson::Value CDFJson(rapidjson::kObjectType);
            rapidjson::Value strValue(rapidjson::kStringType);
            const std::string & CDFStr = Liste_ControleursDeFeux[iCDF]->GetLabel();
            strValue.SetString(CDFStr.c_str(), (int)CDFStr.length(), allocator);
            CDFJson.AddMember("id", strValue, allocator);
            rapidjson::Value positions(rapidjson::kObjectType);
            positions.AddMember("type", "MultiPoint", allocator);
            rapidjson::Value coordinates(rapidjson::kArrayType);

            // Ajout de tous les points aval des tronÃ§ons amont des mouvements concernÃ©s par le CDF
            std::vector<CoupleEntreeSortie> * pCouples = Liste_ControleursDeFeux[iCDF]->GetLstCoupleEntreeSortie();
            std::set<Tuyau*> alreadyTreatedLink;
            for(size_t iCouple = 0; iCouple < pCouples->size(); iCouple++)
            {
                if(alreadyTreatedLink.find(pCouples->at(iCouple).pEntree) == alreadyTreatedLink.end())
                {
                    alreadyTreatedLink.insert(pCouples->at(iCouple).pEntree);
                    rapidjson::Value point(rapidjson::kArrayType);
                    point.PushBack(pCouples->at(iCouple).pEntree->GetAbsAval(), allocator);
                    point.PushBack(pCouples->at(iCouple).pEntree->GetOrdAval(), allocator);
                    coordinates.PushBack(point, allocator);
                }
            }

            positions.AddMember("coordinates", coordinates, allocator);
            CDFJson.AddMember("positions", positions, allocator);
            trafficlights.PushBack(CDFJson, allocator);
        }
        if (!trafficlights.Empty())
        {
            JsonNetwork.AddMember("traffic lights", trafficlights, allocator);
        }

        // Ajout des capteurs ponctuels
        if(m_pGestionsCapteur)
        {
            rapidjson::Value sensors(rapidjson::kArrayType);
            for(size_t iSensor = 0; iSensor < m_pGestionsCapteur->GetGestionCapteursPonctuels()->m_LstCapteurs.size(); iSensor++)
            {
                PonctualSensor * pSensor = dynamic_cast<PonctualSensor*>(m_pGestionsCapteur->GetGestionCapteursPonctuels()->m_LstCapteurs[iSensor]);
                rapidjson::Value sensor(rapidjson::kObjectType);

                rapidjson::Value strValue(rapidjson::kStringType);
                strValue.SetString(pSensor->GetUnifiedID().c_str(), (int)pSensor->GetUnifiedID().length(), allocator);
                sensor.AddMember("id", strValue, allocator);
                rapidjson::Value position(rapidjson::kObjectType);
                position.AddMember("type", "Point", allocator);
                Point sensorPt;
                CalcCoordFromPos(pSensor->GetTuyau(), pSensor->GetPosition(), sensorPt);
                rapidjson::Value coordinates(rapidjson::kArrayType);
                coordinates.PushBack(sensorPt.dbX, allocator);
                coordinates.PushBack(sensorPt.dbY, allocator);
                position.AddMember("coordinates", coordinates, allocator);
                sensor.AddMember("position", position, allocator);
                sensors.PushBack(sensor, allocator);
            }
            if (!sensors.Empty())
            {
                JsonNetwork.AddMember("sensors", sensors, allocator);
            }
        }
    }

    std::string sRes;
    if(bUseJSON)
    {
        rapidjson::StringBuffer buffer;
        buffer.Clear();
        rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
        JsonNetwork.Accept(writer);
        sXmlFlow = buffer.GetString();
    }
    else
    {
        sXmlFlow = m_pXMLUtil->getOuterXml(pXmlDocTrafic->GetSymTronconsNode());
    }

    if(pXmlDocTrafic)
    {
        pXmlDocTrafic->Terminate();
        delete pXmlDocTrafic;
    }

    return sXmlFlow;
}

std::string Reseau::GetNodeVehicles() const
{
    std::string sXmlFlow = "";
    if( GetXmlDocTrafic().size() == 1  &&  GetXmlDocTrafic().front()->GetXMLDocTrafic()->GetSymVehiclesNode() )
    {
        sXmlFlow = m_pXMLUtil->getOuterXml( GetXmlDocTrafic().front()->GetXMLDocTrafic()->GetSymVehiclesNode() );
    }
    return sXmlFlow;
}



bool Reseau::DoVehicleListSensorExtraction() const
{
    return m_bDoVehicleListSensorExtraction;
}

void Reseau::AddVehiculeToDocTrafic(int nID, const std::string & strLibelle, const std::string & strType, const std::string & strGMLType,
        double dbKx, double dbVx, double dbW,
        const std::string & strEntree, const std::string & strSortie, const std::string & strZoneEntree, const std::string & strZoneSortie, const std::string & strRoute,
        double dbInstCreation, const std::string & sVoie, bool bAgressif,
        int iLane, const std::string & sLine, const std::vector<std::string> & initialPath, const std::deque<PlageAcceleration> & plagesAcc,
        const std::string & motifOrigine, const std::string & motifDestination)
{
   // std::deque<DocTrafic *> doctrafics =  GetVariations(m_dbInstSimu, &m_xmlDocTrafics,m_dbLag);
 //   std::deque<DocTrafic *>::iterator itDocTrafic;
    // OTK : test commentÃ© suite Ã  des problÃ¨mes de performances identifiÃ©s depuis cette modif (rev SVN 453).
    // A priori, il faut effecter les AddVehicle dans tous les cas : la dÃ©sactivation de la sortie des trajectoires ne
    // doit a priori pas dÃ©sactiver la crÃ©ation du noeud final VEHS.
    // De plus, des problÃ¨mes de performances arrivent car sans le AddVehicule initial, les mÃ©thodes UpdateItineraireVehicule ou UpdateInstantSortie
    // sont longues car font une recherche du noeud VEH sans le trouver Ã  chaque pas de temps.
    // Modif Ã  valider par Manu ?

   // if( IsSortieTraj() )
    {
        if (m_pModuleAffectation && m_pModuleAffectation->GetCurrentDocTraficTmp())
        {
            m_pModuleAffectation->GetCurrentDocTraficTmp()->AddVehicule(nID, strLibelle, strType, strGMLType,
                dbKx, dbVx, dbW, strEntree, strSortie, strZoneEntree, strZoneSortie, strRoute, dbInstCreation, sVoie, bAgressif, iLane, sLine, initialPath, plagesAcc, motifOrigine, motifDestination, true);
        }
        else
        {
            std::deque< TimeVariation<TraceDocTrafic> >::iterator itAllDocTrafic;
            // Pour Tous les fichiers Ã  extratire
            for( itAllDocTrafic = m_xmlDocTrafics.begin(); itAllDocTrafic != m_xmlDocTrafics.end(); itAllDocTrafic++ )
            {
                // regarde si on doit extraire les instants des vÃ©hicules
                bool bExtract = false;
                if(itAllDocTrafic->m_pPlage != NULL)
                {
                    if(itAllDocTrafic->m_pPlage->m_Debut <= m_dbInstSimu + m_dbLag && itAllDocTrafic->m_pPlage->m_Fin >= m_dbInstSimu+ m_dbLag)
                    {
                        bExtract = true;
                    }
                }

                if(  bExtract )
                {
                    if( m_vehiclesToAdd[itAllDocTrafic->m_pData.get()].size()> 0 )
                    {
                        if (DoSortieTraj()) // OTK - 29062016 - pas de noeuds CREATION si on ne sort pas les trajectoires
                        {
                            std::deque<SVehicleToAdd>::const_iterator it = m_vehiclesToAdd[itAllDocTrafic->m_pData.get()].begin();
                            while (it != m_vehiclesToAdd[itAllDocTrafic->m_pData.get()].end() )
                            {
                                itAllDocTrafic->m_pData->AddVehiculeToCreationNode(it->nID, it->strLibelle, it->strType,
                                    it->dbKx, it->dbVx, it->dbW, it->strEntree, it->strSortie, it->strRoute, dbInstCreation, it->sVoie, it->bAgressif);
                                it++;
                            }
                        }
                        m_vehiclesToAdd[itAllDocTrafic->m_pData.get()].clear();

                    }
                    itAllDocTrafic->m_pData->AddVehicule(nID, strLibelle, strType, strGMLType,
                                dbKx,  dbVx,  dbW,  strEntree, strSortie, strZoneEntree, strZoneSortie, strRoute, dbInstCreation, sVoie,  bAgressif, iLane, sLine, initialPath, plagesAcc, motifOrigine, motifDestination, true);
                }

                else
                {

                    itAllDocTrafic->m_pData->AddVehicule(nID, strLibelle, strType, strGMLType,
                        dbKx, dbVx, dbW, strEntree, strSortie, strZoneEntree, strZoneSortie, strRoute, dbInstCreation, sVoie, bAgressif, iLane, sLine, initialPath, plagesAcc, motifOrigine, motifDestination, false);

                    m_vehiclesToAdd[itAllDocTrafic->m_pData.get()].push_back( SVehicleToAdd( nID, strLibelle, strType,
                                dbKx,  dbVx,  dbW,  strEntree, strSortie, strZoneEntree, strZoneSortie, strRoute, dbInstCreation, sVoie,  bAgressif)) ;

                }
            } // rof all doc trafic
        }
    } // fi is sortie traj


}

bool Reseau::LoadFluxFromNode(SymuViaTripNode * pOrigine, Logger  *pChargement,
                 DOMNode *pFluxNode, TypeVehicule * pTypeVehicle)
{

    int nNbVoie, nTypes;
    double dbDuree;
    string strTmp;


    VectDest  *pVectDest;
    Tuyau * pT = NULL;

    DOMNode *pXMLRepartitionsVoie = NULL;

    if( pFluxNode )
    {
         pXMLRepartitionsVoie =m_pXMLUtil->SelectSingleNode( "./REP_VOIES", pFluxNode->getOwnerDocument(),
                            (DOMElement*)pFluxNode);
    }

     // RÃ©partition des types de vÃ©hicules
    nTypes = (int)m_LstTypesVehicule.size();

    // Nombre de listes de coefficients dÃ©finis (autant que de voies dans le cas ponctuel, une seule pour les zones)
    ZoneDeTerminaison * pZone = dynamic_cast<ZoneDeTerminaison*>(pOrigine);
    int nbCoeffs = pZone != NULL ? 1 : pOrigine->GetOutputConnexion()->m_LstTuyAv.front()->getNb_voies();

    if(pXMLRepartitionsVoie)
    {
        nNbVoie = pOrigine->GetOutputConnexion()->m_LstTuyAv.front()->getNbVoiesDis();
        XMLSize_t countj = pXMLRepartitionsVoie->getChildNodes()->getLength();
        for(XMLSize_t j=0; j<countj;j++)
        {
            std::vector<double> pCoeff;
            boost::shared_ptr<RepartitionEntree> pE = boost::make_shared<RepartitionEntree>();
            DOMNode * xmlnode;

            xmlnode = pXMLRepartitionsVoie->getChildNodes()->item(j);
            if (xmlnode->getNodeType() != DOMNode::ELEMENT_NODE) continue;

            // DurÃ©e
            vector<PlageTemporelle*> plages;
            if(!GetXmlDuree(xmlnode,this,dbDuree,plages, pChargement))
            {
                return false;
            }

            // coeffs
            GetXmlAttributeValue(xmlnode, "coeffs", strTmp, pChargement);
            std::deque<std::string> split = SystemUtil::split(strTmp, ' ');
            if (split.size() != nNbVoie)
            {
                *pChargement << Logger::Error << "ERROR : the number of values for the 'coeffs' attribute of the REP_VOIES node (input : " << pOrigine->GetOutputID();
                *pChargement << Logger::Error << " - repartition : "<< j+1 <<  " ) is invalid (must be equal to the number of lanes of the link " << pOrigine->GetOutputConnexion()->m_LstTuyAv.front()->GetLabel() <<").";
                *pChargement << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                return false;
            }
            double dbS = 0;
            for(int k=0; k<nNbVoie;k++)
            {
                pCoeff.push_back(atof( split.at(k).c_str() ));
                dbS += pCoeff[k];
            }
            if( fabs(dbS - 1) > 0.000001 )
            {
                *pChargement << Logger::Error << "ERROR : the values of the 'coeffs' attribute of the REP_VOIES node (input : " << pOrigine->GetOutputID();
                *pChargement << Logger::Error << " - repartition : "<< j+1 <<  " ) are wrong (the sum must be 1) ";
                *pChargement << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                return false;
            }
            pE->pCoefficients = pCoeff;
            if(plages.size() > 0)
            {
                for(size_t iPlage = 0; iPlage < plages.size(); iPlage++)
                {
                    AddVariation(plages[iPlage], pE, pOrigine->GetLstRepVoieInit(pTypeVehicle));
                }
            }
            else
            {
                AddVariation(dbDuree, pE, pOrigine->GetLstRepVoieInit(pTypeVehicle));
            }
        }

        // vÃ©rification de la couverture temporelle
        vector<PlageTemporelle*> plages;
        for(size_t iPlage = 0; iPlage < pOrigine->GetLstRepVoieInit(pTypeVehicle)->size(); iPlage++)
        {
            PlageTemporelle * pPlage = pOrigine->GetLstRepVoieInit(pTypeVehicle)->at(iPlage).m_pPlage;
            if(pPlage)
            {
                plages.push_back(pPlage);
            }
        }
        if(plages.size() > 0 && !CheckPlagesTemporellesEx(m_dbDureeSimu, plages))
        {
            *pChargement << Logger::Error << "ERROR : The time frames defined for the lanes repartition at input " << pOrigine->GetOutputID() << " don't cover the whole simulation duration !" << std::endl;
            *pChargement << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
            return false;
        }

    }
    else if(dynamic_cast<ZoneDeTerminaison*>(pOrigine) == NULL) // pour le cas des zones, on ne fait rien : on utilisera une repartition homogÃ¨ne sur les voies
    {
        std::vector<double> pCoeff;

        // Si la rÃ©partition n'est pas dÃ©finie, elle est uniforme sur toutes les voies
        if(pOrigine->GetOutputConnexion()->m_LstTuyAv.size() != 0)
        {
            nNbVoie = pOrigine->GetOutputConnexion()->m_LstTuyAv.front()->getNbVoiesDis();
            for(int i=0; i< nNbVoie; i++)
                pCoeff.push_back(1. / nNbVoie);
        }
        boost::shared_ptr<RepartitionEntree> pE = boost::make_shared<RepartitionEntree>();
        pE->pCoefficients = pCoeff;
        AddVariation(m_dbDureeSimu, pE, pOrigine->GetLstRepVoieInit(pTypeVehicle));
    }
    // Fin traitement REPARTITION_VOIE




    // REP_DESTINATIONS
    if(IsUsedODMatrix() && ( pOrigine->IsTypeDemande() || pOrigine->IsTypeDistribution() ) && pFluxNode )
    {
        // Matrices OD de l'entrÃ©e considÃ©rÃ©e
        DOMNode * pXMLRepDestinations;
        DOMNode * pXMLRepDestination;

        pXMLRepDestinations = m_pXMLUtil->SelectSingleNode( "./REP_DESTINATIONS", pFluxNode->getOwnerDocument(), (DOMElement*)pFluxNode);
        SymuViaTripNode * pDestination;
        if(pXMLRepDestinations)
        {
            XMLSize_t countj = pXMLRepDestinations->getChildNodes()->getLength();
            for(XMLSize_t j=0; j<countj; j++)
            {
                pXMLRepDestination = pXMLRepDestinations->getChildNodes()->item(j);
                if (pXMLRepDestination->getNodeType() != DOMNode::ELEMENT_NODE) continue;

                // Duree
                vector<PlageTemporelle*> plages;
                if(!GetXmlDuree(pXMLRepDestination, this, dbDuree,plages, pChargement))
                {
                    return false;
                }

                // Pour chaque type de vÃ©hicule
              //  for(size_t iTV = 0; iTV < m_LstTypesVehicule.size(); iTV++)
                {
                    boost::shared_ptr<SimMatOrigDest> pMatOrigDest = boost::make_shared<SimMatOrigDest>();

                    // Destinations
                    double dbSum = 0;
                    XMLSize_t countk = pXMLRepDestination->getChildNodes()->getLength();
                    for(XMLSize_t k=0; k<countk; k++)
                    {
                        // Destination
                        DOMNode* xmlChildk = pXMLRepDestination->getChildNodes()->item(k);
                        if (xmlChildk->getNodeType() != DOMNode::ELEMENT_NODE) continue;

                        GetXmlAttributeValue(xmlChildk, "sortie", strTmp, pChargement);
                        char cDestType;
						bool bExit =  false;
                        pDestination = GetDestinationFromID(strTmp, cDestType);

						if (cDestType == 'S')	// Ponctual exit ?
							bExit = true;

                        // Correction bug nÂ°39 : prÃ©vention du plantage si la destination n'est pas en aval d'un tronÃ§on
                        if(!pDestination)
                        {
                            *pChargement << Logger::Error << "ERROR : the destination " << strTmp << " is incorrect (the matching extremity may be not declared as the downstream node of a link ?)" << std::endl;
                            *pChargement << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                            return false;
                        }

                        pOrigine->AddDestinations(pDestination);

                        // Coefficient d'affectation
                        double dbCoeffOdAffectation;
                        GetXmlAttributeValue(xmlChildk, "coeffOD", dbCoeffOdAffectation, pChargement);
                        dbSum += dbCoeffOdAffectation;

                        // Ã©ventuelle valeur particuliÃ¨re de teta du logit
                        double dbTetaLogit = 0;
                        bool bHasTetaLogit = GetXmlAttributeValue(xmlChildk, "affectation_teta_logit", dbTetaLogit, pChargement);

                        // Ã©ventuelle valeur particuliÃ¨re du nombre de plus courts chemins Ã  calculer
                        int nNbPlusCourtsCheminsParticulier = 0;
                        bool bHasPlusCourtsChemins = GetXmlAttributeValue(xmlChildk, "nombre_pluscourtchemin", nNbPlusCourtsCheminsParticulier, pChargement);

                        // Evolution nÂ°114 : Ã©ventuels paramÃ¨tres particuliers pour le calcul du commonality factor
                        bool bHasCustomCommonalityParams = false;
                        double dbCommonalityAlpha = m_dbCommonalityAlpha;
                        double dbCommonalityBeta = m_dbCommonalityBeta;
                        double dbCommonalityGamma = m_dbCommonalityGamma;

                        if(GetXmlAttributeValue(xmlChildk, "affectation_commonality_alpha", dbCommonalityAlpha, pChargement))
                        {
                            bHasCustomCommonalityParams = true;
                        }
                        if(GetXmlAttributeValue(xmlChildk, "affectation_commonality_beta", dbCommonalityBeta, pChargement))
                        {
                            bHasCustomCommonalityParams = true;
                        }
                        if(GetXmlAttributeValue(xmlChildk, "affectation_commonality_gamma", dbCommonalityGamma, pChargement))
                        {
                            bHasCustomCommonalityParams = true;
                        }

                        pVectDest = new VectDest;
                        pVectDest->pDest = pDestination;
                        pVectDest->dbCoeff = dbCoeffOdAffectation;
                        pVectDest->bHasTetaLogit = bHasTetaLogit;
                        pVectDest->dbTetaLogit = dbTetaLogit;
                        pVectDest->bHasNbPlusCourtsChemins = bHasPlusCourtsChemins;
                        pVectDest->nNbPlusCourtsChemins = nNbPlusCourtsCheminsParticulier;
                        pVectDest->bHasCustomCommonalityParameters = bHasCustomCommonalityParams;
                        pVectDest->dbCommonalityAlpha = dbCommonalityAlpha;
                        pVectDest->dbCommonalityBeta = dbCommonalityBeta;
                        pVectDest->dbCommonalityGamma = dbCommonalityGamma;

                        // lecture des itinÃ©raires prÃ©dÃ©finis dans RESEAU ou spÃ©cifiÃ©
                        int nRouteCount = (int)xmlChildk->getChildNodes()->getLength();
                        for( int iRoute = 0; iRoute < nRouteCount; ++iRoute)
                        {
                            DOMNode* pRouteNode = xmlChildk->getChildNodes()->item(iRoute); //m_pXMLUtil->SelectNodes("ROUTE", xmlChildk->getOwnerDocument(), (DOMElement*)xmlChildk);

                            // rÃ©cupÃ©ration du coefficient de l'itinÃ©raire
                            double dbItiCoeff = 0;
                            GetXmlAttributeValue(pRouteNode, "coeffAffectation", dbItiCoeff, pChargement);
                            string strRouteId;
                            GetXmlAttributeValue(pRouteNode, "id", strRouteId, pChargement);
                            std::vector<Tuyau*> iti; // route composÃ©e de troncons
                            Connexion * pJunction = NULL;

                            // Teste si l'id de la route est dans le reseau, sinon on lit l'itineraire dans This node
							if (strRouteId != "")
							{
								if (m_routes.find(strRouteId) != m_routes.end())
								{
									iti = m_routes.find(strRouteId)->second.first;
									pJunction = m_routes.find(strRouteId)->second.second;
								}
								else
								{
									*pChargement << Logger::Error << "ERROR : the route " << strRouteId.c_str() << " of the input " << pOrigine->GetOutputID() << " doesn't exist." << std::endl;
									*pChargement << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
									return false;
								}

								// on vÃ©rifie qu'on n'a pas de doublon dans les routes.
								bool bHasSameRoute = false;
								for (size_t iExistingRoute = 0; iExistingRoute < pVectDest->lstItineraires.size(); iExistingRoute++)
								{
									if (pVectDest->lstItineraires[iExistingRoute].second.first.first == iti
										&& pVectDest->lstItineraires[iExistingRoute].second.first.second == pJunction)
									{
										bHasSameRoute = true;
										// Somme des coefficients des routes identiques plutÃ´t qu'ajout d'une nouvelle route doublon (le code d'affectation n'est pas prÃ©vu pour les doublons)
										pVectDest->lstItineraires[iExistingRoute].first += dbItiCoeff;
										*pChargement << Logger::Error << "WARNING : the route " << strRouteId.c_str() << " is identical to the route " << pVectDest->lstItineraires[iExistingRoute].second.second << ". Only the first one is kept with a consequently adapted coefficient." << std::endl;
										*pChargement << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
										break;
									}
								}

								// Check if last link allows to access destination (only for ponctual destination)
								if (iti.size() > 0)
								{
								
									if (bExit && iti.back()->getConnectionAval())
									{
										if (iti.back()->getConnectionAval()->GetNumID() != pDestination->GetInputConnexion()->GetNumID())
										{
											*pChargement << Logger::Error << "ERROR : the route " << strRouteId.c_str() << " doesn't allow to access " << pDestination->GetID() << " for (" << pOrigine->GetID() << "," << pDestination->GetID() << ") pair " << std::endl;
											*pChargement << Logger::Info;
											return false;
										}
									}
								}

                                if (!bHasSameRoute)
                                {
                                    pVectDest->lstItineraires.push_back(std::make_pair(dbItiCoeff, std::make_pair(std::make_pair(iti, pJunction), strRouteId)));
                                    pVectDest->lstRouteNames.push_back(strRouteId);
                                }
                            }
                            else
                            {
                                // RÃ©cupÃ©ration de la liste des tronÃ§ons constituant l'itinÃ©raire
                                DOMXPathResult* pTronconsList = m_pXMLUtil->SelectNodes("TRONCONS_/TRONCON_", pRouteNode->getOwnerDocument(), (DOMElement*)pRouteNode);
                                if( pTronconsList )
                                {
                                    XMLSize_t countTroncons = pTronconsList->getSnapshotLength();
                                    for(XMLSize_t tronconIdx=0; tronconIdx<countTroncons; tronconIdx++)
                                    {
                                        pTronconsList->snapshotItem(tronconIdx);
                                        DOMNode *pTronconNode = pTronconsList->getNodeValue();

                                        // RÃ©cupÃ©ration du libellÃ© du tronÃ§on
                                        std::string tronconLib;
                                        GetXmlAttributeValue(pTronconNode, "libelle", tronconLib, pChargement);

                                        // RÃ©cupÃ©ration du tronÃ§on correspondant
                                        Tuyau * pTuyIti = GetLinkFromLabel(tronconLib);

                                        // Evolution nÂ°40 : ajout d'un message d'erreur si un itinÃ©raire spÃ©cifiÃ©
                                        // n'est pas composÃ© de tronÃ§ons contigus
                                        if(iti.size() > 0)
                                        {
                                            Connexion * pConAssAm = pTuyIti->GetBriqueAmont();
                                            if(!pConAssAm)
                                            {
                                                pConAssAm= pTuyIti->getConnectionAmont();
                                            }
                                            if(!pConAssAm->IsTuyauAmont(iti[iti.size() -1]))
                                            {
                                                *pChargement << Logger::Error << "ERROR : predefined path : links " << iti[iti.size() -1]->GetLabel() << " and " << pTuyIti->GetLabel() << " are not connected." << std::endl;
                                                *pChargement << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                                                return false;
                                            }
                                        }

                                        iti.push_back(pTuyIti);
                                    } // rof each troncon

                                    pTronconsList->release();
                                }// fi pTronconsList != NUL

                                // rÃ©cupÃ©ration de l'Ã©ventuel point de jonction
                                DOMNode * pXMLNoeud = m_pXMLUtil->SelectSingleNode("NOEUD", pRouteNode->getOwnerDocument(), (DOMElement*)pRouteNode);
                                if (pXMLNoeud)
                                {
                                    GetXmlAttributeValue(pXMLNoeud, "id", strTmp, pChargement);
                                    char cTmp;
                                    pJunction = GetConnectionFromID(strTmp, cTmp);
                                }

                                pVectDest->lstItineraires.push_back(std::make_pair(dbItiCoeff, std::make_pair(std::make_pair(iti, pJunction),strRouteId)));


                            } // fi else on crÃ©Ã©e une route depuis this node

                        } //rof chaque noeud route

                        // Calcul du relicat de coefficient non affectÃ© Ã  un itinÃ©raire
                        if(pVectDest->lstItineraires.size() > 0)
                        {
                            double dbItiCoeffSum = 0.0;
                            for(size_t itiIdx = 0; itiIdx < pVectDest->lstItineraires.size(); itiIdx++)
                            {
                                dbItiCoeffSum += pVectDest->lstItineraires[itiIdx].first;
                            }
                            // Pour pouvoir faire fonctionner les anciens fichiers invalides sans effort, on normalise automatiquement
                            // les coefficients si la somme est proche de 1 au sens de l'ancienne tolÃ©rance (0.0001)
                            double dbTolerance = pVectDest->lstItineraires.size() * std::numeric_limits<double>::epsilon();
                            if(fabs(dbItiCoeffSum-1.0) > dbTolerance && fabs(dbItiCoeffSum-1.0) < 0.0001)
                            {
                                // *pChargement << Logger::Warning << "WARNING : la somme des coefficents pour l'ensemble des itinÃ©raires dÃ©finis pour l'entrÃ©e " << pOrigine->GetOutputID() << " et la sortie " << pDestination->GetInputID() << " - variation " << j <<" de l'Ã©lÃ©ment REP_DESTINATION - " << " est proche mais diffÃ©rente de 1 : normalisation automatique des coefficients..." << std::endl;
                                // *pChargement << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                                // Normalisation automatique des coefficients
                                for(size_t itiIdx = 0; itiIdx < pVectDest->lstItineraires.size(); itiIdx++)
                                {
                                    pVectDest->lstItineraires[itiIdx].first /= dbItiCoeffSum;
                                }
                                dbItiCoeffSum = 1.0;
                            }
                            if( dbItiCoeffSum-1.0 > dbTolerance)
                            {
                                *pChargement << Logger::Error << "ERROR : the sum of coefficients for all paths from input " << pOrigine->GetOutputID() << " to output " << pDestination->GetInputID() << " - variation " << j <<" of element REP_DESTINATION - " << " must be lower or equal to 1" << std::endl;
                                *pChargement << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                                return false;
                            }
                            if(1.0-dbItiCoeffSum > dbTolerance)
                            {
                                int nbPlusCourtsChemins = bHasPlusCourtsChemins ? nNbPlusCourtsCheminsParticulier : m_nNbPlusCourtChemin;
                                if((int)pVectDest->lstItineraires.size() >= nbPlusCourtsChemins)
                                {
                                    *pChargement << Logger::Error << "ERROR : the sum of coefficients for all paths from input " << pOrigine->GetOutputID() << " to output " << pDestination->GetInputID() << " - variation " << j <<" de l'Ã©lÃ©ment REP_DESTINATION - " << " must be equal to 1 if the number of predefined paths is equal or higher than the requested number of shortest paths" << std::endl;
                                    *pChargement << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                                    return false;
                                }
                            }
                            pVectDest->dbRelicatCoeff -= dbItiCoeffSum;
                        }

                        pMatOrigDest->MatOrigDest.push_back(pVectDest);

                        if(k == countk - 1)
                        {
                            // VÃ©rification de la somme des coefficients

                            // rmq. : l'Ã©cart sur la somme peut dÃ©passer epsilon (la perte de precision se cumule)
                            double dbTolerance = std::numeric_limits<double>::epsilon() * countk;

                            // Pour pouvoir faire fonctionner les anciens fichiers invalides sans effort, on normalise automatiquement
                            // les coefficients si la somme est proche de 1 au sens de l'ancienne tolÃ©rance (0.0001)
                            if(fabs(dbSum-1.0) > dbTolerance && fabs(dbSum-1.0) < 0.0001)
                            {
                                // *pChargement << Logger::Warning << "WARNING : la somme des coefficents pour l'ensemble des destinations de l'entrÃ©e " << pOrigine->GetOutputID() << " - variation " << j <<" de l'Ã©lÃ©ment REP_DESTINATION - " << " est proche mais diffÃ©rente de 1 : normalisation automatique des coefficients..." << std::endl;
                                // *pChargement << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                                // Normalisation automatique des coefficients
                                for(size_t iOD = 0; iOD < pMatOrigDest->MatOrigDest.size(); iOD++)
                                {
                                    pMatOrigDest->MatOrigDest[iOD]->dbCoeff /= dbSum;
                                }
                                dbSum = 1.0;
                            }

                            if( fabs(dbSum-1) > dbTolerance)
                            {
                                *pChargement << Logger::Error << "ERROR : the coefficients sum to all destinations from input " << pOrigine->GetOutputID() << " - variation " << j <<" of element REP_DESTINATION - " << " must be equal to 1 " << std::endl;
                                *pChargement << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                                return false;
                            }
                        }
                    }




                    if(plages.size() > 0)
                    {
                        for(size_t iPlage = 0; iPlage < plages.size(); iPlage++)
                        {
                            AddVariation<>( plages[iPlage], pMatOrigDest, &pOrigine->GetLstCoeffDestInit(pTypeVehicle) );
                            m_LstItiChangeInstants.insert(plages[iPlage]->m_Debut + GetTimeStep());
                        }
                    }
                    else
                    {
                        std::deque<TimeVariation<SimMatOrigDest>>& lstTVMatOD = pOrigine->GetLstCoeffDestInit(pTypeVehicle);
                        double dbStartPeriodTime = 0;
                        for(size_t iPeriod = 0; iPeriod < lstTVMatOD.size(); iPeriod++)
                        {
                            dbStartPeriodTime += lstTVMatOD[iPeriod].m_dbPeriod;
                        }
                        m_LstItiChangeInstants.insert(dbStartPeriodTime + GetTimeStep());
                        AddVariation<>( dbDuree, pMatOrigDest, &lstTVMatOD);
                    }
                }
            }

            // vÃ©rification de la couverture temporelle pour tous le type de vÃ©hicules
            {
                vector<PlageTemporelle*> plages;
                for(size_t iPlage = 0; iPlage < pOrigine->GetLstCoeffDestInit(pTypeVehicle).size(); iPlage++)
                {
                    PlageTemporelle * pPlage = pOrigine->GetLstCoeffDestInit(pTypeVehicle)[iPlage].m_pPlage;
                    if(pPlage)
                    {
                        plages.push_back(pPlage);
                    }
                }
                if(plages.size() > 0 && !CheckPlagesTemporellesEx(m_dbDureeSimu, plages))
                {
                    *pChargement << Logger::Error << "ERROR : The time frames defined for the REP_DESTINATION nodes of input " << pOrigine->GetOutputID();
                    if (pTypeVehicle)
                    {
                        *pChargement << " for vehicle type " << pTypeVehicle->GetLabel();
                    }
                    *pChargement << " don't cover the whole simulation duration !" << std::endl;
                    *pChargement << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                    return false;
                }
            }

        }
        else
        {
            if(pOrigine->GetLstDemandeInit(pTypeVehicle)->GetLstTV()->size() > 0)
            {
                *pChargement << Logger::Warning << "WARNING : the REP_DESTINATION node of input " << pOrigine->GetOutputID() << " is missing whereas the flow behavior type is set to destination or itinerary.";
                *pChargement << Logger::Warning << "Only vehicles knowing their destination can be created at this input. The requested demand level will not be reached." << endl << endl;
                *pChargement << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
            }
        }
        // fin traitement flux
    }

    if (pFluxNode)
    {
        DOMNode *pXMLRepMotifs = pXMLRepMotifs = m_pXMLUtil->SelectSingleNode("./REP_MOTIFS", pFluxNode->getOwnerDocument(), (DOMElement*)pFluxNode);
        if (pXMLRepMotifs)
        {
            XMLSize_t countj = pXMLRepMotifs->getChildNodes()->getLength();
            DOMNode * pXMLRepMotif;
            for (XMLSize_t j = 0; j<countj; j++)
            {
                pXMLRepMotif = pXMLRepMotifs->getChildNodes()->item(j);
                if (pXMLRepMotif->getNodeType() != DOMNode::ELEMENT_NODE) continue;

                // Duree
                vector<PlageTemporelle*> plages;
                if (!GetXmlDuree(pXMLRepMotif, this, dbDuree, plages, pChargement))
                {
                    return false;
                }

                boost::shared_ptr<CRepMotif> pRepMotif = boost::make_shared<CRepMotif>();

                // Destinations
                XMLSize_t countk = pXMLRepMotif->getChildNodes()->getLength();
                for (XMLSize_t k = 0; k<countk; k++)
                {
                    // Destination
                    DOMNode* xmlChildk = pXMLRepMotif->getChildNodes()->item(k);
                    if (xmlChildk->getNodeType() != DOMNode::ELEMENT_NODE) continue;

                    SymuViaTripNode * pDestination = NULL;
                    if (GetXmlAttributeValue(xmlChildk, "destination", strTmp, pChargement))
                    {
                        char cDestType;
                        pDestination = GetDestinationFromID(strTmp, cDestType);

                        if (!pDestination)
                        {
                            *pChargement << Logger::Error << "ERROR : destination " << strTmp << " could not be found." << std::endl;
                            *pChargement << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                            return false;
                        }
                    }

                    CRepMotifDest repMotifDest;
                    repMotifDest.setDestination(pDestination);

                    double dbSum = 0;
                    XMLSize_t countl = xmlChildk->getChildNodes()->getLength();
                    for (XMLSize_t l = 0; l < countl; l++)
                    {
                        // Coefficient pour un couple motifs OD
                        DOMNode* xmlChildl = xmlChildk->getChildNodes()->item(l);
                        if (xmlChildl->getNodeType() != DOMNode::ELEMENT_NODE) continue;


                        GetXmlAttributeValue(xmlChildl, "motif_origine", strTmp, pChargement);
                        CMotif * pMotifOrigine = GetMotifFromID(strTmp);

                        GetXmlAttributeValue(xmlChildl, "motif_destination", strTmp, pChargement);
                        CMotif * pMotifDestination= GetMotifFromID(strTmp);

                        double dbCoeff;
                        GetXmlAttributeValue(xmlChildl, "coeff", dbCoeff, pChargement);

                        dbSum += dbCoeff;

                        CMotifCoeff motifCoeff(pMotifOrigine, pMotifDestination, dbCoeff);

                        if (!repMotifDest.addMotifCoeff(motifCoeff))
                        {
                            *pChargement << Logger::Error << "ERROR : the motives couple " << motifCoeff.GetMotifOrigine()->GetID() << " -> " << motifCoeff.GetMotifDestination()->GetID() << " has a duplicate for origin " << pOrigine->GetOutputID() << std::endl;
                            *pChargement << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                            return false;
                        }
                    }

                    // VÃ©rification de la somme des coefficients

                    // rmq. : l'Ã©cart sur la somme peut dÃ©passer epsilon (la perte de precision se cumule)
                    double dbTolerance = std::numeric_limits<double>::epsilon() * countl;

                    // Pour pouvoir faire fonctionner les anciens fichiers invalides sans effort, on normalise automatiquement
                    // les coefficients si la somme est proche de 1 au sens de l'ancienne tolÃ©rance (0.0001)
                    if (fabs(dbSum - 1.0) > dbTolerance && fabs(dbSum - 1.0) < 0.0001)
                    {
                        // *pChargement << Logger::Warning << "WARNING : la somme des coefficents pour l'ensemble des couples de motifs origine destination de l'entrÃ©e " << pOrigine->GetOutputID() << " - variation " << j << " de l'Ã©lÃ©ment REP_MOTIF - " << " est proche mais diffÃ©rente de 1 : normalisation automatique des coefficients..." << std::endl;
                        // *pChargement << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                        // Normalisation automatique des coefficients
                        for (size_t iOD = 0; iOD < repMotifDest.getCoeffs().size(); iOD++)
                        {
                            CMotifCoeff & motifCoeff = repMotifDest.getCoeffs()[iOD];
                            motifCoeff.setCoeff(motifCoeff.getCoeff() / dbSum);
                        }
                        dbSum = 1.0;
                    }

                    if (fabs(dbSum - 1) > dbTolerance)
                    {
                        *pChargement << Logger::Error << "ERROR : the coefficents sum for all origin-destination motive couples of input " << pOrigine->GetOutputID() << " - variation " << j << " of element REP_MOTIF - " << " must be equal to 1" << std::endl;
                        *pChargement << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                        return false;
                    }

                    if (!pRepMotif->addRepMotifDest(repMotifDest))
                    {
                        *pChargement << Logger::Error << "ERROR : destination " << repMotifDest.getDestination()->GetID() << " is duplicated for origin " << pOrigine->GetOutputID() << std::endl;
                        *pChargement << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                        return false;
                    }
                }



                if (plages.size() > 0)
                {
                    for (size_t iPlage = 0; iPlage < plages.size(); iPlage++)
                    {
                        AddVariation<>(plages[iPlage], pRepMotif, &pOrigine->GetLstRepMotifInit(pTypeVehicle));
                    }
                }
                else
                {
                    std::deque<TimeVariation<CRepMotif>>& lstTVRepMotif = pOrigine->GetLstRepMotifInit(pTypeVehicle);
                    double dbStartPeriodTime = 0;
                    for (size_t iPeriod = 0; iPeriod < lstTVRepMotif.size(); iPeriod++)
                    {
                        dbStartPeriodTime += lstTVRepMotif[iPeriod].m_dbPeriod;
                    }
                    AddVariation<>(dbDuree, pRepMotif, &lstTVRepMotif);
                }
            }

            // vÃ©rification de la couverture temporelle pour tous le type de vÃ©hicules
            {
                vector<PlageTemporelle*> plages;
                for (size_t iPlage = 0; iPlage < pOrigine->GetLstRepMotifInit(pTypeVehicle).size(); iPlage++)
                {
                    PlageTemporelle * pPlage = pOrigine->GetLstRepMotifInit(pTypeVehicle)[iPlage].m_pPlage;
                    if (pPlage)
                    {
                        plages.push_back(pPlage);
                    }
                }
                if (plages.size() > 0 && !CheckPlagesTemporellesEx(m_dbDureeSimu, plages))
                {
                    *pChargement << Logger::Error << "ERROR : The time frames defined for the REP_MOTIF node for input " << pOrigine->GetOutputID();
                    if (pTypeVehicle)
                    {
                        *pChargement << " for vehicle type " << pTypeVehicle->GetLabel();
                    }
                    *pChargement << " dont cover the whole simulation duration !" << std::endl;
                    *pChargement << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                    return false;
                }
            }

        }
        // fin traitement REP_MOTIFS
    }

    return true;
}

int Reseau::SetRoutes(const std::string & originId, const std::string & destinationId, const std::string & typeVeh, std::vector<std::pair<double, std::vector<string> > > & routes)
{
    int result = 0;

    SymuViaTripNode *pOrigine;
    SymuViaTripNode *pDestination;
    TypeVehicule * pTypeVeh;
    char cTmp;

    // VÃ©rifications
    pOrigine = GetOrigineFromID(originId, cTmp);
    if(!pOrigine || cTmp != 'E')
        return -2;
    char cCnx;
    if(GetConnectionFromID(originId, cCnx))
        return -3;

    pDestination = GetDestinationFromID(destinationId, cTmp);
    if(!pDestination)
        return -4;
    if(GetConnectionFromID(destinationId, cCnx))
        return -5;

    if(typeVeh.empty())
    {
        pTypeVeh = NULL;
    }
    else
    {
        pTypeVeh = GetVehicleTypeFromID(typeVeh);
        if(!pTypeVeh)
        {
            return -6;
        }
    }

    // Test de la validitÃ© des coefficients
    double dbItiCoeffSum = 0.0;
    for(size_t iIti = 0; iIti < routes.size(); iIti++)
    {
        dbItiCoeffSum += routes[iIti].first;
    }
    double dbTolerance = routes.size() * std::numeric_limits<double>::epsilon();
    if(fabs(dbItiCoeffSum-1.0) > dbTolerance && fabs(dbItiCoeffSum-1.0) < 0.0001)
    {
        // code de retour positif mais non nul : permet de gÃ©rer le cas du warning cÃ´tÃ© client.
        result = 1;
        //Chargement<<"WARNING : la somme des coefficents pour l'ensemble des itinÃ©raires dÃ©finis pour l'entrÃ©e " << pOrigine->GetOutputID() << " et la sortie " << pDestination->GetDestinationID() << " - variation " << j <<" de l'Ã©lÃ©ment REP_DESTINATION - " << " est proche mais diffÃ©rente de 1 : normalisation automatique des coefficients..." << std::endl;

        // Normalisation automatique des coefficients
        for(size_t itiIdx = 0; itiIdx < routes.size(); itiIdx++)
        {
            routes[itiIdx].first /= dbItiCoeffSum;
        }
        dbItiCoeffSum = 1.0;
    }
    if( fabs(dbItiCoeffSum-1.0) > dbTolerance)
    {
        return -7;
        //Chargement<<"ERREUR : la somme des coefficents pour l'ensemble des itinÃ©raires dÃ©finis pour l'entrÃ©e " << pOrigine->GetOutputID() << " et la sortie " << pDestination->GetDestinationID() << " - variation " << j <<" de l'Ã©lÃ©ment REP_DESTINATION - " << " doit Ãªtre Ã©gale Ã  1 " << std::endl;
    }

    std::vector<std::pair<double, std::pair<std::vector<Tuyau*>, Connexion*> > > newRoutes;
    for(size_t iIti = 0; iIti < routes.size(); iIti++)
    {
        const std::pair<double, std::vector<string> > & tmpRoute = routes[iIti];

        if(tmpRoute.second.size() == 0)
        {
            return -8; // The route is empty
        }

        std::pair<double, std::pair<std::vector<Tuyau*>, Connexion*> > newRoute;
        newRoute.second.second = NULL;
        Tuyau * pUpT = NULL;
        for(size_t iTuy = 0; iTuy < tmpRoute.second.size(); iTuy++)
        {
            Tuyau * pDownT = GetLinkFromLabel(tmpRoute.second[iTuy]);

            if( !pDownT )
                return -9;	// This link does not belong to the simulated network

            if( pUpT )
                if( !pDownT->GetCnxAssAm()->IsTuyauAmont(pUpT) )
                    return -10;	// The route is wrong

            newRoute.second.first.push_back(pDownT);
            pUpT = pDownT;
        }
        newRoutes.push_back(newRoute);
    }

    pOrigine->SetLstCoeffDestInit(pDestination, pTypeVeh, newRoutes);

    return result;
}

/// <summary>
/// Alternative paths calculation
/// Les extrÃ©mitÃ©s sont identifiÃ©es par des ID: elles peuvent Ãªtre des connexions, des briques de connexion ou des tronÃ§ons
/// The ends of path are identified by ID, their types could be connections, 2D-connections, zones or links
/// </summary>
///<param name="originId">ID of origin</param>
///<param name="destinationId">ID of destination</param>
///<param name="typeVeh">Vehicle type ID</param>
///<param name="dbInstant">Simulation time of calculation</param>
///<param name="nbShortestPaths">Number of alternate shortest paths to calculate</param>
///<param name="method">Calculation method</param>
///<param name="refPath">Reference path: the calculated alternate shortest paths must be different from the reference path</param>
///<param name="linksToAvoid">List of links to avoid</param>
///<param name="paths">Calculated alternate shortest paths</param>
///<param name="refPathCost">Calculated cost of the reference path</param>
///<returns>Code error</returns>
int Reseau::ComputeRoutes(const std::string & originId, const std::string & destinationId, const std::string & typeVeh, double dbInstant, int nbShortestPaths,
    int method, const std::vector<Tuyau*> & refPath, const std::vector<Tuyau*> & linksToAvoid, std::vector<std::pair<double, std::vector<std::string> > > & paths, double & refPathCost)
{
    int result = 0;

    refPathCost = 0;

    Connexion *pOrigine;
	Connexion *pDestination;

    TypeVehicule * pTypeVeh;
    char cTmp;
    TronconOrigine *pTO = NULL;	

    // Check
    pOrigine = GetConnectionFromID(originId, cTmp);
    if(!pOrigine)
    {
        pOrigine = GetBrickFromID(originId);
        if(!pOrigine)
        {
            SymuViaTripNode * pTmp = GetOrigineFromID(originId, cTmp);
            if(pTmp)
            {
                pOrigine = pTmp->GetOutputConnexion();
            }
        }
    }
    if(!pOrigine)
    {
        // Is it a link ?
        Tuyau* pT = GetLinkFromLabel(originId);

        if(pT)
            pTO = new TronconOrigine(pT, NULL);

        if(!pTO)
            return -2;
    }

    pDestination = GetConnectionFromID(destinationId, cTmp);

	if (!pDestination)
	{ 
		pDestination = GetBrickFromID(destinationId);
		if (!pDestination)
		{
			SymuViaTripNode * pTmp = GetDestinationFromID(destinationId, cTmp);
			if (pTmp)
			{
				pDestination = pTmp->GetInputConnexion();
			}
		}
		if (!pDestination)
		{
			// Is it a link ?
			Tuyau* pT = GetLinkFromLabel(destinationId);

			if (pT)
				pDestination = pT->GetCnxAssAv();
			else
			{ 
				if (pTO)
				{
					delete pTO;
				}
				return -3;
			}
		}
	}

    pTypeVeh = GetVehicleTypeFromID(typeVeh);
    if(!pTypeVeh)
    {
        if (pTO)
        {
            delete pTO;
        }
		
        return -6;
    }

    std::vector<PathResult> computedPaths;
    std::map<std::vector<Tuyau*>, double> MapFilteredItis;

    if(pTO)
    {		
		m_pSymuScriptManager->ComputePaths(pTO, pDestination, pTypeVeh, dbInstant, nbShortestPaths, method, refPath, linksToAvoid, computedPaths, MapFilteredItis, false);
		delete pTO;
    }
    else
        m_pSymuScriptManager->ComputePaths(pOrigine, pDestination, pTypeVeh, dbInstant, nbShortestPaths, NULL, method, refPath, linksToAvoid, computedPaths, MapFilteredItis, false);

    for(size_t i = 0; i < computedPaths.size(); i++)
    {
        const PathResult & myPath = computedPaths[i];
        std::vector<std::string> lstTuyaux(myPath.links.size());
        for(size_t iLink = 0; iLink < lstTuyaux.size(); iLink++)
        {
            lstTuyaux[iLink] = myPath.links[iLink]->GetLabel();
        }
        paths.push_back(std::make_pair(myPath.dbCost, lstTuyaux));
    }

    if (!refPath.empty())
    {
        refPathCost = m_pSymuScriptManager->ComputeCost(pTypeVeh, refPath, false);
    }

    return result;
}

int Reseau::ComputeMixedRoutes(const std::string & originId, const std::string & destinationId, const std::string & typeVeh, double dbInstant, int nbShortestPaths, std::map<Parking*, std::vector<std::vector<string> > >  & paths)
{
    int result = 0;

    Connexion *pOrigine;
    SymuViaTripNode *pDestination;
    TypeVehicule * pTypeVeh;
    char cTmp;
    TronconOrigine *pTO = NULL;

    // Check
    pOrigine = GetConnectionFromID(originId, cTmp);
    if (!pOrigine)
    {
        pOrigine = GetBrickFromID(originId);
        if (!pOrigine)
        {
            SymuViaTripNode * pTmp = GetOrigineFromID(originId, cTmp);
            if (pTmp)
            {
                pOrigine = pTmp->GetOutputConnexion();
            }
        }
    }
    if (!pOrigine)
    {
        // Is it a link ?
        Tuyau* pT = GetLinkFromLabel(originId);

        if (pT)
            pTO = new TronconOrigine(pT, NULL);

        if (!pTO)
            return -2;
    }
    pDestination = GetDestinationFromID(destinationId, cTmp);
    if (!pDestination)
    {
        if (pTO)
        {
            delete pTO;
        }
        return -3;
    }


    pTypeVeh = GetVehicleTypeFromID(typeVeh);
    if (!pTypeVeh)
    {
        if (pTO)
        {
            delete pTO;
        }
        return -6;
    }

    std::vector<Tuyau*> refPath, linksToAvoid;
    std::map<Parking*, std::vector<std::pair<double, std::vector<std::string> > > > foundPaths;
    std::map<Parking*, double> mapVLCosts;

    // Recuperation des parkings qui permettent de rallier la destination (du coup il faut les stocker avant...)
    std::map<TripNode*, std::set<Parking*> >::const_iterator iterParkings = m_mapAvailableParkingsForDestination.find(pDestination);
    if (iterParkings != m_mapAvailableParkingsForDestination.end())
    {
        const std::set<Parking*> & candidateParkings = iterParkings->second;
        for (std::set<Parking*>::const_iterator iterParking = candidateParkings.begin(); iterParking != candidateParkings.end(); ++iterParking)
        {
            // Calcul du plus court chemin en voiture vers le parking depuis l'origine :
            std::vector<PathResult> computedPaths;
            std::map<std::vector<Tuyau*>, double> MapFilteredItis;

            if (pTO)
                m_pSymuScriptManager->ComputePaths(pTO, *iterParking, pTypeVeh, dbInstant, 1, 0, refPath, linksToAvoid, computedPaths, MapFilteredItis, false);
            else
                m_pSymuScriptManager->ComputePaths(pOrigine, *iterParking, pTypeVeh, dbInstant, 1, NULL, 0, refPath, linksToAvoid, computedPaths, MapFilteredItis, false);

            if (computedPaths.size() == 1)
            {
                // Le parking est accessible depuis l'origine:
                mapVLCosts[*iterParking] = computedPaths.front().dbCost;

                // Calcul des N plus courts chemins depuis le parking vers la destination finale en transports en commun :
                Connexion * pOutputConnexion = (*iterParking)->GetOutputConnexion();
                if (!pOutputConnexion)
                {
                    // Si ce n'est qu'un parking sans sortie, on prend pour connexion origine du calcul d'itinÃ©raire le noeud amont
                    // de l'entrÃ©e du parking...
                    pOutputConnexion = (*iterParking)->GetInputConnexion()->m_LstTuyAssAm.front()->GetCnxAssAm();
                }
                std::vector<TripNode*> dest(1, pDestination);
                std::vector<std::vector<std::pair<double, std::vector<std::string> > > > pathsTmp;
                m_pSymuScriptManager->ComputePathsPublicTransport(pOutputConnexion, dest, dbInstant, nbShortestPaths, pathsTmp);
                if (!pathsTmp.empty())
                {
                    foundPaths[*iterParking] = pathsTmp.front();
                }
            }
        }
    }

    // On ne conserve que les N meilleurs chemins quelque soit le parking :
    std::multimap<double, std::pair<Parking*, std::vector<std::string> > > orderedPaths;
    std::map<Parking*, std::vector<std::pair<double, std::vector<std::string> > > >::const_iterator iterFoundPath;
    for (iterFoundPath = foundPaths.begin(); iterFoundPath != foundPaths.end(); ++iterFoundPath)
    {
        double dbVLCost = mapVLCosts.at(iterFoundPath->first);
        for (size_t iPath = 0; iPath < iterFoundPath->second.size(); iPath++)
        {
            const std::pair<double, std::vector<std::string> > & foundPath = iterFoundPath->second.at(iPath);
            double dbFullCost = dbVLCost + foundPath.first;
            orderedPaths.insert(std::make_pair(dbFullCost, std::make_pair(iterFoundPath->first, foundPath.second)));
        }
    }

    std::multimap<double, std::pair<Parking*, std::vector<std::string> > >::const_iterator iterOrderedPaths;
    int nbSelectedPaths = 0;
    for (iterOrderedPaths = orderedPaths.begin(); iterOrderedPaths != orderedPaths.end() && nbSelectedPaths < nbShortestPaths; ++iterOrderedPaths)
    {
        paths[iterOrderedPaths->second.first].push_back(iterOrderedPaths->second.second);
        nbSelectedPaths++;
    }

    if (pTO)
    {
        delete pTO;
    }

    return result;
}


int Reseau::ComputeRoutesPublicTransport(const std::string & originId, const std::string & currentLine, const std::string & nextLine, const std::string & destinationId,
    double dbInstant, int nbShortestPaths, std::vector<std::pair<double, std::vector<std::string> > > &paths)
{
    int result = 0;

    Connexion *pOrigine;
    Arret * pStopOrigin = NULL;
    SymuViaTripNode *pDestination;

    PublicTransportLine * pCurrentLine = NULL;
    PublicTransportLine * pNextLine = NULL;

    char cTmp;

    // Check

    if (!currentLine.empty())
    {
        pCurrentLine = (PublicTransportLine*)GetPublicTransportFleet()->GetTrip(currentLine);
        if (!pCurrentLine)
        {
            return -4;
        }
    }

    if (!nextLine.empty())
    {
        pNextLine = (PublicTransportLine*)GetPublicTransportFleet()->GetTrip(nextLine);
        if (!pNextLine)
        {
            return -5;
        }
    }

    pOrigine = GetConnectionFromID(originId, cTmp);
    if (!pOrigine)
    {
        pOrigine = GetBrickFromID(originId);
        if (!pOrigine)
        {
            SymuViaTripNode * pTmp = GetOrigineFromID(originId, cTmp);
            if (pTmp)
            {
                pOrigine = pTmp->GetOutputConnexion();
            }
            if (!pOrigine)
            {
                // Cas d'une demande d'alternative depuis un arrÃªt de bus :
                if (pCurrentLine)
                {
                    pStopOrigin = dynamic_cast<Arret*>(pCurrentLine->GetTripNode(originId));
                }
                else if (pNextLine)
                {
                    pStopOrigin = dynamic_cast<Arret*>(pNextLine->GetTripNode(originId));
                }
                else
                {
                    // Cas particulier d'un parking sans connection qui en ressort (mais dont l'usager peut partir Ã  pieds), cf. mÃªme chose
                    // dans GetParkingInfos ! :

                    // Si ce n'est qu'un parking sans sortie (qui n'a donc pas pu Ãªtre renvoyÃ© par GetOrigineFromID), on prend pour connexion origine du calcul d'itinÃ©raire le noeud amont
                    // de l'entrÃ©e du parking...
                    Parking * pParking = GetParkingFromID(originId);
                    if (pParking && pParking->GetInputConnexion())
                    {
                        pOrigine = pParking->GetInputConnexion()->m_LstTuyAssAm.front()->GetCnxAssAm();
                    }
                }
            }
        }
    }
    if (!pOrigine && !pStopOrigin)
    {
        return -2;
    }
    pDestination = GetDestinationFromID(destinationId, cTmp);
    if (!pDestination)
    {
        return -3;
    }

    if (pOrigine)
    {
        m_pSymuScriptManager->ComputePathsPublicTransport(pOrigine, pDestination, dbInstant, nbShortestPaths, paths);
    }
    else
    {
        m_pSymuScriptManager->ComputePathsPublicTransport(pStopOrigin, pCurrentLine, pNextLine, pDestination, dbInstant, nbShortestPaths, paths);
    }

    return result;
}

bool Reseau::GetPredefinedAlternativeRoutes(TypeVehicule * pTypeVehicule, Tuyau * pDownLink, SymuViaTripNode * pDestinationNode, Tuyau * pDestinationLink, Connexion * pDestinationCon,
    std::vector<std::pair<double, std::vector<std::string> > > & paths)
{
    if (!m_bUseMapRouteFromNodes)
        return false;

    Connexion * pOriginCon = pDownLink->GetCnxAssAm();

    std::vector<std::vector<Tuyau *> *> *pPredefPaths = NULL;
    if (pDestinationNode)
    {
        std::map<Connexion*, std::map<SymuViaTripNode*, std::vector<std::vector<Tuyau *> *> > >::iterator iterOrigin = m_mapRouteFromNodes.find(pOriginCon);
        if (iterOrigin != m_mapRouteFromNodes.end())
        {
            std::map<SymuViaTripNode*, std::vector<std::vector<Tuyau *> *> >::iterator iterDest = iterOrigin->second.find(pDestinationNode);
            if (iterDest != iterOrigin->second.end())
            {
                pPredefPaths = &iterDest->second;
            }
        }
    }
    else if (pDestinationLink)
    {
        std::map<Connexion*, std::map<Tuyau*, std::vector<std::vector<Tuyau *> *> > >::iterator iterOrigin = m_mapRouteFromNodesToLink.find(pOriginCon);
        if (iterOrigin != m_mapRouteFromNodesToLink.end())
        {
            std::map<Tuyau*, std::vector<std::vector<Tuyau *> *> >::iterator iterDest = iterOrigin->second.find(pDestinationLink);
            if (iterDest != iterOrigin->second.end())
            {
                pPredefPaths = &iterDest->second;
            }
        }
    }
    else if (pDestinationCon)
    {
        std::map<Connexion*, std::map<Connexion*, std::vector<std::vector<Tuyau *> *> > >::iterator iterOrigin = m_mapRouteFromNodesToInternalNode.find(pOriginCon);
        if (iterOrigin != m_mapRouteFromNodesToInternalNode.end())
        {
            std::map<Connexion*, std::vector<std::vector<Tuyau *> *> >::iterator iterDest = iterOrigin->second.find(pDestinationCon);
            if (iterDest != iterOrigin->second.end())
            {
                pPredefPaths = &iterDest->second;
            }
        }
    }

    if (pPredefPaths)
    {
        for (size_t iPath = 0; iPath < pPredefPaths->size(); iPath++)
        {
            const vector<Tuyau*> * pathLinks = pPredefPaths->at(iPath);
            if (!pDownLink || (pathLinks->size() > 0 && pathLinks->at(0) == pDownLink))
            {
                double dbPathCost = GetSymuScript()->ComputeCost(pTypeVehicule, *pathLinks, false);
                if (dbPathCost < DBL_MAX)
                {
                    double dbPathCost = GetSymuScript()->ComputeCost(pTypeVehicule, *pathLinks, false);
                    if (dbPathCost < DBL_MAX)
                    {
                        std::vector<std::string> linkNames;
                        for (size_t iLink = 0; iLink < pathLinks->size(); iLink++)
                        {
                            linkNames.push_back(pathLinks->at(iLink)->GetLabel());
                        }
                        paths.push_back(std::make_pair(dbPathCost, linkNames));
                    }
                }
            }
        }
    }

    return true;
}

bool Reseau::WriteTempusFiles()
{
    SQLNetworkExporter sqlExporter;
    bool bOk = sqlExporter.writeSQL(this);

    GTFSExporter gtfsExporter;
    bOk = gtfsExporter.write(this, sqlExporter.getTPTypes()) && bOk;

    return bOk;
}

bool Reseau::GetSectionFromNode(const std::string & strID, DOMNode * pXMLNode, Tuyau * pTuyau, double & dbTmp, double & dbTmp2, Logger* pLogger)
{
    GetXmlAttributeValue(pXMLNode, "position_debut", dbTmp, pLogger);
    GetXmlAttributeValue(pXMLNode, "position_fin", dbTmp2, pLogger);

    bool bTmp = false;
    GetXmlAttributeValue(pXMLNode, "position_relative", bTmp, pLogger);
    if (bTmp)
    {
        dbTmp *= pTuyau->GetLength();
        dbTmp2 *= pTuyau->GetLength();
    }

    dbTmp2 = dbTmp2 == numeric_limits<double>::infinity() ? pTuyau->GetLength() : dbTmp2;

    // VÃ©rification de la position
    if (pTuyau->GetLength() < dbTmp)
    {
        *pLogger << Logger::Error << " ERROR : invalid start position of element " << strID << " ( position_debut > link's length ) : element ignored..." << endl;
        *pLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
    }
    else if (dbTmp2 <= dbTmp)
    {
        *pLogger << Logger::Error << " ERROR : invalid end position of element " << strID << " ( position_fin <= position_debut ) : element ignored..." << endl;
        *pLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
    }
    else
    {
        if (pTuyau->GetLength() < dbTmp2)
        {
            // Pour ne pas spammer pour les plaques, on ne met le warning que si l'erreur est supÃ©rieur Ã  10cm :
            if (pTuyau->GetLength() + 0.1 < dbTmp2)
            {
                *pLogger << Logger::Warning << " WARNING : end position of element " << strID << " greater than the link's length : the end of element is pulled back to the end of the link..." << endl;
                *pLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
            }
            dbTmp2 = pTuyau->GetLength();
        }
        return true;
    }
    return false;
}

void Reseau::InvalidatePathRelatedCaches()
{
    for (size_t iZone = 0; iZone < Liste_zones.size(); iZone++)
    {
        Liste_zones[iZone]->ClearPathCaches();
    }
}

#ifdef USE_SYMUCORE
boost::posix_time::ptime Reseau::GetSimulationStartTime()
{
    const STime & simuStartTime = GetSimuStartTime();
    const SDateTime & simuDate = GetDateSimulation();

    // Cas particulier si pas de date dÃ©finie : on prend 01/01/1970 Ã  00h00m00s
    boost::gregorian::date startDate;
    if (simuDate.m_annee == 0)
    {
        startDate = boost::gregorian::date(1970, 1, 1);
    }
    else
    {
        startDate = boost::gregorian::date(simuDate.m_annee, simuDate.m_mois, simuDate.m_jour);
    }

    return boost::posix_time::ptime(
        startDate,
        boost::posix_time::time_duration(simuStartTime.m_hour, simuStartTime.m_minute, simuStartTime.m_second)
        );
}
bool Reseau::BuildGraph(SymuCore::MultiLayersGraph * pGraph, bool bIsPrimaryGraph)
{
    if (!m_LstTypesVehicule.empty())
    {
        TypeVehicule * pMainType = m_LstTypesVehicule.front();
        for (size_t iMacroType = 0; iMacroType < pGraph->GetListMacroTypes().size(); iMacroType++)
        {
            if (pGraph->GetListMacroTypes().at(iMacroType)->hasVehicleType(pMainType->GetLabel()))
            {
                m_pVLMacroType = pGraph->GetListMacroTypes().at(iMacroType);
                break;
            }
        }
    }
    else
    {
        m_pVLMacroType = NULL;
    }
    if (m_pGraphBuilder)
    {
        delete m_pGraphBuilder;
    }
    m_pGraphBuilder = new SymuCoreGraphBuilder(this, true);
    return m_pGraphBuilder->Build(pGraph, bIsPrimaryGraph);
}
bool Reseau::FillPopulations(SymuCore::MultiLayersGraph * pGraph, SymuCore::Populations & populations,
    const boost::posix_time::ptime & startSimulationTime, const boost::posix_time::ptime & endSimulationTime, bool bIgnoreSubAreas,
    std::vector<SymuCore::Trip> & lstTrips)
{
    SymuCoreDrivingTripsBuilder carTripsBuilder(this);
    return carTripsBuilder.FillPopulations(pGraph, populations, startSimulationTime, endSimulationTime, bIgnoreSubAreas, lstTrips);
}

bool Reseau::FillShortestPathParameters(bool bFillKParameters, bool bUseCommonalityFilter, double& dbAssignementAlpha, double& dbAssignementBeta, double& dbAssignementGamma,
                                        std::vector<double>& dbCommonalityFactorParameters, double& dbWardropTolerance,
                                        std::map<SymuCore::SubPopulation*, std::map<SymuCore::Origin*, std::map<SymuCore::Destination*, SymuCore::ListTimeFrame<double> > > >& KByOD)
{
    dbWardropTolerance = dbWardropTolerance == -1 ? GetWardropTolerance() : dbWardropTolerance;
    dbAssignementAlpha = dbAssignementAlpha == -1 ? GetDijkstraAlpha() : dbAssignementAlpha;
    dbAssignementBeta = dbAssignementBeta == -1 ? GetDijkstraBeta() : dbAssignementBeta;
    dbAssignementGamma = dbAssignementGamma == -1 ? GetDijkstraGamma() : dbAssignementGamma;
    if (bUseCommonalityFilter)
    {
        dbCommonalityFactorParameters[0] = dbCommonalityFactorParameters[0] == -1 ? GetCommonalityAlpha() : dbCommonalityFactorParameters[0];
        dbCommonalityFactorParameters[1] = dbCommonalityFactorParameters[1] == -1 ? GetCommonalityBeta() : dbCommonalityFactorParameters[1];
        dbCommonalityFactorParameters[2] = dbCommonalityFactorParameters[2] == -1 ? GetCommonalityGamma() : dbCommonalityFactorParameters[2];
    }

    TypeVehicule * pTypeVeh;

    // DÃ©finition des paramÃ¨tres par OD
    double startSimu = 0.0, endSimu = GetDureeSimu();
    double dbStartTime, dbEndTime;
    bool isOriginFound;
    bool isDestinationFound;

    std::map<SymuCore::SubPopulation*, std::map<SymuCore::Origin*, std::map<SymuCore::Destination*, SymuCore::ListTimeFrame<double> > > >::iterator itMapAtSubPopulation;
    for(itMapAtSubPopulation = KByOD.begin(); itMapAtSubPopulation != KByOD.end(); itMapAtSubPopulation++)
    {
        SymuCore::SubPopulation* pSubPopulation = itMapAtSubPopulation->first;
        SymuCore::MacroType* pMacroType = pSubPopulation->GetPopulation()->GetMacroType();
        pTypeVeh = NULL;
        if (pMacroType)
            pTypeVeh = GetVehiculeTypeFromMacro(pMacroType);

        std::map<SymuCore::Origin*, std::map<SymuCore::Destination*, SymuCore::ListTimeFrame<double> > >::iterator itMapAtOrigin;
        for(itMapAtOrigin = itMapAtSubPopulation->second.begin(); itMapAtOrigin != itMapAtSubPopulation->second.end(); itMapAtOrigin++)
        {
            SymuCore::Origin* pSymuCoreOrigin = itMapAtOrigin->first;
            isOriginFound = false;

            for(size_t iOrigin = 0; iOrigin < Liste_origines.size() && bFillKParameters; iOrigin++)
            {
                if (!pTypeVeh) //fill by default value
                    break;

                SymuViaTripNode* pSymuViaOrigin = Liste_origines[iOrigin];
                //pNode = GetNodeFromOrigin(pOrigin, pTypeVeh);
                if(pSymuViaOrigin->GetID() == pSymuCoreOrigin->getStrNodeName())
                {
                    const std::deque<TimeVariation<SimMatOrigDest> > & lstCoeffDest = IsInitSimuTrafic() ? pSymuViaOrigin->GetLstCoeffDest(pTypeVeh) : pSymuViaOrigin->GetLstCoeffDestInit(pTypeVeh);

                    if (lstCoeffDest.size() != 0)
                        isOriginFound = true;

                    // Pour chaque variante temporelle ...
                    dbStartTime = 0;
                    for(size_t iVar = 0; iVar < lstCoeffDest.size(); iVar++)
                    {
                        const TimeVariation<SimMatOrigDest> & coeffDestVar = lstCoeffDest[iVar];
                        if(coeffDestVar.m_pData)
                        {
                            const std::deque<VectDest*> & coeffDest = coeffDestVar.m_pData->MatOrigDest;

                            // dÃ©finition de la plage temporelle correspondante au format SymuScript
                            if(coeffDestVar.m_pPlage)
                            {
                                dbStartTime = coeffDestVar.m_pPlage->m_Debut;
                                dbEndTime = coeffDestVar.m_pPlage->m_Fin;
                            }
                            else
                            {
                                dbEndTime = dbStartTime + coeffDestVar.m_dbPeriod;

                            }

                            // Pour chaque destination ...
                            std::map<SymuCore::Destination*, SymuCore::ListTimeFrame<double> >::iterator itMapAtDestination;
                            for(itMapAtDestination = itMapAtOrigin->second.begin(); itMapAtDestination != itMapAtOrigin->second.end(); itMapAtDestination++)
                            {
                                SymuCore::Destination* pSymuCoreDestination = itMapAtDestination->first;
                                SymuCore::ListTimeFrame<double>& timeFrame = itMapAtDestination->second;
                                isDestinationFound = false;

                                for(size_t iDest = 0; iDest < coeffDest.size(); iDest++)
                                {
                                    VectDest * pVectDest = coeffDest[iDest];
                                    if(pVectDest->pDest->GetID() == pSymuCoreDestination->getStrNodeName())
                                    {
                                         if(pVectDest->bHasNbPlusCourtsChemins)
                                         {
                                             isDestinationFound = true;
                                             timeFrame.addTimeFrame(dbStartTime, dbEndTime, boost::make_shared<double>(pVectDest->nNbPlusCourtsChemins));
                                         }
                                    }
                                }
                                if(!isDestinationFound)
                                {
                                    if(timeFrame.size() == 0)
                                        timeFrame.addTimeFrame(startSimu, endSimu, boost::make_shared<double>(m_nNbPlusCourtChemin));
                                }
                            }

                            dbStartTime = dbEndTime;
                        }
                    }
                }
            }//end for each origin
            if(!isOriginFound)
            {
                std::map<SymuCore::Destination*, SymuCore::ListTimeFrame<double> >::iterator itMapAtDestination;
                for(itMapAtDestination = itMapAtOrigin->second.begin(); itMapAtDestination != itMapAtOrigin->second.end(); itMapAtDestination++)
                {
                    itMapAtDestination->second.addTimeFrame(startSimu, endSimu, boost::make_shared<double>(m_nNbPlusCourtChemin));
                }
            }
        }
    }

    return true;
}

bool Reseau::FillLogitParameters(std::map<SymuCore::SubPopulation*, std::map<SymuCore::Origin*, std::map<SymuCore::Destination*, SymuCore::ListTimeFrame<double> > > >& LogitByOD)
{
    double DefaultTetaLogit = GetTetaLogit();
    TypeVehicule * pTypeVeh;

    // DÃ©finition des paramÃ¨tres par OD
    double startSimu = 0.0, endSimu = GetDureeSimu();
    double dbStartTime, dbEndTime;
    bool isOriginFound;
    bool isDestinationFound;

    std::map<SymuCore::SubPopulation*, std::map<SymuCore::Origin*, std::map<SymuCore::Destination*, SymuCore::ListTimeFrame<double> > > >::iterator itMapAtSubPopulation;
    for(itMapAtSubPopulation = LogitByOD.begin(); itMapAtSubPopulation != LogitByOD.end(); itMapAtSubPopulation++)
    {
        SymuCore::SubPopulation* pSubPopulation = itMapAtSubPopulation->first;
        SymuCore::MacroType* pMacroType = pSubPopulation->GetPopulation()->GetMacroType();
        pTypeVeh = NULL;
        if (pMacroType)
            pTypeVeh = GetVehiculeTypeFromMacro(pMacroType);

        std::map<SymuCore::Origin*, std::map<SymuCore::Destination*, SymuCore::ListTimeFrame<double> > >::iterator itMapAtOrigin;
        for(itMapAtOrigin = itMapAtSubPopulation->second.begin(); itMapAtOrigin != itMapAtSubPopulation->second.end(); itMapAtOrigin++)
        {
            SymuCore::Origin* pSymuCoreOrigin = itMapAtOrigin->first;
            isOriginFound = false;

            for(size_t iOrigin = 0; iOrigin < Liste_origines.size(); iOrigin++)
            {
                if (!pTypeVeh) //fill by default value
                    break;

                SymuViaTripNode* pSymuViaOrigin = Liste_origines[iOrigin];
                if(pSymuViaOrigin->GetID() == pSymuCoreOrigin->getStrNodeName())
                {
                    const std::deque<TimeVariation<SimMatOrigDest> > & lstCoeffDest = IsInitSimuTrafic() ? pSymuViaOrigin->GetLstCoeffDest(pTypeVeh) : pSymuViaOrigin->GetLstCoeffDestInit(pTypeVeh);

                    if (lstCoeffDest.size() != 0)
                        isOriginFound = true;

                    // Pour chaque variante temporelle ...
                    dbStartTime = 0;
                    for(size_t iVar = 0; iVar < lstCoeffDest.size(); iVar++)
                    {
                        const TimeVariation<SimMatOrigDest> & coeffDestVar = lstCoeffDest[iVar];
                        if(coeffDestVar.m_pData)
                        {
                            const std::deque<VectDest*> & coeffDest = coeffDestVar.m_pData->MatOrigDest;

                            // dÃ©finition de la plage temporelle correspondante au format SymuScript
                            if(coeffDestVar.m_pPlage)
                            {
                                dbStartTime = coeffDestVar.m_pPlage->m_Debut;
                                dbEndTime = coeffDestVar.m_pPlage->m_Fin;
                            }
                            else
                            {
                                dbEndTime = dbStartTime + coeffDestVar.m_dbPeriod;

                            }

                            // Pour chaque destination ...
                            std::map<SymuCore::Destination*, SymuCore::ListTimeFrame<double> >::iterator itMapAtDestination;
                            for(itMapAtDestination = itMapAtOrigin->second.begin(); itMapAtDestination != itMapAtOrigin->second.end(); itMapAtDestination++)
                            {
                                SymuCore::Destination* pSymuCoreDestination = itMapAtDestination->first;
                                SymuCore::ListTimeFrame<double>& timeFrame = itMapAtDestination->second;
                                isDestinationFound = false;

                                for(size_t iDest = 0; iDest < coeffDest.size(); iDest++)
                                {
                                    VectDest * pVectDest = coeffDest[iDest];
                                    if(pVectDest->pDest->GetID() == pSymuCoreDestination->getStrNodeName())
                                    {
                                         if(pVectDest->bHasTetaLogit)
                                         {
                                             isDestinationFound = true;
                                             timeFrame.addTimeFrame(dbStartTime, dbEndTime, boost::make_shared<double>(pVectDest->dbTetaLogit));
                                         }
                                    }
                                }
                                if(!isDestinationFound)
                                {
                                    if(timeFrame.size() == 0)
                                        timeFrame.addTimeFrame(startSimu, endSimu, boost::make_shared<double>(DefaultTetaLogit));
                                }
                            }

                            dbStartTime = dbEndTime;
                        }
                    }
                }
            }//end for each origin
            if(!isOriginFound)
            {
                std::map<SymuCore::Destination*, SymuCore::ListTimeFrame<double> >::iterator itMapAtDestination;
                for(itMapAtDestination = itMapAtOrigin->second.begin(); itMapAtDestination != itMapAtOrigin->second.end(); itMapAtDestination++)
                {
                    itMapAtDestination->second.addTimeFrame(startSimu, endSimu, boost::make_shared<double>(DefaultTetaLogit));
                }
            }
        }
    }

    return true;
}

bool Reseau::ComputeCosts(const boost::posix_time::ptime & startTime, const boost::posix_time::ptime & endTime,
    const std::vector<SymuCore::MacroType*> & macroTypes, const std::map<SymuCore::SubPopulation*, SymuCore::CostFunction> & mapCostFunctions)
{
    double dbPeriodeDuration = (double)(endTime - startTime).total_microseconds() / 1000000.0;
    double dbEndPeriodInstant = (double)((endTime - GetSimulationStartTime()).total_microseconds()) / 1000000.0;

    for (size_t iZone = 0; iZone < Liste_zones.size(); iZone++)
    {
        ZoneDeTerminaison * pZone = Liste_zones[iZone];
        pZone->ComputeConcentration(dbPeriodeDuration);
        if (m_bUseSpatialTTMethod)
        {
            pZone->ComputeMeanSpeed(dbEndPeriodInstant, NULL, dbPeriodeDuration);
        }
    }

    // rmq. : dans le cas des fonction de cout actuelles (marginals et temps de parcours),on calcule les deux dans tous las cas
    // (cout en calculs nÃ©gligeable a priori et marginals trÃ¨s liÃ©s aux temps de parcours donc calculs imbriquÃ©s),
    // mais on pourra utiliser le paramÃ¨tre eCostFunction pour ne faire d'Ã©ventuels nouveaux calculs de cout que si nÃ©cessaire.
    std::deque<TuyauMicro*>::iterator itT;
    for (size_t iMacroType = 0; iMacroType < macroTypes.size(); iMacroType++)
    {
        SymuCore::MacroType * pMacroType = macroTypes[iMacroType];
        for (itT = m_LstTuyauxMicro.begin(); itT != m_LstTuyauxMicro.end(); ++itT)
        {
            if (!(*itT)->GetBriqueParente())
            {
                // Calcul du temps de parcours du tronÃ§on pour le macrotype
                (*itT)->CalculTempsDeParcours(dbEndPeriodInstant, pMacroType, dbPeriodeDuration);
            }
        }
        // Calcul des temps de parcours des mouvements des briques
        for (size_t i = 0; i < Liste_carrefoursAFeux.size(); i++)
        {
            Liste_carrefoursAFeux[i]->CalculTempsParcours(dbEndPeriodInstant, pMacroType, dbPeriodeDuration);
        }
        for (size_t i = 0; i < Liste_giratoires.size(); i++)
        {
            Liste_giratoires[i]->CalculTempsParcours(dbEndPeriodInstant, pMacroType, dbPeriodeDuration);
        }
        // Calcul des temps de parcours en zone
        for (size_t iZone = 0; iZone < Liste_zones.size(); iZone++)
        {
            ZoneDeTerminaison * pZone = Liste_zones[iZone];
            if (m_bUseSpatialTTMethod)
            {
                pZone->ComputeMeanSpeed(dbEndPeriodInstant, pMacroType, dbPeriodeDuration);
            }

            // Quelle que soit la mÃ©thode de calcul du temps de parcours, on appelle cette mÃ©thode basÃ©e sur les sorties de vÃ©hicules pour le calcul des marginals
            // (ou au moins nettoyer la map des vÃ©hicules utiles au calcul des marginals si on ne les utilise pas)
            pZone->CalculTempsParcours(dbEndPeriodInstant, pMacroType, dbPeriodeDuration);
        }
    }

    // Calculs pour la couche transports en commun :
    for (size_t iPublicTransportLine = 0; iPublicTransportLine < GetPublicTransportFleet()->GetTrips().size(); iPublicTransportLine++)
    {
        ((PublicTransportLine*)GetPublicTransportFleet()->GetTrips().at(iPublicTransportLine))->ComputeCosts();
    }

    // Initialisation de la nouvelle pÃ©riode de calcul des temps de parcours
    for (itT = m_LstTuyauxMicro.begin(); itT != m_LstTuyauxMicro.end(); ++itT)
    {
        if ((*itT)->m_pEdieSensor)
        {
            (*itT)->m_pEdieSensor->PrepareNextPeriod();
        }
    }
    for (size_t iZone = 0; iZone < Liste_zones.size(); iZone++)
    {
        ZoneDeTerminaison * pZone = Liste_zones[iZone];
        if (pZone->m_pMFDSensor)
        {
            pZone->m_pMFDSensor->PrepareNextPeriod();
        }
    }

    return true;
}

TypeVehicule *Reseau::GetVehiculeTypeFromMacro(SymuCore::MacroType *pMacroType)
{
    std::vector<SymuCore::VehicleType*> listVehType = pMacroType->getListVehicleTypes();
    TypeVehicule* pVehicle = NULL;
    for(size_t iVehType = 0; iVehType<listVehType.size(); iVehType++)
    {
        pVehicle = GetVehicleTypeFromID(listVehType[iVehType]->getStrName());
        if(pVehicle)
            break;
    }
    return pVehicle;
}

SymuCore::MacroType * Reseau::GetMacroTypeForVLs()
{
    return m_pVLMacroType;
}

void Reseau::SetUseSpatialTTMethod(bool bValue)
{
    m_bUseSpatialTTMethod = bValue;


    // Finalement on le fait tout le temps, car on en a besoin pour les marginals mÃªme en mode non spatial.
    //if (m_bUseSpatialTTMethod)
    {
        // Construction des diffÃ©rents capteurs d'Edie pour tous les tuyaux micro du rÃ©seau !!!!!
        for (std::deque<TuyauMicro*>::iterator itT = m_LstTuyauxMicro.begin(); itT != m_LstTuyauxMicro.end(); ++itT)
        {
            assert((*itT)->m_pEdieSensor == NULL);
            (*itT)->m_pEdieSensor = new EdieSensor(std::string(), (*itT), 0, (*itT)->GetLength(), EdieSensor::TT_Link, NULL);
            (*itT)->getLstRelatedSensorsBySensorsManagers()[NULL].push_back((*itT)->m_pEdieSensor);
        }

        // ainsi que des capteurs d'Edie pour les vitesses moyennes en zone
        for (size_t iZone = 0; iZone < Liste_zones.size(); iZone++)
        {
            ZoneDeTerminaison * pZone = Liste_zones[iZone];
            assert(pZone->m_pMFDSensor == NULL);

            std::vector<Tuyau*> mfdLinks;
            std::vector<double> endPositions;
            std::vector<double> startPositions;
            std::vector<int> linkTypes;

            PrepareLinksForMFD(pZone->GetInputPosition().GetZone(), GetLogger(), "SymuMaster", mfdLinks, startPositions, endPositions, linkTypes);
            pZone->m_pMFDSensor = new MFDSensor(this, NULL, 0, "SymuMaster", mfdLinks, startPositions, endPositions, linkTypes, true, true);
        }
    }
}

double Reseau::ComputeOffer(SymuViaTripNode * pInput, Tuyau * pLink)
{
    // Remarque : on n'utilise pas les mÃ©thodes type GetFirstVehicle et consors, qui sont optimisÃ©es pour de nombreux appels mais ne fonctionnent
    // que pour le dÃ©but du pas de temps (cf. GetLstVehiculesAnt() ), hors on travaille ici aprÃ¨s le calcul d'un pas de temps.

    double dbAllLanesOffer = 0;
    TypeVehicule * pMainType = m_LstTypesVehicule.front();
    for (size_t iLane = 0; iLane < pLink->GetLstLanes().size(); iLane++)
    {
        VoieMicro* pLane = (VoieMicro*)pLink->GetLstLanes()[iLane];

        if (!pLink->IsVoieInterdite(pMainType, (int)iLane))
        {
            double dbFirstVehiclePos = -DBL_MAX, dbSecondVehiclePos = -DBL_MAX;
            CDiagrammeFondamental *pFollowerDF = NULL, *pLeaderDF = NULL;

            // Si on a dÃ©jÃ  un vÃ©hicule en attente pour la voie, on considÃ¨re qu'on a un espacement minimum pour calcul l'offre
            // de la voie, quelles que soient les positions des vÃ©hicules dÃ©jÃ  insÃ©rÃ©s
            Vehicule * pWaitingVehicle = NULL;
            std::map<Tuyau*, std::map<int, std::deque<boost::shared_ptr<Vehicule>>>>::const_iterator iterLink = pInput->m_mapVehEnAttente.find(pLink);
            if (iterLink != pInput->m_mapVehEnAttente.end())
            {
                std::map<int, std::deque<boost::shared_ptr<Vehicule>>>::const_iterator iterLane = iterLink->second.find((int)iLane);
                if (iterLane != iterLink->second.end() && !iterLane->second.empty())
                {
                    pWaitingVehicle = iterLane->second.back().get();
                }
            }
            
            if (!pWaitingVehicle)
            {
                for (size_t iVeh = 0; iVeh < m_LstVehicles.size(); iVeh++)
                {
                    Vehicule * pVeh = m_LstVehicles[iVeh].get();

                    if (pVeh->GetLink(0) == pLink && pVeh->GetVoie(0) && pVeh->GetVoie(0)->GetNum() == 0)
                    {
                        double dbPosVeh = pVeh->GetPos(0);
                        if (dbFirstVehiclePos == -DBL_MAX)
                        {
                            dbFirstVehiclePos = dbPosVeh;
                            pFollowerDF = pVeh->GetDiagFonda();
                        }
                        else if (dbPosVeh < dbFirstVehiclePos)
                        {
                            dbSecondVehiclePos = dbFirstVehiclePos;
                            pLeaderDF = pFollowerDF;
                            dbFirstVehiclePos = dbPosVeh;
                            pFollowerDF = pVeh->GetDiagFonda();
                        }
                        else if (dbSecondVehiclePos == -DBL_MAX || dbPosVeh < dbSecondVehiclePos)
                        {
                            dbSecondVehiclePos = dbPosVeh;
                            pLeaderDF = pVeh->GetDiagFonda();
                        }
                    }
                }
            }
            else
            {
                // on considÃ¨re un spacing Ã©quivalent Ã  KMax
                dbFirstVehiclePos = 0;
                dbSecondVehiclePos = dbFirstVehiclePos + 1/pWaitingVehicle->GetDiagFonda()->GetKMax();
                pLeaderDF = pWaitingVehicle->GetDiagFonda();
             }

            double dbLaneOffer = 0;
            if (dbSecondVehiclePos != -DBL_MAX && dbFirstVehiclePos != -DBL_MAX)
            {
                double dbSpacing = dbSecondVehiclePos - dbFirstVehiclePos;

                double dbCriticalSpacing = 1 / pLeaderDF->GetKCritique();

                if (dbSpacing >= dbCriticalSpacing)
                {
                    dbLaneOffer = pMainType->GetDebitMax();
                }
                else
                {
                    // Prevent division by zero just in case
                    dbSpacing = std::max<double>(dbSpacing, 0.00001);

                    double dbK = 1 / dbSpacing;
                    double dbV = VAL_NONDEF, dbQ = VAL_NONDEF;
                    int nRes = pLeaderDF->CalculVitEqDiagrOrig(dbV, dbK, dbQ, false);
                    if (nRes != ERR_SUCCES)
                    {
                        log() << Logger::Warning << " WARNING : Error in the fundamental diagram computation. Ignoring lane offer." << endl;
                        log() << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                    }
                    else
                    {
                        dbLaneOffer = dbV / dbSpacing;
                    }
                }   
            }
            else
            {
                // On prend dans ce cas un spacing trÃ¨s grand : on n'aura ainsi pas de restriction de capacitÃ© puisqu'on prendra Qc
                dbLaneOffer = pMainType->GetDebitMax();
            }
            dbAllLanesOffer += dbLaneOffer;
        }

    }

    return dbAllLanesOffer;
}

bool Reseau::GetOffers(const std::vector<std::string> & inputNames, std::vector<double> & offerValuesVect)
{
    for (size_t iInput = 0; iInput < inputNames.size(); iInput++)
    {
        const std::string & inputName = inputNames.at(iInput);

        char cOriginType;
        SymuViaTripNode * pInput = GetOrigineFromID(inputName, cOriginType);

        if (cOriginType != 'E') return false;

        assert(pInput->GetOutputConnexion()->m_LstTuyAssAv.size() == 1);

        Tuyau * pInputLink = pInput->GetOutputConnexion()->m_LstTuyAssAv.front();

        // VÃ©rification de la longueur du tronÃ§on qui doit Ãªtre supÃ©rieure Ã  1 / Kc
        for (size_t iVehType = 0; iVehType < m_LstTypesVehicule.size(); iVehType++)
        {
            TypeVehicule * pVehType = m_LstTypesVehicule[iVehType];
            CDiagrammeFondamental DF(pVehType->GetW(), pVehType->GetKx(), pVehType->GetVx(), NULL);
            double dbKc = DF.GetKCritique();
            if (pInputLink->GetLength() < 1 / dbKc)
            {
                log() << Logger::Error << " Error : The interface link " << pInputLink->GetLabel() << " is too short ( < 1 / Kc) for vehicle type " << pVehType->GetLabel() << endl;
                log() << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                return false;
            }
        }

        double dbOffer = ComputeOffer(pInput, pInputLink);
        offerValuesVect.push_back(dbOffer);
    }
    
    return true;
}

bool Reseau::SetCapacities(const std::string & exitName, const std::string & downstreamRouteName, double dbCapacityValue)
{
    char cDestinationType;
    SymuViaTripNode * pOutput = GetDestinationFromID(exitName, cDestinationType);

    if (cDestinationType != 'S') return false;

    Sortie * pExit = (Sortie*)pOutput;

    if (downstreamRouteName.empty())
    {
        ListOfTimeVariation<tracked_double>* pLstCapacites = pExit->GetLstCapacites();
        pLstCapacites->RemoveVariations();
        pLstCapacites->AddVariation(m_dbDureeSimu, boost::make_shared<tracked_double>(dbCapacityValue));
    }
    else
    {
        // Gestion d'une capacitÃ© par identifiant de route aval Ã  la sortie
        pExit->GetCapacitiesPerRoutes()[downstreamRouteName] = dbCapacityValue;
    }

    return true;
}

#endif // USE_SYMUCORE


template void Reseau::serialize(boost::archive::xml_woarchive & ar, const unsigned int version);
template void Reseau::serialize(boost::archive::xml_wiarchive & ar, const unsigned int version);

template<class Archive>
void Reseau::serialize(Archive & ar, const unsigned int version)
{
    ar & BOOST_SERIALIZATION_NVP(m_bLightSavingMode);
    ar & BOOST_SERIALIZATION_NVP(m_LstVehicles);

    if (m_bLightSavingMode && Archive::is_loading::value)
    {
        for (size_t iVeh = 0; iVeh < m_LstVehicles.size(); iVeh++)
        {
            boost::shared_ptr<Vehicule> pVeh = m_LstVehicles[iVeh];
            if (pVeh->GetFleet()){
                pVeh->GetFleet()->DoVehicleAssociation(pVeh);
            }

            if (DoSortieTraj())
            {
                std::vector<std::string> initialPath;
                for (size_t iTuy = 0; iTuy < pVeh->GetItineraire()->size(); iTuy++)
                {
                    initialPath.push_back(pVeh->GetItineraire()->at(iTuy)->GetLabel());
                }

                SymuViaFleetParameters * pFleetParams = dynamic_cast<SymuViaFleetParameters*>(pVeh->GetFleetParameters());
                AddVehiculeToDocTrafic(pVeh->GetID(), "", pVeh->GetType()->GetLabel(), pVeh->GetType()->GetGMLLabel(), pVeh->GetType()->GetKx(), pVeh->GetVitMax(),
                    pVeh->GetType()->GetW(), pVeh->GetOrigine() ? pVeh->GetOrigine()->GetInputID() : "", pVeh->GetDestination() ? pVeh->GetDestination()->GetInputID() : "",
                    "", "", pFleetParams ? pFleetParams->GetRouteId() : "", -1.0, "",
                    pVeh->IsAgressif(), pVeh->GetVoie(0)->GetNum(), "", initialPath, pVeh->GetType()->GetLstPlagesAcceleration(), "", "");

            }
        }
        for (size_t iVeh = 0; iVeh < m_LstVehicles.size(); iVeh++)
        {
            boost::shared_ptr<Vehicule> pVeh = m_LstVehicles[iVeh];
            if (DoSortieTraj())
            {
                std::vector<std::string> initialPath;
                for (size_t iTuy = 0; iTuy < pVeh->GetItineraire()->size(); iTuy++)
                {
                    initialPath.push_back(pVeh->GetItineraire()->at(iTuy)->GetLabel());
                }

                SymuViaFleetParameters * pFleetParams = dynamic_cast<SymuViaFleetParameters*>(pVeh->GetFleetParameters());
                std::deque< TimeVariation<TraceDocTrafic> >::iterator itAllDocTrafic;
                // Pour Tous les fichiers Ã  extratire
                for (itAllDocTrafic = m_xmlDocTrafics.begin(); itAllDocTrafic != m_xmlDocTrafics.end(); itAllDocTrafic++)
                {
                    itAllDocTrafic->m_pData.get()->UpdateInstEntreeVehicule(pVeh->GetID(), 1.0);

                    m_vehiclesToAdd[itAllDocTrafic->m_pData.get()].push_back(SVehicleToAdd(pVeh->GetID(), "", pVeh->GetType()->GetLabel(), pVeh->GetType()->GetKx(), pVeh->GetVitMax(),
                        pVeh->GetType()->GetW(), pVeh->GetOrigine() ? pVeh->GetOrigine()->GetInputID() : "", pVeh->GetDestination() ? pVeh->GetDestination()->GetInputID() : "",
                        "", "", pFleetParams ? pFleetParams->GetRouteId() : "", 0.0, "",
                        pVeh->IsAgressif()));
                }
            }
        }
    }
    if (!m_bLightSavingMode) {

        ar & BOOST_SERIALIZATION_NVP(m_dbInstSimu);
        ar & BOOST_SERIALIZATION_NVP(m_nInstSim);

        ar & BOOST_SERIALIZATION_NVP(m_nLastVehicleID);

        ar & BOOST_SERIALIZATION_NVP(m_dbSumLengthTrAssNet);

        ar & BOOST_SERIALIZATION_NVP(m_nNbVehCum);

        // Variables caractÃ©risant la simulation
        ar & BOOST_SERIALIZATION_NVP(pas_de_temps);
        ar & BOOST_SERIALIZATION_NVP(hd);
        ar & BOOST_SERIALIZATION_NVP(md);
        ar & BOOST_SERIALIZATION_NVP(sd);
        ar & BOOST_SERIALIZATION_NVP(hf);
        ar & BOOST_SERIALIZATION_NVP(mf);
        ar & BOOST_SERIALIZATION_NVP(sf);

        ar & BOOST_SERIALIZATION_NVP(m_dbDureeSimu);

        ar & BOOST_SERIALIZATION_NVP(m_nodeToUpdate);
        ar & BOOST_SERIALIZATION_NVP(m_recordEventTimes);

        ar & BOOST_SERIALIZATION_NVP(m_VehiclesToCreate);
        ar & BOOST_SERIALIZATION_NVP(m_LstInitVehicule);
        ar & BOOST_SERIALIZATION_NVP(Liste_convergents);
        ar & BOOST_SERIALIZATION_NVP(Liste_divergents);

        ar & BOOST_SERIALIZATION_NVP(Liste_sortie);
        ar & BOOST_SERIALIZATION_NVP(Liste_entree);
        ar & BOOST_SERIALIZATION_NVP(m_mesoNodes);
        ar & BOOST_SERIALIZATION_NVP(Liste_origines);
        ar & BOOST_SERIALIZATION_NVP(Liste_destinations);

        ar & BOOST_SERIALIZATION_NVP(m_LstLTP);


        ar & BOOST_SERIALIZATION_NVP(m_sPrefixOutputFiles);
        ar & BOOST_SERIALIZATION_NVP(m_sSuffixOutputFiles);

        bool bHasPythonUtils;
        if (Archive::is_saving::value)
        {
        bHasPythonUtils = m_pPythonUtils != NULL;
        }
        ar & BOOST_SERIALIZATION_NVP(bHasPythonUtils);
        if (Archive::is_loading::value)
        {
        if (bHasPythonUtils)
        {
        GetPythonUtils();
        }
        }

        if(Archive::is_loading::value)
        {
        InitLogFicSimulation(); // ouverture du fichier de log
        }

        ar & BOOST_SERIALIZATION_NVP(m_LstUserCnxs);

        ar & BOOST_SERIALIZATION_NVP(Liste_repartiteurs);
        ar & BOOST_SERIALIZATION_NVP(Liste_parkings);
        ar & BOOST_SERIALIZATION_NVP(Liste_zones);
        ar & BOOST_SERIALIZATION_NVP(m_pCurrentProcessedNode);

        ar & BOOST_SERIALIZATION_NVP(Liste_giratoires);
        ar & BOOST_SERIALIZATION_NVP(Liste_carrefoursAFeux);

        ar & BOOST_SERIALIZATION_NVP(m_LstMergingObjects);

        ar & BOOST_SERIALIZATION_NVP(Liste_ControleursDeFeux);

        ar & BOOST_SERIALIZATION_NVP(m_LstTypesVehicule);

        ar & BOOST_SERIALIZATION_NVP(m_LstMotifs);

        ar & BOOST_SERIALIZATION_NVP(m_bInitSimuTrafic);
        ar & BOOST_SERIALIZATION_NVP(m_bInitSimuEmissions);

        ar & BOOST_SERIALIZATION_NVP(m_bDoVehicleListSensorExtraction);

        ar & BOOST_SERIALIZATION_NVP(m_bAccBornee);

        // ParamÃ¨tre de changement de voie global
        ar & BOOST_SERIALIZATION_NVP(m_bChgtVoie);
        ar & BOOST_SERIALIZATION_NVP(m_dbFAlphaMandatory);
        ar & BOOST_SERIALIZATION_NVP(m_cChgtVoieMandatoryProba);
        ar & BOOST_SERIALIZATION_NVP(m_cChgtVoieMandatoryMode);
        ar & BOOST_SERIALIZATION_NVP(m_dbRelaxation);
        ar & BOOST_SERIALIZATION_NVP(m_dbDstChgtVoie);

        ar & BOOST_SERIALIZATION_NVP(m_dbDstChgtVoieForce);
        ar & BOOST_SERIALIZATION_NVP(m_dbPhiChgtVoieForce);

        ar & BOOST_SERIALIZATION_NVP(m_bDepassement);
        ar & BOOST_SERIALIZATION_NVP(m_bTraversees);

        ar & BOOST_SERIALIZATION_NVP(m_nOrdreChgtVoieDiscr);

        ar & BOOST_SERIALIZATION_NVP(m_bProcAgressivite);

        ar & BOOST_SERIALIZATION_NVP(m_bChgtVoieGhost);
        ar & BOOST_SERIALIZATION_NVP(m_nChgtVoieGhostDurationMin);
        ar & BOOST_SERIALIZATION_NVP(m_nChgtVoieGhostDurationMax);
        ar & BOOST_SERIALIZATION_NVP(m_dbChgtVoieGhostBevelLength);

        ar & BOOST_SERIALIZATION_NVP(m_nMesoNbVehChgtVoie);

        ar & BOOST_SERIALIZATION_NVP(m_bModeDepassementChgtDir);
        ar & BOOST_SERIALIZATION_NVP(m_dbDistDepassementChgtDir);


        // Variables liÃ©es Ã  l'enregistrement de la simulation
        ar & BOOST_SERIALIZATION_NVP(m_strTitre);
        ar & BOOST_SERIALIZATION_NVP(m_dtDateSimulation);

        ar & BOOST_SERIALIZATION_NVP(m_dtDebutSimu);
        ar & BOOST_SERIALIZATION_NVP(m_dtFinSimu);
        ar & BOOST_SERIALIZATION_NVP(m_dtDebutData);
        ar & BOOST_SERIALIZATION_NVP(m_dbDiffDebutSimuData);

        ar & BOOST_SERIALIZATION_NVP(m_sLogOutFile);
        ar & BOOST_SERIALIZATION_NVP(m_sOutputDir);

        ar & BOOST_SERIALIZATION_NVP(m_bSimuTrafic);
        ar & BOOST_SERIALIZATION_NVP(m_bSimuAcoustique);
        ar & BOOST_SERIALIZATION_NVP(m_bSimuAir);
        ar & BOOST_SERIALIZATION_NVP(m_bSimuSirane);

        ar & BOOST_SERIALIZATION_NVP(Loi_emis);
        ar & BOOST_SERIALIZATION_NVP(m_bRelancerCalculRepartiteur);
        ar & BOOST_SERIALIZATION_NVP(m_nLastIdSegment);
        ar & BOOST_SERIALIZATION_NVP(m_pGestionsCapteur);

        // Type de comportement du flux ou des vehicules
        ar & BOOST_SERIALIZATION_NVP(m_bCptDirectionnel);
        ar & BOOST_SERIALIZATION_NVP(m_bCptDestination);
        ar & BOOST_SERIALIZATION_NVP(m_bCptItineraire);
        ar & BOOST_SERIALIZATION_NVP(m_bAffectationDynamique);

        // Gestion de l'affectation
        ar & BOOST_SERIALIZATION_NVP(m_nNbPlusCourtChemin);

        ar & BOOST_SERIALIZATION_NVP(m_dbKxmin);
        ar & BOOST_SERIALIZATION_NVP(m_dbMaxDebitMax);

        ar & BOOST_SERIALIZATION_NVP(m_bLoiPoursuiteOld);
        ar & BOOST_SERIALIZATION_NVP(m_sLoipoursuite);

        ar & BOOST_SERIALIZATION_NVP(m_bProcDec);
        ar & BOOST_SERIALIZATION_NVP(m_dbDecTaux);

        ar & BOOST_SERIALIZATION_NVP(m_cTypeSortieAcoustique);

        ar & BOOST_SERIALIZATION_NVP(m_bOffreCvgDeltaN);

        ar & BOOST_SERIALIZATION_NVP(m_bCalculTqConvergent);

        ar & BOOST_SERIALIZATION_NVP(m_setTypesRestitution);

        ar & BOOST_SERIALIZATION_NVP(m_LstItiChangeInstants);

        ar & BOOST_SERIALIZATION_NVP(m_LstFleets);

        ar & BOOST_SERIALIZATION_NVP(m_MicroVehicleTypes);
        ar & BOOST_SERIALIZATION_NVP(m_dbUpstreamMicroDistance);
        ar & BOOST_SERIALIZATION_NVP(m_dbDownstreamMicroDistance);
        ar & BOOST_SERIALIZATION_NVP(m_dbSidesMicroDistance);

        ar & BOOST_SERIALIZATION_NVP(m_bEstimateTrafficLightsWaitTime);

        ar & BOOST_SERIALIZATION_NVP(m_mapRouteFromNodes);
        ar & BOOST_SERIALIZATION_NVP(m_mapRouteFromNodesToInternalNode);
        ar & BOOST_SERIALIZATION_NVP(m_mapRouteFromNodesToLink);
        ar & BOOST_SERIALIZATION_NVP(m_bUseMapRouteFromNodes);

        ar & BOOST_SERIALIZATION_NVP(m_pParkingParameters);

        ar & BOOST_SERIALIZATION_NVP(m_bAcousCell);
        ar & BOOST_SERIALIZATION_NVP(m_bAcousSrcs); 

        ar & BOOST_SERIALIZATION_NVP(m_bDebug);
        ar & BOOST_SERIALIZATION_NVP(m_bDebugOD);
        ar & BOOST_SERIALIZATION_NVP(m_bDebugSAS);
        ar & BOOST_SERIALIZATION_NVP(m_bChgtVoieDebug);
        ar & BOOST_SERIALIZATION_NVP(m_bSortieLight);
        ar & BOOST_SERIALIZATION_NVP(m_bSortieTraj);
        ar & BOOST_SERIALIZATION_NVP(m_bDisableTrajectoriesOutput);
        ar & BOOST_SERIALIZATION_NVP(m_SymMode);
        ar & BOOST_SERIALIZATION_NVP(m_bXmlOutput);
        ar & BOOST_SERIALIZATION_NVP(m_bTraceRoute);
        ar & BOOST_SERIALIZATION_NVP(m_bVGPDebug);
        ar & BOOST_SERIALIZATION_NVP(m_bTraceStocks);
        ar & BOOST_SERIALIZATION_NVP(m_bSortieRegime);
        ar & BOOST_SERIALIZATION_NVP(m_bCSVOutput);
        ar & BOOST_SERIALIZATION_NVP(m_bCSVTrajectories);
        ar & BOOST_SERIALIZATION_NVP(m_bCSVSensors);
        ar & BOOST_SERIALIZATION_NVP(m_bGMLOutput);
        ar & BOOST_SERIALIZATION_NVP(m_bTravelTimesOutput);
        ar & BOOST_SERIALIZATION_NVP(m_dbTravelTimesOutputPeriod);

        ar & BOOST_SERIALIZATION_NVP(m_LstTuyaux);
        ar & BOOST_SERIALIZATION_NVP(m_LstTuyauxMacro);
        ar & BOOST_SERIALIZATION_NVP(m_LstTuyauxMicro);
        ar & BOOST_SERIALIZATION_NVP(m_LstTuyauxMeso);
        ar & BOOST_SERIALIZATION_NVP(m_mapTuyaux);

        ar & BOOST_SERIALIZATION_NVP(m_SimulationID);
        ar & BOOST_SERIALIZATION_NVP(m_TraficID);
        ar & BOOST_SERIALIZATION_NVP(m_ReseauID);

        ar & BOOST_SERIALIZATION_NVP(m_nNbCellSirane);
        ar & BOOST_SERIALIZATION_NVP(m_dbMinLongueurCellSirane);
        ar & BOOST_SERIALIZATION_NVP(m_dbPeriodeAgregationSirane);
        ar & BOOST_SERIALIZATION_NVP(m_dbDebutPeriodeSirane);
        ar & BOOST_SERIALIZATION_NVP(m_bExtensionBarycentresSirane);

        ar & BOOST_SERIALIZATION_NVP(m_pRegulationModule);

        ar & BOOST_SERIALIZATION_NVP(m_pCarFollowingFactory);

        // gestion de la seed pour reprise du snapshot au mÃªme endroit de la sÃ©quence
        unsigned int randCount;
        if(Archive::is_saving::value)
        {
        randCount = m_pRandManager->getCount();
        }
        ar & BOOST_SERIALIZATION_NVP(randCount);
        if(Archive::is_loading::value)
        {
        RestoreSeed(randCount);
        }

        ar & BOOST_SERIALIZATION_NVP(m_LstPlagesTemporelles);
        ar & BOOST_SERIALIZATION_NVP(m_bTypeProfil);
        ar & BOOST_SERIALIZATION_NVP(m_dbTetaLogit);
        ar & BOOST_SERIALIZATION_NVP(m_nMode);
        ar & BOOST_SERIALIZATION_NVP(m_dbWardropTolerance);
        ar & BOOST_SERIALIZATION_NVP(m_eShortestPathHeuristic);
        ar & BOOST_SERIALIZATION_NVP(m_dbHeuristicGamma);
        ar & BOOST_SERIALIZATION_NVP(m_dbAStarBeta);
        ar & BOOST_SERIALIZATION_NVP(m_dbDijkstraAlpha);
        ar & BOOST_SERIALIZATION_NVP(m_dbDijkstraBeta);
        ar & BOOST_SERIALIZATION_NVP(m_dbDijkstraGamma);
        ar & BOOST_SERIALIZATION_NVP(m_bCommonalityFilter);
        ar & BOOST_SERIALIZATION_NVP(m_dbCommonalityAlpha);
        ar & BOOST_SERIALIZATION_NVP(m_dbCommonalityBeta);
        ar & BOOST_SERIALIZATION_NVP(m_dbCommonalityGamma);

        ar & BOOST_SERIALIZATION_NVP(m_dbRightTurnPenalty);
        ar & BOOST_SERIALIZATION_NVP(m_dbNonPriorityPenalty);
        ar & BOOST_SERIALIZATION_NVP(m_dbAngleToutDroit);
        ar & BOOST_SERIALIZATION_NVP(m_dbRightTurnPenaltyFactor);
        ar & BOOST_SERIALIZATION_NVP(m_dbNonPriorityPenaltyFactor);

        ar & BOOST_SERIALIZATION_NVP(m_dbLag);

        // Polution atmosphÃ©riques
        ar & BOOST_SERIALIZATION_NVP(m_dbCptCumCO2);
        ar & BOOST_SERIALIZATION_NVP(m_dbCptCumNOx);
        ar & BOOST_SERIALIZATION_NVP(m_dbCptCumPM);

        ar & BOOST_SERIALIZATION_NVP(m_uiSeed);
        ar & BOOST_SERIALIZATION_NVP(m_uiRandomSeed);
        ar & BOOST_SERIALIZATION_NVP(m_bPickedSeed);

        ar & BOOST_SERIALIZATION_NVP(m_pModuleAffectation);
        ar & BOOST_SERIALIZATION_NVP(m_bSaveAffectation);

        ar & BOOST_SERIALIZATION_NVP(m_bDriven);

        SerialiseDOMDocument<Archive>(ar, "m_XMLDocData", XS("ROOT_SYMUBRUIT"), m_XMLDocData, m_pXMLUtil);

        ar & BOOST_SERIALIZATION_NVP(m_xmlDocTrafics);
        ar & BOOST_SERIALIZATION_NVP(m_XmlDocAcoustique);
        ar & BOOST_SERIALIZATION_NVP(m_XmlDocSirane);

        ar & BOOST_SERIALIZATION_NVP(m_bFichierSAS);

        //For now, we don't save connected vehicle in light saving mode
#ifdef USE_SYMUCOM
        ar & BOOST_SERIALIZATION_NVP(m_pSymucomSimulator);
#endif // USE_SYMUCOM
    }
}


