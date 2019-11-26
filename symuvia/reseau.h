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
#ifndef reseauH
#define reseauH

#pragma warning(push)
#pragma warning(disable : 4251)

#include "SymubruitExports.h"

#include "tools.h"
#include "MesoNode.h"
#include "vehicule.h"
#include "ZoneDeTerminaison.h"

#include <boost/shared_ptr.hpp>
#include <xercesc/util/XercesVersion.hpp>

#include "ogr_spatialref.h"

#include <deque>
#include <fstream>
#include <vector>
#include <string>

namespace XERCES_CPP_NAMESPACE {
    class DOMNode;
    class DOMDocument;
}

namespace boost {
    class mutex;
}

namespace eveShared {
    class EveNetwork;
    class EveTroncon;
}

#ifdef USE_SYMUCORE
#include <Utils/SymuCoreConstants.h>
#include <Graph/Model/ListTimeFrame.h>
namespace SymuCore {
    class MultiLayersGraph;
    class MacroType;
    class Origin;
    class Destination;
    class SubPopulation;
    class Node;
    class VehicleType;
    class AbstractPenalty;
    class Pattern;
    class Populations;
    class IUserHandler;
    class Trip;
}
namespace boost {
    namespace posix_time {
        class ptime;
        class time_duration;
    }
}
#endif // USE_SYMUCORE

class Tuyau;
class TuyauMacro;
class TuyauMicro;
class Repartiteur;
class Sortie;
class Entree;
class Arret;
class Loi_emission;
class Vehicule;
class TypeVehicule;
class SensorsManagers;
class DocTrafic;
class XMLDocTrafic;
class DocAcoustique;
class TraceDocTrafic;
class XMLDocAcoustique;
class EVEDocAcoustique;
class XMLDocEmissionsAtmo;
class XMLDocSirane;
class XMLReaderTrafic;
class Convergent;
class Divergent;
class Giratoire;
class CarrefourAFeuxEx;
class ControleurDeFeux;
class Voie;
class ConnectionPonctuel;
class SegmentMacro;
class BriqueDeConnexion;
struct PtConflitTraversee;
struct GrpPtsConflitTraversee;
class VoieMicro;
struct Point;
class PlanDeFeux;
class Affectation;
class SymuViaTripNode;
class Parking;
struct ZoneZLevel;
class RegulationModule;
class MergingObject;
class CSVOutputWriter;
class CTuyauMeso;
class CarFollowingFactory;
class Logger;
class SymuCoreManager;
class AbstractFleet;
class SymuViaFleet;
class PublicTransportFleet;
class DeliveryFleet;
class RepartitionTypeVehicule;
class VehicleToCreate;
class PointDeLivraison;
class CMotif;
class TravelTimesOutputManager;
class ParkingParameters;
class PythonUtils;
class RandManager;
class XMLUtil;
#ifdef USE_SYMUCORE
class SymuCoreGraphBuilder;
#endif // USE_SYMUCORE
#ifdef USE_SYMUCOM
class Simulator;
#endif // USE_SYMUCOM


//#pragma warning(disable : 4996)

///<summary>
/// Classe de gestion d'une simulation
///</summary>
class SYMUBRUIT_EXPORT Reseau
{
    // Contient les donnÃ©es d'un vÃ©hicule dont la crÃ©ation dans les fichiers rÃ©sultat pourra Ãªtre requise ultÃ©rieurement
    // (en cas d'utilisation de plages temporelles pour la restitution des rÃ©sultats)
    struct SVehicleToAdd
    {
        int nID;
        std::string strLibelle;
        std::string strType;
        double dbKx;
        double dbVx;
        double dbW;
        std::string strEntree;
        std::string strSortie;
        std::string strZoneEntree;
        std::string strZoneSortie;
        std::string strRoute;
        double dbInstCreation;
        std::string sVoie;
        bool bAgressif;

        SVehicleToAdd( int nIDIn, const std::string & strLibelleIn, const std::string & strTypeIn, 
            double dbKxIn, double dbVxIn, double dbWIn, 
            const std::string & strEntreeIn, const std::string & strSortieIn,
            const std::string & strZoneEntreeIn, const std::string & strZoneSortieIn, const std::string & strRouteIn,
            double dbInstCreationIn, const std::string & sVoieIn, bool bAgressifIn)
        {
            nID   = nIDIn;
            strLibelle = strLibelleIn;
            strType = strTypeIn;
            dbKx = dbKxIn;
            dbVx = dbVxIn;
            dbW  = dbWIn;
            strEntree  = strEntreeIn;
            strSortie  = strSortieIn;
            strZoneEntree = strZoneEntreeIn;
            strZoneSortie = strZoneSortieIn;
            strRoute   = strRouteIn;
            dbInstCreation = dbInstCreationIn;
            sVoie  = sVoieIn;
            bAgressif  = bAgressifIn;
        }
    };

public:
        // Variables stockant les diffÃ©rents objets constituant du rÃ©seau
        
        std::deque<Tuyau*>       m_LstTuyaux;			// Liste globale des tuyaux
        std::deque<TuyauMacro*>  m_LstTuyauxMacro;		// Sous-liste des tuyaux macroscopique
        std::deque<TuyauMicro*>  m_LstTuyauxMicro;		// Sous-liste des tuyaux microscopiques
        std::deque<CTuyauMeso*>  m_LstTuyauxMeso;       // Sous-liste des tuyaux mesoscopiques
        std::map<std::string, Tuyau*> m_mapTuyaux;     // Map des tuyaux pour accÃ¨s rapide par ID

		std::deque<Connexion*>	m_LstUserCnxs;			// Liste des connexions dÃ©finies par l'utilisateur (liste des connexions du rÃ©seau d'affectation)

	    std::map<std::string, Repartiteur*>    Liste_repartiteurs;     // Liste des rÃ©partiteurs        
        std::map<std::string, Convergent*>     Liste_convergents;      // Liste des convergents
        std::map<std::string, Divergent*>      Liste_divergents;       // Liste des divergents
	    std::deque<Sortie*>         Liste_sortie;           // Liste des sorties
        std::deque<Entree*>         Liste_entree;           // Liste des entrÃ©es
        std::deque<Parking*>        Liste_parkings;         // Liste des parkings
        std::deque<ZoneDeTerminaison*> Liste_zones;         // Liste des zones de terminaison
        std::deque<SymuViaTripNode*>   Liste_origines;      // Liste des origines
        std::deque<SymuViaTripNode*>    Liste_destinations;     // Liste des destinations

        std::deque<Trip*>               m_LstLTP;               // Liste des lignes de bus (ajoutÃ© en tant que champ ici uniquement pour le wrapper python
                                                                // et la conservation de la compatibilitÃ© avec les scripts existants)

        std::deque<Giratoire*>				Liste_giratoires;               // Liste des giratoires
        std::deque<CarrefourAFeuxEx*>		Liste_carrefoursAFeux;          // Liste des carrefours Ã  feux

        std::vector<AbstractFleet*>         m_LstFleets;                    // Liste des flottes

		std::deque<MergingObject*>			m_LstMergingObjects;			// Liste des mergings objects du rÃ©seau (dans le cas oÃ¹ gestion asynchrone de ceux-ci)

        std::deque<ControleurDeFeux*>		Liste_ControleursDeFeux;        // Liste des contrÃ´leurs de feux

        std::deque<TypeVehicule*>			m_LstTypesVehicule;				// Liste des types de vÃ©hicule du rÃ©seau

        std::deque<CMotif*>                 m_LstMotifs;                    // Liste des motifs de dÃ©placement des usagers

        bool                        m_bInitSimuTrafic;          // Indique si l'initialisation de la simulation du trafic a Ã©tÃ© faite        
        bool                        m_bInitSimuEmissions;       // Indique si l'initialisation de la simulation Ã©missions a Ã©tÃ© faite

        bool                        m_bAcousCell;				// Indique si les Ã©missions acoustiques sont restituÃ©es pour chaque cellule du tronÃ§on
		bool                        m_bAcousSrcs;				// Indique si les Ã©missions acoustiques sont restituÃ©es pour chaque source

		double			m_dbInstSimu;							// instant de la simulation
        double			m_dbInstSimuMeso;						// instant de la simulation Mesoscopique
        int				m_nInstSim;								// indice de l'instant simulÃ©     

		int				m_nNbVehCum;							// Cumul du nombre de vÃ©hicules prÃ©sents sur le rÃ©seau Ã  chaque pas de temps

		int             m_nLastVehicleID;                       // ID du dernier vÃ©hicule crÃ©Ã©

		double			m_dbSumLengthTrAssNet;					// Somme des longueurs des tronÃ§ons du rÃ©seau d'affectation

        std::vector<PlageTemporelle *>    m_LstPlagesTemporelles; // Liste des plages temporelles dÃ©finies pour la simulation

        bool            m_bTypeProfil;                          // Type de dÃ©finition des variantes temporelles (profil ou horaire)

        bool            m_bDoVehicleListSensorExtraction;
        // ETS 130731 
        // Mesoscopique processing
        CMesoNode * m_pCurrentProcessedNode;

        //! Noeuds mÃ©soscopiques ordonnÃ©e suivant le temps de l'Ã©venement suivant
        // rmq. anciennement set avec opÃ©rateur de comparaison basÃ© sur l'instant : buggÃ© car impossible alors d'avoir deux noeuds avec le mÃªme instant de prochain Ã©vÃ¨nement.
        std::vector<CMesoNode *> m_mesoNodes;

        std::map<Tuyau*, TronconOrigine*>       m_mapOriginLinksForCreatedVehicles;
        std::map<Tuyau*, TronconDestination*>   m_mapDestinationLinksForCreatedVehicles;


        // Retourne vrai si l'application doit restituer les listes de vehicle des capteurs
        bool DoVehicleListSensorExtraction() const;

        // Ajoute un vÃ©hicule au fichier de restitution en gÃ©rajt les bornes de restitution
        void AddVehiculeToDocTrafic(int nID, const std::string & strLibelle, const std::string & strType, const std::string & strGMLType, 
            double dbKx, double dbVx, double dbW, 
            const std::string & strEntree, const std::string & strSortie, const std::string & strZoneEntree, const std::string & strZoneSortie,
            const std::string & strRoute,
            double dbInstCreation, const std::string & sVoie, bool bAgressif,
            int iLane, const std::string & sLine, const std::vector<std::string> & initialPath, const std::deque<PlageAcceleration> & plagesAcc,
            const std::string & motifOrigine, const std::string & motifDestination);

        // Liste des vÃ©hicules qui ont Ã©tÃ© crÃ©Ã© durant la nom extraction des donnÃ©es et qui le seront oau premier instant d'extraction
        // liste des vÃ©hicules par document d'extraction
        std::map<DocTrafic*, std::deque<SVehicleToAdd > > m_vehiclesToAdd;

        // Liste des vÃ©hicules Ã  crÃ©er au prochain pas de temps (via SymCreateVehicle)
        std::deque<VehicleToCreate*> m_VehiclesToCreate;
public:
		typedef enum
		{
			SYM_MODE_FULL = 0,									// Mode simulation complÃ¨te
			SYM_MODE_STEP_XML = 1,								// Mode simulation pas Ã  pas avec flux XML
			SYM_MODE_STEP_EVE = 2,								// Mode simulation pas Ã  pas avec des classes EVE
            SYM_MODE_STEP_JSON = 3								// Mode simulation pas Ã  pas avec JSON
		} SymMode;

        // Heuristique pour le calcul des plus courts chemins
        enum shortest_path_heuristic {
            HEURISTIC_NONE = 0,         // Pas d'heuristique
            HEURISTIC_EUCLIDIAN = 1     // Distance euclidienne (pour algo A*)
        };

private:

        // Variables caractÃ©risant la simulation
	    double pas_de_temps; // pas de temps
        int hd; // Heure de dÃ©but de la simulation
        int md; // Minute de dÃ©but de la simulation
        int sd; // Seconde de dÃ©but de la simulation
        int hf; // Heure de fin de la simulation
        int mf; // Minute de fin de la simulation
        int sf; // Seconde de fin de la simulation       

        double m_dbDureeSimu;				// durÃ©e de la simulation (en seconde)       		

        bool    m_bDebug;               // Version debug   
        bool    m_bDebugOD;             // Sortie des matrices ODs et variantes
        bool    m_bDebugSAS;            // Sortie des SAS
        bool    m_bChgtVoieDebug;       // Version debug pour les changements de voie
        bool    m_bSortieLight;         // Indique si mode de sortie light (sauvegarde uniquement les donnÃ©es utiles pour SymuPlot et SymuPlayer)
        bool    m_bSortieTraj;          // Indique si les trajectoires doivent Ãªtre inscrites dans le fichier de sortie
        bool    m_bDisableTrajectoriesOutput; // Permet de dÃ©sactiver temporairement la sortie des trajectoires
		SymMode	m_SymMode;				// Mode de fonctionnement selon les valeurs Ã©numÃ©rÃ©es
		bool    m_bXmlOutput;			// Indique si gÃ©nÃ©ration du fichier de sortie au format XML
        bool    m_bTraceRoute;          // Indique si l'itinÃ©raire de chaque vÃ©hicule est tracÃ© dans le fichier trafic     
        bool    m_bVGPDebug;            // Indique si on doit produire des sorties supplÃ©mentaires dans le simulation.log par rapport Ã  la gestion des VGP dans les controleurs de feux
        bool    m_bTraceStocks;         // Indique si on doit produire les sorties relatives aux stocks de stationnement (parkings et tronÃ§ons)
        bool    m_bSortieRegime;        // Indique si on doit produite les sorties relatives Ã  l'Ã©tat fluide ou congestionnÃ© des vÃ©hicules
        bool    m_bCSVOutput;           // Indique si on doit produire les sorties CSV
        bool    m_bCSVTrajectories;     // Indique si on doit produire les sorties CSV de type trajectoires
        bool    m_bCSVSensors;          // Indique si on doit produire les sorties CSV de type capteur
        bool    m_bGMLOutput;           // Indique si on doit produire les sorties GML
        bool    m_bTravelTimesOutput;   // Indique si on doit produire la sortie JSON des temps de parcours
        double  m_dbTravelTimesOutputPeriod; // Periode d'agrÃ©gation des temps de parcours pour la sortie JSON des temps de parcours

        bool    m_bAccBornee;           // Indique si la simulation gÃ¨re l'accÃ©lÃ©ration bornÃ©e

		OGRSpatialReference *m_pOGRSpRefInput;			// SystÃ¨mes de coordonnÃ©es gÃ©orÃ©fÃ©rencÃ©es des donnÃ©es d'entrÃ©e
		OGRSpatialReference *m_pOGRSpRefOutput;			// SystÃ¨mes de coordonnÃ©es gÃ©orÃ©fÃ©rencÃ©es des rÃ©sultats restituÃ©s
		

		// ParamÃ¨tre de changement de voie global
        bool    m_bChgtVoie;            // Indique si la simulation gÃ¨re le changement de voie      
        bool    m_bDepassement;         // Indique si la simulation gÃ¨re le dÃ©passement sur tronÃ§on opposÃ©
        bool    m_bTraversees;          // Indique si la simulation gÃ¨re les traversÃ©es (briques et voies interdites)
		double  m_dbFAlphaMandatory;		// Valeur de alpha pour le changement de voie 'mandatory'
		char	m_cChgtVoieMandatoryProba;	// Type de probabilitÃ© dans le cas d'un changement de voie 'mandatory' (cf. Vehicule::CalculChangementDeVoie)
		char	m_cChgtVoieMandatoryMode;	// Mode de gestion des changements de voie 'mandatory' ('S': synchrone, 'A' : asynchrone)
        double  m_dbRelaxation;         // Coefficient de relaxation du changement de voie
        double  m_dbDstChgtVoie;        // Distance avant la fin du tronÃ§on pour lequel on calcule le chgt de voie
                                        // d'un vÃ©hicule qui ne se trouve pas sur la bonne voie pour prendre le tronÃ§on suivant
        double  m_dbDstChgtVoieForce;   // Idem que m_dbDstChgtVoie mais pour une zone encore plus forcÃ©e avec un phi arbitraire
        double  m_dbPhiChgtVoieForce;   // Phi Ã  appliquer dans la zone dÃ©finie par m_dbDstChgtVoieForce

		int		m_nOrdreChgtVoieDiscr;	// Ordre du changement de voie discretionnaire ( 1 : d'abord Ã  gauche puis Ã  droite, 2 : d'abord Ã  droite puis Ã  gauche, 3 : voie la plus favorable )

		bool	m_bProcAgressivite;		// Indique si la procÃ©dure d'agressivitÃ© des vÃ©hicules est appliquÃ©e

		bool	m_bChgtVoieGhost;			// Indique si l'extension 'ghost' est appliquÃ©e
		double	m_nChgtVoieGhostDurationMin;	// Min de la durÃ©e d'existence du ghost suite Ã  un changement de voie
		double	m_nChgtVoieGhostDurationMax;	// Max de la durÃ©e d'existence du ghost suite Ã  un changement de voie
		double  m_dbChgtVoieGhostBevelLength;	// Dans le cas d'une rÃ©duction de voie, longueur utile en fin de tronÃ§on pour effectuer la procÃ©dure de ghost (le vÃ©hicule sur la voie rÃ©duite s'arrÃªtera donc Ã  longueur(troncon) - chgtvoie_ghost_lengthBevel )

        int     m_nMesoNbVehChgtVoie;           // Nombre de vÃ©hicule meso maximum par voie pouvant Ãªtre sautÃ©s par un vÃ©hicule associÃ© Ã  un mouvement Ã  un noeud non gÃ©nÃ© par les vÃ©hicules prÃ©cÃ©dents en attente au noeud.

        bool    m_bModeDepassementChgtDir;      // Mode anti congestion pour le rÃ©seau de Charlotte : permet aux vÃ©hicules bloquÃ©s par des vÃ©hicules qui ne vont pas sur le mÃªme tronÃ§on aval de les traverser en fin de tronÃ§on
        double  m_dbDistDepassementChgtDir;     // Distance Ã  considÃ©rer avant la fin du tronÃ§on pour le mode m_bModeDepassementChgtDir

        // Variables liÃ©es Ã  l'enregistrement de la simulation
        std::string                 m_strTitre;             // Titre de la simulation
        SDateTime                   m_dtDateSimulation;     // Date de la simulation

        STime						m_dtDebutSimu;          // DÃ©but de la simulation
        STime	                    m_dtFinSimu;            // Fin de la simulation
        SDateTime                   m_dtDebutData;          // Date de dÃ©but des variations
        double                      m_dbDiffDebutSimuData;  // Nombre de seconde sÃ©parant le dÃ©but de la simulation du dÃ©but des variations des donnÃ©es        

        // ETS 140915 Variables liÃ©es Ã  l'extraction
        std::vector< PlageTemporelle *>  m_extractionRange; // DÃ©but de l'extraction (dÃ©faut 0 -> extraction depuis m_dtDebutSimu)
                                                          // et Fin de l'extraction (dÃ©faut 0 -> extraction jusqu'Ã   m_dtFinSimu )

		std::string				        m_sPrefixOutputFiles;	// chemin + prÃ©fixe des fichiers de sortie
        std::string                     m_sSuffixOutputFiles;   // Suffixe des fichiers de sortie (si rÃ©plication)
		std::string						m_sLogOutFile;			// chemin + prÃ©fixe des fichiers de log
        std::string						m_sOutputDir;			// rÃ©pertoire de sortie


        bool m_bSimuTrafic;			// Indique si il faut lancer la simulation du trafic
        bool m_bSimuAcoustique;     // Indique si il faut lancer la simulation acoustique      
        bool m_bSimuAir;            // Indique si il faut lancer la simulation acoustique   
        bool m_bSimuSirane;          // Indique si il faut produire les sorties nÃ©cessaires Ã  Sirane (Ã©missions polluants)

        // Variables liÃ©es au calcul des Ã©missions de bruit
        Loi_emission* Loi_emis; // p sur l'objet loi d'Ã©mission

        // Variables liÃ©es Ã  la simulation microscopique
        bool            m_bRelancerCalculRepartiteur;           // Indique si il faut relancer les calculs des rÃ©partiteurs (dans le cas oÃ¹ la demande ou l'offre ont Ã©tÃ© modifÃ©es au cours du pas de temps)

        int             m_nLastIdSegment;                       // ID du dernier segment crÃ©Ã©

        // Gestion des capteurs
        SensorsManagers *m_pGestionsCapteur;                      // Pointeur sur l'objet gestion capteur

        // Type de comportement du flux ou des vehicules
        bool            m_bCptDirectionnel;                      // Flux ou vÃ©hicules rÃ©partis au niveau des rÃ©partiteurs par des coefficients directionnels
        bool            m_bCptDestination;                       // Les vÃ©hicules connaissent leur destination 
        bool            m_bCptItineraire;                        // Les vÃ©hicules connaissent leur itinÃ©raire
        bool            m_bAffectationDynamique;                 // Indique si on fait une affectation dynamique (pÃ©riodique) ou non dans le mode itinÃ©raire

		// Gestion de l'affectation
        int             m_nNbPlusCourtChemin;                   // Nombre de plus court chemin

        double                      m_dbKxmin;                  // minimum sur l'ensemble des types de vÃ©hicules de Kx
        double                      m_dbMaxDebitMax;            // maximum sur l'ensemble des types de vÃ©hicules du dÃ©bit max

        bool                        m_bLoiPoursuiteOld;         // Indique quelle loi de poursuite est utilisÃ©e
		std::string				    m_sLoipoursuite;				// Loi de poursuite

        bool                        m_bProcDec;                 // Indique si la procÃ©dure de correction de la dÃ©cÃ©lÃ©ration doit Ãªtre lancÃ©e
        double                      m_dbDecTaux;                // Taux de dÃ©cÃ©lartion Ã  prendre en compte		

        char                        m_cTypeSortieAcoustique;        // Type de sortie acoustique (C: cellule, P:ponctuel)

        bool                        m_bOffreCvgDeltaN;          // Indique le type de calcul de l'offre des convergents du rÃ©seau

        bool                        m_bCalculTqConvergent;

        // Polution atmosphÃ©riques
        double                      m_dbCptCumCO2;              // Compteur cumulÃ© de CO2 (en g)
        double                      m_dbCptCumNOx;              // Compteur cumulÃ© de NOx (en g)
        double                      m_dbCptCumPM;               // Compteur cumulÃ© de PM (en g)

		unsigned int				m_uiSeed;					// Graine du gÃ©nÃ©rateur alÃ©atoire
        unsigned int                m_uiRandomSeed;             // Graine du gÃ©nÃ©rateur alÃ©atoire tirÃ©e au hasard (m_uiSeed = 0)
        bool                        m_bPickedSeed;              // Indique si le tirage de la graine alÃ©atoire a Ã©tÃ© effectuÃ© ou non

		Affectation*				m_pModuleAffectation;		// Module d'affectation calculÃ©e
		bool						m_bSaveAffectation;			// Sauvegarde des information d'affectation

        // ParamÃ¨tres pour production des donnÃ©es nÃ©cessairesÃ  Sirane (CityDyne)
        int                         m_nNbCellSirane;             // Nombre de cellules par tronÃ§on pour Sirane
        double                      m_dbMinLongueurCellSirane;   // Longueur minimum d'une cellule pour Sirane
        double                      m_dbPeriodeAgregationSirane; // PÃ©riode d'agrÃ©gation pour les cellules Sirane
        double                      m_dbDebutPeriodeSirane;      // Instant de dÃ©marrage de la pÃ©riode Sirane courante
        bool                        m_bExtensionBarycentresSirane; // Permet de gÃ©nÃ©rer des gÃ©omÃ©tries incluant les barycentre des briques amont/aval et sans diviser en cellules

        double                      m_dbTetaLogit;
        int                         m_nMode;
        double                      m_dbWardropTolerance;

        double                      m_dbDijkstraAlpha;          // Pourcentage de pÃ©nalisation
        double                      m_dbDijkstraBeta;           // Facteur multiplicatif de limitation du cout non pÃ©nalisÃ©
        double                      m_dbDijkstraGamma;          // Facteur multiplicatif de limitation du cout pÃ©nalisÃ©
        shortest_path_heuristic     m_eShortestPathHeuristic;   // Heuristique Ã  utiliser pour le calcul des plus courts chemins
        double                      m_dbHeuristicGamma;         // Facteur multiplicatif Ã  appliquer Ã  l'heuristique
        double                      m_dbAStarBeta;              // Facteur beta de l'algorithme A* (podnÃ©ration de la classe fonctionnelle des liens)

        // evolution SymuVia nÂ°114
        bool                        m_bCommonalityFilter;
        double                      m_dbCommonalityAlpha;
        double                      m_dbCommonalityBeta;
        double                      m_dbCommonalityGamma;

        // paramÃ©trage de la pÃ©nalisation des mouvements pour l'affectation
        double                      m_dbRightTurnPenalty;
        double                      m_dbNonPriorityPenalty;
        double                      m_dbAngleToutDroit;
        double                      m_dbRightTurnPenaltyFactor;
        double                      m_dbNonPriorityPenaltyFactor;

        RegulationModule*           m_pRegulationModule;        // Module de rÃ©gulation

        CarFollowingFactory*        m_pCarFollowingFactory;     // Fabrique des lois de poursuite

        std::map<Voie*, boost::shared_ptr<Vehicule>>  m_mapFirstVehicles;   // map contenant le premier vÃ©hicule de chaque voie. variable de travail, pas la peine de sÃ©rialiser.
        double                      m_dbLag;                    // dÃ©calage ("debut" des variations temporelles)

        std::set<TypeVehicule*>     m_setTypesRestitution;      // Types de vÃ©hicules pour lesquels les trajectoires sont restituÃ©es

        // stock les routes (itineraire) d'aprÃ¨s un identifiant de route
        std::map< std::string, std::pair<std::vector<Tuyau *>, Connexion*> > m_routes;

        // instants auquels les itinÃ©raires prÃ©dÃ©finis sont susceptibles d'avoir changÃ© (pour gain de perf)
        std::set<double>            m_LstItiChangeInstants;

        SymuCoreManager            *m_pSymuScriptManager;

        // ParamÃ©trage mode mixte micro / meso
		bool						m_bMeso;				// Indique si la simulation meso est instanciÃ©e (dynamique ou pas) 
        std::vector<TypeVehicule*>  m_MicroVehicleTypes;
        double                      m_dbUpstreamMicroDistance;
        double                      m_dbDownstreamMicroDistance;
        double                      m_dbSidesMicroDistance;

        // Instant de fin d'initialisation de la simulation (pour indication du temps de simulation du trafic dans le log)
        clock_t                     m_initTime;

        // Map de dÃ©finition des noeuds pour lesquels la mÃ©thode SymGetAlternatePathsFromNodeJSON doit renvoyer des itinÃ©raires prÃ©dÃ©finis.
        std::map<Connexion*, std::map<SymuViaTripNode*, std::vector<std::vector<Tuyau *> *> > > m_mapRouteFromNodes;
        std::map<Connexion*, std::map<Connexion*, std::vector<std::vector<Tuyau *> *> > >       m_mapRouteFromNodesToInternalNode;
        std::map<Connexion*, std::map<Tuyau*, std::vector<std::vector<Tuyau *> *> > >           m_mapRouteFromNodesToLink;
        bool                                                                                    m_bUseMapRouteFromNodes;

        ParkingParameters*          m_pParkingParameters;

        // On conserve en mÃ©moire les destinations accessibles en transports en commun depuis chaque parking pour
        // le calcul des itinÃ©raires mixtes pour le SG
        std::map<TripNode*, std::set<Parking*> > m_mapAvailableParkingsForDestination;

        PythonUtils * m_pPythonUtils;
        RandManager * m_pRandManager;
        XMLUtil *     m_pXMLUtil;

#ifdef USE_SYMUCORE
        // Outil de construction du graph SymuCore et des patterns et Penalty costs associÃ©s
        SymuCoreGraphBuilder*       m_pGraphBuilder;

        // ParamÃ¨tres de calcul des temps de parcours mesurÃ©s
        int                         m_nMarginalsDeltaN;
        bool                        m_bUseMarginalsMeanMethod;
        bool                        m_bUseMarginalsMedianMethod;
        int                         m_nbVehiclesForTravelTimes;
        int                         m_nbPeriodsForTravelTimes;
        bool                        m_bUseSpatialTTMethod;
        double                      m_dbMinSpeedForTravelTime;
        double                      m_dbMaxMarginalsValue;
        bool                        m_bUseTravelTimesAsMarginalsInAreas;
        double                      m_dbConcentrationRatioForFreeFlowCriterion;
        bool                        m_bUseLastBusIfAnyForTravelTimes;
        bool                        m_bUseEmptyTravelTimesForBusLines;
        double                      m_dbMeanBusExitTime;
        int                         m_nbStopsToConnectOutsideOfSubAreas;
        double                      m_dbMaxIntermediateWalkRadius;
        double                      m_dbMaxInitialWalkRadius;
        double                      m_dbWalkSpeed;
        bool                        m_bComputeAllCosts;
        double                      m_dbMinLengthForMarginals;

        SymuCore::IUserHandler*     m_pSymuMasterUserHandler;

        SymuCore::MacroType*        m_pVLMacroType;
#endif // USE_SYMUCORE

        bool                        m_bEstimateTrafficLightsWaitTime;

        boost::mutex*               m_pMutex;
		
        

public:		
		OGRCoordinateTransformation *m_pCoordTransf;	// Transformation

		// Gestion des entrÃ©es XML
		XERCES_CPP_NAMESPACE::DOMDocument*	m_XMLDocData;       // Document XML des donnÃ©es initiales de la simulation

        std::string                 m_SimulationID;             // Identifiant de la section simulation active
        std::string                 m_TraficID;                 // Identifiant de la section trafic active
        std::string                 m_ReseauID;                 // Identifiant de la section reseau active

		// Gestion des sorties XML
        std::deque< TimeVariation<TraceDocTrafic> >    m_xmlDocTrafics ;				// Document XML de sortie du trafic
		DocAcoustique		*	m_XmlDocAcoustique;			// Document XML de sortie des Ã©missions acoustiques
		XMLDocEmissionsAtmo		*	m_XmlDocAtmo;               // Document XML de sortie des Ã©missions atmosphÃ©riques
        XMLDocSirane            *   m_XmlDocSirane;             // Document XML de sortie des cellules pour Sirane (CityDyne)
        CSVOutputWriter         *   m_CSVOutputWriter;          // Writer pour les fichiers CSV
		XMLReaderTrafic*			m_XmlReaderTrafic;			// Objet permettant d'exploiter le fichier XML de trafic  
        std::string					m_strFileToLoadState;       // Fichier pour le chargement du rÃ©seaux SymuVia

        TravelTimesOutputManager*   m_pTravelTimesOutputManager; // Gestion de la production des temps de parcours au format JSON

        Logger*                     m_pLogger;             // gestionnaire du fichier de log de simulation
        Logger*                     m_pLoadingLogger;      // gestionnaire du fichier de log de chargement (non nul uniquement pendant le chargement)
        std::ofstream*  	        g_FicDebug;			   // fichier de debug  
        std::ofstream*          	FicTraficSAS;          // fichiers de simulation trafic SAS (tuyau microscopique)

        std::vector<double>         m_LstInstantsSauvegardes; // Liste des instants pour la sauvegardes de SymuVia
        bool                        m_bLightSavingMode;       // est vraie si la sauvegarde de SymuVia se fait a partir d'un fichier alÃ©ger de toutes donnÃ©es non essentielles
        
        bool                        m_bLogFile;         // est vraie si l'application doit rendre des fichiers de log, faux sinon
        bool                        m_bLogFileLight;    // si vrai, on ne sort que les messages de type avertissement ou erreur
        bool                        m_bFichierSAS;      // PrÃ©sence d'un fichier SAS

		int							m_nIncCnx;			// IncrÃ©ment des identifiants numÃ©riques des connexions
        
        // Variables liÃ©es Ã  la simulation microscopique
		std::deque<boost::shared_ptr<Vehicule>> m_LstInitVehicule;     // Liste initiale des vÃ©hicules de la modÃ©lisation microscopique
        std::vector<boost::shared_ptr<Vehicule>> m_LstVehicles;		 // Liste dynamique des vÃ©hicules de la modÃ©lisation microscopique

        bool                        m_bDriven;                  // Indique si un vÃ©hicule a Ã©tÃ© guidÃ© et donc les initialisations
                                                                // nÃ©cessaires aux calculs dejÃ  faites

        bool                        m_bSymuMasterMode;          // Mode SymuMaster (pas de calcul d'itinÃ©raires, d'affectation dynamique, de crÃ©ation de vÃ©hicules.
        bool                        m_bWithPublicTransportGraph;// Mode de construction du graph symucore incluant les transports publics, pour pouvoir faire des calculs d'itinÃ©raires multimodaux (pour le SG par ex.)

#ifdef USE_SYMUCOM
        Simulator*                  m_pSymucomSimulator;
#endif // USE_SYMUCOM


        // Constructeur, destructeur et assimilÃ©
        ~Reseau(void); // destructeur
        Reseau(void); // constructeur par dÃ©faut
        Reseau(std::string) ; // constructeur (Nom fichier de log)
        void vidange_listes(void);      // complement du destructeur, vide les listes (les deques)

		// Chargement des donnÃ©es XML
        bool LoadReseauXML(const std::string & sXMLFile, const std::string & sScenario, const std::string & sOutDir) ;											// Chargement du fichier d'entrÃ©e au format XML
		bool LoadMouvementsAutorises(Connexion *pCnx, XERCES_CPP_NAMESPACE::DOMNode *xmlNodeMvtsAutorises, Logger * pLogger);
		bool LoadDataPlanDeFeux(ControleurDeFeux *pCtrDeFeux, PlanDeFeux *pPDF, XERCES_CPP_NAMESPACE::DOMNode *pXMLPlansDeFeux, Logger *pChargement);
        bool LoadDemandesAgressivite(SymuViaTripNode * pOrigine, XERCES_CPP_NAMESPACE::DOMNode *pXMLTraficOrigine, double dbLag, Logger * pChargement);  // Chargement de la demande et agressivitÃ© d'une origine
        bool LoadCreationRepartitions(SymuViaTripNode * pOrigine, XERCES_CPP_NAMESPACE::DOMNode *pXMLTraficOrigine, Logger * pChargement);  // Chargement des paramÃ¨tres de crÃ©ation et de rÃ©partition des vÃ©hicules crÃ©Ã©s
        bool LoadRepTypeVehicules(XERCES_CPP_NAMESPACE::DOMNode *pXMLNode, const std::string & strElement, int nbCoeffs, const std::vector<TypeVehicule*> & lstTypes, RepartitionTypeVehicule * pRepTypesVeh, Logger * pChargement); // Chargement d'une rÃ©partition des types de vÃ©hicules

        // Fonctions de gestion de la simulation
        bool Execution();         // Fonction lancant les diffÃ©rents phases de l'execution de la simulation (Nom du fichier de dÃ©finition du rÃ©seau)

		bool InitSimuTrafic(bool bLastReplication = true) ;					                // initialisation de la simulation du trafic
		bool InitSimuEmissions(bool bAcoustic, bool bAir, bool bSirane, bool bOnlyCells = false) ;		// initialisation de la simulation de l'acoustique

		void FinSimuTrafic() ;					// initialisation de la simulation du trafic
		void FinSimuEmissions(bool bAcoustic, bool bAir, bool bSirane) ;				// initialisation de la simulation de l'acoustique

        bool	SimuTrafic(bool & bStepCompleted);			                    // lance les diffÃ©rentes Ã©tapes de simulation du trafic pour un pas de temps. Renvoie vrai quand la simulation est terminÃ©e

        //ETS 130731
        bool InitSimuTraficMeso();
        void SimuTraficMeso();
        void SimuTraficMicroMacro();
		bool	PostTimeStepSimulation();					// Fin de calcul d'un pas de temps d'une simulation
		bool	SimuEmissions(bool bAcoustic, bool bAir, bool bSirane);	// lance les diffÃ©rentes Ã©tapes de simulation des Ã©missions (acoustiques et/ou atmosphÃ©riques et/ou Sirane) pour un pas de temps. Renvoie vrai quand la simulation est terminÃ©e
		bool	LoadTrafic();		                        // chargement des variables du trafic au pas de temps courant

        // Fonctions spÃ©cifiques Ã  l'intÃ©gration EVE                
		void GenReseauCirculationFile (eveShared::EveNetwork * &pNetwork);    // GÃ©nÃ©ration du fichier 'rÃ©seau circulation'
        bool UpdateReseau (const std::string& ) ;                            // Chargement du fichier de modification au format XML        
		std::string GetLibelleTronconEve (Tuyau *pT, int nVoie);


        // Fonctions de renvoi des variables du rÃ©seau
		std::string	GetNetwork(bool bUseJSON = false);
        std::string GetParkingInfos();
        std::string GetNodeVehicles() const; // ETS 140829 creation
        bool WriteTempusFiles();       // ecriture des fichiers pour import dans la BDD tempus
#ifdef USE_SYMUCORE
        boost::posix_time::ptime GetSimulationStartTime();
        bool BuildGraph(SymuCore::MultiLayersGraph * pGraph, bool bIsPrimaryGraph);  // Construction d'un graph au format SymuCore Ã  partir du rÃ©seau
        bool FillPopulations(SymuCore::MultiLayersGraph * pGraph, SymuCore::Populations &populations,
            const boost::posix_time::ptime & startSimulationTime, const boost::posix_time::ptime & endSimulationTime, bool bIgnoreSubAreas,
            std::vector<SymuCore::Trip> & lstTrips);
        bool FillShortestPathParameters(bool bFillKParameters, bool bUseCommonalityFilter, double& dbAssignementAlpha, double& dbAssignementBeta, double& dbAssignementGamma,
                                        std::vector<double>& dbCommonalityFactorParameters, double &dbWardropTolerance,
                                        std::map<SymuCore::SubPopulation*, std::map<SymuCore::Origin*, std::map<SymuCore::Destination*, SymuCore::ListTimeFrame<double> > > >& KByOD);
        bool FillLogitParameters(std::map<SymuCore::SubPopulation*, std::map<SymuCore::Origin*, std::map<SymuCore::Destination*, SymuCore::ListTimeFrame<double> > > >& LogitByOD);
        bool ComputeCosts(const boost::posix_time::ptime & startTime, const boost::posix_time::ptime & endTime, const std::vector<SymuCore::MacroType*> & macroTypes, const std::map<SymuCore::SubPopulation*, SymuCore::CostFunction> & mapCostFunctions);

        bool GetOffers(const std::vector<std::string> & inputNames, std::vector<double> & offerValuesVect);
        bool SetCapacities(const std::string & exitName, const std::string & downstreamRouteName, double dbCapacityValue);
        double ComputeOffer(SymuViaTripNode * pInput, Tuyau * pLink);

        TypeVehicule* GetVehiculeTypeFromMacro(SymuCore::MacroType* pMacroType);
        SymuCore::MacroType * GetMacroTypeForVLs();

        void SetSymuMasterUserHandler(SymuCore::IUserHandler* pUserHandler) { m_pSymuMasterUserHandler = pUserHandler; }
        SymuCore::IUserHandler* GetSymuMasterUserHandler() { return m_pSymuMasterUserHandler; }

        int  GetMarginalsDeltaN() const { return m_nMarginalsDeltaN; }
        void SetMarginalsDeltaN(int iValue) { m_nMarginalsDeltaN = iValue; }

        bool UseMarginalsMeanMethod() { return m_bUseMarginalsMeanMethod; }
        void SetUseMarginalsMeanMethod(bool bValue) { m_bUseMarginalsMeanMethod = bValue; }

        bool UseMarginalsMedianMethod() { return m_bUseMarginalsMedianMethod; }
        void SetUseMarginalsMedianMethod(bool bValue) { m_bUseMarginalsMedianMethod = bValue; }

        int  GetNbVehiclesForTravelTimes() { return m_nbVehiclesForTravelTimes; }
        void SetNbVehiclesForTravelTimes(int iValue) { m_nbVehiclesForTravelTimes = iValue; }

        int  GetNbPeriodsForTravelTimes() { return m_nbPeriodsForTravelTimes; }
        void SetNbPeriodsForTravelTimes(int iValue) { m_nbPeriodsForTravelTimes = iValue; }

        bool UseSpatialTTMethod() { return m_bUseSpatialTTMethod; }
        void SetUseSpatialTTMethod(bool bValue);

        double GetMinSpeedForTravelTime() { return m_dbMinSpeedForTravelTime; }
        void SetMinSpeedForTravelTime(double dValue) { m_dbMinSpeedForTravelTime = dValue; }

        double GetMaxMarginalsValue() { return m_dbMaxMarginalsValue; }
        void SetMaxMarginalsValue(double dValue) { m_dbMaxMarginalsValue = dValue; }

        bool UseTravelTimesAsMarginalsInAreas() { return m_bUseTravelTimesAsMarginalsInAreas; }
        void SetUseTravelTimesAsMarginalsInAreas(bool bValue) { m_bUseTravelTimesAsMarginalsInAreas = bValue; }

        double GetConcentrationRatioForFreeFlowCriterion() { return m_dbConcentrationRatioForFreeFlowCriterion; }
        void SetConcentrationRatioForFreeFlowCriterion(double dbValue) { m_dbConcentrationRatioForFreeFlowCriterion = dbValue; }

        bool UseLastBusIfAnyForTravelTimes() { return m_bUseLastBusIfAnyForTravelTimes; }
        void SetUseLastBusIfAnyForTravelTimes(bool bValue) { m_bUseLastBusIfAnyForTravelTimes = bValue; }

        bool UseEmptyTravelTimesForBusLines() { return m_bUseEmptyTravelTimesForBusLines; }
        void SetUseEmptyTravelTimesForBusLines(bool bValue) { m_bUseEmptyTravelTimesForBusLines = bValue; }

        double GetMeanBusExitTime() { return m_dbMeanBusExitTime; }
        void SetMeanBusExitTime(double dbMeanBusExitTime) { m_dbMeanBusExitTime = dbMeanBusExitTime; }

        int GetNbStopsToConnectOutsideOfSubAreas() { return m_nbStopsToConnectOutsideOfSubAreas; }
        void SetNbStopsToConnectOutsideOfSubAreas(int nbStopsToConnectOutsideOfSubAreas) { m_nbStopsToConnectOutsideOfSubAreas = nbStopsToConnectOutsideOfSubAreas; }

        double GetMaxIntermediateWalkRadius() { return m_dbMaxIntermediateWalkRadius; }
        void SetMaxIntermediateWalkRadius(double dbWalkRadius) { m_dbMaxIntermediateWalkRadius = dbWalkRadius; }

        void SetComputeAllCosts(bool bComputeAllCosts) { m_bComputeAllCosts = bComputeAllCosts; }
        bool GetComputeAllCosts() { return m_bComputeAllCosts; }

        double GetMinLengthForMarginals() { return m_dbMinLengthForMarginals; }
        void SetMinLengthForMarginals(double dbValue) { m_dbMinLengthForMarginals = dbValue; }

        double GetMaxInitialWalkRadius() { return m_dbMaxInitialWalkRadius; }
        void SetMaxInitialWalkRadius(double dbWalkRadius) { m_dbMaxInitialWalkRadius = dbWalkRadius; }

        double GetWalkSpeed() { return m_dbWalkSpeed; }
        void SetWalkSpeed(double dbWalkSpeed) { m_dbWalkSpeed = dbWalkSpeed; }
#endif // USE_SYMUCORE

        void GetPlaqueFromID(const std::string & plaqueId, SymuViaTripNode * & pParentZone, CPlaque * & pPlaque);

        bool EstimateTrafficLightsWaitTime() { return m_bEstimateTrafficLightsWaitTime; }
        void SetEstimateTrafficLightsWaitTime(bool bValue) { m_bEstimateTrafficLightsWaitTime = bValue; }

        double	GetTimeStep(void);     // retourne la valeur d'un pas de temps

        const std::string & GetOutputDir(){return m_sOutputDir;}
		const std::string & GetPrefixOutputFiles(){return m_sPrefixOutputFiles;}
        const std::string & GetSuffixOutputFiles(){ return m_sSuffixOutputFiles; }
        void SetSuffixOutputFiles(const std::string & strSuffix) { m_sSuffixOutputFiles = strSuffix; }

        bool    IsTraceRoute(){return m_bTraceRoute;}

        bool    IsVGPDebug() {return m_bVGPDebug;}

        bool    IsCSVOutput() {return m_bCSVOutput;}
        CSVOutputWriter * GetCSVWriter() {return m_CSVOutputWriter;}

        double  GetTravelTimesOutputPeriod() { return m_dbTravelTimesOutputPeriod; }
        TravelTimesOutputManager * GetTravelTimesOutputManager() { return m_pTravelTimesOutputManager; }

        bool    IsGMLOutput() {return m_bGMLOutput;}

        // Fonctions utilitaires     

        float getDeltaAccel(void);                  // retourne la valeur de l'Ã©cart en accÃ©lÃ©ration  // MQ 26/10/04 : Ajou
        float getDeltaVitesse(void);                // retourne la valeur de l'Ã©cart en vitesse       // MQ 26/10/04 : Ajou

		bool    VerifieVersionFichier(std::ifstream *Fic);    // VÃ©rifie la version du fichier

        bool    IsCptDirectionnel(){return m_bCptDirectionnel;}
        bool    IsCptItineraire()const {return m_bCptItineraire;}
        bool    IsCptDestination(){return m_bCptDestination;}
        bool    IsAffectationDynamique() {return m_bAffectationDynamique;}

		bool	CreateCoordinateTransformation(int sEPSGinput, int sEPSGoutput);	// CrÃ©ation de la fonction de transformation des coordonnÃ©es

        // gestion des DM
        void            CalculVitessePositionDM();

        // Gestion spÃ©cifiques des DMF
        void            MiseAJourListeDMF();
        void            CalculContributionDM();     // Calcule la contribution des DMF

        // Gestion spÃ©cifiques des DM type bus
        void                            GestionDiscontinuitesMobiles(double dbPasTemps);

        // Gestion des vehicules

        // Utile        
        boost::shared_ptr<Vehicule>     GetVehiculeFromID(int nID);
        boost::shared_ptr<Vehicule>     GetVehiculeFromIDThreadSafe(int nID);
        std::deque<TypeVehicule*>       GetLstTypesVehicule(){return m_LstTypesVehicule;}
        boost::shared_ptr<Vehicule>     GetPrevVehicule(Vehicule* pVeh, bool bSearchNextVoie = false, bool bExludeVehiculeDepasse = false);
        void                            ComputeFirstVehicules();
        boost::shared_ptr<Vehicule>     GetFirstVehicule(VoieMicro*  pVoie);
        boost::shared_ptr<Vehicule>     GetFirstVehiculePrecalc(Voie*  pVoie);
        boost::shared_ptr<Vehicule>     GetLastVehicule (VoieMicro*  pVoie, Vehicule * pVehiculeRef = NULL);
        boost::shared_ptr<Vehicule>     GetNearAmontVehicule(VoieMicro*    pVoie, double dbPos, double dbPosMax = -1, Vehicule * pVeh = NULL);
        boost::shared_ptr<Vehicule>     GetNearAmontVehiculeEx(Tuyau* pTa, double dbPos, Tuyau* pTIti, double dbDst, int nVoie = 0);
        boost::shared_ptr<Vehicule>     GetNearAvalVehicule(VoieMicro* pVoie, double dbPos, Vehicule * pVeh = NULL, bool bIgnoreVehiculesenDepassement = false);
        int                             IncLastIdVeh(){return m_nLastVehicleID++;}
        double                          GetFAlphaMandatory(){return m_dbFAlphaMandatory;}
        double                          GetRelaxation(){return m_dbRelaxation;}

        bool                            IsDebugOD(){return m_bDebugOD;}
        bool                            IsDebug(){return m_bDebug;}
        bool                            IsTraceStocks(){return m_bTraceStocks;}
        bool                            IsSortieLight(){return m_bSortieLight;}
        bool                            IsSortieRegime(){return m_bSortieRegime;}
        
        
		
        // Simulation
        void                            MiseAJourVehicules(double dbInstant);
        void                            FinCalculVehicules(double dbInst);
        void                            AddVehicule(boost::shared_ptr<Vehicule> pVehicule);
        void                            InitVehicules(double dbInstant);
        void                            SupprimeVehicules(double dbInstant);
        bool                            IsAccBornee(){return m_bAccBornee;}
        bool                            IsChgtVoie(){return m_bChgtVoie;}
        char							GetChgtVoieMandatoryProba(){return m_cChgtVoieMandatoryProba;}
        char							GetChgtVoieMandatoryMode(){return m_cChgtVoieMandatoryMode;}
        void                            SetRelancerCalculRepartiteur(){m_bRelancerCalculRepartiteur=true;}
        void                            UpdateConvergents();

        void                            ChangementDeVoie(double dbInstant);     

        void                            Depassement(double dbInstant);      

        int                             IncLastIdSegment(){return ++m_nLastIdSegment;}
        int                             GetLastIdVeh(){return m_nLastVehicleID;}

        double                          GetDistanceEntreVehicules(Vehicule *pVeh1, Vehicule* pVeh2, bool bFin = false) ;

        bool                            IsUsedODMatrix(){return m_bCptDestination || m_bCptItineraire;}

        Tuyau*                          GetTuyauReliant(Tuyau *pTAm, Tuyau *pTAv);

		Tuyau*							GetLinkFromLabel(const std::string & ssTuyau);
		ConnectionPonctuel*	            GetConnectionFromID(const std::string & ssCnx, char &cTypeCnx);
        SymuViaTripNode*                GetOrigineFromID(const std::string & ssCnx, char &cTypeCnx);
        SymuViaTripNode*                GetDestinationFromID(const std::string & ssCnx, char &cTypeCnx);
		TypeVehicule*                   GetVehicleTypeFromID(const std::string & ssTuyau);        
		ControleurDeFeux*	            GetTrafficLightControllerFromID(const std::string & ssID);
        CMotif*                         GetMotifFromID(const std::string & ssMotif);
        Parking*                        GetParkingFromID(const std::string & ssParkingID);

        std::deque<TuyauMicro*>*        GetLstTuyauxMicro(){return &m_LstTuyauxMicro;}
        std::deque<CTuyauMeso*>*        GetLstTuyauxMeso(){return &m_LstTuyauxMeso;}
        std::deque<Tuyau*>*             GetLstTuyaux(){return &m_LstTuyaux;}

        const std::map<std::string, Convergent*>&  GetLstConvergents(){return Liste_convergents;}
        const std::map<std::string, Divergent*>&   GetLstDivergents(){return Liste_divergents;}
        std::map<std::string, Repartiteur*>*       GetLstRepartiteurs(){return &Liste_repartiteurs;}
        std::deque<SymuViaTripNode*>    GetLstOrigines(){return Liste_origines;}
        std::deque<SymuViaTripNode*>    GetLstDestinations(){return Liste_destinations;}
        std::deque<ControleurDeFeux*>   GetLstControleursDeFeux(){return Liste_ControleursDeFeux;}

        TuyauMicro*                     GetTuyauMicro(int i){return m_LstTuyauxMicro[i];}

        // Renvoie le SymuViaTripNode dont la sortie sur le rÃ©seau correspond Ã  la connexion passÃ©e en paramÃ¨tre
        SymuViaTripNode*                GetOriginSymuViaTripNode(Connexion * pConnexion);

        // Renvoie le SymuViaTripNode dont l'entrÃ©e sur le rÃ©seau correspond Ã  la connexion passÃ©e en paramÃ¨tre
        SymuViaTripNode*                GetDestinationSymuViaTripNode(Connexion * pConnexion);

        SegmentMacro*                   GetSegmentFromID(int nID);

		Voie*							GetVoieFromID(const std::string & sTuyauID, int NumVoie);

        // Fonction de calcul du numÃ©ro de voie d'un vÃ©hicule qui sort du mode mÃ©so
        int                             ComputeAvailableLaneNumber(Vehicule * pVeh, TuyauMicro * pLink, Tuyau * pPrevLink, double dbWishedPos, std::map<int, double> * mapLastVehPosOnEachLane, double & dbAvailablePos);


        int                             GetInstSim(){return m_nInstSim;}

        double                          GetKxmin(){return m_dbKxmin;}
        double                          GetMaxDebitMax(){return m_dbMaxDebitMax;}
        double                          GetMaxVitMax();           
        double                          GetMinKv();
        double                          GetMainKx();
        boost::shared_ptr<Vehicule>     GetVehiculeAmont(TuyauMicro* pTAv, double dbDst, double dbDstCum, double dbInstant);

		BriqueDeConnexion*              GetBrickFromID(const std::string & ssBriqueID);

        void                            AddConvergent(const std::string & strId, Convergent *pC){Liste_convergents[strId] = pC;}
        void                            RemoveConvergent(Convergent *pC);
        void                            AddDivergent(const std::string & strId, Divergent *pD){Liste_divergents[strId] = pD;}

        double                          GetDureeSimu(){return m_dbDureeSimu;}

        bool                            IsInitSimuTrafic(){return m_bInitSimuTrafic;}
        bool                            IsInitSimuEmissions(){return m_bInitSimuEmissions;}

        bool                            IsSortieTraj(){return m_bSortieTraj;}
        bool                            DoSortieTraj(){ return m_bSortieTraj  && !m_bDisableTrajectoriesOutput; }
        void							SetSortieTraj(bool bSortieTraj){ m_bSortieTraj = bSortieTraj; }

        void                            SetDisableTrajectoriesOutput(bool bDisableTrajectoriesOutput) { m_bDisableTrajectoriesOutput = bDisableTrajectoriesOutput; }

		void							SetSymMode(SymMode Mode) { m_SymMode = Mode; }
		SymMode							GetSymMode() { return m_SymMode; }

        void							SetXmlOutput(bool bXmlOutput){m_bXmlOutput = bXmlOutput;}
        bool							IsXmlOutput(){return m_bXmlOutput;}

        void                            SetSimuTrafic(bool bSimuTrafic){m_bSimuTrafic = bSimuTrafic;}
        void                            SetSimuAcoustique(bool bSimuAcous){m_bSimuAcoustique = bSimuAcous;}
        void                            SetSimuAir(bool bSimuAir){m_bSimuAir = bSimuAir;}
        void                            SetSimuSirane(bool bSimuSirane){m_bSimuSirane= bSimuSirane;}

        bool                            IsSimuAcoustique(){return m_bSimuAcoustique;}
        bool                            IsSimuTrafic(){return m_bSimuTrafic;}
        bool                            IsSimuAir(){return m_bSimuAir;}
        bool                            IsSimuSirane(){return m_bSimuSirane;}

        bool							IsProcAgressivite(){return m_bProcAgressivite;}

        bool                            IsSortieAcoustiquePonctuel(){return m_cTypeSortieAcoustique=='P';}

		double                          LoadLagOfVariation(XERCES_CPP_NAMESPACE::DOMNode *pXmlNodeVar, Logger & logger);

        bool                            IsLoiPoursuiteOld(){return m_bLoiPoursuiteOld;}
		const std::string &             GetLoipoursuite() {return m_sLoipoursuite;}

        bool                            IsOffreCvgDeltaN(){return m_bOffreCvgDeltaN;}
        bool                            IsCalculTqConvergent(){return m_bCalculTqConvergent;}
        
        STime&							GetSimuStartTime(){return m_dtDebutSimu;}
        STime&							GetSimuEndTime(){return m_dtFinSimu;}
        SDateTime&						GetDateSimulation() {return m_dtDateSimulation;}

		// Pilotage
		int                             SendSignalPlan(const std::string & sCDF, const std::string & sSP);
		int                             SendSpeedLimit(const std::string & sSection, const std::string & sVehType, int numVoie, double dbDebutPortion, double dbFinPortion, double dbSpeedLimit);

        void                            GenerateAssignmentNetwork();

        double                          GetDstChgtVoie(){return m_dbDstChgtVoie;}

        bool                            CalculTraversee(Vehicule *pVehEnAttente, std::vector<int> & vehiculeIDs, PtConflitTraversee *pPtCT, GrpPtsConflitTraversee  *pGrpPtsCT,  double dbTf, double dbInstant, bool bDebutPasTemps = false, double dbTt = 0.0);

		bool							HasVehicule( VoieMicro* pV );

		void							ProcedureAgressivite(double dbInstant);
 
		void							LoadInitVehs( XERCES_CPP_NAMESPACE::DOMNode *pXmlNodeInit, Logger & logger);      

		Affectation*					GetAffectationModule(){return this->m_pModuleAffectation;}

        RegulationModule*               GetModuleRegulation() {return m_pRegulationModule;}

        PythonUtils*                    GetPythonUtils();
        RandManager*                    GetRandManager() { return m_pRandManager; }
        XMLUtil*                        GetXMLUtil() { return m_pXMLUtil; }

        CarFollowingFactory*            GetCarFollowingFactory() {return m_pCarFollowingFactory;}

		std::deque<TraceDocTrafic*>		GetXmlDocTrafic() const;

		SensorsManagers*				GetGestionsCapteur(){return m_pGestionsCapteur;}

		std::string						GetTitre() { return m_strTitre; }

		bool							IsChgtVoieGhost(){return m_bChgtVoieGhost;}		
		double							GetChgtVoieGhostDurationMin(){return m_nChgtVoieGhostDurationMin;}
		double							GetChgtVoieGhostDurationMax(){return m_nChgtVoieGhostDurationMax;}
		double							GetChgtVoieGhostBevelLength(){return m_dbChgtVoieGhostBevelLength;}

		void							SortieIndicateursInterDistance();

        int                             GetNbCellSirane() {return m_nNbCellSirane;}
        double                          GetMinLongueurCellSirane() {return m_dbMinLongueurCellSirane;}
        bool                            GetExtensionBarycentresSirane() {return m_bExtensionBarycentresSirane;}

// Pilotage
        int								CreateVehicle(const std::string & sType, const std::string & sEntree, const std::string & sSortie, int nVoie, double dbt, std::deque<std::string> * pIti, const std::string & junctionName, int externalUserID = -1, std::string nextRouteId = std::string());
        int                             CreatePublicTransportUser(const std::string & startStop, const std::string & endStop, const std::string & lineName, double dbt, int externalUserID);
		int								DeleteVehicle(int nID);
		int								DriveVehicle(int nID, const std::string & sTroncon, int nVoie, double dbPos, bool bForce);
		int								AlterRoute(int nIdVeh, const std::deque<std::string> & dqLinks);

        int                             SetRoutes(const std::string & originId, const std::string & destinationId, const std::string & typeVeh, std::vector<std::pair<double, std::vector<std::string> > > & routes);

        int                             ComputeRoutes(const std::string & originId, const std::string & destinationId, const std::string & typeVeh, double dbInstant, int nbShortestPaths,
                                                      int method, const std::vector<Tuyau*> & refPath, const std::vector<Tuyau*> & linksToAvoid,
                                                      std::vector<std::pair<double, std::vector<std::string> > > &paths,
                                                      double & refPathCost);

        int                             ComputeRoutesPublicTransport(const std::string & originId, const std::string & currentLine, const std::string & nextLine, const std::string & destinationId, double dbInstant, int nbShortestPaths,
                                                                     std::vector<std::pair<double, std::vector<std::string> > > &paths);

        int                             ComputeMixedRoutes(const std::string & originId, const std::string & destinationId, const std::string & typeVeh, double dbInstant, int nbShortestPaths, std::map<Parking*, std::vector<std::vector<std::string> > >  & paths);

        bool                            GetPredefinedAlternativeRoutes(TypeVehicule * pTypeVehicule, Tuyau * pDownLink, SymuViaTripNode * pDestinationNode, Tuyau * pDestinationLink, Connexion * pDestinationCon,
                                                                       std::vector<std::pair<double, std::vector<std::string> > > & paths);

        void                            InvalidatePathRelatedCaches();

        // CrÃ©ation sur ordre d'un vÃ©hicule pour un trip et une flotte donnÃ©s
        int                             CreateVehicle(Trip * pTrip, AbstractFleet * pFleet);

        // Ajout/Suppression d'un point de livraison Ã  une tournÃ©e existante
        int                             AddDeliveryPoint(Trip * pTournee, int vehiculeId, PointDeLivraison * pPoint, int positionIndex, int dechargement, int chargement);
        int                             RemoveDeliveryPoint(Trip * pTournee, int vehiculeId, int positionIndex);

        // Renvoie la rectangle englobant du rÃ©seau
        std::vector<Point>              GetBoundingRect();

        double                          GetTetaLogit() {return m_dbTetaLogit;}
        int                             GetMode() {return m_nMode;}
        double                          GetWardropTolerance() {return m_dbWardropTolerance;}
        double                          GetDijkstraAlpha() {return m_dbDijkstraAlpha;}
        double                          GetDijkstraBeta() {return m_dbDijkstraBeta;}
        double                          GetDijkstraGamma() {return m_dbDijkstraGamma;}
        shortest_path_heuristic         GetShortestPathHeuristic() { return m_eShortestPathHeuristic; }
        double                          GetHeuristicGamma() { return m_dbHeuristicGamma; }
        double                          GetAStarBeta() { return m_dbAStarBeta; }
        

        bool                            GetCommonalityFilter() {return m_bCommonalityFilter;}
        double                          GetCommonalityAlpha() {return m_dbCommonalityAlpha;}
        double                          GetCommonalityBeta() {return m_dbCommonalityBeta;}
        double                          GetCommonalityGamma() {return m_dbCommonalityGamma;}

        double                          GetRightTurnPenalty() {return m_dbRightTurnPenalty;}
        double                          GetNonPriorityPenalty() {return m_dbNonPriorityPenalty;}
        double                          GetAngleToutDroit() {return m_dbAngleToutDroit;}
        double                          GetRightTurnPenaltyFactor() {return m_dbRightTurnPenaltyFactor;}
        double                          GetNonPriorityPenaltyFactor() {return m_dbNonPriorityPenaltyFactor;}

        bool                            GetSimulationTraversees() {return m_bTraversees;}

        bool                            IsSymuMasterMode() { return m_bSymuMasterMode; }
        void                            SetSymuMasterMode() { m_bSymuMasterMode = true; }

        bool                            IsWithPublicTransportGraph() { return m_bWithPublicTransportGraph; }
        void                            SetWithPublicTransportGraph() { m_bWithPublicTransportGraph = true; }

        bool                            IsModeDepassementChgtDir() { return m_bModeDepassementChgtDir; }
        double                          GetDistanceDepassementChgtDir() { return m_dbDistDepassementChgtDir; }
        
        // Construit la liste complÃ¨te des tuyaux (incluant les tronÃ§ons internes) correspondant Ã  l'itinÃ©raire spÃ©cifiÃ©
        std::vector<Tuyau*>             ComputeItineraireComplet(const std::vector<Tuyau*> & itineaire);

        void                            InitLogFicSimulation();
        void                            RemoveLogFicSimulation();
        Logger&                         log();
        Logger*                         GetLogger();

        double                          GetLag() {return m_dbLag;}

        void ChangeMesoNodeEventTime(CMesoNode * pNode, double dNewTime, Tuyau * pTuyauOutgoing);

        //! Sauvegarde les informations du controleur de feux
        void SaveControleurDeFeux(const std::deque<TraceDocTrafic *> &xmlDocTrafics);

        //! Repositionne le gestionnaire des tirages alÃ©atoire Ã  une position dÃ©finie pour reproductibilitÃ© en cas de sÃ©rialisation/dÃ©sÃ©rialisation, ou d'affectation convergente
        void RestoreSeed(unsigned int randCount);

        //! Accesseur au wrapper SymuScript
        SymuCoreManager*                GetSymuScript() { return m_pSymuScriptManager; }

        // Renvoie la flotte historique de SymuVia
        SymuViaFleet*                   GetSymuViaFleet();

        // Renvoie la flotte des transports guidÃ©s
        PublicTransportFleet*           GetPublicTransportFleet();

        // Renvoie la flotte des engins de livraison
        DeliveryFleet*                  GetDeliveryFleet();
        

private:
    void Initialize();
	eveShared::EveTroncon * CreateEveTroncon(const std::string &strID, const Point& PtAm, const Point& PtAv, const std::deque<Point*>& lstPtInterne, double dbLarg, double dbLong, const std::string& sSymTroncon, int nSymVoie, std::vector<ZoneZLevel> nzlevels, bool bTronconInterne);
    void UpdateVitessesReg();
    // renvoie l'ID du troncon prioritaire du convergent d'insertion
    std::string GetTronconAmontPrio(XERCES_CPP_NAMESPACE::DOMNode * cvtInsertionNode, Logger & logger); 
    // renvoie le tronÃ§on non prioritaire associÃ©e Ã  une voie d'insertion Ã  partir du rÃ©partiteur amont du troncon d'insertion
    Tuyau * GetTronconAmontNonPrio(int nVoieInsertion, Repartiteur * pRepartiteur);
    // Construit la rÃ©partition des flux associÃ©e Ã  une connexion
    bool BuildRepartitionFlux(Connexion * pConnexion, XERCES_CPP_NAMESPACE::DOMDocument* pDocument, Logger *pChargement);
    // Construit la liste des plages temporelles dÃ©finies dans le fichier scÃ©nario
    void BuildPlagesTemporelles(XERCES_CPP_NAMESPACE::DOMNode * pDOMNodePlagesTemporelles, Logger & loadingLogger);
    // Renvoie le dernier (follower) vÃ©hicule de la liste (ils se trouvent tous Ã  la mÃªme position)
    boost::shared_ptr<Vehicule> GetLastVehicule(std::vector<boost::shared_ptr<Vehicule> > & vehicules);
    // Renvoie le premier (leader) vÃ©hicule de la liste (ils se trouvent tous Ã  la mÃªme position)
    boost::shared_ptr<Vehicule> GetFirstVehicule(std::vector<boost::shared_ptr<Vehicule> > & vehicules);

    // Genere les vehicules aux diffÃ©rentes origines
    bool prvGenerateVehicule();

    // ProcÃ©dure de dÃ©passement Charlotte
    void ProcedureDepassementChgtDirection();

    // MESOSCOPIQUE
    std::deque<CMesoNode*> m_nodeToUpdate;
    //! Enregistrement des temps d'Ã©venement pour le calcul mÃ©soscopique
    std::vector<double > m_recordEventTimes;

    //! CrÃ©Ã©e une demande depuis un noeud Flux/demande
    bool LoadDemandeFromFluxNode(SymuViaTripNode * pOrigine, double dbLag, Logger *pChargement,
                            XERCES_CPP_NAMESPACE::DOMNode * pFluxNode, TypeVehicule * pTypeVehicle);

    //! CrÃ©Ã©e les donnÃ©es contenue dans un fluc d'origine
    bool LoadFluxFromNode(SymuViaTripNode * pOrigine, Logger * pChargement,
                 XERCES_CPP_NAMESPACE::DOMNode *pFluxNode, TypeVehicule * pTypeVehicle);

    //! Switch des tronÃ§ons micro vers meso ou inversement
    void SwitchMicroMeso();

    //! Calcul des tuyaux permettant d'atteindre le tuyau pLink avec une distance infÃ©rieure Ã  micro_distance_cotes
    void AppendSideLinks(Tuyau * pLink, std::set<Tuyau*> & lstLinksToMicro, bool bUpstream, bool bDownstream);

    //! Fonction utilitaire pour le dÃ©codage d'Ã©lÃ©ments dÃ©finis par une position de dÃ©but et de fin sur un trnoÃ§on (avec position relative Ã©ventuelle)
    bool GetSectionFromNode(const std::string & strID, XERCES_CPP_NAMESPACE::DOMNode * pXMLNode, Tuyau * pTuyau, double & dbTmp, double & dbTmp2, Logger* pLogger);

    //! PrÃ©pare les vecteurs Ã  passer au constructeur des capteurs MFD
    void PrepareLinksForMFD(const std::vector<Tuyau*> & zoneLinks, Logger * pLoadingLogger, const std::string & sensorName,
        std::vector<Tuyau*> & Tuyaux, std::vector<double> & dbPosDebut, std::vector<double> & dbPosFin, std::vector<int> & eTuyauType);

    //! Nombre de rÃ©plications Ã  exÃ©cuter
    int m_nReplications;
   
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SÃ©rialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version);
};

#pragma warning(pop)

#endif
