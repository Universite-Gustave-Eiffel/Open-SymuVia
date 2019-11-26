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
#ifndef AbstractFleetH
#define AbstractFleetH

#include "tools.h"
#include "VehicleTypeUsageParameters.h"
#include "AbstractFleetParameters.h"

#include <boost/shared_ptr.hpp>

#include <map>
#include <vector>

class Vehicule;
class Trip;
class TripNode;
class TripLeg;
class Reseau;
class DocTrafic;
class VoieMicro;
class Tuyau;
class AbstractFleetParameters;
class Schedule;
class ScheduleElement;
class ScheduleParameters;
class RepartitionTypeVehicule;
class Logger;
class TraceDocTrafic;
class TripNodeSnapshot;
class TripSnapshot;
class VehicleToCreate;


class AbstractFleetSnapshot {

public:
    AbstractFleetSnapshot();
    virtual ~AbstractFleetSnapshot();

    std::vector<int>                            lstVehicles;        // Liste des vÃ©hicules associÃ©s Ã  la flotte
    std::map<Trip*, int>                        mapScheduleState;   // Map des variables de travail des schedules
    std::map<TripNode*, TripNodeSnapshot*>      mapTripNodeState;   // Map des variables de travail des TripNodes
    std::map<Trip*, TripSnapshot*>              mapTripState;       // Map des variables de travail des trips

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SÃ©rialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version);
};


class AbstractFleet
{
public:

    // Constructeurs / Destructeurs
    AbstractFleet();
    AbstractFleet(Reseau * pNetwork);
    virtual ~AbstractFleet();

    // Accesseurs
    ///////////////////////////////////

    const std::vector<TripNode*> & GetTripNodes();
    const std::vector<Trip*> & GetTrips();
    Trip* GetTrip(const std::string & tripID);
    TripNode* GetTripNode(const std::string & tripNodeID);

    std::vector<boost::shared_ptr<Vehicule> > GetVehicles(Trip * pTrip);

    Schedule * GetSchedule(Trip * pTrip);

    // Chargement de la flotte Ã  partir du scÃ©nario XML (ne fait rien par dÃ©faut : cas de la flotte SymuVia)
    virtual bool Load(XERCES_CPP_NAMESPACE::DOMNode * pXMLNetwork, Logger & loadingLogger) {return true;}

    // MÃ©thodes de traitement
    ///////////////////////////////////

    // Instanciation de l'objet spÃ©cifique Ã  la flotte contenant les paramÃ¨tres d'un vÃ©hicule liÃ©s Ã  celle-ci
    virtual AbstractFleetParameters * CreateFleetParameters() {return new AbstractFleetParameters();}

    // Initialisation de la flotte pour la simulation de trafic
    virtual void InitSimuTrafic(std::deque< TimeVariation<TraceDocTrafic> > & docTrafics);

    // Sorties spÃ©cifiques Ã  la flotte dans les fichiers rÃ©sultats
    virtual void SortieTrafic(DocTrafic *pXMLDocTrafic) {}

    // Traitements spÃ©cifiques Ã  la flotte appelÃ©s lors du FinCalculTrafic
    virtual void FinCalculTrafic(Vehicule * pVeh) {}

    // Activation des vÃ©hicules pour le pas de temps courant en fonction du calendrier
    virtual void ActivateVehicles(double dbInstant, double dbTimeStep);

    // Activation d'un vÃ©hicule sur ordre : Ã  spÃ©cialiser dans les diffÃ©rentes flottes
    virtual void ActivateVehicle(double dbInstant, VehicleToCreate * pVehicleToCreate);

    // Test de l'atteinte de la destination
    virtual bool IsCurrentLegDone(Vehicule * pVehicle, TripLeg * pCurrentLeg,
                                  double dbInstant, VoieMicro * pLane, double laneLength, double startPositionOnLane, double endPositionOnLane, bool bExactPosition);

    // Gestion de la terminaison d'une Ã©tape par un vÃ©hicule de la flotte.
    virtual void OnCurrentLegFinished(boost::shared_ptr<Vehicule> pVehicle, VoieMicro * pDestinationEnterLane, double dbInstDestinationReached, double dbInstant, double dbTimeStep);

    // Met Ã  jour le Trip en fonction des tuyaux parcourus
    virtual void SetLinkUsed(double dbInstant, Vehicule * pVeh, Tuyau * pLink);

    // Effectue les opÃ©rations Ã  faire Ã  l'activation d'un vÃ©hicule
    virtual void OnVehicleActivated(boost::shared_ptr<Vehicule> pVeh, double dbInstant);

    // Associe le vÃ©hicule Ã  la flotte et vice versa
    virtual void DoVehicleAssociation(boost::shared_ptr<Vehicule> pVeh);

    // CrÃ©e et associÃ© au vÃ©hicule les paramÃ¨tres liÃ©s Ã  la flotte
    virtual void AssignFleetparameters(boost::shared_ptr<Vehicule> pVeh);

    // Effectue les opÃ©rations Ã  faire Ã  la destruction d'un vÃ©hicule
    virtual void OnVehicleDestroyed(boost::shared_ptr<Vehicule> pVeh);

    // Construit le libellÃ© du vÃ©hicule, utilisÃ© par exemple pour les bus
    virtual std::string GetVehicleIdentifier(boost::shared_ptr<Vehicule> pVeh) {return std::string();}

    // Calcule la durÃ©e d'arrÃªt Ã  un tripnode.
    virtual double GetStopDuration(boost::shared_ptr<Vehicule> pVehicle, TripNode * pTripNode, bool bIsRealStop);

    // Renvoie une map d'attributs Ã  sortir de faÃ§on spÃ©cifique pour un vÃ©hicule dans le TripNode
    virtual std::map<std::string, std::string> GetOutputAttributes(Vehicule * pV, TripNode * pTripNode);

    // Renvoie une map d'attributs Ã  sortir de faÃ§on spÃ©cifique pour un vÃ©hicule une fois la simulation terminÃ©e
    virtual std::map<std::string, std::string> GetOutputAttributesAtEnd(Vehicule * pV);

    // Accesseur vers les paramÃ¨tres liÃ©s au calendrier
    virtual const std::map<ScheduleElement*, ScheduleParameters*> & GetScheduleParameters() {return m_MapScheduleParameters;}

    // Indique s'il faut sortir la charge courante du vÃ©hicule ou non
    virtual bool OutputVehicleLoad(Vehicule * pVehicle) {return false;}

    // mÃ©thode pour la sauvegarde et restitution de l'Ã©tat des flottes (affectation dynamique convergente).
    // Ã  spÃ©cialiser si des flottes utilisent des variables de simulation supplÃ©mentaires
    virtual AbstractFleetSnapshot * TakeSnapshot();
    virtual void Restore(Reseau * pNetwork, AbstractFleetSnapshot * backup);

protected:

    // Charge les paramÃ¨tres d'usage associÃ©s aux types de vÃ©hicules
    virtual bool LoadUsageParams(XERCES_CPP_NAMESPACE::DOMNode * pXMLFleetNode, Logger & loadingLogger);

    // Activation des vÃ©hicules pour le Trip passÃ© en paramÃ¨tres (si besoin)
    virtual std::vector<boost::shared_ptr<Vehicule> > ActivateVehiclesForTrip(double dbInstant, double dbTimeStep, Trip * pTrip);

    // CrÃ©ation d'un vÃ©hicule
    virtual boost::shared_ptr<Vehicule> CreateVehicle(double dbInstant, double dbTimeStep, Trip * pTrip, int nID, ScheduleElement * pScheduleElement);

    // Calcul l'itinÃ©raire pour rÃ©aliser le trip, s'il n'est pas prÃ©dÃ©fini
    virtual void ComputeTripPath(Trip * pTrip, TypeVehicule * pTypeVeh);

    // Ajoute un TripNode en tant que nouvelle destination intÃ©rmÃ©diaire d'un Trip potentiellement rÃ©alisÃ© par le vÃ©hicule pVehicle.
    virtual bool AddTripNodeToTrip(Trip * pTrip, TripNode * pTripNode, size_t tripLegIndex, Vehicule * pVehicle);

    // Supprime un TripLeg d'un Trip potentiellement rÃ©alisÃ© par le vÃ©hicule pVehicle.
    virtual bool RemoveTripLegFromTrip(Trip * pTrip, size_t tripLegIndex, Vehicule * pVehicle);

protected:

    Reseau                                     *m_pNetwork;

    // Ensemble des TripNodes dÃ©finis pour la flotte
    std::vector<TripNode*>                      m_LstTripNodes;

    // Ensemble des Trips dÃ©finis pour la flotte
    std::vector<Trip*>                          m_LstTrips;

    // Schedules associÃ©s aux Trips dÃ©finis pour la flotte
    std::map<Trip*, Schedule*>                  m_MapSchedules;

    // RÃ©partition des types de vÃ©hicules pour la flotte
    std::map<Trip*, RepartitionTypeVehicule*>   m_MapRepartitionTypeVehicule;

    // Ensemble des vÃ©hicules associÃ©s Ã  la flotte et prÃ©sents sur le rÃ©seau.
    std::vector<boost::shared_ptr<Vehicule> >   m_LstVehicles;

    // ParamÃ¨tres spÃ©cifiques Ã  la flotte pour chaque type de vÃ©hicule
    std::map<TypeVehicule*, VehicleTypeUsageParameters> m_MapUsageParameters;

    // ParamÃ¨tres liÃ©s Ã  un Ã©lÃ©ment de calendrier. Attention, ce champ n'a pas l'owership sur les objets ScheduleParameters (il faut les nettoyer
    // dans les spÃ©cialisations de la classe AbstractFleet (car il peut y avoir des ScheduleParameters non associÃ©s Ã  un ScheduleElements).
    std::map<ScheduleElement*, ScheduleParameters*> m_MapScheduleParameters;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SÃ©rialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive& ar, const unsigned int version);
};

#endif // AbstractFleetH