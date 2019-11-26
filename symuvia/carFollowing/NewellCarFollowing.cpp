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
#include "NewellCarFollowing.h"

#include "NewellContext.h"
#include "../reseau.h"
#include "../vehicule.h"
#include "../voie.h"
#include "../tuyau.h"
#include "../DiagFonda.h"
#include "../ConnectionPonctuel.h"
#include "../ControleurDeFeux.h"
#include "../convergent.h"
#include "../RandManager.h"
#include "../MergingObject.h"
#include "../divergent.h"
#include "../sensors/SensorsManager.h"

using namespace std;

NewellCarFollowing::NewellCarFollowing()
{
}

NewellCarFollowing::~NewellCarFollowing()
{
}

CarFollowingContext * NewellCarFollowing::CreateContext(double dbInstant, bool bIsPostProcessing)
{
    return new NewellContext(m_pNetwork, m_pVehicle, dbInstant, bIsPostProcessing);
}

bool NewellCarFollowing::IsPositionComputed()
{
    return true;
}

bool NewellCarFollowing::IsSpeedComputed()
{
    return false;
}

bool NewellCarFollowing::IsAccelerationComputed()
{
    return false;
}

double NewellCarFollowing::GetMaxInfluenceRange()
{
    return 1.0 / m_pNetwork->GetMinKv();
}

void NewellCarFollowing::InternalCompute(double dbTimeStep, double dbInstant, CarFollowingContext * pContext)
{
    bool bFreeFlow = true;
    m_dbComputedTravelledDistance = ComputeFreeFlowDistance(dbTimeStep, dbInstant, pContext);

    if(!pContext->GetLeaders().empty())
    {
        double dbCongestedDistance = ComputeCongestedDistance(dbInstant, dbTimeStep, pContext);
        if(dbCongestedDistance < m_dbComputedTravelledDistance)
        {
            bFreeFlow = false;
            m_dbComputedTravelledDistance = dbCongestedDistance;
        }
    }
    pContext->SetFreeFlow(bFreeFlow);
}

double NewellCarFollowing::ComputeFreeFlowDistance(double dbTimeStep, double dbInstant, CarFollowingContext * pContext)
{
    // OTK - LOI - lors de l'implÃ©mentation des nouvelles lois de poursuite, on veillera Ã 
    // factoriser des choses (dÃ©termination de la vitesse dÃ©sirÃ©e par tuyau en particulier ?) si possible

    const vector<VoieMicro*> lstLanes = pContext->GetLstLanes();
    double dbStartPosition = pContext->GetStartPosition();
    double dbTravelledDistance = 0;
    double dbRemainingTime = dbTimeStep;
    for(size_t iLane = 0; iLane < lstLanes.size(); iLane++)
    {
        VoieMicro * pLane = lstLanes[iLane];
        double startPositionOnLane = iLane == 0 ? dbStartPosition : 0;
        double laneLength = pLane->GetLongueurEx(m_pVehicle->GetType());

        // rmq. OTK : petites imprÃ©cisions ici (non dÃ©tections des portions de vitesses rÃ©glementaires si on passe "par dessus" au cours d'un mÃªme pas de temps ?) 
        double dbMaximumSpeedOnLane = ((Tuyau*)pLane->GetParent())->GetVitRegByTypeVeh(m_pVehicle->GetType(), dbInstant, startPositionOnLane, pLane->GetNum());
        double dbDesiredSpeedOnLane = std::min<double>(m_pVehicle->GetVitMax(), dbMaximumSpeedOnLane);
        // si calcul avec accÃ©lÃ©ration bornÃ©e
        if( m_pNetwork->IsAccBornee() )
        {
            dbDesiredSpeedOnLane = std::min<double>(dbDesiredSpeedOnLane, m_pVehicle->GetVit(1) + m_pVehicle->GetAccMax(m_pVehicle->GetVit(1)) * dbTimeStep);
        }

        // Calcul temps pour atteindre la connexion
        double dbt = (pLane->GetLength() - startPositionOnLane ) / dbDesiredSpeedOnLane;

        if(dbt >= dbRemainingTime || (iLane == lstLanes.size()-1))
        {
            // pas le temps d'atteindre la voie suivante, on arrÃªte ici
            dbTravelledDistance += dbRemainingTime * dbDesiredSpeedOnLane;
            break;
        }
        else
        {
            // le vÃ©hicule a le temps de passer sur la voie suivante
            dbTravelledDistance += laneLength - startPositionOnLane;
            dbRemainingTime -= dbt;
        }
    }
    return dbTravelledDistance;
}

double NewellCarFollowing::ComputeRelaxationEvolution(double dbCurrentValue, CarFollowingContext * pContext, boost::shared_ptr<Vehicule> pVehLeader, double dbLeaderSpeed, double dbTimeStep)
{
    double result;

    double dbKv = - pVehLeader->GetDiagFonda()->GetKMax() * pVehLeader->GetDiagFonda()->GetW() / ( pVehLeader->GetVit(1) -  pVehLeader->GetDiagFonda()->GetW());
    double dbKvFin = - pVehLeader->GetDiagFonda()->GetKMax() * pVehLeader->GetDiagFonda()->GetW() / ( dbLeaderSpeed -  pVehLeader->GetDiagFonda()->GetW());

    double dbRapDeltaNSurKw = dbCurrentValue / ( - pContext->GetVehicle()->GetDiagFonda()->GetKMax() * pVehLeader->GetDiagFonda()->GetW());

    if(((NewellContext*)pContext)->IsContraction())
    {
        result = dbCurrentValue;
    }
    else
    {
        if( (m_pNetwork->GetTimeStep() - dbRapDeltaNSurKw) >= - 0.0001 )        
            result = std::min<double>(1, (dbCurrentValue / dbKv + std::min<double>( dbRapDeltaNSurKw / dbTimeStep * (dbLeaderSpeed - pVehLeader->GetVit(1)) + m_pNetwork->GetRelaxation(), dbLeaderSpeed)*m_pNetwork->GetTimeStep())*dbKvFin);
        else
            result = std::min<double>(1, (dbCurrentValue / dbKv + std::min<double>( (dbLeaderSpeed - pVehLeader->GetVit(1)) + m_pNetwork->GetRelaxation(), dbLeaderSpeed)*m_pNetwork->GetTimeStep())*dbKvFin);

        result = std::max<double>(dbCurrentValue, result);
    }
    return result;
}

double NewellCarFollowing::ComputeJorgeEta(double dbInstant, NewellContext * pNewellContext)
{
    double dbFactor;

    if(m_pVehicle->IsJorgeAgressif())
    {
        if(dbInstant <= pNewellContext->GetJorgeStartTime() + (m_pVehicle->GetType()->GetAgressiveEtaT()-1.0) / m_pVehicle->GetType()->GetEpsylon0())
        {
            // plateau prÃ©cÃ©dent
            if(dbInstant <= pNewellContext->GetJorgeStartTime())
            {
                dbFactor = 1.0;
            }
            // phase montante
            else
            {
                dbFactor = 1.0 + m_pVehicle->GetType()->GetEpsylon0() * (dbInstant - pNewellContext->GetJorgeStartTime());
            }
        }
        else
        {
            // plateau suivant
            if( dbInstant >= pNewellContext->GetJorgeEndTime())
            {
                dbFactor = 1.0;
            }
            // phase descendante
            else
            {
                dbFactor = 1.0 + m_pVehicle->GetType()->GetEpsylon1() * (pNewellContext->GetJorgeEndTime() - dbInstant);
            }
        }
    }
    else
    {
        if(dbInstant <= pNewellContext->GetJorgeStartTime() + (1.0 - m_pVehicle->GetType()->GetShyEtaT()) / m_pVehicle->GetType()->GetEpsylon0())
        {
            // plateau prÃ©cÃ©dent
            if(dbInstant <= pNewellContext->GetJorgeStartTime())
            {
                dbFactor = 1.0;
            }
            else
            {
                // phase descendante
                dbFactor = 1.0 - m_pVehicle->GetType()->GetEpsylon0() * (dbInstant - pNewellContext->GetJorgeStartTime());
            }
        }
        else
        {
            // plateau suivant
            if( dbInstant >= pNewellContext->GetJorgeEndTime())
            {
                dbFactor = 1.0;
            }
            else
            {
                // phase montante
                dbFactor = 1.0 - m_pVehicle->GetType()->GetEpsylon1() * (pNewellContext->GetJorgeEndTime() - dbInstant);
            }
        }
    }

    return dbFactor;
}

double NewellCarFollowing::ComputeCongestedDistance(double dbInstant, double dbTimeStep, CarFollowingContext * pContext)
{
    // La position congestionnÃ©e n'a de sens que si on a un leader.
    assert(pContext->GetLeaders().size() != 0);

    boost::shared_ptr<Vehicule> pVehLeader = pContext->GetLeaders().front();
    boost::shared_ptr<Vehicule> pVehLeaderLeader;
    double dbLeaderDistance = pContext->GetLeaderDistances().front();
    double dbLeaderSpeed;
    if(pVehLeader->IsDejaCalcule())
    {
        dbLeaderSpeed = pVehLeader->GetVit(0);
    }
    else
    {
        // Calcul de la vitesse approchÃ©e du leader
        if(pContext->GetLeaders().size() > 1)
        {
            pVehLeaderLeader =  m_pNetwork->GetPrevVehicule(pVehLeader.get(), true);
            dbLeaderSpeed = std::min<double>( pVehLeader->GetVitRegTuyAct(), pVehLeader->GetCarFollowing()->CalculVitesseApprochee(pVehLeader, pVehLeaderLeader, m_pNetwork->GetDistanceEntreVehicules(pVehLeader.get(), pVehLeaderLeader.get())));			
        }
        else
            if(m_pNetwork->IsAccBornee())
                dbLeaderSpeed = std::min<double>( std::min<double>(pVehLeader->GetVitRegTuyAct(), pVehLeader->GetVitMax()), pVehLeader->GetVit(1) + (m_pVehicle->GetAccMax(pVehLeader->GetVit(1)) * m_pNetwork->GetTimeStep()) ); 
            else
                dbLeaderSpeed = std::min<double>(pVehLeader->GetVitRegTuyAct(), pVehLeader->GetVitMax()); 
    }

    // -- Traitement des cas particuliers des leaders qui dÃ©cÃ©lÃ¨re (Ã  cause d'un feu rouge, d'un arrÃªt ou d'un convergent --
    if(!pContext->IsPostProcessing())
    {
        // (car dans ces cas, la vitesse est surestimÃ©e)
        // CP du leader qui dÃ©cÃ©lÃ¨re Ã  cause d'un feu rouge
        if( !pVehLeaderLeader )
        {
            // La fin du tronÃ§on peut-elle Ãªtre atteinte par le leader ?
            if( (pVehLeader->GetLink(1) != NULL) && (pVehLeader->GetPos(1) + dbTimeStep*dbLeaderSpeed >  pVehLeader->GetLink(1)->GetLength()) )
            {
                ControleurDeFeux *pCDF= NULL;
                if( pVehLeader->GetLink(1)->GetCnxAssAv() )
                {
                    if( pVehLeader->GetLink(1)->GetCnxAssAv()->GetCtrlDeFeux() )
                    {
                        ControleurDeFeux *pCDF = pVehLeader->GetLink(1)->GetCnxAssAv()->GetCtrlDeFeux();
                    }
                }
                else
                {
                    if( pVehLeader->GetLink(1)->getConnectionAval()->GetCtrlDeFeux() )
                    {
                        ControleurDeFeux *pCDF = pVehLeader->GetLink(1)->getConnectionAval()->GetCtrlDeFeux();
                    }
                }
                if(pCDF&&
                    pCDF->GetCouleurFeux(dbInstant, pVehLeader->GetLink(1), pVehLeader->GetNextTuyau()) == FEU_ROUGE )	// Le feu est rouge
                {
                    dbLeaderSpeed = (pVehLeader->GetLink(1)->GetLength() - pVehLeader->GetPos(1) ) / dbTimeStep;
                }
            }
        }

        // CP du leader qui est suceptible de s'insÃ©rer dans un giratoire (donc sur un tuyau non prioritaire). Si c'est le cas, on considÃ¨re le cas le pire
        // Ã  savoir : le leader ne s'insÃ¨re pas et sa vitesse est calculÃ©e dans ce cas
        if( pVehLeader->GetLink(1) && pVehLeader->GetLink(1)->get_Type_aval() == 'C')
        {
            Convergent  *pCvgt;
            pCvgt = (Convergent*)pVehLeader->GetLink(1)->getConnectionAval();

            if( pCvgt->GetNbElAmont() >= 1)
            {
                if( m_pNetwork->GetPrevVehicule(pVehLeader.get(), false ).get() == NULL) // Leader de sa voie
                {
                    if(pCvgt->IsVoieAmontNonPrioritaire(pVehLeader->GetVoie(1)))
                    {
                        double dbVit = -1;

                        if(!pVehLeader->IsDejaCalcule())    // Prise en compte du dernier instant d'insertion du convergent
                            dbVit = pCvgt->CalculVitesseLeader( pVehLeader.get(), dbInstant, dbTimeStep);   
                        else 
                            dbVit = pVehLeader->GetVit(0);

                        if(dbVit == -1)
                            dbVit = std::min<double>(pVehLeader->GetVit(1) + pVehLeader->GetAccMax(m_pVehicle->GetVit(1)) * dbTimeStep , pVehLeader->GetVitRegTuyAct());
                        else
                            dbVit = std::min<double>(dbVit,std::min<double>(pVehLeader->GetVit(1) + pVehLeader->GetAccMax(m_pVehicle->GetVit(1)) * dbTimeStep , pVehLeader->GetVitRegTuyAct() ));

                        if( pVehLeader->GetPos(1) + dbVit * dbTimeStep > pVehLeader->GetVoie(1)->GetLength() )
                        {
                            dbLeaderSpeed = (pVehLeader->GetVoie(1)->GetLength() - pVehLeader->GetPos(1))/dbTimeStep;
                        }
                    }
                }
            }
        }
    }

    // Calcul de delta N Ã  la fin du pas de temps Ã  partir de sa valeur au dÃ©but du pas de temps
    // Si un changement de voie a eu lieu (au dÃ©but du pas de temps comme considÃ©rÃ©), une nouvelle valeur 
    // de delta N a du Ãªtre calculÃ©

    NewellContext * pNewellContext = (NewellContext*)pContext;

    double dbDeltaNDebut = pNewellContext->GetDeltaN();  
    double dbDeltaNFin = ComputeRelaxationEvolution(dbDeltaNDebut, pContext, pVehLeader, dbLeaderSpeed, dbTimeStep);
    pNewellContext->SetDeltaN(dbDeltaNFin);

    double dbRapDeltaNSurKw = dbDeltaNDebut / ( - pVehLeader->GetDiagFonda()->GetKMax() * pVehLeader->GetDiagFonda()->GetW());
    double dbKvFin = - pVehLeader->GetDiagFonda()->GetKMax() * pVehLeader->GetDiagFonda()->GetW() / ( dbLeaderSpeed -  pVehLeader->GetDiagFonda()->GetW());

    // Si on n'est pas en mode Jorge, on regarde si on doit le devenir
    // TODO - JORGE - question : si le vÃ©hicule n'est ni timide ni agressif, on devrait utiliser eta = 1 et pas le deltaN de newell ... mais j usqu'Ã  quand (quitter le rÃ©gime congestionnÃ© ?) ?
    if(!pNewellContext->IsJorge() && m_pVehicle->GetLink(1) && ((TuyauMicro*)m_pVehicle->GetLink(1))->IsAgressif() && (m_pVehicle->IsJorgeAgressif() || m_pVehicle->IsJorgeShy()))
    {
        // TODO - JORGE - rendre paramÃ©trable le 10% (0.1)
        bool bDeltaSpeed = (pVehLeader->GetVit(1) == 0 && m_pVehicle->GetVit(1) > 0) || (pVehLeader->GetVit(1) != 0 && (m_pVehicle->GetVit(1) - pVehLeader->GetVit(1)) / pVehLeader->GetVit(1) > 0.1);
        if(bDeltaSpeed)
        {
            pNewellContext->SetJorge(true);
            pNewellContext->SetJorgeStartTime(dbInstant - m_pNetwork->GetTimeStep());

            if(m_pVehicle->IsJorgeAgressif())
            {
                pNewellContext->SetJorgeEndTime((m_pVehicle->GetType()->GetAgressiveEtaT()-1.0) / m_pVehicle->GetType()->GetEpsylon0()
                    + (m_pVehicle->GetType()->GetAgressiveEtaT()-1.0) / m_pVehicle->GetType()->GetEpsylon1());
            }
            else
            {
                assert(m_pVehicle->IsJorgeShy());
                pNewellContext->SetJorgeEndTime((1.0 - m_pVehicle->GetType()->GetShyEtaT()) / m_pVehicle->GetType()->GetEpsylon0()
                    + (1.0-m_pVehicle->GetType()->GetShyEtaT()) / m_pVehicle->GetType()->GetEpsylon1());
            }
        }
    }
    // Si on est en mode Jorge, on regarde si on doit en sortir
    else if(pNewellContext->IsJorge() && dbInstant >= pNewellContext->GetJorgeEndTime())
    {
        pNewellContext->SetJorge(false);
    }


    double dbFactor, dbFactorStart;
    // TODO - JORGE - rajouter des warnings sur les valeurs d'eta et epsylon (> 1 si agressif, < 1 si timide, etc...) au chargement du rÃ©seau ?
    if(pNewellContext->IsJorge())
    {
        dbFactor = ComputeJorgeEta(dbInstant, pNewellContext);
        dbFactorStart = ComputeJorgeEta(dbInstant-dbTimeStep, pNewellContext);

        // TODO - JORGE - voir pour "casser" Jorge si plus de leader ou s'il change ? ou dans d'autres conditions ?

    }
    else
    {
        // utilisation du deltaN classique
        dbFactor = dbDeltaNFin;
        dbFactorStart = dbDeltaNDebut;
    }

    double dbPosCong;
    if( (m_pNetwork->GetTimeStep() - dbRapDeltaNSurKw )>= - 0.0001 )
    {
        dbPosCong =  dbLeaderDistance + dbLeaderSpeed * m_pNetwork->GetTimeStep() - dbFactor / dbKvFin;

		// If the current vehicle doesn't move when relaxation is applied, deltaN remains constant during the time step (in order to avoid gridlock into roundabout) - no relaxation -
		if (dbPosCong <= 0.0001 && dbDeltaNDebut < 1)
		{
			pNewellContext->SetDeltaN(dbDeltaNDebut);
			dbPosCong = dbLeaderDistance + dbLeaderSpeed * m_pNetwork->GetTimeStep() - dbDeltaNDebut / dbKvFin;
		}
    }
    else
    {
        double dbAlpha = - pVehLeader->GetDiagFonda()->GetW() * pVehLeader->GetDiagFonda()->GetKMax() * m_pNetwork->GetTimeStep() / dbFactorStart;

        if(dbTimeStep < m_pNetwork->GetTimeStep() ) // Le vÃ©hicule a Ã©tÃ© crÃ©Ã© au cours du pas de temps
        {
            // OTK - LOI - Question : gros doute, ce serait pas plutÃ´t dbTimeStep ici ? (et peut Ãªtre ailleurs dans cette mÃ©thode ?) Si on regarde dans convergent, on mutliplie par le temps correspondant
            // Ã  l'avancÃ©e du vÃ©hicule ...
            dbPosCong = dbLeaderDistance  + dbLeaderSpeed * m_pNetwork->GetTimeStep() - dbFactor / dbKvFin;
        }
        else
        {
            dbPosCong = dbAlpha * dbLeaderDistance + dbLeaderSpeed * m_pNetwork->GetTimeStep() - dbAlpha * dbFactor  / dbKvFin;

			// If the current vehicle doesn't move when relaxation is applied, deltaN remains constant during the time step (in order to avoid gridlock into roundabout) - no relaxation -
			if (dbPosCong <= 0.0001 && dbDeltaNDebut < 1)
			{
				pNewellContext->SetDeltaN(dbDeltaNDebut);
				dbPosCong = dbAlpha * dbLeaderDistance + dbLeaderSpeed * m_pNetwork->GetTimeStep() - dbAlpha * dbDeltaNDebut / dbKvFin;
			}
			
        }
    }

    return dbPosCong;
}

//================================================================
double NewellCarFollowing::CalculVitesseApprochee
//----------------------------------------------------------------
// Fonction  :  Calcule de la vitesse approchÃ©e d'un vÃ©hicule Ã  la fin
//              du pas de temps dans le cas de la nouvelle loi de poursuite
// Remarque  :  (3) de 'A microscopic dual-regime model for roundabouts'
// Version du:  07/12/2007
// Historique:  07/12/2007 (C.BÃ©carie - Tinea )
//              CrÃ©ation
//================================================================
(
    boost::shared_ptr<Vehicule> pVehicle,
    boost::shared_ptr<Vehicule> pVehLeader,
    double dbDistanceBetweenVehicles
)
{    
    double      dbRapDeltaNSurKw, dbVitFin, dbAlpha; 

    // OTK - LOI - globalement, j'ai un doute sur les GetTimeStep, dans les cas des vÃ©hicules qui rentrer sur le rÃ©seau (avec un GetTimeStep plus petit que celui du rÃ©seau...

    double dbDeltaN = ((NewellContext*)m_pVehicle->GetCarFollowing()->GetCurrentContext())->GetDeltaN();

    dbRapDeltaNSurKw = dbDeltaN / ( - pVehLeader->GetDiagFonda()->GetKMax() * pVehLeader->GetDiagFonda()->GetW());

    if( (m_pNetwork->GetTimeStep() - dbRapDeltaNSurKw) >= - 0.0001 )
    {
        if( dbDeltaN >= 1)
            dbVitFin = std::min<double>( pVehicle->GetVitMax(), dbDistanceBetweenVehicles / m_pNetwork->GetTimeStep() - dbDeltaN / ( pVehLeader->GetDiagFonda()->GetKMax() * m_pNetwork->GetTimeStep()));
		else
		{
			dbVitFin = std::min<double>(pVehicle->GetVitMax(), std::max<double>(dbDistanceBetweenVehicles / m_pNetwork->GetTimeStep() - dbDeltaN / pVehLeader->GetDiagFonda()->GetKMax() * (1 / m_pNetwork->GetTimeStep()) - m_pNetwork->GetRelaxation(), 0));			
		}
    }
    else
    {
        dbAlpha = - pVehLeader->GetDiagFonda()->GetW() * pVehLeader->GetDiagFonda()->GetKMax() * m_pNetwork->GetTimeStep() / dbDeltaN;
        if( dbDeltaN >= 1)
            dbVitFin = std::min<double>( pVehicle->GetVitMax(), dbAlpha * dbDistanceBetweenVehicles / m_pNetwork->GetTimeStep() + pVehLeader->GetDiagFonda()->GetW() );
		else
		{
			dbVitFin = std::min<double>(pVehicle->GetVitMax(), std::max<double>(0, dbAlpha * dbDistanceBetweenVehicles / m_pNetwork->GetTimeStep() + pVehLeader->GetDiagFonda()->GetW() - m_pNetwork->GetRelaxation()));			
		}
    }

    if(m_pNetwork->IsAccBornee())
        dbVitFin = std::min<double>( dbVitFin, pVehicle->GetVit(1) + m_pNetwork->GetTimeStep() * pVehicle->GetAccMax(pVehicle->GetVit(1)) );

    return dbVitFin;
}

void NewellCarFollowing::CalculAgressivite(double dbInstant)
{
    double dbPosFluide, dbPosAgr;
    double dbSeq;
    double dbRatioContraction;
    double dbSagg;

    NewellContext * pContext = (NewellContext*)GetCurrentContext();

    // Si le vÃ©hicule n'est pas encore sur le rÃ©seau, pas d'agressivitÃ©
    if( m_pVehicle->GetPos(1) < 0)
        return;

    // Si le vÃ©hicule vient d'Ãªtre crÃ©Ã©, pas d'agressivitÃ© de la part du vÃ©hicule
    if( !m_pVehicle->GetLink(1) || m_pVehicle->GetLink(1)->GetType() != Tuyau::TT_MICRO )
        return;

    // Si le vÃ©hicule sort du rÃ©seau Ã  la fin du pas de temps, pas d'agressivitÃ© de la part du vÃ©hicule
    if( !m_pVehicle->GetLink(0) || m_pVehicle->GetLink(0)->GetType() != Tuyau::TT_MICRO )
        return;

    // Si tronÃ§on courant non agressif, pas d'agressivitÃ© de la part du vÃ©hicule
    if( m_pVehicle->GetLink(1) == m_pVehicle->GetLink(0) && !((TuyauMicro*)m_pVehicle->GetLink(1))->IsAgressif() )
    {
        pContext->SetContraction(false);
        return;
    }

    // Si changement de tronÃ§on et au moins un des deux tronÃ§ons non agressif, pas d'agressivitÃ© de la part du vÃ©hicule
    if( m_pVehicle->GetLink(1) != m_pVehicle->GetLink(0) && (!((TuyauMicro*)m_pVehicle->GetLink(1))->IsAgressif() || !((TuyauMicro*)m_pVehicle->GetLink(0))->IsAgressif()))
    {
        pContext->SetContraction(false);
        return;
    }

    // Si pas de leader assignÃ©, pas d'agressivitÃ© de la part du vÃ©hicule
    boost::shared_ptr<Vehicule> pLeader = m_pVehicle->GetLeader();
    if( !pLeader )
    {
        pContext->SetContraction(false);
        return;
    }

    // Si le vÃ©hicule passe un convergent, pas d'agressivitÃ© de la part du vÃ©hicule
    if( m_pVehicle->GetLink(1) != m_pVehicle->GetLink(0) && m_pVehicle->GetLink(1)->get_Type_aval()=='C' )
    {
        pContext->SetContraction(false);
        return;
    }

    // Le vÃ©hicule est-il en train de dÃ©cÃ©lerer ?
    if( m_pVehicle->GetVit(0) < m_pVehicle->GetVit(1) && fabs(m_pVehicle->GetVit(0) - m_pVehicle->GetVit(1)) > 0.001 )
    {
        // Le vÃ©hicule est-il en phase de relaxation ?
        if( pContext->GetDeltaN() <= ((NewellContext*)GetPreviousContext())->GetDeltaN() )
        {
            // Calcul de la position sans dÃ©cÃ©lÃ©rer
            dbPosFluide = ComputeFreeFlowDistance( m_pNetwork->GetTimeStep(), dbInstant, pContext);

            // Calcul du ratio de contraction
            dbSeq = - m_pVehicle->GetType()->GetW() + pLeader->GetVit(0) / (- m_pVehicle->GetType()->GetW() * m_pVehicle->GetType()->GetKx() );
            dbRatioContraction = ( pLeader->GetPos(0) - dbPosFluide ) / dbSeq;

            // MAJ de la position
            if( dbRatioContraction < m_pVehicle->GetType()->GetValAgr() )
            {
                dbSagg = m_pVehicle->GetType()->GetValAgr() * dbSeq;
                dbPosAgr = pLeader->GetPos(0)  - dbSagg;
                pContext->SetDeltaN(m_pVehicle->GetType()->GetValAgr());
                pContext->SetContraction(false);
            }
            else
            {
                if(dbRatioContraction>1)
                {
                    dbPosAgr = m_pVehicle->GetPos(0);
                    pContext->SetContraction(false);
                }
                else
                {
                    dbPosAgr = dbPosFluide;
                    pContext->SetDeltaN(dbRatioContraction);
                    pContext->SetContraction(true);
                }
            }

            m_pVehicle->SetPos(dbPosAgr);
            m_pVehicle->SetVit(m_pVehicle->GetDstParcourueEx() / m_pNetwork->GetTimeStep());
            m_pVehicle->SetAcc(( m_pVehicle->GetVit(0) - m_pVehicle->GetVit(1) ) / m_pNetwork->GetTimeStep());
        }
    }
}


void NewellCarFollowing::ApplyLaneChange(VoieMicro * pVoieOld, VoieMicro * pVoieCible, boost::shared_ptr<Vehicule> pVehLeader, boost::shared_ptr<Vehicule> pVehFollower,
    boost::shared_ptr<Vehicule> pVehLeaderOrig)
{
    if( m_pNetwork->IsChgtVoieGhost() )	// Si procÃ©dure ghost, le follower de la voie cible conserve le ghost de son ancien leader
    {
        if( !m_pVehicle->IsOutside() )  // Exclusion du traitement des vÃ©hicules s'insÃ©rant dans le trafic depuis un TripNode hors voie (ils n'ont pas de follower sur la voie origine)
        {
            double dbDeltaN = 1;   

            if( pVehFollower )
            {
                // Calcul de l'espacement au dÃ©but du pas de temps                       
                double dbS = m_pNetwork->GetDistanceEntreVehicules( pVehFollower.get(), m_pVehicle);

                // Calcul de Kvfin
                double dbKvFin = - m_pVehicle->GetDiagFonda()->GetKMax() * m_pVehicle->GetDiagFonda()->GetW() / ( m_pVehicle->GetVit(1) -  m_pVehicle->GetDiagFonda()->GetW());

                // Mise Ã  jour de delta N
                dbDeltaN = std::min<double>(1, dbKvFin * dbS);    
            }
            double dbThisDeltaN = 1;
            if(GetCurrentContext())
            {
                dbThisDeltaN = ((NewellContext*)GetCurrentContext())->GetDeltaN();
            }
            int nGhostRemain = (int)ceil( (1- std::min<double>(dbDeltaN, dbThisDeltaN) )*m_pNetwork->GetChgtVoieGhostDurationMax() + std::min<double>(dbDeltaN, dbThisDeltaN)*m_pNetwork->GetChgtVoieGhostDurationMin()) ;				// Initialisation du compteur de vie du ghost	
            m_pVehicle->SetGhostRemain(nGhostRemain);
            if( nGhostRemain > 0)
            {
                boost::shared_ptr<Vehicule> pVehFollowerOld = m_pNetwork->GetNearAmontVehicule(pVoieOld, m_pVehicle->GetPos(1) * pVoieOld->GetLength() / pVoieCible->GetLength() );
                m_pVehicle->SetGhostFollower(pVehFollowerOld);  // Follower du ghost (sur la voie origine))
                m_pVehicle->SetGhostLeader(pVehLeaderOrig);		// Leader du ghost (sur la voie origine)
                m_pVehicle->SetGhostVoie(pVoieOld);
            }
        }
    }
}


double NewellCarFollowing::ComputeLaneSpeed(VoieMicro * pTargetLane)
{
    double dbVitVoieCible = 0.0;
    double dbSf;

    // Recherche des vÃ©hicules 'follower' et 'leader' de la voie cible
    boost::shared_ptr<Vehicule> pFollower = m_pNetwork->GetNearAmontVehicule(pTargetLane, pTargetLane->GetLength() / m_pVehicle->GetVoie(1)->GetLength() * m_pVehicle->GetPos(1) + 0.1 );
    boost::shared_ptr<Vehicule> pLeader = m_pNetwork->GetNearAvalVehicule(pTargetLane, pTargetLane->GetLength() / m_pVehicle->GetVoie(1)->GetLength() * m_pVehicle->GetPos(1) + 0.1 );

    // Calcul de l'inter-distance entre le vÃ©hicule follower et le vehicule leader de la voie cible
    if(pFollower && pLeader)
    {
        dbSf = m_pNetwork->GetDistanceEntreVehicules(pLeader.get(), pFollower.get());		
    }
    else
    {   
        // OTK - LOI - DIAG - remplacer par une notion gÃ©nÃ©rique ?
        dbSf = 1/m_pVehicle->GetDiagFonda()->GetKCritique();
    }

    // Calcul de la vitesse associÃ©e Ã  l'inter-distance entre le vÃ©hicule follower et leader de la voie cible
    if(dbSf>0)
    {
        double dbVal1 = 1/dbSf;
        double dbVal2 = VAL_NONDEF;

        if(pLeader)
            pLeader->GetDiagFonda()->CalculVitEqDiagrOrig(dbVitVoieCible, dbVal1, dbVal2, true); // Utilisation du diagramme fondamental du vÃ©hicule leader de la voie cible
        else
            m_pVehicle->GetDiagFonda()->CalculVitEqDiagrOrig(dbVitVoieCible, dbVal1, dbVal2, true); // Utilisation du diagramme fondamental du vÃ©hicule candidat au changement de voie

        if(pLeader)
        {                
            dbVitVoieCible = pLeader->GetVit(1);
        }                
    }
    
    return dbVitVoieCible;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SÃ©rialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template void NewellCarFollowing::serialize(boost::archive::xml_woarchive & ar, const unsigned int version);
template void NewellCarFollowing::serialize(boost::archive::xml_wiarchive & ar, const unsigned int version);

template<class Archive>
void NewellCarFollowing::serialize(Archive& ar, const unsigned int version)
{
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(AbstractCarFollowing);
}
