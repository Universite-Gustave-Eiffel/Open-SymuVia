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
#ifndef NewellCarFollowingH
#define NewellCarFollowingH

#include "AbstractCarFollowing.h"

class NewellContext;
class NewellCarFollowing : public AbstractCarFollowing
{
public:
    NewellCarFollowing();
    virtual ~NewellCarFollowing();

    virtual CarFollowingContext * CreateContext(double dbInstant, bool bIsPostProcessing = false);

    virtual bool IsPositionComputed();
    virtual bool IsSpeedComputed();
    virtual bool IsAccelerationComputed();

    virtual double GetMaxInfluenceRange();

    // OTK - LOI - Question : voir si on laisse cette mÃ©thode, ou bien si on estime la vitesse du leader avec la loi Newell, ce qui est Ã  prÃ©sent
    // possible puisqu'on dÃ©finit le diagramme fondamental Newell pour tous les vÃ©hicules.
    virtual double CalculVitesseApprochee(boost::shared_ptr<Vehicule> pVehicle, boost::shared_ptr<Vehicule> pVehLeader, double dbDistanceBetweenVehicles);

    virtual void CalculAgressivite(double dbInstant);

    virtual void ApplyLaneChange(VoieMicro * pVoieOld, VoieMicro * pVoieCible, boost::shared_ptr<Vehicule> pVehLeader,
        boost::shared_ptr<Vehicule> pVehFollower, boost::shared_ptr<Vehicule> pVehLeaderOrig);

    virtual double ComputeLaneSpeed(VoieMicro * pTargetLane);

protected:
    virtual void InternalCompute(double dbTimeStep, double dbInstant, CarFollowingContext * pContext);

private:
    double ComputeFreeFlowDistance(double dbTimeStep, double dbInstant, CarFollowingContext * pContext);
    double ComputeCongestedDistance(double dbInstant, double dbTimeStep, CarFollowingContext * pContext);

    double ComputeRelaxationEvolution(double dbCurrentValue, CarFollowingContext * pContext, boost::shared_ptr<Vehicule> pVehLeader, double dbLeaderSpeed, double dbTimeStep);

    double ComputeJorgeEta(double dbInstant, NewellContext * pContext);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SÃ©rialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version);
};

#endif // NewellCarFollowingH
