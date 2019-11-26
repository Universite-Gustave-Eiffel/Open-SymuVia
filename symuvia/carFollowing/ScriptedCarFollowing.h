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
#ifndef ScriptedCarFollowingH
#define ScriptedCarFollowingH

#include "AbstractCarFollowing.h"

#include <boost/python/dict.hpp>

#include <string>
#include <vector>

class ScriptedCarFollowing : public AbstractCarFollowing
{
public:
    ScriptedCarFollowing();
    virtual ~ScriptedCarFollowing();

    void Copy(ScriptedCarFollowing * pOriginal);

    virtual bool IsPositionComputed();
    virtual bool IsSpeedComputed();
    virtual bool IsAccelerationComputed();

    virtual double GetVehicleLength();

    virtual double GetMaxInfluenceRange();

    // OTK - LOI - Question : voir si on laisse cette mÃ©thode, ou bien si on estime la vitesse du leader avec la loi Newell, ce qui est Ã  prÃ©sent
    // possible puisqu'on dÃ©finit le diagramme fondamental Newell pour tous les vÃ©hicules.
    virtual double CalculVitesseApprochee(boost::shared_ptr<Vehicule> pVehicle, boost::shared_ptr<Vehicule> pVehLeader, double dbDistanceBetweenVehicles);

    virtual void CalculAgressivite(double dbInstant);

    virtual bool TestLaneChange(double dbInstant, boost::shared_ptr<Vehicule> pVehFollower, boost::shared_ptr<Vehicule> pVehLeader, boost::shared_ptr<Vehicule> pVehLeaderOrig,
        VoieMicro * pVoieCible, bool bForce, bool bRabattement, bool bInsertion);

    virtual void ApplyLaneChange(VoieMicro * pVoieOld, VoieMicro * pVoieCible, boost::shared_ptr<Vehicule> pVehLeader,
        boost::shared_ptr<Vehicule> pVehFollower, boost::shared_ptr<Vehicule> pVehLeaderOrig);

    virtual bool TestConvergentInsertion(PointDeConvergence * pPointDeConvergence, Tuyau *pTAmP, VoieMicro* pVAmP, VoieMicro* pVAmNP, int nVoieGir, double dbInstant, int j, double dbOffre, double dbDemande1, double dbDemande2,
        boost::shared_ptr<Vehicule> pVehLeader, boost::shared_ptr<Vehicule> pVehFollower, double & dbTr, double & dbTa);

    virtual double ComputeLaneSpeed(VoieMicro * pTargetLane);

    // Accesseurs
    const std::string & GetID();
    void SetID(const std::string & strId);

    void SetStateVariable(EStateVariable stateVariable);
    EStateVariable GetStateVariable();

    void SetInternalComputeFuncName(const std::string & funcName);
    const std::string & GetInternalComputeFuncName();
    void SetVehicleLengthFuncName(const std::string & funcName);
    const std::string & GetVehicleLengthFuncName();
    void SetMaxInfluenceDistanceFuncName(const std::string & funcName);
    const std::string & GetMaxInfluenceDistanceFuncName();
    void SetApproxSpeedFuncName(const std::string & funcName);
    const std::string & GetApproxSpeedFuncName();
    void SetAggressivityFuncName(const std::string & funcName);
    const std::string & GetAggressivityFuncName();
    void SetTestLaneChangeFuncName(const std::string & funcName);
    const std::string & GetTestLaneChangeFuncName();
    void SetApplyLaneChangeFuncName(const std::string & funcName);
    const std::string & GetApplyLaneChangeFuncName();
    void SetTestConvergentInsertionFuncName(const std::string & funcName);
    const std::string & GetTestConvergentInsertionFuncName();
    void SetLaneSpeedFuncName(const std::string & funcName);
    const std::string & GetLaneSpeedFuncName();

    void SetModuleName(const std::string & moduleName, Reseau * pNetwork);
    const std::string & GetModuleName();

    boost::python::dict& GetParameters();

protected:
    virtual void InternalCompute(double dbTimeStep, double dbInstant, CarFollowingContext * pContext);

private:
    template<class T>
    T ExecutePythonFunc(const std::string & functionName, CarFollowingContext * pContext, boost::python::dict locals = boost::python::dict(), const std::vector<std::string> &additionalParams = std::vector<std::string>());

    void ExecutePythonFunc(const std::string & functionName, CarFollowingContext * pContext, boost::python::dict locals, const std::vector<std::string> &additionalParams);

    std::string getModuleName();

private:
    std::string    m_ID;
    EStateVariable m_StateVariable;

    std::string    m_InternalComputeFuncName;
    std::string    m_VehicleLengthFuncName;
    std::string    m_MaxInfluenceDistanceFuncName;
    std::string    m_ApproxSpeedFuncName;
    std::string    m_AggressivityFuncName;
    std::string    m_TestLaneChangeFuncName;
    std::string    m_ApplyLaneChangeFuncName;
    std::string    m_TestConvergentInsertionFuncName;
    std::string    m_LaneSpeedFuncName;

    std::string    m_ModuleName;

    boost::python::dict m_Parameters;

    std::string    m_ShortModuleName;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SÃ©rialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version);
};

#endif // ScriptedCarFollowingH
