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
#ifndef CarFollowingContextH
#define CarFollowingContextH

#include <boost/shared_ptr.hpp>

#include <vector>

namespace boost {
    namespace serialization {
        class access;
    }
}

class Reseau;
class Vehicule;
class VoieMicro;
class Arret;

class CarFollowingContext
{
public:
    CarFollowingContext();
    CarFollowingContext(Reseau * pNetwork, Vehicule * pVehicle, double dbInstant, bool bIsPostProcessing);
    virtual ~CarFollowingContext();

    // Construction du contexte pour le début du pas de temps courant
    virtual void Build(double dbRange, CarFollowingContext * pPreviousContext);

    // Définition d'un contexte arbitraire (pour utilisation en post-traitement des convergents, par exemple)
    virtual void SetContext(const std::vector<boost::shared_ptr<Vehicule> > & leaders,
                            const std::vector<double> & leaderDistances,
                            const std::vector<VoieMicro*> & lstLanes,
                            double dbStartPosition,
                            CarFollowingContext * pOriginalContext = NULL,
                            bool bRebuild = false);

    // Accesseurs
    const std::vector<boost::shared_ptr<Vehicule> > & GetLeaders();
    void SetLeaders(const std::vector<boost::shared_ptr<Vehicule> > & newLeaders);
    const std::vector<double> & GetLeaderDistances();
    const std::vector<VoieMicro*> & GetLstLanes();
    double GetStartPosition();
    bool IsFreeFlow() const;
    void SetFreeFlow(bool bFreeFlow);

    Vehicule * GetVehicle();
    
    bool IsPostProcessing() const;

    virtual void CopyTo(CarFollowingContext * pDestinationContext);

private:
    // Construction de la liste des voies potentiellement impactantes
    void BuildLstLanes(double dbRange);

    // Recherche du leader potentiellement impactant
    void SearchLeaders(double dbRange);

protected:
    // Réseau
    Reseau                                     *m_pNetwork;
    // Instant du contexte
    double                                      m_dbInstant;
    // Véhicule auquel se rapporte le contexte
    Vehicule                                    *m_pVehicle;


    // Véhicules leader
    std::vector<boost::shared_ptr<Vehicule> >   m_Leaders;
    // Distances séparant le véhicule de ses leaders
    std::vector<double>                         m_LeaderDistances;
    // Listes des voies à considérer pour l'application de la loi de poursuite
    std::vector<VoieMicro*>                     m_LstLanes;
    // Position initiale du véhicule sur la première voie
    double                                      m_dbStartPosition;

    // Indique si le contexte est un contexte de post-traitement ou pas (permet de modifier le comportement
    // en cas de calcul des convergents en post-traitement, par exemple).
    bool                                        m_bIsPostProcessing;

    // Flag indiquant si la loi de poursuite a déterminé un état fluide ou congestionné pour le pas de temps associé au contexte
    bool                                        m_bIsFreeFlow;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Sérialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version);
};

#endif // CarFollowingContextH
