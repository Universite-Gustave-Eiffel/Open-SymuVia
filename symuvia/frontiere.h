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
#ifndef frontiereH
#define frontiereH

namespace boost {
    namespace serialization {
        class access;
    }
}

/*===========================================================================================*/
/* Classe de modélisation d'une frontière                                                    */
/*===========================================================================================*/

class Frontiere
{
public:
        // Constructeur
        Frontiere    (){};
        Frontiere    (int nNbPadTempsHist, double dbVitInit);

        // Destructeur
        ~Frontiere   ();

private:

        double *m_pN;               // pointeur sur le tableau de stockage de N
        double *m_pVit;             // pointeur sur le tableau de stockage de la vitesse
        double *m_pAcc;             // pointeur sur le tableau de stockage de l'accélération

        int     m_nNbPasTempsHist;  // Nombre de pas de temps historisé pour N

public:
        void    InitSimu(double dbVitInit);
        double  GetN    (int nDiffTemps);
        double  GetVit  (int nDiffTemps);
        double  GetAcc  (int nDiffTemps);

        double  GetLastN            (){return m_pN[m_nNbPasTempsHist-1];};
        double  GetBefLastN         (){return m_pN[m_nNbPasTempsHist-2];};
        double  GetLastVit          (){return m_pVit[m_nNbPasTempsHist-1];};
        double  GetBefLastVit       (){return m_pVit[m_nNbPasTempsHist-2];};

        int     GetNbPasTempsHist(){return m_nNbPasTempsHist;};

        void    DecalVarTrafic      ();

        void    SetN    (double dbN)    {m_pN[0]    = dbN;};
        void    SetVit  (double dbVit)  {m_pVit[0]  = dbVit;};
        void    SetAcc  (double dbAcc)  {m_pAcc[0]  = dbAcc;};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Sérialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
	friend class boost::serialization::access;
    template<class Archive>
	void serialize(Archive & ar, const unsigned int version);
};
#endif
 