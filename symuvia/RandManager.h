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
#ifndef RandManagerH
#define RandManagerH

#include <boost/random.hpp>

// rmq. on utilise la valeur pour Windows de RAND_MAX de stdlib.h. car c'est la valeur utilisÃ©e historiquement par SymuVia.
// on n'utilise pas directement RAND_MAX car sa valeur est diffÃ©rente sous Linux.
#define MAXIMUM_RANDOM_NUMBER 0x7fff

/// Classe de gestion de la gÃ©nÃ©ration des nombres alÃ©atoires (avec reprise possible de la sÃ©quence
/// pseudo-alÃ©atoire Ã  la reprise d'un snapshot)
class RandManager
{
public:

    // Constructeur par dÃ©faut
    RandManager();

    /// Initialisation de la sÃ©quence alÃ©atoire
    void mySrand(unsigned int seed, unsigned int nbCalls = 0);

    int myRand();

    int myPoissonRand(double mean);

    int myBinomialRand(double probability, int trialNb);

    double myExponentialRand(double rate);

    double myNormalRand(double dbMean, double dbSigma);

    double myNormalRand(double dbMean, double dbSigma, double dbMin, double dbMax);

    unsigned int getCount();

private:
    unsigned int m_Count;
    boost::mt19937 m_RandomGenerator;
    boost::uniform_int<> m_Distribution;
    boost::variate_generator< boost::mt19937&, boost::uniform_int<> > m_LimitedInt;
};

#endif // RandManagerH