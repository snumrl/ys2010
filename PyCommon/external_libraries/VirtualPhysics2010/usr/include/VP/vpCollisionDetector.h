/*
VirtualPhysics v0.9

VirtualPhysics was developed as part of the project entitled "Development of 
Real-time Physics Simulation Engine for e-Entertainments" which was financially
supported by the grant from the strategic technology development program
(Project No. 2008-F-033-02) of both the MKE(Ministry of Knowledge Economy) and
MCST(Ministry of Culture, Sports and Tourism) of Korea.

Copyright (c) 2008-2010, Jinwook Kim, Korea Institute of Science and Technology
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, 
      this list of conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
	  and/or other materials provided with the distribution.

   3. Only research partners of the project "Development of Real-time Physics 
      Simulation Engine for e-Entertainments" can modify this list of 
	  conditions and the following disclaimer with the consent of the author, 
	  where the research partners refer to all principal investigators 
	  involved in the project. 

THIS SOFTWARE IS PROVIDED BY THE AUTHOR "AS IS" AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO 
EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
CONTRACT, STRICT LIABILITY, OR TORT(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/

#ifndef VP_COLLISION_DETECTOR
#define VP_COLLISION_DETECTOR

#include <VP/vpDataType.h>

/*!
	\class vpCollisionDetector
	\brief abstract class for detecting collision between rigid bodies
*/

class vpCollisionDetector
{
	friend class				 vpWorld;

public:

							 vpCollisionDetector();
	/*!
		The method is called right after the world is initialized.
	*/
	virtual void			 Initialize(void) = 0;

	/*!
		It is responsible to build vpCollisionDetector::m_sCollisionList
		vpCollisionDetector::m_sCollisionList is empty.
		So just 'push_back' an index of the collision pair from m_sCollisionLUT 
	*/	
	virtual void			 DetectCollision(void) = 0;

	const _array<intList>	&GetCollisionList(void) const;

	void					 SetSkinDepth(const scalar &);
	scalar					 GetSkinDepth(void) const;

	void					 SetMaxNumSimulContact(int);

protected:

	void					 IgnoreCollision(vpBody *B0, vpBody *B1);
	void					 Attach(vpWorld *);

	void					 initialize(void);

	vpWorld					*m_pWorld;
	vpBodyPairSet			 m_sNonCollidablePair;

	_array<intList>			 m_sCollisionList;

	vpColPairArray			 m_sCollisionLUT;

	scalar					 m_rSkinDepth;

	int						 m_iMaxNumSimulContact;				// maximum number of simultaneous contact, default value = 10 
};

#endif
