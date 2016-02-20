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

#ifndef _VP_RENDERER_
#define _VP_RENDERER_

#include <VP/vphysics.h>
#include "GLFramework.h"
#include <map>

class vpRenderer
{
public:

										 vpRenderer();
										~vpRenderer();

	void								 SetTarget(const vpWorld *);
	void								 Render(bool apply_material = true);
	void								 SetMaterial(const vpBody *, glMaterial *);
	void								 DrawContacPoints(bool flag = true);
	void								 SetCaption(const char buf[]);

protected:

	void								_Initialize(void);
	const vpWorld						*m_pWorld;

	map<const vpBody *, int>			 m_sBodyGroupMap;				// to get m_sBodyGroup index associated with vpBody
	map<const vpBody *, unsigned int>	 m_sDisplayListMap;				// to get displaylist associated with vpBody
	
	struct BODY_PAIR					 { const vpBody *pBody; unsigned int iDisplayList; };
	struct BODY_GROUP					 { glMaterial *pMaterial; vector<BODY_PAIR> bodyPair; };

	vector<BODY_GROUP>					 m_sBodyGroup;
	bool								 m_bInitialize;
	bool								 m_bDrawContactPoints;
	char								 m_szCaption[1024];
};

#endif
