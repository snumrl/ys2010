#include "stdafx.h"

#include "../../common_sources/bputil.h"

#include "../../common_sources/vputil.h"
#include "csVpModel.h"
#include "csVpWorld.h"

#define MAX_X 1	// 0001
#define MAX_Y 2	// 0010
#define MAX_Z 4	// 0100

BOOST_PYTHON_MODULE(csVpWorld)
{
	class_<VpWorld>("VpWorld", init<const object&>())
		.def("step", &VpWorld::step)
		.def("initialize", &VpWorld::initialize)
		.def("calcPenaltyForce", &VpWorld::calcPenaltyForce)
		.def("applyPenaltyForce", &VpWorld::applyPenaltyForce)
		.def("getBodyNum", &VpWorld::getBodyNum)
		.def("calcPenaltyForce_Boxes", &VpWorld::calcPenaltyForce_Boxes)
	;
}


VpWorld::VpWorld(const object& config)
{
	_world.SetTimeStep(XD(config.attr("timeStep")));
	_world.SetGravity(pyVec3_2_Vec3(config.attr("gravity")));

	_planeHeight = XD(config.attr("planeHeight"));
	_lockingVel = XD(config.attr("lockingVel"));

	if(XB(config.attr("useDefaultContactModel")))
	{
		vpMaterial::GetDefaultMaterial()->SetDynamicFriction(1.);

		_ground.AddGeometry(new vpBox(Vec3(10000, 0, 10000)));
		_ground.SetFrame(Vec3(0, _planeHeight, 0));
		_ground.SetGround();
		_world.AddBody(&_ground);
	}
}

void VpWorld::step()
{
	_world.StepAhead();
}

void VpWorld::initialize()
{
	_world.Initialize();
}

// @return ( bodyIDs, positions, postionLocals, forces)
tuple VpWorld::calcPenaltyForce( const bp::list& bodyIDsToCheck, const bp::list& mus, scalar Ks, scalar Ds )
{
	bp::list bodyIDs, positions, forces, positionLocals;
	int bodyID;
	numeric::array O_Vec3(make_tuple(0.,0.,0.));
	vpBody* pBody;
	vpGeom* pGeom;
	char type;
	scalar data[3];
	Vec3 position, velocity, force, positionLocal;

	for(int i=0; i<len(bodyIDsToCheck); ++i)
	{
		bodyID = XI(bodyIDsToCheck[i]);
		pBody = _world.GetBody(bodyID);

		for(int j=0; j<pBody->GetNumGeometry(); ++j)
		{
			pGeom = const_cast<vpGeom*>(pBody->GetGeometry(j));

			const vector<Vec3>& verticesLocal = pGeom->getVerticesLocal();
			const vector<Vec3>& verticesGlobal = pGeom->getVerticesGlobal();

			for(int k=0; k<verticesLocal.size(); ++k)
			{
				positionLocal = verticesLocal[k];
				position = verticesGlobal[k];
				velocity = pBody->GetLinVelocity(positionLocal);

				bool penentrated = _calcPenaltyForce(pBody, position, velocity, force, Ks, Ds, XD(mus[i]));
				if(penentrated)
				{
					bodyIDs.append(bodyID);

					object pyPosition = O_Vec3.copy();
					Vec3_2_pyVec3(position, pyPosition);
					positions.append(pyPosition);

					object pyForce = O_Vec3.copy();
					Vec3_2_pyVec3(force, pyForce);
					forces.append(pyForce);

					object pyPositionLocal = O_Vec3.copy();
					Vec3_2_pyVec3(positionLocal, pyPositionLocal);
					positionLocals.append(pyPositionLocal);
				}
			}
		}
	}
	return make_tuple(bodyIDs, positions, positionLocals, forces);
}


// @param position �ۿ��� ����(global)
// @param [out] force �߻��� penalty force(global)
// @return penalty force�� �߻������� true
bool VpWorld::_calcPenaltyForce( vpBody* pBody, const Vec3& position, const Vec3& velocity, Vec3& force, scalar Ks, scalar Ds, scalar mu )
{
	Vec3 vRelVel, vNormalRelVel, vTangentialRelVel;
	scalar normalRelVel, tangentialRelVel;
	const Vec3 vNormal(0,1,0);
	Vec3 vNormalForce, vFrictionForce;
	scalar normalForce=0., frictionForce=0.;

	vRelVel = velocity;
	normalRelVel = Inner(vRelVel, vNormal);
	vNormalRelVel = normalRelVel * vNormal;
	vTangentialRelVel = vRelVel - vNormalRelVel;
	tangentialRelVel = Norm(vTangentialRelVel);

	if(position[1] > _planeHeight)
		return false;
	else
	{
		// normal reaction force
		normalForce = Ks*(_planeHeight - position[1]);
		normalForce -= Ds*velocity[1];
		if(normalForce<0.) normalForce = 0.;
		vNormalForce = normalForce * vNormal;
		
		// tangential reaction force
		frictionForce = mu * normalForce;

		// ������ ������ �� �̲������� ���� �����ϱ� ����
		// rigid body�̹Ƿ� point locking�� ���� ���������� ������ �����
		// �̲������� ������ �ӵ��� �ݴ�������� ū �������� �ۿ뿡 ���� step������ 
		// �ٽ� �� �ݴ�������� �������� �ۿ��ϸ鼭 ������ �ϸ� �̲������� ����
		// �̸� �����ϱ� ���� ���� �ӵ� ���Ͽ����� �������� ���� ������ �����ϵ��� �ӽ� �ڵ�
		if(tangentialRelVel < _lockingVel) 
			frictionForce *= tangentialRelVel/_lockingVel;

		vFrictionForce = frictionForce * -Normalize(vTangentialRelVel);

		force = vNormalForce + vFrictionForce;
		return true;
	}
}

void VpWorld::applyPenaltyForce( const bp::list& bodyIDs, const bp::list& positionLocals, const bp::list& forces )
{
	int bodyID;
	vpBody* pBody;
	Vec3 position, force;

	for(int i=0; i<len(bodyIDs); ++i)
	{
		bodyID = XI(bodyIDs[i]);
		pyVec3_2_Vec3(positionLocals[i], position);
		pyVec3_2_Vec3(forces[i], force);

		pBody = _world.GetBody(bodyID);
		pBody->ApplyGlobalForce(force, position);
	}
}

// @return ( bodyIDs, positions, postionLocals, forces)
tuple VpWorld::calcPenaltyForce_Boxes( const bp::list& boxSizes, const bp::list boxFrames, const bp::list& bodyIDsToCheck, const bp::list& mus, scalar Ks, scalar Ds )
{
	bp::list bodyIDs, positions, forces, positionLocals;
	int bodyID;
	numeric::array O_Vec3(make_tuple(0.,0.,0.));
	const vpBody* pBody;
	vpGeom* pGeom;
	char type;
	scalar data[3];
	Vec3 position, velocity, force, positionLocal;

	Vec3 boxSize;
	SE3 boxFrame;

	for(int a=0; a<len(boxSizes); ++a)
	{
		pyVec3_2_Vec3(boxSizes[a], boxSize);
		pySE3_2_SE3(boxFrames[a], boxFrame);

		for(int i=0; i<len(bodyIDsToCheck); ++i)
		{
			bodyID = XI(bodyIDsToCheck[i]);
			pBody = _world.GetBody(bodyID);

			for(int j=0; j<pBody->GetNumGeometry(); ++j)
			{
				pGeom = const_cast<vpGeom*>(pBody->GetGeometry(j));

				const vector<Vec3>& verticesLocal = pGeom->getVerticesLocal();
				const vector<Vec3>& verticesGlobal = pGeom->getVerticesGlobal();

				for(int k=0; k<verticesLocal.size(); ++k)
				{
					positionLocal = verticesLocal[k];
					position = verticesGlobal[k];
					velocity = pBody->GetLinVelocity(positionLocal);

					bool penentrated = _calcPenaltyForce_Boxes(boxSize, boxFrame, pBody, position, velocity, force, Ks, Ds, XD(mus[i]));
					if(penentrated)
					{
						bodyIDs.append(bodyID);

						object pyPosition = O_Vec3.copy();
						Vec3_2_pyVec3(position, pyPosition);
						positions.append(pyPosition);

						object pyForce = O_Vec3.copy();
						Vec3_2_pyVec3(force, pyForce);
						forces.append(pyForce);

						object pyPositionLocal = O_Vec3.copy();
						Vec3_2_pyVec3(positionLocal, pyPositionLocal);
						positionLocals.append(pyPositionLocal);
					}
				}
			}
		}
	}
	return make_tuple(bodyIDs, positions, positionLocals, forces);
}

// @param position �ۿ��� ����(global)
// @param [out] force �߻��� penalty force(global)
// @return penalty force�� �߻������� true
bool VpWorld::_calcPenaltyForce_Boxes( const Vec3& boxSize, const SE3& boxFrame, const vpBody* pBody, const Vec3& position, const Vec3& velocity, Vec3& force, scalar Ks, scalar Ds, scalar mu )
{
	Vec3 vRelVel, vNormalRelVel, vTangentialRelVel;
	scalar normalRelVel, tangentialRelVel;
	const Vec3 vNormal(0,1,0);
	Vec3 vNormalForce, vFrictionForce;
	scalar normalForce=0., frictionForce=0.;

	// box ó�� ����
	Vec3 position_moved, velocity_moved, force_moved;
	const Vec3 vZero(0,0,0);
	SE3 boxR;
	scalar boxHeight;

	boxHeight = boxSize[1]/2.;
	position_moved = Inv(boxFrame) * position;
	boxR = boxFrame; boxR.SetPosition(vZero);
	velocity_moved = Inv(boxR) * velocity;


//	vRelVel = velocity;
	vRelVel = velocity_moved;
	normalRelVel = Inner(vRelVel, vNormal);
	vNormalRelVel = normalRelVel * vNormal;
	vTangentialRelVel = vRelVel - vNormalRelVel;
	tangentialRelVel = Norm(vTangentialRelVel);

//	if(position[1] > _planeHeight)
	if(position_moved[1] > boxHeight || 
		position_moved[0] < -boxSize[0]/2. || position_moved[0] > boxSize[0]/2. ||
		position_moved[2] < -boxSize[2]/2. || position_moved[2] > boxSize[2]/2.)
		return false;
	else
	{
		// normal reaction force
//		normalForce = Ks*(_planeHeight - position[1]);
//		normalForce -= Ds*velocity[1];
		normalForce = Ks*(boxHeight - position_moved[1]);
		normalForce -= Ds*velocity_moved[1];
		if(normalForce<0.) normalForce = 0.;
		vNormalForce = normalForce * vNormal;
		
		// tangential reaction force
		frictionForce = mu * normalForce;

		// ������ ������ �� �̲������� ���� �����ϱ� ����
		// rigid body�̹Ƿ� point locking�� ���� ���������� ������ �����
		// �̲������� ������ �ӵ��� �ݴ�������� ū �������� �ۿ뿡 ���� step������ 
		// �ٽ� �� �ݴ�������� �������� �ۿ��ϸ鼭 ������ �ϸ� �̲������� ����
		// �̸� �����ϱ� ���� ���� �ӵ� ���Ͽ����� �������� ���� ������ �����ϵ��� �ӽ� �ڵ�
		if(tangentialRelVel < _lockingVel) 
			frictionForce *= tangentialRelVel/_lockingVel;

		vFrictionForce = frictionForce * -Normalize(vTangentialRelVel);

		force_moved = vNormalForce + vFrictionForce;
		force = boxFrame * force_moved;

		return true;
	}
}

