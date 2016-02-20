#pragma once

class Motion;
class matrixn;
class vector3N;
class quaterN;
class bitvectorn;
class Bone;

namespace MotionUtilTwo
{
	void getOpponentPos(Motion const& m_Motion, vector3N& out, int start, int end);
	void setOpponentPos(Motion& m_Motion, const vector3N& in, int start);

	// opponentpos�� orientation (angle)�� length�� decompose�ؼ� return.
	void getOpponentPos(Motion const& m_Motion, matrixn& out, int start, int end);
	void setOpponentPos(Motion& m_Motion, const matrixn& in, int start);
}
