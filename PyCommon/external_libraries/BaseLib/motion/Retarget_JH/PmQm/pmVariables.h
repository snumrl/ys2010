
#ifndef __PM_ENVIRONMENT_VARIABLES_H
#define __PM_ENVIRONMENT_VARIABLES_H

#define PM_MAX_FRAMES   80000

extern int				PenvManipulateLevel;
extern bool				PenvFineAdjustment;
extern bool				PenvIKControl;
extern int				PenvControlRange;
extern bool				PenvLocalControl;
extern bool				PenvInsertConstraints;
extern bool				PenvDampingEnabled;
extern bool				PenvJointLimitEnabled;
extern bool				PenvStaticBalanceEnabled;
extern int				PenvPresmoothing;
extern bool				PenvMotionManipulationEnabled;

#define PENV_INTERPOLATORY_SUBDIVISION	0
#define PENV_RELAXATION_SUBDIVISION		1
#define PENV_B_SPLINE_SUBDIVISION		2

#define PENV_UNIFORM_DOWN_SAMPLING		0
#define PENV_NON_UNIFORM_DOWN_SAMPLING	1

#define PENV_IK_ANALYTICAL				0
#define PENV_IK_NUMERICAL				1
#define PENV_IK_HYBRID					2

#define PENV_IK_CG_METHOD				0
#define PENV_IK_LS_SVD_METHOD			1
#define PENV_IK_LS_DAMP_METHOD			2

extern int				PenvMRAsubdivisionType;
extern int				PenvMRAdownSamplingType;

extern int				PenvIKsolverType;
extern int				PenvIKsolverMethod;

extern m_real			PenvDampingCoeff   [PM_HUMAN_NUM_LINKS+1];
extern m_real			PenvJointLimitCoeff[PM_HUMAN_NUM_LINKS+1];
extern m_real			PenvRigidityCoeff  [PM_HUMAN_NUM_LINKS+1];

extern m_real			PenvIKerrorBound;
extern m_real			PenvIKtolerance;

#endif