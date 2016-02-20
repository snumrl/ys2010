import psyco; psyco.full()
from fltk import *
import copy, os.path, cPickle, time
import numpy as np

import sys
if '../PyCommon/modules' not in sys.path:
    sys.path.append('../PyCommon/modules')
import Math.mmMath as mm
import Math.csMath as cm
import Math.ysFunctionGraph as yfg
import Renderer.ysRenderer as yr
import Renderer.csVpRenderer as cvr
import Simulator.csVpWorld as cvw
import Simulator.csVpModel as cvm
import Simulator.ysVpUtil as yvu
import GUI.ysSimpleViewer as ysv
import GUI.ysMultiViewer as ymv
import ArticulatedBody.ysControl as yct
import ArticulatedBody.ysReferencePoints as yrp
import Motion.ysMotionAnalysis as yma
import Motion.ysBipedAnalysis as yba
import Motion.ysMotion as ym
import Motion.ysMotionBlend as ymb
import Motion.ysMotionExtend as ymt
import Motion.ysSkeletonEdit as yse
import Motion.ysTrajectoryEdit as yte
import Motion.mmAnalyticIK as aik
import Util.ysMatplotEx as ymp
import Resource.ysMotionLoader as yf
import Simulator.ysPhysConfig as ypc

#MOTION_COLOR = (128,128,128)
#CHARACTER_COLOR = (102,102,153)
MOTION_COLOR = (213,111,162)
CHARACTER_COLOR = (20,166,188)
FEATURE_COLOR = (255,102,0)

def different_mass_distribution():
    
    class ForceInfo:
        def __init__(self, startFrame, duration, force):
            self.startFrame = startFrame    # frame
            self.duration = duration        # sec
            self.force = force              # Newton
            self.targetBody = None
            
    def normal_mcfg():
        mcfgfile = open(dir + 'mcfg', 'r')
        mcfg = cPickle.load(mcfgfile)
        mcfgfile.close()
        return mcfg

    def heavy1Leg_mcfg(scale):
        massMap = {}
        massMap = massMap.fromkeys(['Head', 'Head_Effector', 'Hips', 'LeftArm', 'LeftFoot', 'LeftForeArm', 'LeftHand', 'LeftHand_Effector', 'LeftLeg', 'LeftShoulder1', 'LeftToes', 'LeftToes_Effector', 'LeftUpLeg', 'RightArm', 'RightFoot', 'RightForeArm', 'RightHand', 'RightHand_Effector', 'RightLeg', 'RightShoulder', 'RightToes', 'RightToes_Effector', 'RightUpLeg', 'Spine', 'Spine1'], 0.)
        
        # torso : 10
        massMap['Hips'] += 2.
        massMap['Spine'] += 8.
        
        # head : 3
        massMap['Spine1'] += 3.
        
        # right upper arm : 2
        massMap['RightArm'] += 2.
        
        # left upper arm : 2
        massMap['LeftArm'] += 2.
        
        # right lower arm : 1
        massMap['RightForeArm'] = 1.
    #    massMap['RightForeArm'] = 2.
        
        # left lower arm : 1
        massMap['LeftForeArm'] = 1.
    #    massMap['LeftForeArm'] = 2.
        
        # right thigh : 7
        massMap['Hips'] += 2.
        massMap['RightUpLeg'] += 5.
        
        # left thigh : 7
        massMap['Hips'] += 2.
        massMap['LeftUpLeg'] += 5.
        
        # right shin : 5
        massMap['RightLeg'] += 5.
        
        # left shin : 5
#        massMap['LeftLeg'] += 5.
        massMap['LeftLeg'] += 5.*scale
        
        # right foot : 4
        massMap['RightFoot'] += 2.
        
        # left foot : 4
        massMap['LeftFoot'] += 2.
        
        mcfg = ypc.ModelConfig()
        mcfg.defaultDensity = 1000.
        mcfg.defaultBoneRatio = .9
    
        for name in massMap:
            node = mcfg.addNode(name)
            node.mass = massMap[name]
            
        node = mcfg.getNode('Hips')
        node.length = .2
        node.width = .25
        
        node = mcfg.getNode('Spine1')
        node.length = .2
        node.offset = (0,0,0.1)
        
        node = mcfg.getNode('Spine')
        node.width = .22
        
        node = mcfg.getNode('RightFoot')
        node.length = .25
        node.width = .1
        node.geom = 'MyFoot1'
        
        node = mcfg.getNode('LeftFoot')
        node.length = .25
        node.width = .1
        node.geom = 'MyFoot1'
        
        node = mcfg.getNode('LeftLeg')
        node.density *= scale
        
        cylinderRadius = 0.125811/2. * 1.
        cylinderRadius *= scale**.5
        cylinderLength = 0.25
        
        return mcfg, (cylinderRadius, cylinderLength)
    
#    def heavyBody_mcfg():
#        massMap = {}
#        massMap = massMap.fromkeys(['Head', 'Head_Effector', 'Hips', 'LeftArm', 'LeftFoot', 'LeftForeArm', 'LeftHand', 'LeftHand_Effector', 'LeftLeg', 'LeftShoulder1', 'LeftToes', 'LeftToes_Effector', 'LeftUpLeg', 'RightArm', 'RightFoot', 'RightForeArm', 'RightHand', 'RightHand_Effector', 'RightLeg', 'RightShoulder', 'RightToes', 'RightToes_Effector', 'RightUpLeg', 'Spine', 'Spine1'], 0.)
#        
#        # torso : 10
#        massMap['Hips'] += 2.
#        massMap['Spine'] += 8.
#        
#        # head : 3
#        massMap['Spine1'] += 3.
#        
#        # right upper arm : 2
#        massMap['RightArm'] += 2.
#        
#        # left upper arm : 2
#        massMap['LeftArm'] += 2.
#        
#        # right lower arm : 1
#        massMap['RightForeArm'] = 1.
#    #    massMap['RightForeArm'] = 2.
#        
#        # left lower arm : 1
#        massMap['LeftForeArm'] = 1.
#    #    massMap['LeftForeArm'] = 2.
#        
#        # right thigh : 7
#        massMap['Hips'] += 2.
#        massMap['RightUpLeg'] += 5.
#        
#        # left thigh : 7
#        massMap['Hips'] += 2.
#        massMap['LeftUpLeg'] += 5.
#        
#        # right shin : 5
#        massMap['RightLeg'] += 5.
#        
#        # left shin : 5
#        massMap['LeftLeg'] += 5.
#        
#        # right foot : 4
#        massMap['RightFoot'] += 2.
#        
#        # left foot : 4
#        massMap['LeftFoot'] += 2.
#        
#        mcfg = ypc.ModelConfig()
#        mcfg.defaultDensity = 1000.
#        mcfg.defaultBoneRatio = .9
#    
#        for name in massMap:
#            node = mcfg.addNode(name)
##            node.mass = massMap[name]
#            node.mass = massMap[name]*2
#            
#        node = mcfg.getNode('Hips')
#        node.length = .2
#        node.width = .25
#        
#        node = mcfg.getNode('Spine1')
#        node.length = .2
#        node.offset = (0,0,0.1)
#        
#        node = mcfg.getNode('Spine')
#        node.width = .22
#        
#        node = mcfg.getNode('RightFoot')
#        node.length = .25
#        node.width = .1
#        node.geom = 'MyFoot1'
#        
#        node = mcfg.getNode('LeftFoot')
#        node.length = .25
#        node.width = .1
#        node.geom = 'MyFoot1'
#        
#        return mcfg
#    
#    def heavyTorso_mcfg():
#        massMap = {}
#        massMap = massMap.fromkeys(['Head', 'Head_Effector', 'Hips', 'LeftArm', 'LeftFoot', 'LeftForeArm', 'LeftHand', 'LeftHand_Effector', 'LeftLeg', 'LeftShoulder1', 'LeftToes', 'LeftToes_Effector', 'LeftUpLeg', 'RightArm', 'RightFoot', 'RightForeArm', 'RightHand', 'RightHand_Effector', 'RightLeg', 'RightShoulder', 'RightToes', 'RightToes_Effector', 'RightUpLeg', 'Spine', 'Spine1'], 0.)
#        
#        # torso : 10
#        massMap['Hips'] += 2.
#        massMap['Spine'] += 8.
#        
#        # head : 3
#        massMap['Spine1'] += 3.
#        
#        # right upper arm : 2
#        massMap['RightArm'] += 2.
#        
#        # left upper arm : 2
#        massMap['LeftArm'] += 2.
#        
#        # right lower arm : 1
#        massMap['RightForeArm'] = 1.
#    #    massMap['RightForeArm'] = 2.
#        
#        # left lower arm : 1
#        massMap['LeftForeArm'] = 1.
#    #    massMap['LeftForeArm'] = 2.
#        
#        # right thigh : 7
#        massMap['Hips'] += 2.
#        massMap['RightUpLeg'] += 5.
#        
#        # left thigh : 7
#        massMap['Hips'] += 2.
#        massMap['LeftUpLeg'] += 5.
#        
#        # right shin : 5
#        massMap['RightLeg'] += 5.
#        
#        # left shin : 5
#        massMap['LeftLeg'] += 5.
#        
#        # right foot : 4
#        massMap['RightFoot'] += 2.
#        
#        # left foot : 4
#        massMap['LeftFoot'] += 2.
#        
#        massMap['Spine'] *= 1.6
#        
#        mcfg = ypc.ModelConfig()
#        mcfg.defaultDensity = 1000.
#        mcfg.defaultBoneRatio = .9
#    
#        for name in massMap:
#            node = mcfg.addNode(name)
#            node.mass = massMap[name]
#            
#        node = mcfg.getNode('Hips')
#        node.length = .2
#        node.width = .25
#        
#        node = mcfg.getNode('Spine1')
#        node.length = .2
#        node.offset = (0,0,0.1)
#        
#        node = mcfg.getNode('Spine')
#        node.width = .22
#        node.offset = (0,0.05,0)
#        
#        node = mcfg.getNode('RightFoot')
#        node.length = .25
#        node.width = .1
#        node.geom = 'MyFoot1'
#        
#        node = mcfg.getNode('LeftFoot')
#        node.length = .25
#        node.width = .1
#        node.geom = 'MyFoot1'
#        
#        return mcfg
#    
#    def heavy2Legs_mcfg():
#        massMap = {}
#        massMap = massMap.fromkeys(['Head', 'Head_Effector', 'Hips', 'LeftArm', 'LeftFoot', 'LeftForeArm', 'LeftHand', 'LeftHand_Effector', 'LeftLeg', 'LeftShoulder1', 'LeftToes', 'LeftToes_Effector', 'LeftUpLeg', 'RightArm', 'RightFoot', 'RightForeArm', 'RightHand', 'RightHand_Effector', 'RightLeg', 'RightShoulder', 'RightToes', 'RightToes_Effector', 'RightUpLeg', 'Spine', 'Spine1'], 0.)
#        
#        # torso : 10
#        massMap['Hips'] += 2.
#        massMap['Spine'] += 8.
#        
#        # head : 3
#        massMap['Spine1'] += 3.
#        
#        # right upper arm : 2
#        massMap['RightArm'] += 2.
#        
#        # left upper arm : 2
#        massMap['LeftArm'] += 2.
#        
#        # right lower arm : 1
#        massMap['RightForeArm'] = 1.
#    #    massMap['RightForeArm'] = 2.
#        
#        # left lower arm : 1
#        massMap['LeftForeArm'] = 1.
#    #    massMap['LeftForeArm'] = 2.
#        
#        # right thigh : 7
#        massMap['Hips'] += 2.
#        massMap['RightUpLeg'] += 5.
#        
#        # left thigh : 7
#        massMap['Hips'] += 2.
#        massMap['LeftUpLeg'] += 5.
#        
#        # right shin : 5
#        massMap['RightLeg'] += 5.
#        
#        # left shin : 5
#        massMap['LeftLeg'] += 5.
#        
#        # right foot : 4
#        massMap['RightFoot'] += 2.
#        
#        # left foot : 4
#        massMap['LeftFoot'] += 2.
#        
#        mcfg = ypc.ModelConfig()
#        mcfg.defaultDensity = 1000.
#        mcfg.defaultBoneRatio = .9
#    
#        for name in massMap:
#            node = mcfg.addNode(name)
#            node.mass = massMap[name]*2. if 'Leg' in name else massMap[name]
#            
#        node = mcfg.getNode('Hips')
#        node.length = .2
#        node.width = .25
#        
#        node = mcfg.getNode('Spine1')
#        node.length = .2
#        node.offset = (0,0,0.1)
#        
#        node = mcfg.getNode('Spine')
#        node.width = .22
#        
#        node = mcfg.getNode('RightFoot')
#        node.length = .25
#        node.width = .1
#        node.geom = 'MyFoot1'
#        
#        node = mcfg.getNode('LeftFoot')
#        node.length = .25
#        node.width = .1
#        node.geom = 'MyFoot1'
#        
#        return mcfg
#    
#    def heavyHead_mcfg():
#        massMap = {}
#        massMap = massMap.fromkeys(['Head', 'Head_Effector', 'Hips', 'LeftArm', 'LeftFoot', 'LeftForeArm', 'LeftHand', 'LeftHand_Effector', 'LeftLeg', 'LeftShoulder1', 'LeftToes', 'LeftToes_Effector', 'LeftUpLeg', 'RightArm', 'RightFoot', 'RightForeArm', 'RightHand', 'RightHand_Effector', 'RightLeg', 'RightShoulder', 'RightToes', 'RightToes_Effector', 'RightUpLeg', 'Spine', 'Spine1'], 0.)
#        
#        # torso : 10
#        massMap['Hips'] += 2.
#        massMap['Spine'] += 8.
#        
#        # head : 3
#        massMap['Spine1'] += 3.*3
#        
#        # right upper arm : 2
#        massMap['RightArm'] += 2.
#        
#        # left upper arm : 2
#        massMap['LeftArm'] += 2.
#        
#        # right lower arm : 1
#        massMap['RightForeArm'] = 1.
#    #    massMap['RightForeArm'] = 2.
#        
#        # left lower arm : 1
#        massMap['LeftForeArm'] = 1.
#    #    massMap['LeftForeArm'] = 2.
#        
#        # right thigh : 7
#        massMap['Hips'] += 2.
#        massMap['RightUpLeg'] += 5.
#        
#        # left thigh : 7
#        massMap['Hips'] += 2.
#        massMap['LeftUpLeg'] += 5.
#        
#        # right shin : 5
#        massMap['RightLeg'] += 5.
#        
#        # left shin : 5
#        massMap['LeftLeg'] += 5.
#        
#        # right foot : 4
#        massMap['RightFoot'] += 2.
#        
#        # left foot : 4
#        massMap['LeftFoot'] += 2.
#        
#        mcfg = ypc.ModelConfig()
#        mcfg.defaultDensity = 1000.
#        mcfg.defaultBoneRatio = .9
#    
#        for name in massMap:
#            node = mcfg.addNode(name)
#            node.mass = massMap[name]
#            
#        node = mcfg.getNode('Hips')
#        node.length = .2
#        node.width = .25
#        
#        node = mcfg.getNode('Spine1')
#        node.length = .2
#        node.offset = (0,0,0.1)
#        
#        node = mcfg.getNode('Spine')
#        node.width = .22
#        
#        node = mcfg.getNode('RightFoot')
#        node.length = .25
#        node.width = .1
#        node.geom = 'MyFoot1'
#        
#        node = mcfg.getNode('LeftFoot')
#        node.length = .25
#        node.width = .1
#        node.geom = 'MyFoot1'
#        
#        return mcfg
    
    #===============================================================================
    # load motion
    #===============================================================================
    MULTI_VIEWER = True
    CAMERA_TRACKING = True
    
    SAVE_SIMULATION = False
    SAVE_DIR = './saved_simulation/simulated_'
    
    # global parameters
    Kt = 20.;       Dt = 2*(Kt**.5)
    Ks = 2000.;    Ds = 2*(Ks**.5)
    K_stb_vel = .1
    mu = 1.
    
    # constaants
    c_min_contact_vel = 100.
#    c_min_contact_vel = 2.
    c_min_contact_time = .7
    c_landing_duration = .2
    c_taking_duration = .3
    c_swf_mid_offset = .0
    c_swf_stability = .5
    c_locking_vel = .05
    
    dir = './ppmotion/'
    

    K_swp_vel_sag = .1; K_swp_vel_cor = .4; K_swp_pos_sag = 1.; K_swp_pos_cor = 0.
    K_stp_pos = .6
    filename = 'wd2_WalkForwardNormal00_REPEATED.bvh'


    motion_ori = yf.readBvhFile(dir+filename)
    frameTime = 1/motion_ori.fps
    
    if not SAVE_SIMULATION:
        motion_saved = yf.readBvhFile(SAVE_DIR + filename)
    
    if 'REPEATED' in filename:
        REPEATED = True
        CAMERA_TRACKING = True
    else:
        REPEATED = False

    #===============================================================================
    # options
    #===============================================================================
    SEGMENT_EDITING =           True
    STANCE_FOOT_STABILIZE =     True
    MATCH_STANCE_LEG =          True
    SWING_FOOT_PLACEMENT =      True
    SWING_FOOT_HEIGHT =         True
    
    SWING_FOOT_ORIENTATION =    False
    
    STANCE_FOOT_PUSH =          True
    STANCE_FOOT_BALANCING =     True
    
    stitch_func = lambda x : 1. - yfg.hermite2nd(x)
    stf_stabilize_func = yfg.concatenate([yfg.hermite2nd, yfg.one], [c_landing_duration])
    match_stl_func = yfg.hermite2nd
#    match_stl_func_y = yfg.hermite2nd
    swf_placement_func = yfg.hermite2nd
    swf_height_func = yfg.hermite2nd
    swf_height_sine_func = yfg.sine
    stf_balancing_func = yfg.concatenate([yfg.hermite2nd, yfg.one], [c_landing_duration])
    
#    forceInfos = [ForceInfo(70, .4, (100,0,0))]
    forceInfos = []

    #===============================================================================
    # initialize character
    #===============================================================================
    
    cr = yr.CylindersRenderer([None], [mm.I_SE3()], FEATURE_COLOR)
    
    if SAVE_SIMULATION:
        motion_mcfg =  normal_mcfg()
        character_mcfg = motion_mcfg
    else:
        motion_mcfg = normal_mcfg()
        
        character_mcfg = normal_mcfg()

#        character_mcfg, cr.cylinderSizes[0] = heavy1Leg_mcfg(2.)
#        character_mcfg, cr.cylinderSizes[0] = heavy1Leg_mcfg(4.)
#        character_mcfg, cr.cylinderSizes[0] = heavy1Leg_mcfg(6.)
        character_mcfg, cr.cylinderSizes[0] = heavy1Leg_mcfg(8.)
        
#        character_mcfg = heavyTorso_mcfg()
        
##        character_mcfg = heavyBody_mcfg()
##        character_mcfg = heavy2Legs_mcfg()
##        character_mcfg = heavyHead_mcfg()

    wcfg = ypc.WorldConfig()
    wcfg.planeHeight = 0.
    wcfg.useDefaultContactModel = False
    wcfg.lockingVel = c_locking_vel
    stepsPerFrame = 30
    wcfg.timeStep = (frameTime)/stepsPerFrame
    
    vpWorld = cvw.VpWorld(wcfg)
    motionModel = cvm.VpMotionModel(vpWorld, motion_ori[0], motion_mcfg)
    controlModel = cvm.VpControlModel(vpWorld, motion_ori[0], character_mcfg)
    vpWorld.initialize()
    print controlModel
    
    controlModel.initializeHybridDynamics()
    
    #===============================================================================
    # load segment info
    #===============================================================================
    skeleton = motion_ori[0].skeleton
    
    segname = os.path.splitext(filename)[0]+'.seg'
    segfile = open(dir+segname, 'r')
    seginfo = cPickle.load(segfile)
    segfile.close()
    
    intervals = [info['interval'] for info in seginfo]
    states = [info['state'] for info in seginfo]
    temp_motion = copy.deepcopy(motion_ori)
    segments = yma.splitMotionIntoSegments(temp_motion, intervals)    
    print len(intervals), 'segments'
    for i in range(len(intervals)):
        print '%dth'%i, yba.GaitState.text[states[i]], intervals[i], ',', 
    print

    motion_seg_orig = ym.JointMotion()
    motion_seg_orig += segments[0]
    motion_seg = ym.JointMotion()
    motion_seg += segments[0]
    motion_stitch = ym.JointMotion()
    motion_stitch += segments[0]

    motion_stf_stabilize = ym.JointMotion()
    motion_match_stl = ym.JointMotion()
    motion_swf_placement = ym.JointMotion()
    motion_swf_height = ym.JointMotion()
    motion_swf_orientation = ym.JointMotion()
    motion_stf_balancing = ym.JointMotion()
    motion_stf_push = ym.JointMotion()
    motion_control = ym.JointMotion()

    motion_debug1 = ym.JointMotion()
    motion_debug2 = ym.JointMotion()
    motion_debug3 = ym.JointMotion()
    
    P = ym.JointMotion()
    P_hat = ym.JointMotion()
    M_tc = ym.JointMotion()
    M_hat_tc_1 = ym.JointMotion()
    
    if SAVE_SIMULATION:
        motion_saving = ym.JointMotion()
    
    #===============================================================================
    # loop variable
    #===============================================================================
    seg_index = [0]
    acc_offset = [0]
    extended = [False]
    prev_R_swp = [None]
    stl_y_limit_num = [0]
    stl_xz_limit_num = [0]
    avg_dCM = [mm.O_Vec3()]
#    avg_stf_v = [mm.O_Vec3()]
#    avg_stf_av = [mm.O_Vec3()]
    
#    stf_push_func = [yfg.zero]
    step_length_cur = [0.]

    step_length_tar = [0.]
    step_axis = [mm.O_Vec3()]
            
    #===============================================================================
    # information
    #===============================================================================
    bodyIDsToCheck = range(vpWorld.getBodyNum())
    mus = [mu]*len(bodyIDsToCheck)
    
    bodyMasses = controlModel.getBodyMasses()
    totalMass = controlModel.getTotalMass()

    lID = controlModel.name2id('LeftFoot');      rID = controlModel.name2id('RightFoot')
    lUpLeg = skeleton.getJointIndex('LeftUpLeg');rUpLeg = skeleton.getJointIndex('RightUpLeg')
    lKnee = skeleton.getJointIndex('LeftLeg');   rKnee = skeleton.getJointIndex('RightLeg')
    lFoot = skeleton.getJointIndex('LeftFoot');  rFoot = skeleton.getJointIndex('RightFoot')
    spine = skeleton.getJointIndex('Spine')
    
    uppers = [skeleton.getJointIndex(name) for name in ['Hips', 'Spine', 'Spine1', 'LeftArm', 'LeftForeArm', 'RightArm', 'RightForeArm']]
    upperMass = sum([bodyMasses[i] for i in uppers])
    lLegs = [skeleton.getJointIndex(name) for name in ['LeftUpLeg', 'LeftLeg', 'LeftFoot']]
    rLegs = [skeleton.getJointIndex(name) for name in ['RightUpLeg', 'RightLeg', 'RightFoot']]
    allJoints = set(range(skeleton.getJointNum()))
    
    halfFootHeight = controlModel.getBodyShape(lFoot)[1] / 2.
    
    for fi in forceInfos:
        fi.targetBody = spine
        
    #===============================================================================
    # rendering
    #===============================================================================
    rd_CM = [None]; rd_CP = [None]; rd_CMP = [None]
    rd_forces = [None]; rd_force_points = [None]
    rd_torques = []; rd_joint_positions = []
    
    rd_point1 = [None]
    rd_point2 = [None]
    rd_vec1 = [None];   rd_vecori1 = [None] 
    rd_vec2 = [None];   rd_vecori2 = [None]
    rd_frame1 = [None]
    rd_frame2 = [None]
    
    if MULTI_VIEWER:
        if SAVE_SIMULATION:
            viewer = ymv.MultiViewer(800, 655)
            viewer.setRenderers1([cvr.VpModelRenderer(motionModel, MOTION_COLOR, yr.POLYGON_FILL)])
            viewer.setRenderers2([cvr.VpModelRenderer(controlModel, CHARACTER_COLOR, yr.POLYGON_FILL)])
        else:
            viewer = ymv.MultiViewer(800, 655)
#            viewer = ymv.MultiViewer(1600, 1255)
            viewer.setRenderers1([cvr.VpModelRenderer(motionModel, CHARACTER_COLOR, yr.POLYGON_FILL)])
            viewer.setRenderers2([cvr.VpModelRenderer(controlModel, CHARACTER_COLOR, yr.POLYGON_FILL), cr])
#        viewer.glWindow1.groundOffset = [-15, 0, 0]
#        viewer.glWindow2.groundOffset = [-15, 0, 0]
    else:
        viewer = ysv.SimpleViewer()
    #    viewer.record(False)
    
        viewer.doc.addRenderer('motionModel', cvr.VpModelRenderer(motionModel, (0,150,255), yr.POLYGON_LINE))
        viewer.doc.addRenderer('controlModel', cvr.VpModelRenderer(controlModel, (200,200,200), yr.POLYGON_LINE))
        
    #    viewer.doc.addObject('motion_ori', motion_ori)
    #    viewer.doc.addRenderer('motion_ori', yr.JointMotionRenderer(motion_ori, (0,100,255), yr.LINK_BONE))
#        viewer.doc.addRenderer('motion_seg_orig', yr.JointMotionRenderer(motion_seg_orig, (0,100,255), yr.LINK_BONE))
#        viewer.doc.addRenderer('motion_seg', yr.JointMotionRenderer(motion_seg, (0,150,255), yr.LINK_BONE))
#        viewer.doc.addRenderer('motion_stitch', yr.JointMotionRenderer(motion_stitch, (0,255,200), yr.LINK_BONE))
#        viewer.doc.addRenderer('motion_stf_stabilize', yr.JointMotionRenderer(motion_stf_stabilize, (255,0,0), yr.LINK_BONE))
#        viewer.doc.addRenderer('motion_match_stl', yr.JointMotionRenderer(motion_match_stl, (255,200,0), yr.LINK_BONE))
#        viewer.doc.addRenderer('motion_swf_placement', yr.JointMotionRenderer(motion_swf_placement, (255,100,255), yr.LINK_BONE))
#        viewer.doc.addRenderer('motion_swf_height', yr.JointMotionRenderer(motion_swf_height, (50,255,255), yr.LINK_BONE))
        viewer.doc.addRenderer('motion_swf_orientation', yr.JointMotionRenderer(motion_swf_orientation, (255,100,0), yr.LINK_BONE))
        viewer.doc.addRenderer('motion_stf_push', yr.JointMotionRenderer(motion_stf_push, (50,255,200), yr.LINK_BONE))
    #    viewer.doc.addRenderer('motion_stf_balancing', yr.JointMotionRenderer(motion_stf_balancing, (255,100,255), yr.LINK_BONE))
#        viewer.doc.addRenderer('motion_control', yr.JointMotionRenderer(motion_control, (255,0,0), yr.LINK_BONE))
    
#        viewer.doc.addRenderer('motion_debug1', yr.JointMotionRenderer(motion_debug1, (0,255,0), yr.LINK_BONE))
#        viewer.doc.addRenderer('motion_debug2', yr.JointMotionRenderer(motion_debug2, (255,0,255), yr.LINK_BONE))
#        viewer.doc.addRenderer('motion_debug3', yr.JointMotionRenderer(motion_debug3, (255,255,0), yr.LINK_BONE))

#        viewer.doc.addRenderer('M_tc', yr.JointMotionRenderer(M_tc, (255,255,0), yr.LINK_BONE))
#        viewer.doc.addRenderer('P_hat', yr.JointMotionRenderer(P_hat, (255,255,0), yr.LINK_BONE))
#        viewer.doc.addRenderer('P', yr.JointMotionRenderer(P, (255,255,0), yr.LINK_BONE))
#        viewer.doc.addRenderer('M_hat_tc_1', yr.JointMotionRenderer(M_hat_tc_1, (255,255,0), yr.LINK_BONE))
    
    #    viewer.doc.addRenderer('rd_CM', yr.PointsRenderer(rd_CM, (255,255,0)))
    #    viewer.doc.addRenderer('rd_CP', yr.PointsRenderer(rd_CP, (255,0,0)))
    #    viewer.doc.addRenderer('rd_CMP', yr.PointsRenderer(rd_CMP, (0,255,0)))
    #    viewer.doc.addRenderer('forces', yr.ForcesRenderer(rd_forces, rd_force_points, (255,0,0), ratio=.01, fromPoint=False))
#        viewer.doc.addRenderer('torques', yr.VectorsRenderer(rd_torques, rd_joint_positions, (255,0,0)))
    
    #    viewer.doc.addRenderer('rd_point1', yr.PointsRenderer(rd_point1, (0,255,0)))
    #    viewer.doc.addRenderer('rd_point2', yr.PointsRenderer(rd_point2, (255,0,0)))
#        viewer.doc.addRenderer('rd_vec1', yr.VectorsRenderer(rd_vec1, rd_vecori1, (255,0,0)))
    #    viewer.doc.addRenderer('rd_vec2', yr.VectorsRenderer(rd_vec2, rd_vecori2, (0,255,0)))
    #    viewer.doc.addRenderer('rd_frame1', yr.FramesRenderer(rd_frame1, (0,200,200)))
    #    viewer.doc.addRenderer('rd_frame2', yr.FramesRenderer(rd_frame2, (200,200,0)))
    #    viewer.setMaxFrame(len(motion_ori)-1)

    if not REPEATED:
        viewer.setMaxFrame(len(motion_ori)-1)
    else:
        viewer.setMaxFrame(300)
        
    if CAMERA_TRACKING:
        if MULTI_VIEWER:
            cameraTargets1 = [None] * (viewer.getMaxFrame()+1)
            cameraTargets2 = [None] * (viewer.getMaxFrame()+1)
        else:
            cameraTargets = [None] * (viewer.getMaxFrame()+1)
            
    pt = [0.]
    def postFrameCallback_Always(frame):
        if frame==1: pt[0] = time.time()
        if frame==31: print 'elapsed time for 30 frames:', time.time()-pt[0]
        if CAMERA_TRACKING:
            if MULTI_VIEWER:
                if cameraTargets1[frame]==None:
                    cameraTargets1[frame] = motionModel.getBodyPositionGlobal(0)
#                    cameraTargets1[frame] = motion_ori[frame].getJointPositionGlobal(0)
                viewer.setCameraTarget1(cameraTargets1[frame])
                
                if cameraTargets2[frame]==None:
                    cameraTargets2[frame] = controlModel.getJointPositionGlobal(0)
                viewer.setCameraTarget2(cameraTargets2[frame])
                
            else:
                if cameraTargets[frame]==None:
                    cameraTargets[frame] = controlModel.getJointPositionGlobal(0)
                viewer.setCameraTarget(cameraTargets[frame])
        if plot!=None:
            plot.updateVline(frame)
    viewer.setPostFrameCallback_Always(postFrameCallback_Always)

    plot = None
#    plot = ymp.InteractivePlot()
    if plot!=None:
        plot.setXlimit(0, len(motion_ori))
        plot.setYlimit(0., 1.)
        plot.addDataSet('zero')
        plot.addDataSet('diff')
        plot.addDataSet('debug1')
        plot.addDataSet('debug2')

    def viewer_onClose(data):
        if plot!=None:
            plot.close()
        viewer.onClose(data)
    viewer.callback(viewer_onClose)
         
    def simulateCallback(frame):
        # seginfo
        segIndex = seg_index[0]
        curState = seginfo[segIndex]['state']
        curInterval = yma.offsetInterval(acc_offset[0], seginfo[segIndex]['interval'])
        stanceLegs = seginfo[segIndex]['stanceHips']
        swingLegs = seginfo[segIndex]['swingHips']
        stanceFoots = seginfo[segIndex]['stanceFoots']
        swingFoots = seginfo[segIndex]['swingFoots']
        swingKnees = seginfo[segIndex]['swingKnees']
        groundHeight = seginfo[segIndex]['ground_height']
        maxStfPushFrame = seginfo[segIndex]['max_stf_push_frame']
        
        prev_frame = frame-1 if frame>0 else 0
#        prev_frame = frame
        
        # information
#        dCM_tar = yrp.getCM(motion_seg.getJointVelocitiesGlobal(frame), bodyMasses, upperMass, uppers)
#        CM_tar = yrp.getCM(motion_seg.getJointPositionsGlobal(frame), bodyMasses, upperMass, uppers)
##        dCM_tar = yrp.getCM(motion_seg.getJointVelocitiesGlobal(frame), bodyMasses, totalMass)
##        CM_tar = yrp.getCM(motion_seg.getJointPositionsGlobal(frame), bodyMasses, totalMass)
#        stf_tar = motion_seg.getJointPositionGlobal(stanceFoots[0], frame)
#        CMr_tar = CM_tar - stf_tar

        dCM_tar = motion_seg.getJointVelocityGlobal(0, prev_frame)
        CM_tar = motion_seg.getJointPositionGlobal(0, prev_frame)
#        dCM_tar = yrp.getCM(motion_seg.getJointVelocitiesGlobal(prev_frame), bodyMasses, upperMass, uppers)
#        CM_tar = yrp.getCM(motion_seg.getJointPositionsGlobal(prev_frame), bodyMasses, upperMass, uppers)
#        dCM_tar = yrp.getCM(motion_seg.getJointVelocitiesGlobal(prev_frame), bodyMasses, totalMass)
#        CM_tar = yrp.getCM(motion_seg.getJointPositionsGlobal(prev_frame), bodyMasses, totalMass)
        stf_tar = motion_seg.getJointPositionGlobal(stanceFoots[0], prev_frame)
        CMr_tar = CM_tar - stf_tar
            
        dCM = avg_dCM[0]
        CM = controlModel.getJointPositionGlobal(0)
#        CM = yrp.getCM(controlModel.getJointPositionsGlobal(), bodyMasses, upperMass, uppers)
#        CM = yrp.getCM(controlModel.getJointPositionsGlobal(), bodyMasses, totalMass)
        CMreal = yrp.getCM(controlModel.getJointPositionsGlobal(), bodyMasses, totalMass)
        stf = controlModel.getJointPositionGlobal(stanceFoots[0])
        CMr = CM - stf
        
        diff_dCM = mm.projectionOnPlane(dCM-dCM_tar, (1,0,0), (0,0,1))
        diff_dCM_axis = np.cross((0,1,0), diff_dCM)
        rd_vec1[0] = diff_dCM; rd_vecori1[0] = CM_tar
        
        diff_CMr = mm.projectionOnPlane(CMr-CMr_tar, (1,0,0), (0,0,1))
#        rd_vec1[0] = diff_CMr; rd_vecori1[0] = stf_tar
        diff_CMr_axis = np.cross((0,1,0), diff_CMr)
        
        direction = mm.normalize2(mm.projectionOnPlane(dCM_tar, (1,0,0), (0,0,1)))
#        direction = mm.normalize2(mm.projectionOnPlane(dCM, (1,0,0), (0,0,1)))
        directionAxis = np.cross((0,1,0), direction)
        
        diff_dCM_sag, diff_dCM_cor = mm.projectionOnVector2(diff_dCM, direction)
#        rd_vec1[0] = diff_dCM_sag; rd_vecori1[0] = CM_tar
        diff_dCM_sag_axis = np.cross((0,1,0), diff_dCM_sag)
        diff_dCM_cor_axis = np.cross((0,1,0), diff_dCM_cor)
            
        diff_CMr_sag, diff_CMr_cor = mm.projectionOnVector2(diff_CMr, direction)
        diff_CMr_sag_axis = np.cross((0,1,0), diff_CMr_sag)
        diff_CMr_cor_axis = np.cross((0,1,0), diff_CMr_cor)
            
        t = (frame-curInterval[0])/float(curInterval[1]-curInterval[0])
        t_raw = t
        if t>1.: t=1.
        
        
        p_root = motion_stitch[frame].getJointPositionGlobal(0)
        R_root = motion_stitch[frame].getJointOrientationGlobal(0)

        motion_seg_orig.goToFrame(frame)
        motion_seg.goToFrame(frame)
        motion_stitch.goToFrame(frame)
        
        motion_debug1.append(motion_stitch[frame].copy())
        motion_debug1.goToFrame(frame)
        motion_debug2.append(motion_stitch[frame].copy())
        motion_debug2.goToFrame(frame)
        motion_debug3.append(motion_stitch[frame].copy())
        motion_debug3.goToFrame(frame)
        
        # paper implementation
        M_tc.append(motion_stitch[prev_frame])
        M_tc.goToFrame(frame)
        P_hat.append(M_tc[frame].copy())
        P_hat.goToFrame(frame)
        
        p_temp = ym.JointPosture(skeleton)
        p_temp.rootPos = controlModel.getJointPositionGlobal(0)
        p_temp.setJointOrientationsLocal(controlModel.getJointOrientationsLocal())
        P.append(p_temp)
        P.goToFrame(frame)
        
        # stance foot stabilize
        motion_stf_stabilize.append(motion_stitch[frame].copy())
        motion_stf_stabilize.goToFrame(frame)
        if STANCE_FOOT_STABILIZE:
            for stanceFoot in stanceFoots:
                R_target_foot = motion_seg[frame].getJointOrientationGlobal(stanceFoot)
                R_current_foot = motion_stf_stabilize[frame].getJointOrientationGlobal(stanceFoot)
                motion_stf_stabilize[frame].setJointOrientationGlobal(stanceFoot, cm.slerp(R_current_foot, R_target_foot , stf_stabilize_func(t)))
#                R_target_foot = motion_seg[frame].getJointOrientationLocal(stanceFoot)
#                R_current_foot = motion_stf_stabilize[frame].getJointOrientationLocal(stanceFoot)
#                motion_stf_stabilize[frame].setJointOrientationLocal(stanceFoot, cm.slerp(R_current_foot, R_target_foot , stf_stabilize_func(t)))

        # match stance leg 
        motion_match_stl.append(motion_stf_stabilize[frame].copy())
        motion_match_stl.goToFrame(frame)
        if MATCH_STANCE_LEG:
            if curState!=yba.GaitState.STOP:
                for i in range(len(stanceLegs)):
                    stanceLeg = stanceLegs[i]
                    stanceFoot = stanceFoots[i]
                    
#                    # motion stance leg -> character stance leg as time goes
                    R_motion = motion_match_stl[frame].getJointOrientationGlobal(stanceLeg)
                    R_character = controlModel.getJointOrientationGlobal(stanceLeg)
                    motion_match_stl[frame].setJointOrientationGlobal(stanceLeg, cm.slerp(R_motion, R_character, match_stl_func(t)))

#                    t_y = match_stl_func_y(t)
#                    t_xz = match_stl_func(t)
#                    
#                    R_motion = motion_match_stl[frame].getJointOrientationGlobal(stanceLeg)
#                    R_character = controlModel.getJointOrientationGlobal(stanceLeg)
#                    R = np.dot(R_character, R_motion.T)
#                    R_y, R_xz = mm.projectRotation((0,1,0), R)
#                    motion_match_stl[frame].mulJointOrientationGlobal(stanceLeg, mm.scaleSO3(R_xz, t_xz))
#                    motion_match_stl[frame].mulJointOrientationGlobal(stanceLeg, mm.scaleSO3(R_y, t_y))

        # swing foot placement
        motion_swf_placement.append(motion_match_stl[frame].copy())
        motion_swf_placement.goToFrame(frame)
        if SWING_FOOT_PLACEMENT:
            t_swing_foot_placement = swf_placement_func(t);
            
            if extended[0]:
                R_swp_sag = prev_R_swp[0][0]
                R_swp_cor = prev_R_swp[0][1]
            else:
                R_swp_sag = mm.I_SO3(); R_swp_cor = mm.I_SO3()
                R_swp_sag = np.dot(R_swp_sag, mm.exp(diff_dCM_sag_axis * K_swp_vel_sag * -t_swing_foot_placement))
                R_swp_cor = np.dot(R_swp_cor, mm.exp(diff_dCM_cor_axis * K_swp_vel_cor * -t_swing_foot_placement))
                if np.dot(direction, diff_CMr_sag) < 0:
                    R_swp_sag = np.dot(R_swp_sag, mm.exp(diff_CMr_sag_axis * K_swp_pos_sag * -t_swing_foot_placement))
                R_swp_cor = np.dot(R_swp_cor, mm.exp(diff_CMr_cor_axis * K_swp_pos_cor * -t_swing_foot_placement))

            for i in range(len(swingLegs)):
                swingLeg = swingLegs[i]
                swingFoot = swingFoots[i] 
                
                # save swing foot global orientation
#                R_swf = motion_swf_placement[frame].getJointOrientationGlobal(swingFoot)
                
                # rotate swing leg
                motion_swf_placement[frame].mulJointOrientationGlobal(swingLeg, R_swp_sag)
                motion_swf_placement[frame].mulJointOrientationGlobal(swingLeg, R_swp_cor)
                
                # restore swing foot global orientation
#                motion_swf_placement[frame].setJointOrientationGlobal(swingFoot, R_swf)
                
                prev_R_swp[0] = (R_swp_sag, R_swp_cor)

        # swing foot height
        motion_swf_height.append(motion_swf_placement[frame].copy())
        motion_swf_height.goToFrame(frame)
        if SWING_FOOT_HEIGHT:
            for swingFoot in swingFoots:
                stanceFoot = stanceFoots[0]

                # save foot global orientation
                R_foot = motion_swf_height[frame].getJointOrientationGlobal(swingFoot)
                R_stance_foot = motion_swf_height[frame].getJointOrientationGlobal(stanceFoot)

                height_tar = motion_swf_height[frame].getJointPositionGlobal(swingFoot)[1] - motion_swf_height[frame].getJointPositionGlobal(stanceFoot)[1]
#                motion_debug1[frame] = motion_swf_height[frame].copy()

                # rotate
                motion_swf_height[frame].rotateByTarget(controlModel.getJointOrientationGlobal(0))
#                motion_debug2[frame] = motion_swf_height[frame].copy()
#                motion_debug2[frame].translateByTarget(controlModel.getJointPositionGlobal(0))

                height_cur = motion_swf_height[frame].getJointPositionGlobal(swingFoot)[1] - motion_swf_height[frame].getJointPositionGlobal(stanceFoot)[1]

                offset_height = (height_tar - height_cur) * swf_height_func(t) * c_swf_stability
                offset_sine = c_swf_mid_offset * swf_height_sine_func(t)
                
                offset = 0.
                offset += offset_height
                offset += offset_sine

                if offset > 0.:
                    newPosition =  motion_swf_height[frame].getJointPositionGlobal(swingFoot)
                    newPosition[1] += offset
                    aik.ik_analytic(motion_swf_height[frame], swingFoot, newPosition)
                else:
                    newPosition =  motion_swf_height[frame].getJointPositionGlobal(stanceFoot)
                    newPosition[1] -= offset
                    aik.ik_analytic(motion_swf_height[frame], stanceFoot, newPosition)

                # return
#                motion_debug3[frame] = motion_swf_height[frame].copy()
#                motion_debug3[frame].translateByTarget(controlModel.getJointPositionGlobal(0))
                motion_swf_height[frame].rotateByTarget(R_root)
                
                # restore foot global orientation
                motion_swf_height[frame].setJointOrientationGlobal(swingFoot, R_foot)
                motion_swf_height[frame].setJointOrientationGlobal(stanceFoot, R_stance_foot)

                if plot!=None:
                    plot.addDataPoint('debug1', frame, height_tar)
                    plot.addDataPoint('debug2', frame, height_cur)
#                    plot.addDataPoint('diff', frame, diff)

        # swing foot orientation
        motion_swf_orientation.append(motion_swf_height[frame].copy())
        motion_swf_orientation.goToFrame(frame)
        if SWING_FOOT_ORIENTATION:
#    swf_orientation_func = yfg.concatenate([yfg.zero, yfg.hermite2nd, yfg.one], [.25, .75])
            for swingFoot in swingFoots:
                R_target_foot = motion_seg[curInterval[1]].getJointOrientationGlobal(swingFoot)
                R_current_foot = motion_swf_orientation[frame].getJointOrientationGlobal(swingFoot)
                motion_swf_orientation[frame].setJointOrientationGlobal(swingFoot, cm.slerp(R_current_foot, R_target_foot, swf_orientation_func(t)))
#    swf_stabilize_func = yfg.concatenate([yfg.hermite2nd, yfg.one], [c_taking_duration])
            # push orientation
#            for swingFoot in swingFoots:
#                R_target_foot = motion_seg[frame].getJointOrientationGlobal(swingFoot)
#                R_current_foot = motion_swf_orientation[frame].getJointOrientationGlobal(swingFoot)
#                motion_swf_orientation[frame].setJointOrientationGlobal(swingFoot, cm.slerp(R_current_foot, R_target_foot , swf_stabilize_func(t)))
            
        # stance foot push                
        motion_stf_push.append(motion_swf_orientation[frame].copy())
        motion_stf_push.goToFrame(frame)
        if STANCE_FOOT_PUSH:
            for swingFoot in swingFoots:
#                max_t = (maxStfPushFrame)/float(curInterval[1]-curInterval[0])
#                stf_push_func = yfg.concatenate([yfg.sine, yfg.zero], [max_t*2])
                stf_push_func = yfg.concatenate([yfg.sine, yfg.zero], [c_taking_duration*2])
                
                R_swp_sag = mm.I_SO3()
#                R_swp_sag = np.dot(R_swp_sag, mm.exp(diff_dCM_sag_axis * K_stp_vel * -stf_push_func(t)))
                
#                if step_length_cur[0] < step_length_tar[0]:
#                    ratio = step_length_cur[0] / step_length_tar[0]
#                    R_max = maxmaxStfPushFrame
#                    R_zero = 
                R_swp_sag = np.dot(R_swp_sag, mm.exp((step_length_tar[0] - step_length_cur[0])*step_axis[0] * K_stp_pos * -stf_push_func(t)))
                    
                motion_stf_push[frame].mulJointOrientationGlobal(swingFoot, R_swp_sag)
                
        # stance foot balancing 
        motion_stf_balancing.append(motion_stf_push[frame].copy())
        motion_stf_balancing.goToFrame(frame)
        if STANCE_FOOT_BALANCING:
            R_stb = mm.exp(diff_dCM_axis * K_stb_vel * stf_balancing_func(t))
            for stanceFoot in stanceFoots:
                if frame < 5: continue
                motion_stf_balancing[frame].mulJointOrientationGlobal(stanceFoot, R_stb)
                    
        # control trajectory
        motion_control.append(motion_stf_balancing[frame].copy())
        motion_control.goToFrame(frame)
        
        #=======================================================================
        # tracking with inverse dynamics
        #=======================================================================
        th_r = motion_control.getDOFPositions(frame)
        th = controlModel.getDOFPositions()
        dth_r = motion_control.getDOFVelocities(frame)
        dth = controlModel.getDOFVelocities()
        ddth_r = motion_control.getDOFAccelerations(frame)
        ddth_des = yct.getDesiredDOFAccelerations(th_r, th, dth_r, dth, ddth_r, Kt, Dt)

        #=======================================================================
        # simulation
        #=======================================================================
        CP = mm.v3(0.,0.,0.)
        F = mm.v3(0.,0.,0.)
        avg_dCM[0] = mm.v3(0.,0.,0.)
        
        # external force rendering info
        del rd_forces[:]; del rd_force_points[:]
        for fi in forceInfos:
            if fi.startFrame <= frame and frame < fi.startFrame + fi.duration*(1/frameTime):
                rd_forces.append(fi.force)
                rd_force_points.append(controlModel.getBodyPositionGlobal(fi.targetBody))
                    
        for i in range(stepsPerFrame):
            
            bodyIDs, contactPositions, contactPositionLocals, contactForces = vpWorld.calcPenaltyForce(bodyIDsToCheck, mus, Ks, Ds)
            vpWorld.applyPenaltyForce(bodyIDs, contactPositionLocals, contactForces)
            
            # apply external force
            for fi in forceInfos:
                if fi.startFrame <= frame and frame < fi.startFrame + fi.duration*(1/frameTime):
                    controlModel.applyBodyForceGlobal(fi.targetBody, fi.force)
            
            controlModel.setDOFAccelerations(ddth_des)
            controlModel.solveHybridDynamics()
            
            vpWorld.step()
#            yvu.align2D(controlModel)

            if len(contactForces) > 0:
                CP += yrp.getCP(contactPositions, contactForces)
                F += sum(contactForces)
            avg_dCM[0] += controlModel.getJointVelocityGlobal(0)
#            avg_dCM[0] += yrp.getCM(controlModel.getJointVelocitiesGlobal(), bodyMasses, upperMass, uppers)
#            avg_dCM[0] += yrp.getCM(controlModel.getJointVelocitiesGlobal(), bodyMasses, totalMass)

#            if len(stanceFoots)>0:
#                avg_stf_v[0] += controlModel.getJointVelocityGlobal(stanceFoots[0])
#                avg_stf_av[0] += controlModel.getJointAngVelocityGlobal(stanceFoots[0])
        
        CP /= stepsPerFrame
        F /= stepsPerFrame
        avg_dCM[0] /= stepsPerFrame
        
#        if len(stanceFoots)>0:
#            avg_stf_v[0] /= stepsPerFrame
#            avg_stf_av[0] /= stepsPerFrame
#            rd_vec1[0] = avg_stf_av[0]; rd_vec1[0][0] = 0.; rd_vec1[0][2] = 0.
#            rd_vecori1[0]= controlModel.getJointPositionGlobal(stanceFoots[0])

        #=======================================================================
        # segment editing
        #=======================================================================
        lastFrame = False
        
        if SEGMENT_EDITING:
            if curState==yba.GaitState.STOP:
                if frame == len(motion_seg)-1:
                    lastFrame = True
                    
            elif (curState==yba.GaitState.LSWING or curState==yba.GaitState.RSWING) and t>c_min_contact_time:
                swingID = lID if curState==yba.GaitState.LSWING else rID

                contact = False
                if swingID in bodyIDs:
                    minContactVel = 1000.
                    for i in range(len(bodyIDs)):
                        if bodyIDs[i]==swingID:
                            vel = controlModel.getBodyVelocityGlobal(swingID, contactPositionLocals[i])
                            vel[1] = 0
                            contactVel = mm.length(vel)
                            if contactVel < minContactVel: minContactVel = contactVel 
                    if minContactVel < c_min_contact_vel: contact = True
                
                extended[0] = False
                
                if contact:
#                    print frame, 'foot touch'
                    lastFrame = True
                    acc_offset[0] += frame - curInterval[1]
                    
                elif frame == len(motion_seg)-1:
                    print frame, 'extend frame', frame+1
                    
                    preserveJoints = []
#                    preserveJoints = [lFoot, rFoot]
#                    preserveJoints = [lFoot, rFoot, lKnee, rKnee]
#                    preserveJoints = [lFoot, rFoot, lKnee, rKnee, lUpLeg, rUpLeg]
                    stanceKnees = [rKnee] if curState==yba.GaitState.LSWING else [lKnee]   
                    preserveJoints = [stanceFoots[0], stanceKnees[0], stanceLegs[0]]
   
                    diff = 3
                    motion_seg_orig.extend([motion_seg_orig[-1]])
                    motion_seg.extend(ymt.extendByIntegration_root(motion_seg, 1, diff))
                    
                    motion_stitch.extend(ymt.extendByIntegration_constant(motion_stitch, 1, preserveJoints, diff))

#                    # extend for swing foot ground speed matching & swing foot height lower
##                    extendedPostures = ymt.extendByIntegration(motion_stitch, 1, preserveJoints, diff)
##                    extendedPostures = [motion_stitch[-1]] 
##
#                    extendFrameNum = frame - curInterval[1] + 1
#                    k = 1.-extendFrameNum/5.
#                    if k<0.: k=0.
#                    extendedPostures = ymt.extendByIntegrationAttenuation(motion_stitch, 1, preserveJoints, diff, k)
#
##                    if len(swingFoots)>0 and np.inner(dCM_tar, dCM)>0.:
##                        print frame, 'speed matching'
##                        R_swf = motion_stitch[-1].getJointOrientationGlobal(swingFoots[0])
##                        
##                        p_swf = motion_stitch[-1].getJointPositionGlobal(swingFoots[0])
##                        v_swf = motion_stitch.getJointVelocityGlobal(swingFoots[0], frame-diff, frame)
##                        a_swf = motion_stitch.getJointAccelerationGlobal(swingFoots[0], frame-diff, frame)
##                        p_swf += v_swf * (frameTime) + a_swf * (frameTime)*(frameTime)
##                        aik.ik_analytic(extendedPostures[0], swingFoots[0], p_swf)
##                        
##                        extendedPostures[0].setJointOrientationGlobal(swingFoots[0], R_swf)
#
#                    motion_stitch.extend(extendedPostures)
                    
                    extended[0] = True
        else:
            if frame == len(motion_seg)-1: lastFrame = True
                    
        if lastFrame:
            if segIndex < len(segments)-1:
                print '%d (%d): end of %dth seg (%s, %s)'%(frame, frame-curInterval[1],segIndex, yba.GaitState.text[curState], curInterval)
                if plot!=None: plot.addDataPoint('diff', frame, (frame-curInterval[1])*.01)
                
                if len(stanceFoots)>0 and len(swingFoots)>0:
#                    step_cur = controlModel.getJointPositionGlobal(swingFoots[0]) - controlModel.getJointPositionGlobal(stanceFoots[0])
#                    step_tar = motion_seg[curInterval[1]].getJointPositionGlobal(swingFoots[0]) - motion_seg[curInterval[1]].getJointPositionGlobal(stanceFoots[0])
                    step_cur = controlModel.getJointPositionGlobal(0) - controlModel.getJointPositionGlobal(stanceFoots[0])
                    step_tar = motion_seg[curInterval[1]].getJointPositionGlobal(0) - motion_seg[curInterval[1]].getJointPositionGlobal(stanceFoots[0])
                    
                    step_cur = mm.projectionOnPlane(step_cur, (1,0,0), (0,0,1))
                    step_tar = mm.projectionOnPlane(step_tar, (1,0,0), (0,0,1))
                    
                    step_cur_sag, step_cur_cor = mm.projectionOnVector2(step_cur, direction)
                    step_tar_sag, step_tar_cor = mm.projectionOnVector2(step_tar, direction)
                    
                    step_length_tar[0] = mm.length(step_tar_sag)
                    if np.inner(step_tar_sag, step_cur_sag) > 0:
                        step_length_cur[0] = mm.length(step_cur_sag)
                    else:
                        step_length_cur[0] = -mm.length(step_cur_sag)
                    
                    step_axis[0] = directionAxis
                    
#                    rd_vec1[0] = step_tar_sag
#                    rd_vecori1[0] = motion_seg[curInterval[1]].getJointPositionGlobal(stanceFoots[0])
#                    rd_vec2[0] = step_cur_sag
#                    rd_vecori2[0] = controlModel.getJointPositionGlobal(stanceFoots[0])

                seg_index[0] += 1
                curSeg = segments[seg_index[0]]
                stl_y_limit_num[0] = 0
                stl_xz_limit_num[0] = 0
                
                del motion_seg_orig[frame+1:]
                motion_seg_orig.extend(ymb.getAttachedNextMotion(curSeg, motion_seg_orig[-1], False, False))
                
                del motion_seg[frame+1:]
                del motion_stitch[frame+1:]
                transitionLength = len(curSeg)-1

#                motion_seg.extend(ymb.getAttachedNextMotion(curSeg, motion_seg[-1], False, False))
#                motion_stitch.extend(ymb.getStitchedNextMotion(curSeg, motion_control[-1], transitionLength, stitch_func, True, False))

                d = motion_seg[-1] - curSeg[0]
                d.rootPos[1] = 0.
                motion_seg.extend(ymb.getAttachedNextMotion(curSeg, d, True, False))
                d = motion_control[-1] - curSeg[0]
                d.rootPos[1] = 0.
                motion_stitch.extend(ymb.getStitchedNextMotion(curSeg, d, transitionLength, stitch_func, True, False))
                
#                motion_seg.extend(ymb.getAttachedNextMotion(curSeg, motion_seg[-1], False, True))
#                motion_stitch.extend(ymb.getStitchedNextMotion(curSeg, motion_control[-1], transitionLength, stitch_func, True, True))
            else:
                motion_seg_orig.append(motion_seg_orig[-1])
                motion_seg.append(motion_seg[-1])
                motion_stitch.append(motion_control[-1])
                
        cr.Ts[0] = np.dot(controlModel.getJointFrame(lKnee), mm.R2T(mm.rotX(90*mm.RAD)))
        
        # rendering
        if SAVE_SIMULATION:
            motionModel.update(motion_ori[frame])
            
            p_temp = ym.JointPosture(skeleton)
            p_temp.initLocalRs()
            p_temp.rootPos = controlModel.getJointPositionGlobal(0)
            p_temp.setJointOrientationsLocal(controlModel.getJointOrientationsLocal())
            motion_saving.append(p_temp)
            
            if frame == viewer.getMaxFrame():
                yf.writeBvhFile(SAVE_DIR+filename, motion_saving)
                print SAVE_DIR+filename, 'saved'
        else:
            motionModel.update(motion_saved[frame])
            
        
    viewer.setSimulateCallback(simulateCallback)
    
    viewer.startTimer(frameTime / 1.4)
#    viewer.startTimer(frameTime * .4)
    viewer.show()
    
    Fl.run()
    
def different_skeleton_length():
    
    class ForceInfo:
        def __init__(self, startFrame, duration, force):
            self.startFrame = startFrame    # frame
            self.duration = duration        # sec
            self.force = force              # Newton
            self.targetBody = None
            
    def normal_mcfg():
        mcfgfile = open(dir + 'mcfg', 'r')
        mcfg = cPickle.load(mcfgfile)
        mcfgfile.close()
        return mcfg



    #===============================================================================
    # load motion
    #===============================================================================
    MULTI_VIEWER = True
    CAMERA_TRACKING = True
    
    SAVE_DIR = './saved_simulation/simulated_'
    
    # global parameters
    Kt = 20.;       Dt = 2*(Kt**.5)
    Ks = 2000.;    Ds = 2*(Ks**.5)
    K_stb_vel = .1
    mu = 1.
    
    # constaants
    c_min_contact_vel = 100.
#    c_min_contact_vel = 2.
    c_min_contact_time = .7
    c_landing_duration = .2
    c_taking_duration = .3
    c_swf_mid_offset = .0
    c_swf_stability = .5
    c_locking_vel = .05
    
    dir = './ppmotion/'
    
    # short2legs, long2legs
    K_swp_vel_sag = .1; K_swp_vel_cor = .4; K_swp_pos_sag = 1.; K_swp_pos_cor = 0.; K_swp_pos_sag_ext = 0.;
    K_stp_pos = 1.
#    filename = 'wd2_WalkForwardNormal00.bvh'
    filename = 'wd2_WalkForwardNormal00_REPEATED.bvh'

##    # short1leg
#    K_swp_vel_sag = .1; K_swp_vel_cor = .2; K_swp_pos_sag = 1.; K_swp_pos_cor = .2; K_swp_pos_sag_ext = .0;
#    K_stp_pos = 0.
##    filename = 'wd2_WalkHandWav00.bvh'
#    filename = 'wd2_WalkHandWav00_REPEATED.bvh'


    motion_ori = yf.readBvhFile(dir+filename)
    frameTime = 1/motion_ori.fps
    
    motion_saved = yf.readBvhFile(SAVE_DIR + filename)
    
    if 'REPEATED' in filename:
        REPEATED = True
        CAMERA_TRACKING = True
    else:
        REPEATED = False

    #===============================================================================
    # options
    #===============================================================================
    SEGMENT_EDITING =           True
    STANCE_FOOT_STABILIZE =     True
    MATCH_STANCE_LEG =          True
    SWING_FOOT_PLACEMENT =      True
    SWING_FOOT_HEIGHT =         True
    
    SWING_FOOT_ORIENTATION =    False
    
    STANCE_FOOT_PUSH =          True
    STANCE_FOOT_BALANCING =     True
    
    stitch_func = lambda x : 1. - yfg.hermite2nd(x)
    stf_stabilize_func = yfg.concatenate([yfg.hermite2nd, yfg.one], [c_landing_duration])
    match_stl_func = yfg.hermite2nd
#    match_stl_func_y = yfg.hermite2nd
    swf_placement_func = yfg.hermite2nd
    swf_height_func = yfg.hermite2nd
    swf_height_sine_func = yfg.sine
    stf_balancing_func = yfg.concatenate([yfg.hermite2nd, yfg.one], [c_landing_duration])
    
#    forceInfos = [ForceInfo(70, .4, (100,0,0))]
    forceInfos = []

    #===============================================================================
    # initialize character
    #===============================================================================
    def buildMassMap():
        massMap = {}
        massMap = massMap.fromkeys(['Head', 'Head_Effector', 'Hips', 'LeftArm', 'LeftFoot', 'LeftForeArm', 'LeftHand', 'LeftHand_Effector', 'LeftLeg', 'LeftShoulder1', 'LeftToes', 'LeftToes_Effector', 'LeftUpLeg', 'RightArm', 'RightFoot', 'RightForeArm', 'RightHand', 'RightHand_Effector', 'RightLeg', 'RightShoulder', 'RightToes', 'RightToes_Effector', 'RightUpLeg', 'Spine', 'Spine1'], 0.)
        
        # torso : 10
        massMap['Hips'] += 2.
        massMap['Spine'] += 8.
        
        # head : 3
        massMap['Spine1'] += 3.
        
        # right upper arm : 2
        massMap['RightArm'] += 2.
        
        # left upper arm : 2
        massMap['LeftArm'] += 2.
        
        # right lower arm : 1
        massMap['RightForeArm'] = 1.
    #    massMap['RightForeArm'] = 2.
        
        # left lower arm : 1
        massMap['LeftForeArm'] = 1.
    #    massMap['LeftForeArm'] = 2.
        
        # right thigh : 7
        massMap['Hips'] += 2.
        massMap['RightUpLeg'] += 5.
        
        # left thigh : 7
        massMap['Hips'] += 2.
        massMap['LeftUpLeg'] += 5.
        
        # right shin : 5
        massMap['RightLeg'] += 5.
        
        # left shin : 5
        massMap['LeftLeg'] += 5.
        
        # right foot : 4
        massMap['RightFoot'] += 2.
        
        # left foot : 4
        massMap['LeftFoot'] += 2.
        
        return massMap        
    def short2Legs_mcfg1():
        massMap = buildMassMap()
        
        mcfg = ypc.ModelConfig()
        mcfg.defaultDensity = 1000.
        mcfg.defaultBoneRatio = .9
        
        totalMass = 0.
        for name in massMap:
            node = mcfg.addNode(name)
            node.mass = massMap[name]
            
#            if 'Leg' in name: node.mass /= 4.
#            if 'Foot' in name: node.mass /= 2.
#            else: node.mass /= 2.
            totalMass += node.mass
        print totalMass
            
        node = mcfg.getNode('Hips')
        node.length = .2
        node.width = .25
        
        node = mcfg.getNode('Spine1')
        node.length = .2
        node.offset = (0,0,0.1)
        
        node = mcfg.getNode('Spine')
        node.width = .22
        
        node = mcfg.getNode('RightFoot')
#        node.length = .25
        node.length = .2
        node.width = .1
        node.geom = 'MyFoot1'
        
        node = mcfg.getNode('LeftFoot')
#        node.length = .25
        node.length = .2
        node.width = .1
        node.geom = 'MyFoot1'
        
        return mcfg

    def short2Legs_mcfg2():
        massMap = buildMassMap()
        
        mcfg = ypc.ModelConfig()
        mcfg.defaultDensity = 1000.
        mcfg.defaultBoneRatio = .9
        
        for name in massMap:
            node = mcfg.addNode(name)
            node.mass = massMap[name]
            
            if 'Leg' in name: node.mass /= 4.
            if 'Foot' in name: node.mass /= 2.
#            else: node.mass /= 2.
            
        node = mcfg.getNode('Hips')
        node.length = .2
        node.width = .25
        
        node = mcfg.getNode('Spine1')
        node.length = .2
        node.offset = (0,0,0.1)
        
        node = mcfg.getNode('Spine')
        node.width = .22
        
        node = mcfg.getNode('RightFoot')
#        node.length = .25
        node.length = .2
        node.width = .1
        node.geom = 'MyFoot1'
        
        node = mcfg.getNode('LeftFoot')
#        node.length = .25
        node.length = .2
        node.width = .1
        node.geom = 'MyFoot1'
        
        return mcfg

    def short2Legs_mcfg3():
        massMap = buildMassMap()
        
        mcfg = ypc.ModelConfig()
        mcfg.defaultDensity = 1000.
        mcfg.defaultBoneRatio = .9
        
        totalMass = 0.
        for name in massMap:
            node = mcfg.addNode(name)
            node.mass = massMap[name]
            
            if 'Leg' in name: node.mass /= 4.
#            if 'Foot' in name: node.mass /= 2.
            else: node.mass /= 2.
            
            totalMass += node.mass
        print totalMass
            
        node = mcfg.getNode('Hips')
        node.length = .2
        node.width = .25
        
        node = mcfg.getNode('Spine1')
        node.length = .2
        node.offset = (0,0,0.1)
        
        node = mcfg.getNode('Spine')
        node.width = .22
        
        node = mcfg.getNode('RightFoot')
#        node.length = .25
        node.length = .2
        node.width = .1
        node.geom = 'MyFoot1'
        
        node = mcfg.getNode('LeftFoot')
#        node.length = .25
        node.length = .2
        node.width = .1
        node.geom = 'MyFoot1'
        
        return mcfg

    def short2Legs_edit(motion, shorten_length=.21):
        offset_y = shorten_length
        yse.offsetJointLocal(motion, 'LeftLeg', (0, offset_y, 0), False)
        yse.offsetJointLocal(motion, 'RightLeg', (0, offset_y, 0), False)
        yse.offsetJointLocal(motion, 'LeftFoot', (0, offset_y, 0), False)
        yse.offsetJointLocal(motion, 'RightFoot', (0, offset_y, 0), False)
        motion.translateByOffset((0, -offset_y*2, 0))
    def short1Leg_edit(motion, shorten_length = .01):
        offset_y = shorten_length
        yse.offsetJointLocal(motion, 'LeftLeg', (0, offset_y, 0))
        yse.offsetJointLocal(motion, 'LeftFoot', (0, offset_y, 0))
        lFootPos = motion[0].getJointPositionGlobal(motion[0].skeleton.getJointIndex('LeftFoot'))
        rFootPos = motion[0].getJointPositionGlobal(motion[0].skeleton.getJointIndex('RightFoot'))
        yte.setPositionTarget2(motion, 'RightFoot', mm.v3(rFootPos[0], lFootPos[1], rFootPos[2]), [0,25], motion[0].skeleton.getJointIndex('RightFoot'))
    def long2Legs_edit(motion, extend_length=.21):
        offset_y = -extend_length
        yse.offsetJointLocal(motion, 'LeftLeg', (0, offset_y, 0), False)
        yse.offsetJointLocal(motion, 'RightLeg', (0, offset_y, 0), False)
        yse.offsetJointLocal(motion, 'LeftFoot', (0, offset_y, 0), False)
        yse.offsetJointLocal(motion, 'RightFoot', (0, offset_y, 0), False)
        motion.translateByOffset((0, -offset_y*2, 0))
        
    motion_mcfg = normal_mcfg()
    motion_firstPosture = copy.deepcopy(motion_ori[0])

    skeleton = motion_ori[0].skeleton 
    lFoot = skeleton.getJointIndex('LeftFoot')
    lHip = skeleton.getJointIndex('LeftUpLeg')
#    print motion_ori[0].skeleton
    print motion_ori[0].getJointPositionGlobal(lFoot)
    print motion_ori[0].getJointPositionGlobal(lHip)
    
    #===========================================================================
    # options
    #===========================================================================
    # short2legs
#    character_mcfg = short2Legs_mcfg1()
##    character_mcfg = short2Legs_mcfg2()
    character_mcfg = short2Legs_mcfg3()
    short2Legs_edit(motion_ori)
    c_swf_mid_offset = .04
    mu = 2.0

    # long2legs
#    character_mcfg = normal_mcfg()
#    long2Legs_edit(motion_ori)
###    long2Legs_edit(motion_ori, .3)
    
#    # short1leg
#    character_mcfg = normal_mcfg()
#    short1Leg_edit(motion_ori, .01)
##    short1Leg_edit(motion_ori, .02)
#    c_swf_mid_offset = .04
#    mu = 2.0
    #===========================================================================
    
    character_firstPosture = motion_ori[0]

    print motion_ori[0].getJointPositionGlobal(lFoot)
    print motion_ori[0].getJointPositionGlobal(lHip)
#    print motion_ori[0].skeleton
        
    wcfg = ypc.WorldConfig()
    wcfg.planeHeight = 0.
    wcfg.useDefaultContactModel = False
    wcfg.lockingVel = c_locking_vel
    stepsPerFrame = 30
    wcfg.timeStep = (frameTime)/stepsPerFrame
    
    vpWorld = cvw.VpWorld(wcfg)
    motionModel = cvm.VpMotionModel(vpWorld, motion_firstPosture, motion_mcfg)
    controlModel = cvm.VpControlModel(vpWorld, character_firstPosture, character_mcfg)
    vpWorld.initialize()
    print controlModel
    
    controlModel.initializeHybridDynamics()
    
    #===============================================================================
    # load segment info
    #===============================================================================
    skeleton = motion_ori[0].skeleton
    
    segname = os.path.splitext(filename)[0]+'.seg'
    segfile = open(dir+segname, 'r')
    seginfo = cPickle.load(segfile)
    segfile.close()
    
    intervals = [info['interval'] for info in seginfo]
    states = [info['state'] for info in seginfo]
    temp_motion = copy.deepcopy(motion_ori)
    segments = yma.splitMotionIntoSegments(temp_motion, intervals)    
    print len(intervals), 'segments'
    for i in range(len(intervals)):
        print '%dth'%i, yba.GaitState.text[states[i]], intervals[i], ',', 
    print

    motion_seg_orig = ym.JointMotion()
    motion_seg_orig += segments[0]
    motion_seg = ym.JointMotion()
    motion_seg += segments[0]
    motion_stitch = ym.JointMotion()
    motion_stitch += segments[0]

    motion_stf_stabilize = ym.JointMotion()
    motion_match_stl = ym.JointMotion()
    motion_swf_placement = ym.JointMotion()
    motion_swf_height = ym.JointMotion()
    motion_swf_orientation = ym.JointMotion()
    motion_stf_balancing = ym.JointMotion()
    motion_stf_push = ym.JointMotion()
    motion_control = ym.JointMotion()

    motion_debug1 = ym.JointMotion()
    motion_debug2 = ym.JointMotion()
    motion_debug3 = ym.JointMotion()
    
    P = ym.JointMotion()
    P_hat = ym.JointMotion()
    M_tc = ym.JointMotion()
    M_hat_tc_1 = ym.JointMotion()
    
    #===============================================================================
    # loop variable
    #===============================================================================
    seg_index = [0]
    acc_offset = [0]
    extended = [False]
    prev_R_swp = [None]
    stl_y_limit_num = [0]
    stl_xz_limit_num = [0]
    avg_dCM = [mm.O_Vec3()]
#    avg_stf_v = [mm.O_Vec3()]
#    avg_stf_av = [mm.O_Vec3()]
    
#    stf_push_func = [yfg.zero]
    step_length_cur = [0.]

    step_length_tar = [0.]
    step_axis = [mm.O_Vec3()]
            
    #===============================================================================
    # information
    #===============================================================================
    bodyIDsToCheck = range(vpWorld.getBodyNum())
    mus = [mu]*len(bodyIDsToCheck)
    
    bodyMasses = controlModel.getBodyMasses()
    totalMass = controlModel.getTotalMass()

    lID = controlModel.name2id('LeftFoot');      rID = controlModel.name2id('RightFoot')
    lUpLeg = skeleton.getJointIndex('LeftUpLeg');rUpLeg = skeleton.getJointIndex('RightUpLeg')
    lKnee = skeleton.getJointIndex('LeftLeg');   rKnee = skeleton.getJointIndex('RightLeg')
    lFoot = skeleton.getJointIndex('LeftFoot');  rFoot = skeleton.getJointIndex('RightFoot')
    spine = skeleton.getJointIndex('Spine')
    
    uppers = [skeleton.getJointIndex(name) for name in ['Hips', 'Spine', 'Spine1', 'LeftArm', 'LeftForeArm', 'RightArm', 'RightForeArm']]
    upperMass = sum([bodyMasses[i] for i in uppers])
    lLegs = [skeleton.getJointIndex(name) for name in ['LeftUpLeg', 'LeftLeg', 'LeftFoot']]
    rLegs = [skeleton.getJointIndex(name) for name in ['RightUpLeg', 'RightLeg', 'RightFoot']]
    allJoints = set(range(skeleton.getJointNum()))
    
    halfFootHeight = controlModel.getBodyShape(lFoot)[1] / 2.
    
    for fi in forceInfos:
        fi.targetBody = spine
        
    #===============================================================================
    # rendering
    #===============================================================================
    rd_CM = [None]; rd_CP = [None]; rd_CMP = [None]
    rd_forces = [None]; rd_force_points = [None]
    rd_torques = []; rd_joint_positions = []
    
    rd_point1 = [None]
    rd_point2 = [None]
    rd_vec1 = [None];   rd_vecori1 = [None] 
    rd_vec2 = [None];   rd_vecori2 = [None]
    rd_frame1 = [None]
    rd_frame2 = [None]
    
    if MULTI_VIEWER:
        viewer = ymv.MultiViewer(800, 655)
#        viewer = ymv.MultiViewer(1600, 1255)
        viewer.setRenderers1([cvr.VpModelRenderer(motionModel, CHARACTER_COLOR, yr.POLYGON_FILL)])
        viewer.setRenderers2([cvr.VpModelRenderer(controlModel, CHARACTER_COLOR, yr.POLYGON_FILL)])
#        viewer.glWindow1.groundOffset = [-15, 0, 0]
#        viewer.glWindow2.groundOffset = [-15, 0, 0]
    else:
        viewer = ysv.SimpleViewer()
    #    viewer.record(False)
    
        viewer.doc.addRenderer('motionModel', cvr.VpModelRenderer(motionModel, (0,150,255), yr.POLYGON_LINE))
        viewer.doc.addRenderer('controlModel', cvr.VpModelRenderer(controlModel, (200,200,200), yr.POLYGON_LINE))
        
    #    viewer.doc.addObject('motion_ori', motion_ori)
    #    viewer.doc.addRenderer('motion_ori', yr.JointMotionRenderer(motion_ori, (0,100,255), yr.LINK_BONE))
#        viewer.doc.addRenderer('motion_seg_orig', yr.JointMotionRenderer(motion_seg_orig, (0,100,255), yr.LINK_BONE))
#        viewer.doc.addRenderer('motion_seg', yr.JointMotionRenderer(motion_seg, (0,150,255), yr.LINK_BONE))
#        viewer.doc.addRenderer('motion_stitch', yr.JointMotionRenderer(motion_stitch, (0,255,200), yr.LINK_BONE))
#        viewer.doc.addRenderer('motion_stf_stabilize', yr.JointMotionRenderer(motion_stf_stabilize, (255,0,0), yr.LINK_BONE))
#        viewer.doc.addRenderer('motion_match_stl', yr.JointMotionRenderer(motion_match_stl, (255,200,0), yr.LINK_BONE))
#        viewer.doc.addRenderer('motion_swf_placement', yr.JointMotionRenderer(motion_swf_placement, (255,100,255), yr.LINK_BONE))
#        viewer.doc.addRenderer('motion_swf_height', yr.JointMotionRenderer(motion_swf_height, (50,255,255), yr.LINK_BONE))
        viewer.doc.addRenderer('motion_swf_orientation', yr.JointMotionRenderer(motion_swf_orientation, (255,100,0), yr.LINK_BONE))
        viewer.doc.addRenderer('motion_stf_push', yr.JointMotionRenderer(motion_stf_push, (50,255,200), yr.LINK_BONE))
    #    viewer.doc.addRenderer('motion_stf_balancing', yr.JointMotionRenderer(motion_stf_balancing, (255,100,255), yr.LINK_BONE))
#        viewer.doc.addRenderer('motion_control', yr.JointMotionRenderer(motion_control, (255,0,0), yr.LINK_BONE))
    
#        viewer.doc.addRenderer('motion_debug1', yr.JointMotionRenderer(motion_debug1, (0,255,0), yr.LINK_BONE))
#        viewer.doc.addRenderer('motion_debug2', yr.JointMotionRenderer(motion_debug2, (255,0,255), yr.LINK_BONE))
#        viewer.doc.addRenderer('motion_debug3', yr.JointMotionRenderer(motion_debug3, (255,255,0), yr.LINK_BONE))

#        viewer.doc.addRenderer('M_tc', yr.JointMotionRenderer(M_tc, (255,255,0), yr.LINK_BONE))
#        viewer.doc.addRenderer('P_hat', yr.JointMotionRenderer(P_hat, (255,255,0), yr.LINK_BONE))
#        viewer.doc.addRenderer('P', yr.JointMotionRenderer(P, (255,255,0), yr.LINK_BONE))
#        viewer.doc.addRenderer('M_hat_tc_1', yr.JointMotionRenderer(M_hat_tc_1, (255,255,0), yr.LINK_BONE))
    
    #    viewer.doc.addRenderer('rd_CM', yr.PointsRenderer(rd_CM, (255,255,0)))
    #    viewer.doc.addRenderer('rd_CP', yr.PointsRenderer(rd_CP, (255,0,0)))
    #    viewer.doc.addRenderer('rd_CMP', yr.PointsRenderer(rd_CMP, (0,255,0)))
    #    viewer.doc.addRenderer('forces', yr.ForcesRenderer(rd_forces, rd_force_points, (255,0,0), ratio=.01, fromPoint=False))
#        viewer.doc.addRenderer('torques', yr.VectorsRenderer(rd_torques, rd_joint_positions, (255,0,0)))
    
    #    viewer.doc.addRenderer('rd_point1', yr.PointsRenderer(rd_point1, (0,255,0)))
    #    viewer.doc.addRenderer('rd_point2', yr.PointsRenderer(rd_point2, (255,0,0)))
#        viewer.doc.addRenderer('rd_vec1', yr.VectorsRenderer(rd_vec1, rd_vecori1, (255,0,0)))
    #    viewer.doc.addRenderer('rd_vec2', yr.VectorsRenderer(rd_vec2, rd_vecori2, (0,255,0)))
    #    viewer.doc.addRenderer('rd_frame1', yr.FramesRenderer(rd_frame1, (0,200,200)))
    #    viewer.doc.addRenderer('rd_frame2', yr.FramesRenderer(rd_frame2, (200,200,0)))
    #    viewer.setMaxFrame(len(motion_ori)-1)

    if not REPEATED:
        viewer.setMaxFrame(len(motion_ori)-1)
    else:
        viewer.setMaxFrame(300)
        
    if CAMERA_TRACKING:
        if MULTI_VIEWER:
            cameraTargets1 = [None] * (viewer.getMaxFrame()+1)
            cameraTargets2 = [None] * (viewer.getMaxFrame()+1)
        else:
            cameraTargets = [None] * (viewer.getMaxFrame()+1)
            
    pt = [0.]
    def postFrameCallback_Always(frame):
        if frame==1: pt[0] = time.time()
        if frame==31: print 'elapsed time for 30 frames:', time.time()-pt[0]
        if CAMERA_TRACKING:
            if MULTI_VIEWER:
                if cameraTargets1[frame]==None:
                    cameraTargets1[frame] = motionModel.getBodyPositionGlobal(0)
#                    cameraTargets1[frame] = motion_ori[frame].getJointPositionGlobal(0)
                viewer.setCameraTarget1(cameraTargets1[frame])
                
                if cameraTargets2[frame]==None:
                    cameraTargets2[frame] = controlModel.getJointPositionGlobal(0)
                viewer.setCameraTarget2(cameraTargets2[frame])
                
            else:
                if cameraTargets[frame]==None:
                    cameraTargets[frame] = controlModel.getJointPositionGlobal(0)
                viewer.setCameraTarget(cameraTargets[frame])
        if plot!=None:
            plot.updateVline(frame)
    viewer.setPostFrameCallback_Always(postFrameCallback_Always)

    plot = None
#    plot = ymp.InteractivePlot()
    if plot!=None:
        plot.setXlimit(0, len(motion_ori))
        plot.setYlimit(0., 1.)
        plot.addDataSet('zero')
        plot.addDataSet('diff')
        plot.addDataSet('debug1')
        plot.addDataSet('debug2')

    def viewer_onClose(data):
        if plot!=None:
            plot.close()
        viewer.onClose(data)
    viewer.callback(viewer_onClose)
         
    def simulateCallback(frame):
        # seginfo
        segIndex = seg_index[0]
        curState = seginfo[segIndex]['state']
        curInterval = yma.offsetInterval(acc_offset[0], seginfo[segIndex]['interval'])
        stanceLegs = seginfo[segIndex]['stanceHips']
        swingLegs = seginfo[segIndex]['swingHips']
        stanceFoots = seginfo[segIndex]['stanceFoots']
        swingFoots = seginfo[segIndex]['swingFoots']
        swingKnees = seginfo[segIndex]['swingKnees']
        groundHeight = seginfo[segIndex]['ground_height']
        maxStfPushFrame = seginfo[segIndex]['max_stf_push_frame']
        
        prev_frame = frame-1 if frame>0 else 0
#        prev_frame = frame
        
        # information
#        dCM_tar = yrp.getCM(motion_seg.getJointVelocitiesGlobal(frame), bodyMasses, upperMass, uppers)
#        CM_tar = yrp.getCM(motion_seg.getJointPositionsGlobal(frame), bodyMasses, upperMass, uppers)
##        dCM_tar = yrp.getCM(motion_seg.getJointVelocitiesGlobal(frame), bodyMasses, totalMass)
##        CM_tar = yrp.getCM(motion_seg.getJointPositionsGlobal(frame), bodyMasses, totalMass)
#        stf_tar = motion_seg.getJointPositionGlobal(stanceFoots[0], frame)
#        CMr_tar = CM_tar - stf_tar

        dCM_tar = motion_seg.getJointVelocityGlobal(0, prev_frame)
        CM_tar = motion_seg.getJointPositionGlobal(0, prev_frame)
#        dCM_tar = yrp.getCM(motion_seg.getJointVelocitiesGlobal(prev_frame), bodyMasses, upperMass, uppers)
#        CM_tar = yrp.getCM(motion_seg.getJointPositionsGlobal(prev_frame), bodyMasses, upperMass, uppers)
#        dCM_tar = yrp.getCM(motion_seg.getJointVelocitiesGlobal(prev_frame), bodyMasses, totalMass)
#        CM_tar = yrp.getCM(motion_seg.getJointPositionsGlobal(prev_frame), bodyMasses, totalMass)
        stf_tar = motion_seg.getJointPositionGlobal(stanceFoots[0], prev_frame)
        CMr_tar = CM_tar - stf_tar
            
        dCM = avg_dCM[0]
        CM = controlModel.getJointPositionGlobal(0)
#        CM = yrp.getCM(controlModel.getJointPositionsGlobal(), bodyMasses, upperMass, uppers)
#        CM = yrp.getCM(controlModel.getJointPositionsGlobal(), bodyMasses, totalMass)
        CMreal = yrp.getCM(controlModel.getJointPositionsGlobal(), bodyMasses, totalMass)
        stf = controlModel.getJointPositionGlobal(stanceFoots[0])
        CMr = CM - stf
        
        diff_dCM = mm.projectionOnPlane(dCM-dCM_tar, (1,0,0), (0,0,1))
        diff_dCM_axis = np.cross((0,1,0), diff_dCM)
        rd_vec1[0] = diff_dCM; rd_vecori1[0] = CM_tar
        
        diff_CMr = mm.projectionOnPlane(CMr-CMr_tar, (1,0,0), (0,0,1))
#        rd_vec1[0] = diff_CMr; rd_vecori1[0] = stf_tar
        diff_CMr_axis = np.cross((0,1,0), diff_CMr)
        
        direction = mm.normalize2(mm.projectionOnPlane(dCM_tar, (1,0,0), (0,0,1)))
#        direction = mm.normalize2(mm.projectionOnPlane(dCM, (1,0,0), (0,0,1)))
        directionAxis = np.cross((0,1,0), direction)
        
        diff_dCM_sag, diff_dCM_cor = mm.projectionOnVector2(diff_dCM, direction)
#        rd_vec1[0] = diff_dCM_sag; rd_vecori1[0] = CM_tar
        diff_dCM_sag_axis = np.cross((0,1,0), diff_dCM_sag)
        diff_dCM_cor_axis = np.cross((0,1,0), diff_dCM_cor)
            
        diff_CMr_sag, diff_CMr_cor = mm.projectionOnVector2(diff_CMr, direction)
        diff_CMr_sag_axis = np.cross((0,1,0), diff_CMr_sag)
        diff_CMr_cor_axis = np.cross((0,1,0), diff_CMr_cor)
            
        t = (frame-curInterval[0])/float(curInterval[1]-curInterval[0])
        t_raw = t
        if t>1.: t=1.
        
        
        p_root = motion_stitch[frame].getJointPositionGlobal(0)
        R_root = motion_stitch[frame].getJointOrientationGlobal(0)

        motion_seg_orig.goToFrame(frame)
        motion_seg.goToFrame(frame)
        motion_stitch.goToFrame(frame)
        
        motion_debug1.append(motion_stitch[frame].copy())
        motion_debug1.goToFrame(frame)
        motion_debug2.append(motion_stitch[frame].copy())
        motion_debug2.goToFrame(frame)
        motion_debug3.append(motion_stitch[frame].copy())
        motion_debug3.goToFrame(frame)
        
        # paper implementation
        M_tc.append(motion_stitch[prev_frame])
        M_tc.goToFrame(frame)
        P_hat.append(M_tc[frame].copy())
        P_hat.goToFrame(frame)
        
        p_temp = ym.JointPosture(skeleton)
        p_temp.rootPos = controlModel.getJointPositionGlobal(0)
        p_temp.setJointOrientationsLocal(controlModel.getJointOrientationsLocal())
        P.append(p_temp)
        P.goToFrame(frame)
        
        # stance foot stabilize
        motion_stf_stabilize.append(motion_stitch[frame].copy())
        motion_stf_stabilize.goToFrame(frame)
        if STANCE_FOOT_STABILIZE:
            for stanceFoot in stanceFoots:
                R_target_foot = motion_seg[frame].getJointOrientationGlobal(stanceFoot)
                R_current_foot = motion_stf_stabilize[frame].getJointOrientationGlobal(stanceFoot)
                motion_stf_stabilize[frame].setJointOrientationGlobal(stanceFoot, cm.slerp(R_current_foot, R_target_foot , stf_stabilize_func(t)))
#                R_target_foot = motion_seg[frame].getJointOrientationLocal(stanceFoot)
#                R_current_foot = motion_stf_stabilize[frame].getJointOrientationLocal(stanceFoot)
#                motion_stf_stabilize[frame].setJointOrientationLocal(stanceFoot, cm.slerp(R_current_foot, R_target_foot , stf_stabilize_func(t)))

        # match stance leg 
        motion_match_stl.append(motion_stf_stabilize[frame].copy())
        motion_match_stl.goToFrame(frame)
        if MATCH_STANCE_LEG:
            if curState!=yba.GaitState.STOP:
                for i in range(len(stanceLegs)):
                    stanceLeg = stanceLegs[i]
                    stanceFoot = stanceFoots[i]
                    
#                    # motion stance leg -> character stance leg as time goes
                    R_motion = motion_match_stl[frame].getJointOrientationGlobal(stanceLeg)
                    R_character = controlModel.getJointOrientationGlobal(stanceLeg)
                    motion_match_stl[frame].setJointOrientationGlobal(stanceLeg, cm.slerp(R_motion, R_character, match_stl_func(t)))

#                    t_y = match_stl_func_y(t)
#                    t_xz = match_stl_func(t)
#                    
#                    R_motion = motion_match_stl[frame].getJointOrientationGlobal(stanceLeg)
#                    R_character = controlModel.getJointOrientationGlobal(stanceLeg)
#                    R = np.dot(R_character, R_motion.T)
#                    R_y, R_xz = mm.projectRotation((0,1,0), R)
#                    motion_match_stl[frame].mulJointOrientationGlobal(stanceLeg, mm.scaleSO3(R_xz, t_xz))
#                    motion_match_stl[frame].mulJointOrientationGlobal(stanceLeg, mm.scaleSO3(R_y, t_y))

        # swing foot placement
        motion_swf_placement.append(motion_match_stl[frame].copy())
        motion_swf_placement.goToFrame(frame)
        if SWING_FOOT_PLACEMENT:
            t_swing_foot_placement = swf_placement_func(t);
            
            if extended[0]:
                R_swp_sag = prev_R_swp[0][0]
                R_swp_cor = prev_R_swp[0][1]
            else:
                R_swp_sag = mm.I_SO3(); R_swp_cor = mm.I_SO3()
                R_swp_sag = np.dot(R_swp_sag, mm.exp(diff_dCM_sag_axis * K_swp_vel_sag * -t_swing_foot_placement))
                R_swp_cor = np.dot(R_swp_cor, mm.exp(diff_dCM_cor_axis * K_swp_vel_cor * -t_swing_foot_placement))
                if np.dot(direction, diff_CMr_sag) < 0:
                    R_swp_sag = np.dot(R_swp_sag, mm.exp(diff_CMr_sag_axis * K_swp_pos_sag * -t_swing_foot_placement))
                else:
                    R_swp_sag = np.dot(R_swp_sag, mm.exp(diff_CMr_sag_axis * K_swp_pos_sag_ext * -t_swing_foot_placement))
                R_swp_cor = np.dot(R_swp_cor, mm.exp(diff_CMr_cor_axis * K_swp_pos_cor * -t_swing_foot_placement))

            for i in range(len(swingLegs)):
                swingLeg = swingLegs[i]
                swingFoot = swingFoots[i] 
                
                # save swing foot global orientation
#                R_swf = motion_swf_placement[frame].getJointOrientationGlobal(swingFoot)
                
                # rotate swing leg
                motion_swf_placement[frame].mulJointOrientationGlobal(swingLeg, R_swp_sag)
                motion_swf_placement[frame].mulJointOrientationGlobal(swingLeg, R_swp_cor)
                
                # restore swing foot global orientation
#                motion_swf_placement[frame].setJointOrientationGlobal(swingFoot, R_swf)
                
                prev_R_swp[0] = (R_swp_sag, R_swp_cor)

        # swing foot height
        motion_swf_height.append(motion_swf_placement[frame].copy())
        motion_swf_height.goToFrame(frame)
        if SWING_FOOT_HEIGHT:
            for swingFoot in swingFoots:
                stanceFoot = stanceFoots[0]

                # save foot global orientation
                R_foot = motion_swf_height[frame].getJointOrientationGlobal(swingFoot)
                R_stance_foot = motion_swf_height[frame].getJointOrientationGlobal(stanceFoot)

                height_tar = motion_swf_height[frame].getJointPositionGlobal(swingFoot)[1] - motion_swf_height[frame].getJointPositionGlobal(stanceFoot)[1]
#                motion_debug1[frame] = motion_swf_height[frame].copy()

                # rotate
                motion_swf_height[frame].rotateByTarget(controlModel.getJointOrientationGlobal(0))
#                motion_debug2[frame] = motion_swf_height[frame].copy()
#                motion_debug2[frame].translateByTarget(controlModel.getJointPositionGlobal(0))

                height_cur = motion_swf_height[frame].getJointPositionGlobal(swingFoot)[1] - motion_swf_height[frame].getJointPositionGlobal(stanceFoot)[1]

                offset_height = (height_tar - height_cur) * swf_height_func(t) * c_swf_stability
                offset_sine = c_swf_mid_offset * swf_height_sine_func(t)
                
                offset = 0.
                offset += offset_height
                offset += offset_sine

                if offset > 0.:
                    newPosition =  motion_swf_height[frame].getJointPositionGlobal(swingFoot)
                    newPosition[1] += offset
                    aik.ik_analytic(motion_swf_height[frame], swingFoot, newPosition)
                else:
                    newPosition =  motion_swf_height[frame].getJointPositionGlobal(stanceFoot)
                    newPosition[1] -= offset
                    aik.ik_analytic(motion_swf_height[frame], stanceFoot, newPosition)

                # return
#                motion_debug3[frame] = motion_swf_height[frame].copy()
#                motion_debug3[frame].translateByTarget(controlModel.getJointPositionGlobal(0))
                motion_swf_height[frame].rotateByTarget(R_root)
                
                # restore foot global orientation
                motion_swf_height[frame].setJointOrientationGlobal(swingFoot, R_foot)
                motion_swf_height[frame].setJointOrientationGlobal(stanceFoot, R_stance_foot)

                if plot!=None:
                    plot.addDataPoint('debug1', frame, height_tar)
                    plot.addDataPoint('debug2', frame, height_cur)
#                    plot.addDataPoint('diff', frame, diff)

        # swing foot orientation
        motion_swf_orientation.append(motion_swf_height[frame].copy())
        motion_swf_orientation.goToFrame(frame)
        if SWING_FOOT_ORIENTATION:
#    swf_orientation_func = yfg.concatenate([yfg.zero, yfg.hermite2nd, yfg.one], [.25, .75])
            for swingFoot in swingFoots:
                R_target_foot = motion_seg[curInterval[1]].getJointOrientationGlobal(swingFoot)
                R_current_foot = motion_swf_orientation[frame].getJointOrientationGlobal(swingFoot)
                motion_swf_orientation[frame].setJointOrientationGlobal(swingFoot, cm.slerp(R_current_foot, R_target_foot, swf_orientation_func(t)))
#    swf_stabilize_func = yfg.concatenate([yfg.hermite2nd, yfg.one], [c_taking_duration])
            # push orientation
#            for swingFoot in swingFoots:
#                R_target_foot = motion_seg[frame].getJointOrientationGlobal(swingFoot)
#                R_current_foot = motion_swf_orientation[frame].getJointOrientationGlobal(swingFoot)
#                motion_swf_orientation[frame].setJointOrientationGlobal(swingFoot, cm.slerp(R_current_foot, R_target_foot , swf_stabilize_func(t)))
            
        # stance foot push                
        motion_stf_push.append(motion_swf_orientation[frame].copy())
        motion_stf_push.goToFrame(frame)
        if STANCE_FOOT_PUSH:
            for swingFoot in swingFoots:
#                max_t = (maxStfPushFrame)/float(curInterval[1]-curInterval[0])
#                stf_push_func = yfg.concatenate([yfg.sine, yfg.zero], [max_t*2])
                stf_push_func = yfg.concatenate([yfg.sine, yfg.zero], [c_taking_duration*2])
                
                R_swp_sag = mm.I_SO3()
#                R_swp_sag = np.dot(R_swp_sag, mm.exp(diff_dCM_sag_axis * K_stp_vel * -stf_push_func(t)))
                
#                if step_length_cur[0] < step_length_tar[0]:
#                    ratio = step_length_cur[0] / step_length_tar[0]
#                    R_max = maxmaxStfPushFrame
#                    R_zero = 
                R_swp_sag = np.dot(R_swp_sag, mm.exp((step_length_tar[0] - step_length_cur[0])*step_axis[0] * K_stp_pos * -stf_push_func(t)))
                    
                motion_stf_push[frame].mulJointOrientationGlobal(swingFoot, R_swp_sag)
                
        # stance foot balancing 
        motion_stf_balancing.append(motion_stf_push[frame].copy())
        motion_stf_balancing.goToFrame(frame)
        if STANCE_FOOT_BALANCING:
            R_stb = mm.exp(diff_dCM_axis * K_stb_vel * stf_balancing_func(t))
            for stanceFoot in stanceFoots:
                if frame < 5: continue
                motion_stf_balancing[frame].mulJointOrientationGlobal(stanceFoot, R_stb)
                    
        # control trajectory
        motion_control.append(motion_stf_balancing[frame].copy())
        motion_control.goToFrame(frame)
        
        #=======================================================================
        # tracking with inverse dynamics
        #=======================================================================
        th_r = motion_control.getDOFPositions(frame)
        th = controlModel.getDOFPositions()
        dth_r = motion_control.getDOFVelocities(frame)
        dth = controlModel.getDOFVelocities()
        ddth_r = motion_control.getDOFAccelerations(frame)
        ddth_des = yct.getDesiredDOFAccelerations(th_r, th, dth_r, dth, ddth_r, Kt, Dt)

        #=======================================================================
        # simulation
        #=======================================================================
        CP = mm.v3(0.,0.,0.)
        F = mm.v3(0.,0.,0.)
        avg_dCM[0] = mm.v3(0.,0.,0.)
        
        # external force rendering info
        del rd_forces[:]; del rd_force_points[:]
        for fi in forceInfos:
            if fi.startFrame <= frame and frame < fi.startFrame + fi.duration*(1/frameTime):
                rd_forces.append(fi.force)
                rd_force_points.append(controlModel.getBodyPositionGlobal(fi.targetBody))
                    
        for i in range(stepsPerFrame):
            
            bodyIDs, contactPositions, contactPositionLocals, contactForces = vpWorld.calcPenaltyForce(bodyIDsToCheck, mus, Ks, Ds)
            vpWorld.applyPenaltyForce(bodyIDs, contactPositionLocals, contactForces)
            
            # apply external force
            for fi in forceInfos:
                if fi.startFrame <= frame and frame < fi.startFrame + fi.duration*(1/frameTime):
                    controlModel.applyBodyForceGlobal(fi.targetBody, fi.force)
            
            controlModel.setDOFAccelerations(ddth_des)
            controlModel.solveHybridDynamics()
            
            vpWorld.step()
#            yvu.align2D(controlModel)

            if len(contactForces) > 0:
                CP += yrp.getCP(contactPositions, contactForces)
                F += sum(contactForces)
            avg_dCM[0] += controlModel.getJointVelocityGlobal(0)
#            avg_dCM[0] += yrp.getCM(controlModel.getJointVelocitiesGlobal(), bodyMasses, upperMass, uppers)
#            avg_dCM[0] += yrp.getCM(controlModel.getJointVelocitiesGlobal(), bodyMasses, totalMass)

#            if len(stanceFoots)>0:
#                avg_stf_v[0] += controlModel.getJointVelocityGlobal(stanceFoots[0])
#                avg_stf_av[0] += controlModel.getJointAngVelocityGlobal(stanceFoots[0])
        
        CP /= stepsPerFrame
        F /= stepsPerFrame
        avg_dCM[0] /= stepsPerFrame
        
#        if len(stanceFoots)>0:
#            avg_stf_v[0] /= stepsPerFrame
#            avg_stf_av[0] /= stepsPerFrame
#            rd_vec1[0] = avg_stf_av[0]; rd_vec1[0][0] = 0.; rd_vec1[0][2] = 0.
#            rd_vecori1[0]= controlModel.getJointPositionGlobal(stanceFoots[0])

        #=======================================================================
        # segment editing
        #=======================================================================
        lastFrame = False
        
        if SEGMENT_EDITING:
            if curState==yba.GaitState.STOP:
                if frame == len(motion_seg)-1:
                    lastFrame = True
                    
            elif (curState==yba.GaitState.LSWING or curState==yba.GaitState.RSWING) and t>c_min_contact_time:
                swingID = lID if curState==yba.GaitState.LSWING else rID

                contact = False
                if swingID in bodyIDs:
                    minContactVel = 1000.
                    for i in range(len(bodyIDs)):
                        if bodyIDs[i]==swingID:
                            vel = controlModel.getBodyVelocityGlobal(swingID, contactPositionLocals[i])
                            vel[1] = 0
                            contactVel = mm.length(vel)
                            if contactVel < minContactVel: minContactVel = contactVel 
                    if minContactVel < c_min_contact_vel: contact = True
                
                extended[0] = False
                
                if contact:
#                    print frame, 'foot touch'
                    lastFrame = True
                    acc_offset[0] += frame - curInterval[1]
                    
                elif frame == len(motion_seg)-1:
                    print frame, 'extend frame', frame+1
                    
                    preserveJoints = []
#                    preserveJoints = [lFoot, rFoot]
#                    preserveJoints = [lFoot, rFoot, lKnee, rKnee]
#                    preserveJoints = [lFoot, rFoot, lKnee, rKnee, lUpLeg, rUpLeg]
                    stanceKnees = [rKnee] if curState==yba.GaitState.LSWING else [lKnee]   
                    preserveJoints = [stanceFoots[0], stanceKnees[0], stanceLegs[0]]
   
                    diff = 3
                    motion_seg_orig.extend([motion_seg_orig[-1]])
                    motion_seg.extend(ymt.extendByIntegration_root(motion_seg, 1, diff))
                    
                    motion_stitch.extend(ymt.extendByIntegration_constant(motion_stitch, 1, preserveJoints, diff))

#                    # extend for swing foot ground speed matching & swing foot height lower
##                    extendedPostures = ymt.extendByIntegration(motion_stitch, 1, preserveJoints, diff)
##                    extendedPostures = [motion_stitch[-1]] 
##
#                    extendFrameNum = frame - curInterval[1] + 1
#                    k = 1.-extendFrameNum/5.
#                    if k<0.: k=0.
#                    extendedPostures = ymt.extendByIntegrationAttenuation(motion_stitch, 1, preserveJoints, diff, k)
#
##                    if len(swingFoots)>0 and np.inner(dCM_tar, dCM)>0.:
##                        print frame, 'speed matching'
##                        R_swf = motion_stitch[-1].getJointOrientationGlobal(swingFoots[0])
##                        
##                        p_swf = motion_stitch[-1].getJointPositionGlobal(swingFoots[0])
##                        v_swf = motion_stitch.getJointVelocityGlobal(swingFoots[0], frame-diff, frame)
##                        a_swf = motion_stitch.getJointAccelerationGlobal(swingFoots[0], frame-diff, frame)
##                        p_swf += v_swf * (frameTime) + a_swf * (frameTime)*(frameTime)
##                        aik.ik_analytic(extendedPostures[0], swingFoots[0], p_swf)
##                        
##                        extendedPostures[0].setJointOrientationGlobal(swingFoots[0], R_swf)
#
#                    motion_stitch.extend(extendedPostures)
                    
                    extended[0] = True
        else:
            if frame == len(motion_seg)-1: lastFrame = True
                    
        if lastFrame:
            if segIndex < len(segments)-1:
                print '%d (%d): end of %dth seg (%s, %s)'%(frame, frame-curInterval[1],segIndex, yba.GaitState.text[curState], curInterval)
                if plot!=None: plot.addDataPoint('diff', frame, (frame-curInterval[1])*.01)
                
                if len(stanceFoots)>0 and len(swingFoots)>0:
#                    step_cur = controlModel.getJointPositionGlobal(swingFoots[0]) - controlModel.getJointPositionGlobal(stanceFoots[0])
#                    step_tar = motion_seg[curInterval[1]].getJointPositionGlobal(swingFoots[0]) - motion_seg[curInterval[1]].getJointPositionGlobal(stanceFoots[0])
                    step_cur = controlModel.getJointPositionGlobal(0) - controlModel.getJointPositionGlobal(stanceFoots[0])
                    step_tar = motion_seg[curInterval[1]].getJointPositionGlobal(0) - motion_seg[curInterval[1]].getJointPositionGlobal(stanceFoots[0])
                    
                    step_cur = mm.projectionOnPlane(step_cur, (1,0,0), (0,0,1))
                    step_tar = mm.projectionOnPlane(step_tar, (1,0,0), (0,0,1))
                    
                    step_cur_sag, step_cur_cor = mm.projectionOnVector2(step_cur, direction)
                    step_tar_sag, step_tar_cor = mm.projectionOnVector2(step_tar, direction)
                    
                    step_length_tar[0] = mm.length(step_tar_sag)
                    if np.inner(step_tar_sag, step_cur_sag) > 0:
                        step_length_cur[0] = mm.length(step_cur_sag)
                    else:
                        step_length_cur[0] = -mm.length(step_cur_sag)
                    
                    step_axis[0] = directionAxis
                    
#                    rd_vec1[0] = step_tar_sag
#                    rd_vecori1[0] = motion_seg[curInterval[1]].getJointPositionGlobal(stanceFoots[0])
#                    rd_vec2[0] = step_cur_sag
#                    rd_vecori2[0] = controlModel.getJointPositionGlobal(stanceFoots[0])

                seg_index[0] += 1
                curSeg = segments[seg_index[0]]
                stl_y_limit_num[0] = 0
                stl_xz_limit_num[0] = 0
                
                del motion_seg_orig[frame+1:]
                motion_seg_orig.extend(ymb.getAttachedNextMotion(curSeg, motion_seg_orig[-1], False, False))
                
                del motion_seg[frame+1:]
                del motion_stitch[frame+1:]
                transitionLength = len(curSeg)-1

#                motion_seg.extend(ymb.getAttachedNextMotion(curSeg, motion_seg[-1], False, False))
#                motion_stitch.extend(ymb.getStitchedNextMotion(curSeg, motion_control[-1], transitionLength, stitch_func, True, False))

                d = motion_seg[-1] - curSeg[0]
                d.rootPos[1] = 0.
                motion_seg.extend(ymb.getAttachedNextMotion(curSeg, d, True, False))
                d = motion_control[-1] - curSeg[0]
                d.rootPos[1] = 0.
                motion_stitch.extend(ymb.getStitchedNextMotion(curSeg, d, transitionLength, stitch_func, True, False))
                
#                motion_seg.extend(ymb.getAttachedNextMotion(curSeg, motion_seg[-1], False, True))
#                motion_stitch.extend(ymb.getStitchedNextMotion(curSeg, motion_control[-1], transitionLength, stitch_func, True, True))
            else:
                motion_seg_orig.append(motion_seg_orig[-1])
                motion_seg.append(motion_seg[-1])
                motion_stitch.append(motion_control[-1])
                
                
        # rendering
        motionModel.update(motion_saved[frame])
            
        
    viewer.setSimulateCallback(simulateCallback)
    
    viewer.startTimer(frameTime / 1.4)
#    viewer.startTimer(frameTime * .4)
    viewer.show()
    
    Fl.run()
    
def calc_data():
    # short 1 leg .01
    short_leg = mm.length(np.array([ 0.      , -0.434227,  0.013727])) + mm.length(np.array([-0.00253549, -0.410924  , -0.0146059 ]))
    normal_leg = mm.length(np.array([ 0.      , -0.444227,  0.013727])) + mm.length(np.array([ 0.00253549, -0.420924  , -0.0146059 ]))
    print 1. - short_leg / normal_leg
    
    # short 1 leg .02
    short_leg = mm.length(np.array([ 0.      , -0.424227,  0.013727])) + mm.length(np.array([-0.00253549, -0.400924  , -0.0146059 ]))
    normal_leg = mm.length(np.array([ 0.      , -0.444227,  0.013727])) + mm.length(np.array([-0.00253549, -0.420924  , -0.0146059 ]))
    print 1. - short_leg / normal_leg
    
    
pass
#different_mass_distribution()
different_skeleton_length()
#calc_data()