
#ifndef __PM_MASK_H
#define __PM_MASK_H

typedef  __int64  PmMaskType; // Windows
//typedef  __uint64_t  PmMaskType; // G++

#define PM_MASK_NULL            0x0000000000000000
#define PM_MASK_PELVIS          0x0000000000000001
#define PM_MASK_SPINE_1         0x0000000000000002
#define PM_MASK_SPINE_2         0x0000000000000004
#define PM_MASK_SPINE_3         0x0000000000000008
#define PM_MASK_SPINE_4         0x0000000000000010
#define PM_MASK_CHEST           0x0000000000000020
#define PM_MASK_NECK            0x0000000000000040
#define PM_MASK_HEAD            0x0000000000000080
#define PM_MASK_RIGHT_SHOULDER  0x0000000000000100
#define PM_MASK_LEFT_SHOULDER   0x0000000000000200
#define PM_MASK_RIGHT_COLLAR    0x0000000000000400
#define PM_MASK_LEFT_COLLAR     0x0000000000000500
#define PM_MASK_UPPER_RIGHT_ARM 0x0000000000001000
#define PM_MASK_UPPER_LEFT_ARM  0x0000000000002000
#define PM_MASK_LOWER_RIGHT_ARM 0x0000000000004000
#define PM_MASK_LOWER_LEFT_ARM  0x0000000000008000
#define PM_MASK_UPPER_RIGHT_LEG 0x0000000000010000
#define PM_MASK_UPPER_LEFT_LEG  0x0000000000020000
#define PM_MASK_LOWER_RIGHT_LEG 0x0000000000040000
#define PM_MASK_LOWER_LEFT_LEG  0x0000000000080000
#define PM_MASK_RIGHT_FOOT      0x0000000000100000
#define PM_MASK_LEFT_FOOT       0x0000000000200000
#define PM_MASK_RIGHT_TOE       0x0000000000400000
#define PM_MASK_LEFT_TOE        0x0000000000800000
#define PM_MASK_RIGHT_PALM      0x0000000001000000
#define PM_MASK_LEFT_PALM       0x0000000002000000
#define PM_MASK_RIGHT_HEEL		0x0000000004000000
#define	PM_MASK_LEFT_HEEL		0x0000000008000000
#define PM_MASK_RIGHT_FINGER_11 0x0000000010000000
#define PM_MASK_RIGHT_FINGER_12 0x0000000020000000
#define PM_MASK_RIGHT_FINGER_13 0x0000000040000000
#define PM_MASK_RIGHT_FINGER_21 0x0000000080000000

#define PM_MASK_RIGHT_FINGER_22 (0x00000001<<8)
#define PM_MASK_RIGHT_FINGER_23 (0x00000002<<8)
#define PM_MASK_RIGHT_FINGER_31 (0x00000004<<8)
#define PM_MASK_RIGHT_FINGER_32 (0x00000008<<8)
#define PM_MASK_RIGHT_FINGER_33 (0x00000010<<8)
#define PM_MASK_RIGHT_FINGER_41 (0x00000020<<8)
#define PM_MASK_RIGHT_FINGER_42 (0x00000040<<8)
#define PM_MASK_RIGHT_FINGER_43 (0x00000080<<8)
#define PM_MASK_RIGHT_FINGER_51 (0x00000100<<8)
#define PM_MASK_RIGHT_FINGER_52 (0x00000200<<8)
#define PM_MASK_RIGHT_FINGER_53 (0x00000400<<8)
#define PM_MASK_LEFT_FINGER_11  (0x00000800<<8)
#define PM_MASK_LEFT_FINGER_12  (0x00001000<<8)
#define PM_MASK_LEFT_FINGER_13  (0x00002000<<8)
#define PM_MASK_LEFT_FINGER_21  (0x00004000<<8)
#define PM_MASK_LEFT_FINGER_22  (0x00008000<<8)
#define PM_MASK_LEFT_FINGER_23  (0x00010000<<8)
#define PM_MASK_LEFT_FINGER_31  (0x00020000<<8)
#define PM_MASK_LEFT_FINGER_32  (0x00040000<<8)
#define PM_MASK_LEFT_FINGER_33  (0x00080000<<8)
#define PM_MASK_LEFT_FINGER_41  (0x00100000<<8)
#define PM_MASK_LEFT_FINGER_42  (0x00200000<<8)
#define PM_MASK_LEFT_FINGER_43  (0x00400000<<8)
#define PM_MASK_LEFT_FINGER_51  (0x00800000<<8)
#define PM_MASK_LEFT_FINGER_52  (0x01000000<<8)
#define	PM_MASK_LEFT_FINGER_53	(0x02000000<<8)
#define	PM_MASK_SCALE			(0x04000000<<8)

/*
#define PM_MASK_RIGHT_FINGER_32 0x0000000100000000
#define PM_MASK_RIGHT_FINGER_33 0x0000000200000000
#define PM_MASK_RIGHT_FINGER_41 0x0000000400000000
#define PM_MASK_RIGHT_FINGER_42 0x0000000800000000
#define PM_MASK_RIGHT_FINGER_43 0x0000001000000000
#define PM_MASK_RIGHT_FINGER_51 0x0000002000000000
#define PM_MASK_RIGHT_FINGER_52 0x0000004000000000
#define PM_MASK_RIGHT_FINGER_53 0x0000008000000000
#define PM_MASK_LEFT_PALM       0x0000010000000000
#define PM_MASK_LEFT_FINGER_11  0x0000020000000000
#define PM_MASK_LEFT_FINGER_12  0x0000040000000000
#define PM_MASK_LEFT_FINGER_13  0x0000080000000000
#define PM_MASK_LEFT_FINGER_21  0x0000100000000000
#define PM_MASK_LEFT_FINGER_22  0x0000200000000000
#define PM_MASK_LEFT_FINGER_23  0x0000400000000000
#define PM_MASK_LEFT_FINGER_31  0x0000800000000000
#define PM_MASK_LEFT_FINGER_32  0x0001000000000000
#define PM_MASK_LEFT_FINGER_33  0x0002000000000000
#define PM_MASK_LEFT_FINGER_41  0x0004000000000000
#define PM_MASK_LEFT_FINGER_42  0x0008000000000000
#define PM_MASK_LEFT_FINGER_43  0x0010000000000000
#define PM_MASK_LEFT_FINGER_51  0x0020000000000000
#define PM_MASK_LEFT_FINGER_52  0x0040000000000000
#define	PM_MASK_LEFT_FINGER_53	0x0080000000000000
#define PM_MASK_RIGHT_HEEL		0x0100000000000000
#define	PM_MASK_LEFT_HEEL		0x0200000000000000
#define	PM_MASK_SCALE			0x0400000000000000
*/

#define PM_MASK_LEFT_FINGER_1	(PM_MASK_LEFT_FINGER_11 | PM_MASK_LEFT_FINGER_12 | PM_MASK_LEFT_FINGER_13)
#define PM_MASK_LEFT_FINGER_2	(PM_MASK_LEFT_FINGER_21 | PM_MASK_LEFT_FINGER_22 | PM_MASK_LEFT_FINGER_23)
#define PM_MASK_LEFT_FINGER_3	(PM_MASK_LEFT_FINGER_31 | PM_MASK_LEFT_FINGER_32 | PM_MASK_LEFT_FINGER_33)
#define PM_MASK_LEFT_FINGER_4	(PM_MASK_LEFT_FINGER_41 | PM_MASK_LEFT_FINGER_42 | PM_MASK_LEFT_FINGER_43)
#define PM_MASK_LEFT_FINGER_5	(PM_MASK_LEFT_FINGER_51 | PM_MASK_LEFT_FINGER_52 | PM_MASK_LEFT_FINGER_53)

#define PM_MASK_RIGHT_FINGER_1	(PM_MASK_RIGHT_FINGER_11 | PM_MASK_RIGHT_FINGER_12 | PM_MASK_RIGHT_FINGER_13)
#define PM_MASK_RIGHT_FINGER_2	(PM_MASK_RIGHT_FINGER_21 | PM_MASK_RIGHT_FINGER_22 | PM_MASK_RIGHT_FINGER_23)
#define PM_MASK_RIGHT_FINGER_3	(PM_MASK_RIGHT_FINGER_31 | PM_MASK_RIGHT_FINGER_32 | PM_MASK_RIGHT_FINGER_33)
#define PM_MASK_RIGHT_FINGER_4	(PM_MASK_RIGHT_FINGER_41 | PM_MASK_RIGHT_FINGER_42 | PM_MASK_RIGHT_FINGER_43)
#define PM_MASK_RIGHT_FINGER_5	(PM_MASK_RIGHT_FINGER_51 | PM_MASK_RIGHT_FINGER_52 | PM_MASK_RIGHT_FINGER_53)

#define PM_MASK_LEFT_FINGERS	(PM_MASK_LEFT_FINGER_1  | PM_MASK_LEFT_FINGER_2  | PM_MASK_LEFT_FINGER_3  | PM_MASK_LEFT_FINGER_4  | PM_MASK_LEFT_FINGER_5)
#define PM_MASK_RIGHT_FINGERS	(PM_MASK_RIGHT_FINGER_1 | PM_MASK_RIGHT_FINGER_2 | PM_MASK_RIGHT_FINGER_3 | PM_MASK_RIGHT_FINGER_4 | PM_MASK_RIGHT_FINGER_5)

#define PM_MASK_LEFT_HAND		(PM_MASK_LEFT_PALM  | PM_MASK_LEFT_FINGERS)
#define PM_MASK_RIGHT_HAND		(PM_MASK_RIGHT_PALM | PM_MASK_RIGHT_FINGERS)
#define PM_MASK_HAND            (PM_MASK_LEFT_HAND | PM_MASK_RIGHT_HAND)

#define PM_MASK_RIGHT_ARM       (PM_MASK_RIGHT_SHOULDER | PM_MASK_UPPER_RIGHT_ARM | PM_MASK_LOWER_RIGHT_ARM | PM_MASK_RIGHT_PALM)
#define PM_MASK_LEFT_ARM        (PM_MASK_LEFT_SHOULDER | PM_MASK_UPPER_LEFT_ARM  | PM_MASK_LOWER_LEFT_ARM  | PM_MASK_LEFT_PALM)
#define PM_MASK_ARM             (PM_MASK_RIGHT_ARM | PM_MASK_LEFT_ARM)
#define PM_MASK_RIGHT_LEG       (PM_MASK_UPPER_RIGHT_LEG | PM_MASK_LOWER_RIGHT_LEG | PM_MASK_RIGHT_FOOT | PM_MASK_RIGHT_TOE)
#define PM_MASK_LEFT_LEG        (PM_MASK_UPPER_LEFT_LEG  | PM_MASK_LOWER_LEFT_LEG  | PM_MASK_LEFT_FOOT  | PM_MASK_LEFT_TOE)
#define PM_MASK_LEG             (PM_MASK_RIGHT_LEG | PM_MASK_LEFT_LEG)
#define PM_MASK_LIMB			(PM_MASK_ARM | PM_MASK_LEG)

#define PM_MASK_TORSO           (PM_MASK_PELVIS | PM_MASK_SPINE_1 | PM_MASK_SPINE_2 | PM_MASK_SPINE_3 | PM_MASK_SPINE_4 | PM_MASK_CHEST)
#define PM_MASK_UPPER_HALF      (PM_MASK_TORSO | PM_MASK_NECK | PM_MASK_HEAD | PM_MASK_ARM | PM_MASK_HAND)
#define PM_MASK_LOWER_HALF      (PM_MASK_PELVIS | PM_MASK_LEG)

#define PM_MASK_IK_JOINT        (PM_MASK_TORSO | PM_MASK_NECK | PM_MASK_HEAD | PM_MASK_LIMB)
#define PM_MASK_ALL_BODY        (PM_MASK_UPPER_HALF | PM_MASK_LOWER_HALF)
#define PM_MASK_ALL_JOINT       (PM_MASK_ALL_BODY ^ PM_MASK_PELVIS)

PmMaskType  MaskBit( int );
int	GetDOF( int );
int GetDOF( PmMaskType );

#endif
