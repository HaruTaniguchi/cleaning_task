#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rospy
import roslib
import actionlib
import smach
import smach_ros

from std_msgs.msg import String, Float64
from base_control import BaseControl
from happymimi_navigation.srv import NaviLocation
from enter_room.srv import EnterRoom
from happymimi_voice_msgs.srv import TTS, YesNo, ActionPlan
from happymimi_manipulation_msgs.srv import RecognitionToGrasping, RecognitionToGraspingRequest
from actplan_executor.msg import APExecutorAction, APExecutorGoal

tts_srv = rospy.ServiceProxy('/tts', StrTrg)


class Enter(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes = ['enter_finish'])

        self.enter_srv = rospy.ServiceProxy('/enter_room_server',EnterRoom)

    def execute(self,userdate):
        rospy.loginfo("Executing state: ENTER")
        tts_srv("Start GPSR")
        self.enter_srv(0.8,0.2)
        return 'enter_finish'


# class ListenCommand(smach.state):
#     def __init__(self):
#         smach.State.__init__(self,outcoms=[''])
        



# class YesNo(smach.State):
#     def __init__(self):
#         smach.State.__init__(self,outcoms = ['yesno_success'])

#         self.navi_srv = rospy.ServiceProxy('/navi_location_server', NaviLocation)

#         self.arm_srv = rospy.ServiceProxy('/servo/arm', StrTrg)

#         self.base_control = BaseControl()

#     def execute(self,userdata):
#         rospy.loginfo('Executing state:GRASP')
#         # if userdata.GOP_count_in == 0:
#         #     self.base_control.rotateAngle(userdata.find_angle_in)
#         #     rospy.sleep(0.5)
#         #     self.base_control.translateDist(0.8)
#         #     result = self.arm_srv('receive')
#         #     tts_srv('/cml/pass_thank')
#         #     self.arm_srv('carry')
#         #     return 'YesNo_finish'

#         # else:

class YesNo(smach.State):   #航のコピーから少し改変,まだ未完(ゴミ箱への移動やreturnなど)
    def __init__(self):
        smach.State.__init__(self,
                            outcomes = ['yes_no_success',
                                        'yes_no_retry',
                                        'yes_no_failed'])
        self.yesno_srv = rospy.ServiceProxy('/yes_no', YesNo)
        self.grasp_srv = rospy.ServiceProxy('/recognition_to_grasping', RecognitionToGrasping)

    def execute(self):
        answer = self.yesno_srv().result
        if answer:
            self.grasp_result = self.grasp_srv(RecognitionToGraspingRequest(target_name='bottle')).result   #bottle
            if self.grasp_result == False:
                if grasp_count >= 3: #####
                    self.tts_srv("/fd/grasp_failed")
                    #break
                    return 'yes_no_failed'
                    
                else:
                    self.tts_srv("/fd/grasp_retry")
                    grasp_count += 1
                    return 'yes_no_retry'
                
            else:
                self.tts_srv('/fd/grasp_success')
                #break
                return 'yes_no_success'

        else:
            self.grasp_result = self.grasp_srv(RecognitionToGraspingRequest(target_name='cup')).result  #cup
            if self.grasp_result == False:
                if grasp_count >= 3: #####
                    self.tts_srv("/fd/grasp_failed")
                    #break
                    return 'yes_no_failed'
                    
                else:
                    self.tts_srv("/fd/grasp_retry")
                    grasp_count += 1
                    return 'yes_no_retry'

            else:
                self.tts_srv('/fd/grasp_success')
                #break
                return 'yes_no_success'

        


class ExeAction(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes = ['action_success',
                                         'action_failure'],
                             input_keys = ['cmd_in_action',
                                           'cmd_in_data'])

    def execute(self, userdata):
        rospy.loginfo("Executing state: EXE_ACTION")
        ac = actionlib.SimpleActionClient('actplan_executor', APExecutorAction)
        ac.wait_for_server()
        # Goalの生成
        goal = APExecutorGoal()
        goal.action = userdata.cmd_in_action
        goal.data = userdata.cmd_in_data
        print goal
        ac.send_goal(goal)
        ac.wait_for_result()
        result = ac.get_result().data
        if result == 'success':
            rospy.loginfo("Success ExeActionPlan")
            return 'action_success'
        else:
            rospy.loginfo("Failed ExeActionPlan")
            return 'action_failure'





if __name__=='__main__':
    rospy.init_node('cleaningtask_master')
    rospy.loginfo("Start")
    sm_top = smach.StateMachine('finish_sm_top')
    sm_top.userdata.grasp_count = 0
    # sm_top.userdata.GOP_count = 0
    # sm_top.userdata.find_angle = 20
    with sm_top:
        smach.StateMachine.add(
                'ENTER',
                Enter(),
                transitions = {'enter_finish':'YESNO'})

        smach.StateMachine.add(
                'YESNO',
                YesNo(),
                transitions = {'yes_no_success':'EXE_ACTION',
                                'yes_no_retry':'YESNO',
                                'yes_no_failed':''})

        smach.StateMachine.add(
                'EXE_ACTION',
                ExeAction(),
                transitions = {'action_success':'',
                                'action_failure':''})