#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy		# ROS 라이브러리
import random		# 랜덤 수를 추출하기 위한 라이브러리
# from yolov8_msgs.msg import Point2D	# first_msg.msg 파일 불러오기
from yolov8_msgs.msg import Point2D
def main():
    # 퍼블리시 노드 초기화
    rospy.init_node('start_node', anonymous=False)
    
    # 퍼블리셔 변수
    pub = rospy.Publisher('first_topic', Point2D, queue_size=10)
    
    # 1초마다 반복(변수=rate)
    rate = rospy.Rate(1) #1Hz

    msg = Point2D()	# 메시지 변수 선언
    count = 1

    # 중단되거나 사용자가 강제종료(ctrl+C) 전까지 계속 실행
    while not rospy.is_shutdown():
        # 메시지 내용 담기
        ## start_time: 메시지를 생성해 publish 하는 시각
        ## msg_seq: 메시지 순서(번호)
        ## original_num: 계산의 대상이 될 수로, 1부터 100까지의 수 중 무작위로 하나를 뽑음

        msg.x = 1
        msg.y = 2

        # 터미널에 출력
        rospy.loginfo("------")
        rospy.loginfo("msg x: %d", msg.x)
        rospy.loginfo("msg y: %d", msg.y)
                
        # 메시지를 퍼블리시
        pub.publish(msg)
        
        # 정해둔 주기(hz)만큼 일시중단
        rate.sleep()

        count += 1

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass