#!/usr/bin/env python3
import rospy                #Python Bibliothek für ROS
import geometry_msgs.msg    #geometry_msgs ist nötig, weil move_base/simplegoal eine Nachricht dieses Typs erwartet
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import tf                   #tf Bibliothek um mit den Frames zu arbeiten
import math                 #für die Berechnung des Abstandes


if __name__ == '__main__':

    rospy.init_node('coordinator')              #Started die Node mit den Namen "coordinator"
    listener = tf.TransformListener()           #Erzeugt ein tf-listener Objekt
    tb1_pub = rospy.Publisher("tb1/move_base_simple/goal", PoseStamped, queue_size=10)  #publisher um die Nachricht zum Topic "tb1/move_base_simple/goal"
    #Falls mehr TBots hier einfügen                                                     #zu senden, erwartet eine Nachricht vom Typ "PoseStamped"
    tb2_pub = rospy.Publisher("tb2/move_base_simple/goal", PoseStamped, queue_size=10)
    tb3_pub = rospy.Publisher("tb3/move_base_simple/goal", PoseStamped, queue_size=10)
    tb4_pub = rospy.Publisher("tb4/move_base_simple/goal", PoseStamped, queue_size=10)
    tb5_pub = rospy.Publisher("tb5/move_base_simple/goal", PoseStamped, queue_size=10)

    while not rospy.is_shutdown():                  #Führt die Schleife aus, solange die Node nicht beendet wurde
        
        x_inp ,y_inp = (input("Enter X, Y coordinates of the destination: ")).split() #Liest user input und teilt in x und y auf
        x_goal = float(x_inp)       #Wandelt Strings in Float um
        y_goal = float(y_inp)

        xy_goal = PoseStamped()                     #definiert Var. xy_goal als PoseStamped
        xy_goal.header.seq = 0
        xy_goal.header.frame_id = 'map'             #Information zu welchen Koordinaten System die Koordinaten der Nachricht gehören
        xy_goal.header.stamp = rospy.Time.now()     #Information der aktuellen Zeit
        xy_goal.pose = Pose(Point(x_goal,y_goal,0.0) , Quaternion(0,0,0,1))     #gibt der Nachricht die gewünschte Position 


        try:
            (tb1_trans,tb1_rot) = listener.lookupTransform('map', 'tb1/base_link', rospy.Time(0)) #Fragt die Koordinate von dem Baselink-Frame des Tbots im Bezug zum Map-Frame ab
                                                                                                  #und gibt als Rückgabewert zwei Listen mit translatorischen und rotatorischen Koordinaten
            (tb2_trans,tb2_rot) = listener.lookupTransform('map', 'tb2/base_link', rospy.Time(0)) 
            #Wenn mehr TBots genutzt werden, restliche Anweisungen auskommentieren
            #(tb3_trans,tb3_rot) = listener.lookupTransform('map', 'tb3/base_link', rospy.Time(0))

            #(tb4_trans,tb4_rot) = listener.lookupTransform('map', 'tb4/base_link', rospy.Time(0))
            
            #(tb5_trans,tb5_rot) = listener.lookupTransform('map', 'tb5/base_link', rospy.Time(0))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):       #Fängt Fehler ab, falls fehler mit den Frames auftreten
            continue
            
        
        #print("X1: " , tb1_trans[0],"Y1: ", tb1_trans[1])   
        #print("X2: " , tb2_trans[0],"Y2: ", tb2_trans[1])
        tb_trans_list = [tb1_trans, tb2_trans]              #Liste der trans. Koordinaten: wenn mehr TBots, auch hier hinzufügen


        #print("X1: " , tb_trans_list[0][0],"Y1: ", tb_trans_list[0][1]) 
        #print("X2: " , tb_trans_list[1][0],"Y2: ", tb_trans_list[1][1]) 
        i = 0 #index
        distance_list = [] #deklariert leeres Array
        for tb_trans in tb_trans_list:   #for-Schleife, führt so viele Iterationen durch wie Elemente in der Liste(tb_trans_list)
                                               
            distance_list.append(math.sqrt( ( x_goal - tb_trans_list[i][0] )**2 + ( y_goal - tb_trans_list[i][1] )**2 ) ) #Füllt Liste mit den Abständen zum Ziel
            i = i + 1 

        #print(distance_list[0],distance_list[1])


        dist_min_index = distance_list.index(min(distance_list))   #Sucht aus der Liste den kleinsten Wert und gibt davon den Index
                                                                   #der Index steht für den jeweiligen TBot

        if dist_min_index == 0 :                #Falls Index 0 -> TBot 1 den Befehl geben zu der Koordinate zu Fahren
            tb1_pub.publish(xy_goal)                                   #Falls mehr TBots, hier hinzufügen
            print("TBot1 moves to X: ",x_goal," Y: ", y_goal)    
        elif dist_min_index == 1:
            tb2_pub.publish(xy_goal)
            print("TBot2 moves to X: ",x_goal," Y: ", y_goal)
        #elif dist_min_index == 2:
        #    tb3_pub.publish(xy_goal)
        #    print("TBot3 moves to X: ",x_goal," Y: ", y_goal)
        #elif dist_min_index == 3:
        #    tb4_pub.publish(xy_goal) 
        #    print("TBot4 moves to X: ",x_goal," Y: ", y_goal) 
        #elif dist_min_index == 4:
        #    tb5_pub.publish(xy_goal)  
        #    print("TBot5 moves to X: ",x_goal," Y: ", y_goal)  
        else:
            pass