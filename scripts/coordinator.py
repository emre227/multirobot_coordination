#!/usr/bin/env python3
import rospy                #Python Bibliothek für ROS
import geometry_msgs.msg    #geometry_msgs ist nötig, weil move_base/simplegoal eine Nachricht dieses Typs erwartet
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import tf                   #tf Bibliothek um mit den Frames zu arbeiten
import math                 #für die Berechnung des Abstandes


## @package multirobot_coordination
# Dokumentation für den coordinator.py
#
# Der coordinator.py fragt nach einem User-Input für die Zielkoordinate und speichert diesen dann ab.
# Danach werden die Koordinaten der TurtleBots abgefragt und der Abstand zum Ziel wird berechnet. 
# Aus einer Liste mit den Abständen wird der niedrigste ermittelt, der zugehörige TurtleBot
# bekommt dann den Befehl sich zum Ziel zu bewegen.

if __name__ == '__main__':

    
    # Started die Node mit den Namen "coordinator"
    # @param 'coordinator' Name der Node die erstellt wird
    rospy.init_node('coordinator') 
    # Erzeugt ein tf-listener Objekt um die Koordinatentransformation abzuhören            
    listener = tf.TransformListener()  
    # erzeugt Publisher um die Nachricht zum Topic "tb1/move_base_simple/goal"
    # zu senden, erwartet eine Nachricht vom Typ "PoseStamped"
    # falls mehr TBots genutzt werden müssen hier weitere hinzugefügt werden
    tb1_pub = rospy.Publisher("tb1/move_base_simple/goal", PoseStamped, queue_size=10)  
    tb2_pub = rospy.Publisher("tb2/move_base_simple/goal", PoseStamped, queue_size=10)  
    tb3_pub = rospy.Publisher("tb3/move_base_simple/goal", PoseStamped, queue_size=10)

    # Führt die Schleife aus, solange die Node nicht beendet wurde
    while not rospy.is_shutdown():                 
        # Liest User-Input und teilt in ihn in X und Y Koordinate auf
        x_inp ,y_inp = (input("Enter X, Y coordinates of the destination: ")).split() 
        # Wandelt die Input-Strings in Float um und speichert sie unter neuen Namen ab
        x_goal = float(x_inp)       
        y_goal = float(y_inp)
        # definiert Var. xy_goal als PoseStamped
        xy_goal = PoseStamped()                     
        xy_goal.header.seq = 0
        # I nformation zu welchen Koordinaten System die Koordinaten der Nachricht gehören
        xy_goal.header.frame_id = 'map' 
        # Information der aktuellen Zeit            
        xy_goal.header.stamp = rospy.Time.now()   
        # übergibt der Variable die Zielkoordinaten
        xy_goal.pose = Pose(Point(x_goal,y_goal,0.0) , Quaternion(0,0,0,1))     


        try:
            #Fragt die Koordinate von dem Baselink-Frame des Tbots im Bezug zum Map-Frame ab
            #und gibt als Rückgabewert zwei Listen mit translatorischen und rotatorischen Koordinaten
            #Wenn Mehr TBOTs genutzt werden müssen hier neue hinzugefügt werden
            (tb1_trans,tb1_rot) = listener.lookupTransform('map', 'tb1/base_link', rospy.Time(0)) 
                                                                                                  
            (tb2_trans,tb2_rot) = listener.lookupTransform('map', 'tb2/base_link', rospy.Time(0)) 
            
            (tb3_trans,tb3_rot) = listener.lookupTransform('map', 'tb3/base_link', rospy.Time(0))
        #Fängt Fehler ab, falls fehler mit den Frames auftreten
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):      
            continue
            
        
        #Liste der translatorischen Koordinaten: wenn mehr TBots, auch hier hinzufügen
        tb_trans_list = [tb1_trans, tb2_trans, tb3_trans]              

        # index zum Hochzählen der Listen
        i = 0
        # deklariert leeres Array wo die Abstände zum Ziel gespeichert werden
        distance_list = [] 
        # for-Schleife, führt so viele Iterationen durch wie Elemente in der Liste(tb_trans_list)
        for tb_trans in tb_trans_list:   
             # Füllt Liste mit den Abständen zum Ziel, Abstand wird nach der Formel für den Abstand zweier Punkte berechnet                         
            distance_list.append(math.sqrt( ( x_goal - tb_trans_list[i][0] )**2 + ( y_goal - tb_trans_list[i][1] )**2 ) )
            i = i + 1 


        # Sucht aus der Liste den kleinsten Wert und gibt davon den Index
        # der Index steht für den jeweiligen TBot
        dist_min_index = distance_list.index(min(distance_list))   
        # if-Bedingung, je nach Index wird der passende TBot ausgewählt
        # 
        # Falls mehr TBots, hier hinzufügen
        if dist_min_index == 0 : 
            # Sendet die Nachricht xy_goal an den zuvor festgelegten Topic               
            tb1_pub.publish(xy_goal)    
            # Ausgabe                               
            print("TBot1 moves to X: ",x_goal," Y: ", y_goal)    
        elif dist_min_index == 1:
            tb2_pub.publish(xy_goal)
            print("TBot2 moves to X: ",x_goal," Y: ", y_goal)
        elif dist_min_index == 2:
            tb3_pub.publish(xy_goal)
            print("TBot3 moves to X: ",x_goal," Y: ", y_goal)
        else:
            pass